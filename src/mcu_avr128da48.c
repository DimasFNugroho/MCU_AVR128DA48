// NAME:      mcu_atmega328p.c
// DATE:      Sep 15 2015
//
// copyright (c) 2004-17 Before Technology Limited

// MCU HAL for Atmel ATMega328P

//------------------------------------------------------------------------------
// INCLUDES
//------------------------------------------------------------------------------

#include "app.h"            // contains HAS_ defines
#include "uA_general.h"
#include "mcu_api.h"
#include "mcu_avr.h"
#include "mcu_avr128da48.h"
#include "uAmcus.h"

//------------------------------------------------------------------------------
// DEFINES
//------------------------------------------------------------------------------

#define F_CPU 24000000    // fixme belongs in target!

#ifndef COM_BAUD
#define COM_BAUD 57600L
#endif
#ifndef ISR_TIMER
#define ISR_TIMER 2
#endif

// This macro converts global address to local (banked) address.
#define SPI_BUFFER_SIZE      64

#define PWM_CH_0A 1  // max 4 PWM channels, as one timer is ISR
#define PWM_CH_0B 2
#if ISR_TIMER == 0
#define PWM_CH_1A 1
#define PWM_CH_1B 2
#else
#define PWM_CH_1A 0
#define PWM_CH_1B 3
#endif
#define PWM_CH_2A 0
#define PWM_CH_2B 3

//------------------------------------------------------------------------------
// FUNCTION PROTOTYPES
//------------------------------------------------------------------------------

void  _mcu_osc_init          (void);
void  _mcu_timer_init        (void);
uchar _mcu_pwm_channel  (uchar pin);

//------------------------------------------------------------------------------
// EXTERNALS
//------------------------------------------------------------------------------

extern void service_foreground (void);

//------------------------------------------------------------------------------
// CONSTANT DATA
//------------------------------------------------------------------------------

// UART bits constants
#if UA_LEVEL > 1
FLASH_VAR(const uchar avr_bits1[]) = {0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x04,0x06,0x06};
FLASH_VAR(const uchar avr_bits2[]) = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x04};
#endif

//------------------------------------------------------------------------------
// VARIABLES
//------------------------------------------------------------------------------

#if NUM_SPIS > 0
static char  spi_has_sent    = FALSE;
#endif

#if NUM_I2CS > 0 || defined(EAU_HAS_I2C)
static char  i2c_has_sent    = FALSE;
#endif

#if NUM_ADCS > 0
static uint  myAdcValues[8];
static uchar myAdcChannel;    // channel counter for service
static uchar myAdcBits;
//static uchar myAdcReference;
#endif

#if (NUM_PWMS > 0) || defined(PWM_HAS_BANGER)
#define MAX_PWMS 4
static uchar myPwms    [MAX_PWMS];
#ifdef PWM_HAS_BANGER
static uchar myPwmTimer[MAX_PWMS];
static uchar myPwmPin  [MAX_PWMS];
#endif
#endif

static uchar myErrors;

//------------------------------------------------------------------------------
// CODE
//------------------------------------------------------------------------------

// NAME:        mcu_init
// DESCRIPTION: Initialize MCU
void mcu_init(void)
{
 //   _mcu_osc_init();     // appears to be done in the startup code
    _mcu_timer_init();
#if NUM_PWMS + NUM_PULSES > 0
    mcu_pwm_init();
#endif
//    _flash_init();     // FLASH not supported
//    mcu_eeprom_init(); // nothing required
#ifdef DEBUG_A2
    mcu_bit_init(PC_2,MCU_PIN_OUTPUT);
#endif
    // mcu_bit_init(PC_3,MCU_PIN_OUTPUT);

}

// NAME:        mcu_get_mcu
// DESCRIPTION: Provide an ID code for the MCU
uint mcu_get_mcu(void)
{
    return MCU_AVR128DA48; // FIXME there are variants such as 328
}

// NAME:        mcu_get_bootloader
// DESCRIPTION: Provide an ID code for the bootloader
//              0 = factory default, probably no bootloader
//              1 = 2 kbytes, standard loader
//              2 = 1 kbyte, unknown loader
//              3 = 512 bytes, optiboot
uchar mcu_get_bootloader(void)
{
    uchar id;

    cli();
//    id = boot_lock_fuse_bits_get(GET_HIGH_FUSE_BITS);
    sei();

    return (id >> 1) & 0x3;
}


// NOTE these can be made into macros to improve performance
// NAME:        mcu_interrupts_on
// DESCRIPTION:
void mcu_interrupts_on(void)
{
    sei();
}

// NAME:        mcu_interrupts_on
// DESCRIPTION:
void mcu_interrupts_off(void)
{
    cli();
}

// NAME:        _mcu_timer_init
// DESCRIPTION: Initialize timer interrupt
// These clock configurations use /8 prescaler (2MHz)
// for 20.83us, greater accuracy is possible with /1
// if a 16 bit timer is available (333 x 62.5ns = 20.81).
// 42 x 8 = 336 x 62.5us = 21us
void _mcu_timer_init(void)
{
#if ISR_TIMER == 0                  // 8 bit, D5 & D6
    OCR0A   = TIMER_PERIOD_AVR - 1; // 25 us interrupt
    TCCR0A  = 2;                    // no waveform generated
    TCCR0B  = 2;                    // /8
    TIMSK0 |= (1 << OCIE0A);        // enable interrupt
#endif
#if ISR_TIMER == 1                  // 16 bit, D9 & D10
    OCR1A   = TIMER_PERIOD_AVR - 1; // 1/2 us per tick
    TCCR1A  = 0;                    // no waveform generated, CTC mode
    TCCR1B  = 0x0a;                 // CTC mode, /8
    TIMSK1 |= (1 << OCIE1A);        // enable interrupt
#endif
#if ISR_TIMER == 2                  // 8 bit, D3 & D11
    OCR2A   = TIMER_PERIOD_AVR - 1; // 25 us interrupt
    TCCR2A  = 2;                    // no waveform generated
    TCCR2B  = 2;                    // /8
    TIMSK2 |= (1 << OCIE2A);        // enable interrupt
#endif
}

// NAME:        timer_isr
// DESCRIPTION: System timer interrupt
#if ISR_TIMER == 0
ISR(TIMER0_COMPA_vect)
#endif
#if ISR_TIMER == 1
ISR(TIMER1_COMPA_vect)
#endif
#if ISR_TIMER == 2
ISR(TIMER2_COMPA_vect)
#endif

{
#ifdef HAS_VIO
	PORTB       = vio_data[0];            // perform virtual IO
    vio_data[0] = PORTB;
	PORTC       = vio_data[1];
    vio_data[1] = PORTC;
	PORTD       = vio_data[2];
    vio_data[2] = PORTD;
#endif
	service_foreground();
}


ISR(BADISR_vect)
{
    // user code here
}

// NAME:        mcu_timer_start
// DESCRIPTION: Start a 16-bit timer
void mcu_timer_start(void)
{
}

// NAME:        mcu_timer_stop
// DESCRIPTION: Stop 16-bit timer, return value
uint mcu_timer_stop(void)
{
    return 0;
}

//------------------------------------------------------------------------------
// CODE: EEPROM
//------------------------------------------------------------------------------

// NAME:            mcu_eeprom_erase
// DESCRIPTION: Erase a number of bytes in EEPROM
void mcu_eeprom_erase  (uchar *adr,uint n)
{
    while(n--)
             eeprom_write_byte(adr++,0xff);
}

// NAME:        mcu_eeprom_read
// DESCRIPTION: Read a byte from EEPROM
uchar mcu_eeprom_read  (uchar *adr)
{
    return eeprom_read_byte((void *)adr);
}

// NAME:        mcu_eeprom_read_w
// DESCRIPTION: Read a word from EEPROM
uint mcu_eeprom_read_w(uchar *adr)
{
    //uint  my_ee_adr;

  //  my_ee_adr = (uint)adr & 0xfffe;
  //  return eeprom_read_word((void *)my_ee_adr);
	  return eeprom_read_word((uint16_t *)adr);
}

// NAME:        mcu_eeprom_write_n
// DESCRIPTION: Write n bytes to EEPROM
//    note the uchar lands in the low byte of the uint (two uints per sector)
void mcu_eeprom_write_n(uchar *adr,uchar *values,uint n)
{
    for(;n > 0;n--)
    {
        mcu_eeprom_write(adr,*values);
        ++adr;
        ++values;
    }
}

// NAME:        mcu_eeprom_write
// DESCRIPTION: Write one byte at address
//    address is expressed as an offset (not physical)
//    note the uchar lands in the low byte of the uint (two uints per sector)
void mcu_eeprom_write(uchar *adr,uchar value)
{
    eeprom_write_byte(adr,value);
}

// NAME:        mcu_eeprom_write_w
// DESCRIPTION: Write 16bit word to EEPROM
//    note the uchar lands in the low byte of the uint (two uints per sector)
void mcu_eeprom_write_w(uchar *adr,uint value)
{
    eeprom_write_word((uint16_t *)adr,value);
}

//------------------------------------------------------------------------------
// CODE: FLASH
//------------------------------------------------------------------------------

#ifdef HAS_FLASH

// NAME:        _flash_init
// DESCRIPTION: Initialize the flash module
void _flash_init (void)
{
}

// NAME:        mcu_flash_sector_erase
// DESCRIPTION: Erase a sector (512 bytes) of flash
char mcu_flash_sector_erase(uchar *address)
{
    return FALSE;
}

// NAME:        mcu_flash_sector_write
// DESCRIPTION: Writes data to the block pointed at by the address
char mcu_flash_sector_write(uchar *address,uchar *values)
{
    return FALSE;
}

// NAME:        mcu_flash_sector_read
// DESCRIPTION: Writes a byte to the block pointed at by the address
char mcu_flash_sector_read(uchar *address,uchar *values)
{
    return FALSE;
}

#endif

//------------------------------------------------------------------------------
// CODE: ANALOG
//------------------------------------------------------------------------------

// ADCSRA  =  ADEN-ADSC-ADATE-ADIF-ADIE-ADPS2-ADPS1-ADPS0
// ADMUX   =  REFS1 - REFS0 - ADLAR - - - MUX3 - MUX2 - MUX1 - MUX0

#if NUM_ADCS > 0


// NAME:           mcu_analog_channel
// DESCRIPTION:    Convert pin to channel
uchar mcu_analog_channel(uchar pin)
{
    uchar channel;

    if(pin == 22) return 7;
    if(pin == 19) return 6;

    return pin - PC_0;
}

// NAME:           mcu_analog_pin_enable
// DESCRIPTION:    Make this an analog pin
void mcu_analog_pin_enable(uchar pin)
{
	// probably ought to make it an input, but that is the default state
}

// NAME:           mcu_analog_init
// DESCRIPTION:    Initialize the ADC
void mcu_analog_init(uchar base_channel,uchar n_channels)
{
    myAdcChannel   = 0;
    myAdcBits      = MCU_ADC_BITS;
    //myAdcReference = 0;             // AREF, internal ref turned off
    //myAdcReference = 1;           // AVCC with external cap on AREF pin
                                    // WARNING: if external voltage applied, turning on internal creates a SHORT!
    ADCSRA  = 6;                    // enable ADC, prescaler = /64
    ADCSRA |= (1 << ADEN);          //
///    ADMUX = myAdcReference << 6;    // select channel
    ADMUX = 0;
    ADCSRA |= (1 << ADSC);
}

// NAME:           mcu_analog_set_bits
// DESCRIPTION:    Set ADC bit depth
void mcu_analog_set_bits(uchar bits)
{
    if((bits == 8)
    || (bits == 10))
        myAdcBits = bits;
}

// NAME:           mcu_analog_get_bits
// DESCRIPTION:    Get ADC bit depth
uchar mcu_analog_get_bits(void)
{
    return myAdcBits;
}

// NAME:           mcu_analog_service
// DESCRIPTION:    Periodic service
void mcu_analog_service(void)

{
    uchar lsbyte;
    uint msbyte;

   if((ADCSRA & (1 << ADSC)) == 0)
   {
       lsbyte = ADCL;                                     // grab conversion
       msbyte = (uint)ADCH;                               // (assume its ready)
       if(myAdcBits == 8)                                 // result is right justified
           myAdcValues[myAdcChannel] = (msbyte << 6) | (lsbyte >> 2); // FIXME this can be done better
       else
           myAdcValues[myAdcChannel] = (msbyte << 8) | lsbyte;
       myAdcChannel = (myAdcChannel + 1) & 0x07;          // 8 channels
//    ++myAdcChannel;
//    if(myAdcChannel > 5)
//        myAdcChannel = 0;
       ADMUX = myAdcChannel;                              // select channel
//       ADMUX = (myAdcReference << 6) | myAdcChannel;      // select channel
       ADCSRA |= (1 << ADSC);                             // start conversion
    }
}

// NAME:           mcu_analog_has
// DESCRIPTION:    Find out if conversion complete
uchar mcu_analog_has(void)
{
//    return ((ADCSRA & (1 << ADSC)) == 0) ? TRUE : FALSE;
    return TRUE;
}

// NAME:           mcu_analog_in
// DESCRIPTION:    Get conversion value
uint mcu_analog_in(uchar ch)
{
    return myAdcValues[ch];
}

#endif

//-------------------------------------------------------------------------------
// CODE: TOUCH
//-------------------------------------------------------------------------------

#if NUM_TOUCH > 0
// NAME:           touch_init
// DESCRIPTION:    Initialize touch
void  mcu_touch_init		    (uchar pin)                // configure pin as touch
{

}

// NAME:           touch_in
// DESCRIPTION:    Get touch state
char  mcu_touch_in		    (uchar pin)
{

	return 0;
}
#endif

//-------------------------------------------------------------------------------
// CODE: PULSE
//-------------------------------------------------------------------------------

#if NUM_PULSES > 0

// NOTE this implementation is limited in scope
//      it has a high resolution of 0.5 us

// NAME:        mcu_pulse_enable_set
// DESCRIPTION:
void mcu_pulse_enable_set(uchar pin,char state)
{
    mcu_pwm_enable_set(pin,state);
    TCCR0A = 0x03;     // Fast PWM mode 0x03 = fast PWM
    TCCR2A = 0x03;
}

// NAME:        mcu_pulse_set
// DESCRIPTION:
void mcu_pulse_set(uchar pin,uint duration)
{
    uchar i;

    i               = _mcu_pwm_channel(pin);
    myPulseNext[i]  = duration;
    //myPulseState[i] = PULSE_STATE_START; // 8 bit assign
}

// NAME:        mcu_pulse_service
// DESCRIPTION: Service all pulse pins
void mcu_pulse_service(void)
{
    uchar step;
    uchar state;
    uchar value;
    uchar ch_new;
    static uchar ch;
    static uchar periodTimer;
    static char  halfCycle;

  // PORTC |= 0x04;
   // ch    = (ch + 1) & 0x03;
    step  = _mcu_pulse_count(ch);

    //----- manage the period of pulses -----------
    if((step < COUNT_START)        // s--------e
    || (step > COUNT_END))         // about 25% active
    {
        state = myPulseState[ch];
        if(state != PULSE_STATE_IDLE)
        {
            //----- start section -----------------
            if(step < COUNT_START)
            {
                    if(state == PULSE_STATE_0_END)
                    state = PULSE_STATE_1_0;
                else
                if(state == PULSE_STATE_1_END)  // beginning of first pulse
                    state = PULSE_STATE_2_0;
            }
        //----- end section -------------------
            else
            {
                if(state == PULSE_STATE_START)
                {
 //                       _mcu_pulse_start(ch);
                    if(ch == 0)
                    {
                        OCR2B   = 255;
                        TCCR2A |= 0x20;              // change to PWM
                    }
                    else
                    if(ch == 1)
                    {
                        OCR0B   = 255;
                        TCCR0A |= 0x20;              // change to PWM
                    }
                    else
                    if(ch == 2)
                    {
                        OCR0A   = 255;
                        TCCR0A |= 0x80;              // change to PWM
                    }
                    else
                    {
                        OCR2A   = 255;
                        TCCR2A |= 0x80;              // change to PWM
                    }

                    state = PULSE_STATE_0_END;
                 }
                    else
                if(state == PULSE_STATE_1_0)     // end of 1st pulse
                {
                        value = (uchar)(myPulseTimer[ch]);
                        if(ch == 0)
                            OCR2B = value;             // one pulse!
                        else
                            if(ch == 1)
                           OCR0B = value;
                    else
                    if(ch == 2)
                           OCR0A = value;
                    else
                           OCR2A = value;
                        state = PULSE_STATE_1_END;
                }
                else
                if(state == PULSE_STATE_2_0)         // end of last pulse
                {
                        if(step >= myPulseTimer[ch])
                        {
                         //   _mcu_pulse_end(ch);
                        if(ch == 0)
                        {
                            PORTD  &= ~0x08;                 // PD3 low
                            TCCR2A &= ~0x30;                 // stop generating PWM
                            OCR2B   = 255;
                        }
                        else
                        if(ch == 1)
                        {
                            PORTD  &= ~0x20;                 // PD5 low
                            TCCR0A &= ~0x30;                 // stop generating PWM
                            OCR0B   = 255;
                        }
                        else
                        if(ch == 2)
                        {
                            PORTD  &= ~0x40;                 // PD6 low
                            TCCR0A &= ~0xc0;                 // stop generating PWM
                                OCR0A   = 255;
                            }
                            else
                            {
                                PORTB  &= ~0x08;                 // PB3 low
                                TCCR2A &= ~0xc0;                 // stop generating PWM
                                OCR2A   = 255;
                            }
                            state = PULSE_STATE_IDLE;
                        }
                    }
                    }
                myPulseState[ch] = state;
                }
        }
        else
        if(step < COUNT_START + 20)
        {
            if(!halfCycle)
            {
                halfCycle = TRUE;
                ch_new = pgm_read_byte(&(pulse_chan[periodTimer]));
                if((ch_new != 0 )
                && ((ch_new - 1) != ch))
                {
                        ch = ch_new - 1;
                        myPulseTimer[ch] = myPulseNext[ch];
                        myPulseState[ch] = PULSE_STATE_START;
                }
                    ++periodTimer;
                        //    _mcu_pulse_new(ch);
            }
        }
        else
        if(step > COUNT_END - 20)
        {
            if(halfCycle)
            {
                halfCycle = FALSE;
                if(periodTimer >= 19)
                    periodTimer = 0;
            }
        }
}
/*
// NAME:        mcu_pulse_stop
// DESCRIPTION:
void mcu_pulse_stop(uchar pin)
{
    // FIXME this does nothing
    // it seems kind of useless in the application

 //   mcu_pwm_set_8b(pin,0);
}
*/
#endif

//-------------------------------------------------------------------------------
// CODE: PWM 8b (for 8b and 16b timers)
//-------------------------------------------------------------------------------

#if (NUM_PWMS + NUM_PULSES + NUM_PWM16S) > 0

// Three timers, one is dedicated to the timer interrupt
// Each timer has 2 output compares, so 2 x 2 = 4 PWMs available
// Timer 1 is 16 bit, timers 0 and 2 are 8 bit
// Timer 1 is the interrupt timer
// The 4 PWMs are mapped to "channels" (not element channels, rather, MCU-level)

// unlike some other MCUs, the channel is the pin number

// Arduino "channel"   pin    timer    output compare
//  D3      PD_3       PD_3   2        B
//  D5      PD_5       PD_5   0        B
//  D6      PD_6       PD_6   0        A
//  D9      PB_1       PB_1   1        A
//  D10     PB_2       PB_2   1        B
//  D11     PB_3       PB_3   2        A

// Timer 0 =  8b PD6/PD5 (A/B)
// Timer 1 = 16b PB1/PB2
// Timer 2 =  8b PB3/PD3

// D5/D6 timer 0, D3/B3 timer 2

// COM bits (0xc0 and 0x30 of TCCRxA)
// 00=GPIO 01=toggle on match 10=clear 11=set (inverted)

// IMPORTANT NOTE fast PWM cannot generate a continuous 0
//                it does generate a continuous 1 if OCR=255

// PWM BANGER
//
// User of the banger and hardware PWMs are mutually exclusive
// So when banger is enabled, the hardware PWMS do not operate

// NAME:        pwm_init
// DESCRIPTION: Initialize PWM peripheral
void mcu_pwm_init(void)
{
#ifndef PWM_HAS_BANGER
#if ISR_TIMER == 1
    TCNT2  = TCNT0;    // make the counters match
#endif
#if ISR_TIMER != 0
    OCR0A  = 0;
    OCR0B  = 0;
    TCCR0B = 0x03;     // 16MHz clock, Fast PWM @ 7843 Hz, phase @ 3921 Hz
    TCCR0A = 0x01;     // Fast PWM mode (x3= fast PWM x1=phase correct)
    //    TCCR0A = 0xA3;     // Fast PWM mode (x3= fast PWM x1=phase correct)
    //    TCCR0A = 0x03;     // Fast PWM mode (x3= fast PWM x1=phase correct)
#endif
#if (ISR_TIMER != 1) && (NUM_PWM16S == 0)
    OCR1A  = 0;
    OCR1B  = 0;
       TCCR1A = (1 << WGM11);                                  // NON Inverted PWM
        TCCR1B = (1 << WGM13) | (1 <<  WGM12) | (1 << CS11);   // PRESCALER=8 MODE 14(FAST PWM)
        ICR1   = 255;                                          // period = 8 bit).

    // FIXME
#endif
#if ISR_TIMER != 2
    OCR2A  = 0;
    OCR2B  = 0;
    TCCR2A = 0x01;     // normal PWM works best with phase-corrected
    TCCR2B = 0x04;
#endif
#endif
}

// NAME:        _mcu_pwm_channel
// DESCRIPTION: Get PWM channel from pin
uchar _mcu_pwm_channel(uchar pin)
{

#ifndef PWM_HAS_BANGER
    switch(pin)
    {
#if ISR_TIMER != 0
        case PD_5: return PWM_CH_0A; // Timer 0
        case PD_6: return PWM_CH_0B;
#endif
#if ISR_TIMER != 1
        case PB_1: return PWM_CH_1A; // Timer 1
        case PB_2: return PWM_CH_1B;
#endif
#if ISR_TIMER != 2
        case PD_3: return PWM_CH_2A; // Timer 2
        case PB_3: return PWM_CH_2B;
#endif
    }
#else
    uchar ch;

    for(ch = 0;ch < MAX_PWMS;ch++)
        if(myPwmPin[ch] == pin)
            return ch;
#endif
    return 255;                         // FIXME defaulting to a valid channel #!
}

// FIXME using pin # as channel, which works as long as there is no
//       bit PWM.

// NAME:        mcu_pwm_enable_set
// DESCRIPTION: Enable or disable a PWM channel (8b/16b)
void mcu_pwm_enable_set(uchar pin,uchar state)
{
  //  uchar reg;

#ifndef PWM_HAS_BANGER
    switch(pin)  // COM bits are AABBxxxx in TCCRxA register
    {
#if ISR_TIMER != 0
    case PD_6: // 0 A
          TCCR0A = (TCCR0A & ~0xc0) | ((state) ? 0x80 : 0x00); break;
    case PD_5: // 0 B
        TCCR0A = (TCCR0A & ~0x30) | ((state) ? 0x20 : 0x00); break;
#endif
#if (ISR_TIMER != 1) // works with both 8 and 16 bit PWM
    case PB_1: // 1 A
            TCCR1A = (TCCR1A & ~0xc0) | ((state) ? 0x80 : 0x00); break;
    case PB_2: // 1 B
            TCCR1A = (TCCR1A & ~0x30) | ((state) ? 0x20 : 0x00); break;
#endif
#if ISR_TIMER != 2
    case PB_3: // 2 A
          TCCR2A = (TCCR2A & ~0xc0) | ((state) ? 0x80 : 0x00); break;
    case PD_3: // 2 B
            TCCR2A = (TCCR2A & ~0x30) | ((state) ? 0x20 : 0x00); break;
#endif
    }
#endif
    mcu_bit_init(pin,(state) ? MCU_PIN_OUTPUT : MCU_PIN_INPUT);       // any kind of PWM must be an output
}

#ifndef PWM_PERIOD
#define PWM_PERIOD 200
#endif


// NAME:        mcu_pwm_set_8b
// DESCRIPTION: Sets the 8 bit compare value for a single PWM channel
//              This is for cases where 0-255 is 0-100%
void mcu_pwm_set_8b(uchar ch,uchar value)
{
#ifndef PWM_HAS_BANGER
        switch(ch)
    {
#if ISR_TIMER != 0
        case PD_6:
                OCR0A = value;                    // set timer compare value (duty cycle)
       //         myPwms[PWM_CH_0A] = value;        // save value for later recall
                break;
        case PD_5:
            OCR0B = value;
        //    myPwms[PWM_CH_0B] = value;
                break;
#endif
#if (ISR_TIMER != 1)
        case PB_1:
                OCR1A = (uint)value;
//                myPwms[PWM_CH_1A] = value;

                break;
        case PB_2:
                OCR1B = (uint)value;
//                myPwms[PWM_CH_1B] = value;
                break;
#endif
#if ISR_TIMER != 2
        case PB_3:
                OCR2A = value;
//                myPwms[PWM_CH_2A] = value;

                break;
        case PD_3:
                OCR2B = value;
//                myPwms[PWM_CH_2B] = value;

                break;
#endif
    }
#endif
}

#if (NUM_PWMS > 0)
// NAME:        mcu_pwm_set
// DESCRIPTION: Sets the duty cycle for a single PWM channel
void mcu_pwm_set(uchar pin, uchar value)
{
#ifndef PWM_HAS_BANGER
    uchar v;

    v = (value << 1) + (value >> 1) + (value / 20); // 0-100% mapped to 0-255
                                                    // FIXME a LUT would be faster
    mcu_pwm_set_8b(pin,value);
#else
    uchar ch;

    ch = _mcu_pwm_channel(pin);
    if(ch < MAX_PWMS)
    {
        myPwms[ch] = value;
        if((value == 0)
           || (value == 100))
                mcu_bit_out(myPwmPin[ch],value);
    }

#endif
}

// NAME:        mcu_pwm_get
// DESCRIPTION: Gets the duty cycle for a single PWM channel
uint mcu_pwm_get(uchar pin)
{

#ifndef PWM_HAS_BANGER
    switch(pin)
    {
#if ISR_TIMER != 0
        case PD_6: return (uint)OCR0A;

            //    return myPwms[PWM_CH_0A];    // FIXME could read from timer compare register
        case PD_5: return (uint)OCR0B;                       //       IF not a banger
        //    return myPwms[PWM_CH_0B];
#endif
#if (ISR_TIMER != 1)
        case PB_1: return OCR1A;
            //return myPwms[PWM_CH_1A];
        case PD_2: return OCR1B;
            //return myPwms[PWM_CH_1B];
#endif
#if ISR_TIMER != 2
        case PB_3: return (uint)OCR2A;
            //return myPwms[PWM_CH_2A];
        case PD_3: return (uint)OCR2B;
            //return myPwms[PWM_CH_2B];
#endif
    }
#else
    uchar ch;

    ch = _mcu_pwm_channel(pin);
    if(ch < NUM_PWMS)
        return (uint)myPwms[ch];
#endif
    return 0;
}

/*
// NAME:        mcu_pwm_get_count
// DESCRIPTION: Get current counter register value for channel
uchar mcu_pwm_get_count(uchar ch)
{
    if((ch == PD_3)
    || (ch == PB_3))
        return TCNT2;
    return TCNT0;      // PD_5 or PD_6
}
*/

#endif

#ifdef PWM_HAS_BANGER
// NAME:        mcu_pwm_pin_set
// DESCRIPTION: Set pin for bit banging.
void mcu_pwm_pin_set(uchar ch,uchar pin)
{

    myPwmPin[ch] = pin;
}

// NAME:        mcu_pwm_service
// DESCRIPTION: Called from foreground task at 25us
// PWM frequency depends on how many channels:
// 1 ch = 400 Hz
// 2 ch = 200 Hz
// 4 ch = 100 Hz (max channels)

void mcu_pwm_service(void)
{
    static uchar pch;
    uchar pt;
    uchar pd;

    //for(pch = 0;pch < NUM_PWMS;pch++)
    //{
    ++pch;
    if(pch >= NUM_PWMS)
            pch = 0;
    pd = myPwms[pch];


  //  if(pch  == 0)
  //          pd = 30;               // TMP FOR TESTING
  //      mcu_bit_out(myPwmPin[pch],1); this makes the motor turn
  //  else
  //      mcu_bit_out(myPwmPin[pch],0);
//return;

    if((pd > 0)
    && (pd < 100))
    {
        pt = myPwmTimer[pch];
        if(++pt == 100)
        {
           pt = 0;
           mcu_bit_out(myPwmPin[pch],1);
        }
        else
        if(pt == pd)
            mcu_bit_out(myPwmPin[pch],0);
        myPwmTimer[pch] = pt;
    }
    //}
}
#endif
#endif

#if NUM_PWM16S > 0
//#ifndef PWM_HAS_BANGER // causes compile issues
#if ISR_TIMER != 1

//-------------------------------------------------------------------------------
// CODE: 16b PWM
//-------------------------------------------------------------------------------

// NAME:        mcu_pwm16_init
// DESCRIPTION: Initialize PWM peripheral
//              Selects the highest frequency possible
void mcu_pwm16_init(uint range)
{
#if (ISR_TIMER != 1) && (NUM_PWM16S > 0)
       TCCR1A = (1 << WGM11);                                 // NON Inverted PWM
        TCCR1B = (1 << WGM13) | (1 <<  WGM12) | (1 << CS10);   // PRESCALER=1 MODE 14(FAST PWM)
        ICR1   = range;                                        // period = user specified.
#endif
}

// NAME:        mcu_pwm16_set
// DESCRIPTION: Sets the 16 bit compare value for a single PWM channel

void mcu_pwm16_set(uchar pin,uint value)
{
        switch(pin)
    {
        case PB_1: OCR1A = value; break;
        case PB_2: OCR1B = value;
    }
}

// NAME:           servo_init
// DESCRIPTION:    Init servo
//                 Rotation from 0-180 degrees, with 5.5us per degree
//                 Pulse Cycle:    20ms
void mcu_servo_init(uchar pin)
{
    // Period = 20ms Standard).
    switch(pin)
    {
#if ISR_TIMER != 0
        case PD_6:
        case PD_5:
            OCR0A  = 0;
            OCR0B  = 0;
            TCCR0A = 0x01;
            TCCR0B = 0x05;
            break;
#endif
#if ISR_TIMER != 2 // FIXME different than for channel 0!!
        case PB_3:
        case PD_3:
            OCR2A  = 0;
            OCR2B  = 0;
                TCCR2A = 0x01;     // normal PWM works best with phase-corrected
            TCCR2B = 0x05;     //  / 1024, yields 16.4 ms
            break;
#endif
#if ISR_TIMER != 1
        case PB_1:
        case PB_2:
            //uA_tx('i');
            TCCR1A = (1 << WGM11) | (1 << COM1A1) | (1 << COM1B1);// NON Inverted PWM
            TCCR1B = (1 << WGM13) | (1 << WGM12)  | (1 << CS11);  // PRESCALER=8 MODE 14(FAST PWM)
            ICR1   = 39999;                                       // Period = 20ms Standard).
            break;
#endif
    }
}

// NAME:           mcu_servo_set
// DESCRIPTION:    Set servo angle
//                 11 clocks = 5.5us = 1 degree
//                 1ms (0 degree) = 2000 * 0.5us
/*
void mcu_servo_set(uchar pin,uint angleTimesTen)
{
    switch(pin)
    {
#if ISR_TIMER != 0
        case PD_6:
        case PD_5:
#endif
#if ISR_TIMER != 2
        case PB_3:
        case PD_3:
              mcu_pwm_set_8b(pin,13 + (angleTimesTen / 140)); // 13 = 1.0ms, 180/13 steps = 14
            break;
#endif
#if ISR_TIMER != 1
        default:
            mcu_pwm16_set(pin,2000 + angleTimesTen + (angleTimesTen / 10)); // angle * 1.1 //(11 * angle));
#endif
    }
}
*/
// NAME:           mcu_servo_set_pulse
// DESCRIPTION:    Set servo pulse width in microseconds
void mcu_servo_set_pulse(uchar pin,uint usecs)
{
//    switch(pin)
//    {
#if ISR_TIMER != 0
//        case PD_6:
//        case PD_5:
#endif
#if ISR_TIMER != 2
//        case PB_3:
//        case PD_3:
#endif
//                if(usecs < 78) usecs = 78;
//              mcu_pwm_set_8b(pin,usecs / 78); // tick period is 78 us
//            break;
#if ISR_TIMER != 1
//        default:


            mcu_pwm16_set(pin,usecs << 1); // tick period is 500 ns
#endif
//    }
}

//-------------------------------------------------------------------------------
// CODE: TONE GENERATION
//-------------------------------------------------------------------------------

// NAME:        tone_init
// DESCRIPTION: Init PWM channel for 16b Timer 1
//              Timer/Counter 1/3 initialization
//              Clock source: System Clock
//              Clock value : 2MHz
//              Mode        : CTC top= OCR1A
//              OCxA output : Clear on compare match
void mcu_tone_init(uchar pin)
{
    TCCR1B = (1 << WGM12) | (1 << CS11);  // CTC mode, prescaler=/8
    TCNT1  = 0;
    mcu_pwm16_set_period(pin,0);
}

// NAME:        mcu_pwm16_set_period
// DESCRIPTION: Set the period of the tone
void mcu_pwm16_set_period(uchar pin,uint value)
{
    switch(pin)
    {
        case PB_1: 
              if(value == 0)
                  TCCR1A = (1 << COM1A1); // zero
              else
                  TCCR1A = (1 << COM1A0); // toggle
              OCR1A = value;
            break;
        case PB_2:
              if(value == 0)
                    TCCR1A = (1 << COM1B1);
              else
                  TCCR1A = (1 << COM1B0);
              OCR1A = value;
    }
}
#endif
//#endif
#endif

//-------------------------------------------------------------------------------
// CODE: UART
//-------------------------------------------------------------------------------

// NAME:        mcu_uart_init
// DESCRIPTION: Initialize UART peripheral
void mcu_uart_init(uchar ch)
{
    myErrors = 0;
    UCSR0C |= 0x0e;  // asynch, 8-N-2
    mcu_uart_set_baud(ch,COM_BAUD);
}

// NAME:        mcu_uart_set_baud
// DESCRIPTION: Set baud rate
void mcu_uart_set_baud(uchar ch,long baud)
{
    UCSR0A = (1 << U2X0);       // 2 MHz clock
    UBRR0H = 0;
    switch(baud)
    {
		case  9600L  : UBRR0L = 207; break;
		case 19200L  : UBRR0L = 103; break;
		case 31250L  : UBRR0L = 63;  break;
		case 38400L  : UBRR0L = 51;  break;
		case 57600L  : UBRR0L = 34;  break;
		case 115200L : UBRR0L = 16;  break;
		case 250000L : UBRR0L = 7;   break;
    }
}

// NAME:        uart_get_errors
// DESCRIPTION: Get error flags
uchar mcu_uart_get_errors(uchar ch)
{
    uchar errors;

    errors = myErrors;
    myErrors = 0;

    return errors;
}

#if UA_LEVEL > 1
const uchar parity_bits[] = {0x00,0x03,0x02};

// NAME:        mcu_uart_set_parity
// DESCRIPTION: Set parity
void mcu_uart_set_parity(uchar ch,uchar parity)
{

    if(parity <= 2)
        UCSR0C = (UCSR0C & ~0x30) | (parity_bits[parity] << UPM00);
}

// NAME:        mcu_uart_set_bits
// DESCRIPTION: Set number of bits
void mcu_uart_set_bits(uchar ch,uchar bits)
{

    if((bits >= 5)
    && (bits <= 9))
    {
        UCSR0B = (UCSR0B & ~0x04) | pgm_read_byte(&(avr_bits2[bits]));
        UCSR0C = (UCSR0C & ~0x06) | pgm_read_byte(&(avr_bits1[bits]));
    }
}
#endif


// NAME:        mcu_uart_tx_enable
// DESCRIPTION: Enable/disable transmitter
void mcu_uart_tx_enable(uchar ch,char state)
{
    if(state)
        UCSR0B |=  (1 << TXEN0);
    else
        UCSR0B &= ~(1 << TXEN0);
}

// NAME:        mcu_uart_tx_ready
// DESCRIPTION: Indicate if transmitter can accept a byte
char mcu_uart_tx_ready(uchar ch)
{
    return (UCSR0A & (1 << UDRE0)) ? TRUE : FALSE;
}

// NAME:        mcu_uart_tx_c
// DESCRIPTION: Output a byte on the transmitter
void mcu_uart_tx_c(uchar ch,uchar c)
{
    if(UCSR0A & (1 << UDRE0)) // mcu_uart_tx_ready()
            UDR0 = c;
}

// NAME:        mcu_uart_tx_break
// DESCRIPTION: See if a break has occurred
void  mcu_uart_tx_break     (uchar ch,char state)
{
    // NOTE: no break support in AVR
    // this would require either lowering baud rate and transmitting 0
    // or changing to GPIO and driving the pin
    // perhaps with a service in the interrupt?


}

// NAME:        mcu_uart_rx_enable
// DESCRIPTION: Enable/disable the receiver
void mcu_uart_rx_enable(uchar ch,char state)
{
    if(state)
        UCSR0B |=  (1 << RXEN0);
    else
        UCSR0B &= ~(1 << RXEN0);
}

// NAME:        mcu_uart_rx_c
// DESCRIPTION: Receive one character
// RETURNS:     Character received, or -1 if one not available
uchar mcu_uart_rx_c(uchar ch)
{
    if(UCSR0A & (1 << RXC0))
    {
        myErrors |= UCSR0A & 0x1c;
        return (uchar)UDR0;
    }
    return 0;
}

// NAME:        mcu_uart_rx_has
// DESCRIPTION: Indicate if a byte is available in the receiver
char mcu_uart_rx_has(uchar ch)
{
    return (UCSR0A & (1 << RXC0));// ? TRUE : FALSE;
}

// NAME:        mcu_uart_rx_break
// DESCRIPTION: See if a break has occurred
char mcu_uart_rx_break(uchar ch)
{
    return (UCSR0A & (1 << FE0));// ? TRUE : FALSE; // actually a framing error!
}

//-------------------------------------------------------------------------------
// CODE: SPI
//-------------------------------------------------------------------------------

#if NUM_SPIS > 0

// NAME:        spi_init
// DESCRIPTION: Initialises the Serial Peripheral Interface
void mcu_spi_init(uchar ch,uchar isMaster,char isWord)
{
    SPCR = 0;                                          // starting clean
    DDRB &= ~(1 << DDB4);                              // MISO = in
    DDRB |=  (1 << DDB5) | (1 << DDB3) | (1 << DDB2);  // MOSI, SCK, SS = out
    mcu_spi_set_baud(ch,1500);                         // default baud is 1.5Mbps
    mcu_spi_set_master(ch,isMaster);
    SPCR |= (1 << SPE);
}

// NAME:        spi_set_baud
// DESCRIPTION: Set SPI baud rate as arg x 1k
void mcu_spi_set_baud(uchar ch,long baud)
{
    uchar lsbits;
    uchar msbit;

    lsbits = 0;
    msbit  = 0;
    if(baud >= 6000000)          // / 2
        msbit = 1;
    else
    if(baud >= 3000000)          // / 4
         ;
    else
    if(baud >= 1500000)          // / 8
    {
        lsbits = 1;
        msbit  = 1;
    }
    else
    if(baud >= 750000)           // / 16
        lsbits = 1;
    else
    if(baud >= 375000)           // / 32
    {
        msbit  = 1;
        lsbits = 2;
    }
    else
    if(baud >= 187000)          // / 64
        lsbits = 2;
    else
        lsbits = 3;             // / 128
    SPCR = (SPCR & ~0x3) | lsbits;
    SPSR = (SPSR & ~0x1) | msbit;
}

// NAME:        spi_set_master
// DESCRIPTION: Set SPI to master or slave
void mcu_spi_set_master(uchar ch,char is_master)
{
    if(is_master)
        SPCR |=  (1 << MSTR);
    else
        SPCR &= ~(1 << MSTR);
}

// NAME:        spi_is_master
// DESCRIPTION: Indicate is SPI is in master mode
char mcu_spi_is_master(uchar ch)
{
    return (SPCR & (1 << MSTR)) ? 1 : 0;
}

// NAME:        mcu_spi_tx_enable
// DESCRIPTION: Set SPI slave select to low (active) or high (inactive)
//              Must not be changed when TX in progress!
void mcu_spi_tx_enable(uchar ch,char state)
{
    // this MCU uses a hardware-controlled SS line
}

// NAME:        spi_tx_ready
// DESCRIPTION: Indicate if SPI is ready to transmit
char mcu_spi_tx_ready(uchar ch)
{
    return (SPSR & (1 << SPIF)) || !spi_has_sent;
}

// NAME:        spi_tx_c
// DESCRIPTION: Transmit a byte if SPI is ready
char mcu_spi_tx_c(uchar ch,uchar value)
{
    if((SPSR & (1 << SPIF))
    || !spi_has_sent)
    {
        SPDR = value;
        spi_has_sent = TRUE;
        return TRUE;
    }
    return FALSE;
}

// NAME:        spi_tx_word
// DESCRIPTION: Transmit a word if SPI is ready
char mcu_spi_tx_w(uchar ch,uint value)
{
    return 0;   // cannot transmit a word
}

// NAME:        spi_rx_has
// DESCRIPTION: Indicate if a received byte is available
char mcu_spi_rx_has(uchar ch)
{
    return (SPSR & (1 << SPIF)) ? 1 : 0;
}

// NAME:        spi_rx_c
// DESCRIPTION: Get a  byte from SPI
char mcu_spi_rx_c(uchar ch,uchar *value)
{
    if(SPSR & (1 << SPIF))
    {
        *value = SPDR;
        return 1;
    }
    return 0;
}

// NAME:        spi_rx_word
// DESCRIPTION: Get a 16bit word from SPI (if applicable)
char mcu_spi_rx_w(uchar ch,uint *value)
{

    return 0;  // cannot receive a word
}
#endif

//------------------------------------------------------------------------------
// CODE: GPIO
//------------------------------------------------------------------------------

// NAME:        mcu_bit_init
// DESCRIPTION: Initialize I/O bit
void mcu_bit_init(uchar pin, uchar mode)
{
    volatile uchar *reg;
    uchar bit;

    bit = 0;
    reg = &DDRB;
    switch(pin)
    {
        case PB_0:  // PORTB pins are contiguous
        case PB_1:
        case PB_2:
        case PB_3:
        case PB_4:
        case PB_5:
                bit = pin - PB_0; reg = &DDRB;break;
        case PC_0: // PORTC pins are contiguous
        case PC_1:
        case PC_2:
        case PC_3:
        case PC_4:
        case PC_5:
                bit = pin - PC_0; reg = &DDRC;break;

        case PD_0: bit = 0; reg = &DDRD;break;
        case PD_1: bit = 1; reg = &DDRD;break;
        case PD_2: bit = 2; reg = &DDRD;break;
        case PD_3: bit = 3; reg = &DDRD;break;
        case PD_4: bit = 4; reg = &DDRD;break;
        case PD_5: bit = 5; reg = &DDRD;break;
        case PD_6: bit = 6; reg = &DDRD;break;
        case PD_7: bit = 7; reg = &DDRD;break;
    }
    if(mode == MCU_PIN_OUTPUT)
    {
          //  mcu_bit_out(pin,0); // should it??
            *reg |=   1 << bit;
    }
        else
    {
            *reg &= ~(1 << bit);
        if(mode == MCU_PIN_PULLUP)
                mcu_bit_out(pin,1);
    }
}

// NAME:        mcu_bit_out
// DESCRIPTION: Output a GPIO bit
void mcu_bit_out(uchar pin,uchar state)
{
    if(state)
    {
        switch(pin)
        {
            case PB_0:  PORTB |= 0x01; break;
            case PB_1:  PORTB |= 0x02; break;
            case PB_2:  PORTB |= 0x04; break;
            case PB_3:  PORTB |= 0x08; break;
            case PB_4:  PORTB |= 0x10; break;
            case PB_5:  PORTB |= 0x20; break;

            case PC_0:  PORTC |= 0x01; break;
            case PC_1:  PORTC |= 0x02; break;
            case PC_2:  PORTC |= 0x04; break;
            case PC_3:  PORTC |= 0x08; break;
            case PC_4:  PORTC |= 0x10; break;
            case PC_5:  PORTC |= 0x20; break;

            case PD_2:  PORTD |= 0x04; break;
            case PD_3:  PORTD |= 0x08; break;
            case PD_4:  PORTD |= 0x10; break;
            case PD_5:  PORTD |= 0x20; break;
            case PD_6:  PORTD |= 0x40; break;
            case PD_7:  PORTD |= 0x80; break;
        }
    }
    else
    {
        switch(pin)
        {
            case PB_0:  PORTB &= ~0x01; break;
            case PB_1:  PORTB &= ~0x02; break;
            case PB_2:  PORTB &= ~0x04; break;
            case PB_3:  PORTB &= ~0x08; break;
            case PB_4:  PORTB &= ~0x10; break;
            case PB_5:  PORTB &= ~0x20; break;

            case PC_0:  PORTC &= ~0x01; break;
            case PC_1:  PORTC &= ~0x02; break;
            case PC_2:  PORTC &= ~0x04; break;
            case PC_3:  PORTC &= ~0x08; break;
            case PC_4:  PORTC &= ~0x10; break;
            case PC_5:  PORTC &= ~0x20; break;

            case PD_2:  PORTD &= ~0x04; break;
            case PD_3:  PORTD &= ~0x08; break;
            case PD_4:  PORTD &= ~0x10; break;
            case PD_5:  PORTD &= ~0x20; break;
            case PD_6:  PORTD &= ~0x40; break;
            case PD_7:  PORTD &= ~0x80; break;
        }
    }
}

//#define BITS_NOT_1_0
// NAME:        mcu_bit_in
// DESCRIPTION: Input a GPIO bit
char mcu_bit_in(uchar pin)
{
    switch(pin)
    {
#ifdef BITS_NOT_1_0
        case PB_0:  return (PINB & 0x01);
        case PB_1:  return (PINB & 0x02);
        case PB_2:  return (PINB & 0x04);
        case PB_3:  return (PINB & 0x08);
        case PB_4:  return (PINB & 0x10);
        case PB_5:  return (PINB & 0x20);

        case PC_0:  return (PINC & 0x01);
        case PC_1:  return (PINC & 0x02);
        case PC_2:  return (PINC & 0x04);
        case PC_3:  return (PINC & 0x08);
        case PC_4:  return (PINC & 0x10);
        case PC_5:  return (PINC & 0x20);

        case PD_2:  return (PIND & 0x04);
        case PD_3:  return (PIND & 0x08);
        case PD_4:  return (PIND & 0x10);
        case PD_5:  return (PIND & 0x20);
        case PD_6:  return (PIND & 0x40);
        case PD_7:  return (PIND & 0x80);
#else
        case PB_0:  return (PINB & 0x01);
        case PB_1:  return (PINB & 0x02) ? 1 : 0;
        case PB_2:  return (PINB & 0x04) ? 1 : 0;
        case PB_3:  return (PINB & 0x08) ? 1 : 0;
        case PB_4:  return (PINB & 0x10) ? 1 : 0;
        case PB_5:  return (PINB & 0x20) ? 1 : 0;

        case PC_0:  return (PINC & 0x01);
        case PC_1:  return (PINC & 0x02) ? 1 : 0;
        case PC_2:  return (PINC & 0x04) ? 1 : 0;
        case PC_3:  return (PINC & 0x08) ? 1 : 0;
        case PC_4:  return (PINC & 0x10) ? 1 : 0;
        case PC_5:  return (PINC & 0x20) ? 1 : 0;

        case PD_2:  return (PIND & 0x04) ? 1 : 0;
        case PD_3:  return (PIND & 0x08) ? 1 : 0;
        case PD_4:  return (PIND & 0x10) ? 1 : 0;
        case PD_5:  return (PIND & 0x20) ? 1 : 0;
        case PD_6:  return (PIND & 0x40) ? 1 : 0;
        case PD_7:  return (PIND & 0x80) ? 1 : 0;
#endif
    }

    return 0;

}

// NAME:        mcu_bit_pulse
// DESCRIPTION: Drive pin high then low
void mcu_bit_pulse(uchar pin)
{
    mcu_bit_out(pin,1);
    mcu_bit_out(pin,0);
}

// NAME:        mcu_bit_spi
// DESCRIPTION: Send a byte out SPI-style
void mcu_bit_spi(uchar pin_c,uchar pin_d,uchar value)
{
    uchar bit;

    mcu_bit_out(pin_c,0);                      // FIXME not optimized
    for(bit = 0;bit < 8;bit++)
    {
//        mcu_bit_out(pin_d,(value & 0x80) ? 1 : 0); // MSB first
        mcu_bit_out(pin_d,value & 0x80); // MSB first
        mcu_bit_out(pin_c,1);
        mcu_bit_out(pin_c,0);
//        mcu_bit_pulse(pin_c);
        value <<= 1;
    }
}

#ifdef BITSPI_PORT_CLOCK
#define bitspi_pulse() \
    BITSPI_PORT_CLOCK |=  BITSPI_CLOCK;\
    BITSPI_PORT_CLOCK &= ~BITSPI_CLOCK;

#define bitspi_out(mask) \
    if(value & mask)\
        BITSPI_PORT_DATA |=  BITSPI_DATA;\
    else\
        BITSPI_PORT_DATA &= ~BITSPI_DATA;

// NAME:        mcu_bit_spi_fast
// DESCRIPTION: Optimizes SPI bit banger, 2 dedicated pins
void mcu_bit_spi_fast(uchar value)
{
    BITSPI_PORT_CLOCK &= ~BITSPI_CLOCK;
    bitspi_out(0x80);
    bitspi_pulse();
    bitspi_out(0x40);
    bitspi_pulse();
    bitspi_out(0x20);
    bitspi_pulse();
    bitspi_out(0x10);
    bitspi_pulse();
    bitspi_out(0x08);
    bitspi_pulse();
    bitspi_out(0x04);
    bitspi_pulse();
    bitspi_out(0x02);
    bitspi_pulse();
    bitspi_out(0x01);
    bitspi_pulse();
}
#endif

#ifdef HAS_BYTE_IO
// NAME:        mcu_byte_out
// DESCRIPTION: Output a GPIO byte (to a port)
void mcu_byte_out(uchar pin,uchar value)
{
    switch(pin)
    {
        case PB_0:
        case PB_1:
        case PB_2:
        case PB_3:
        case PB_4:
        case PB_5:  PORTB = value; break;

        case PC_0:
        case PC_1:
        case PC_2:
        case PC_3:
        case PC_4:
        case PC_5:  PORTC = value; break;

        case PD_0:
        case PD_1:
        case PD_2:
        case PD_3:
        case PD_4:
        case PD_5:
        case PD_6:
        case PD_7:  PORTD = value; break;
    }
}

// NAME:        mcu_byte_in
// DESCRIPTION: Input a GPIO byte (from a port)
uchar mcu_byte_in(uchar pin)
{
    switch(pin)
    {
        case PB_0:
        case PB_1:
        case PB_2:
        case PB_3:
        case PB_4:
        case PB_5:  return PORTB;

        case PC_0:
        case PC_1:
        case PC_2:
        case PC_3:
        case PC_4:
        case PC_5:  return PORTC;

        case PD_0:
        case PD_1:
        case PD_2:
        case PD_3:
        case PD_4:
        case PD_5:
        case PD_6:
        case PD_7:  return PORTD;
    }
    return 0;
}
#endif

// NAME:        mcu_delay_100us
// DESCRIPTION:  Delay 100 microseconds
void mcu_delay_100us(void)
{
   uint i;

   for(i = 0;i < 90;i++)   // more or less
       ;
}

//------------------------------------------------------------------------------
// CODE: I2C
//------------------------------------------------------------------------------

#if NUM_I2CS > 0 || defined(EAU_HAS_I2C)
// for prescaler = 1 , min baud rate = 31 khz
// for prescaler = 4, min baud rate = 8 khz
// for prescaler = 16 , min baud rate = 2 khz
// for prescaler = 64 , min baud rate = 500 hz

// TWCR  control register: TWEN, TWINT,TWSTA,TWSTO,TWEA
//       TWINT = indication that last action ordered is complete
//       TWSTA = generate a start
//       TWSTO = generate a stop
//       TWEA  = got an ack (if master)
// TWSR  status register
// TWINT indicates an action has been completed
//
// API calls
//
//    1. _start
//    2. _is_busy  (continue if true)
//    3. _tx_ready (will be if TWINT is set)
//    4. _tx_c     (sending address)
//    LOOP:
//    5. _tx_ready (will be if TWINT is set)
//    6. _tx_c     (sending data)
//    7. _get_ack
//
//    8. _stop
char    i2c_ack_state = 0;   // 0 is nack, 1 is ack

#define _i2c_twint_clear()   ((1 << TWEN) | (1 << TWINT))
#define _i2c_twint_active()  (TWCR & (1 << TWINT))
#define _i2c_status()        (TWSR & 0xf8)

// NAME:        mcu_i2c_init
// DESCRIPTION: Initialises I2C
void mcu_i2c_init (char isMaster)
{
    TWCR         = 0;
	i2c_has_sent = 0;
    TWSR         = 0x00;
    ///mcu_i2c_set_baud(20000);
    TWBR = (uchar)(72);
    mcu_bit_init    (PC_4,MCU_PIN_PULLUP);
    mcu_bit_init    (PC_5,MCU_PIN_PULLUP);
   // TWCR        |= (1 << TWEN);
}

// NAME:        i2c_channel
// DESCRIPTION: Selects channel for MCUs with multi-I2Cs
void mcu_i2c_channel (uchar ch)
{
    // nothing with this mcu
}

// NAME:        mcu_i2c_set_baud
// DESCRIPTION: set the baud rate 31k - 1M
void mcu_i2c_set_baud(long baud)
{
  //  if((baud >= I2C_BAUD_MIN)
  //  && (baud <= I2C_BAUD_MAX))
  //  {
      /*  if(baud >= 31000)*/
        //{
            TWBR = (uchar)72;//(((F_CPU / baud) - 16) / (2 * 4));
            //TWSR = 0x00;
        /*}
        else
        if(baud >= 8000)
        {*/
      //      TWBR = (uchar)(((F_CPU / baud) - 16) / (2 * 4));
      //      TWSR = 0x01;
        /*}
        else
        if(baud >= 2000)
        {*/
        //    TWBR = (uchar)(((F_CPU / baud) - 16) / (2 * 16));
        //    TWSR = 0x02;
        /*}
        else
        {*/
          //  TWBR = (uchar)(((F_CPU / baud) - 16) / (2 * 64));
          //  TWSR = 0x03;
        //}
   // }
}

// NAME:        mcu_i2c_restart
// DESCRIPTION: Generate a restart condition
void mcu_i2c_restart(void)
{
	TWCR = _i2c_twint_clear() | (1 << TWSTA); // same as start
}

// NAME:        mcu_i2c_start
// DESCRIPTION: generate a Start condition on the 2-wire Serial Bus
//              After start is generated, TWINT=1, status = 0x08
void mcu_i2c_start(void)
{
    TWCR = _i2c_twint_clear() | (1 << TWSTA); // generate start, avoiding RMW
}

// FIXME should we have is_busy indicating if other cycle underway?
//       even if other MCU has the bus?
//       maybe need mcu_i2c_has_bus?


// NAME:        mcu_i2c_has_started
// DESCRIPTION: Indicate if bus is busy after a START command ie did start work
char mcu_i2c_has_started (void)
{
	if(_i2c_twint_active())
	{
	    return (_i2c_status() == 0x08) ? 1 : 0;
	    // also 0x10 for repeated start
	}

    return 0;
}

// FIXME is cycle complete?

// NAME:        mcu_i2c_get_ack
// DESCRIPTION: check ack: 0=not ready 1=ack 2=nack
char mcu_i2c_get_ack(void)
{
	uchar status;

	if(_i2c_twint_active())
	{
		status = _i2c_status();
		if((status == 0x18)       // ack, SLA+W
		|| (status == 0x28)       // ack, data transmit
		|| (status == 0x40)       // ack, SLA+R
		|| (status == 0x50)       // ack, data received
		)
			return 1;
		if((status == 0x20)       // nack, SLA+W
		|| (status == 0x30)       // nack, data transmit
		|| (status == 0x38)       // arbitration lost
		|| (status == 0x48)       // nack, SLA+R
		|| (status == 0x58)       // nack, data received
		)
		    return 2;             // nack
	}

	return 0;
}

// NAME:         mcu_i2c_stop
// DESCRIPTION:  Generate a stop condition
void mcu_i2c_stop(void)
{
   TWCR = _i2c_twint_clear() | (1 << TWSTO); // generate stop, avoiding RMW
}

// NAME:         mcu_i2c_end
// DESCRIPTION:  Complete a comm cycle, resetting the STOP bit
void mcu_i2c_end(void)
{
   TWCR = _i2c_twint_clear();
}

// NAME:        mcu_i2c_tx_ready
// DESCRIPTION: Indicate if I2C is ready to transmit
//    We assume this is in the context of sending a packet
//    So just looking for a TWINT high
//    It will be always, if START or address was sent
char mcu_i2c_tx_ready(void)
{
	char is_complete;

	if(!i2c_has_sent)                              // TWINT isn't set on the first xmit
    	    return TRUE;
	is_complete = _i2c_twint_active() ? TRUE : FALSE;
    if(is_complete)
    	     i2c_has_sent = FALSE;

    return !i2c_has_sent || is_complete;           // NOTE had been | not ||
}

// NAME:        mcu_i2c_tx_c
// DESCRIPTION: Transmit a character
//    As tx_ready will have been called beforehand
//    This is just writing the data register
void mcu_i2c_tx_c(uchar c)
{

   TWDR = c;
   TWCR = _i2c_twint_clear();   // clear TWINT flag to move forward
   i2c_has_sent = TRUE;
  //uA_tx_nybble(c >> 4);
  //uA_tx_nybble(c & 0xf);
}

// NAME:        mcu_i2c_rx_start
// DESCRIPTION: Start a read cycle, specifying ACK or NAK
//              Sending ACK tells the slave we will keep receiving
void mcu_i2c_rx_start (char ack)
{
	uchar control;

	control = _i2c_twint_clear();
	if(ack)
		control |= (1 << TWEA);
	TWCR = control;
}

// NAME:        mcu_i2c_rx_has
// DESCRIPTION: Report if a byte received
//    Assume packet read has been initiated etc
//    So we are waiting for a TWINT with a read success status
//    0x50 and 0x58 status codes indicate read success, with ACK/NACK
char mcu_i2c_rx_has (void)
{
	char status;

	if(!_i2c_twint_active())
		return FALSE;
	status = ((_i2c_status() & 0xf0) == 0x50) ? TRUE : FALSE;
  //  TWCR   = _i2c_twint_clear();
    	    // FIXME where to store NACK/ACK?
	return status;
}

// NAME:        mcu_i2c_rx_c
// DESCRIPTION: Receive data
//   High level will have called rx_has first
//   So we assume conditions are correct to get
//   Therefore this is just reading the data from register
uchar mcu_i2c_rx_c(void)
{
   return TWDR;
}


// NAME:        mcu_i2c_rx_request
// DESCRIPTION:
void mcu_i2c_rx_request (uchar adr,uchar n)
{
  // FIXME this is needed!
}

#endif



/*
// NAME:        mcu_get_reset
// DESCRIPTION: Get the kind of reset that last occurred
uchar mcu_get_reset(void)
{
    return 0;
}

// NAME:           mcu_watchdog_init
// DESCRIPTION:    Init the watchdog
void mcu_watchdog_init(void)
{
}

// NAME:           mcu_watchdog_service
// DESCRIPTION:
void mcu_watchdog_service(void)
{
}

// NAME:           mcu_stop
// DESCRIPTION:    Stop MCU
void mcu_stop(void)
{
}
*/
/*
// NAME:        mcu_i2c_get_status
// DESCRIPTION: get I2C status
uchar mcu_i2c_get_status(void)
{
    return (TWCR & (1 << TWINT)) ?  (TWSR & 0xF8) : 0;          // check for end of transmission
}
*/
/*
// NAME:        mcu_i2c_get_nack
// DESCRIPTION: check not ack
uchar mcu_i2c_get_nack (void)
{
    TWCR = _i2c_twint_clear();        // start receiving without acknowledging reception
    return (TWCR & (1 << TWINT)) ?  TWDR : 0; // FIXME WRONG
}
*/


/*
// NAME: i2c_set_baud
// DESCRIPTION: set the baud rate
void mcu_i2c_set_baud(long baud)
{
if((baud >= I2C_BAUD_MIN) && (baud <= I2C_BAUD_MAX))
{
if (baud >= 31000)
{
TWBR = (uchar)(((F_CPU / baud) - 16) / 2);
TWSR = 0x00;
}
else if(baud >= 8000)
{
TWBR = (uchar)(((F_CPU / baud) - 16) / (2*4));
TWSR = 0x01;
}
else if(baud >= 2000)
{
TWBR = (uchar)(((F_CPU / baud) - 16) / (2*16));
TWSR = 0x02;
}
else
{
TWBR = (uchar)(((F_CPU / baud) - 16) / (2*64));
TWSR = 0x03;
}
}
}*/

/*
switch(pin)
{
#if ISR_TIMER != 0
case PD_6:
    OCR0A = v;                        // set timer compare value (duty cycle)
    myPwms[PWM_CH_0A] = value;        // save value for later recall
    break;                            // FIXME why not read directly from OCRxx ??
case PD_5:
    OCR0B = v;
    myPwms[PWM_CH_0B] = value;
    break;
#endif
#if (ISR_TIMER != 1)
case PB_1:
    OCR1A = v;
    myPwms[PWM_CH_1A] = value;
    break;
case PB_2:
    OCR1B = v;
    myPwms[PWM_CH_1B] = value;
    break;
#endif
#if ISR_TIMER != 2
case PB_3:
    OCR2A = v;
    myPwms[PWM_CH_2A] = value;
    break;
case PD_3:
    OCR2B = v;
    myPwms[PWM_CH_2B] = value;
    break;
#endif
}*/

/*
 * oddly, this code uses about 100 bytes more RAM
 * than the current code as above (8 bytes less flash!)
    case PB_0: bit = 0; reg = &DDRB;break;
        case PB_1: bit = 1; reg = &DDRB;break;
        case PB_2: bit = 2; reg = &DDRB;break;
        case PB_3: bit = 3; reg = &DDRB;break;
        case PB_4: bit = 4; reg = &DDRB;break;
        case PB_5: bit = 5; reg = &DDRB;break;

        case PC_0: bit = 0; reg = &DDRC;break;
        case PC_1: bit = 1; reg = &DDRC;break;
        case PC_2: bit = 2; reg = &DDRC;break;
        case PC_3: bit = 3; reg = &DDRC;break;
        case PC_4: bit = 4; reg = &DDRC;break;
        case PC_5: bit = 5; reg = &DDRC;break;
*/

/*
#if NUM_PULSES > 0
FLASH_VAR(const uchar pulse_chan[20]) = // TDM of pulse channel
{                                       // pulses are skewed ie not time-aligned
    1,1,1,2,2,2,3,3,3,4,4,4,
    0,0,0,0,0,0,0,0
};
#endif
static uint  spi_rxBuffer[SPI_BUFFER_SIZE];
static uchar spi_rxBufferIdx = 0;

#define PULSE_STATE_IDLE  0
#define PULSE_STATE_START 1
#define PULSE_STATE_255   2
#define PULSE_STATE_0_END 3
#define PULSE_STATE_1_0   4
#define PULSE_STATE_1_END 5
#define PULSE_STATE_2_0   6
#if NUM_PULSES > 0
static uint  myPulseNext [4];
static uint  myPulseTimer[4];
static uchar myPulseState[4];
#endif

*/
/*
void _mcu_pulse_start(uchar ch)
{
    if(ch == 0)
    {
        OCR2B   = 255;
        TCCR2A |= 0x20;              // change to PWM
    }
    else
    if(ch == 1)
    {
        OCR0B   = 255;
        TCCR0A |= 0x20;              // change to PWM
    }
    else
    if(ch == 2)
    {
        OCR0A   = 255;
        TCCR0A |= 0x80;              // change to PWM
    }
    else
    {
        OCR2A   = 255;
        TCCR2A |= 0x80;              // change to PWM
    }
}
*//*
void _mcu_pulse_end(uchar ch)
{
if(ch == 0)
{
    PORTD  &= ~0x08;                 // PD3 low
    TCCR2A &= ~0x30;                 // stop generating PWM
    OCR2B   = 255;
}
else
if(ch == 1)
{
    PORTD  &= ~0x20;                 // PD5 low
    TCCR0A &= ~0x30;                 // stop generating PWM
    OCR0B   = 255;
}
else
if(ch == 2)
{
    PORTD  &= ~0x40;                 // PD6 low
    TCCR0A &= ~0xc0;                 // stop generating PWM
    OCR0A   = 255;
}
else
{
    PORTB  &= ~0x08;                 // PB3 low
    TCCR2A &= ~0xc0;                 // stop generating PWM
    OCR2A   = 255;
}
}*/


/*
// NAME:        mcu_pulse_new
// DESCRIPTION:
void _mcu_pulse_new(void)
{
    myPulseTimer[0] = myPulseNext[0];
    myPulseState[0] = PULSE_STATE_START;
     myPulseTimer[1] = myPulseNext[1];
    myPulseState[1] = PULSE_STATE_START;
    myPulseTimer[2] = myPulseNext[2];
    myPulseState[2] = PULSE_STATE_START;
    myPulseTimer[3] = myPulseNext[3];
    myPulseState[3] = PULSE_STATE_START;
}
*/

/*
// NAME:        mcu_eeprom_init
// DESCRIPTION: Initialize EEPROM
void  mcu_eeprom_init(void)
{
    // nothing required
}
*/
