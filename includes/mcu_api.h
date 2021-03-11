// NAME:        mcu_api.h
// DATE:        Feb 24 2016
//
// Development sequence:
//
// 1  initialization, timer interrupt, interrupt on/off, delay
// 2  uart
// 3  byte, bit
// 4  eeprom
// 5  analog, dac
// 6  pwm, pulse
// 7  spi
// 8  i2c
// 9  touch
// 10 can
// 11 flash
// 12 i2s
//
// Development system:
//
// An Uno-compatible hardware device with desired MCU
// Grove shield
// Grove cable ->scope probe (quick switching system would be ideal)
// ComTest Uno with LCD screen
//
// Windows computer
// ComTest software running on PC
// McuDev software running on PC (used for editing and updating project C files)
// Native development system (used to compile and upload the project)
//
// NOTE ideally a bootstrap loader would be installed, so that once uart is in place,
// the remainder of the process (steps 3+) could be performed without the native software.

// The project's main.c will contain a command interface to specify tests.
//
// Development process:
//
// 1a init:   flip output pin in 25us timer interrupt, scope it, ensure period is correct)
// 1b init:   flip output pin from delay entry/exit, scope it, ensure period is correct
// 2  uart:   at 57600, transmit a startup message, echo chars typed but with addition
// 3a byte:   flip port on, port off, then each port bit in sequence, scope
// 3b bit:    flip all bits on, all off, then all in sequence, scope
// 3c bit:    set all bits to input, scan, print out bit values as they change
// 4  eeprom: write, read a sequence, verify by comparison
// 5a analog: scan, print out analog values as they change
// 6  pwm:    vary pwm from 0-100, scope, verify period and changing duty cycle
// 7  spi:    use comtest to verify
// 8  i2c:    use comtest to verify
// 9  can:    ?
// 10 flash:  same test as eeprom

// copyright (c) 2004-17 Before Technology Limited

#ifndef MCU_API
#define MCU_API

//------------------------------------------------------------------------------
// INCLUDES
//------------------------------------------------------------------------------

#include "uA_general.h"

//------------------------------------------------------------------------------
// DEFINES
//------------------------------------------------------------------------------

#define MCU_PIN_INPUT	    0
#define MCU_PIN_OUTPUT	    1
#define MCU_PIN_ANALOG	    2
#define MCU_PIN_PULLUP	    3
#define MCU_PIN_PULLDOWN	    4
#define MCU_PIN_OC   	    5
#define MCU_PIN_DRIVE	    6 // high drive output
#define MCU_PIN_SLEW	    		7 // output w/slew rate control
#define MCU_PIN_DRIVESLEW	8 // high drive & slew rate control

#define MCU_BAUD_INIT       57600L

#define MCU_ENDIAN_LITTLE   0
#define MCU_ENDIAN_BIG      1

#define MCU_BOOTLOADER_NONE     0    // based upon AVR bootloader size bits
#define MCU_BOOTLOADER_STANDARD 1    // 2k
#define MCU_BOOTLOADER_UNKNOWN  2    // 1k
#define MCU_BOOTLOADER_OPTIBOOT 3    // 512

//------------------------------------------------------------------------------
// DATA TYPES
//------------------------------------------------------------------------------

typedef struct  // a message, including its description
{
    uint   id;
    uint   id_mask;           
    uchar  priority;    
    uint   period;           // period oftransmission (ms)
    uchar  length;
    uchar  data[8];
    uint   timestamp;        // time received/sent (0- 999.9 ms)
}can_message_t;

//------------------------------------------------------------------------------
// FUNCTION PROTOTYPES
//------------------------------------------------------------------------------

// Functions marked with * are essential for a basic OS build 
// To perform a port to a new MCU, create these functions first
// All other functions can be stubs
// A system ISR must be defined, calling into service_foreground();

void  mcu_init			    (void);                   // * must init systick ISR
void  mcu_service           (void);                   // for use of sockets, most often empty
uint  mcu_get_family	  	    (void);                   // provide an code for this MCU family (eg AVR)
uint  mcu_get_mcu 	  	    (void);                   // provide an code for this MCU model (eg atmega328p)
ulong mcu_get_id		        (uchar index);            // provide an unique ID for this MCU (ie a 96 bit number)
uchar mcu_get_bootloader		(void);                   // provide a bootloader code
uchar mcu_get_endian		  	(void);                   // little or big = 0/1
uchar mcu_get_bits		  	(void);                   // 8, 16, or 32

void  mcu_delay_100us       (void);                    // 100 microsecond delay in a loop
// change these two to macros
void  mcu_interrupts_off    (void);                    // *
void  mcu_interrupts_on	    (void);                    // *
void  mcu_timer_start       (void);                    // zero and start a 16 bit timer
uint  mcu_timer_stop        (void);                    // stop 16b timer and return value

void  mcu_uart_init  	    (uchar ch);                 // *
void  mcu_uart_set_baud     (uchar ch,long baud);       // *
void  mcu_uart_set_parity   (uchar ch,uchar parity);
void  mcu_uart_set_bits     (uchar ch,uchar bits);
void  mcu_uart_tx_enable    (uchar ch,char state);      // *
char  mcu_uart_tx_ready     (uchar ch);                 // *
void  mcu_uart_tx_c  	    (uchar ch,uchar c);         // *
void  mcu_uart_tx_break     (uchar ch,char state);
void  mcu_uart_rx_enable    (uchar ch,char state);      // *
char  mcu_uart_rx_has	    (uchar ch);                 // *
uchar mcu_uart_rx_c  	    (uchar ch);                 // *
char  mcu_uart_rx_break     (uchar ch);
uchar mcu_uart_get_errors   (uchar ch);

void  mcu_byte_out	        (uchar pin,uchar value);               // port I/O
uchar mcu_byte_in		    (uchar pin);                           // but referenced by pin

void  mcu_bit_init		    (uchar pin,uchar mode);
void  mcu_bit_out	        (uchar pin,uchar state);
char  mcu_bit_in		    (uchar pin);
void  mcu_bit_pulse		    (uchar pin);
void  mcu_bit_spi		    (uchar pin_c,uchar pin_d,uchar value); // send out 8 bits SPI-style
void  mcu_bit_spi_fast      (uchar value);                          // SPI with compiled-in banger pins

void  mcu_eeprom_init       (void);
void  mcu_eeprom_erase      (uchar *adr,uint n);
uchar mcu_eeprom_read	    (uchar *adr);
uint  mcu_eeprom_read_w	    (uchar *adr);                         // fixme uint*?
void  mcu_eeprom_write      (uchar *adr,uchar value);             // *
void  mcu_eeprom_write_w    (uchar *adr,uint  value);             // *
void  mcu_eeprom_write_n    (uchar *adr,uchar *values,uint n);

uchar mcu_analog_channel    (uchar pin);
void  mcu_analog_init       (uchar base_channel,uchar n_channels);
void  mcu_analog_pin_enable (uchar pin);
void  mcu_analog_reference  (uchar reference);                     // internal, external, etc
uint  mcu_analog_in         (uchar ch);
void  mcu_analog_service	(void);
uchar mcu_analog_get_bits   (void);
void  mcu_analog_set_bits   (uchar bits);

void  mcu_analog_fast_init    (void);
uint  mcu_analog_fast_in      (void);
void  mcu_analog_fast_service (void);


void  mcu_dac_init          (uchar pin);
void  mcu_dac_out           (uchar pin,uint value);
uchar mcu_dac_get_bits      (void);

void  mcu_touch_init		    (uchar pin);                // configure pin as touch
char  mcu_touch_in		    (uchar pin);                // get touch state

void  mcu_servo_init        (uchar pin);                // 16b PWM
void  mcu_servo_set         (uchar pin,uint angleTimesTen);
void  mcu_servo_set_pulse   (uchar pin,uint  usecs);    // pulse width in microseconds

void  mcu_tone_init         (uchar pin);

void  mcu_pwm16_init        (uint range);
void  mcu_pwm16_set         (uchar pin,uint pwm16b);
void  mcu_pwm16_set_period  (uchar pin,uint period);

void  mcu_pwm_init          (void);
uchar mcu_pwm_channel       (uchar pin);                // return PWM channel for pin
void  mcu_pwm_service       (void);                     // PWM banger
void  mcu_pwm_pin_set       (uchar ch,uchar pin);
void  mcu_pwm_enable_set    (uchar pin,uchar state);    // 8/16b
uint  mcu_pwm_get           (uchar pin);

void  mcu_pwms_set          (uchar pin,uchar pwm100);
void  mcu_pwm_set           (uchar pin,uchar pwm100);   // 8b only (includes 16b timer as 8b)
void  mcu_pwm_set_8b        (uchar pin,uchar pwm8b);

void  mcu_pulse_enable_set  (uchar pin,char state);
void  mcu_pulse_set         (uchar pin,uint duration);
void  mcu_pulse_service     (void);                     // call at ISR rate (~25us)
void  mcu_pulse_stop        (uchar pin);

void  mcu_spi_init          (uchar ch,uchar isMaster,char isWord);
void  mcu_spi_set_baud      (uchar ch,long baud_rate);
void  mcu_spi_set_master    (uchar ch,char is_master);
char  mcu_spi_is_master     (uchar ch);
void  mcu_spi_tx_enable     (uchar ch,char state);
char  mcu_spi_tx_ready      (uchar ch);
char  mcu_spi_tx_c          (uchar ch,uchar  value);
char  mcu_spi_tx_w          (uchar ch,uint value);
uchar mcu_spi_tx_w_n        (uchar ch,uint value,uchar n);
char  mcu_spi_rx_has        (uchar ch);
char  mcu_spi_rx_c          (uchar ch,uchar *value);
char  mcu_spi_rx_w          (uchar ch,uint *value);
uchar mcu_spi_trx_c         (uchar ch,uchar  value);

void  mcu_i2c_init           (char isMaster);
void  mcu_i2c_set_channel    (uchar ch);
void  mcu_i2c_set_baud       (long baud);
char  mcu_i2c_tx_ready       (void);
void  mcu_i2c_tx_c           (uchar c);
void  mcu_i2c_rx_request     (uchar adr,uchar n);      // request n chars from slave at adr
void  mcu_i2c_rx_start       (char ack);
char  mcu_i2c_rx_has         (void);
uchar mcu_i2c_rx_c           (void);
void  mcu_i2c_start          (void);
void  mcu_i2c_restart        (void);
void  mcu_i2c_stop           (void);               // generate a stop condition on the bus
void  mcu_i2c_end            (void);               // do whatever might be necessary to end the cycle
char  mcu_i2c_is_busy        (void);               // bus already in use
char  mcu_i2c_has_started    (void);               // attempt at start succeeded
uchar mcu_i2c_get_status     (void);
char  mcu_i2c_get_ack        (void);
uchar mcu_i2c_get_nack       (void);
void  mcu_i2c_ack            (void);               // ack received byte

void  mcu_can_init          (char mode);               // MCU-specific (peripheral interface)
void  mcu_can_id_set        (uint id);
uchar mcu_can_tx            (can_message_t *msg);
uchar mcu_can_tx_is_busy    (uchar buffer);
void  mcu_can_rx_filter_set (uchar filter,uint adr,uint mask);
uchar mcu_can_rx_has        (void);
uchar mcu_can_rx            (can_message_t *msg);

char  mcu_flash_sector_erase (uchar *address);               // erase 512-byte sector
char  mcu_flash_sector_write (uchar *address,uchar *values); // write a sector
char  mcu_flash_sector_read  (uchar *address,uchar *values); // read a sector

void  mcu_usb_init              (void);
uchar mcu_usb_event_get         (void);
void  mcu_usb_event_clear       (uchar event);
void  mcu_usb_address_set       (uchar adr);
void  mcu_usb_address_enable    (void);
void  mcu_usb_control_configure (void);
void  mcu_usb_clock_set         (char state);
uchar mcu_usb_data_read       (void);
void  mcu_usb_data_write      (uchar value);
char  mcu_usb_tx_is_free      (void);
char  mcu_usb_tx_is_ready     (void);
void  mcu_usb_tx_flush        (void);
void  mcu_usb_tx_enable2      (void);
void  mcu_usb_tx_enable       (void);
void  mcu_usb_tx_send         (void);
void  mcu_usb_rx_free         (void);
char  mcu_usb_rx_is_free      (void);
char  mcu_usb_rx_is_full      (void);
char  mcu_usb_rx_has          (void);
void  mcu_usb_rx_ack          (void);
void  mcu_usb_setup_clear     (void);
char  mcu_usb_setup_request   (void);
void  mcu_usb_flags_clear     (void);
uint  mcu_usb_get_frame       (void);
char  mcu_usb_rw_allowed      (void);
char  mcu_usb_has_rxortx      (void);
char  mcu_usb_ep_set           (uchar ep);
char  mcu_usb_ep_configure     (uchar ep,uchar data_type,uchar buffer_size,uchar buffer_type);
void  mcu_usb_ep_deconfigure   (uchar ep);
char  mcu_usb_ep_is_stalled    (uchar ep);
void  mcu_usb_ep_reset         (uchar ep);
void  mcu_usb_stall_send       (void);
void  mcu_usb_stall_clear     (void);
void  mcu_usb_reset_all        (void);

void  mcu_i2s_init           (void);                  // I2S audio
void  mcu_i2s_tx_enable      (char state);
void  mcu_i2s_rx_enable      (char state);
void  mcu_i2s_service        (void);
char  mcu_i2s_set_frame      (uchar bits,uchar channels);
char  mcu_i2s_set_rate       (ulong rate);

void  mcu_dma_init           (uchar ch);                         // Direct Memory Access
void  mcu_dma_set_buffer     (uchar ch,uchar *buffer,uint size);
void  mcu_dma_set_source     (uchar ch,uchar src);               // source/target specified by code

ulong mcu_random_get         (void);

void  mcu_rtc_init           (void);
void  mcu_rtc_set_on         (char state);
ulong mcu_rtc_get_seconds    (void);                             // time in seconds
void  mcu_rtc_set_seconds    (ulong seconds);
void  mcu_rtc_calibrate      (long calibration);                // +- parts per billion
uchar mcu_rtc_get            (uchar field);
void  mcu_rtc_set            (uchar field,uchar value);

void  mcu_crc_init           (uchar size,char endian);
void  mcu_crc_start          (uchar *adr,ulong size);
char  mcu_crc_is_ready       (void);
ulong mcu_crc_get            (void);

void  mcu_aes_init           (void);
void  mcu_aes_start          (uchar *src,ulong size,uchar *dst); // AES encryption engine
char  mcu_aes_is_ready       (void);

// these are meant to be implemented, but aren't used yet
void  mcu_stop              (void);
uchar mcu_get_reset         (void);
void  mcu_watchdog_init     (void);
void  mcu_watchdog_feed     (void);

// some ideas for functions to be added:
//void  mcu_delay_us          (uint us);                // n microsecond delay in a loop
//char  mcu_uart_rx_idle      (uchar ch);
//uchar mcu_pwm_get_count     (uchar pin);

extern void service_foreground (void);                  // code for ISR

#endif

