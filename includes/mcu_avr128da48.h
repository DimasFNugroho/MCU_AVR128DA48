// NAME:        mcu_avr128da48.h
// DESCRIPTION: 
// DATE:        Feb 1 2021
//
// copyright (c) 2021 Before Technology Limited

#ifndef AVR128DA48_H
#define AVR128DA48_H

#define MCU_PORTS     3                // GPIO port count
#define MCU_ENDIAN    MCU_ENDIAN_BIG
#define MCU_ADC_BITS  10
#define I2C_BAUD_MIN  31000L
#define I2C_BAUD_MAX  1000000L
#define SPI_SS_HARD              // hardware SPI slave select

//#define mcu_interrupts_off()  cli()
//#define mcu_interrupts_on()   sei()

enum
{
  DUMMY,
  PD_3,    // PIN 1  D3  PWM
  PD_4,    // PD4   - PIN 2  D4
  GND,     // GND   - PIN 3
  VCC,     // VCC   - PIN 4
  GND2,    // GND   - PIN 5
  VCC2,    // VCC   - PIN 6
  OSC,     // OSC   - PIN 7
  OSC2,    // OSC   - PIN 8

  PD_5,    // PD5   - PIN 9  D5  PWM
  PD_6,    // PD6   - PIN 10 D6  PWM
  PD_7,    // PD7   - PIN 11 D7
  PB_0,    // PB0   - PIN 12 D8
  PB_1,    // PB1   - PIN 13 D9  PWM
  PB_2,    // PB2   - PIN 14 D10 PWM
  PB_3,    // PB3   - PIN 15 D11 PWM MOSI
  PB_4,    // PB4   - PIN 16 D12 MISO

  PB_5,    // PB5   - PIN 17 D13 SCK
  AVCC,    // AVCC  - PIN 18
  ADC6,    // ADC6  - PIN 19
  AREF,    // AREF  - PIN 20
  GND3,    // GND   - PIN 21
  ADC7,    // ADC7  - PIN 22
  PC_0,    // PC0   - PIN 23 A0
  PC_1,    // PC1   - PIN 24 A1

  PC_2,    // PC2   - PIN 25 A2
  PC_3,    // PC3   - PIN 25 A3
  PC_4,    // PC4   - PIN 27 A4  SDA
  PC_5,    // PC5   - PIN 28 A5  SCL
  RESET,   // RESET - PIN 29
  PD_0,    // PD0   - PIN 30 RX
  PD_1,    // PD1   - PIN 31 TX
  PD_2     // PD2   - PIN 32 D2
} pin_enum;

#endif



