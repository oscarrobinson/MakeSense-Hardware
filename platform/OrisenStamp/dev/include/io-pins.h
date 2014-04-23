/**
 * \defgroup orisen-stamp-io-pins Orisen Stamp IO Pins
 *
 * @{
 */

/**
 * \file
 *				This code allows manipulation of the io pins on
 *				the Orisen Stamp
 * \author
 *         Stephen Hailes
 */
#ifndef IOPINS_H
#define IOPINS_H

#include "contiki.h"
#include "lib/include/gpio.h"
#include "adc.h"


/*
 * Definitions for each of the 20 pins - for the
 * signal pins, we give the function to which the pin is
 * initialised, and the GPIO connection that the pin can
 * also be changed into.
 *
 * Pin    Initial function			Additional list										Arrangement
 * ---    ----------------			---------------							------------------------
 * T1			SDA  (I2C)							GPIO13										T15									 T20
 * T2			SCL  (I2C)							GPIO12										T16									 T19
 * T3			GPIO09									TMR1											T7									 T18
 * T4			GPIO08									TMR0											T8									 T17
 * T5			TX   (UART2)						GPIO18										T6									 T11
 * T6			RX   (UART2)						GPIO19										T5									 T12
 * T7			RTS  (UART2)						GPIO21										T1   T3					T9   T13
 * T8			CTS  (UART2)						GPIO20										T2   T4					T10  T14
 * T9			KBI7										GPIO29
 * T10		KBI6										GPIO28										ADC0								 GND
 * T11		SS   (SPI)							GPIO04										ADC1								 VCC
 * T12		MISO (SPI)							GPIO05										RTS									 GND
 * T13		MOSI (SPI)							GPIO06										CTS									 VIN
 * T14		SCK  (SPI)							GPIO07										RX									 SS
 * T15		ADC0										GPIO30										TX									 MISO
 * T16		ADC1										GPIO31										SDA  GPIO9			KBI7 MOSI
 * T17		VIN																								SCL  GPIO8			KBI6 SCK
 * T18		GND
 * T19		VCC
 * T20		GND
*/

typedef enum {USEGPIO, ALTERNATE} 						pinFunction;
typedef enum {INPUT, OUTPUT} 									pinDirection;
typedef enum {PULLUP, PULLDOWN, NOPULLUP} 		pinPullup;
typedef enum {HYSTERESIS_ON, HYSTERESIS_OFF}	pinHysteresis;

typedef struct {
	pinFunction			function;
	pinDirection		direction;
	pinPullup				pullup;
	pinHysteresis		hysteresis;
} ioPinConfig;

typedef struct {
	uint8_t					pin;
	ioPinConfig			config;
} ioPin;	

static ioPin initialSettings[] = {
	{ 13, {ALTERNATE,		OUTPUT,		  PULLUP,		HYSTERESIS_OFF}},	// T1  SDA
	{ 12, {ALTERNATE,		OUTPUT,		  PULLUP,		HYSTERESIS_OFF}},	// T2  SCL
	{  9, {USEGPIO,			INPUT,		NOPULLUP,		HYSTERESIS_OFF}},	// T3  GPIO09
	{  8, {USEGPIO,			INPUT,		NOPULLUP,		HYSTERESIS_OFF}},	// T4  GPIO08
	{ 18, {ALTERNATE,		OUTPUT,		NOPULLUP,		HYSTERESIS_OFF}},	// T5  UART2 TX
	{ 19, {ALTERNATE,		INPUT,		NOPULLUP,		HYSTERESIS_OFF}},	// T6  UART2 RX
	{ 21, {ALTERNATE,		OUTPUT,		NOPULLUP,		HYSTERESIS_OFF}},	// T7  UART2 RTS
	{ 20, {ALTERNATE,		INPUT,		NOPULLUP,		HYSTERESIS_OFF}},	// T8  UART2 CTS
	{ 29, {ALTERNATE,		INPUT,		  PULLUP,		HYSTERESIS_OFF}},	// T9  KBI7
	{ 28, {ALTERNATE,		INPUT,		  PULLUP,		HYSTERESIS_OFF}},	// T10 KBI6
	{ 04, {ALTERNATE,		OUTPUT,		NOPULLUP,		HYSTERESIS_OFF}},	// T11 SS
	{ 05, {ALTERNATE,		INPUT,		  PULLUP,		HYSTERESIS_OFF}},	// T12 MISO
	{ 06, {ALTERNATE,		OUTPUT,		NOPULLUP,		HYSTERESIS_OFF}},	// T13 MOSI
	{ 07, {ALTERNATE,		OUTPUT,		NOPULLUP,		HYSTERESIS_OFF}},	// T14 SCK
	{ 30, {ALTERNATE,		INPUT,		NOPULLUP,		HYSTERESIS_OFF}},	// T15 ADC0
	{ 31, {ALTERNATE,		INPUT,		NOPULLUP,		HYSTERESIS_OFF}}	// T16 ADC1
};

#define SET_IO_PIN(pin, f, d, p, e, h) \
	GPIO->FUNC_SEL.pin    = f; \
	GPIO->PAD_DIR.pin     = d; \
	GPIO->PAD_PU_SEL.pin  = p; \
	GPIO->PAD_PU_EN.pin   = e; \
  GPIO->PAD_HYST_EN.pin = h;




void ioPins_init();
void ioPins_configurePin(uint8_t pin, pinFunction function, pinDirection direction, pinPullup pullup, pinHysteresis hysteresis);
void ioPins_setValue(uint8_t pin, uint8_t value);
uint32_t ioPins_getValue(uint8_t pin);
uint32_t ioPins_getBatt();

#endif

/** @} */
