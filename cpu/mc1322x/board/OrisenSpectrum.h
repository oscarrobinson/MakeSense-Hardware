/*
 * Copyright (c) 2013, Stephen Hailes <s.hailes@cs.ucl.ac.uk>
 *
 * Test code for mc1322x Spectrum functions
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * $Id$
 */

#ifndef BOARD_Spectrum_H
#define BOARD_Spectrum_H

/*******************************************************************************
Muxed IO lines
	IO_0
		KBI_4 = GPIO26	i/p
		KBI_0 = GPIO22	o/p or high-Z

	IO_1
		I_1   = MDO07 (GPIO58) i/p (function 1)
		KBI_1 = GPIO23	o/p or high-Z

	IO_2
		I_2   = MDO06 (GPIO57)	i/p (function 1)
		O_2   = MDO04 (GPIO55)	o/p or high-Z

	IO_3
		I_3   = MDO05 (GPIO56)	i/p (function 1)
		O_3   = MDO03 (GPIO54)	o/p or high-Z

***
	Initially,
		GPIO26 = i/p, ignore interrupts
		GPIO22 = i/p (high-Z), disable interrupts
		GPIO58 = i/p (function 1)
		GPIO23 = i/p, (high-Z), disable interrupts
		GPIO57 = i/p (function 1)
		GPIO55 = i/p (high-Z) (function 1)
		GPIO56 = i/p (function 1)
		GPIO54 = i/p (high-Z) (function 1)
*******************************************************************************/
#define IO_0_IN						GPIO_26
#define IO_0_OUT					GPIO_22
#define IO_0_OUT_LOW			(GPIO->DATA_RESET.IO_0_OUT = 1)
#define IO_0_OUT_HIGH			(GPIO->DATA_SET.IO_0_OUT = 1)
#define IO_0_READ					(GPIO->DATA.IO_0_IN)

#define IO_1_IN						GPIO_58
#define IO_1_OUT					GPIO_23
#define IO_1_OUT_LOW			(GPIO->DATA_RESET.IO_1_OUT = 1)
#define IO_1_OUT_HIGH			(GPIO->DATA_SET.IO_1_OUT = 1)
#define IO_1_READ					(GPIO->DATA.IO_1_IN)

#define IO_2_IN						GPIO_57
#define IO_2_OUT					GPIO_55
#define IO_2_OUT_LOW			(GPIO->DATA_RESET.IO_2_OUT = 1)
#define IO_2_OUT_HIGH			(GPIO->DATA_SET.IO_2_OUT = 1)
#define IO_2_READ					(GPIO->DATA.IO_2_IN)

#define IO_3_IN						GPIO_56
#define IO_3_OUT					GPIO_54
#define IO_3_OUT_LOW			(GPIO->DATA_RESET.IO_3_OUT = 1)
#define IO_3_OUT_HIGH			(GPIO->DATA_SET.IO_3_OUT = 1)
#define IO_3_READ					(GPIO->DATA.IO_3_IN)


/*******************************************************************************
	P2_SEL0 = ADC5 (GPIO35)
	P2_SEL1 = ADC6 (GPIO36)
	ADC5 and ADC6 are used as selectors to switches.
	when P2_SEL0 is high, then
		P2_DATA0 is connected to SDA
		P2_DATA1 is connected to SCL 
	when P2_SEL0 is low, then
		P2_DATA0 is connected to UART2_RX 
		P2_DATA1 is connected to UART2_TX 
	when P2_SEL1 is high, then
		IO_2_SW is connected to IO_2 (effectively GPIO55/57)
		IO_3_SW is connected to IO_3 (effectively GPIO54/56)
	when P2_SEL1 is low, then
		IO_2_SW is connected to UART2_RTS 
		IO_3_SW is connected to UART2_CTS 
***
	Initially,
		GPIO35 = o/p, value 1 (I2C line active)
		GPIO36 = o/p, value 1 (P2_IO lines active)
*******************************************************************************/
#define P2_SEL_0					GPIO_35
#define P2_SEL_0_LOW			(GPIO->DATA_RESET.P2_SEL_0 = 1)
#define P2_SEL_0_HIGH			(GPIO->DATA_SET.P2_SEL_0 = 1)

#define P2_SEL_1					GPIO_36
#define P2_SEL_1_LOW			(GPIO->DATA_RESET.P2_SEL_1 = 1)
#define P2_SEL_1_HIGH			(GPIO->DATA_SET.P2_SEL_1 = 1)


/*******************************************************************************
Status and control lines
		SD_A	 = MSEO0_B (GPIO59)	(uSD card detection - low when card present)
		SD_PWR = MSEO1_B (GPIO60) (uSD power up enable - active high)
		FAULT  = EVTO_B  (GPIO62)	(charging fault)
		ACPR   = EVTI_B  (GPIO63)	(charger power supply status - low when OK)
***
	Initially, 
		GPIO59 = i/p
		GPIO60 = o/p - initial value low (disabled)
		GPIO62 = i/p
		GPIO63 = i/p
*******************************************************************************/
#define SD_CARD_SWITCH		GPIO_59
#define SD_CARD_PRESENT		(!GPIO->DATA.SD_CARD_SWITCH)
#define SD_PWR_EN					GPIO_60
#define SD_PWR_EN_LOW		  (GPIO->DATA_RESET.SD_PWR_EN = 1)
#define SD_PWR_EN_HIGH		(GPIO->DATA_SET.SD_PWR_EN = 1)

#define CHARGE_FAULT_PIN	GPIO_62
#define CHARGE_FAULT			(!GPIO->DATA.CHARGE_FAULT_PIN)

#define CHARGE_ACPR_PIN		GPIO_63
#define CHARGE_POWER_OK		(!GPIO->DATA.CHARGE_ACPR_PIN)


/*******************************************************************************
Device interrupt status lines
		INT_RTC = MCKO  (GPIO50)
		INT_MPU = RDY_B (GPIO61)
		GPS_TIMEPULSE = MDO02 (GPIO53)

***
	Initially, 
		GPIO50 = i/p
		GPIO61 = i/p
		GPIO53 = i/p
*******************************************************************************/
#define INT_RTC						GPIO_50
#define INT_MPU						GPIO_61
#define INT_RTC_READ			(GPIO->DATA.INT_RTC)
#define INT_WPU_READ			(GPIO->DATA.INT_WPU)


/*******************************************************************************
	KBI2 = GPIO24
	KBI3 = GPIO25
	KBI5 = GPIO27
	KBI2, KBI3 and KBI5 are connected to the CHRG output from the Li-Ion charger
		KBI5 is the input that gives the state of charging
		KBI2/3 are used to determine what that state is - KBI2 is a !OE line to a buffer with input given by KBI3
			KBI2=1, KBI3=X =>
					KBI5==0	=> charging (for all values of charging current)
					KBI5==1	=> charging complete
				
			KBI2=0, KBI3=1 =>
					KBI5==0	=> charging (current > 10% of full scale)
					KBI5==1	=> charging but current has dropped below 10% of full scale

  ADC0 = GPIO30
		ADC0 is connected to an op-amp set up as a voltage follower (unity gain buffer)
		It measures the voltage given by a voltage divider set to divide VBAT in the
    ratio 1.2 : 3.3 (i.e. to give VBAT * 3.3/4.5 as input to the ADC, remembering that
    full scale voltage on ADC is 3.3V)
***
	Initially, 
		GPIO24 = o/p
		GPIO25 = i/p
		GPIO27 = i/p
		GPIO30 = i/p - ADC
*******************************************************************************/
#define CHARGE_STATUS_OE					GPIO_24
#define CHARGE_STATUS_OUT					GPIO_25
#define CHARGE_STATUS_IN					GPIO_27
#define CHARGE_STATUS_OE_LOW			(GPIO->DATA_RESET.CHARGE_STATUS_OE = 1)
#define CHARGE_STATUS_OE_HIGH			(GPIO->DATA_SET.CHARGE_STATUS_OE = 1)
#define CHARGE_STATUS_OUT_LOW			(GPIO->DATA_RESET.CHARGE_STATUS_OUT = 1)
#define CHARGE_STATUS_OUT_HIGH		(GPIO->DATA_SET.CHARGE_STATUS_OUT = 1)
#define CHARGE_STATUS_READ				(GPIO->DATA.CHARGE_STATUS_IN)

#define VBAT_IN										GPIO_30
#define VBAT_READ									(GPIO->DATA.VBAT_IN)

/*******************************************************************************
	KBI_6 = GPIO28	i/p interrupt from AND gate (I_1, I_2, I_3)
	KBI_7 = GPIO29	i/p interrupt from AND gate (INT_RTC, GPS_TIMEPULSE, INT_MPU)
	Can work out which because
		I_1	== GPIO58
		I_2	== GPIO57
		I_3	== GPIO56

		INT_RTC 			== GPIO50
		GPS_TIMEPULSE == GPIO53
		INT_MPU 			== GPIO61
		NB - see below, because P2_IO_2 and P2_IO_3 are switched and so potentially connected to
		different inputs
***
	Initially,
		GPIO28 = i/p, ignore interrupts
		GPIO29 = i/p, ignore interrupts
*******************************************************************************/

// Nothing to be set here; defaults will do.


/*******************************************************************************
LEDS
	TX_ON (GPIO44) = Green LED, active low
	RX_ON (GPIO45) = Blue  LED, active low
***
	Initially,
		GPIO44 = o/p, value 1 (off)
		GPIO45 = o/p, value 1 (off)
*******************************************************************************/

// There is no RED led on the board, so make this the same as green
#define GPIO_LED_BLUE  	GPIO_43
#define GPIO_LED_GREEN  GPIO_42
#define GPIO_LED_RED  	GPIO_42

#define LED1						GPIO_LED_BLUE
#define LED2						GPIO_LED_GREEN
#define LED3						GPIO_LED_RED

#define LED1_ON					(GPIO->DATA_RESET.LED1 = 1)
#define LED1_OFF				(GPIO->DATA_SET.LED1 = 1)
#define LED2_ON					(GPIO->DATA_RESET.LED2 = 1)
#define LED2_OFF				(GPIO->DATA_SET.LED2 = 1)
#define LED3_ON					(GPIO->DATA_RESET.LED3 = 1)
#define LED3_OFF				(GPIO->DATA_SET.LED3 = 1)

#define LED_BLUE				43
#define LED_GREEN				42
#define LED_RED					42

/*******************************************************************************
Expansion bus

Some of the lines on the expansion bus have been set elsewhere, but there are some
remaining:

SSI   	(TX/RX)													- defaults to GPIO pins
SPI   	(SCK/MOSI/MISO/SS/BITCK/FSYNC)	- defaults to GPIO pins
TMR			(2 - 3)													- defaults to GPIO pins
UART2 	(RTS/CTS/RX/TX) 								- defaults to GPIO pins
ADC			(0-3)														- defaults to GPIO pins

*******************************************************************************/

/*******************************************************************************
Serial ID line (DS2411X)
This is a Maxim bidirectional 1-wire interface. Requires a pull-up.
	SER_ID = TX_ON	(GPIO44)
***
	Initially, 
		GPIO44 = i/p with WPU set
*******************************************************************************/
#define SER_ID_BI					GPIO_44
#define SER_ID_OUT_LOW		(GPIO->DATA_RESET.SER_ID_BI = 1)
#define SER_ID_OUT_HIGH		(GPIO->DATA_SET.SER_ID_BI = 1)
#define SER_ID_READ				(GPIO->DATA.SER_ID_BI)
#define SER_ID_INPUT			(GPIO->PAD_DIR.SER_ID_BI = 0)
#define SER_ID_OUTPUT			(GPIO->PAD_DIR.SER_ID_BI = 1)


/*******************************************************************************
GPS device (AMY-6M)
		GPS_TIMEPULSE = MDO02 (GPIO53)
		GPS_PIO7      = MDO01 (GPIO52) - Used to wake GPS from sleep
		GPS_PIO17     = MDO00 (GPIO51) - Just a general GPIO line

***
	Initially, 
		GPIO53 = i/p
		GPIO52 = o/p
		GPIO51 = i/p
*******************************************************************************/
#define GPS_TIMEPULSE							GPIO_53
#define GPS_WAKEUP								GPIO_52
#define GPS_PIO17									GPIO_51
#define GPS_TIMEPULSE_READ				(GPIO->DATA.GPS_TIMEPULSE)
#define GPS_WAKEUP_LOW						(GPIO->DATA_RESET.GPS_WAKEUP = 1)
#define GPS_WAKEUP_HIGH						(GPIO->DATA_SET.GPS_WAKEUP = 1)
#define GPS_PIO17_READ						(GPIO->DATA.GPS_PIO17)


/* XTAL TUNE parameters */
/* see http://devl.org/pipermail/mc1322x/2009-December/000162.html */
/* for details about how to make this measurment */

/* Prime also needs an addtional 12pf on board */
/* Coarse tune: add 4pf */
#define CTUNE_4PF 1
/* Coarse tune: add 0-15 pf */
#define CTUNE 14
/* Fine tune: add FTUNE * 156fF (FTUNE is 4bits) */
#define FTUNE 0



// Node specific
#define BOA_MACPANID			0xAAAA
#define BASE_MAC16ADDR		0x0011
#define NODE_NO						0x01
#define NODE_MAC16ADDR		0x1111

/* Baud rate */
/*  230400 bps, INC=767, MOD=9999, 24Mhz 16x samp */
/*  115200 bps, INC=767, MOD=9999, 24Mhz 8x samp */

// Used in tests
#define MOD 		9999
#define INC 		767
#define SAMP		UCON_SAMP_8X

// Used in setting up the node
#define NODE_MOD	9999
#define NODE_INC	767
#define NODE_SAMP	UCON_SAMP_8X

// Node settings
#define NVM_DATA_ADDRESS	0x1E000


// Settings for ds2411 one wire protocol
//
/* Someone left MSP430-specific code in the ds2411 driver. */
#define splhigh() ( 0 )
#define splx(s) {}

/* Initialisation of pin is done in contiki-spectrum-main, so this is empty */
#define PIN_INIT()

/* Set 1-Wire low or high. */
#define OUTP_0() { SER_ID_OUT_LOW; }		// Claim bus by pulling low.
#define OUTP_1() { SER_ID_OUT_HIGH; }	  // Drive bus high rather than using WP, simply to get sharper edges on the pulse

/* Set pin as input or output */
#define DS2411_INPUT()  { SER_ID_INPUT; }
#define DS2411_OUTPUT() { SER_ID_OUTPUT; }

/* Read one bit. */
#define INP()    ( SER_ID_READ )

/*
 * Delay for u microseconds
 * There is a special case for 6 us for the TMOTE, because the overhead of calling
 * the function dominates the delay, at the slower clock speed, but we use the same
 * mechanism for now.
 *
 * As measured, a call to clock_delay_usec seems to give us a delay that is
 * a pretty constant 5us too long.
 */
#define udelay(u)  { clock_delay_usec(u-5); }
#define udelay_6() udelay(6)


#include <std_conf.h>

#endif
