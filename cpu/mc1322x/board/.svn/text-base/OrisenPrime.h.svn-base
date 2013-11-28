/*
 * Copyright (c) 2011, Graeme McPhillips <g.mcphillips@cs.ucl.ac.uk>
 *
 * Test code for mc1322x Prime functions
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

#ifndef BOARD_Prime_H
#define BOARD_Prime_H

/* Prime Bus IO lines*/

#define IO_0_OUT			GPIO_23
#define IO_0_IN				GPIO_27
#define IO_0_OUT_LOW		(GPIO->DATA_RESET.IO_0_OUT = 1)
#define IO_0_OUT_HIGH		(GPIO->DATA_SET.IO_0_OUT = 1)
#define IO_0_READ			(GPIO->DATA.IO_0_IN)

#define IO_1_OUT			GPIO_22
#define IO_1_IN				GPIO_26
#define IO_1_OUT_LOW		(GPIO->DATA_RESET.IO_1_OUT = 1)
#define IO_1_OUT_HIGH		(GPIO->DATA_SET.IO_1_OUT = 1)
#define IO_1_READ			(GPIO->DATA.IO_1_IN)

#define IO_2_OUT			GPIO_24
#define IO_2_IN				GPIO_28
#define IO_2_OUT_LOW		(GPIO->DATA_RESET.IO_2_OUT = 1)
#define IO_2_OUT_HIGH		(GPIO->DATA_SET.IO_2_OUT = 1)
#define IO_2_READ			(GPIO->DATA.IO_2_IN)

#define IO_3_OUT			GPIO_25
#define IO_3_IN				GPIO_29
#define IO_3_OUT_LOW		(GPIO->DATA_RESET.IO_3_OUT = 1)
#define IO_3_OUT_HIGH		(GPIO->DATA_SET.IO_3_OUT = 1)
#define IO_3_READ			(GPIO->DATA.IO_3_IN)

/* Prime Push Button*/
#define BUTTON1		GPIO_28
#define BUTTON2		GPIO_29

/* Prime SD Card present switch*/
#define SD_CARD_SWITCH			GPIO_55
#define SD_CARD_PRESENT		(!GPIO->DATA.SD_CARD_SWITCH)

/* Port 1 Enable */
#define PORT1_ENABLE_PIN		GPIO_58
#define PORT1_ENABLE		(GPIO->DATA_SET.PORT1_ENABLE_PIN = 1)
#define PORT1_DISABLE		(GPIO->DATA_RESET.PORT1_ENABLE_PIN = 1)

/* Port 1 GPIO */
#define PORT1_IO_1			GPIO_08
#define PORT1_IO_1_READ		(GPIO->DATA.PORT1_IO_1)
#define PORT1_IO_1_LOW		(GPIO->DATA_RESET.PORT1_IO_1 = 1)
#define PORT1_IO_1_HIGH		(GPIO->DATA_SET.PORT1_IO_1 = 1)

#define PORT1_IO_2			GPIO_09
#define PORT1_IO_2_READ		(GPIO->DATA.PORT1_IO_2)
#define PORT1_IO_2_LOW		(GPIO->DATA_RESET.PORT1_IO_2 = 1)
#define PORT1_IO_2_HIGH		(GPIO->DATA_SET.PORT1_IO_2 = 1)

#define PORT1_IO_3			GPIO_18
#define PORT1_IO_3_READ		(GPIO->DATA.PORT1_IO_3)
#define PORT1_IO_3_LOW		(GPIO->DATA_RESET.PORT1_IO_3 = 1)
#define PORT1_IO_3_HIGH		(GPIO->DATA_SET.PORT1_IO_3 = 1)

#define PORT1_IO_4			GPIO_19
#define PORT1_IO_4_READ		(GPIO->DATA.PORT1_IO_4)
#define PORT1_IO_4_LOW		(GPIO->DATA_RESET.PORT1_IO_4 = 1)
#define PORT1_IO_4_HIGH		(GPIO->DATA_SET.PORT1_IO_4 = 1)

#define PORT1_IO_5			GPIO_20
#define PORT1_IO_5_READ		(GPIO->DATA.PORT1_IO_5)
#define PORT1_IO_5_LOW		(GPIO->DATA_RESET.PORT1_IO_5 = 1)
#define PORT1_IO_5_HIGH		(GPIO->DATA_SET.PORT1_IO_5 = 1)

#define PORT1_IO_6			GPIO_21
#define PORT1_IO_6_READ		(GPIO->DATA.PORT1_IO_6)
#define PORT1_IO_6_LOW		(GPIO->DATA_RESET.PORT1_IO_6 = 1)
#define PORT1_IO_6_HIGH		(GPIO->DATA_SET.PORT1_IO_6 = 1)

/* These are ADC1 and ADC2 */
/* Not for using as IO */
/*
#define PORT1_IO_7			GPIO_34
#define PORT1_IO_7_READ		(GPIO->DATA.PORT1_IO_7)
#define PORT1_IO_7_LOW		(GPIO->DATA_RESET.PORT1_IO_7 = 1)
#define PORT1_IO_7_HIGH		(GPIO->DATA_SET.PORT1_IO_7 = 1)

#define PORT1_IO_8			GPIO_35
#define PORT1_IO_8_READ		(GPIO->DATA.PORT1_IO_8)
#define PORT1_IO_8_LOW		(GPIO->DATA_RESET.PORT1_IO_8 = 1)
#define PORT1_IO_8_HIGH		(GPIO->DATA_SET.PORT1_IO_8 = 1)
*/

/* Port 2 Enable */
#define PORT2_ENABLE_PIN		GPIO_56
#define PORT2_ENABLE		(GPIO->DATA_SET.PORT2_ENABLE_PIN = 1)
#define PORT2_DISABLE		(GPIO->DATA_RESET.PORT2_ENABLE_PIN = 1)

/* Port 2 GPIO */
#define PORT2_IO_1			GPIO_01
#define PORT2_IO_1_READ		(GPIO->DATA.PORT2_IO_1)
#define PORT2_IO_1_LOW		(GPIO->DATA_RESET.PORT2_IO_1 = 1)
#define PORT2_IO_1_HIGH		(GPIO->DATA_SET.PORT2_IO_1 = 1)

#define PORT2_IO_2			GPIO_00
#define PORT2_IO_2_READ		(GPIO->DATA.PORT2_IO_2)
#define PORT2_IO_2_LOW		(GPIO->DATA_RESET.PORT2_IO_2 = 1)
#define PORT2_IO_2_HIGH		(GPIO->DATA_SET.PORT2_IO_2 = 1)

#define PORT2_IO_3			GPIO_02
#define PORT2_IO_3_READ		(GPIO->DATA.PORT2_IO_3)
#define PORT2_IO_3_LOW		(GPIO->DATA_RESET.PORT2_IO_3 = 1)
#define PORT2_IO_3_HIGH		(GPIO->DATA_SET.PORT2_IO_3 = 1)

#define PORT2_IO_4			GPIO_03
#define PORT2_IO_4_READ		(GPIO->DATA.PORT2_IO_4)
#define PORT2_IO_4_LOW		(GPIO->DATA_RESET.PORT2_IO_4 = 1)
#define PORT2_IO_4_HIGH		(GPIO->DATA_SET.PORT2_IO_4 = 1)

#define PORT2_IO_5			GPIO_05
#define PORT2_IO_5_READ		(GPIO->DATA.PORT2_IO_5)
#define PORT2_IO_5_LOW		(GPIO->DATA_RESET.PORT2_IO_5 = 1)
#define PORT2_IO_5_HIGH		(GPIO->DATA_SET.PORT2_IO_5 = 1)

#define PORT2_IO_6			GPIO_07
#define PORT2_IO_6_READ		(GPIO->DATA.PORT2_IO_6)
#define PORT2_IO_6_LOW		(GPIO->DATA_RESET.PORT2_IO_6 = 1)
#define PORT2_IO_6_HIGH		(GPIO->DATA_SET.PORT2_IO_6 = 1)

#define PORT2_IO_7			GPIO_06
#define PORT2_IO_7_READ		(GPIO->DATA.PORT2_IO_7)
#define PORT2_IO_7_LOW		(GPIO->DATA_RESET.PORT2_IO_7 = 1)
#define PORT2_IO_7_HIGH		(GPIO->DATA_SET.PORT2_IO_7 = 1)

#define PORT2_IO_8			GPIO_04
#define PORT2_IO_8_READ		(GPIO->DATA.PORT2_IO_8)
#define PORT2_IO_8_LOW		(GPIO->DATA_RESET.PORT2_IO_8 = 1)
#define PORT2_IO_8_HIGH		(GPIO->DATA_SET.PORT2_IO_8 = 1)

/* Gyro Data Enable*/
#define DEN_G_PIN			GPIO_57
#define GYRO_DATA_ENABLE	(GPIO->DATA_SET.DEN_G_PIN = 1)
#define GYRO_DATA_DISABLE	(GPIO->DATA_RESET.DEN_G_PIN = 1)

/* Prime LEDs */
// There are no RED or YELLOW leds on the board,
// so leave these undefined
//
#define GPIO_LED_BLUE  	GPIO_45
#define GPIO_LED_GREEN  GPIO_44

#define LED1						GPIO_LED_BLUE
#define LED2						GPIO_LED_GREEN

#define LED1_ON					(GPIO->DATA_RESET.LED1 = 1)
#define LED1_OFF				(GPIO->DATA_SET.LED1 = 1)
#define LED2_ON					(GPIO->DATA_RESET.LED2 = 1)
#define LED2_OFF				(GPIO->DATA_SET.LED2 = 1)

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
#define BOA_MACPANID		0xAAAA
#define BASE_MAC16ADDR		0x0011
#define NODE_NO			0x01
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

#include <std_conf.h>

#endif
