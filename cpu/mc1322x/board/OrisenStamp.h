/*
 * Copyright (c) 2011, Graeme McPhillips <g.mcphillips@cs.ucl.ac.uk>
 *
 * Test code for mc1322x Stamp functions
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

#ifndef BOARD_Stamp_H
#define BOARD_Stamp_H

/* Stamp Bus IO lines*/

//#define IO_0_OUT				GPIO_23
//#define IO_0_IN					GPIO_27
//#define IO_0_OUT_LOW		(GPIO->DATA_RESET.IO_0_OUT = 1)
//#define IO_0_OUT_HIGH		(GPIO->DATA_SET.IO_0_OUT = 1)
//#define IO_0_READ				(GPIO->DATA.IO_0_IN)

/* Stamp Push Button*/
#define BUTTON					GPIO_26

/* Stamp Real time clock */
#define RTC_IRQ					GPIO_27

/* Stamp SD Card present switch*/
#define SD_CARD_SWITCH	GPIO_44
#define SD_CARD_PRESENT	(!GPIO->DATA.SD_CARD_SWITCH)

/* Stamp LEDs */
// There is no RED led on the board, so leave it undefined
//
#define GPIO_LED_BLUE  	GPIO_45
#define GPIO_LED_GREEN  GPIO_43
#define GPIO_LED_YELLOW GPIO_42

#define LED1						GPIO_LED_BLUE
#define LED2						GPIO_LED_GREEN
#define LED3						GPIO_LED_YELLOW

#define LED1_ON					(GPIO->DATA_RESET.LED1 = 1)
#define LED1_OFF				(GPIO->DATA_SET.LED1 = 1)
#define LED2_ON					(GPIO->DATA_RESET.LED2 = 1)
#define LED2_OFF				(GPIO->DATA_SET.LED2 = 1)
#define LED3_ON					(GPIO->DATA_RESET.LED3 = 1)
#define LED3_OFF				(GPIO->DATA_SET.LED3 = 1)

// ADC reads are limited to ADC0 and ADC1 plus the
// battery reference voltage
//
#define ADC_CHANS_ENABLED_CONF 0x103

/* XTAL TUNE parameters */
/* see http://devl.org/pipermail/mc1322x/2009-December/000162.html */
/* for details about how to make this measurment */

/* Stamp also needs an addtional 12pf on board */
/* Coarse tune: add 4pf */
#define CTUNE_4PF 1
/* Coarse tune: add 0-15 pf */
#define CTUNE 14
/* Fine tune: add FTUNE * 156fF (FTUNE is 4bits) */
#define FTUNE 0


// Node specific
#define BOA_MACPANID		0xAAAA
#define BASE_MAC16ADDR	0x0011
#define NODE_NO					0x01
#define NODE_MAC16ADDR	0x1111

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
