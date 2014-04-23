/*
 * Copyright (c) 2011, Graeme McPhillips <g.mcphillips@cs.ucl.ac.uk>
 *
 * Test code for mc1322x Bracelet functions
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

#ifndef CO_ADS1015_H_
#define CO_ADS1015_H_

//#include <stdbool.h>

// Added by Jagun
extern const struct sensors_sensor co_sensor;

#define CO_ADS1015_I2C_ADDR				((0x90)>>1)
#define CO_ADS1015_CONV_REG				(0x00)			//
#define CO_ADS1015_CONF_REG				(0x01)			//
#define CO_ADS1015_LO_THRESH_REG			(0x02)			//
#define CO_ADS1015_HI_THRESH_REG			(0x03)			//

#define CO_ADS1015_AIN0					((0x04)<<4)		// AINP = AIN0 and AINN = GND
#define CO_ADS1015_AIN1					((0x05)<<4)		// AINP = AIN1 and AINN = GND
#define CO_ADS1015_AIN2					((0x06)<<4)		// AINP = AIN2 and AINN = GND
#define CO_ADS1015_AIN3					((0x07)<<4)		// AINP = AIN3 and AINN = GND
#define CO_ADS1015_RANGE_MASK				(0x8F)			// This is a negative mask, i.e. the zero's represent the position of the AIN values
#define CO_ADS1015_GAIN0					((0x00)<<1)		// FS=+-6.144V
#define CO_ADS1015_GAIN1					((0x01)<<1)		// FS=+-4.096V
#define CO_ADS1015_GAIN2					((0x02)<<1)		// FS=+-2.048V
#define CO_ADS1015_GAIN3					((0x03)<<1)		// FS=+-1.024V
#define CO_ADS1015_GAIN_MASK				(0xF1)			// This is a negative mask, i.e. the zero's represent the position of the AIN values
#define CO_ADS1015_START_SINGLE_CONV		(0x80)			// when in power-down mode


struct co_ads1015_data {
	int16_t adc0 ;	//decided to make it all possible by adding numbers.
	int16_t adc1 ;
	int16_t adc2 ;
	int16_t adc3 ;
} ;

void co_ads1015_init(uint8_t,uint8_t);
void co_ads1015_get_data(uint8_t co_ads1015_ainx, uint8_t *buffer);
void co_ads1015_decode_data(struct co_ads1015_data *const ba, uint8_t *buffer);
void co_ads1015_start_adc(void);
uint8_t co_ads1015_check_dataready(void);
#endif /* CO_ADS1015_H_ */
