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

#ifndef SHT25_H_
#define SHT25_H_

//#include <stdbool.h>

#define SHT25_I2C_ADDR				((0x80)>>1)
#define SHT25_TRIGGER_T_HOLD		(0xE3)			//
#define SHT25_TRIGGER_RH_HOLD		(0xE5)			//
#define SHT25_TRIGGER_T				(0xF3)			//
#define SHT25_TRIGGER_RH			(0xF5)			//
#define SHT25_WRITE_USER_REG		(0xE6)			//
#define SHT25_READ_USER_REG			(0xE7)			//
#define SHT25_SOFT_RESET			(0xFE)			//
#define SHT25_STAT					(0x02)			// ‘0’: temperature, ‘1’ humidity

struct sht25_data {
	int16_t humidity ;
	int16_t temp ;
} ;

void sht25_init(void);
void sht25_soft_reset(void);
void sht25_trigger_t(void);
void sht25_trigger_rh(void);
void sht25_get_data(uint8_t *buffer);
void sht25_decode(struct sht25_data *const hum_data , uint8_t *buffer);


#endif /* SHT25_H_ */
