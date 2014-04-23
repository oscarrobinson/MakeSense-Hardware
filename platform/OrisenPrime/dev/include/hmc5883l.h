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

#ifndef _HMC5883L_H_
#define _HMC5883L_H_

//#include <stdbool.h>

// Added by Jagun Kwon (j.kwon@cs.ucl.ac.uk)
extern const struct sensors_sensor hmc5883l_sensor;
// Sensor value types to be returned from value function
#define HMC5883L_LATCH_DATA	0	// This must be invoked first
#define HMC5883L_MAG_X_RAW	1
#define HMC5883L_MAG_Y_RAW	2
#define HMC5883L_MAG_Z_RAW	3

#define RAW_HMC5883L_DATA 0

#define HMC5883L_I2C_ADDR           ((0x3C)>>1)

#define HMC5883L_RA_CONFIG_A        0x00
#define HMC5883L_RA_CONFIG_B        0x01
#define HMC5883L_RA_MODE            0x02
#define HMC5883L_RA_DATAX_H         0x03
#define HMC5883L_RA_DATAX_L         0x04
#define HMC5883L_RA_DATAY_H         0x05
#define HMC5883L_RA_DATAY_L         0x06
#define HMC5883L_RA_DATAZ_H         0x07
#define HMC5883L_RA_DATAZ_L         0x08
#define HMC5883L_RA_STATUS          0x09
#define HMC5883L_RA_ID_A            0x0A
#define HMC5883L_RA_ID_B            0x0B
#define HMC5883L_RA_ID_C            0x0C

#define HMC5883L_AVERAGING_1        ((0x00)<<5)
#define HMC5883L_AVERAGING_2        ((0x01)<<5)
#define HMC5883L_AVERAGING_4        ((0x02)<<5)
#define HMC5883L_AVERAGING_8        ((0x03)<<5)

#define HMC5883L_RATE_0P75          ((0x00)<<2)
#define HMC5883L_RATE_1P5           ((0x01)<<2)
#define HMC5883L_RATE_3             ((0x02)<<2)
#define HMC5883L_RATE_7P5           ((0x03)<<2)
#define HMC5883L_RATE_15            ((0x04)<<2)
#define HMC5883L_RATE_30            ((0x05)<<2)
#define HMC5883L_RATE_75            ((0x06)<<2)

#define HMC5883L_BIAS_NORMAL        0x00
#define HMC5883L_BIAS_POSITIVE      0x01
#define HMC5883L_BIAS_NEGATIVE      0x02

#define HMC5883L_GAIN_1370          ((0x00)<<5)
#define HMC5883L_GAIN_1090          ((0x01)<<5)
#define HMC5883L_GAIN_820           ((0x02)<<5)
#define HMC5883L_GAIN_660           ((0x03)<<5)
#define HMC5883L_GAIN_440           ((0x04)<<5)
#define HMC5883L_GAIN_390           ((0x05)<<5)
#define HMC5883L_GAIN_330           ((0x06)<<5)
#define HMC5883L_GAIN_220           ((0x07)<<5)

#define HMC5883L_MODEREG_BIT        1
#define HMC5883L_MODEREG_LENGTH     2

#define HMC5883L_MODE_CONTINUOUS    0x00
#define HMC5883L_MODE_SINGLE        0x01
#define HMC5883L_MODE_IDLE          0x02

#define HMC5883L_STATUS_LOCK_BIT    1
#define HMC5883L_STATUS_READY_BIT   0

struct hmc5883l_data {
	int16_t mag_x ;
	int16_t mag_y ;
	int16_t mag_z ;
} ;

// typedef struct hmc5883l_data hmc5883l_d

void hmc5883l_init(void);
void hmc5883l_start(void);
void hmc5883l_stop(void);
void hmc5883l_get_data(uint8_t *buffer);
void hmc5883l_decode(struct hmc5883l_data *const mag_data , uint8_t *buffer);

#endif /* HMC5883L_H_ */
