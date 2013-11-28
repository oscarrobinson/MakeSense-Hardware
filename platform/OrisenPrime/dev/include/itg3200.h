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

#ifndef ITG3200_H_
#define ITG3200_H_

//#include <stdbool.h>

#define RAW_ITG3200_DATA 1

#define ITG3200_I2C_ADDR				((0xD0)>>1)
//#define ITG3200_I2C_ADDR				((0xD2)<<1)

#define ITG3200_WHO_AM_I					(0x00)
#define ITG3200_SMPLRT_DIV					(0x15)
#define ITG3200_DLPF_FS						(0x16)
#define ITG3200_INT_CFG						(0x17)
#define ITG3200_INT_STATUS					(0x1A)
#define ITG3200_TEMP_OUT					(0x1B)
#define ITG3200_GYRO_XOUT					(0x1D)
#define ITG3200_GYRO_YOUT					(0x1F)
#define ITG3200_GYRO_ZOUT					(0x21)
#define ITG3200_PWR_MGM						(0x3E)

#define ITG3200_DLPFFS_FS_SEL				(0x18)
#define ITG3200_DLPFFS_DLPF_CFG				(0x07)
#define ITG3200_INTCFG_ACTL					(0x80)
#define ITG3200_INTCFG_OPEN					(0x40)
#define ITG3200_INTCFG_LATCH_INT_EN			(0x20)
#define ITG3200_INTCFG_INT_ANYRD_2CLEAR		(0x10)
#define ITG3200_INTCFG_ITG_RDY_EN			(0x04)
#define ITG3200_INTCFG_RAW_RDY_EN			(0x01)
#define ITG3200_INTSTATUS_ITG_RDY			(0x04)
#define ITG3200_INTSTATUS_RAW_DATA_RDY		(0x01)
#define ITG3200_PWRMGM_HRESET				(0x80)
#define ITG3200_PWRMGM_SLEEP				(0x40)
#define ITG3200_PWRMGM_STBY_XG				(0x20)
#define ITG3200_PWRMGM_STBY_YG				(0x10)
#define ITG3200_PWRMGM_STBY_ZG				(0x08)
#define ITG3200_PWRMGM_CLK_SEL				(0x07)

#define ITG3200_NOSRDIVIDER         0
#define ITG3200_RANGE2000           3
#define ITG3200_BW256_SR8           0
#define ITG3200_BW188_SR1           1
#define ITG3200_BW098_SR1           2
#define ITG3200_BW042_SR1           3
#define ITG3200_BW020_SR1           4
#define ITG3200_BW010_SR1           5
#define ITG3200_BW005_SR1           6
#define ITG3200_ACTIVE_ONHIGH       0
#define ITG3200_ACTIVE_ONLOW        1
#define ITG3200_PUSH_PULL           0
#define ITG3200_OPEN_DRAIN          1
#define ITG3200_PULSE_50US          0
#define ITG3200_UNTIL_INT_CLEARED   1
#define ITG3200_READ_STATUSREG      0
#define ITG3200_READ_ANYREG         1
#define ITG3200_NORMAL              0
#define ITG3200_STANDBY             1
#define ITG3200_INTERNALOSC         0
#define ITG3200_PLL_XGYRO_REF       1
#define ITG3200_PLL_YGYRO_REF       2
#define ITG3200_PLL_ZGYRO_REF       3
#define ITG3200_PLL_EXTERNAL32      4
#define ITG3200_PLL_EXTERNAL19      5

struct itg3200_data {
	int16_t temp ;
	int16_t gyro_x ;
	int16_t gyro_y ;
	int16_t gyro_z ;
} ;


void itg3200_init(void);
void itg3200_stop(void);
void itg3200_start(void);
void itg3200_get_data(uint8_t *buffer);
void itg3200_decode(struct itg3200_data *const ba , uint8_t *buffer);


// Added by Jagun Kwon (j.kwon@cs.ucl.ac.uk)
extern const struct sensors_sensor itg3200_sensor;
// Sensor value types to be returned from value function
#define ITG3200_GET_DATA	0   // This must be invoked first
#define ITG3200_GYRO_X		1
#define ITG3200_GYRO_Y		2
#define ITG3200_GYRO_Z		3
#define ITG3200_TEMP		4


#endif /* ITG3200_H_ */
