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

#ifndef MPU6000_H_
#define MPU6000_H_

#include "ssi.h"

#define MPU6000_WHOAMI				0x75
#define	MPU6000_SMPLRT_DIV			0x19
#define MPU6000_CONFIG				0x1A
#define MPU6000_GYRO_CONFIG			0x1B
#define MPU6000_ACC_CONFIG		0x1C
#define MPU6000_INT_PIN_CFG			0x37
#define	MPU6000_INT_ENABLE			0x38
#define MPU6000_ACC_XOUT_H		0x3B
#define MPU6000_ACC_XOUT_L		0x3C
#define MPU6000_ACC_YOUT_H		0x3D
#define MPU6000_ACC_YOUT_L		0x3E
#define MPU6000_ACC_ZOUT_H		0x3F
#define MPU6000_ACC_ZOUT_L		0x40
#define MPU6000_TEMP_OUT_H			0x41
#define MPU6000_TEMP_OUT_L			0x42
#define MPU6000_GYRO_XOUT_H			0x43
#define	MPU6000_GYRO_XOUT_L			0x44
#define MPU6000_GYRO_YOUT_H			0x45
#define	MPU6000_GYRO_YOUT_L			0x46
#define MPU6000_GYRO_ZOUT_H			0x47
#define	MPU6000_GYRO_ZOUT_L			0x48
#define MPU6000_USER_CTRL			0x6A
#define	MPU6000_PWR_MGMT_1			0x6B
#define	MPU6000_PWR_MGMT_2			0x6C

// Configuration bits  MPU 6000
#define MPU6000_SLEEP 					0x40	/* MPU6000_PWR_MGMT_1 */
#define MPU6000_DEVICE_RESET 			0x80	/* MPU6000_PWR_MGMT_1 */
#define	MPU6000_I2C_IF_DIS              0x10	/* MPU6000_USER_CTRL */

#define MPU6000_ENABLE		(GPIO->DATA_SET.IMU_EN = 1)
#define MPU6000_DISABLE		(GPIO->DATA_RESET.IMU_EN = 1)
//#define MPU6000_CS_ENABLE		(GPIO->DATA_RESET.SPI_SS = 1)
//#define MPU6000_CS_DISABLE		(GPIO->DATA_SET.SPI_SS = 1)


enum {
	MPU6000_EXT_SYNC_SET_IN_DISABLE = 0,
	MPU6000_EXT_SYNC_SET_TEMP,
	MPU6000_EXT_SYNC_SET_GYROX,
	MPU6000_EXT_SYNC_SET_GYROY,
	MPU6000_EXT_SYNC_SET_GYROZ,
	MPU6000_EXT_SYNC_SET_ACCX,
	MPU6000_EXT_SYNC_SET_ACCY,
	MPU6000_EXT_SYNC_SET_ACCZ
} ;

enum {
	MPU6000_DLPF_CFG_256HZ_NOLPF2 = 0,
	MPU6000_DLPF_CFG_188HZ,
	MPU6000_DLPF_CFG_98HZ,
	MPU6000_DLPF_CFG_42HZ,
	MPU6000_DLPF_CFG_20HZ,
	MPU6000_DLPF_CFG_10HZ,
	MPU6000_DLPF_CFG_5HZ,
	MPU6000_DLPF_CFG_2100HZ_NOLPF
} ;

enum {
	MPU6000_FS_SEL_250DPS = 0,
	MPU6000_FS_SEL_500DPS,
	MPU6000_FS_SEL_1000DPS,
	MPU6000_FS_SEL_2000DPS
} ;

enum {
	MPU6000_AFS_SEL_2G = 0,
	MPU6000_AFS_SEL_4G,
	MPU6000_AFS_SEL_8G,
	MPU6000_AFS_SEL_16G
} ;

enum {
	MPU6000_ACC_HPF_RESET = 0,
	MPU6000_ACC_HPF_5HZ,
	MPU6000_ACC_HPF_2HZ5,
	MPU6000_ACC_HPF_1HZ25,
	MPU6000_ACC_HPF_0HZ63,
	MPU6000_ACC_HPF_RES1,
	MPU6000_ACC_HPF_RES2,
	MPU6000_ACC_HPF_HOLD
} ;

enum {
	MPU6000_CLKSEL_8MHZ = 0,
	MPU6000_CLKSEL_PLL_GYROX,
	MPU6000_CLKSEL_PLL_GYROY,
	MPU6000_CLKSEL_PLL_GYROZ,
	MPU6000_CLKSEL_PLL_EXT32K,
	MPU6000_CLKSEL_PLL_EXT19M,
	MPU6000_CLKSEL_RES,
	MPU6000_CLKSEL_RESET
} ;

enum {
	MPU6000_LP_WAKE_CTRL_1HZ25 = 0,
	MPU6000_LP_WAKE_CTRL_2HZ5,
	MPU6000_LP_WAKE_CTRL_5HZ,
	MPU6000_LP_WAKE_CTRL_10HZ
} ;

struct mpu6000_data {
	int16_t acc_x ;
	int16_t acc_y ;
	int16_t acc_z ;
	int16_t temp ;
	int16_t gyro_x ;
	int16_t gyro_y ;
	int16_t gyro_z ;
} ;


void mpu6000_init(void);
//void mpu6000_stop(void);
//void mpu6000_start(void);
void mpu_read(void);
void mpu6000_spi_write(uint8_t reg, uint8_t data);
uint8_t mpu6000_spi_read(uint8_t reg);
//void mpu6000_get_data(uint8_t *buffer);
//void mpu6000_decode(struct mpu6000_data *const ba , uint8_t *buffer);




#endif /* MPU6000_H_ */
