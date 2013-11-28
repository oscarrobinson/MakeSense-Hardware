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

#ifndef BMA180_H_
#define BMA180_H_

// Added by Jagun Kwon (j.kwon@cs.ucl.ac.uk)
extern const struct sensors_sensor bma180_sensor;
// Sensor value types to be returned from value function
#define BMA180_ACC_X		1
#define BMA180_ACC_Y		2
#define BMA180_ACC_Z		3
#define BMA180_TEMPERATURE 	4	// BMA180_TEMP already exists


//#include <stdbool.h>

// Jagun: This is the correct address? ((0x80>>1)
//#define BMA180_I2C_ADDR					((0x80)>>1)
#define BMA180_I2C_ADDR					((0x82)>>1)
#define BMA180_RANGE					((0x04)<<1)		// +/- 4G Full scale range
#define BMA180_RANGE_MASK				(0xF1)			// This is a negative mask, i.e. the zero's represent the position of the RANGE values
#define BMA180_MODE						((0x00))		// Low noise mode
#define BMA180_MODE_MASK				(0xFC)			// This is a negative mask, i.e. the zero's represent the position of the MODE values
#define BMA180_BW						((0x00)<<4)		// 1200Hz Bandwidth
#define BMA180_BW_MASK					(0x0F)			// This is a negative mask, i.e. the zero's represent the position of the BW values
#define BMA180_EE_W						((0x01)<<4)		// The write enable bit
#define BMA180_EE_W_MASK				(0xEF)			// This is a negative mask, i.e. the zero represents the position of the EE_W value
#define BMA180_SLEEP					((0x01)<<1)		// The sleep enable bit
#define BMA180_SMP_SKIP					(0x01)			// The sample skip enable bit
#define BMA180_LAT_INT					(0x01)			// The latch int enable bit
#define BMA180_NEW_DATA_INT				((0x01)<<1)		// The new data int enable bit
#define BMA180_TAPSENS_INT				((0x01)<<3)		// The tapsens int enable bit
#define BMA180_TAPSENS_FILT				(0x01)			// The filter enable bit for tapsens
#define BMA180_TAPSENS_INT_X			((0x01)<<3)		// The tapsens int enable bit
#define BMA180_RESET_INT				((0x01)<<6)		// The new data int enable bit
#define BMA180_TAPSENS_DUR				(0x07)			// The tapsens duration


#define BMA180_CHIP_ID					(0x00)
#define BMA180_VERSION					(0x01)
#define BMA180_ACC_X_LSB				(0x02)
#define BMA180_ACC_X_MSB				(0x03)
#define BMA180_ACC_Y_LSB				(0x04)
#define BMA180_ACC_Y_MSB				(0x05)
#define BMA180_ACC_Z_LSB				(0x06)
#define BMA180_ACC_Z_MSB				(0x07)
#define BMA180_TEMP						(0x08)
#define BMA180_STATUS_REG1				(0x09)
#define BMA180_STATUS_REG2				(0x0A)
#define BMA180_STATUS_REG3				(0x0B)
#define BMA180_STATUS_REG4				(0x0C)
#define BMA180_CTRL_REG0				(0x0D)
#define BMA180_CTRL_REG1				(0x0E)
#define BMA180_CTRL_REG2				(0x0F)
#define BMA180_RESET					(0x10)
#define BMA180_BW_TCS					(0x20)
#define BMA180_CTRL_REG3				(0x21)
#define BMA180_CTRL_REG4				(0x22)
#define BMA180_HY						(0x23)
#define BMA180_SLOPE_TAPSENS_INFO		(0x24)
#define BMA180_HIGH_LOW_INFO			(0x25)
#define BMA180_LOW_DUR					(0x26)
#define BMA180_HIGH_DUR					(0x27)
#define BMA180_TAPSENS_TH				(0x28)
#define BMA180_LOW_TH					(0x29)
#define BMA180_HIGH_TH					(0x2A)
#define BMA180_SLOPE_TH					(0x2B)
#define BMA180_CD1_CUSTOMER_DATA		(0x2C)
#define BMA180_CD2_CUSTOMER_DATA		(0x2D)
#define BMA180_TCO_X					(0x2E)
#define BMA180_TCO_Y					(0x2F)
#define BMA180_TCO_Z					(0x30)
#define BMA180_GAIN_T					(0x31)
#define BMA180_GAIN_X					(0x32)
#define BMA180_GAIN_Y					(0x33)
#define BMA180_GAIN_Z					(0x34)
#define BMA180_OFFSET_LSB1				(0x35)
#define BMA180_OFFSET_LSB2				(0x36)
#define BMA180_OFFSET_T					(0x37)
#define BMA180_OFFSET_X					(0x38)
#define BMA180_OFFSET_Y					(0x39)
#define BMA180_OFFSET_Z					(0x3A)
#define BMA180_EE_BW_TCS				(0x40)
#define BMA180_EE_CTRL_REG3				(0x41)
#define BMA180_EE_CTRL_REG4				(0x42)
#define BMA180_EE_HY					(0x43)
#define BMA180_EE_SLOPE_TAPSENS_INFO	(0x44)
#define BMA180_EE_HIGH_LOW_INFO			(0x45)
#define BMA180_EE_LOW_DUR				(0x46)
#define BMA180_EE_HIGH_DUR				(0x47)
#define BMA180_EE_TAPSENS_TH			(0x48)
#define BMA180_EE_LOW_TH				(0x49)
#define BMA180_EE_HIGH_TH				(0x4A)
#define BMA180_EE_SLOPE_TH				(0x4B)
#define BMA180_EE_CD1_CUSTOMER_DATA		(0x4C)
#define BMA180_EE_CD2_CUSTOMER_DATA		(0x4D)
#define BMA180_EE_TCO_X					(0x4E)
#define BMA180_EE_TCO_Y					(0x4F)
#define BMA180_EE_TCO_Z					(0x50)
#define BMA180_EE_GAIN_T				(0x51)
#define BMA180_EE_GAIN_X				(0x52)
#define BMA180_EE_GAIN_Y				(0x53)
#define BMA180_EE_GAIN_Z				(0x54)
#define BMA180_EE_OFFSET_LSB1			(0x55)
#define BMA180_EE_OFFSET_LSB2			(0x56)
#define BMA180_EE_OFFSET_T				(0x57)
#define BMA180_EE_OFFSET_X				(0x58)
#define BMA180_EE_OFFSET_Y				(0x59)
#define BMA180_EE_OFFSET_Z				(0x5A)

struct bma180_data {
	int16_t acc_x ;
	int16_t acc_y ;
	int16_t acc_z ;
	int8_t temp ;
} ;

void bma180_init(void);
void bma180_init_tapsens(void);
void bma180_set_int(uint8_t set_int);
void bma180_reset_int(void);
void bma180_start(void);
void bma180_stop(void);
void bma180_get_data(uint8_t *buffer);
void bma180_decode(struct bma180_data *const acc_data , uint8_t *buffer);
void bma180_get_acc_x(struct bma180_data *const acc_data);
void bma180_get_acc_y(struct bma180_data *const acc_data);
void bma180_get_acc_z(struct bma180_data *const acc_data);
void bma180_get_temp(struct bma180_data *const acc_data);

#endif /* BMA180_H_ */
