/*
 * asd1015.c
 *
 *  Created on: 1 Sep 2011
 *      Author: mcphillips
 */

#include <stdio.h>
#include "i2c.h"
#include "include/ads1015.h"

extern volatile uint8_t timer_delay;
//volatile int16_t temp_adc;
uint8_t conf_set[] = {0,0};
uint8_t ain_set;

void ads1015_init(uint8_t ads1015_ainx, uint8_t ads1015_gainx) {

	uint8_t request = ADS1015_CONF_REG ;
	uint8_t receive[] = {0,0} ;
	uint8_t set[] = {0,0,0} ;

	i2c_transmitinit( ADS1015_I2C_ADDR, 1, &request ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;

	i2c_receiveinit( ADS1015_I2C_ADDR, 2, receive ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;

	set[0] = ADS1015_CONF_REG ;
	conf_set[0] = set[0];
	set[1] =  ((receive[0] & ADS1015_RANGE_MASK & ADS1015_GAIN_MASK) | ads1015_ainx | ads1015_gainx | ADS1015_START_SINGLE_CONV);
	set[1] =  0x82;
	conf_set[1] = set[1];
	ain_set = ads1015_ainx;
	set[2] =  receive[1];

	i2c_transmitinit( ADS1015_I2C_ADDR, 3, set ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;

}

void ads1015_start_adc(void) {

	i2c_transmitinit( ADS1015_I2C_ADDR, 2, conf_set ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;

}


void ads1015_get_data(struct ads1015_data *const ba) {

	int16_t temp_adc;
	uint8_t request_MSB = ADS1015_CONV_REG ;
	uint8_t precv[] = {0,0} ;

	i2c_transmitinit( ADS1015_I2C_ADDR, 1, &request_MSB ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;

	i2c_receiveinit( ADS1015_I2C_ADDR, 2, precv ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;

	temp_adc = ((precv[0]<<4) + (precv[1]>>4));
	if ((temp_adc & 0x2000) == 0x2000){
		temp_adc = (((~(temp_adc) & 0x3FFF)+1)*-1) ;
	}

	switch (ain_set){
	case ((0x04)<<4):
		ba->adc0 = temp_adc;
		break;
	case ((0x05)<<4):
		ba->adc1 = temp_adc;
		break;
	case ((0x06)<<4):
		ba->adc2 = temp_adc;
		break;
	case ((0x07)<<4):
		ba->adc3 = temp_adc;
		break;
	}
}



