/*
 * max5479.c
 *
 *  Created on: 1 Sep 2011
 *      Author: mcphillips
 */

#include <stdio.h>
#include "i2c.h"
//#include "../tests/include/software_defines.h"
#include "MAX5479.h"

void max5479_load_wiper_a(uint8_t res) {
	uint8_t set[] = {0,0} ;

	set[0] = MAX5479_WIPER_A_VREG ;
	set[1] =  res;

	i2c_transmitinit( MAX5479_I2C_ADDR, 2, set ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;
}

void max5479_load_wiper_b(uint8_t res) {
	uint8_t set[] = {0,0} ;

	set[0] = MAX5479_WIPER_B_VREG ;
	set[1] =  res;

	i2c_transmitinit( MAX5479_I2C_ADDR, 2, set ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;
}

void max5479_load_wiper_ab(uint8_t res) {
	uint8_t set[] = {0,0} ;

	set[0] = MAX5479_WIPER_AB_VREG ;
	set[1] =  res;

	i2c_transmitinit( MAX5479_I2C_ADDR, 2, set ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;
}

void max5479_set_wiper_a_NVREG(uint8_t res) {
	uint8_t set[] = {0,0} ;

	set[0] = MAX5479_WIPER_A_NVREG ;
	set[1] =  res;

	i2c_transmitinit( MAX5479_I2C_ADDR, 2, set ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;
}

void max5479_set_wiper_b_NVREG(uint8_t res) {
	uint8_t set[] = {0,0} ;

	set[0] = MAX5479_WIPER_B_NVREG ;
	set[1] =  res;

	i2c_transmitinit( MAX5479_I2C_ADDR, 2, set ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;
}

void max5479_set_wiper_ab(uint8_t res) {
	uint8_t set[] = {0,0} ;

	set[0] = MAX5479_WIPER_AB_NVREG ;
	set[1] =  res;

	i2c_transmitinit( MAX5479_I2C_ADDR, 2, set ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;
}
