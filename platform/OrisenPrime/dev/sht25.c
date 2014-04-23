/*
 * sht25.c
 *
 *  Created on: 1 Sep 2011
 *      Author: mcphillips
 */

#include <stdio.h>
#include <math.h>
#include "i2c.h"
#include "include/sht25.h"

#define DEBUG 1
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

void sht25_init(void){
	uint8_t request = SHT25_READ_USER_REG;
	uint8_t read = 0;

	i2c_transmitinit( SHT25_I2C_ADDR, 1, &request ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;

	i2c_receiveinit( SHT25_I2C_ADDR, 1, &read ) ;
	PRINTF("SHT25 user reg = %x\n",read);
}

void sht25_soft_reset(void){
	uint8_t request = SHT25_SOFT_RESET ;

	i2c_transmitinit( SHT25_I2C_ADDR, 1, &request ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;
}

void sht25_trigger_t(void){
	uint8_t request = SHT25_TRIGGER_T ;

	i2c_transmitinit( SHT25_I2C_ADDR, 1, &request ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;
}
void sht25_trigger_rh(void){
	uint8_t request = SHT25_TRIGGER_RH ;

	i2c_transmitinit( SHT25_I2C_ADDR, 1, &request ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;
}
void sht25_get_data(uint8_t *buffer){
	uint8_t read[] = {0,0,0};

	i2c_receiveinit( SHT25_I2C_ADDR, 3, read ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;

	if ((read[1] & SHT25_STAT) == SHT25_STAT){
		buffer[8] = read[0];
		buffer[9] = read[1];
		buffer[10] = read[2];
	}
	else {
		buffer[5] = read[0];
		buffer[6] = read[1];
		buffer[7] = read[2];
	}
}

void sht25_decode(struct sht25_data *const hum_data , uint8_t *buffer) {
	hum_data->humidity = -6 + 125 / 65536.0 * (float)((buffer[9] & 0xFC) + (buffer[8]<<8));
	hum_data->temp = -46.85 + 175.72 / 65536.0 * (float)((buffer[6] & 0xFC) + (buffer[5]<<8));
}


