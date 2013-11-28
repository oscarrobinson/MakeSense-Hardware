/*
 * bma180.c
 *
 *  Created on: 1 Sep 2011
 *      Author: mcphillips
 *
 *  Updated on: 29 Aug 2012 by Jagun Kwon (j.kwon@cs.ucl.ac.uk)
 *  Refactored to comply with Contiki sensor interface structure
 */


#include <stdio.h>
#include "i2c.h"
#include "include/bma180.h"
#include "lib/sensors.h"    // for Contiki sensor interface

//extern volatile uint8_t go_flag;
//extern volatile uint8_t timer_delay;

void bma180_init(void) {

	uint8_t receive = 0 ;
	uint8_t set[] = {0,0} ;

	uint8_t request = BMA180_CTRL_REG0 ;
	i2c_transmitinit( BMA180_I2C_ADDR, 1, &request ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;
	i2c_receiveinit( BMA180_I2C_ADDR, 1, &receive ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;
//	printf("CTRL_REG0 = %x\n",receive);

	set[0] = BMA180_CTRL_REG0 ;
	set[1] = ((receive & BMA180_EE_W_MASK) | BMA180_EE_W) ;
	i2c_transmitinit( BMA180_I2C_ADDR, 2, set ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;

//	request = BMA180_CTRL_REG0 ;
//	i2c_transmitinit( BMA180_I2C_ADDR, 1, &request ) ;
//	while(!i2c_transferred()) /* Wait for transfer */ ;
//	i2c_receiveinit( BMA180_I2C_ADDR, 1, &receive ) ;
//	while(!i2c_transferred()) /* Wait for transfer */ ;
//	printf("CTRL_REG0 = %x\n",receive);

	request = BMA180_BW_TCS ;
	i2c_transmitinit( BMA180_I2C_ADDR, 1, &request ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;
	i2c_receiveinit( BMA180_I2C_ADDR, 1, &receive ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;
//	printf("BW_TCS = %x\n",receive);

	set[0] = BMA180_BW_TCS ;
	set[1] = ((receive & BMA180_BW_MASK) | BMA180_BW) ;
	i2c_transmitinit( BMA180_I2C_ADDR, 2, set ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;

//	request = BMA180_BW_TCS ;
//	i2c_transmitinit( BMA180_I2C_ADDR, 1, &request ) ;
//	while(!i2c_transferred()) /* Wait for transfer */ ;
//	i2c_receiveinit( BMA180_I2C_ADDR, 1, &receive ) ;
//	while(!i2c_transferred()) /* Wait for transfer */ ;
//	printf("BW_TCS = %x\n",receive);


	request = BMA180_OFFSET_LSB1 ;
	i2c_transmitinit( BMA180_I2C_ADDR, 1, &request ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;
	i2c_receiveinit( BMA180_I2C_ADDR, 1, &receive ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;
//	printf("OFFSET_LSB1 = %x\n",receive);

	set[0] = BMA180_OFFSET_LSB1 ;
	set[1] = ((receive & BMA180_RANGE_MASK) | BMA180_RANGE | BMA180_SMP_SKIP) ;
	i2c_transmitinit( BMA180_I2C_ADDR, 2, set ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;

//	request = BMA180_OFFSET_LSB1 ;
//	i2c_transmitinit( BMA180_I2C_ADDR, 1, &request ) ;
//	while(!i2c_transferred()) /* Wait for transfer */ ;
//	i2c_receiveinit( BMA180_I2C_ADDR, 1, &receive ) ;
//	while(!i2c_transferred()) /* Wait for transfer */ ;
//	printf("OFFSET_LSB1 = %x\n",receive);

//	request = BMA180_TCO_Z ;
//	i2c_transmitinit( BMA180_I2C_ADDR, 1, &request ) ;
//	while(!i2c_transferred()) /* Wait for transfer */ ;
//	i2c_receiveinit( BMA180_I2C_ADDR, 1, &receive ) ;
//	while(!i2c_transferred()) /* Wait for transfer */ ;
//
//	set[0] = BMA180_TCO_Z;
//	set[1] = ((receive & BMA180_MODE_MASK) | BMA180_MODE) ;
//	i2c_transmitinit( BMA180_I2C_ADDR, 2, set ) ;
//	while(!i2c_transferred()) /* Wait for transfer */ ;

}

void bma180_init_tapsens(void){
	uint8_t receive = 0 ;
	uint8_t set[] = {0,0} ;

//set tapsens threshold
	set[0] = BMA180_TAPSENS_TH ;
	set[1] = 0xF0 ;
	i2c_transmitinit( BMA180_I2C_ADDR, 2, set ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;
//set tapsense int x and tapsense filter
	uint8_t request = BMA180_SLOPE_TAPSENS_INFO ;
	i2c_transmitinit( BMA180_I2C_ADDR, 1, &request ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;
	i2c_receiveinit( BMA180_I2C_ADDR, 1, &receive ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;
//	printf("CTRL_REG0 = %x\n",receive);

	set[0] = BMA180_SLOPE_TAPSENS_INFO ;
	set[1] = ((receive & 0xF0) | BMA180_TAPSENS_INT_X) ;
	i2c_transmitinit( BMA180_I2C_ADDR, 2, set ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;
// set tapsens duration
	request = BMA180_GAIN_T ;
	i2c_transmitinit( BMA180_I2C_ADDR, 1, &request ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;
	i2c_receiveinit( BMA180_I2C_ADDR, 1, &receive ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;
//	printf("CTRL_REG0 = %x\n",receive);

	set[0] = BMA180_GAIN_T ;
	set[1] = ((receive & 0xF8) | BMA180_TAPSENS_DUR) ;
	i2c_transmitinit( BMA180_I2C_ADDR, 2, set ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;
}

void bma180_set_int(uint8_t set_int){
	uint8_t receive = 0 ;
	uint8_t set[] = {0,0} ;

	uint8_t request = BMA180_CTRL_REG3 ;
	i2c_transmitinit( BMA180_I2C_ADDR, 1, &request ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;
	i2c_receiveinit( BMA180_I2C_ADDR, 1, &receive ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;
//	printf("CTRL_REG3 = %x\n",receive);

	set[0] = BMA180_CTRL_REG3;
	set[1] = set_int ;
//	set[1] = (0x02) ;
	i2c_transmitinit( BMA180_I2C_ADDR, 2, set ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;

	request = BMA180_CTRL_REG3 ;
	i2c_transmitinit( BMA180_I2C_ADDR, 1, &request ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;
	i2c_receiveinit( BMA180_I2C_ADDR, 1, &receive ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;
//	printf("CTRL_REG3 = %x\n",receive);
}


void bma180_reset_int(void){
	uint8_t receive = 0 ;
	uint8_t set[] = {0,0} ;

	uint8_t request = BMA180_CTRL_REG0 ;
	i2c_transmitinit( BMA180_I2C_ADDR, 1, &request ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;
	i2c_receiveinit( BMA180_I2C_ADDR, 1, &receive ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;

	set[0] = BMA180_CTRL_REG0 ;
	set[1] = (receive | BMA180_RESET_INT) ;
	i2c_transmitinit( BMA180_I2C_ADDR, 2, set ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;
}

void bma180_start(void) {
	uint8_t request = BMA180_CTRL_REG0 ;
	uint8_t receive = 0 ;
	uint8_t set[] = {0,0} ;

	i2c_transmitinit( BMA180_I2C_ADDR, 1, &request ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;

	i2c_receiveinit( BMA180_I2C_ADDR, 1, &receive ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;

	set[0] = BMA180_CTRL_REG0 ;
	set[1] = (receive & ~BMA180_SLEEP) ;

	i2c_transmitinit( BMA180_I2C_ADDR, 2, set ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;
}

void bma180_stop(void) {
	uint8_t request = BMA180_CTRL_REG0 ;
	uint8_t receive = 0 ;
	uint8_t set[] = {0,0} ;

	i2c_transmitinit( BMA180_I2C_ADDR, 1, &request ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;

	i2c_receiveinit( BMA180_I2C_ADDR, 1, &receive ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;

	set[0] = BMA180_CTRL_REG0 ;
	set[1] = (receive | BMA180_SLEEP) ;

	i2c_transmitinit( BMA180_I2C_ADDR, 2, set ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;
}


void bma180_get_data(uint8_t *buffer) {

	uint8_t request_MSB = BMA180_ACC_X_LSB ;

	i2c_transmitinit( BMA180_I2C_ADDR, 1, &request_MSB ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;

	i2c_receiveinit( BMA180_I2C_ADDR, 7, buffer ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;
}

void bma180_decode(struct bma180_data *const acc_data , uint8_t *buffer) {

	int16_t temp_val;
	temp_val = ((buffer[0]>>2) + (buffer[1]<<6));
	if ((temp_val & 0x2000) == 0x2000){
		temp_val = (((~(temp_val) & 0x3FFF)+1)*-1) ;
	}
	acc_data->acc_x = temp_val;
		temp_val = ((buffer[2]>>2) + (buffer[3]<<6));
	if ((temp_val & 0x2000) == 0x2000){
		temp_val = (((~(temp_val) & 0x3FFF)+1)*-1) ;
	}
	acc_data->acc_y = temp_val;
		temp_val = ((buffer[4]>>2) + (buffer[5]<<6));
	if ((temp_val & 0x2000) == 0x2000){
		temp_val = (((~(temp_val) & 0x3FFF)+1)*-1) ;
	}
	acc_data->acc_z = temp_val;

	acc_data->temp = buffer[6];
}

//void bma180_decode(struct bma180_data *const acc_data , uint8_t *buffer) {
//
//	acc_data->acc_x = ((buffer[0] && 0xFC) + (buffer[1]<<8)) / 4;
//	acc_data->acc_y = ((buffer[2] && 0xFC) + (buffer[3]<<8)) / 4;
//	acc_data->acc_z = ((buffer[4] && 0xFC) + (buffer[5]<<8)) / 4;
//	acc_data->temp = buffer[6];
//}


void bma180_get_acc_x(struct bma180_data *const acc_data) {

	int16_t temp_val;
	uint8_t request_MSB = BMA180_ACC_X_LSB ;
	uint8_t precv[] = {0,0} ;

	i2c_transmitinit( BMA180_I2C_ADDR, 1, &request_MSB ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;

	i2c_receiveinit( BMA180_I2C_ADDR, 2, precv ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;

	temp_val = ((precv[0]>>2) + (precv[1]<<6));

	if ((temp_val & 0x2000) == 0x2000){
		temp_val = (((~(temp_val) & 0x3FFF)+1)*-1) ;
	}

	acc_data->acc_x = temp_val;
}

void bma180_get_acc_y(struct bma180_data *const acc_data) {

	int16_t temp_val;
	uint8_t request_MSB = BMA180_ACC_Y_LSB ;
	uint8_t precv[] = {0,0} ;

	i2c_transmitinit( BMA180_I2C_ADDR, 1, &request_MSB ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;

	i2c_receiveinit( BMA180_I2C_ADDR, 2, precv ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;

	temp_val = ((precv[0]>>2) + (precv[1]<<6));

	if ((temp_val & 0x2000) == 0x2000){
		temp_val = (((~(temp_val) & 0x3FFF)+1)*-1) ;
	}

	acc_data->acc_y = temp_val;
}

void bma180_get_acc_z(struct bma180_data *const acc_data) {

	int16_t temp_val;
	uint8_t request_MSB = BMA180_ACC_Z_LSB ;
	uint8_t precv[] = {0,0} ;

	i2c_transmitinit( BMA180_I2C_ADDR, 1, &request_MSB ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;

	i2c_receiveinit( BMA180_I2C_ADDR, 2, precv ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;

	temp_val = ((precv[0]>>2) + (precv[1]<<6));

	if ((temp_val & 0x2000) == 0x2000){
		temp_val = (((~(temp_val) & 0x3FFF)+1)*-1) ;
	}

	acc_data->acc_z = temp_val;
}


void bma180_get_temp(struct bma180_data *const acc_data) {

	uint8_t request_MSB = BMA180_TEMP ;
	uint8_t precv = 0;

	i2c_transmitinit( BMA180_I2C_ADDR, 1, &request_MSB ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;

	i2c_receiveinit( BMA180_I2C_ADDR, 1, &precv ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;

	acc_data->temp = precv;
}


// Jagun: 29 Aug 2012: added the following to comply with
// Contiki sensor interface structure
struct bma180_data __temp_acc_data;

static int
value(int type)
{
    switch(type) {
        case BMA180_ACC_X:
			bma180_get_acc_x(&__temp_acc_data);
            return __temp_acc_data.acc_x;

        case BMA180_ACC_Y:
			bma180_get_acc_y(&__temp_acc_data);
            return __temp_acc_data.acc_y;

        case BMA180_ACC_Z:
			bma180_get_acc_z(&__temp_acc_data);
            return __temp_acc_data.acc_z;

        case BMA180_TEMPERATURE:
			bma180_get_temp(&__temp_acc_data);
            return __temp_acc_data.temp;
    }
}

static int
status(int type)
{
    // TODO We don't seem to have an equivalent for this
    return 1;
}

static int
configure(int type, int c)
{
	bma180_init();
	//bma180_start();

    return 1;
}

// Instantiate the sensor 
SENSORS_SENSOR(bma180_sensor, "bma180-Sensor", value, configure, status);


