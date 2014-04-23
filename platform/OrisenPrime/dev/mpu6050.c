/*
 * mpu6050.c
 *
 *  Created on: 6 Dec 2011
 *      Author: mcphillips
 *
 *  Updated on: 01 Sept 2012 by Jagun Kwon (j.kwon@cs.ucl.ac.uk)
 *  Refactored to comply with Contiki sensor interface structure
 *  mpu6050: 6-axis motion tracking device (InvenSense)
 */

#include <stdio.h>
#include "i2c.h"
#include "include/mpu6050.h"

#include "lib/sensors.h"

extern volatile uint8_t timer_delay;

void mpu6050_stop(void) {

//	uint8_t receive = 0 ;
	uint8_t set[] = {0,0} ;

	set[0] = MPU6050_PWR_MGMT_1 ;
	set[1] = 0x40;		// Just switching it off

	i2c_transmitinit( MPU6050_I2C_ADDR, 2, set ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;

}

void mpu6050_init(void) {

////	uint8_t receive = 0 ;
//	uint8_t set[] = {0,0} ;
//
//	set[0] = MPU6050_DLPF_FS ;
//	set[1] = (MPU6050_DLPFFS_FS_SEL | MPU6050_BW020_SR1);		// Setting 10Hz low pass filter
//
//	i2c_transmitinit( MPU6050_I2C_ADDR, 2, set ) ;
//	while(!i2c_transferred()) /* Wait for transfer */ ;
//
//	set[0] = MPU6050_SMPLRT_DIV ;
//	set[1] = 9;		// Setting 100Hz sample rate
//
//	i2c_transmitinit( MPU6050_I2C_ADDR, 2, set ) ;
//	while(!i2c_transferred()) /* Wait for transfer */ ;
}

void mpu6050_start(void) {

//	uint8_t receive = 0 ;
	uint8_t set[] = {0,0} ;

	set[0] = MPU6050_PWR_MGMT_1 ;
	set[1] = 0x00;		// Just switching it on - clear sleep bit

	i2c_transmitinit( MPU6050_I2C_ADDR, 2, set ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;

}

void mpu6050_get_data(uint8_t *buffer) {

	uint8_t request_MSB = MPU6050_ACC_XOUT_H ;

	i2c_transmitinit( MPU6050_I2C_ADDR, 1, &request_MSB ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;

	i2c_receiveinit( MPU6050_I2C_ADDR, 14, buffer ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;
}

void mpu6050_decode(struct mpu6050_data *const ba , uint8_t *buffer) {

	ba->acc_x = ((buffer[0]<<8) + buffer[1]);

	ba->acc_y = ((buffer[2]<<8) + buffer[3]);

	ba->acc_z = ((buffer[4]<<8) + buffer[5]);

	ba->temp = ((buffer[6]<<8) + buffer[7]);

	ba->gyro_x = ((buffer[8]<<8) + buffer[9]);

	ba->gyro_y = ((buffer[10]<<8) + buffer[11]);

	ba->gyro_z = ((buffer[12]<<8) + buffer[13]);
}


// Jagun: 01 Sept 2012: added the following to comply with
// Contiki sensor interface structure
static int
value(int type)
{
    static uint8_t buffer;
    static struct mpu6050_data ba;

    switch(type) {
        //  Must get mag_x, y, z after getting the data first
        case MPU6050_GET_DATA: // In fact, get data and decode it
        default:
            mpu6050_get_data(&buffer);
            mpu6050_decode(&ba, &buffer);
            return buffer;
        case MPU6050_GYRO_X:
            return ba.gyro_x;
        case MPU6050_GYRO_Y:
            return ba.gyro_y;
        case MPU6050_GYRO_Z:
            return ba.gyro_z;
        case MPU6050_ACC_X:
            return ba.acc_x;
        case MPU6050_ACC_Y:
            return ba.acc_y;
        case MPU6050_ACC_Z:
            return ba.acc_z;
        case MPU6050_TEMP:
            return ba.temp;
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
	mpu6050_init();
    mpu6050_start();
	// TODO Also implement the DEACTIVATE code
    return 1;
}

// Instantiate the sensor 
SENSORS_SENSOR(mpu6050_sensor, "mpu6050-Sensor", value, configure, status);

