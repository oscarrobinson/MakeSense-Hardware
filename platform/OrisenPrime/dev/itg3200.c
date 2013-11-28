/*
 * itg3200.c
 *
 *  Created on: 6 Dec 2011
 *      Author: mcphillips
 *
 *  Updated on: 01 Sept 2012 by Jagun Kwon (j.kwon@cs.ucl.ac.uk)
 *  Refactored to comply with Contiki sensor interface structure
 *  itg3200: Three-Axis Digital Gyroscope (Sparkfun)
 */

#include <stdio.h>
#include "i2c.h"
#include "include/itg3200.h"

#include "lib/sensors.h"

extern volatile uint8_t timer_delay;

void itg3200_stop(void) {

//	uint8_t receive = 0 ;
	uint8_t set[] = {0,0} ;

	set[0] = ITG3200_PWR_MGM ;
	set[1] = 0x40;		// Just switching it off

	i2c_transmitinit( ITG3200_I2C_ADDR, 2, set ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;

}

void itg3200_init(void) {

//	uint8_t receive = 0 ;
	uint8_t set[] = {0,0} ;

	set[0] = ITG3200_DLPF_FS ;
	set[1] = (ITG3200_DLPFFS_FS_SEL | ITG3200_BW020_SR1);		// Setting 20Hz low pass filter

	i2c_transmitinit( ITG3200_I2C_ADDR, 2, set ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;

	set[0] = ITG3200_SMPLRT_DIV ;
	set[1] = 9;		// Setting 100Hz sample rate

	i2c_transmitinit( ITG3200_I2C_ADDR, 2, set ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;
}

void itg3200_start(void) {

//	uint8_t receive = 0 ;
	uint8_t set[] = {0,0} ;

	set[0] = ITG3200_PWR_MGM ;
	set[1] = 0x00;		// Just switching it on - clear sleep bit

	i2c_transmitinit( ITG3200_I2C_ADDR, 2, set ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;

}

void itg3200_get_data(uint8_t *buffer) {

	uint8_t request_MSB = ITG3200_TEMP_OUT ;

	i2c_transmitinit( ITG3200_I2C_ADDR, 1, &request_MSB ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;

	i2c_receiveinit( ITG3200_I2C_ADDR, 8, buffer ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;
}

void itg3200_decode(struct itg3200_data *const ba , uint8_t *buffer) {

	ba->temp = ((buffer[0]<<8) + buffer[1]);

	ba->gyro_x = ((buffer[2]<<8) + buffer[3]);

	ba->gyro_y = ((buffer[4]<<8) + buffer[5]);

	ba->gyro_z = ((buffer[6]<<8) + buffer[7]);
}


// Jagun: 01 Sept 2012: added the following to comply with
// Contiki sensor interface structure
static int
value(int type)
{
    static uint8_t buffer;
    static struct itg3200_data ba;

    switch(type) {
        //  Must get mag_x, y, z after getting the data first
        case ITG3200_GET_DATA: // In fact, get data and decode it
        default:
            itg3200_get_data(&buffer);
            itg3200_decode(&ba, &buffer);
            return buffer;
        case ITG3200_GYRO_X:
            return ba.gyro_x;
        case ITG3200_GYRO_Y:
            return ba.gyro_y;
        case ITG3200_GYRO_Z:
            return ba.gyro_z;
        case ITG3200_TEMP:
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
    itg3200_init();
    itg3200_start();
    return 1;
}

// Instantiate the sensor : 3-Axis Digital Compass
SENSORS_SENSOR(itg3200_sensor, "itg3200-Sensor", value, configure, status);


