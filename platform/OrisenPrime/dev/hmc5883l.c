/*
 * hmc5883l.c
 *
 *  Created on: 6 Dec 2011
 *      Author: mcphillips
 *
 *  Updated on: 01 Sept 2012 by Jagun Kwon (j.kwon@cs.ucl.ac.uk)
 *  Refactored to comply with Contiki sensor interface structure
 *  hmc5883l: 3-Axis Digital Compass
 */

#include <stdio.h>
#include "i2c.h"
#include "include/hmc5883l.h"

#include "lib/sensors.h"    // for Contiki sensor interface

extern volatile uint8_t timer_delay;

void hmc5883l_init(void) {
	uint8_t set[] = {0,0} ;

	set[0] = HMC5883L_RA_CONFIG_A ;
	set[1] = (HMC5883L_AVERAGING_2 | HMC5883L_RATE_75 | HMC5883L_BIAS_NORMAL);
	i2c_transmitinit( HMC5883L_I2C_ADDR, 2, set ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;

	set[0] = HMC5883L_RA_CONFIG_B ;
	set[1] = HMC5883L_GAIN_1090;	// default is HMC5883L_GAIN_1090 - if changing change the scaling below too.
	i2c_transmitinit( HMC5883L_I2C_ADDR, 2, set ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;
}

void hmc5883l_start(void) {
	uint8_t set[] = {0,0} ;

	set[0] = HMC5883L_RA_MODE ;
	set[1] = HMC5883L_MODE_CONTINUOUS;
	i2c_transmitinit( HMC5883L_I2C_ADDR, 2, set ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;
}

void hmc5883l_stop(void) {
	uint8_t set[] = {0,0} ;

	set[0] = HMC5883L_RA_MODE ;
	set[1] = HMC5883L_MODE_IDLE;
	i2c_transmitinit( HMC5883L_I2C_ADDR, 2, set ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;
}

void hmc5883l_get_data(uint8_t *buffer) {
	uint8_t request_MSB = HMC5883L_RA_DATAX_H ;

	i2c_transmitinit( HMC5883L_I2C_ADDR, 1, &request_MSB ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;

	i2c_receiveinit( HMC5883L_I2C_ADDR, 6, buffer ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;
}

void hmc5883l_decode(struct hmc5883l_data *mag_data , uint8_t *buffer) {
	mag_data->mag_x = ((buffer[0]<<8) + buffer[1]);
	mag_data->mag_z = ((buffer[2]<<8) + buffer[3]);
	mag_data->mag_y = ((buffer[4]<<8) + buffer[5]);
}


// Jagun: 01 Sept 2012: added the following to comply with
// Contiki sensor interface structure
static int
value(int type)
{
    static uint8_t buffer[6];
    static struct hmc5883l_data mag_data;

    switch (type) {
					// Must get mag_x, y, z after getting the data first
        case HMC5883L_LATCH_DATA:	// In fact, get data and decode it
	default:
	    hmc5883l_get_data(buffer);
	    hmc5883l_decode(&mag_data, buffer);
            return 1;
        case HMC5883L_MAG_X_RAW:
	    return mag_data.mag_x;
        case HMC5883L_MAG_Y_RAW:
	    return mag_data.mag_y;
        case HMC5883L_MAG_Z_RAW:
	    return mag_data.mag_z;
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
	hmc5883l_init();
	hmc5883l_start();
    return 1;
}

// Instantiate the sensor : 3-Axis Digital Compass
SENSORS_SENSOR(hmc5883l_sensor, "hmc5883l-Sensor", value, configure, status);


