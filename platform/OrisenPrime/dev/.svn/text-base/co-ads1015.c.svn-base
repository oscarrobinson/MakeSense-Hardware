/*
 *  CO sensor reader & decoder
 *  co-asd1015.c
 *
 *  Created on: 1 Sep 2011
 *      Author: mcphillips
 *
 *  Updated on: 20 Aug 2012 by Jagun Kwon (j.kwon@cs.ucl.ac.uk)
 *  Refactored to comply with Contiki sensor interface structure
 */

#include <stdio.h>
#include "i2c.h"
#include "include/co-ads1015.h"
#include "lib/sensors.h"


void co_ads1015_init(uint8_t co_ads1015_ainx, uint8_t ads1015_gainx) {

	uint8_t request = CO_ADS1015_CONF_REG ;
	uint8_t receive[] = {0,0} ;
	uint8_t set[] = {0,0,0} ;

	i2c_transmitinit( CO_ADS1015_I2C_ADDR, 1, &request ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;

	i2c_receiveinit( CO_ADS1015_I2C_ADDR, 2, receive ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;

	set[0] = CO_ADS1015_CONF_REG ;
	//conf_set[0] = set[0];
	set[1] =  ((receive[0] & CO_ADS1015_RANGE_MASK & CO_ADS1015_GAIN_MASK) | co_ads1015_ainx | ads1015_gainx | CO_ADS1015_START_SINGLE_CONV|0x01);
	// set[1] =  ((receive[0] & CO_ADS1015_RANGE_MASK & CO_ADS1015_GAIN_MASK) | co_ads1015_ainx | ads1015_gainx | CO_ADS1015_START_SINGLE_CONV);
	//conf_set[1] = set[1];
	set[2] =  receive[1];
	//printf("set[1]=%02X, set[2]=%02X\r\n", set[1], set[2]);
	//set[2] = 0x83;

	i2c_transmitinit( CO_ADS1015_I2C_ADDR, 3, set ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;

}


void co_ads1015_get_data(uint8_t co_ads1015_ainx, uint8_t *buffer) {

	uint8_t request_MSB = CO_ADS1015_CONV_REG ;
	uint8_t read[] = {0,0} ;

	i2c_transmitinit( CO_ADS1015_I2C_ADDR, 1, &request_MSB ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;

	i2c_receiveinit( CO_ADS1015_I2C_ADDR, 2, read ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;
	
	//printf("read = 0x%02X, 0x%02X\r\n", read[0], read[1]);
	switch (co_ads1015_ainx){
		case ((0x04)<<4):
			buffer[5] = read[0];
			buffer[6] = read[1];
			break;
		case ((0x05)<<4):
			buffer[7] = read[0];
			buffer[8] = read[1];
			break;
		case ((0x06)<<4):
			buffer[9] = read[0];
			buffer[10] = read[1];
			break;
		case ((0x07)<<4):
			buffer[11] = read[0];
			buffer[12] = read[1];
			break;
		}
}

uint8_t co_ads1015_check_dataready(void){
	uint8_t request = CO_ADS1015_CONF_REG ;
	uint8_t read[] = {0,0} ;

	i2c_transmitinit( CO_ADS1015_I2C_ADDR, 1, &request ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;

	i2c_receiveinit( CO_ADS1015_I2C_ADDR, 2, read ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;

	//printf("0x%02X, ", (char) read[0]);
	return ((read[0]&0x80)>>7);
}
	
void co_ads1015_decode_data(struct co_ads1015_data *const ba, uint8_t *buffer) {
	int16_t temp_adc;
	// int16_t mask = 0x0800, mask_val = 0x7FF;
	// TODO Jagun-In Graeme's code the mask values are different
	const int16_t mask = 0x2000, mask_val = 0x3FFF;
	temp_adc = ((buffer[5]<<4) + (buffer[6]>>4));
	//printf("Data[0]=0x%X, Data[1]=0x%X, temp_adc=0x%X\r\n",buffer[5], buffer[6], temp_adc );
	if ((temp_adc & mask) == mask){
		temp_adc = (((~(temp_adc) & mask_val)+1) *-1) ;
	}
	ba->adc0 = temp_adc;

	temp_adc = ((buffer[7]<<4) + (buffer[8]>>4));
	if ((temp_adc & mask) == mask){
		temp_adc = (((~(temp_adc) & mask_val)+1) *-1) ;
	}
	ba->adc1 = temp_adc;

	temp_adc = ((buffer[9]<<4) + (buffer[10]>>4));
	if ((temp_adc & mask) == mask){
		temp_adc = (((~(temp_adc) & mask_val)+1)*-1) ;
	}
	ba->adc2 = temp_adc;

	temp_adc = ((buffer[11]<<4) + (buffer[12]>>4));
	if ((temp_adc & mask) == mask){
		temp_adc = (((~(temp_adc) & mask_val)+1)*-1) ;
	}
	ba->adc3 = temp_adc;
#if 0
	temp_adc = (buffer[5]<<4) + (buffer[6]>>4);
	if ((buffer[5] & 0x80) == 0x80){
		temp_adc = temp_adc | 0xF800;
	}

	ba->adc0 = temp_adc;
#endif
//end
}

void co_ads1015_start_adc(void) {

	uint8_t request = CO_ADS1015_CONV_REG ;
	i2c_transmitinit( CO_ADS1015_I2C_ADDR, 1, &request ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;
	clock_delay_msec(50);
}


// Jagun: 20 Aug 2012: added the following to comply with
// Contiki sensor interface structure

// TODO make these less visible to others...
struct co_ads1015_data co_adc_data;
uint8_t co_ads1015_data_buffer[15];

static int
getCOadc(void)
{
//    co_ads1015_start_adc();
	co_ads1015_init(CO_ADS1015_AIN0,CO_ADS1015_GAIN2);
    co_ads1015_get_data(CO_ADS1015_AIN0, co_ads1015_data_buffer);
    set_fletcher_ck(&co_ads1015_data_buffer[2],11);
    co_ads1015_decode_data(&co_adc_data, co_ads1015_data_buffer);
    //PRINTF("ADC0 value = %d\n", co_adc_data.adc0);
    return co_adc_data.adc0;
}

static int
value(int type)
{
	// TODO Cater for RAW and DECODED values
	switch(type) {
		case 0:
		default:
			return getCOadc();
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
	// TODO Check if these are necessary: Jagun: didn't make any difference
	/*
	//what are these parameters fNEWor - we can do without?
    co_ads1015_data_buffer[0] = 0xBF;
    co_ads1015_data_buffer[1] = 0x3F;
    co_ads1015_data_buffer[2] = 0x06;
    co_ads1015_data_buffer[3] = 0x01;
    co_ads1015_data_buffer[4] = 0x0F;
    co_ads1015_data_buffer[12] = 0x00;
	*/

	co_ads1015_init(CO_ADS1015_AIN0,CO_ADS1015_GAIN2);
	return 1;
}

// Instantiate the sensor 
SENSORS_SENSOR(co_sensor, "CO-Sensor", value, configure, status);


