/**
 * \file
 *         Driver for the MS5611-01BA03 barometric pressure sensor in I2C mode
 *
 * 				 We cannot easily do the calculation needed to transform the raw
 *         values into calibrated values because the numbers get too large
 *         to hold in standard C types for this compiler. In short, we need
 *				 64-bit integers in which to do the calculations. But even long
 *         longs are only 32 bits on this platform.
 *
 *				 Instead we will allow a client to obtain the config values and
 * 				 the raw data to allow the calculation to be done elsewhere.
 *
 * \author
 *         Stephen Hailes
 */


#include <stdio.h>
#include "i2c.h"
#include "include/ms5611.h"
#include "lib/sensors.h"    // for Contiki sensor interface

static uint16_t config[8];

void ms5611_init(void) {
	uint8_t cmd = 0;

	cmd = MS5611_RESET;
	i2c_transmitinit( MS5611_I2C_ADDR, 1, &cmd );
	while(!i2c_transferred());

	clock_delay_msec(3);			// Must delay by at least 2.8ms	according to the datasheet
}

void ms5611_start(void) {
	uint8_t cmd = 0;
  uint8_t res[2];

	cmd = MS5611_PROM_FACTORY;
	i2c_transmitinit( MS5611_I2C_ADDR, 1, &cmd );
	while(!i2c_transferred());
	i2c_receiveinit( MS5611_I2C_ADDR, 2, res );
	while(!i2c_transferred());
	config[0] = res[1] + (res[0] << 8);

	cmd = MS5611_PROM_C1;
	i2c_transmitinit( MS5611_I2C_ADDR, 1, &cmd );
	while(!i2c_transferred());
	i2c_receiveinit( MS5611_I2C_ADDR, 2, res );
	while(!i2c_transferred());
	config[1] = res[1] + (res[0] << 8);

	cmd = MS5611_PROM_C2;
	i2c_transmitinit( MS5611_I2C_ADDR, 1, &cmd );
	while(!i2c_transferred());
	i2c_receiveinit( MS5611_I2C_ADDR, 2, res );
	while(!i2c_transferred());
	config[2] = res[1] + (res[0] << 8);

	cmd = MS5611_PROM_C3;
	i2c_transmitinit( MS5611_I2C_ADDR, 1, &cmd );
	while(!i2c_transferred());
	i2c_receiveinit( MS5611_I2C_ADDR, 2, res );
	while(!i2c_transferred());
	config[3] = res[1] + (res[0] << 8);

	cmd = MS5611_PROM_C4;
	i2c_transmitinit( MS5611_I2C_ADDR, 1, &cmd );
	while(!i2c_transferred());
	i2c_receiveinit( MS5611_I2C_ADDR, 2, res );
	while(!i2c_transferred());
	config[4] = res[1] + (res[0] << 8);

	cmd = MS5611_PROM_C5;
	i2c_transmitinit( MS5611_I2C_ADDR, 1, &cmd );
	while(!i2c_transferred());
	i2c_receiveinit( MS5611_I2C_ADDR, 2, res );
	while(!i2c_transferred());
	config[5] = res[1] + (res[0] << 8);

	cmd = MS5611_PROM_C6;
	i2c_transmitinit( MS5611_I2C_ADDR, 1, &cmd );
	while(!i2c_transferred());
	i2c_receiveinit( MS5611_I2C_ADDR, 2, res );
	while(!i2c_transferred());
	config[6] = res[1] + (res[0] << 8);

	cmd = MS5611_PROM_SERIAL_CODE;
	i2c_transmitinit( MS5611_I2C_ADDR, 1, &cmd );
	while(!i2c_transferred());
	i2c_receiveinit( MS5611_I2C_ADDR, 2, res );
	while(!i2c_transferred());
	config[7] = res[1] + (res[0] << 8);
}

void ms5611_stop(void) {
}


void ms5611_latch_pressure(int type) {
	uint8_t request;

  switch (type) {
    case MS5611_LATCH_PRESSURE_256:								// Start conversion. Then need to wait for 900us
  		request = MS5611_CONVERT_D1_256;
			break;
    case MS5611_LATCH_PRESSURE_512:								// Start conversion. Then need to wait for 3ms
  		request = MS5611_CONVERT_D1_512;
			break;
    case MS5611_LATCH_PRESSURE_1024:							// Start conversion. Then need to wait for 4ms
  		request = MS5611_CONVERT_D1_1024;
			break;
    case MS5611_LATCH_PRESSURE_2048:							// Start conversion. Then need to wait for 6ms
  		request = MS5611_CONVERT_D1_2048;
			break;
    case MS5611_LATCH_PRESSURE_4096:							// Start conversion. Then need to wait for 10ms
  		request = MS5611_CONVERT_D1_4096;
			break;
		default:
			return;
  }
	i2c_transmitinit(MS5611_I2C_ADDR, 1, &request);
	while(!i2c_transferred());
}

void ms5611_latch_temperature(int type) {
	uint8_t request;

  switch (type) {
    case MS5611_LATCH_TEMPERATURE_256:						// Start conversion. Then need to wait for 900us
  		request = MS5611_CONVERT_D2_256;
			break;
    case MS5611_LATCH_TEMPERATURE_512:						// Start conversion. Then need to wait for 3ms
  		request = MS5611_CONVERT_D2_512;
			break;
    case MS5611_LATCH_TEMPERATURE_1024:						// Start conversion. Then need to wait for 4ms
  		request = MS5611_CONVERT_D2_1024;
			break;
    case MS5611_LATCH_TEMPERATURE_2048:						// Start conversion. Then need to wait for 6ms
  		request = MS5611_CONVERT_D2_2048;
			break;
    case MS5611_LATCH_TEMPERATURE_4096:						// Start conversion. Then need to wait for 10ms
  		request = MS5611_CONVERT_D2_4096;
			break;
		default:
			return;
  }
	i2c_transmitinit(MS5611_I2C_ADDR, 1, &request);
	while(!i2c_transferred());
}

unsigned int ms5611_adc_read() {
	uint8_t request;
  uint8_t res[3];

  request = MS5611_ADC_READ;
	i2c_transmitinit(MS5611_I2C_ADDR, 1, &request);
	while(!i2c_transferred());

	i2c_receiveinit(MS5611_I2C_ADDR, 3, res);
	while(!i2c_transferred());

	return (res[2] + (res[1] << 8) + (res[0] << 16));
}

static int
value(int type)
{
  static struct MS5611_data data;

  switch (type) {
    case MS5611_LATCH_PRESSURE_256:								// Start conversion. Then need to wait for 900us
    case MS5611_LATCH_PRESSURE_512:								// Start conversion. Then need to wait for 3ms
    case MS5611_LATCH_PRESSURE_1024:							// Start conversion. Then need to wait for 4ms
    case MS5611_LATCH_PRESSURE_2048:							// Start conversion. Then need to wait for 6ms
    case MS5611_LATCH_PRESSURE_4096:							// Start conversion. Then need to wait for 10ms
			ms5611_latch_pressure(type);
			return 1;

    case MS5611_LATCH_TEMPERATURE_256:						// Start conversion. Then need to wait for 900us
    case MS5611_LATCH_TEMPERATURE_512:						// Start conversion. Then need to wait for 3ms
    case MS5611_LATCH_TEMPERATURE_1024:						// Start conversion. Then need to wait for 4ms
    case MS5611_LATCH_TEMPERATURE_2048:						// Start conversion. Then need to wait for 6ms
    case MS5611_LATCH_TEMPERATURE_4096:						// Start conversion. Then need to wait for 10ms
			ms5611_latch_temperature(type);
			return 1;

		case MS5611_PRESSURE_RAW:
			data.raw_pressure = ms5611_adc_read();			// Get the value - must have asked to latch it first
			return (data.raw_pressure);
		case MS5611_TEMPERATURE_RAW:
			data.raw_temperature = ms5611_adc_read();		// Get the value - must have asked to latch it first
			return (data.raw_temperature);
		case MS5611_FACTORY:
			return (config[MS5611_FACTORY]);
		case MS5611_PSENS:
			return (config[MS5611_PSENS]);
		case MS5611_POFF:
			return (config[MS5611_POFF]);
		case MS5611_TCS:
			return (config[MS5611_TCS]);
		case MS5611_TCO:
			return (config[MS5611_TCO]);
		case MS5611_TREF:
			return (config[MS5611_TREF]);
		case MS5611_TSENS:
			return (config[MS5611_TSENS]);
		case MS5611_CRC:
			return (config[MS5611_CRC]);
		default:
			return -1;
  }
}

static int
status(int type)
{
    // TODO
    return 1;
}

static int
configure(int type, int c)
{
	ms5611_init();
	ms5611_start();
  return 1;
}

// Instantiate the sensor
SENSORS_SENSOR(ms5611_sensor, "MS5611-01BA03-Sensor", value, configure, status);


