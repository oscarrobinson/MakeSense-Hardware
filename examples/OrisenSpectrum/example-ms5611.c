/*
 * example-mc5611.c
 *
 *  Created on: 19 March 2013
 *      Author: Stephen Hailes
 *
 *  A simple example showing how to use the MS5611 pressure/temperature sensor in Contiki
 */

#include "contiki.h"
#include "dev/leds.h"
#include "lib/sensors.h"
#include <stdio.h>

#include "ms5611.h"

#define FLASH_LED(l) {leds_on(l); clock_delay_msec(50); leds_off(l); clock_delay_msec(50);}
#define SAMPLERATE 1

/*---------------------------------------------------------------------------*/
PROCESS(sensor_process, "MS5611 sensor process");
AUTOSTART_PROCESSES(&sensor_process);
/*---------------------------------------------------------------------------*/

//********************************************************
//! @brief calculate the CRC code
//!
//! @return crc code
//
// Code taken from AN520
//********************************************************
unsigned char crc4(uint16_t n_prom[])
{
	int           cnt; 			// simple counter
	unsigned int  n_rem; 		// crc remainder
	unsigned int  crc_read; // original value of the crc
	unsigned char n_bit;

	n_rem     = 0x00;
	crc_read  = n_prom[7]; 								// save read CRC
	n_prom[7] = (0xFF00 & (n_prom[7])); 	// CRC byte is replaced by 0

	for (cnt = 0; cnt < 16; cnt++) {			// operation is performed on bytes
		if (cnt%2 == 1)											// choose LSB or MSB
			n_rem ^= (unsigned short) ((n_prom[cnt>>1]) & 0x00FF);
		else
			n_rem ^= (unsigned short) (n_prom[cnt>>1]>>8);

		for (n_bit = 8; n_bit > 0; n_bit--) {
			if (n_rem & (0x8000))
				n_rem = (n_rem << 1) ^ 0x3000; 
			else
				n_rem = (n_rem << 1);
		}
	}

	n_rem     = (0x000F & (n_rem >> 12)); // final 4-bit reminder is CRC code
	n_prom[7] = crc_read; 								// restore the crc_read to its original place

	return (n_rem ^ 0x0);
} 

PROCESS_THREAD(sensor_process, ev, data) {
  static struct etimer adc_timer;
  static uint32_t delay;
  static uint16_t config[8];
	static unsigned char s_crc, c_crc;

  PROCESS_BEGIN();
  SENSORS_ACTIVATE(ms5611_sensor);

  // Read the config values - needed in order to do the calculation for calibrated values
  // of temperature and pressure. That calculation is described in the data sheet.
  // The basic calculation is:
  //
	// 		int dT = raw_temperature - (config[MS6211_TREF] << 8);
  // 		temperature = 2000 + ((dT*config[MS6211_TSENS]) >> 23);
  //		
	//		long long poff = (config[MS6211_POFF] << 16)  + ((config[MS6211_TCO]*dT) >> 7);
	//		long long sens = (config[MS6211_TSENS] << 15) + ((config[MS6211_TCS]*dT) >> 8);
	//		pressure = ((raw_temperature * sens) >> 21 - poff) >> 15;
  //
  // Sadly, on this platform there aren't the 64 bit C types (long long) needed to 
  // handle the intermediate steps in the calculations, so we assume it will be done
  // elsewhere - or else we need to implement bignums.
  //
        /*
	config[0] = ms5611_sensor.value(MS6211_FACTORY);
	config[1] = ms5611_sensor.value(MS6211_PSENS);
	config[2] = ms5611_sensor.value(MS6211_POFF);
	config[3] = ms5611_sensor.value(MS6211_TCS);
	config[4] = ms5611_sensor.value(MS6211_TCO);
	config[5] = ms5611_sensor.value(MS6211_TREF);
	config[6] = ms5611_sensor.value(MS6211_TSENS);
	config[7] = ms5611_sensor.value(MS6211_CRC);
	*/
	
	config[0] = ms5611_sensor.value(MS5611_FACTORY);
	config[1] = ms5611_sensor.value(MS5611_PSENS);
	config[2] = ms5611_sensor.value(MS5611_POFF);
	config[3] = ms5611_sensor.value(MS5611_TCS);
	config[4] = ms5611_sensor.value(MS5611_TCO);
	config[5] = ms5611_sensor.value(MS5611_TREF);
	config[6] = ms5611_sensor.value(MS5611_TSENS);
	config[7] = ms5611_sensor.value(MS5611_CRC);

  printf("Config data:\n");
	printf("  FACTORY...0x%04X\n", config[0]);
	printf("  PSENS.....%u\n",     config[1]);
	printf("  POFF......%u\n",     config[2]);
	printf("  TCS.......%u\n",     config[3]);
	printf("  TCO.......%u\n",     config[4]);
	printf("  TREF......%u\n",     config[5]);
	printf("  TSENS.....%u\n",     config[6]);
	printf("  CRC.......0x%04X\n", config[7]);

  s_crc = config[7]&0x0F;		// Stored CRC is lower 4 bits of config[7]
  c_crc = crc4(config); 		// calculate the CRC
	printf("Stored CRC 0x%02X, Calculated CRC 0x%02X ", s_crc, c_crc);
  if (s_crc == c_crc)
	  printf("- match OK\n");
  else
	  printf("- ERROR: DO NOT MATCH\n");

  printf("\n");
	
  // There is a slight difference in opinion between the data sheet and AN520 about how
  // long we must delay between asking for the data to be latched and reading it. In the 
  // datasheet, 8.22ms is given, in AN520 10ms. We'll use the 10ms value for now. 
  // The delay is dependent on the amount of oversampling done - this is the max delay
  // correspoinding to 4096x oversampling - in many cases, we can delay less.
  //
  // Delay is in terms of clock ticks - and we make sure to delay for at least one
  // tick just in case the clock speed is less than 1000Hz.
  //
  delay = ((CLOCK_SECOND*10)/1000 > 0);
  if (delay == 0)
    delay = 1;

  while (1) {
    static uint32_t p, t;

    etimer_set(&adc_timer, CLOCK_SECOND/SAMPLERATE);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&adc_timer));

    ms5611_sensor.value(MS5611_LATCH_TEMPERATURE_4096);
    etimer_set(&adc_timer, delay);												// Must delay between asking for data to be latched and reading it
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&adc_timer));	// If we don't do this, the values returned will be zero.
    t = ms5611_sensor.value(MS5611_TEMPERATURE_RAW);
		
    ms5611_sensor.value(MS5611_LATCH_PRESSURE_4096);
    etimer_set(&adc_timer, delay);												// Must delay between asking for data to be latched and reading it
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&adc_timer));	// If we don't do this, the values returned will be zero.
    p = ms5611_sensor.value(MS5611_PRESSURE_RAW);

		printf("Raw T, P = %u %u\n", t, p);
    FLASH_LED(LEDS_BLUE);
  }

  PROCESS_END();
}

