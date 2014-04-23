/*
 * itg3200_example.c
 *
 *  Created on: 01 Sept 2012
 *      Author: Jagun Kwon
 *
 *  A simple example showing how to use the sensor in Contiki
 */

#include "contiki.h"

//define COOJASIM
#ifndef COOJASIM
#include "utils.h"
#endif

#include "dev/leds.h"
#include <stdio.h>
#include <string.h>

#include "lib/random.h"
#include "dev/button-sensor.h"
#include "i2c.h"
#include "board/OrisenPrime.h"
#include "clock.h"
#include "uart1.h"
#include <math.h>	// for floating point operation

#include "itg3200.h"	// for itg3200 gyro sensor


#define APPS

static struct etimer adc_timer;
static struct etimer led_timer;

#define DEBUG 1
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif


/*---------------------------------------------------------------------------*/
//#define SAMPLERATE 1  // in seconds
#define SAMPLERATE .1  // in seconds
/*---------------------------------------------------------------------------*/

PROCESS(sensor_process, "simple sensor process");
AUTOSTART_PROCESSES(&sensor_process);
/*---------------------------------------------------------------------------*/

PROCESS_THREAD(sensor_process, ev, data) {

	uint8_t buffer;
	int16_t gyro_x;
	int16_t gyro_y;
	int16_t gyro_z;
	int16_t temp;

	PROCESS_BEGIN();
		i2c_enable();

     	SENSORS_ACTIVATE(itg3200_sensor);

		etimer_set(&adc_timer, CLOCK_SECOND);

		while(1) {

			PROCESS_WAIT_EVENT();

			if(etimer_expired(&adc_timer)) {

				etimer_set(&adc_timer, CLOCK_SECOND*SAMPLERATE);
				LED1_ON;
				etimer_set(&led_timer, CLOCK_SECOND*0.5);

				// Must get the data first before getting gyro_x, y, z
				buffer = itg3200_sensor.value(ITG3200_GET_DATA);

				gyro_x = itg3200_sensor.value(ITG3200_GYRO_X);
				gyro_y = itg3200_sensor.value(ITG3200_GYRO_Y);
				gyro_z = itg3200_sensor.value(ITG3200_GYRO_Z);
				temp = itg3200_sensor.value(ITG3200_TEMP);
				PRINTF("ITG3200 Gyro output X,Y,Z = %d,%d,%d.\n",
						gyro_x, gyro_y, gyro_z);
				PRINTF("ITG3200 Gyro temp output = %d\n", temp);

			}//etimer_expired

			if(etimer_expired(&led_timer)) {
				LED1_OFF;
			}

		}//end while

	PROCESS_END();
}

