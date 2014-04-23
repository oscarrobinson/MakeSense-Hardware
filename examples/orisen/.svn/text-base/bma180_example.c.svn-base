/*
 * bma180_example.c
 *
 *  Created on: 29 Aug 2012
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


#include "bma180.h"	// for bma180 sensor


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
#define SAMPLERATE 1  // in seconds
/*---------------------------------------------------------------------------*/

PROCESS(sensor_process, "simple sensor process");
AUTOSTART_PROCESSES(&sensor_process);
/*---------------------------------------------------------------------------*/

PROCESS_THREAD(sensor_process, ev, data) {

	int8_t temp;
	int16_t acc_x;
	int16_t acc_y;
	int16_t acc_z;

	PROCESS_BEGIN();
		i2c_enable();

        SENSORS_ACTIVATE(bma180_sensor);

		etimer_set(&adc_timer, CLOCK_SECOND);

		while(1) {

			PROCESS_WAIT_EVENT();

			if(etimer_expired(&adc_timer)) {

				etimer_set(&adc_timer, CLOCK_SECOND*SAMPLERATE);
				LED1_ON;
				etimer_set(&led_timer, CLOCK_SECOND*0.5);

				// CAUTION: It is safe to assign sensor readings to a variable
				// and then use the variable.

				temp = bma180_sensor.value(BMA180_TEMPERATURE);
				acc_x = bma180_sensor.value(BMA180_ACC_X);
				acc_y = bma180_sensor.value(BMA180_ACC_Y);
				acc_z = bma180_sensor.value(BMA180_ACC_Z);
				PRINTF("Current temperature is = %d\n", temp);
				PRINTF("BMA180 Acc X,Y,Z = %d,%d,%d.\n", acc_x, acc_y, acc_z);

			}//etimer_expired

			if(etimer_expired(&led_timer)) {
				LED1_OFF;
			}

		}//end while

	PROCESS_END();
}

