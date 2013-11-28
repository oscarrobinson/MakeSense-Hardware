/*
 * SHT2x_example.c
 *
 *  Created on: 27 Aug 2012
 *      Author: Jagun Kwon
 *
 *  A simple example showing how to use the sensor in Contiki
 */

#include "contiki.h"

//define COOJASIM
#ifndef COOJASIM
#include "utils.h"
#include "adc.h"
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
#include "SHT2x.h"	// SHT2x sensor

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

	PROCESS_BEGIN();
		i2c_enable();

        SENSORS_ACTIVATE(sht2x_sensor);

		etimer_set(&adc_timer, CLOCK_SECOND);

		while(1) {

			PROCESS_WAIT_EVENT();

			if(etimer_expired(&adc_timer)) {

				etimer_set(&adc_timer, CLOCK_SECOND*SAMPLERATE);
				LED1_ON;
				etimer_set(&led_timer, CLOCK_SECOND*0.5);

				PRINTF("Current temperature is = %d\n",
						sht2x_sensor.value(SHT2X_TEMP));
				PRINTF("Current humidity is = %d\n",
						sht2x_sensor.value(SHT2X_HUMIDITY));

			}//etimer_expired

			if(etimer_expired(&led_timer)) {
				LED1_OFF;
			}

		}//end while

	PROCESS_END();
}

