/*
 * co-ads1015.c
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
#include "co-ads1015.h"	// ADC connected to the CO sensor
#include "i2c.h"
//#include "board/OrisenPrime.h"
#include "clock.h"
#include "uart1.h"
#include <math.h>	// for floating point operation

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

PROCESS(COmon_process, "CO monitoring process");
AUTOSTART_PROCESSES(&COmon_process);
/*---------------------------------------------------------------------------*/

PROCESS_THREAD(COmon_process, ev, data) {

	PROCESS_BEGIN();
		i2c_enable();

		SENSORS_ACTIVATE(co_sensor);

		etimer_set(&adc_timer, CLOCK_SECOND);

		while(1) {

			PROCESS_WAIT_EVENT();

			if(etimer_expired(&adc_timer)) {

				etimer_set(&adc_timer, CLOCK_SECOND*SAMPLERATE);
				LED1_ON;
				etimer_set(&led_timer, CLOCK_SECOND*0.5);

				PRINTF("Current CO value is = %d\n", co_sensor.value(0));

			}//etimer_expired

			if(etimer_expired(&led_timer)) {
				LED1_OFF;
			}

		}//end while

	PROCESS_END();
}

