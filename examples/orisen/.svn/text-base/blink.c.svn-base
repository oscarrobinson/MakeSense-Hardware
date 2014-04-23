/*
 * co-ads1015.c
 *
 *  Created on: 27 Aug 2012
 *      Author: Jagun Kwon
 *
 *  A simple example showing how to use the sensor in Contiki
 */

#include "contiki.h"

#include "dev/leds.h"
#include "clock.h"
#include "mc1322x.h"

#define APPS

static struct etimer led_on_timer;
static struct etimer led_off_timer;

#define DEBUG 1
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif


PROCESS(blink_process, "LED Blink process");
AUTOSTART_PROCESSES(&blink_process);
/*---------------------------------------------------------------------------*/

PROCESS_THREAD(blink_process, ev, data) {

	PROCESS_BEGIN();
	  LED1_OFF;
		etimer_set(&led_on_timer, CLOCK_SECOND);

		while(1) {

			PROCESS_WAIT_EVENT();

			if(etimer_expired(&led_on_timer)) {

				etimer_set(&led_on_timer, CLOCK_SECOND);
				LED1_ON;
				etimer_set(&led_off_timer, CLOCK_SECOND*0.5);

			}//etimer_expired


			if(etimer_expired(&led_off_timer)) {
				LED1_OFF;
			}

		}//end while

	PROCESS_END();
}

