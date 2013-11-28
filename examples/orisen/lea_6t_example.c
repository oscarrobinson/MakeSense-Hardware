/*
 * lea_6t_example.c
 *
 *  Created on: 01 Sept 2012
 *  Author: Jagun Kwon and Stephen Hailes
 *  Last updated: 21 Jan 2013
 *
 *  A simple example showing how to use the Ublox Lea_6T GPS receiver in Contiki
 *  TODO: Currently doesn not conform to the Contiki sensor structre
 */

#include "contiki.h"
#include "dev/leds.h"
#include "lib/sensors.h"
#include <stdio.h>

#include <string.h>

#include "include/lea_6t.h"	// for GPS sensor

static struct etimer adc_timer;

#define FLASH_LED(l) {leds_on(l); clock_delay_msec(50); leds_off(l); clock_delay_msec(50);}
/*---------------------------------------------------------------------------*/
#define SAMPLERATE 1  // in seconds
//#define SAMPLERATE .001  // in seconds // 0.5 seems working well >500
// 0.01 works fine > 2000 bytes
/*---------------------------------------------------------------------------*/

PROCESS(sensor_process, "simple sensor process");
AUTOSTART_PROCESSES(&sensor_process);
/*---------------------------------------------------------------------------*/

PROCESS_THREAD(sensor_process, ev, data) {

	struct nav_sol_packet gps_data;
	uint8_t gps_data_buffer[255];
	char gps_str[255];

	static int cnt=0;

    //uint16_t byte_count;

  PROCESS_BEGIN();
//  SENSORS_ACTIVATE(lea_6t_sensor);
	printf("Initialising GPS...\n");
	lea_6t_init_gps();

	i2c_enable();
	printf("Initialised GPS...\n");
// TODO should be able to configure - to get NMEA data or UBIX data format

    uint8_t request = LEA_6T_DATA_STREAM ;

    i2c_transmitinit( LEA_6T_I2C_ADDR, 1, &request ) ;
    while(!i2c_transferred()) /* Wait for transfer */ ;

  while (1) {
    etimer_set(&adc_timer, CLOCK_SECOND*SAMPLERATE);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&adc_timer));

	request_RXM_RAW();	// Request only RXM_RAW msg

	// TODO: Trying two different parsers
	gps_ubx_parse(gps_data_buffer[0]);
	//ubx_parser(gps_data_buffer[0]);

		if (lea_6t_get_ubx_msg()) {	// read and parse UBX msg
// TODO
//			lea_6t_decode_gps(&gps_data, gps_data_buffer);
			extern struct GpsUbx gps_ubx;

			if (gps_ubx.msg_available == TRUE) {
				gps_ubx_read_message(&gps_data);  // This actually parse msg
				gps_ubx.msg_available = FALSE;

				sprintf(gps_str,"gps,%d,%d,%ld,%d,%ld,%ld,%ld,%ld\n",
					gps_data.gpsfix, gps_data.numsv, gps_data.itow,
					gps_data.week, gps_data.pacc,
					gps_data.ecef_x, gps_data.ecef_y, gps_data.ecef_z) ;
				printf(gps_str);
			}
		}
		printf(".\n");

    FLASH_LED(LEDS_BLUE);
  }
  PROCESS_END();
}

