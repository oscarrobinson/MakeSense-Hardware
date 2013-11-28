/*
 * amy_6m_example.c
 *
 *  Created on: 11 June 2013
 *  Author: Jagun Kwon
 *  Last updated: 21 June 2013
 *
 *  A simple example showing how to use the Ublox Amy_6M GPS receiver in Contiki
 *  TODO: Currently doesn not conform to the Contiki sensor structre
 */

#include "contiki.h"
#include "dev/leds.h"
#include "lib/sensors.h"
#include <stdio.h>

#include <string.h>

#include "include/amy_6m.h"	// for GPS sensor

#define RX_WITH_DMA 1

static struct etimer adc_timer;

#define FLASH_LED(l) {leds_on(l); clock_delay_msec(50); leds_off(l); clock_delay_msec(50);}
/*---------------------------------------------------------------------------*/
#define SAMPLERATE 1  // in seconds
//#define SAMPLERATE .05  // in seconds // 0.5 seems working well >500
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
//  SENSORS_ACTIVATE(amy_6m_sensor);
	printf("Initialising GPS...\n");
	amy_6m_init_gps();

	printf("Initialised GPS...\n");
// TODO should be able to configure - to get NMEA data or UBIX data format

    uint8_t request = AMY_6M_DATA_STREAM ;

    i2c_transmitinit( AMY_6M_I2C_ADDR, 1, &request ) ;
    while(!i2c_transferred()) /* Wait for transfer */ ;
	//uart2_putc(request);

  while (1) {
    etimer_set(&adc_timer, CLOCK_SECOND*SAMPLERATE);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&adc_timer));

	// AMY_6M is not capable of this
	//request_RXM_RAW();	// Request only RXM_RAW msg

	// TODO: Trying two different parsers
	//gps_ubx_parse(gps_data_buffer[0]);
	//ubx_parser(gps_data_buffer[0]);

	//if (amy_6m_get_ubx_msg()) {	// read and parse UBX msg
	if (amy_6m_get_nmea_msg(gps_data_buffer)) {	// read and parse NMEA msg
//			amy_6m_decode_gps(&gps_data, gps_data_buffer);
//		extern struct GpsUbx gps_ubx;

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
//	printf(".\n");

    FLASH_LED(LEDS_BLUE);
  }
  PROCESS_END();
}

