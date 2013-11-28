/*
 * example-HMC5883L.c
 *
 *  Created on: 01 Sept 2012
 *      Author: Jagun Kwon and Stephen Hailes
 *
 *  A simple example showing how to use the HMC5883L magnetometer in Contiki
 */

#include "contiki.h"
#include "dev/leds.h"
#include "lib/sensors.h"
#include <stdio.h>

#include "hmc5883l.h"

static struct etimer adc_timer;

#define FLASH_LED(l) {leds_on(l); clock_delay_msec(50); leds_off(l); clock_delay_msec(50);}
/*---------------------------------------------------------------------------*/
#define SAMPLERATE 1  // in seconds
/*---------------------------------------------------------------------------*/

PROCESS(sensor_process, "simple sensor process");
AUTOSTART_PROCESSES(&sensor_process);
/*---------------------------------------------------------------------------*/

PROCESS_THREAD(sensor_process, ev, data) {
  int16_t mag_x;
  int16_t mag_y;
  int16_t mag_z;

  PROCESS_BEGIN();
  SENSORS_ACTIVATE(hmc5883l_sensor);

  while (1) {
    etimer_set(&adc_timer, CLOCK_SECOND*SAMPLERATE);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&adc_timer));

    // Must latch the data first before getting mag_x, y, z
    hmc5883l_sensor.value(HMC5883L_LATCH_DATA);
    mag_x = hmc5883l_sensor.value(HMC5883L_MAG_X_RAW);
    mag_y = hmc5883l_sensor.value(HMC5883L_MAG_Y_RAW);
    mag_z = hmc5883l_sensor.value(HMC5883L_MAG_Z_RAW);
    printf("HMC5883L Mag X,Y,Z = %4d, %4d, %4d\n", mag_x, mag_y, mag_z);
    FLASH_LED(LEDS_BLUE);
  }

  PROCESS_END();
}

