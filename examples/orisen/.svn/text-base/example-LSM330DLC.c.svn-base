/*
 * example-LSM330DLC.c
 *
 *  Created on: 27th January 2013
 *      Author: Stephen Hailes
 *
 *  A simple example showing how to use the LSM330DLC IMU in Contiki
 */

#include "contiki.h"
#include "dev/leds.h"
#include "lib/sensors.h"
#include <stdio.h>

#include "lsm330dlc.h"

static struct etimer adc_timer;

#define FLASH_LED(l) {leds_on(l); clock_delay_msec(50); leds_off(l); clock_delay_msec(50);}
/*---------------------------------------------------------------------------*/
#define SAMPLERATE 1  // in seconds
/*---------------------------------------------------------------------------*/
PROCESS(sensor_process, "simple sensor process");
AUTOSTART_PROCESSES(&sensor_process);
/*---------------------------------------------------------------------------*/

PROCESS_THREAD(sensor_process, ev, data) {
  int16_t acc_x, acc_y, acc_z;
  int16_t gyr_x, gyr_y, gyr_z;

  PROCESS_BEGIN();
  SENSORS_ACTIVATE(lsm330dlc_sensor);

	// Enable FIFO mode for the accelerometer
	//lsm330dlc_FIFOModeEnable_A(LSM330DLC_FIFO_MODE_A);

/*
	lsm330dlc_sensor.configure(SENSORS_ACTIVE,
		LSM330DLC_SET_POWER_NORMAL | LSM330DLC_SET_FIFO_MODE_FIFO |
		LSM330DLC_SET_ACC_FULLSCALE_2G | LSM330DLC_SET_ACC_FREQ_100Hz |
		LSM330DLC_SET_GYR_FULLSCALE_2000);
*/
//	lsm330dlc_sensor.configure(SENSORS_ACTIVE,LSM330DLC_SET_POWER_NORMAL | LSM330DLC_SET_FIFO_MODE_STREAM | LSM330DLC_SET_GYR_FULLSCALE_500);

lsm330dlc_sensor.configure(
        SENSORS_ACTIVE,
        LSM330DLC_SET_POWER_NORMAL |
        LSM330DLC_SET_FIFO_MODE_STREAM |
        LSM330DLC_SET_ACC_FULLSCALE_2G |
        LSM330DLC_SET_ACC_FREQ_200Hz |
        LSM330DLC_SET_GYR_FREQ_190Hz |
        LSM330DLC_SET_GYR_FULLSCALE_500);

  while (1) {
   etimer_set(&adc_timer, CLOCK_SECOND*SAMPLERATE);
   PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&adc_timer));

    // Must latch the data first before getting mag_x, y, z
    lsm330dlc_sensor.value(LSM330DLC_LATCH_DATA);
    acc_x = lsm330dlc_sensor.value(LSM330DLC_ACC_X_RAW);
    acc_y = lsm330dlc_sensor.value(LSM330DLC_ACC_Y_RAW);
    acc_z = lsm330dlc_sensor.value(LSM330DLC_ACC_Z_RAW);
    gyr_x = lsm330dlc_sensor.value(LSM330DLC_GYR_X_RAW);
    gyr_y = lsm330dlc_sensor.value(LSM330DLC_GYR_Y_RAW);
    gyr_z = lsm330dlc_sensor.value(LSM330DLC_GYR_Z_RAW);
    printf("LSM330DLC Acc = %d, %d, %d, Gyr = %d, %d, %d\n", acc_x, acc_y, acc_z, gyr_x, gyr_y, gyr_z);


	// TODO
	// Jagun: Ideally we should be able to configure other parameters this way
	// lsm330dlc_sensor.configure(SENSORS_ACTIVE, LSM330DLC_FIFO_MODE) etc
    FLASH_LED(LEDS_BLUE);
  }

  PROCESS_END();
}

