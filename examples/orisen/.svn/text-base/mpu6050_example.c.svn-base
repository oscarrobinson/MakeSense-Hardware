/*
 * mpu6050_example.c
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

#include "mpu6050.h"	// for mpu6050 6-axis motion tracking device


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

	uint8_t buffer;
	int16_t gyro_x;
	int16_t gyro_y;
	int16_t gyro_z;
	int16_t acc_x;
	int16_t acc_y;
	int16_t acc_z;
	int16_t temp;

	PROCESS_BEGIN();
		i2c_enable();

     	SENSORS_ACTIVATE(mpu6050_sensor);

		etimer_set(&adc_timer, CLOCK_SECOND);

		while(1) {

			PROCESS_WAIT_EVENT();

			if(etimer_expired(&adc_timer)) {

				etimer_set(&adc_timer, CLOCK_SECOND*SAMPLERATE);
				LED1_ON;
				etimer_set(&led_timer, CLOCK_SECOND*0.5);

				// Must get the data first before getting gyro_x, y, z
				buffer = mpu6050_sensor.value(MPU6050_GET_DATA);

				// You could also use a data struct in mpu6050.h
				// to obtain all the values in one go
				gyro_x = mpu6050_sensor.value(MPU6050_GYRO_X);
				gyro_y = mpu6050_sensor.value(MPU6050_GYRO_Y);
				gyro_z = mpu6050_sensor.value(MPU6050_GYRO_Z);
				acc_x = mpu6050_sensor.value(MPU6050_ACC_X);
				acc_y = mpu6050_sensor.value(MPU6050_ACC_Y);
				acc_z = mpu6050_sensor.value(MPU6050_ACC_Z);
				temp = mpu6050_sensor.value(MPU6050_TEMP);
				PRINTF("MPU6050 Gyro output X,Y,Z = %d,%d,%d.\n",
						gyro_x, gyro_y, gyro_z);
				PRINTF("MPU6050 Acc output X,Y,Z = %d,%d,%d.\n",
						acc_x, acc_y, acc_z);
				PRINTF("MPU6050 temp output = %d\n", temp);

			}//etimer_expired

			if(etimer_expired(&led_timer)) {
				LED1_OFF;
			}

		}//end while

	PROCESS_END();
}

