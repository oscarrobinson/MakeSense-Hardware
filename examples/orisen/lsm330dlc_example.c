/*
 * lsm330dlc_example.c
 *
 *  Created on: 18 Jan 2013
 *  Author: Jagun Kwon
 *  Last updated: 22 Jan 2013
 *
 *  A simple example showing how to use the LSM330DLC accelerometer in Contiki
 *  TODO: Currently doesn not conform to the Contiki sensor structre
 */

#include "contiki.h"
#include "dev/leds.h"
#include "lib/sensors.h"
#include <stdio.h>

#include "include/lsm330dlc_driver.h"	// MEMs driver


static struct etimer adc_timer;

#define FLASH_LED(l) {leds_on(l); clock_delay_msec(50); leds_off(l); clock_delay_msec(50);}
/*---------------------------------------------------------------------------*/
#define SAMPLERATE 1  // in seconds
/*---------------------------------------------------------------------------*/

PROCESS(sensor_process, "simple sensor process");
AUTOSTART_PROCESSES(&sensor_process);
/*---------------------------------------------------------------------------*/

PROCESS_THREAD(sensor_process, ev, data) {
	uint8_t buffer[50];
	uint8_t response;
	int len = 0;
	uint8_t statusreg;  
	uint8_t position=0, old_position=0;
	AxesRaw_t accdata;
	LSM330DLC_AngRateRaw_t gyro_data;


	//uint8_t buf[] = {0,0,0,0,0,0};

	PROCESS_BEGIN();

	// Initialise the sensor module
	LSM330DLC_Init();

// TODO Somehow this was necessary to enable all axes EVEN THOUGH we do this
// in the Init function! On a soft start, we don't have to do this again.
// But on a cold start (i.e., switching off and on the board rather than reset)
// We needed this again.. Investigate why this happens.
/*
  buf[0] = LSM330DLC_CTRL_REG1_A;
  buf[1] = 0x20 | LSM330DLC_ACC_ENABLE_ALL_AXES;
    i2c_transmitinit(LSM330DLC_I2C_ACC_ADDR, 2, buf);
    while(!i2c_transferred());

// Set High resolution mode
  buf[0] = LSM330DLC_CTRL_REG4_A;
  buf[1] = 0x08;
    i2c_transmitinit(LSM330DLC_I2C_ACC_ADDR, 2, buf);
    while(!i2c_transferred());
*/
// These are equivelant to the above code
/*
	LSM330DLC_SetMode_A(LSM330DLC_NORMAL_A);
	LSM330DLC_SetAxis_A(LSM330DLC_X_ENABLE_A | LSM330DLC_Y_ENABLE_A | LSM330DLC_Z_ENABLE_A);
*/

// TODO
//  SENSORS_ACTIVATE(lsm330dlc_sensor);

	while (1) {
//		etimer_set(&adc_timer, CLOCK_SECOND*SAMPLERATE);
//		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&adc_timer));

		//Read ACC Status Reg  
		response = LSM330DLC_GetStatusReg_A(&statusreg);

		printf("Response: 0x%X, statusreg: 0x%X, status&&0x08: 0x%X\n",
			response, statusreg, statusreg && 0x08);

		// 0x08: Status_reg_a: Is there new data available?
		if(response == 1 && (statusreg && 0x08)){
			//getACC Raw data
			response = LSM330DLC_GetAccAxesRaw_A(&accdata);
			if(response==1){    //debug print axies value for MKI109V1 board 
				len = sprintf((char*)buffer, "ACC X=%6d ACC Y=%6d ACC Z=%6d \r\n", accdata.AXIS_X, accdata.AXIS_Y, accdata.AXIS_Z);
				printf("DEBUG: LSM330DLC: %s\n", buffer);

			}
		}
		FLASH_LED(LEDS_BLUE);
	}

//	}

	PROCESS_END();
}

