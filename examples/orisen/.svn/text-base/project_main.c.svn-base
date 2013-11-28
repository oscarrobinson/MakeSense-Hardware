/*
 * project_main.c
 *
 *  Created on: 31 Aug 2011
 *      Author: mcphillips
 *
 *  Updated on: 20 Aug 2012 by Jagun Kwon (j.kwon@cs.ucl.ac.uk)
 *  Refactored to comply with Contiki sensor interface structure
 */

#include "contiki.h"

//define COOJASIM
#ifndef COOJASIM
#include "utils.h"
#include "adc.h"
#endif

#include "dev/leds.h"
#include <stdio.h> /* For printf() */
#include <string.h>

#include "lib/random.h"
#include "net/rime.h"
#include "net/rime/collect.h"
#include "dev/button-sensor.h"
#include "net/rime/rimeaddr.h"
#include "net/netstack.h"
#include "bmp085.h"
//#include "dev/sc16is750.h"	//gps
#include "SHT2x.h"
#include "co-ads1015.h"
#include "MAX5479.h"
#include "uSDcard.h"
#include "cfs/cfs.h"
#include "i2c.h"
#include "board/OrisenPrime.h"
#include "clock.h"
#include "uart1.h"
#include <math.h>	// for floating point operation
#include "common.h"

#define APPS
//#include "shell.h"
#include "shell-time.h"


#if TIMESYNCH_CONF_ENABLED
#include "net/rime/timesynch.h"
#endif

static struct collect_conn tc;

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
unsigned long packetseq = 0;

struct sendpacket packettosend;

// TODO Now in the dev driver
//struct co_ads1015_data co_adc_data;
//uint8_t co_ads1015_data_buffer[15];

// TODO not being used?
//uint8_t sht25_data_buffer[15];

// TODO moved to dev driver
//bmp085_t bmp085;

uint8_t numRecords=0;

#define SDCARD
/*---------------------------------------------------------------------------*/
static void recv(const rimeaddr_t *originator, uint8_t seqno, uint8_t hops) {
	struct sendpacket *rsp;
	struct Sensor_Data rsd;
	int i;

}
/*---------------------------------------------------------------------------*/
static const struct collect_callbacks callbacks = { recv };
/*---------------------------------------------------------------------------*/


/* TODO: now obsolete - this is done in the device driver!
static void getTempRH(void) {
	u16_t uT, uRH;
	float translatedT, translatedRH;

	uT=SHT2x_GetTemp();
	translatedT = SHT2x_CalcTemperatureC(uT);
	packettosend.sd.sht25_temp = (u16_t)(translatedT*10);

	uRH = SHT2x_GetRH();
	translatedRH = SHT2x_CalcRH(uRH);
	packettosend.sd.RHData = (u16_t)(translatedRH*10);

	//PRINTF("temp=%d, RH=%d\r\n", (u16_t)(translatedT*10), (u16_t)(translatedRH*10));
}
*/

/* TODO - moved to the dev driver - obsolete now!
static void getPress(){
	u16_t rawTemp;
	u32_t rawPres, translatedT, translatedP;
	rawTemp = bmp085_get_ut();
	rawPres = bmp085_get_up();
	translatedT = bmp085_get_temperature((unsigned long)rawTemp);
	translatedP = bmp085_get_pressure(rawPres);
	//PRINTF("temp=%d, press=%d\r\n\r\n", translatedT, translatedP);
	packettosend.sd.bmp085_temp = translatedT;
	packettosend.sd.bmp085_press = translatedP;
}
*/

/* TODO - moved to the dev driver
static void getCOadc(void) {
	co_ads1015_start_adc();
	co_ads1015_get_data(CO_ADS1015_AIN0, co_ads1015_data_buffer);
					//set_fletcher_ck(&co_ads1015_data_buffer[2],11);
	co_ads1015_decode_data(&co_adc_data, co_ads1015_data_buffer);
	//PRINTF("ADC0 value = %d\n", co_adc_data.adc0);
	packettosend.sd.ADCValue = co_adc_data.adc0;
}
*/

static void testcfs(void){
	char buf[]="123\n", readbuf[10];
	const char filename[]="test.dat";
	char bw;
	cfs_open("test.dat", CFS_WRITE );
	printf("file opened\r\n");
	cfs_write(1, (const char *)buf, sizeof(buf));
	printf("data written\r\n");
	cfs_close(1);
	printf("file closed\r\n");
	cfs_open(filename, CFS_READ);
	printf("file opened\r\n");
	cfs_read(1, readbuf, sizeof(buf));
	printf("file read %s\r\n", readbuf);
	cfs_close(1);
	printf("file close\r\n");
}

/*********************************/
static void sensorinit(void) {

	/****************************************************/
	/*******      Start of sensor buffers      **********/
	/****************************************************/
/*
		//what are these parameters fNEWor - we can do without?
		co_ads1015_data_buffer[0] = 0xBF;
		co_ads1015_data_buffer[1] = 0x3F;
		co_ads1015_data_buffer[2] = 0x06;
		co_ads1015_data_buffer[3] = 0x01;
		co_ads1015_data_buffer[4] = 0x0F;
		co_ads1015_data_buffer[12] = 0x00;
*/

	/* Initialise GPS and clear buffer */
	//un-comment and run this code below just once for each CO board. It writes 0xFF to the nvm of the digital pot
	// The unit is now set to power up on startup
	//max5479_set_wiper_a_NVREG(0xff);
	//PRINTF("Set wiper a to 0xff\n");

/* TODO - moved to dev driver
	co_ads1015_init(CO_ADS1015_AIN0,CO_ADS1015_GAIN2);
*/
	// Instead, we use this macro to initialise the sensor
//	SENSORS_ACTIVATE(co_sensor);

/* TODO - moved to dev driver
	SHT2x_SoftReset();
*/

/* TODO - moved to dev driver
	//bmp085_get_calib(&bmp085_calib_buffer);
	bmp085.bus_write = &bmp085_I2C_write;
	bmp085.bus_read = &bmp085_I2C_read;
	bmp085.oversampling_setting = 0;
	bmp085.delay_msec = &bmp085_delay;

	bmp085_init(&bmp085);
*/
#ifdef SDCARD
	uSDcard_init();
	printf("disk initialised\r\n");
	testcfs();
#endif
	clock_delay_msec(200);
}


/*---------------------------------------------------------------------------*/
PROCESS(COmon_process, "CO monitoring process");
AUTOSTART_PROCESSES(&COmon_process);
/*---------------------------------------------------------------------------*/

PROCESS_THREAD(COmon_process, ev, data) {
	char buff[50];

	PROCESS_BEGIN();
		i2c_enable();


		sensorinit();
		SENSORS_ACTIVATE(co_sensor);
		SENSORS_ACTIVATE(sht2x_sensor);
		SENSORS_ACTIVATE(bmp085_sensor);


		random_init(rimeaddr_node_addr.u8[0]);
		collect_open(&tc, 130, COLLECT_ROUTER, &callbacks);

		etimer_set(&adc_timer, CLOCK_SECOND);

		//etimer_set(&gps_timer, CLOCK_SECOND * GPSSAMPLERATE);


		cfs_open("log.dat", CFS_APPEND);
		PRINTF("file opened\r\n");
		while(1) {

			PROCESS_WAIT_EVENT();

			if(etimer_expired(&adc_timer)) {

				etimer_set(&adc_timer, CLOCK_SECOND*SAMPLERATE);
				LED1_ON;
				etimer_set(&led_timer, CLOCK_SECOND*0.5);

				// TODO Now, instead of calling getCOadc();
				// we use the following expression
				packettosend.sd.ADCValue = co_sensor.value(0);

				// TODO Instead of calling getTempRH();
				// we use the following expressions
				packettosend.sd.sht25_temp = sht2x_sensor.value(SHT2X_TEMP); 
				packettosend.sd.RHData = sht2x_sensor.value(SHT2X_HUMIDITY); 

				// TODO Instead of calling getPress();
				// we use the following expressions
				packettosend.sd.bmp085_temp = bmp085_sensor.value(BMP085_TEMP);
				packettosend.sd.bmp085_press = bmp085_sensor.value(BMP085_PRESSURE);


				packettosend.c_packetno = packetseq;
#if TIMESYNCH_CONF_ENABLED
				//packettosend.send_time = timesynch_time();
				packettosend.send_time = clock_seconds();
#else
				packettosend.send_time = clock_seconds();
#endif

				sprintf(buff, "%03d, %d.%05d, %d, %d, %d, %d, %d\r\n", packettosend.c_packetno,
						packettosend.send_time, timesynch_time(), packettosend.sd.ADCValue, packettosend.sd.sht25_temp,
						packettosend.sd.RHData, packettosend.sd.bmp085_temp, packettosend.sd.bmp085_press);

				//PRINTF("Base Time:%d: ", timesynch_time());
				PRINTF("%s", buff);

				if (numRecords >=10) {
					cfs_close(1);
					PRINTF("file closed\r\n");
				}
				else{
					//cfs_write(1, "+sensor=", 8);
					//PRINTF("write %s, %d bytes\r\n", buff, strlen(buff));
					cfs_write(1, buff, strlen(buff));
					numRecords++;

				}


				//

				packetseq++;

				packetbuf_clear();
				packetbuf_copyfrom(&packettosend, sizeof(packettosend)+1);
				collect_send(&tc, sizeof(packettosend)+1);

			}//etimer_expired

			if(etimer_expired(&led_timer)) {
				LED1_OFF;
			}

		}//end while

		PROCESS_END();
	}

