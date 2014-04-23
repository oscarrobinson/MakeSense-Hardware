#include <stdio.h>
#include "contiki.h"
#include "dev/leds.h"
#include "io-pins.h"

#define FLASH_LED(l) {leds_on(l); clock_delay_msec(50); leds_off(l); clock_delay_msec(50);}


/*
 * Pin    Initial function			Additional list										Arrangement
 * ---    ----------------			---------------							------------------------
 * T1			SDA  (I2C)							GPIO13										T15									 T20
 * T2			SCL  (I2C)							GPIO12										T16									 T19
 * T3			GPIO09									TMR1											T7									 T18
 * T4			GPIO08									TMR0											T8									 T17
 * T5			TX   (UART2)						GPIO18										T6									 T11
 * T6			RX   (UART2)						GPIO19										T5									 T12
 * T7			RTS  (UART2)						GPIO21										T1   T3					T9   T13
 * T8			CTS  (UART2)						GPIO20										T2   T4					T10  T14
 * T9			KBI7										GPIO29
 * T10		KBI6										GPIO28										ADC0								 GND
 * T11		SS   (SPI)							GPIO04										ADC1								 VCC
 * T12		MISO (SPI)							GPIO05										RTS									 GND
 * T13		MOSI (SPI)							GPIO06										CTS									 VIN
 * T14		SCK  (SPI)							GPIO07										RX									 SS
 * T15		ADC0										GPIO30										TX									 MISO
 * T16		ADC1										GPIO31										SDA  GPIO9			KBI7 MOSI
 * T17		VIN																								SCL  GPIO8			KBI6 SCK
 * T18		GND
 * T19		VCC
 * T20		GND
 */

/*---------------------------------------------------------------------------*/
PROCESS(test_iopins_process, "Test IO Pins");
AUTOSTART_PROCESSES(&test_iopins_process);
/*---------------------------------------------------------------------------*/

PROCESS_THREAD(test_iopins_process, ev, data)
{ PROCESS_BEGIN();

  static struct etimer et;

  leds_off(LEDS_ALL);

  int i = 0;

  // Set T4 as an output
  ioPins_configurePin(4, USEGPIO, OUTPUT, NOPULLUP, HYSTERESIS_OFF);

  while (1) {
  	static uint32_t v = 0;

  	etimer_set(&et, CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

		printf("\n");

		// T4 has been configured as a GPIO output and T3 as an input.
    // If you wire T3 and T4 together then the changes made in T4
    // should be reflected in T3 - and they should tick 0->1->0...
    //
		ioPins_setValue(4, v);
		printf("Value(4) = output %d\n", v);

		v = ioPins_getValue(3);
		printf("Value(3) = input  %d\n", v);

    v = (v == 0) ? 1 : 0;

    // Read ADC0, ADC1 and the battery reference value ADC9
    //
		printf("ADC0: %04u\r\n", ioPins_getValue(15));
		printf("ADC1: %04u\r\n", ioPins_getValue(16));

    // Get the battery voltage
		printf("BATT: %04u\r\n", ioPins_getBatt());
  }


  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
