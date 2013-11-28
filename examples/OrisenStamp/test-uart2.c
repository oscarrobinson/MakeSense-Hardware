#include <stdio.h>
#include "contiki.h"
#include "contiki-uart.h"
#include "dev/leds.h"
#include <mc1322x.h>
#include <board.h>


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
PROCESS(test_uart2_process, "Test UART2");
AUTOSTART_PROCESSES(&test_uart2_process);
/*---------------------------------------------------------------------------*/

PROCESS_THREAD(test_uart2_process, ev, data)
{
  PROCESS_BEGIN();

  static struct etimer et;
  static char *str = "Hello world\n";
  static int   strlen = 13;
  static int i, j;

  leds_off(LEDS_ALL);

  // Set UART2 to 115200 baud
	uart2_init(INC, MOD, SAMP);

  // Write to the serial - but also allow for a physical loopback
  // through wired connection between UART2 TX and UART2 RX pins
  //
  // Tests our TX and RX
  //
  printf("Sending ten hello worlds\n");
  for (j = 0; j < 10; j++) {
    etimer_set(&et, CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

		for (i = 0; i < strlen; i++) {
			uart2_putc(str[i]);
		
			while (uart2_can_get()) {
				printf("%c", uart2_getc());
			}
		}

		FLASH_LED(LEDS_BLUE);
		FLASH_LED(LEDS_BLUE);
  }

  // Now loopback out input on UART2 to our output on UART2
  // This allows testing of TX on the other side by
  // running minicom, typing something in and seeing whether
  // the input is reflected
  //
  printf("Loopback\n");
  while (1) {
		while (uart2_can_get()) {
    	char ch;
			ch = uart2_getc();
			uart2_putc(ch);
			printf("%c", ch);
			if (ch == '\r') {
			  FLASH_LED(LEDS_BLUE);
      }
		}
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
