#include <stdio.h>
#include "contiki.h"
#include "contiki-uart.h"
#include "dev/leds.h"
#include <mc1322x.h>
#include <board.h>
#include "net/rime.h"


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


  static void abc_recv(struct abc_conn *c)
  {
    printf("abc message received: '%s'\n", (char *)packetbuf_dataptr());
    FLASH_LED(LEDS_BLUE);
  }
  static const struct abc_callbacks abc_call = {abc_recv};
  static struct abc_conn abc;
PROCESS(example_abc_process, "send data");
PROCESS_THREAD(example_abc_process, ev, data)
{
  static struct etimer et;
  PROCESS_EXITHANDLER(abc_close(&abc);)
  PROCESS_BEGIN();
  	

  char c = *((char*)data);
  int length = 0;
  while(c!='-'){
  	length++;
    c=*((char*)data+length);
  }
  int i=0;
  char broadcast[length];
  for(i=0;i<length;i++){
  	broadcast[i]=*((char*)data+i);
  }

  abc_open(&abc, 128, &abc_call);
 int x=0;
 for(x=0;x<length;x++){
	printf("%c",broadcast[x]);
 }
 printf("\n");
 packetbuf_copyfrom(broadcast, length);
 abc_send(&abc);
 FLASH_LED(LEDS_GREEN);

 printf("abc message sent\n");

  PROCESS_END();
}


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
  int counter = 0;
  int size = 30;
  char data[size];
  abc_open(&abc, 128, &abc_call);
  while (1) {
    char ch = uart2_getc();
	//printf("%c\n",ch);
	
	if (ch=='\n'){
		int i=0;
		for(i=counter;i<size;i++){
			data[i]='-';
		}
		//sprintf(toSend,"ab");			
		counter=0;
		if(data[size-1]=='-'){
			process_start (&example_abc_process, data);
			FLASH_LED(LEDS_GREEN);
		}		
	}
    else{
		data[counter]=ch;
		counter++;
 	 }
  }
  PROCESS_END();	
}
/*---------------------------------------------------------------------------*/
