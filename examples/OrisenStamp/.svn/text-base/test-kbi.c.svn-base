#include <stdio.h>
#include "contiki.h"
#include "dev/leds.h"
#include "kbi_events.h"
#include "RTC.h"

#define FLASH_LED(l) {leds_on(l); clock_delay_msec(50); leds_off(l); clock_delay_msec(50);}

/*---------------------------------------------------------------------------*/
PROCESS(test_kbi_process, "Test kbi");
AUTOSTART_PROCESSES(&test_kbi_process);
/*---------------------------------------------------------------------------*/

static void T9Callback()
{
  //printf("T9 BLIP\n");
  leds_on(LEDS_BLUE);
  clock_delay_msec(1);
  leds_off(LEDS_BLUE);
}

static void T10Callback()
{
  //printf("T10 BLIP\n");
  leds_on(LEDS_YELLOW);
  clock_delay_msec(1);
  leds_off(LEDS_YELLOW);
}

PROCESS_THREAD(test_kbi_process, ev, data)
{ PROCESS_BEGIN();

  leds_off(LEDS_ALL);

	struct RTC_time t = {
		0,		// hundredths
		0,		// tenths
		56,		// seconds
		34,		// minutes
		12,		// hours
		30,		// day
		01,		// month
		15		// year
	};

	RTC_setTime(&t);

  // Alarm can be set with a callback
  // (or this can be NULL)
	kbi_event_setHandler(T9,  T9Callback);
	kbi_event_setHandler(T10, T10Callback);

  int i = 0;

  while (1) {
  	printf("GOING TO SLEEP\n");
		clock_delay_msec(50);

		/* ------------
		 * Go to sleep
     * ------------
		 */
  	*CRM_SLEEP_CNTL = 0x71; 				// hibernate, keep all RAM pages, retain state, don't power GPIO, approx. 2kHz = 16.1uA 

    while((*CRM_STATUS & 0x1) == 0)	// wait for the sleep cycle to complete
    { continue; }
  
    *CRM_STATUS = 1; 								// write 1 to sleep_sync --- this clears the bit (it's a r1wc bit) and powers down

		/* ------------
		 * Wake up again
     * ------------
		 */
  	printf("AWAKE AGAIN\n");
	  for (i = 0; i < 2; i++)
			FLASH_LED(LEDS_ALL);

		PROCESS_YIELD();

		RTC_getTime(&t);
		printf("%02d/%02d/%02d %02d:%02d:%02d.%d%d\n", t.day, t.month, t.year, t.hours, t.minutes, t.seconds, t.tenths, t.hundredths);
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
