#include "contiki.h"
#include "lib/include/gpio.h"
#include "dev/leds.h"
#include "mc1322x.h"
#include "board.h"
#include <stdio.h>

PROCESS(hello_world_process, "Hello world process");
AUTOSTART_PROCESSES(&hello_world_process);

#define FLASH_LED(l) {leds_on(l); clock_delay_msec(50); leds_off(l); clock_delay_msec(50);}

PROCESS_THREAD(hello_world_process, ev, data)
{
  static struct etimer et;

  PROCESS_BEGIN();

  while (1) {
    printf("Hello world\n");

    FLASH_LED(LEDS_BLUE);
    FLASH_LED(LEDS_GREEN);
    FLASH_LED(LEDS_YELLOW);

    etimer_set(&et, 1*CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
  }
  
  PROCESS_END();
}


