#include <stdio.h>
#include "contiki.h"
#include "button-sensors.h"
#include "dev/leds.h"

/*---------------------------------------------------------------------------*/
PROCESS(test_button_process, "Test button");
AUTOSTART_PROCESSES(&test_button_process);
/*---------------------------------------------------------------------------*/
static uint8_t active;
PROCESS_THREAD(test_button_process, ev, data)
{ PROCESS_BEGIN();
  SENSORS_ACTIVATE(button_sensor);

  leds_off(LEDS_BLUE | LEDS_GREEN | LEDS_YELLOW);

  while(1) {
    PROCESS_WAIT_EVENT_UNTIL(ev == sensors_event && data == &button_sensor);
    leds_on(LEDS_BLUE | LEDS_GREEN | LEDS_YELLOW);
    PROCESS_WAIT_EVENT_UNTIL(ev == sensors_event && data == &button_sensor);
    leds_off(LEDS_BLUE | LEDS_GREEN | LEDS_YELLOW);
  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
