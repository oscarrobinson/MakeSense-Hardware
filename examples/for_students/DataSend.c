#include "contiki.h"
#include "net/rime.h"
#include "random.h"
#include "dev/leds.h"
#include <stdio.h>

#define FLASH_LED(l) {leds_on(l); clock_delay_msec(50); leds_off(l); clock_delay_msec(50);}
/*---------------------------------------------------------------------------*/
PROCESS(example_abc_process, "ABC example");
AUTOSTART_PROCESSES(&example_abc_process);

/* Modified by Stephen Hailes @ UCLCS */
static void
abc_recv(struct abc_conn *c)
{
  printf("abc message received: '%s'", (char *)packetbuf_dataptr());
  FLASH_LED(LEDS_BLUE);
}

static char randomChar()
{
   int random = 97+random_rand()%26;
   char randChar = (char)random;
   return randChar;
}


static const struct abc_callbacks abc_call = {abc_recv};
static struct abc_conn abc;

PROCESS_THREAD(example_abc_process, ev, data)
{
  static struct etimer et;

  PROCESS_EXITHANDLER(abc_close(&abc);)

  PROCESS_BEGIN();

  abc_open(&abc, 128, &abc_call);

  while(1) {
    /* Delay 3 seconds*/
    etimer_set(&et, CLOCK_SECOND*3);


    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
    
    char str[3];
    char cOne=randomChar();
    char cTwo=randomChar();
    char cThree=randomChar();
    sprintf(str,"%c%c%c",cOne,cTwo,cThree);

    packetbuf_copyfrom(str, 4);
    abc_send(&abc);
    FLASH_LED(LEDS_GREEN);

    printf("abc message sent\n");
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
