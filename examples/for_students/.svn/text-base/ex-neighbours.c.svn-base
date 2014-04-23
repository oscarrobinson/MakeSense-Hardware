/*
 * Copyright (c) 2009, Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

/**
 * \file
 *         Neighbour exercise: example neighbours
 * \author
 *         Fredrik Osterlind <fros@sics.se>
 */

#include "contiki.h"
#include "net/rime.h"
#include "dev/leds.h"
#include "lib/random.h"
#include <stdio.h>

#include "packets.h"
#include "myneighbours.h"

#define PRINTADDR(addr) \
  { int __addri;                                            \
    for (__addri = sizeof(addr)-1; __addri > 0; __addri--)  \
      printf("%02X:", addr[__addri]);                       \
    printf("%02X", addr[__addri]);                          \
  }

/*---------------------------------------------------------------------------*/
PROCESS(test_neighbours_process, "Test neighbour library");
AUTOSTART_PROCESSES(&test_neighbours_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(test_neighbours_process, ev, data)
{
  static struct etimer et;

  PROCESS_BEGIN();

  /* Init neighbour library */
  myneighbours_init();

  while(1) {
    struct report_msg msg;
    rimeaddr_t addr;
    int lqi;
    int i;

    /* Delay 1 second */
    etimer_set(&et, CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

    /* TODO fix this */
    /* Randomly update a neighbour entry */
    rimeaddr_copy(&addr, &rimeaddr_null);    
    addr.u8[0] = 1 + (random_rand() % 6); /* [1-6] */

    lqi = (int) (random_rand() & 0xFF);		// Invent an LQI value.

    myneighbours_update(&addr, lqi);
    printf("Updated neighbour ");
    PRINTADDR(addr.u8);
    printf(" with lqi = %i\r\n", lqi);

    /* Delay 1 more second */
    etimer_set(&et, CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

    /* Create report msg */
    msg.type = TYPE_REPORT;

    /* Extract top neighbours */
    msg.neighbour1_lqi = myneighbours_best(&msg.neighbour1);
    msg.neighbour2_lqi = myneighbours_second_best(&msg.neighbour2);
    msg.neighbour3_lqi = myneighbours_third_best(&msg.neighbour3);

    /* Print report */
    printf("TOP 3 NEIGHBOURS:\n");
    printf("Neighbour 1 = ");
    PRINTADDR(msg.neighbour1.u8);
    printf(" with lqi = %i\r\n", msg.neighbour1_lqi);

    printf("Neighbour 2 = ");
    PRINTADDR(msg.neighbour2.u8);
    printf(" with lqi = %i\r\n", msg.neighbour2_lqi);

    printf("Neighbour 3 = ");
    PRINTADDR(msg.neighbour3.u8);
    printf(" with lqi = %i\r\n", msg.neighbour3_lqi);
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
