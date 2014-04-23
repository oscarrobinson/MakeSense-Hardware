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
 *         neighbour exercise: top 3 neighbours by LQI (SKELETON)
 * \author
 *         Fredrik Osterlind <fros@sics.se>
 */

#include "contiki.h"
#include "net/rime.h"
#include "dev/leds.h"
#include "random.h"
#include <stdio.h>

#include "packets.h"
#include "myneighbours.h"


#define FLASH_LED(l) {leds_on(l); clock_delay_msec(50); leds_off(l); clock_delay_msec(50);}

#define PRINTADDR(addr) \
  { int __addri;                                            \
    for (__addri = sizeof(addr)-1; __addri > 0; __addri--)  \
      printf("%02X:", addr[__addri]);                       \
    printf("%02X", addr[__addri]);                          \
  }

/*---------------------------------------------------------------------------*/
PROCESS(report_neighbours_process, "Report neighbours");
PROCESS(send_mag_process, "Broadcast magnetometer");
AUTOSTART_PROCESSES(&report_neighbours_process, &send_mag_process);
/*---------------------------------------------------------------------------*/
static void
recv_mag(struct broadcast_conn *c, const rimeaddr_t *from)
{
  struct mag_msg msg;

  if (packetbuf_datalen() != sizeof(struct mag_msg)) {
    /* Bad size */
    return;
  }

  packetbuf_copyto(&msg); /* Copy packet data */
  if (msg.type != TYPE_MAG) {
    /* Bad type */
    return;
  }

  /* We received a broadcast message from a neighbour.
   * Update our neighbour list using neighbours LQI */
  printf("TODO #9 Update our neighbour list\n"); /* ex-neighbours.c */
  FLASH_LED(LEDS_GREEN);
}
/*---------------------------------------------------------------------------*/
static void
recv_report(struct unicast_conn *c, const rimeaddr_t *from)
{
  /* We should never receive any reports, only the projector should do that! */
}
/*---------------------------------------------------------------------------*/
static const struct broadcast_callbacks broadcast_call = {recv_mag};
static struct broadcast_conn mag_broadcast;
static const struct unicast_callbacks unicast_call = {recv_report};
static struct unicast_conn report_unicast;
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(send_mag_process, ev, data)
{
  static struct etimer et;
  static int xpos, ypos;

  PROCESS_EXITHANDLER(broadcast_close(&mag_broadcast);)

  PROCESS_BEGIN();

  /* Open broadcast channel on MAG_CHANNEL */
  printf("TODO #1 Open broadcast channel on MAG_CHANNEL\n"); /* ex-broadcast-lqi.c */

  /* Send mag every 5 seconds */
  etimer_set(&et, 5*CLOCK_SECOND);

  xpos = random_rand() % 100;
  ypos = random_rand() % 100;
  printf("TODO #3 Enter an estimate of our (X,Y) coordinates [0-100]: (%d,%d)\n", xpos, ypos);

  while(1) {
    struct mag_msg msg;

    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
    etimer_reset(&et);

    msg.type = TYPE_MAG;

    /* Our position */
    msg.pos_x = xpos;
    msg.pos_y = ypos;

    /* Sample mag */
    msg.mag_x = 0;
    msg.mag_y = 0;
    msg.mag_z = 0;
    printf("TODO #4 Sample mag sensor: [%4d, %4d, %4d]\n", msg.mag_x, msg.mag_y, msg.mag_z); /* ex-mag.c */

    /* Broadcast mag and position message */
    packetbuf_copyfrom((char*)&msg, sizeof(struct mag_msg));
    printf("TODO #2 Send broadcast message\n"); /* ex-broadcast-lqi.c */
    FLASH_LED(LEDS_BLUE);
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(report_neighbours_process, ev, data)
{
  static struct etimer et;
  PROCESS_EXITHANDLER(unicast_close(&report_unicast);)

  PROCESS_BEGIN();

  /* Initialize neighbour library */
  printf("TODO #8 Initialize neighbour library\n"); /* ex-neighbours.c */

  /* Open unicast channel on REPORT_CHANNEL */
  printf("TODO #5 Open unicast channel on REPORT_CHANNEL\n"); /* /examples/rime/example-unicast.c */

  /* Delay 2.5 seconds */
  etimer_set(&et, 5*CLOCK_SECOND/2);
  PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

  /* Send neighbour report every 5 seconds */
  etimer_set(&et, 5*CLOCK_SECOND);

  while(1) {
    struct report_msg msg;
    rimeaddr_t projector;

    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
    etimer_reset(&et);

    /* Create report */
    msg.type = TYPE_REPORT;

    /* Extract top neighbours sorted by LQI */
    rimeaddr_copy(&msg.neighbour1, &rimeaddr_null); msg.neighbour1_lqi = 0;
    rimeaddr_copy(&msg.neighbour2, &rimeaddr_null); msg.neighbour2_lqi = 0;
    rimeaddr_copy(&msg.neighbour3, &rimeaddr_null); msg.neighbour3_lqi = 0;
    printf("TODO #10 Extract top 3 neighbours:\n  ");
    PRINTADDR(msg.neighbour1.u8);
    printf(" lqi = %i\n  ", msg.neighbour1_lqi);
    PRINTADDR(msg.neighbour2.u8);
    printf(" lqi = %i\n  ", msg.neighbour2_lqi);
    PRINTADDR(msg.neighbour3.u8);
    printf(" lqi = %i\n",   msg.neighbour3_lqi);	/* ex-neighbours.c */

    /* Enter projector address */
    rimeaddr_copy(&projector, &rimeaddr_null);
    printf("TODO #6 Enter projector Rime address: ");
    PRINTADDR(projector.u8);
    printf("\n");

    /* Send unicast message with neighbour information to projector */
    packetbuf_copyfrom((char*)&msg, sizeof(struct report_msg));
    printf("TODO #7 Send unicast message to projector\n"); /* /examples/rime/example-unicast.c */

    FLASH_LED(LEDS_BLUE);
    FLASH_LED(LEDS_BLUE);
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
