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
 *         neighbour exercise: top 3 neighbours by RSSI (SKELETON)
 * \author
 *         Fredrik Osterlind <fros@sics.se>
 */

#include "contiki.h"
#include "net/rime.h"
#include "dev/leds.h"
#include "lib/sensors.h"
#include "hmc5883l.h"
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
  myneighbours_update(from, (int)packetbuf_attr(PACKETBUF_ATTR_LINK_QUALITY));
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
  SENSORS_ACTIVATE(hmc5883l_sensor);

  /* Open broadcast channel on LIGHT_CHANNEL */
  broadcast_open(&mag_broadcast, MAG_CHANNEL, &broadcast_call);

  /* Send light every 5 seconds */
  etimer_set(&et, 5*CLOCK_SECOND);

  xpos = random_rand() % 100;
  ypos = random_rand() % 100;

  while(1) {
    struct mag_msg msg;

    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
    etimer_reset(&et);

    msg.type = TYPE_MAG;

    /* Our position */
    msg.pos_x = xpos; /* TODO Enter your estimated room position */
    msg.pos_y = ypos;

    // Must latch the data first before getting mag_x, y, z
    hmc5883l_sensor.value(HMC5883L_LATCH_DATA);
    msg.mag_x = hmc5883l_sensor.value(HMC5883L_MAG_X_RAW);
    msg.mag_y = hmc5883l_sensor.value(HMC5883L_MAG_Y_RAW);
    msg.mag_z = hmc5883l_sensor.value(HMC5883L_MAG_Z_RAW);

    /* Broadcast light and position message */
    packetbuf_copyfrom((char*)&msg, sizeof(struct mag_msg));
    broadcast_send(&mag_broadcast);
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
  myneighbours_init();

  /* Open unicast channel on REPORT_CHANNEL */
  unicast_open(&report_unicast, REPORT_CHANNEL, &unicast_call);

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
    msg.neighbour1_lqi = myneighbours_best(&msg.neighbour1);
    msg.neighbour2_lqi = myneighbours_second_best(&msg.neighbour2);
    msg.neighbour3_lqi = myneighbours_third_best(&msg.neighbour3);
    printf("My top 3 neighbours:\n  ");
    PRINTADDR(msg.neighbour1.u8);
    printf(" lqi = %i\n  ", msg.neighbour1_lqi);
    PRINTADDR(msg.neighbour2.u8);
    printf(" lqi = %i\n  ", msg.neighbour2_lqi);
    PRINTADDR(msg.neighbour3.u8);
    printf(" lqi = %i\n",   msg.neighbour3_lqi);

    /* Enter projector address */
    rimeaddr_copy(&projector, &rimeaddr_null);
    projector.u8[0] = 1;
    printf("The projector Rime address is: ");
    PRINTADDR(projector.u8);
    printf("\n");

    /* Send unicast message with neighbour information to projector */
    packetbuf_copyfrom((char*)&msg, sizeof(struct report_msg));
    unicast_send(&report_unicast, &projector);

    FLASH_LED(LEDS_BLUE);
    FLASH_LED(LEDS_BLUE);
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
