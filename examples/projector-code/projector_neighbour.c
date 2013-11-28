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
 *         neighbour exercise: sink program
 * \author
 *         Fredrik Osterlind <fros@sics.se>
 */


#include "contiki.h"
#include "net/rime.h"
#include "dev/leds.h"
#include "lib/sensors.h"
#include "hmc5883l.h"
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
PROCESS(sink_process, "Report neighbours");
AUTOSTART_PROCESSES(&sink_process);
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
   * Print it */
  PRINTADDR(from->u8);
  printf(" MAG pos=(%u,%u) mag=[%d,%d,%d]\n", msg.pos_x, msg.pos_y, msg.mag_x, msg.mag_y, msg.mag_z);
  FLASH_LED(LEDS_GREEN);
}
/*---------------------------------------------------------------------------*/
static void
recv_report(struct unicast_conn *c, const rimeaddr_t *from)
{ 
  if (packetbuf_datalen() == sizeof(struct report_msg)) {
    struct report_msg msg;
    packetbuf_copyto(&msg); /* Copy packet data */
    if (msg.type != TYPE_REPORT) {
      /* Bad type */
      return;
    }

    PRINTADDR(from->u8);
    printf(" REPORT ");
    PRINTADDR(msg.neighbour1.u8);
    printf(" lqi=%i ",  msg.neighbour1_lqi);
    PRINTADDR(msg.neighbour2.u8);
    printf(" lqi=%i ",  msg.neighbour2_lqi);
    PRINTADDR(msg.neighbour3.u8);
    printf(" lqi=%i\n", msg.neighbour3_lqi);

    FLASH_LED(LEDS_GREEN);
    FLASH_LED(LEDS_GREEN);

    return;
  }

  if (packetbuf_datalen() == sizeof(struct alarm_msg)) {
    /* TODO Alarm messages? */
    struct alarm_msg msg;
    packetbuf_copyto(&msg); /* Copy packet data */
    if (msg.type != TYPE_ALARM) {
      /* Bad type */
      return;
    }

    PRINTADDR(from->u8);
    printf(" ALARM ");
    PRINTADDR(msg.bad_neighbour.u8);

    FLASH_LED(LEDS_GREEN);
    FLASH_LED(LEDS_GREEN);
    FLASH_LED(LEDS_GREEN);
    return;
  }
}
/*---------------------------------------------------------------------------*/
static const struct broadcast_callbacks broadcast_call = {recv_mag};
static struct broadcast_conn mag_broadcast;
static const struct unicast_callbacks unicast_call = {recv_report};
static struct unicast_conn report_unicast;
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(sink_process, ev, data)
{
  rimeaddr_t addr;

  PROCESS_EXITHANDLER(
      broadcast_close(&mag_broadcast);
      unicast_close(&report_unicast);)

  PROCESS_BEGIN();

  // Set our address to 0.0....0.1
  rimeaddr_copy(&addr, &rimeaddr_null);
  addr.u8[0] = 1;
  rimeaddr_set_node_addr(&addr);

  broadcast_open(&mag_broadcast, MAG_CHANNEL, &broadcast_call);
  unicast_open(&report_unicast, REPORT_CHANNEL, &unicast_call);

  printf("I AM THE SINK\n");

  PROCESS_WAIT_EVENT_UNTIL(0);

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
