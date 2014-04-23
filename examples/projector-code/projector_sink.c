/*
 * Copyright (c) 2007, Swedish Institute of Computer Science.
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
 *
 * This file is part of the Contiki operating system.
 *
 * $Id: example-unicast.c,v 1.5 2010/02/02 16:36:46 adamdunkels Exp $
 */

/**
 * \file
 *         Best-effort single-hop unicast example
 * \author
 *         Adam Dunkels <adam@sics.se>
 */

#include "contiki.h"
#include "net/rime.h"
#include "dev/leds.h"
#include <stdio.h>

#define FLASH_LED(l) {leds_on(l); clock_delay_msec(50); leds_off(l); clock_delay_msec(50);}

#define PRINTADDR(addr) \
  { int __addri;                                            \
    for (__addri = sizeof(addr)-1; __addri > 0; __addri--)  \
      printf("%02X:", addr[__addri]);                       \
    printf("%02X", addr[__addri]);                          \
  }

/*---------------------------------------------------------------------------*/
PROCESS(example_abc_process,     "ABC projector code");
PROCESS(example_unicast_process, "Unicast projector code");
PROCESS(example_trickle_process, "Trickle projector code");

AUTOSTART_PROCESSES(&example_abc_process, &example_unicast_process, &example_trickle_process);
/*---------------------------------------------------------------------------*/
static void
abc_recv(struct abc_conn *c)
{
  printf("abc message received: '%s'\r\n", (char *)packetbuf_dataptr());
  FLASH_LED(LEDS_BLUE);
}

static void
unicast_recv(struct unicast_conn *c, const rimeaddr_t *from)
{ 
  printf("Unicast message received from: ");
  PRINTADDR(from->u8);
  printf(" - %s\r\n", (char *)packetbuf_dataptr());
  FLASH_LED(LEDS_BLUE);
}

static void
trickle_recv(struct trickle_conn *c)
{ 
  printf("Trickle message received: %s\r\n", (char *)packetbuf_dataptr());
  FLASH_LED(LEDS_BLUE);
}

/*---------------------------------------------------------------------------*/
static const struct abc_callbacks abc_call = {abc_recv};
static struct abc_conn abc;
static const struct unicast_callbacks unicast_callbacks = {unicast_recv};
static struct unicast_conn uc;
const static struct trickle_callbacks trickle_call = {trickle_recv};
static struct trickle_conn trickle;
/*---------------------------------------------------------------------------*/

PROCESS_THREAD(example_abc_process, ev, data)
{
  static struct etimer et;

  PROCESS_EXITHANDLER(abc_close(&abc);)
  PROCESS_BEGIN();

  abc_open(&abc, 128, &abc_call);

  // Wait forever
  PROCESS_WAIT_EVENT_UNTIL(0);

  PROCESS_END();
}


PROCESS_THREAD(example_unicast_process, ev, data)
{ 
  rimeaddr_t addr;
  PROCESS_EXITHANDLER(unicast_close(&uc);)
  PROCESS_BEGIN();

  // Set our address to 0.0....0.1
  rimeaddr_copy(&addr, &rimeaddr_null);
  addr.u8[0] = 1;
  rimeaddr_set_node_addr(&addr);

  // And print this out to be sure.
  rimeaddr_copy(&addr, &rimeaddr_node_addr);
  printf("We are the sink node with address: ");
  PRINTADDR(rimeaddr_node_addr.u8);
  printf("\n");

  unicast_open(&uc, 129, &unicast_callbacks);

  // Wait forever
  PROCESS_WAIT_EVENT_UNTIL(0);

  PROCESS_END();
}

PROCESS_THREAD(example_trickle_process, ev, data)
{
  PROCESS_EXITHANDLER(trickle_close(&trickle);)
  PROCESS_BEGIN();

  trickle_open(&trickle, CLOCK_SECOND, 130, &trickle_call);

  // Wait forever
  PROCESS_WAIT_EVENT_UNTIL(0);

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
