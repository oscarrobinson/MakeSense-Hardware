/*
 (c) 2009, Swedish Institute of Computer Science.
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
 *         Neighbour exercise: example print magnetometer values
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

/*---------------------------------------------------------------------------*/
PROCESS(sample_mag_process, "Sample magnetometer sensor");
AUTOSTART_PROCESSES(&sample_mag_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(sample_mag_process, ev, data)
{
  static struct etimer et;

  PROCESS_BEGIN();
  SENSORS_ACTIVATE(hmc5883l_sensor);

  /* Sample every 2 seconds */
  etimer_set(&et, 2*CLOCK_SECOND);

  while(1) {
    struct mag_msg msg;

    /* Wait for timer */
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
    /* Reset timer:
     * It will expire exactly 2 seconds after last expiration */
    etimer_reset(&et);

    msg.type = TYPE_MAG;

    /* Our position */
    msg.pos_x = 0; /* TODO Estimate X coordinate [1-100] */
    msg.pos_y = 0; /* TODO Estimate Y coordinate [1-100] */

    // Must latch the data first before getting mag_x, y, z
    hmc5883l_sensor.value(HMC5883L_LATCH_DATA);
    msg.mag_x = hmc5883l_sensor.value(HMC5883L_MAG_X_RAW);
    msg.mag_y = hmc5883l_sensor.value(HMC5883L_MAG_Y_RAW);
    msg.mag_z = hmc5883l_sensor.value(HMC5883L_MAG_Z_RAW);

    /* Print position and mag */
    printf("pos=(%u,%u) mag=[%4d, %4d, %4d]\n", msg.pos_x, msg.pos_y, msg.mag_x, msg.mag_y, msg.mag_z);
    FLASH_LED(LEDS_GREEN);
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
