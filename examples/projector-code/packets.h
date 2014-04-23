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
 *         Neighbour exercise: packet structures
 * \author
 *         Fredrik Osterlind <fros@sics.se>
 */

#ifndef __PACKETS_H__
#define __PACKETS_H__

#include "contiki.h"
#include "net/rime.h"

#define MAG_CHANNEL 	128 /* Mag Rime channel  */
#define REPORT_CHANNEL 	129 /* Report Rime channel */
/*---------------------------------------------------------------------------*/
#define TYPE_MAG 1 	    /* Mag data message */

struct mag_msg {
  uint16_t type;
  uint8_t  pos_x; 	    /* Our position estimate [1-100]x[1-100] */
  uint8_t  pos_y;
  int16_t  mag_x; 	    /* Magnetometer value - x axis */
  int16_t  mag_y; 	    /* Magnetometer value - y axis */
  int16_t  mag_z; 	    /* Magnetometer value - z axis */
};

/*---------------------------------------------------------------------------*/
#define TYPE_REPORT 2       /* Neighbour report message */

struct report_msg {
  uint16_t   type;
  rimeaddr_t neighbour1;    /* Our top 3 neighbours */
  int        neighbour1_lqi;
  rimeaddr_t neighbour2;
  int        neighbour2_lqi;
  rimeaddr_t neighbour3;
  int        neighbour3_lqi;
};

/*---------------------------------------------------------------------------*/
#define TYPE_ALARM 3        /* Alarm message */

struct alarm_msg {
  uint16_t   type;
  rimeaddr_t bad_neighbour;
};

#endif /* __PACKETS_H__ */
