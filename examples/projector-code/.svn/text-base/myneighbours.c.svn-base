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
 *         Neighbour exercise: neighbourhood library example
 * \author
 *         Fredrik Osterlind <fros@sics.se>
 */

#include "contiki.h"
#include "net/rime.h"

#include "myneighbours.h"

#include <stdio.h>

#define PRINTADDR(addr) \
  { int __addri;                                            \
    for (__addri = sizeof(addr)-1; __addri > 0; __addri--)  \
      printf("%02X:", addr[__addri]);                       \
    printf("%02X", addr[__addri]);                          \
  }


struct myneighbour {
  rimeaddr_t 	addr; 	/* 2b */
  int16_t 	lqi; 	/* 2b */
};

#define MAX_NEIGHBOURS 20
static struct myneighbour neighbours[MAX_NEIGHBOURS]; /* Neighbour array */

/*---------------------------------------------------------------------------*/
void
myneighbours_init(void)
{
  int i;
  for (i=0; i < MAX_NEIGHBOURS; i++) {
    rimeaddr_copy(&neighbours[i].addr, &rimeaddr_null);
  }
}
/*---------------------------------------------------------------------------*/
void
myneighbours_update(const rimeaddr_t *neighbour, int16_t lqi)
{
  int i, j;

  /* Locate neighbour */
  for (i=0; i < MAX_NEIGHBOURS; i++) {
    if ( rimeaddr_cmp(&neighbours[i].addr, neighbour)
      || rimeaddr_cmp(&neighbours[i].addr, &rimeaddr_null)) {
      break;
    }
  }

  if (i>=MAX_NEIGHBOURS) {
    /* Neighbour list is full! */
    return;
  }

  if (rimeaddr_cmp(&neighbours[i].addr, neighbour)) {
    /* Update existing neighbour */
    neighbours[i].lqi = lqi;

    PRINTADDR(neighbours[i].addr.u8);
    printf(" lqi = %i UPDATED\r\n", neighbours[i].lqi);
    return;
  }

  /* Add new neighbour */
  rimeaddr_copy(&neighbours[i].addr, neighbour);
  neighbours[i].lqi = lqi;

  PRINTADDR(neighbours[i].addr.u8);
  printf(" lqi = %i ADDED\r\n", neighbours[i].lqi);
}

/*---------------------------------------------------------------------------*/
static int16_t
get_neighbour_lqi(int rank, rimeaddr_t *neighbour)
{
  int i, idx_1=-1, idx_2=-1, idx_3=-1;

  /* Extract top 3 neighbours */
  for (i=0; i < MAX_NEIGHBOURS; i++) {
    if (rimeaddr_cmp(&neighbours[i].addr, &rimeaddr_null)) {
      /* End of list */
      break;
    }

    /* Sort */
    if (idx_1 < 0
        || neighbours[i].lqi > neighbours[idx_1].lqi) {
      idx_3 = idx_2;
      idx_2 = idx_1;
      idx_1 = i;
    } else if (idx_2 < 0
        || neighbours[i].lqi > neighbours[idx_2].lqi) {
      idx_3 = idx_2;
      idx_2 = i;
    } else if (idx_3 < 0
        || neighbours[i].lqi > neighbours[idx_3].lqi) {
      idx_3 = i;
    }
  }

  /* Rank: [1-3] */
  if (rank == 1) {
    i = idx_1;
  } else if (rank == 2) {
    i = idx_2;
  } else if (rank == 3) {
    i = idx_3;
  } else {
    /*printf("error: bad rank: %i\n", rank);*/
    rimeaddr_copy(neighbour, &rimeaddr_null);
    return 0;
  }
  if (i < 0) {
    rimeaddr_copy(neighbour, &rimeaddr_null);
    return 0;
  }
  rimeaddr_copy(neighbour, &neighbours[i].addr);
  return neighbours[i].lqi;
}

/*---------------------------------------------------------------------------*/
int16_t
myneighbours_best(rimeaddr_t *neighbour)
{
  return get_neighbour_lqi(1, neighbour);
}
/*---------------------------------------------------------------------------*/
int16_t
myneighbours_second_best(rimeaddr_t *neighbour)
{
  return get_neighbour_lqi(2, neighbour);
}
/*---------------------------------------------------------------------------*/
int16_t
myneighbours_third_best(rimeaddr_t *neighbour)
{
  return get_neighbour_lqi(3, neighbour);
}
/*---------------------------------------------------------------------------*/
