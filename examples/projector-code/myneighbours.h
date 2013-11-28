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

#ifndef __MYNEIGHBOURS_H__
#define __MYNEIGHBOURS_H__

#include "contiki.h"
#include "net/rime.h"

/**
 * Initialise library.
 */
void myneighbours_init(void);

/**
 * Register new neighbour, or update existing neighbour.
 */
void myneighbours_update(const rimeaddr_t *neighbour, int16_t lqi);

/**
 * Returns the neighbour with the strongest LQI.
 */
int16_t myneighbours_best(rimeaddr_t *neighbour);

/**
 * Returns the neighbour with the second strongest LQI.
 */
int16_t myneighbours_second_best(rimeaddr_t *neighbour);

/**
 * Returns the neighbour with the third strongest LQI.
 */
int16_t myneighbours_third_best(rimeaddr_t *neighbour);

#endif /* __MYNEIGHBOURS_H__ */
