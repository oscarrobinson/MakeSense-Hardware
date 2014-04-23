/*
 * Copyright (c) 2011, Graeme McPhillips <g.mcphillips@cs.ucl.ac.uk>
 *
 * Test code for mc1322x Bracelet functions
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
 * $Id$
 */

#ifndef MAX5479_H_
#define MAX5479_H_

//#include <stdbool.h>

#define MAX5479_I2C_ADDR				((0x50)>>1)
#define MAX5479_WIPER_A_VREG			(0x11)			//
#define MAX5479_WIPER_A_NVREG			(0x21)			//
#define MAX5479_WIPER_A_NVREG_TO_VREG	(0x61)			// Copy NVREG to VREG.
#define MAX5479_WIPER_A_VREG_TO_NVREG	(0x51)			// Copy VREG to NVREG

#define MAX5479_WIPER_B_VREG			(0x12)			//
#define MAX5479_WIPER_B_NVREG			(0x22)			//
#define MAX5479_WIPER_B_NVREG_TO_VREG	(0x62)			// Copy NVREG to VREG.
#define MAX5479_WIPER_B_VREG_TO_NVREG	(0x52)			// Copy VREG to NVREG

#define MAX5479_WIPER_AB_VREG			(0x13)			//
#define MAX5479_WIPER_AB_NVREG			(0x23)			//
#define MAX5479_WIPER_AB_NVREG_TO_VREG	(0x63)			// Copy NVREG to VREG.
#define MAX5479_WIPER_AB_VREG_TO_NVREG	(0x53)			// Copy VREG to NVREG

void max5479_load_wiper_a(uint8_t);
void max5479_load_wiper_b(uint8_t);
void max5479_load_wiper_ab(uint8_t);

void max5479_set_wiper_a(uint8_t);
void max5479_set_wiper_b(uint8_t);
void max5479_set_wiper_ab(uint8_t);

#endif /* MAX5479_H_ */
