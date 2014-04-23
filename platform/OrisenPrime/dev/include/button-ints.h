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


/* Defines for Timers */
/* timer x enable bits */
#define TMR0		0x01	/* use timer 0 */
#define TMR1		0x02	/* use timer 1 */
#define TMR2		0x04	/* use timer 2 */
#define TMR3		0x08	/* use timer 3 */

/* TMR Status and Control Registers (TMRX_SCTRL) */
#define TCF			0x8000	/* Timer Compare Flag. This bit indicates when a successful compare occurred. */

/* TMR Comparator Status and Control Registers (TMRX_CSCTRL) */
#define TCF1		0x0010	/*Timer Compare 1Status. This bit indicates a successful comparison between the timer and COMP1 register has occurred. */


/* timer 0 */
/* delay function */
#define COUNT_MODE0 1      /* use rising edge of primary source */
#define PRIME_SRC0  0xe    /* Perip. clock with 128 prescale (for 24Mhz = 187500Hz)*/
#define SEC_SRC0    0      /* don't need this */
#define ONCE0       0      /* keep counting */
#define LEN0        1      /* count until compare then reload with value in LOAD */
#define DIR0        0      /* count up */
#define CO_INIT0    0      /* other counters cannot force a re-initialization of this counter */
#define OUT_MODE0   0      /* OFLAG is asserted while counter is active */

/* timer 1 */
#define COUNT_MODE1 1      /* use rising edge of primary source */
#define PRIME_SRC1  0xf    /* Perip. clock with 128 prescale (for 24Mhz = 187500Hz)*/
#define SEC_SRC1    0      /* don't need this */
#define ONCE1       0      /* keep counting */
#define LEN1        1      /* count until compare then reload with value in LOAD */
#define DIR1        0      /* count up */
#define CO_INIT1    0      /* other counters cannot force a re-initialization of this counter */
#define OUT_MODE1   0      /* OFLAG is asserted while counter is active */



/* End of Defines for Timer */

