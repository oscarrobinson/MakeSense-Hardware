/*
 * Copyright (c) 2010, Mariano Alvira <mar@devl.org> and other contributors
 * to the MC1322x project (http://mc1322x.devl.org) and Contiki.
 *
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
 * This file is part of the Contiki OS.
 *
 * $Id$
 */
#include "contiki-conf.h"
#include "dev/leds.h"
#include "mc1322x.h"
#include "board.h"

#ifdef GPIO_LED_RED
#define LED_ARCH_RED   	GPIO_LED_RED
#else
#undef  LED_ARCH_RED
#endif

#ifdef GPIO_LED_GREEN
#define LED_ARCH_GREEN 	GPIO_LED_GREEN
#else
#undef  LED_ARCH_GREEN
#endif

#ifdef GPIO_LED_BLUE
#define LED_ARCH_BLUE  	GPIO_LED_BLUE
#else
#undef  LED_ARCH_BLUE
#endif

#ifdef GPIO_LED_YELLOW
#define LED_ARCH_YELLOW GPIO_LED_YELLOW
#else
#undef  LED_ARCH_YELLOW
#endif

//#define LED_ARCH_YELLOW (LED_ARCH_RED   | LED_ARCH_GREEN)
#define LED_ARCH_PURPLE (LED_ARCH_RED   | LED_ARCH_BLUE)
#define LED_ARCH_CYAN   (LED_ARCH_GREEN | LED_ARCH_BLUE)
#define LED_ARCH_WHITE  (LED_ARCH_RED   | LED_ARCH_GREEN | LED_ARCH_BLUE)

void leds_arch_init(void)
{
	/* set gpio func_sel to gpio (0) */
	/* and set led gpios to output */
#ifdef LED_ARCH_RED
	GPIO->FUNC_SEL.LED_ARCH_RED    = 0;
	GPIO->PAD_DIR.LED_ARCH_RED     = 1;
#endif
#ifdef LED_ARCH_GREEN
	GPIO->FUNC_SEL.LED_ARCH_GREEN  = 0;
	GPIO->PAD_DIR.LED_ARCH_GREEN   = 1;
#endif
#ifdef LED_ARCH_BLUE
	GPIO->FUNC_SEL.LED_ARCH_BLUE   = 0;
	GPIO->PAD_DIR.LED_ARCH_BLUE    = 1;
#endif
#ifdef LED_ARCH_YELLOW
	GPIO->FUNC_SEL.LED_ARCH_YELLOW = 0;
	GPIO->PAD_DIR.LED_ARCH_YELLOW  = 1;
#endif


	/* set leds off */
	leds_arch_set(0);
}

unsigned char leds_arch_get(void)
{ unsigned char rv = 0;
#ifdef LED_ARCH_RED
	rv = rv | ((GPIO->DATA.LED_ARCH_RED)    ? 0 : LEDS_RED);
#endif
#ifdef LED_ARCH_GREEN
	rv = rv | ((GPIO->DATA.LED_ARCH_GREEN)  ? 0 : LEDS_GREEN);
#endif
#ifdef LED_ARCH_BLUE
	rv = rv | ((GPIO->DATA.LED_ARCH_BLUE)   ? 0 : LEDS_BLUE);
#endif
#ifdef LED_ARCH_YELLOW
	rv = rv | ((GPIO->DATA.LED_ARCH_YELLOW) ? 0 : LEDS_YELLOW);
#endif

  return rv;
}

void leds_arch_set(unsigned char leds)
{
#ifdef LED_ARCH_RED
	if (leds & LEDS_RED)    { gpio_reset(LED_ARCH_RED);    } else { gpio_set(LED_ARCH_RED);    }
#endif
#ifdef LED_ARCH_GREEN
	if (leds & LEDS_GREEN)  { gpio_reset(LED_ARCH_GREEN);  } else { gpio_set(LED_ARCH_GREEN);  }
#endif
#ifdef LED_ARCH_BLUE
	if (leds & LEDS_BLUE)   { gpio_reset(LED_ARCH_BLUE);   } else { gpio_set(LED_ARCH_BLUE);   }
#endif
#ifdef LED_ARCH_YELLOW
 	if (leds & LEDS_YELLOW) { gpio_reset(LED_ARCH_YELLOW); } else { gpio_set(LED_ARCH_YELLOW); }
#endif
}


