/*
 * Library to help deal with events on the Orisen Stamp
 * kbi pins.
 *
 * Author: Stephen Hailes
 *
 */

#include "include/kbi_events.h"
#include "mc1322x.h"
#include <signal.h>
#include "lib/include/gpio.h"

#define DEBUG 1

#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

PROCESS(kbi_event_process, "kbi event handler process");

static void (*eventHandlerT9)(void)    = NULL;
static void (*eventHandlerT10)(void)   = NULL;
static struct process *eventProcessT9  = NULL;
static struct process *eventProcessT10 = NULL;

static uint8_t T9_flag  = 0;
static uint8_t T10_flag = 0;

void kbi_event_init()
{	
	// T10 = kbi6 Interrupt Initialisation
	kbi_edge(6);
	enable_ext_wu(6);
	kbi_pol_neg(6);
	clear_kbi_evnt(6);
	GPIO->PAD_DIR.GPIO_28    = 0;
	GPIO->PAD_PU_SEL.GPIO_28 = 1;
	GPIO->PAD_PU_EN.GPIO_28  = 1;

	// T9 = kbi7 Interrupt Initialisation
	kbi_edge(7);
	enable_ext_wu(7);
	kbi_pol_neg(7);
	clear_kbi_evnt(7);
	GPIO->PAD_DIR.GPIO_29    = 0;
	GPIO->PAD_PU_SEL.GPIO_29 = 1;
	GPIO->PAD_PU_EN.GPIO_29  = 1;

  T9_flag  = 0;
  T10_flag = 0;

	// Start with the kbi pins disabled - we have no
  // handler at present for either anyway
	disable_irq_kbi(6);
	disable_irq_kbi(7);
}

void kbi_event_setHandler(kbi_event_input in, void (*event_handler)(void))
{ 
	switch (in) {
	  case T9:
			eventHandlerT9  = event_handler;
			eventProcessT9  = PROCESS_CURRENT();
			T9_flag = 0;
  		if (event_handler == NULL)
				disable_irq_kbi(7);
			else
				enable_irq_kbi(7);
			break;
	  case T10:
			eventHandlerT10 = event_handler;
			eventProcessT10 = PROCESS_CURRENT();
			T10_flag = 0;
  		if (event_handler == NULL)
				disable_irq_kbi(6);
			else
				enable_irq_kbi(6);
			break;
	}
}

void kbi_event_clearHandler(kbi_event_input in)
{ 
  switch (in) {
    case T9:
			eventHandlerT9  = NULL;
			eventProcessT9  = NULL;
			break;
    case T10:
			eventHandlerT10 = NULL;
			eventProcessT10 = NULL;
			break;
  }
}

PROCESS_THREAD(kbi_event_process, ev, data)
{ 
	PROCESS_BEGIN();
 
  while(1) {
    PROCESS_YIELD();

    if (ev != PROCESS_EVENT_POLL) {
      continue;
    }

    // Let the calling process know that the alarm has expired
    // Both by calling the callback and by posting an event to
    // the process that set up the alarm in the first place.
    //
    if (T9_flag == 1) {
		  if (eventHandlerT9 != NULL)
		    eventHandlerT9();

		  if (eventProcessT9 != NULL)
				process_post(eventProcessT9, KBI_EVENT_T9, NULL);

			T9_flag = 0;
		}

    if (T10_flag == 1) {
		  if (eventHandlerT10 != NULL)
		    eventHandlerT10();

		  if (eventProcessT10 != NULL)
				process_post(eventProcessT10, KBI_EVENT_T10, NULL);

			T10_flag = 0;
		}
  }
  
  PROCESS_END();
}

// Interrupts are on kbi6 and kbi7
//
void kbi6_isr(void)
{ 
	T10_flag = 1;
  process_poll(&kbi_event_process);

	clear_kbi_evnt(6);
}

void kbi7_isr(void)
{ 
	T9_flag = 1;
  process_poll(&kbi_event_process);

	clear_kbi_evnt(7);
}


