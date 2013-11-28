#ifndef __KBI_EVENTS_H
#define __KBI_EVENTS_H

#include "sys/process.h"

enum {
	KBI_EVENT_T9,
	KBI_EVENT_T10
};


typedef enum {
	T9,
	T10
} kbi_event_input;

// Function definitions
//
void kbi_event_init();
void kbi_event_setHandler(kbi_event_input in, void (*event_handler)(void));
void kbi_event_clearHandler(kbi_event_input in);

PROCESS_NAME(kbi_event_process);

#endif /* __RTC_H */



