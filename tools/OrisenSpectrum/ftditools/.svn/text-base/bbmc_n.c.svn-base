/*
 * bitbang_test.c
 *
 *  Created on: Apr 8, 2013
 *      Author: mcphillips
 */

/* Usage: bbmc_n [options] 				*/
/* 				options  = 	-r [reset]	*/
/* 							-e [erase]	*/

/* control reset and VREF2 lines */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <getopt.h>
#include <ftdi.h>

#define ERASE_SET				0xC8
#define RESET_SET				0xC4
#define RESET_ERASE_SET	0xC0
#define RELEASE					0xCC

static uint32_t vendid = 0x0403; uint32_t prodid = 0x6015;
struct ftdi_context ftdic;
int f;

void reset(void);
void erase(void);

int main(int argc, char *argv[])
{
  if (ftdi_init(&ftdic) < 0) {
    fprintf(stderr, "ftdi_init failed\n");
    return EXIT_FAILURE;
  }

  f = ftdi_usb_open(&ftdic, vendid, prodid);
  if (f < 0 && f != -5) {
    fprintf(stderr, "unable to open ftdi device: %d (%s)\n", f, ftdi_get_error_string(&ftdic));
    exit(-1);
  }
  printf("ftdi open succeeded\n");

	while (1) {
		int c;

		c = getopt(argc, argv,"er");

		switch (c) {
			case 'e':
				erase();
				break;
			case 'r':
				reset();
				break;
			default:
				printf("** You must specify either [e]rase or [r]eset **\n");
				break;
		}
		break;
	}

  printf("disabling bitbang mode\n");

	f = ftdi_set_bitmode(&ftdic, RELEASE, BITMODE_RESET);
	if (f < 0) {
		fprintf(stderr, "set_bitmode failed, error %d (%s)\n", f, ftdi_get_error_string(&ftdic));
		exit(-1);
	}

  ftdi_disable_bitbang(&ftdic);
//    ftdi_usb_reset(&ftdic);

  ftdi_usb_close(&ftdic);
  ftdi_deinit(&ftdic);
  
  return 0;
}

void reset()
{
	f = ftdi_set_bitmode(&ftdic, RELEASE, BITMODE_CBUS);
	if (f < 0) {
		fprintf(stderr, "set_bitmode failed, error %d (%s)\n", f, ftdi_get_error_string(&ftdic));
		exit(-1);
	}

	f = ftdi_set_bitmode(&ftdic, RESET_SET, BITMODE_CBUS);
	if (f < 0) {
		fprintf(stderr, "set_bitmode failed, error %d (%s)\n", f, ftdi_get_error_string(&ftdic));
		exit(-1);
	}

	f = ftdi_set_bitmode(&ftdic, RELEASE, BITMODE_CBUS);
	if (f < 0) {
		fprintf(stderr, "set_bitmode failed, error %d (%s)\n", f, ftdi_get_error_string(&ftdic));
		exit(-1);
	}
}

void erase()
{
	printf("setting VREF2 erase\n");

	f = ftdi_set_bitmode(&ftdic, RELEASE, BITMODE_CBUS);
	if (f < 0) {
		fprintf(stderr, "set_bitmode failed, error %d (%s)\n", f, ftdi_get_error_string(&ftdic));
		exit(-1);
	}

	f = ftdi_set_bitmode(&ftdic, ERASE_SET, BITMODE_CBUS);
	if (f < 0) {
		fprintf(stderr, "set_bitmode failed, error %d (%s)\n", f, ftdi_get_error_string(&ftdic));
		exit(-1);
	}

	printf("toggle reset\n");

	f = ftdi_set_bitmode(&ftdic, (RESET_ERASE_SET), BITMODE_CBUS);
	if (f < 0) {
		fprintf(stderr, "set_bitmode failed, error %d (%s)\n", f, ftdi_get_error_string(&ftdic));
		exit(-1);
	}

	f = ftdi_set_bitmode(&ftdic, ERASE_SET, BITMODE_CBUS);
	if (f < 0) {
		fprintf(stderr, "set_bitmode failed, error %d (%s)\n", f, ftdi_get_error_string(&ftdic));
		exit(-1);
	}

	printf("waiting for erase\n");

	sleep(2);

	printf("setting VREF2 normal\n");

	f = ftdi_set_bitmode(&ftdic, RELEASE, BITMODE_CBUS);
	if (f < 0) {
		fprintf(stderr, "set_bitmode failed, error %d (%s)\n", f, ftdi_get_error_string(&ftdic));
		exit(-1);
	}
}


