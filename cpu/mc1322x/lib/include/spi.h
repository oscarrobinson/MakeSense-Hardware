/*
 * spi.h
 *
 *  Created on: 28 Jul 2011
 *      Author: anton
 *       *      Modded by Graeme McPhillips
 */
/*----------------------------------------------------------------------------*/

#ifndef SPI_H_
#define SPI_H_

#include <stdint.h>
#include "isr.h"
#include "gpio.h"

#define SPI_BASE	(0x80002000)

struct SPI_struct {

	union {
		uint32_t TX_DATA ;
		struct SPI_TX_DATA {
			uint32_t	  :24 ;
			uint32_t DATA :8 ;	/* Last 8 as bit 31 is the one shifted onto MOSI */
		} TX_DATAbits ;
	} ;
	union {
		uint32_t RX_DATA ;
		struct SPI_RX_DATA {
			uint32_t DATA :8 ;	/* First 8, as MISO is shifted into bit[0] */
			uint32_t	  :24 ;
		} RX_DATAbits ;
	} ;
	union {
		uint32_t CLK_CTRL ;
		struct SPI_CLK_CTRL {
			uint32_t DATA_LENGTH :7 ;
			uint32_t START		 :1 ;
			uint32_t SCK_COUNT	 :8 ;
			uint32_t		  	 :16 ;
		} CLK_CTRLbits ;
	} ;
	union {
		uint32_t SETUP ;
		struct SPI_SETUP {
			uint32_t SS_SETUP	:2 ;
			uint32_t SS_DELAY	:2 ;
			uint32_t SDO_INACTIVE_ST :2 ;
			uint32_t 			:2 ;
			uint32_t SCK_POL	:1 ;
			uint32_t SCK_PHASE	:1 ;
			uint32_t MISO_PHASE :1 ;
			uint32_t 			:1 ;
			uint32_t SCK_FREQ	:3 ;
			uint32_t 			:1 ;
			uint32_t MODE		:1 ;
			uint32_t SPI3WIRE	:1 ;
			uint32_t 			:14 ;
		} SETUPbits ;
	} ;
	union {
		uint32_t STATUS ;
		struct SPI_STATUS {
			uint32_t INT		:1 ;
			uint32_t 			:3 ;
			uint32_t OVERFLOW	:1 ;
			uint32_t 			:3 ;
			uint32_t FIRST_DATA	:1 ;
			uint32_t 			:23 ;
		} STATUSbits ;
	} ;

} ;

static volatile struct SPI_struct * const SPI = (void *)(SPI_BASE) ;

/*----------------------------------------------------------------------------*/

enum {
	SPI_FREQ_12MHZ = 0,
	SPI_FREQ_6MHZ,
	SPI_FREQ_3MHZ,
	SPI_FREQ_1500KHZ,
	SPI_FREQ_750KHZ,
	SPI_FREQ_375KHZ,
	SPI_FREQ_187KHZ,		/* 187.5 kHz */
	SPI_FREQ_94KHZ			/* 93.75 kHz */
} ;

/* Settings below for master mode. Slave mode settings in comments */
enum {
	SPI_SS_AUTO_HIGH = 0,	/* High */
	SPI_SS_AUTO_LOW,		/* Low */
	SPI_SS_MAN_LOW,			/* High */
	SPI_SS_MAN_HIGH			/* Low */
} ;

enum {
	SPI_MODE_MASTER = 0,
	SPI_MODE_SLAVE
} ;

/*----------------------------------------------------------------------------*/

#define SPI_SS		GPIO_04
#define SPI_MISO	GPIO_05
#define SPI_MOSI	GPIO_06
#define SPI_SCK		GPIO_07

/*----------------------------------------------------------------------------*/

#define spi_start()			SPI->CLK_CTRLbits.START = 1

#define spi_done()			(SPI->STATUSbits.INT == 1)
#define spi_int_clear()		SPI->STATUSbits.INT = 1

#define spi_set_sck(x)		SPI->CLK_CTRLbits.SCK_COUNT = (x - 1)
#define spi_data_len(x)		SPI->CLK_CTRLbits.DATA_LENGTH = (x)

#define spi_set_speed(x)	SPI->SETUPbits.SCK_FREQ = x

#define spi_ss_low()		SPI->SETUPbits.SS_SETUP = 2
#define spi_ss_high()		SPI->SETUPbits.SS_SETUP = 3

#define spi_slave_recv()	(SPI->STATUSbits.FIRST_DATA == 1)

/*----------------------------------------------------------------------------*/

/* The SPI Interrupt Service Routine */
void spi_isr (void);

/*
 * Initialize the spi bus. Designed to work with an SD card so this method
 * would need to be altered for different devices.
 *
 */
void spi_init(void) ;

/*
 * Generic method that writes and reads a byte.
 *
 * \param data_out The data to be sent on the spi bus.
 *
 * \return The byte received on the spi bus.
 *
 */
uint8_t spi_rw_byte(uint8_t) ;

/*----------------------------------------------------------------------------*/

#endif /* SPI_H_ */
