/*
 * spi.c
 *
 *  Created on: 2 Aug 2011
 *      Author: anton
 *      Modded by Graeme McPhillips
 */
/*----------------------------------------------------------------------------*/

#include "spi.h"
#include <stdio.h>

/*----------------------------------------------------------------------------*/

void spi_init(void) {

	/* Activate spi module for SD use (mode 0 - CPHA=0, CPOL=0) */
	SPI->SETUPbits.MODE = SPI_MODE_MASTER ;
	SPI->SETUPbits.SCK_PHASE = 0 ;
	SPI->SETUPbits.SCK_POL = 0 ;
	SPI->SETUPbits.SS_SETUP = SPI_SS_MAN_LOW ;
	SPI->SETUPbits.SDO_INACTIVE_ST = 1 ;
	SPI->SETUPbits.SCK_FREQ =  SPI_FREQ_375KHZ ;	/* for SD initialization */

	/* PU on MISO */
	GPIO->PAD_PU_EN.SPI_MISO = 1 ;
	GPIO->PAD_PU_SEL.SPI_MISO = 1 ;

	/* Set GPIO mode */
	GPIO->FUNC_SEL.SPI_SS	= 1 ;
	GPIO->FUNC_SEL.SPI_MISO = 1 ;
	GPIO->FUNC_SEL.SPI_MOSI = 1 ;
	GPIO->FUNC_SEL.SPI_SCK	= 1 ;

}

/*----------------------------------------------------------------------------*/

/* Generic function to send and receive a byte */
uint8_t spi_rw_byte(uint8_t data_out) {

	uint8_t data_in ;

	spi_set_sck(8) ;
	spi_data_len(8) ;
	SPI->TX_DATAbits.DATA = data_out ;
	spi_start() ;

	while(!spi_done()) { /* Wait */ }

	data_in = SPI->RX_DATAbits.DATA ;

	spi_int_clear() ;

	return data_in ;
}

/*----------------------------------------------------------------------------*/

/* The SPI Interrupt Service Routine */
void spi_isr (void) {
//	uint8_t dummy;
	spi_int_clear();



}
