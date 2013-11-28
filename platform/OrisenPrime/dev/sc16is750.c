/*
 * sc16is750.c
 *
 *  Created on: 1 Sep 2011
 *      Author: mcphillips
 */

#include <stdio.h>
#include "i2c.h"
#include "uart.h"
#include "include/sc16is750.h"

//extern volatile uint8_t timer_delay;
// TODO Compiler complains - undefined reference to timer_delay. Where is it?
volatile uint8_t timer_delay;

void sc16is750_init(void) {

	uint8_t set[] = {0,0} ;

	//	printf("about to reset SC16IS750:\n")
	set[0] = SC16IS750_IO_CONTROL ;
	set[1] = 0x08 ;
	i2c_transmitinit( SC16IS750_I2C_ADDR, 2, set ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;


	//	printf("about to set SC16IS750 LCR:\n") ;
	set[0] = SC16IS750_LCR ;
	set[1] = 0x80 ;
	i2c_transmitinit( SC16IS750_I2C_ADDR, 2, set ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;

	//	printf("about to set SC16IS750 DLL:\n") ;
	set[0] = SC16IS750_DLL ;
	set[1] = 0x60 ;	//should be 0x60
	i2c_transmitinit( SC16IS750_I2C_ADDR, 2, set ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;

	//	printf("about to set SC16IS750 DLH:\n") ;
	set[0] = SC16IS750_DLH ;
	set[1] = 0x00 ;
	i2c_transmitinit( SC16IS750_I2C_ADDR, 2, set ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;

	//	printf("about to set SC16IS750 LCR:\n") ;
	set[0] = SC16IS750_LCR ;
	set[1] = 0xBF ;
	i2c_transmitinit( SC16IS750_I2C_ADDR, 2, set ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;

	//	printf("about to set SC16IS750 EFR:\n") ;
	set[0] = SC16IS750_EFR ;
	set[1] = 0x10 ;
	i2c_transmitinit( SC16IS750_I2C_ADDR, 2, set ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;

	//	printf("about to set SC16IS750 LCR:\n") ;
	set[0] = SC16IS750_LCR ;
	set[1] = 0x03 ;
	i2c_transmitinit( SC16IS750_I2C_ADDR, 2, set ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;

	//	printf("about to set SC16IS750 FCR:\n") ;
	set[0] = SC16IS750_FCR ;
	set[1] = 0xC1 ; // was 0x01 - but set it to 0x81 to enable an interrupt after 56 bytes
	i2c_transmitinit( SC16IS750_I2C_ADDR, 2, set ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;

	//	printf("about to set SC16IS750 I/O direction:\n") ;
	set[0] = SC16IS750_IO_DIR ;
	set[1] = 0xD0 ;
	i2c_transmitinit( SC16IS750_I2C_ADDR, 2, set ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;

	//	printf("about to set SC16IS750 I/O state:\n")
	set[0] = SC16IS750_IO_STATE ;
	set[1] = 0x50 ;
	i2c_transmitinit( SC16IS750_I2C_ADDR, 2, set ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;

	set[0] = SC16IS750_IO_CONTROL ;
	set[1] = 0x01;
	i2c_transmitinit( SC16IS750_I2C_ADDR, 2, set ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;

}

void sc16is750_reset(void){
	uint8_t set[] = {0,0} ;

	set[0] = SC16IS750_IO_CONTROL ;
	set[1] = 0x08 ;

//	printf("about to reset SC16IS750 I/O control:\n") ;
	i2c_transmitinit( SC16IS750_I2C_ADDR, 2, set ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;
	timer_delay = 10;
	while(timer_delay > 0){}
//	printf("reset SC16IS750 I/O control:\n") ;
}

void sc16is750_en_int(void){
	uint8_t set[] = {0,0} ;
	set[0] = SC16IS750_IER ;
	set[1] = 0x01 ;

	i2c_transmitinit( SC16IS750_I2C_ADDR, 2, set ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;
}

void sc16is750_dis_int(void){
	uint8_t set[] = {0,0} ;
	set[0] = SC16IS750_IER ;
	set[1] = 0x00 ;

	i2c_transmitinit( SC16IS750_I2C_ADDR, 2, set ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;
}

void sc16is750_en_io_int(void){
	uint8_t set[] = {0,0} ;
	set[0] = SC16IS750_IO_INT_ENA ;
	set[1] = 0x04 ;

	i2c_transmitinit( SC16IS750_I2C_ADDR, 2, set ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;
}

void sc16is750_dis_io_int(void){
	uint8_t set[] = {0,0} ;
	set[0] = SC16IS750_IO_INT_ENA ;
	set[1] = 0x00 ;

	i2c_transmitinit( SC16IS750_I2C_ADDR, 2, set ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;
}

void sc16is750_set_baud_rate(void) {
	uint8_t set[] = {0,0} ;

	set[0] = SC16IS750_LCR ;
	set[1] = 0x80 ;
	i2c_transmitinit( SC16IS750_I2C_ADDR, 2, set ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;

	set[0] = SC16IS750_DLL ;
	set[1] = 0x08 ;				/* baud rate 115200 */
	i2c_transmitinit( SC16IS750_I2C_ADDR, 2, set ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;

	set[0] = SC16IS750_DLH ;
	set[1] = 0x00 ;
	i2c_transmitinit( SC16IS750_I2C_ADDR, 2, set ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;

	set[0] = SC16IS750_LCR ;
	set[1] = 0x03 ;
	i2c_transmitinit( SC16IS750_I2C_ADDR, 2, set ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;

}

int sc16is750_read_io_state(void) {
	uint8_t request = SC16IS750_IO_STATE ;
	uint8_t receive = 0 ;

	i2c_transmitinit( SC16IS750_I2C_ADDR, 1, &request ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;

	i2c_receiveinit( SC16IS750_I2C_ADDR, 1, &receive ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;

//	printf("Read out IIR:\n") ;
//	printf("SC16IS750_IO_STATE: %x\n", receive) ;
	return receive;

}

int sc16is750_read_iir(void) {
	uint8_t request = SC16IS750_IIR ;
	uint8_t receive = 0 ;

	i2c_transmitinit( SC16IS750_I2C_ADDR, 1, &request ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;

	i2c_receiveinit( SC16IS750_I2C_ADDR, 1, &receive ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;

//	printf("Read out IIR:\n") ;
//	printf("SC16IS750_IIR_STATE: %x\n", receive) ;
	return receive;

}

void sc16is750_reset_rx_fifo(void) {
//	i2c_enable() ;

	uint8_t request = SC16IS750_FCR ;
	uint8_t receive = 0 ;
	uint8_t set[] = {0,0} ;

	i2c_transmitinit( SC16IS750_I2C_ADDR, 1, &request ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;

	i2c_receiveinit( SC16IS750_I2C_ADDR, 1, &receive ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;

	set[0] = SC16IS750_FCR ;
	set[1] = (receive | SC16IS750_RST_RX_FIFO | SC16IS750_EN_FIFO) ;

	i2c_transmitinit( SC16IS750_I2C_ADDR, 2, set ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;

//	printf("RX FIFO RESET:\n") ;

//	i2c_disable() ;
}

void sc16is750_reset_tx_fifo(void) {
//	i2c_enable() ;

	uint8_t request = SC16IS750_FCR ;
	uint8_t receive = 0 ;
	uint8_t set[] = {0,0} ;

	i2c_transmitinit( SC16IS750_I2C_ADDR, 1, &request ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;

	i2c_receiveinit( SC16IS750_I2C_ADDR, 1, &receive ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;

	set[0] = SC16IS750_FCR ;
	set[1] = (receive | SC16IS750_RST_TX_FIFO | SC16IS750_EN_FIFO) ;

	i2c_transmitinit( SC16IS750_I2C_ADDR, 2, set ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;

//	printf("RX FIFO RESET:\n") ;

//	i2c_disable() ;
}

int sc16is750_get_rx_fifo_level(void) {
//	i2c_enable() ;

	uint8_t request = SC16IS750_RXLVL ;
	uint8_t receive = 0 ;

	i2c_transmitinit( SC16IS750_I2C_ADDR, 1, &request ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;

	i2c_receiveinit( SC16IS750_I2C_ADDR, 1, &receive ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;

//	printf("F: %d\n", receive) ;

	return receive;

//	i2c_disable() ;
}

void sc16is750_get_uart(void) {
//	i2c_enable() ;

	uint8_t request = SC16IS750_RHR ;
	uint8_t receive = 0 ;

	i2c_transmitinit( SC16IS750_I2C_ADDR, 1, &request ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;

	i2c_receiveinit( SC16IS750_I2C_ADDR, 1, &receive ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;

//	printf("%02x ", receive) ;
	uart1_putc(receive);

//	i2c_disable() ;
}

void sc16is750_put_uart(uint8_t uart_tx) {
//	i2c_enable() ;

	uint8_t set[] = {SC16IS750_THR,uart_tx} ;

	i2c_transmitinit( SC16IS750_I2C_ADDR, 2, set ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;

//	i2c_disable() ;
}

#if 1

void sc16is750_get_gps(uint8_t *buffer) {

	uint8_t request = SC16IS750_RHR ;

	i2c_transmitinit( SC16IS750_I2C_ADDR, 1, &request ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;

	i2c_receiveinit( SC16IS750_I2C_ADDR, 60, buffer ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;
}
#endif

int sc16is750_check_ck(uint8_t *buffer, uint8_t len ) {
	uint8_t ck_a = 0;
	uint8_t ck_b = 0;
	uint8_t i = 0;
	for (i = 0; i < (len - 4); i++ ){
		ck_a = ck_a + buffer[i+2];
		ck_b = ck_b + ck_a;
	}
//	printf("ck_a %x cal_a %x ck_b %x cal_b %x ",buffer[])
	if ((ck_a == buffer[len-2]) && (ck_b == buffer[len-1])) return 1;
	else return 0;
}


void sc16is750_decode_gps(struct nav_sol_packet *nsp, uint8_t *buffer) {
	nsp->header = buffer[0] + (buffer[1]<<8);
	nsp->id = buffer[2] + (buffer[3]<<8);
	nsp->length = buffer[4] + (buffer[5]<<8) ;
	nsp->itow = buffer[6] + (buffer[7]<<8) + (buffer[8]<<16) + (buffer[9]<<24) ;
	nsp->frac = buffer[10] + (buffer[11]<<8) + (buffer[12]<<16) + (buffer[13]<<24) ;
	nsp->week = buffer[14] + (buffer[15]<<8) ;
	nsp->gpsfix = buffer[16] ;
	nsp->flags = buffer[17] ;
	nsp->ecef_x = buffer[18] + (buffer[19]<<8) + (buffer[20]<<16) + (buffer[21]<<24) ;
	nsp->ecef_y = buffer[22] + (buffer[23]<<8) + (buffer[24]<<16) + (buffer[25]<<24) ;
	nsp->ecef_z = buffer[26] + (buffer[27]<<8) + (buffer[28]<<16) + (buffer[29]<<24) ;
	nsp->pacc = buffer[30] + (buffer[31]<<8) + (buffer[32]<<16) + (buffer[33]<<24) ;
	nsp->ecefvx = buffer[34] + (buffer[35]<<8) + (buffer[36]<<16) + (buffer[37]<<24) ;
	nsp->ecefvy = buffer[38] + (buffer[39]<<8) + (buffer[40]<<16) + (buffer[41]<<24) ;
	nsp->ecefvz = buffer[42] + (buffer[43]<<8) + (buffer[44]<<16) + (buffer[45]<<24) ;
	nsp->sacc = buffer[46] + (buffer[47]<<8) + (buffer[48]<<16) + (buffer[49]<<24) ;
	nsp->pdop = buffer[50] + (buffer[51]<<8) ;
	nsp->res1 = buffer[52] ;
	nsp->numsv = buffer[53] ;
	nsp->res2 = buffer[54] + (buffer[55]<<8) + (buffer[56]<<16) + (buffer[57]<<24) ;
	nsp->ck_a = buffer[58] ;
	nsp->ck_b = buffer[59] ;
}


void sc16is750_set_gps(void) {

	uint8_t set[8][15] = {
			{ SC16IS750_THR, 0xB5, 0x62, 0x06, 0x01, 0x06, 0x00, 0x01, 0x06, 0x00, 0x01, 0x00, 0x00, 0x15, 0x9c }, //NAV-SOL uart1
			{ SC16IS750_THR, 0xB5, 0x62, 0x06, 0x01, 0x06, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFD, 0x15 }, //NMEW-GPGGA off
			{ SC16IS750_THR, 0xB5, 0x62, 0x06, 0x01, 0x06, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0xFE, 0x1A }, //NMEW-GPGLL off
			{ SC16IS750_THR, 0xB5, 0x62, 0x06, 0x01, 0x06, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x1F }, //NMEW-GPGSA off
			{ SC16IS750_THR, 0xB5, 0x62, 0x06, 0x01, 0x06, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x24 }, //NMEW-GPGSV off
			{ SC16IS750_THR, 0xB5, 0x62, 0x06, 0x01, 0x06, 0x00, 0xF0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x01, 0x29 }, //NMEW-GPRMC off
			{ SC16IS750_THR, 0xB5, 0x62, 0x06, 0x01, 0x06, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x02, 0x2E } //NMEW-GPVTG off
	};

	i2c_transmitinit( SC16IS750_I2C_ADDR, 15, set[0]) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;
	timer_delay = 10;
	while(timer_delay > 0){}

	i2c_transmitinit( SC16IS750_I2C_ADDR, 15, set[1] ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;
	timer_delay = 10;
	while(timer_delay > 0){}

	i2c_transmitinit( SC16IS750_I2C_ADDR, 15, set[2] ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;
	timer_delay = 10;
	while(timer_delay > 0){}

	i2c_transmitinit( SC16IS750_I2C_ADDR, 15, set[3] ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;
	timer_delay = 10;
	while(timer_delay > 0){}

	i2c_transmitinit( SC16IS750_I2C_ADDR, 15, set[4] ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;
	timer_delay = 10;
	while(timer_delay > 0){}

	i2c_transmitinit( SC16IS750_I2C_ADDR, 15, set[5] ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;
	timer_delay = 10;
	while(timer_delay > 0){}

	i2c_transmitinit( SC16IS750_I2C_ADDR, 15, set[6] ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;

}

void sc16is750_start_timepulse(void) {

//	uint8_t set[29] = {SC16IS750_THR, 0xB5, 0x62, 0x07, 0x14, 0x00, 0x40, 0x42, 0x0f, 0x00, 0xa0, 0x86, 0x01, 0x00, 0x01, 0x01, 0x00, 0x00, 0x34, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x12, 0x91 };

	uint8_t set[46] = {	 SC16IS750_THR, 0xB5, 0x62, 0x06, 0x31, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
										0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x99, 0x99, 0x99, 0x19,
										0x00, 0x00, 0x00, 0x00, 0xef, 0x00, 0x00, 0x00, 0xac, 0xcd };


	i2c_transmitinit( SC16IS750_I2C_ADDR, 46, set) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;
}

void sc16is750_stop_gps(void) {

	uint8_t set[13] = {SC16IS750_THR, 0xB5, 0x62, 0x06, 0x04, 0x04, 0x00, 0xFF, 0x07, 0x08, 0x00, 0x1C, 0x85 };

	i2c_transmitinit( SC16IS750_I2C_ADDR, 13, set) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;
}

void sc16is750_start_gps(void) {

	uint8_t set[13] = {SC16IS750_THR, 0xB5, 0x62, 0x06, 0x04, 0x04, 0x00, 0xFF, 0x07, 0x09, 0x00, 0x1D, 0x87 };

	i2c_transmitinit( SC16IS750_I2C_ADDR, 13, set) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;
}
void sc16is750_shutdown_gps(void) {

	uint8_t set[13] = {SC16IS750_THR, 0xB5, 0x62, 0x06, 0x04, 0x04, 0x00, 0xFF, 0x07, 0x07, 0x00, 0x1B, 0x83 };

	i2c_transmitinit( SC16IS750_I2C_ADDR, 13, set) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;
}

void sc16is750_reset_gps_uart(void){
	uint8_t temp = 0;
	while (temp == 0){
		sc16is750_reset_rx_fifo();
		timer_delay = 50;
		while(timer_delay > 0){}
		if (sc16is750_get_rx_fifo_level() == 0){
			temp = 1;
		}
	}
}
