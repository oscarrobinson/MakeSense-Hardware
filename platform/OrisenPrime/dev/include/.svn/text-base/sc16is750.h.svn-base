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

#ifndef SC16IS750_H_
#define SC16IS750_H_

#include <stdbool.h>

#define SC16IS750_I2C_ADDR					((0x9A)>>1)

#define SC16IS750_EN_FIFO					(0x01)		// The reset TX FIFO bit
#define SC16IS750_RST_RX_FIFO				((0x01)<<1)		// The reset RX FIFO bit
#define SC16IS750_RST_TX_FIFO				((0x01)<<2)		// The reset TX FIFO bit

#define SC16IS750_RHR						((0x00)<<3)
#define SC16IS750_THR						((0x00)<<3)
#define SC16IS750_IER						((0x01)<<3)
#define SC16IS750_FCR						((0x02)<<3)
#define SC16IS750_IIR						((0x02)<<3)
#define SC16IS750_LCR						((0x03)<<3)
#define SC16IS750_MCR						((0x04)<<3)
#define SC16IS750_LSR						((0x05)<<3)
#define SC16IS750_MSR						((0x06)<<3)
#define SC16IS750_SPR						((0x07)<<3)
#define SC16IS750_TCR						((0x06)<<3)
#define SC16IS750_TLR						((0x07)<<3)
#define SC16IS750_TXLVL						((0x08)<<3)
#define SC16IS750_RXLVL						((0x09)<<3)
#define SC16IS750_IO_DIR					((0x0A)<<3)
#define SC16IS750_IO_STATE					((0x0B)<<3)
#define SC16IS750_IO_INT_ENA				((0x0C)<<3)
#define SC16IS750_IO_CONTROL				((0x0E)<<3)
#define SC16IS750_EFCR						((0x0F)<<3)

#define SC16IS750_DLL						((0x00)<<3)	// The special register set is accessible only when LCR[7] = 1 and not 0xBF
#define SC16IS750_DLH						((0x01)<<3)

#define SC16IS750_EFR						((0x02)<<3)	// Enhanced Feature Registers are only accessible when LCR = 0xBF
#define SC16IS750_XON1						((0x04)<<3)
#define SC16IS750_XON2						((0x05)<<3)
#define SC16IS750_XOFF1						((0x06)<<3)
#define SC16IS750_XOFF2						((0x07)<<3)

struct nav_sol_packet {
	uint16_t header ;
	uint16_t id ;
	uint16_t length ;
	uint32_t itow ;
	int32_t frac ;
	int16_t week ;
	uint8_t gpsfix ;
	uint8_t flags ;
	int32_t ecef_x ;
	int32_t ecef_y ;
	int32_t  ecef_z ;
	uint32_t pacc ;
	int32_t ecefvx ;
	int32_t ecefvy ;
	int32_t ecefvz ;
	uint32_t sacc ;
	uint16_t pdop ;
	uint8_t res1 ;
	uint8_t numsv ;
	uint32_t res2 ;
	uint8_t ck_a ;
	uint8_t ck_b ;
} ;

void sc16is750_init(void) ;
void sc16is750_start_timepulse(void);
void sc16is750_reset(void);
int sc16is750_read_iir(void);
int sc16is750_read_io_state(void);
void sc16is750_en_int(void);
void sc16is750_en_io_int(void);
void sc16is750_dis_int(void);
void sc16is750_dis_io_int(void);
void sc16is750_set_baud_rate(void) ;
void sc16is750_reset_rx_fifo(void) ;
int sc16is750_get_rx_fifo_level(void) ;
void sc16is750_reset_tx_fifo(void);
void sc16is750_get_uart(void) ;
void sc16is750_put_uart(uint8_t) ;
void sc16is750_get_gps(uint8_t *buffer);
int sc16is750_check_ck(uint8_t *buffer, uint8_t len);
void sc16is750_decode_gps(struct nav_sol_packet *nsp, uint8_t *buffer);
void sc16is750_stop_gps(void);
void sc16is750_start_gps(void);
void sc16is750_shutdown_gps(void);
void sc16is750_set_gps(void) ;
void sc16is750_reset_gps_uart(void);

#endif /* SC16IS750_H_ */
