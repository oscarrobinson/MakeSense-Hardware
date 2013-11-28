/*
 * Copyright (c) 2011, Graeme McPhillips <g.mcphillips@cs.ucl.ac.uk> and
 * 2013, Jagun Kwon <jagun.kwon@ucl.ac.uk>
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

#ifndef LEA_6T_H_
#define LEA_6T_H_

#include <stdbool.h>
#include "typedefs.h"

#include "ubx_protocol.h"

#define LEA_6T_I2C_ADDR					((0x84)>>1)
#define LEA_6T_BYTES_AVAILABLE_HIGH		0xFD
#define LEA_6T_BYTES_AVAILABLE_LOW		0xFE
#define LEA_6T_DATA_STREAM				0xFF

#define TRUE		1
#define FALSE		0

/* UBX parser status */
#define UNINIT        0
#define GOT_SYNC1     1
#define GOT_SYNC2     2
#define GOT_CLASS     3
#define GOT_ID        4
#define GOT_LEN1      5
#define GOT_LEN2      6
#define GOT_PAYLOAD   7
#define GOT_CHECKSUM1 8

/* last error type */
#define GPS_UBX_ERR_NONE         0
#define GPS_UBX_ERR_OVERRUN      1
#define GPS_UBX_ERR_MSG_TOO_LONG 2
#define GPS_UBX_ERR_CHECKSUM     3
#define GPS_UBX_ERR_UNEXPECTED   4
#define GPS_UBX_ERR_OUT_OF_SYNC  5

#define UTM_HEM_NORTH 0
#define UTM_HEM_SOUTH 1

#define GpsUartSend1(c) GpsLink(Transmit(c))
#define GpsUartSetBaudrate(_a) GpsLink(SetBaudrate(_a))
#define GpsUartRunning GpsLink(TxRunning)
#define GpsUartSendMessage GpsLink(SendMessage)

#define UbxInitCheksum() { gps_ubx.send_ck_a = gps_ubx.send_ck_b = 0; }
#define UpdateChecksum(c) { gps_ubx.send_ck_a += c; gps_ubx.send_ck_b += gps_ubx.send_ck_a; }
#define UbxTrailer() { GpsUartSend1(gps_ubx.send_ck_a);  GpsUartSend1(gps_ubx.send_ck_b); GpsUartSendMessage(); }

#define UbxSend1(c) { uint8_t i8=c; GpsUartSend1(i8); UpdateChecksum(i8); }
#define UbxSend2(c) { uint16_t i16=c; UbxSend1(i16&0xff); UbxSend1(i16 >> 8); }
#define UbxSend1ByAddr(x) { UbxSend1(*x); }
#define UbxSend2ByAddr(x) { UbxSend1(*x); UbxSend1(*(x+1)); }
#define UbxSend4ByAddr(x) { UbxSend1(*x); UbxSend1(*(x+1)); UbxSend1(*(x+2)); UbxSend1(*(x+3)); }

#define UbxHeader(nav_id, msg_id, len) {        \
    GpsUartSend1(UBX_SYNC1);                    \
    GpsUartSend1(UBX_SYNC2);                    \
    UbxInitCheksum();                           \
    UbxSend1(nav_id);                           \
    UbxSend1(msg_id);                           \
    UbxSend2(len);                              \
  }

#define GPS_NB_CHANNELS 16

#define GPS_UBX_MAX_PAYLOAD 255


// UBX message packet structure
struct GpsUbx {
  //bool_t msg_available;
  u8t msg_available;
  uint8_t msg_buf[GPS_UBX_MAX_PAYLOAD] __attribute__ ((aligned));
  uint8_t msg_id;
  uint8_t msg_class;

  uint8_t status;
  uint16_t len;
  uint8_t msg_idx;
  uint8_t ck_a, ck_b;
  uint8_t send_ck_a, send_ck_b;
  uint8_t error_cnt;
  uint8_t error_last;

  uint8_t status_flags;
  uint8_t sol_flags;
  uint8_t have_velned;
};

//extern struct GpsUbx gps_ubx;

// NAV-SOL packet structure
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


/*----------------*/
/* Standard types */
/*----------------*/
typedef signed int             I;   //!<  = I4
typedef unsigned int           U;   //!<  = U4
typedef unsigned long          BL;  //!<  = L4 (TRUE or FALSE only)
typedef signed char            I1;  //!<  signed 1 byte integer
typedef signed short           I2;  //!<  signed 2 byte integer
typedef signed int             I4;  //!<  signed 4 byte integer
typedef signed long long int   I8;  //!<  signed 8 byte integer
typedef unsigned char          U1;  //!<  unsigned 1 byte integer
typedef unsigned char          X1;  //!<  unsigned 1 byte integer, to be interpreted as bitmask
typedef unsigned short         U2;  //!<  unsigned 2 byte integer
typedef unsigned short         X2;  //!<  unsigned 2 byte integer, to be interpreted as bitmask
typedef unsigned int           U4;  //!<  unsigned 4 byte integer
typedef unsigned int           X4;  //!<  unsigned 4 byte integer, to be interpreted as bitmask
typedef unsigned long long int U8;  //!<  unsigned 8 byte integer
typedef float                  R4;  //!<  4 byte floating point
typedef double                 R8;  //!<  8 byte floating point
typedef char                   CH;  //!<  ASCII character
typedef unsigned char          L1;  //!<  1 byte logical (TRUE or FALSE only)
typedef unsigned short         L2;  //!<  2 byte logical (TRUE or FALSE only)
typedef unsigned int           L4;  //!<  4 byte logical (TRUE or FALSE only)
typedef unsigned int            L;  //!<  4 byte logical (TRUE or FALSE only)

// Class ID for UBX_RXM
#define UBX_RXM_ID						0x02	// Class ID
#define UBX_RXM_RAW_ID					0x10	// Message ID within Class
#define UBX_RXM_SFRB_ID					0x11	// Message ID within Class
#define UBX_RXM_SVSI_ID					0x12	// Message ID within Class

typedef struct GPS_UBX_RXM_RAW_s
{
    I4  iTOW;                     //!< Measurement integer millisecond GPS time of week  (Receiver Time)
    I2  week;                     //!< Measurement GPS week number (Receiver Time).
    U1  numSV;                    //!< # of satellites following.
    U1  res1;                     //!< Reserved
    //REPEAT: GPS_UBX_RXM_RAW_CPMES_t repeat0[numSV];          

} GPS_UBX_RXM_RAW_t,*GPS_UBX_RXM_RAW_pt;

//! Optional Sub-Structure of #GPS_UBX_RXM_RAW_t
typedef struct GPS_UBX_RXM_RAW_CPMES_s
{
    R8  cpMes;                    //!< Carrier phase measurement [L1 cycles]
    R8  prMes;                    //!< Pseudorange measurement [m]
    R4  doMes;                    //!< Doppler measurement [Hz]
    U1  sv;                       //!< Space Vehicle Number
    I1  mesQI;                    //!< Nav Measurements Quality Indicator:
    I1  cno;                      //!< Signal strength C/No. (dbHz)
    U1  lli;                      //!< Loss of lock indicator (RINEX definition)

} GPS_UBX_RXM_RAW_CPMES_t,*GPS_UBX_RXM_RAW_CPMES_pt;


typedef struct GPS_UBX_RXM_SFRB_s
{
    U1  chn;                      //!< Channel Number
    U1  svid;                     //!< ID of Satellite transmitting Subframe
    X4  dwrd[10];                 //!< Words of Data

} GPS_UBX_RXM_SFRB_t,*GPS_UBX_RXM_SFRB_pt;

typedef struct GPS_UBX_RXM_SVSI_s
{
    I4  iTOW;                     //!< Measurement integer millisecond GPS time of week
    I2  week;                     //!< Measurement GPS week number.
    U1  numVis;                   //!< number of visible satellites
    U1  numSV;                    //!< number of per-SV data blocks following
    //REPEAT: GPS_UBX_RXM_SVSI_SVID_t repeat0[numSV];          

} GPS_UBX_RXM_SVSI_t,*GPS_UBX_RXM_SVSI_pt;

typedef struct GPS_UBX_RXM_SVSI_SVID_s
{
    U1  svid;                     //!< Satellite ID
    X1  svFlag;                   //!< Information Flags
    I2  azim;                     //!< Azimuth
    I1  elev;                     //!< Elevation
    X1  age;                      //!< Age of Almanach and Ephemeris: 

} GPS_UBX_RXM_SVSI_SVID_t,*GPS_UBX_RXM_SVSI_SVID_pt;



int lea_6t_get_bytes_available(void) ;
void lea_6t_get_data_stream(uint8_t *buffer, uint16_t len);
int lea_6t_check_ck(uint8_t *buffer, uint8_t len);
void lea_6t_decode_gps(struct nav_sol_packet *nsp, uint8_t *buffer);
void lea_6t_init_gps(void) ;

void gps_write(char *buf, int size);

int lea_6t_nmea_check_ck(uint8_t *buffer, uint8_t len );
int lea_6t_get_nmea_msg(uint8_t *buffer);
void gps_nmea_parse( uint8_t c );

void request_RXM_RAW();
int lea_6t_get_ubx_msg();
void gps_ubx_parse( uint8_t c );
void gps_ubx_read_message(struct nav_sol_packet *gps);

void set_ubx_cfg_msg(u8t _class, u8t id, u8t rate1);


void setup();
void loop();
void configureUblox(u8t *settingsArrayPointer);
void calcChecksum(u8t *checksumPayload, u8t payloadSize);
void sendUBX(u8t *UBXmsg, u8t msgLength);
u8t getUBX_ACK(u8t *msgID);
void printHex(uint8_t *data, uint8_t length);
void setBaud(u8t baudSetting);


#endif /* LEA_6T_H_ */
