/*
 * lea_6t.c
 *
 *  Created on: 1 Sep 2011
 *      Author: mcphillips
 *  Modified by Jagun Kwon @ UCL-CS (jagun.kwon@ucl.ac.uk)
 *  Last updated on: 22 Jan. 2013
 */

#include <stdio.h>
#include "i2c.h"
#include "include/lea_6t.h"
#include <string.h>
#include "clock.h"

#define DEBUG 1

#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

//extern volatile uint8_t timer_delay;

/**
 * The buffer, we store one nmea-line in
 * for parsing.
 */
#define NMEA_MAXLEN 255
char nmea_msg_buf[NMEA_MAXLEN];
int  nmea_msg_len = 0;

// This struct stores Ubx gps data temporarily when parsing
struct GpsUbx gps_ubx;

// This flag will be set to TRUE once a complete packet has been received.
// Used by gps_nmea_parse() below
volatile short gps_msg_received = 0;//FALSE;

#define I2C_TIMEOUT			20000

void i2c_transferred_timeout(uint16_t maxiteration)
{
	while(!i2c_transferred() && --maxiteration) /* Wait for transfer */ ;
	if(!maxiteration) {
		PRINTF("\nDEBUG: I2C Timeout Detected!!\n");
		i2c_force_reset();
		//i2c_disable();
		i2c_enable();	
	}

}

int lea_6t_get_bytes_available(void) {

	uint8_t request = LEA_6T_BYTES_AVAILABLE_HIGH ;
	uint8_t read[] = {0,0} ;
	uint16_t receive = 0;

	i2c_transmitinit( LEA_6T_I2C_ADDR, 1, &request ) ;
//	while(!i2c_transferred()) /* Wait for transfer */ ;
	i2c_transferred_timeout(I2C_TIMEOUT);

	i2c_receiveinit( LEA_6T_I2C_ADDR, 1, &read[0] ) ;
//	while(!i2c_transferred()) /* Wait for transfer */ ;
	i2c_transferred_timeout(I2C_TIMEOUT);

	request = LEA_6T_BYTES_AVAILABLE_LOW ;
	i2c_transmitinit( LEA_6T_I2C_ADDR, 1, &request ) ;
//	while(!i2c_transferred()) /* Wait for transfer */ ;
	i2c_transferred_timeout(I2C_TIMEOUT);

	i2c_receiveinit( LEA_6T_I2C_ADDR, 1, &read[1] ) ;
//	while(!i2c_transferred()) /* Wait for transfer */ ;
	i2c_transferred_timeout(I2C_TIMEOUT);
	receive = (read[0]<<8)|read[1];

//	if (receive){
//		PRINTF("Number of Bytes available: %d\n", receive) ;
//	}

	return receive;

}

void lea_6t_get_data_stream(uint8_t *buffer, uint16_t len) {

	uint8_t request = LEA_6T_DATA_STREAM ;

	i2c_transmitinit( LEA_6T_I2C_ADDR, 1, &request ) ;
	//while(!i2c_transferred()) /* Wait for transfer */ ;
	i2c_transferred_timeout(I2C_TIMEOUT);

	clock_delay_msec(6);

	i2c_receiveinit( LEA_6T_I2C_ADDR, len, buffer ) ;
	//while(!i2c_transferred()) /* Wait for transfer */ ;
	i2c_transferred_timeout(I2C_TIMEOUT);
}


// Implements the Ublox's UBIX checksum algorithm
// FIXME:Unless we have received a full packet, this function will not run.
// Will just return 0, so that the caller will not decode the buffer content
int lea_6t_check_ck(uint8_t *buffer, uint8_t len ) {
	uint8_t ck_a = 0;
	uint8_t ck_b = 0;
	uint8_t i = 0;

	for (i = 0; i < (len - 4); i++ ){
		ck_a = ck_a + buffer[i+2];
		ck_b = ck_b + ck_a;
	}

	//printf("ck_a %x cal_a %x ck_b %x cal_b %x ",buffer[0], buffer[1])
	//printf("sync1 %x sync2 %x class\n",buffer[0], buffer[1]);
	//DEBUG - for loop
	PRINTF("DEBUG: lea_6t_check_ck:Buffer contents:\n");
	for (i = 0; i < len; i++) {
			PRINTF("%x ",buffer[i]);
	}
	PRINTF("\n");

	if ((ck_a == buffer[len-2]) && (ck_b == buffer[len-1])) {
		return 1;
	} else {
		PRINTF("DEBUG: ck_a=%x, ck_b=%x, buffer[len-2]=%x, buffer[len-1]=%x\n",
			ck_a, ck_b, buffer[len-2], buffer[len-1]);
		return 0;
	}
}

// Implements the NMEA checksum calculation algorithm: Xor everything
int lea_6t_nmea_check_ck(uint8_t *buffer, uint8_t len ) {
	char checksum = 0;
	uint8_t ck_a = 0;
	uint8_t ck_b = 0;
	char chk_calculated[3];
	char chk_provided[3];
	uint8_t i = 0;

	// The last three bytes are * and checksums.
	for (i = 0; i < (len - 3); i++ ){
		checksum ^= buffer[i];
	}

	// DEBUG: Printing out the content of the NMEA packet
	for (i = 0; i < len; i++) {
			PRINTF("%c",buffer[i]);
	}
	PRINTF("\n");

	for (i = 0; i < len; i++) {
			PRINTF("0x%.2X ",buffer[i]);
	}
	PRINTF("\n");

	ck_a = checksum & 0xF0;
	ck_a >>= 4;
	ck_b = checksum & 0x0F;

//	printf("DEBUG: cs1=%x, cs2=%x, buffer[len-2]=%x, buffer[len-1]=%x\n",
//			ck_a, ck_b, buffer[len-2], buffer[len-1]);
	sprintf(chk_calculated, "%x%x",ck_a, ck_b);
	sprintf(chk_provided, "%c%c",buffer[len-2], buffer[len-1]);

	if (!strncasecmp(chk_calculated, chk_provided, 2)) {
//		printf("DEBUG: Checksum passed!\n");
		return 1;
	} else {
//		printf("DEBUG: Checksum failed.\n");
		return 0;
	}
}


// This function will return a complete, valid NMEA message OR
// return 0 in case of a checksum failure
// This should be used by user instead of lea_6t_get_data_stream
int lea_6t_get_nmea_msg(uint8_t *buffer) {

	uint8_t data[1];
	int len = 0;

	while (1) {
		lea_6t_get_data_stream(data, 1);	// read one byte each time
		gps_nmea_parse(data[0]);

        if (gps_msg_received == 1) {
			gps_msg_received = 0;

			PRINTF("DEBUG: nmea_msg_len = %d\n", nmea_msg_len);
			len = nmea_msg_len;
        	nmea_msg_len = 0;

			// A complete NMEA message received.
			// Perform a checksum check here.
			//if (lea_6t_check_ck(nmea_msg_buf, len)) {
			// Use NMEA-specific checksum (XOR)
			if (lea_6t_nmea_check_ck(nmea_msg_buf, len)) {
				// If checksum is OK, copy msg and return
				strncpy(buffer, nmea_msg_buf, len);
				return 1;	// Success
			} else {
				return 0;	// Checksum Failure
			}
		}
	}
}

void request_RXM_RAW() {
	// Poll for the Raw message here
	uint8_t req_RXM_RAW[] = 
			{ 0xB5, 0x62, 0x02, 0x10, 0x00, 0x00, 0x12, 0x38}; //Poll RXM_RAW
	//gps_write(req_RXM_RAW, sizeof(req_RXM_RAW));

	i2c_transmitinit( LEA_6T_I2C_ADDR, 8, req_RXM_RAW) ;
	while(!i2c_transferred()) ; // Wait for transfer 
}


// This function will store a complete, valid UBX message in gps_ubx
int lea_6t_get_ubx_msg() {

	uint8_t data[1];
	int		bytes;

//	PRINTF("Started reading a message... bytes available: %d\n",
//		lea_6t_get_bytes_available());

	while ((bytes = lea_6t_get_bytes_available()) > 0 &&
		gps_ubx.msg_available == FALSE) {

//		clock_delay_msec(100);

		lea_6t_get_data_stream(data, 1);// read one byte each time
// Alternative
//		i2c_receiveinit( LEA_6T_I2C_ADDR, 1, data) ;
//	   	while(!i2c_transferred()) /* Wait for transfer */ ;

		PRINTF("0x%.2X ",data[0]);

		gps_ubx_parse(data[0]);
	}

	if (gps_ubx.msg_available == TRUE) {
		PRINTF("JUST Got one msg!\n");
		return 1;
	} else
		return 0;
}


void lea_6t_decode_gps(struct nav_sol_packet *nsp, uint8_t *buffer) {
	nsp->header = buffer[0] + (buffer[1]<<8);
	nsp->id = buffer[2] + (buffer[3]<<8);
	PRINTF("DEBUG: header:%x id:%x\n", nsp->header, nsp->id);

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


void gps_write(char *buf, int size){
	int i;

// FIXME Somehow, this leads to lockups!!
	i2c_transmitinit( LEA_6T_I2C_ADDR, size, buf) ;
	while(!i2c_transferred()) ; // Wait for transfer 


/*
	for (i=0; i<size; i++) {
//		PRINTF(".");
		i2c_transmitinit( LEA_6T_I2C_ADDR, 1, &buf[i]) ;
		while(!i2c_transferred())	; // Wait for transfer ;
	}
//	PRINTF("\n");
*/
}


static void ubx_checksum(unsigned char *ck_a,
                        unsigned char *ck_b,
                        unsigned char *packet,
                        int size){
  u8t a, b;
  a = b = 0x00;
  while(size-- > 0){
    a += *(packet++);
    b += a;
  }
  *ck_a = a;
  *ck_b = b;
}


#define UBX_CFG_RATE_PS   (u16t)6
#define UBX_CFG_RATE_MEAS (u16t)250
#define UBX_CFG_RATE_NAV  (u16t)1
#define UBX_CFG_RAET_TIME (u16t)0

static void set_ubx_cfg_rate()
{
  unsigned char packet[14];
  packet[0] = 0xB5;
  packet[1] = 0x62;
  packet[2] = 0x06;
  packet[3] = 0x08;
  packet[4 + 0]  = (UBX_CFG_RATE_PS & 0xFF);
  packet[4 + 1]  = (UBX_CFG_RATE_PS >> 8) & 0xFF;
  packet[6 + 0]  = (UBX_CFG_RATE_MEAS & 0xFF);
  packet[6 + 1]  = (UBX_CFG_RATE_MEAS >> 8) & 0xFF;
  packet[6 + 2]  = (UBX_CFG_RATE_NAV & 0xFF);
  packet[6 + 3]  = (UBX_CFG_RATE_NAV >> 8) & 0xFF;
  packet[6 + 4]  = (UBX_CFG_RAET_TIME & 0xFF);
  packet[6 + 5]  = (UBX_CFG_RAET_TIME >> 8) & 0xFF;
  ubx_checksum(&(packet[sizeof(packet) - 2]), &(packet[sizeof(packet) - 1]), &(packet[2]), sizeof(packet) - 4);
  gps_write(packet, sizeof(packet));
}


#define UBX_CFG_TP_PS          (u16t)20
#define UBX_CFG_TP_INTERVAL    (u32t)1000000
#define UBX_CFG_TP_LENGTH      (u32t)1000
//#define UBX_CFG_TP_STATUS      (s8)-1
#define UBX_CFG_TP_STATUS      (i8t)-1
#define UBX_CFG_TP_TIMEREF     (u8t)0
#define UBX_CFG_TP_CABLE_DELAY (u16t)50
#define UBX_CFG_TP_RF_DELAY    (u16t)820
#define UBX_CFG_TP_USER_DELAY  (u32t)0

static void set_ubx_cfg_tp()
{
  unsigned char packet[28];
  packet[0] = 0xB5;
  packet[1] = 0x62;
  packet[2] = 0x06;
  packet[3] = 0x07;
  packet[4 + 0]  = (UBX_CFG_TP_PS & 0xFF);
  packet[4 + 1]  = (UBX_CFG_TP_PS >> 8) & 0xFF;
  packet[6 + 0]  = (UBX_CFG_TP_INTERVAL & 0xFF);
  packet[6 + 1]  = (UBX_CFG_TP_INTERVAL >> 8) & 0xFF;
  packet[6 + 2]  = (UBX_CFG_TP_INTERVAL >> 16) & 0xFF;
  packet[6 + 3]  = (UBX_CFG_TP_INTERVAL >> 24) & 0xFF;
  packet[6 + 4]  = (UBX_CFG_TP_LENGTH & 0xFF);
  packet[6 + 5]  = (UBX_CFG_TP_LENGTH >> 8) & 0xFF;
  packet[6 + 6]  = (UBX_CFG_TP_LENGTH >> 16) & 0xFF;
  packet[6 + 7]  = (UBX_CFG_TP_LENGTH >> 24) & 0xFF;
  packet[6 + 8]  = (UBX_CFG_TP_STATUS & 0xFF);
  packet[6 + 9]  = (UBX_CFG_TP_TIMEREF & 0xFF);
  packet[6 + 10] = 0x00;
  packet[6 + 11] = 0x00;
  packet[6 + 12] = (UBX_CFG_TP_CABLE_DELAY & 0xFF);
  packet[6 + 13] = (UBX_CFG_TP_CABLE_DELAY >> 8) & 0xFF;
  packet[6 + 14] = (UBX_CFG_TP_RF_DELAY & 0xFF);
  packet[6 + 15] = (UBX_CFG_TP_RF_DELAY >> 8) & 0xFF;
  packet[6 + 16] = (UBX_CFG_TP_USER_DELAY & 0xFF);
  packet[6 + 17] = (UBX_CFG_TP_USER_DELAY >> 8) & 0xFF;
  packet[6 + 18] = (UBX_CFG_TP_USER_DELAY >> 16) & 0xFF;
  packet[6 + 19] = (UBX_CFG_TP_USER_DELAY >> 24) & 0xFF;
  ubx_checksum(&(packet[sizeof(packet) - 2]), &(packet[sizeof(packet) - 1]), &(packet[2]), sizeof(packet) - 4);
  gps_write(packet, sizeof(packet));
}


#define UBX_CFG_PRT_PS        (u16t)20
//#define UBX_CFG_PRT_BAUDRATE  (u32t)115200
//#define UBX_CFG_PRT_BAUDRATE  (u32t)9600	// due to too many checksum failures
#define UBX_CFG_PRT_BAUDRATE  (u32t)57600
// set port configuration - DDC port (i2c)
static void set_ubx_cfg_prt()
{
	// Enable TX-ready feature
	// B5 62 06 00,14 00,00 00 01 00 00 00 00 00 00 00 00 00 07 00 03 00 00 00 00 00 25 A4
  // UBX

  unsigned char packet[UBX_CFG_PRT_PS + 8];
  packet[0] = 0xB5;
  packet[1] = 0x62;
  packet[2] = 0x06;
  packet[3] = 0x00;
  packet[4 + 0]  = (UBX_CFG_PRT_PS & 0xFF);
  packet[4 + 1]  = (UBX_CFG_PRT_PS >> 8) & 0xFF;
  packet[6 + 0]  = 0x00; // Port.NO
  packet[6 + 1]  = 0x00; // Res0
  packet[6 + 2]  = 0x01; // Res1	// used to be 00	TX-ready enabled
  packet[6 + 3]  = 0x00;
  packet[6 + 4]  = 0x00;//0xC0; // (M)11000000(L)
  packet[6 + 5]  = 0x00;//0x08; // (M)00001000(L)
  packet[6 + 6]  = 0x00; // (M)00000000(L)
  packet[6 + 7]  = 0x00; //
//  packet[6 + 8]  = 0x00;//(UBX_CFG_PRT_BAUDRATE & 0xFF);
//  packet[6 + 9]  = 0x00;//((UBX_CFG_PRT_BAUDRATE >> 8) & 0xFF);
//  packet[6 + 10] = 0x00;//((UBX_CFG_PRT_BAUDRATE >> 16) & 0xFF);
//  packet[6 + 11] = 0x00;//((UBX_CFG_PRT_BAUDRATE >> 24) & 0xFF);
  packet[6 + 8]  = (UBX_CFG_PRT_BAUDRATE & 0xFF);
  packet[6 + 9]  = ((UBX_CFG_PRT_BAUDRATE >> 8) & 0xFF);
  packet[6 + 10] = ((UBX_CFG_PRT_BAUDRATE >> 16) & 0xFF);
  packet[6 + 11] = ((UBX_CFG_PRT_BAUDRATE >> 24) & 0xFF);
  packet[6 + 12] = 0x07; // in - UBX,NMEA,RAW
  packet[6 + 13] = 0x00;
  packet[6 + 14] = 0x03; // out - UBX and NMEA?
//  packet[6 + 14] = 0x01; // out - UBX only
  packet[6 + 15] = 0x00;
  packet[6 + 16] = 0x00;
  packet[6 + 17] = 0x00;
  packet[6 + 18] = 0x00;
  packet[6 + 19] = 0x00;

/*
  ubx_checksum(&(packet[sizeof(packet) - 2]), &(packet[sizeof(packet) - 1]), &(packet[2]), sizeof(packet) - 4);
  gps_write(packet, sizeof(packet));
*/
/*
  {
    u8t i;
    for(i = 0; i < 3; i++){
      packet[6 + 0]  = i;
      ubx_checksum(&(packet[sizeof(packet) - 2]), &(packet[sizeof(packet) - 1]), &(packet[2]), sizeof(packet) - 4);
      gps_write(packet, sizeof(packet));

	  // Check ACK
      //getUBX_ACK(&packet[2]); //Passes Class ID and Message ID to the ACK Receive function
    }
  }
*/

  // NMEA
/*
  char *str1 = "$PUBX,41,0,0007,0003,115200,1*1A\r\n";
  char *str2 = "$PUBX,41,1,0007,0003,115200,1*1B\r\n";
  char *str3 = "$PUBX,41,2,0007,0003,115200,1*18\r\n";  

//  gps_write("$PUBX,41,0,0007,0003,115200,1*1A\r\n", strlen(str1));
  gps_write(str1, strlen(str1));  
//  gps_write("$PUBX,41,1,0007,0003,115200,1*1B\r\n", strlen(str1));
//  gps_write("$PUBX,41,2,0007,0003,115200,1*18\r\n", strlen(str1));
  gps_write(str2, strlen(str2));  
  gps_write(str3, strlen(str3));
*/

}


#define UBX_CFG_MSG_PS    (u16t)6

//static void set_ubx_cfg_msg(u8t _class, u8t id, u8t rate1)
void set_ubx_cfg_msg(u8t _class, u8t id, u8t rate1)
{
  unsigned char packet[14];
  packet[0] = 0xB5;
  packet[1] = 0x62;
  packet[2] = 0x06;
  packet[3] = 0x01;
  packet[4 + 0]  = (UBX_CFG_MSG_PS & 0xFF);
  packet[4 + 1]  = (UBX_CFG_MSG_PS >> 8) & 0xFF;
  packet[6 + 0]  = _class;
  packet[6 + 1]  = id;
  packet[6 + 2]  = rate1;//(u8)0;
  packet[6 + 3]  = rate1;
  packet[6 + 4]  = rate1;//(u8)0;
  packet[6 + 5]  = (u8t)0;
  ubx_checksum(&(packet[sizeof(packet) - 2]), &(packet[sizeof(packet) - 1]), &(packet[2]), sizeof(packet) - 4);
  gps_write(packet, sizeof(packet));

	int i;
	printf("DEBUG: CFG-MSG written...\n");
  for (i=0; i<14; i++) {
	printf("0x%0.2X ", packet[i]);
	}
	printf("\n");
}


void UBX_CalcSum(unsigned char *str,int len,unsigned char *sum)
{
    int i;
    *(sum + 0) = *(sum + 1) = 0;
    for(i = 0;i < len;i++){
        *(sum + 0) = *(sum + 0) + *(str+i);
        *(sum + 1) = *(sum + 1) + *(sum + 0);
    }

  	PRINTF("UBX checksum: %0.2X, %0.2X\n", *sum, *(sum+1));
}

// Ubx init
void gps_ubx_init(void) {
   gps_ubx.status = UNINIT;
   gps_ubx.msg_available = FALSE;
   gps_ubx.error_cnt = 0;
   gps_ubx.error_last = GPS_UBX_ERR_NONE;
   gps_ubx.have_velned = 0;
}


// Initialise the GPS module
void lea_6t_init_gps(void) {

	// Init UBX related data structures
	gps_ubx_init();

	set_ubx_cfg_prt();              // baudrate& port config
//  set_ubx_cfg_rate();
//  set_ubx_cfg_tp();

  set_ubx_cfg_msg(0xF0, 0x00, 0);  // Disable NMEA msgs
  set_ubx_cfg_msg(0xF0, 0x01, 0);  // Disable NMEA msgs
  set_ubx_cfg_msg(0xF0, 0x02, 0);  // Disable NMEA msgs
  set_ubx_cfg_msg(0xF0, 0x03, 0);  // Disable NMEA msgs
  set_ubx_cfg_msg(0xF0, 0x04, 0);  // Disable NMEA msgs
  set_ubx_cfg_msg(0xF0, 0x05, 0);  // Disable NMEA msgs
  set_ubx_cfg_msg(0xF0, 0x06, 0);  // Disable NMEA msgs
  set_ubx_cfg_msg(0xF0, 0x07, 0);  // Disable NMEA msgs
  set_ubx_cfg_msg(0xF0, 0x08, 0);  // GPZDA - to be disabled
  set_ubx_cfg_msg(0xF0, 0x09, 0);  // Disable NMEA msgs
  set_ubx_cfg_msg(0xF0, 0x0A, 0);  // Disable NMEA msgs
  set_ubx_cfg_msg(0xF0, 0x41, 0);  // Disable NMEA msgs
  set_ubx_cfg_msg(0xF0, 0x42, 0);  // Disable NMEA msgs

//  set_ubx_cfg_msg(0x01, 0x02, 1);  // NAV-POSLLH  // 28 + 8 = 36 bytes
//  set_ubx_cfg_msg(0x01, 0x03, 4);  // NAV-STATUS  // 16 + 8 = 24 bytes
//  set_ubx_cfg_msg(0x01, 0x04, 4);  // NAV-DOP     // 18 + 8 = 26 bytes
//  set_ubx_cfg_msg(0x01, 0x06, 1);  // NAV-SOL     // 52 + 8 = 60 bytes
  set_ubx_cfg_msg(0x01, 0x06, 0);  // NAV-SOL     // 52 + 8 = 60 bytes
//  set_ubx_cfg_msg(0x01, 0x12, 1);  // NAV-VELNED  // 36 + 8 = 44 bytes
//  set_ubx_cfg_msg(0x01, 0x30, 8);  // NAV-SVINFO  // (8 + 12 * x) + 8 = 112 bytes (@8)
//  set_ubx_cfg_msg(0x02, 0x10, 1);  // RXM-RAW     // (8 + 24 * x) + 8 = 208 bytes (@8)
//  set_ubx_cfg_msg(0x02, 0x11, 1);  // RXM-SFRB    // 42 + 8 = 50 bytes
  set_ubx_cfg_msg(0x02, 0x10, 0);  // RXM-RAW     // (8 + 24 * x) + 8 = 208 bytes (@8)
  set_ubx_cfg_msg(0x02, 0x11, 0);  // RXM-SFRB    // 42 + 8 = 50 bytes

/*
unsigned char chkStr[][11] = {
            { 0xB5,0x62,0x06,0x01,0x03,0x00,0x02,0x10,0x02,0xFF,0xFF }, //RXM-RAW (0x02 0x10)
            { 0xB5,0x62,0x06,0x01,0x03,0x00,0x02,0x30,0x02,0xFF,0xFF }, //RXM-ALM (0x02 0x30)
            { 0xB5,0x62,0x06,0x01,0x03,0x00,0x02,0x31,0x02,0xFF,0xFF }, //RXM-EPH (0x02 0x31)
            //{ 0xB5,0x62,0x06,0x01,0x03,0x00,0x02,0x41,0x02,0xFF,0xFF },
            { 0xB5,0x62,0x06,0x01,0x03,0x00,0x02,0x11,0x02,0xFF,0xFF }, //RXM-SFRB (0x02 0x11)
            { 0xB5,0x62,0x06,0x01,0x03,0x00,0x02,0x20,0x02,0xFF,0xFF }, //RXM-SVSI (0x02 0x20)
            { 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF }, //END
        };
int i,j;

		UBX_CalcSum(&set[0][2], 10, &set[0][12]);

        for(j=0;chkStr[j][0] != 0x00;j++){
            UBX_CalcSum(&chkStr[j][2],7,&chkStr[j][9]);
//  ubx_checksum(&(chkStr[sizeof(chkStr) - 2]), &(chkStr[sizeof(chkStr) - 1]), &(chkStr[2]), sizeof(chkStr) - 4);

            for(i = 0; i < sizeof(chkStr[0]);i++){

  				gps_write(&chkStr[j][i], 1);
//                gps.putc(chkStr[j][i]);

            }

//            printf("%d : SET UBX Rate : %s\n",j,(UBX_WaitAck(&UBXPacket) == 1) ? "ACK" : "NAK");
       }
*/

/*
// Save the current configuration (0 BBR, 1 FLASH, 2 i2c-EEPROM)
	uint8_t cfg_cfg[] = {0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x21, 0xAF};

	i2c_transmitinit( LEA_6T_I2C_ADDR, sizeof(cfg_cfg), cfg_cfg) ;
	while(!i2c_transferred()) ; // Wait for transfer 
	clock_delay_msec(100);
*/

// FOR testing Arduino's GPS solution
//	setup();
//	loop();

	i2c_enable();

}

/**
 * This is an NMEA parser.
 * It reads one character at a time
 * setting gps_msg_received to TRUE
 * after a full line.
 */
void gps_nmea_parse( uint8_t c ) {
// TODO this parser will also need to handle ubx format as well

  //reject empty lines
  if (nmea_msg_len == 0) {
     if (c == '\r' || c == '\n' || c == '$')
       return;
  }

  // fill the buffer, unless it's full
  if (nmea_msg_len < NMEA_MAXLEN - 1) {

      // messages end with a linefeed
      if (c == '\r' || c == '\n') {
        gps_msg_received = 1;
      } else {
        nmea_msg_buf[nmea_msg_len] = c;
        nmea_msg_len ++;
      }
  }

  if (nmea_msg_len >= NMEA_MAXLEN - 1)
     gps_msg_received = 1;
}


/* UBX parsing */
void gps_ubx_parse( uint8_t c ) {
  if (gps_ubx.status < GOT_PAYLOAD) {
    gps_ubx.ck_a += c;
    gps_ubx.ck_b += gps_ubx.ck_a;
  }
  switch (gps_ubx.status) {
  case UNINIT:
    if (c == UBX_SYNC1) {
      gps_ubx.status++;
	}
    break;
  case GOT_SYNC1:
    if (c != UBX_SYNC2) {
      gps_ubx.error_last = GPS_UBX_ERR_OUT_OF_SYNC;
      goto error;
    }
    gps_ubx.ck_a = 0;
    gps_ubx.ck_b = 0;
    gps_ubx.status++;
    break;
  case GOT_SYNC2:
    if (gps_ubx.msg_available) {
      /* Previous message has not yet been parsed: discard this one */
      gps_ubx.error_last = GPS_UBX_ERR_OVERRUN;
      goto error;
    }
    gps_ubx.msg_class = c;
    gps_ubx.status++;
    break;
  case GOT_CLASS:
    gps_ubx.msg_id = c;
    gps_ubx.status++;
    break;
  case GOT_ID:
    gps_ubx.len = c;
    gps_ubx.status++;
    break;
  case GOT_LEN1:
    gps_ubx.len |= (c<<8);
    if (gps_ubx.len > GPS_UBX_MAX_PAYLOAD) {
      gps_ubx.error_last = GPS_UBX_ERR_MSG_TOO_LONG;
      goto error;
    }
    gps_ubx.msg_idx = 0;
    gps_ubx.status++;
    break;
  case GOT_LEN2:
    gps_ubx.msg_buf[gps_ubx.msg_idx] = c;
    gps_ubx.msg_idx++;
    if (gps_ubx.msg_idx >= gps_ubx.len) {
      gps_ubx.status++;
    }
    break;
  case GOT_PAYLOAD:
    if (c != gps_ubx.ck_a) {
      gps_ubx.error_last = GPS_UBX_ERR_CHECKSUM;
PRINTF("DEBUG: Checksum-a failed\n");
      goto error;
    }
    gps_ubx.status++;
    break;
  case GOT_CHECKSUM1:
    if (c != gps_ubx.ck_b) {
PRINTF("DEBUG: Checksum-b failed\n");
      gps_ubx.error_last = GPS_UBX_ERR_CHECKSUM;
      goto error;
    }
PRINTF("DEBUG: MSG received\n");
    gps_ubx.msg_available = TRUE;
    goto restart;
    break;
  default:
    gps_ubx.error_last = GPS_UBX_ERR_UNEXPECTED;
PRINTF("DEBUG: UBX ERR UNEXPECTED\n");
    goto error;
  }
  return;
 error:
  gps_ubx.error_cnt++;
 restart:
  gps_ubx.status = UNINIT;

  // Added by Jagun
//  gps_ubx.ck_a = 0;
//  gps_ubx.ck_b = 0;
  return;
}

void gps_ubx_read_message(struct nav_sol_packet *gps) {

PRINTF("DEBUG: Received a msg with CLASS:0x%02X, ID:0x%02X\n", gps_ubx.msg_class, gps_ubx.msg_id);

  gps_ubx.msg_available = FALSE;

  if (gps_ubx.msg_class == UBX_NAV_ID) {
    if (gps_ubx.msg_id == UBX_NAV_SOL_ID) {
PRINTF("UBX_NAV_SOL_ID msg received\n");
#ifdef GPS_TIMESTAMP
      /* get hardware clock ticks */
/*
      SysTimeTimerStart(gps.t0);
      gps.t0_tow        = UBX_NAV_SOL_ITOW(gps_ubx.msg_buf);
      gps.t0_tow_frac   = UBX_NAV_SOL_Frac(gps_ubx.msg_buf);
*/
#endif
      gps->itow        = UBX_NAV_SOL_ITOW(gps_ubx.msg_buf);
      gps->week       = UBX_NAV_SOL_week(gps_ubx.msg_buf);
      gps->gpsfix        = UBX_NAV_SOL_GPSfix(gps_ubx.msg_buf);
      gps->ecef_x = UBX_NAV_SOL_ECEF_X(gps_ubx.msg_buf);
      gps->ecef_y = UBX_NAV_SOL_ECEF_Y(gps_ubx.msg_buf);
      gps->ecef_z = UBX_NAV_SOL_ECEF_Z(gps_ubx.msg_buf);
      gps->pacc       = UBX_NAV_SOL_Pacc(gps_ubx.msg_buf);
      gps->ecefvx = UBX_NAV_SOL_ECEFVX(gps_ubx.msg_buf);
      gps->ecefvy = UBX_NAV_SOL_ECEFVY(gps_ubx.msg_buf);
      gps->ecefvz = UBX_NAV_SOL_ECEFVZ(gps_ubx.msg_buf);
      gps->sacc       = UBX_NAV_SOL_Sacc(gps_ubx.msg_buf);
      gps->pdop       = UBX_NAV_SOL_PDOP(gps_ubx.msg_buf);
      gps->numsv     = UBX_NAV_SOL_numSV(gps_ubx.msg_buf);
    } else if (gps_ubx.msg_id == UBX_NAV_POSLLH_ID) {

PRINTF("UBX_NAV_POSLLH_ID msg received\n");
/*
      gps.lla_pos.lat = RadOfDeg(UBX_NAV_POSLLH_LAT(gps_ubx.msg_buf));
      gps.lla_pos.lon = RadOfDeg(UBX_NAV_POSLLH_LON(gps_ubx.msg_buf));
      gps.lla_pos.alt = UBX_NAV_POSLLH_HEIGHT(gps_ubx.msg_buf);
      gps.hmsl        = UBX_NAV_POSLLH_HMSL(gps_ubx.msg_buf);
#if GPS_USE_LATLONG
      // Computes from (lat, long) in the referenced UTM zone
      struct LlaCoor_f lla_f;
      lla_f.lat = ((float) gps.lla_pos.lat) / 1e7;
      lla_f.lon = ((float) gps.lla_pos.lon) / 1e7;
      struct UtmCoor_f utm_f;
      utm_f.zone = nav_utm_zone0;
      // convert to utm
      utm_of_lla_f(&utm_f, &lla_f);
      // copy results of utm conversion
      gps.utm_pos.east = utm_f.east*100;
      gps.utm_pos.north = utm_f.north*100;
      gps.utm_pos.alt = gps.lla_pos.alt;
      gps.utm_pos.zone = nav_utm_zone0;
#else
*/
    }
    else if (gps_ubx.msg_id == UBX_NAV_POSUTM_ID) {
PRINTF("UBX_NAV_POSUTM_ID msg received\n");
/*
      gps.utm_pos.east = UBX_NAV_POSUTM_EAST(gps_ubx.msg_buf);
      gps.utm_pos.north = UBX_NAV_POSUTM_NORTH(gps_ubx.msg_buf);
      uint8_t hem = UBX_NAV_POSUTM_HEM(gps_ubx.msg_buf);
      if (hem == UTM_HEM_SOUTH)
        gps.utm_pos.north -= 1000000000; // Subtract false northing: -10000km
      gps.utm_pos.alt = UBX_NAV_POSUTM_ALT(gps_ubx.msg_buf)*10;
      gps.hmsl = gps.utm_pos.alt;
      gps.lla_pos.alt = gps.utm_pos.alt; // FIXME: with UTM only you do not receive ellipsoid altitude
      gps.utm_pos.zone = UBX_NAV_POSUTM_ZONE(gps_ubx.msg_buf);
#endif
*/
    }
    else if (gps_ubx.msg_id == UBX_NAV_VELNED_ID) {
PRINTF("UBX_NAV_VELNED_ID msg received\n");
/*
      gps.speed_3d = UBX_NAV_VELNED_Speed(gps_ubx.msg_buf);
      gps.gspeed = UBX_NAV_VELNED_GSpeed(gps_ubx.msg_buf);
      gps.ned_vel.x = UBX_NAV_VELNED_VEL_N(gps_ubx.msg_buf);
      gps.ned_vel.y = UBX_NAV_VELNED_VEL_E(gps_ubx.msg_buf);
      gps.ned_vel.z = UBX_NAV_VELNED_VEL_D(gps_ubx.msg_buf);
      // Ublox gives I4 heading in 1e-5 degrees, apparenty from 0 to 360 degrees (not -180 to 180)
      // I4 max = 2^31 = 214 * 1e5 * 100 < 360 * 1e7: overflow on angles over 214 deg -> casted to -214 deg
      // solution: First to radians, and then scale to 1e-7 radians
      // First x 10 for loosing less resolution, then to radians, then multiply x 10 again
      gps.course = (RadOfDeg(UBX_NAV_VELNED_Heading(gps_ubx.msg_buf)*10)) * 10;
      gps.cacc = (RadOfDeg(UBX_NAV_VELNED_CAcc(gps_ubx.msg_buf)*10)) * 10;
      gps.tow = UBX_NAV_VELNED_ITOW(gps_ubx.msg_buf);
      gps_ubx.have_velned = 1;
*/
    }
    else if (gps_ubx.msg_id == UBX_NAV_SVINFO_ID) {
PRINTF("UBX_NAV_SVINFO_ID msg received\n");
/*
      gps.nb_channels = Min(UBX_NAV_SVINFO_NCH(gps_ubx.msg_buf), GPS_NB_CHANNELS);
      uint8_t i;
      for(i = 0; i < gps.nb_channels; i++) {
        gps.svinfos[i].svid = UBX_NAV_SVINFO_SVID(gps_ubx.msg_buf, i);
        gps.svinfos[i].flags = UBX_NAV_SVINFO_Flags(gps_ubx.msg_buf, i);
        gps.svinfos[i].qi = UBX_NAV_SVINFO_QI(gps_ubx.msg_buf, i);
        gps.svinfos[i].cno = UBX_NAV_SVINFO_CNO(gps_ubx.msg_buf, i);
        gps.svinfos[i].elev = UBX_NAV_SVINFO_Elev(gps_ubx.msg_buf, i);
        gps.svinfos[i].azim = UBX_NAV_SVINFO_Azim(gps_ubx.msg_buf, i);
      }
*/
    }
    else if (gps_ubx.msg_id == UBX_NAV_STATUS_ID) {
PRINTF("UBX_NAV_STATUS_ID msg received\n");
      gps->gpsfix = UBX_NAV_STATUS_GPSfix(gps_ubx.msg_buf);
      gps_ubx.status_flags = UBX_NAV_STATUS_Flags(gps_ubx.msg_buf);
      gps_ubx.sol_flags = UBX_NAV_SOL_Flags(gps_ubx.msg_buf);
    }
  }
  // NOW, checking for RXM-RAW and child packets
  else if (gps_ubx.msg_class == UBX_RXM_ID) {
	PRINTF("UBX_RXM_ID msg received\n");
	if (gps_ubx.msg_id == UBX_RXM_RAW_ID) {
	PRINTF("UBX_RXM_RAW_ID msg received\n");
	}
  }

}



// TODO: implement Contiki specific sensor interface, i.e., value, configure, status functionsx




/* ------------------------------------ */
// TODO The code below is from http://playground.arduino.cc/UBlox/GPS

//boolean gpsStatus[] = {false, false, false, false, false, false, false};
u8t gpsStatus[] = {FALSE, FALSE, FALSE, FALSE, FALSE, FALSE, FALSE};
unsigned long start;

//HardwareSerial gpsSerial(Serial1);

void setup()
{
//  gpsSerial.begin(9600); 
  // START OUR SERIAL DEBUG PORT
//  Serial.begin(115200);
  //
  //Settings Array contains the following settings: [0]NavMode, [1]DataRate1, [2]DataRate2, [3]PortRateByte1, [4]PortRateByte2, [5]PortRateByte3, 
  //[6]NMEA GLL Sentence, [7]NMEA GSA Sentence, [8]NMEA GSV Sentence, [9]NMEA RMC Sentence, [10]NMEA VTG Sentence
  //NavMode: 
  //Pedestrian Mode    = 0x03
  //Automotive Mode    = 0x04
  //Sea Mode           = 0x05
  //Airborne < 1G Mode = 0x06
  //
  //DataRate:
  //1Hz     = 0xE8 0x03
  //2Hz     = 0xF4 0x01
  //3.33Hz  = 0x2C 0x01
  //4Hz     = 0xFA 0x00
  //
  //PortRate:
  //4800   = C0 12 00
  //9600   = 80 25 00
  //19200  = 00 4B 00  **SOFTWARESERIAL LIMIT FOR ARDUINO UNO R3!**
  //38400  = 00 96 00  **SOFTWARESERIAL LIMIT FOR ARDUINO MEGA 2560!**
  //57600  = 00 E1 00
  //115200 = 00 C2 01
  //230400 = 00 84 03
  //
  //NMEA Messages: 
  //OFF = 0x00
  //ON  = 0x01
  //
  u8t settingsArray[] = {0x03, 0xFA, 0x00, 0x00, 0xE1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; //
  configureUblox(settingsArray); 
}

void loop()
{
	u8t incoming_char;

  while(1) {
	if (lea_6t_get_bytes_available()) 	// Jagun
//    if(gpsSerial.available())
    {
    // THIS IS THE MAIN LOOP JUST READS IN FROM THE GPS SERIAL AND ECHOS OUT TO THE ARDUINO SERIAL.
    //Serial.write(gpsSerial.read()); 
		lea_6t_get_data_stream(&incoming_char, 1);	// Jagun
		printf("0x%0.2X ", incoming_char);
    }
	} 
}   


void configureUblox(u8t *settingsArrayPointer) {
  u8t gpsSetSuccess = 0;
  printf("Configuring u-Blox GPS initial state...\n");

  //Generate the configuration string for Navigation Mode
  u8t setNav[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, *settingsArrayPointer, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  calcChecksum(&setNav[2], sizeof(setNav) - 4);

  //Generate the configuration string for Data Rate
  u8t setDataRate[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, settingsArrayPointer[1], settingsArrayPointer[2], 0x01, 0x00, 0x01, 0x00, 0x00, 0x00};
  calcChecksum(&setDataRate[2], sizeof(setDataRate) - 4);

  //Generate the configuration string for Baud Rate
  u8t setPortRate[] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, settingsArrayPointer[3], settingsArrayPointer[4], settingsArrayPointer[5], 0x00, 0x07, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  calcChecksum(&setPortRate[2], sizeof(setPortRate) - 4);

  u8t setGLL[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x2B};
  u8t setGSA[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x32};
  u8t setGSV[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x39};
  u8t setRMC[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x04, 0x40};
  u8t setVTG[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x46};

  clock_delay_msec(2500);
  //delay(2500);

  while(gpsSetSuccess < 3)
  {
    printf("Setting Navigation Mode... \n");
    //sendUBX(&setNav[0], sizeof(setNav));  //Send UBX Packet
    sendUBX(setNav, sizeof(setNav));  //Send UBX Packet
    gpsSetSuccess += getUBX_ACK(&setNav[2]); //Passes Class ID and Message ID to the ACK Receive function
    if (gpsSetSuccess == 5) {
      gpsSetSuccess -= 4;
      setBaud(settingsArrayPointer[4]);
      clock_delay_msec(1000);
      u8t lowerPortRate[] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x80, 0x25, 0x00, 0x00, 0x07, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA2, 0xB5};
      sendUBX(lowerPortRate, sizeof(lowerPortRate));
//      gpsSerial.begin(9600);
      clock_delay_msec(1000);      
    }
    if(gpsSetSuccess == 6) gpsSetSuccess -= 4;
    if (gpsSetSuccess == 10) gpsStatus[0] = true;
  }
  if (gpsSetSuccess == 3) printf("Navigation mode configuration failed.");
  gpsSetSuccess = 0;
  while(gpsSetSuccess < 3) {
    printf("Setting Data Update Rate... \n");
    sendUBX(&setDataRate[0], sizeof(setDataRate));  //Send UBX Packet
    gpsSetSuccess += getUBX_ACK(&setDataRate[2]); //Passes Class ID and Message ID to the ACK Receive function      
    if (gpsSetSuccess == 10) gpsStatus[1] = true;
    if (gpsSetSuccess == 5 | gpsSetSuccess == 6) gpsSetSuccess -= 4;
  }
  if (gpsSetSuccess == 3) printf("Data update mode configuration failed.");
  gpsSetSuccess = 0;


  while(gpsSetSuccess < 3 && settingsArrayPointer[6] == 0x00) {
    printf("Deactivating NMEA GLL Messages \n");
    sendUBX(setGLL, sizeof(setGLL));
    gpsSetSuccess += getUBX_ACK(&setGLL[2]);
    if (gpsSetSuccess == 10) gpsStatus[2] = true;
    if (gpsSetSuccess == 5 | gpsSetSuccess == 6) gpsSetSuccess -= 4;
  }
  if (gpsSetSuccess == 3) printf("NMEA GLL Message Deactivation Failed!");
  gpsSetSuccess = 0;

  while(gpsSetSuccess < 3 && settingsArrayPointer[7] == 0x00) {
    printf("Deactivating NMEA GSA Messages \n");
    sendUBX(setGSA, sizeof(setGSA));
    gpsSetSuccess += getUBX_ACK(&setGSA[2]);
    if (gpsSetSuccess == 10) gpsStatus[3] = true;
    if (gpsSetSuccess == 5 | gpsSetSuccess == 6) gpsSetSuccess -= 4;
  }
  if (gpsSetSuccess == 3) printf("NMEA GSA Message Deactivation Failed!");  
  gpsSetSuccess = 0;

  while(gpsSetSuccess < 3 && settingsArrayPointer[8] == 0x00) {
    printf("Deactivating NMEA GSV Messages \n");
    sendUBX(setGSV, sizeof(setGSV));
    gpsSetSuccess += getUBX_ACK(&setGSV[2]);
    if (gpsSetSuccess == 10) gpsStatus[4] = true;
    if (gpsSetSuccess == 5 | gpsSetSuccess == 6) gpsSetSuccess -= 4;
  }
  if (gpsSetSuccess == 3) printf("NMEA GSV Message Deactivation Failed!\n");  
  gpsSetSuccess = 0;

  while(gpsSetSuccess < 3 && settingsArrayPointer[9] == 0x00) {
    printf("Deactivating NMEA RMC Messages \n");
    sendUBX(setRMC, sizeof(setRMC));
    gpsSetSuccess += getUBX_ACK(&setRMC[2]);
    if (gpsSetSuccess == 10) gpsStatus[5] = true;
    if (gpsSetSuccess == 5 | gpsSetSuccess == 6) gpsSetSuccess -= 4;
  }
  if (gpsSetSuccess == 3) printf("NMEA RMC Message Deactivation Failed!");  
  gpsSetSuccess = 0;

  while(gpsSetSuccess < 3 && settingsArrayPointer[10] == 0x00) {
    printf("Deactivating NMEA VTG Messages \n");
    sendUBX(setVTG, sizeof(setVTG));
    gpsSetSuccess += getUBX_ACK(&setVTG[2]);
    if (gpsSetSuccess == 10) gpsStatus[6] = true;
    if (gpsSetSuccess == 5 | gpsSetSuccess == 6) gpsSetSuccess -= 4;
  }
  if (gpsSetSuccess == 3) printf("NMEA VTG Message Deactivation Failed!\n");

  gpsSetSuccess = 0;
  if (settingsArrayPointer[4] != 0x25) {
    printf("Setting Port Baud Rate... \n");
    sendUBX(&setPortRate[0], sizeof(setPortRate));
    setBaud(settingsArrayPointer[4]);
    printf("Success!\n");
    clock_delay_msec(500);
  }
}


void calcChecksum(u8t *checksumPayload, u8t payloadSize) {
  u8t CK_A = 0, CK_B = 0;
  int i = 0;
  for (i = 0; i < payloadSize ;i++) {
    CK_A = CK_A + *checksumPayload;
    CK_B = CK_B + CK_A;
    checksumPayload++;
  }
  *checksumPayload = CK_A;
  checksumPayload++;
  *checksumPayload = CK_B;
}

void sendUBX(u8t *UBXmsg, u8t msgLength) {
  int i;
//  for(i = 0; i < msgLength; i++) {
	gps_write(UBXmsg, msgLength);		// Jagun
//    gpsSerial.write(UBXmsg[i]);
//    gpsSerial.flush();
//  }
	gps_write("\r\n", 2);
//  gpsSerial.println();
//  gpsSerial.flush();
}


u8t getUBX_ACK(u8t *msgID) {
  u8t CK_A = 0, CK_B = 0;
  u8t incoming_char[2];
//  boolean headerReceived = false;
//  unsigned long ackWait = millis();
  unsigned long ackWait = (unsigned long)clock_time();
  u8t ackPacket[10] = {0xB5, 0x62, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  int i = 0;
  while (1) {
//    if (gpsSerial.available()) {
	if (lea_6t_get_bytes_available()) {	// Jagun
		lea_6t_get_data_stream(incoming_char, 1);	// Jagun
//      incoming_char = gpsSerial.read();

		printf("0x%0.2X ", incoming_char[0]);
		//printf("%c", incoming_char[0]);

      if (incoming_char[0] == ackPacket[i]) {
        i++;
      }
      else if (i > 2) {
        ackPacket[i] = incoming_char[0];
        i++;
      }
    }
    if (i > 9) break;
    if (((unsigned long)clock_time() - ackWait) > CLOCK_SECOND * 1.2) {
//    if ((millis() - ackWait) > 1500) {
      printf("ACK Timeout\n");
      return 5;
    }
    if (i == 4 && ackPacket[3] == 0x00) {
      printf("NAK Received\n");
      return 1;
    }
  }

  for (i = 2; i < 8 ;i++) {
  CK_A = CK_A + ackPacket[i];
  CK_B = CK_B + CK_A;
  }
  if (msgID[0] == ackPacket[6] && msgID[1] == ackPacket[7] && CK_A == ackPacket[8] && CK_B == ackPacket[9]) {
    printf("Success!\n");
    printf("ACK Received! ");
    printHex(ackPacket, sizeof(ackPacket));
    return 10;
	}
  else {
    printf("ACK Checksum Failure: ");
    printHex(ackPacket, sizeof(ackPacket));
    clock_delay_msec(1000);
    return 1;
  }
}


void printHex(uint8_t *data, uint8_t length) // prints 8-bit data in hex
{
  char tmp[length*2+1];
  u8t first ;
  int j=0;
  u8t i;

  for (i = 0; i < length; i++) 
  {
    first = (data[i] >> 4) | 48;
    if (first > 57) tmp[j] = first + (u8t)7;
    else tmp[j] = first ;
    j++;

    first = (data[i] & 0x0F) | 48;
    if (first > 57) tmp[j] = first + (u8t)7; 
    else tmp[j] = first;
    j++;
  }
  tmp[length*2] = 0;
  for (i = 0, j = 0; i < sizeof(tmp); i++) {
    //Serial.print(tmp[i]);
    //putchar(tmp[i]);	// Doesn't work! TODO
    printf("%c", tmp[i]);
    if (j == 1) {
      printf(" "); 
      j = 0;
    }
    else j++;
  }
  printf("\n");
}

void setBaud(u8t baudSetting) {
// TODO Skip for now
/*
  if (baudSetting == 0x12) gpsSerial.begin(4800);
  if (baudSetting == 0x4B) gpsSerial.begin(19200);
  if (baudSetting == 0x96) gpsSerial.begin(38400);
  if (baudSetting == 0xE1) gpsSerial.begin(57600);
  if (baudSetting == 0xC2) gpsSerial.begin(115200);
  if (baudSetting == 0x84) gpsSerial.begin(230400);
*/
}
