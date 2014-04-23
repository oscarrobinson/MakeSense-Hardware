//==============================================================================
//    Orisen micro-SDcard driver
//==============================================================================
// File      :  uSDcard.c
// Author    :  Venus Shum
//==============================================================================

#include "include/uSDcard.h"

volatile uint16_t file_number;
volatile uint8_t data_file_number;
volatile uint8_t data_file_count = 0;
volatile uint8_t data_packet_count = 0;
char file_str[11];

volatile uint16_t  Timer1, Timer2;	// from ChaN FF
FRESULT rc;
FATFS fatfs;			/* File system object */
FIL fil;					/* File object */
volatile BYTE rtcYear = 110, rtcMon = 10, rtcMday = 15, rtcHour, rtcMin, rtcSec;

DIR dir;					/* Directory object */
FILINFO fno;			/* File information object */
UINT bw;
UINT br;
//	UINT i;
//BYTE buff[120];		// was 128
uint8_t buff[60];

volatile uint8_t system_state = 0;
volatile uint8_t shadow_state = 0;
volatile uint8_t sensor_state = 0;
volatile uint8_t system_event = 0;
volatile uint8_t shadow_event = 0;
volatile uint8_t system_command = 0;
volatile uint8_t shadow_command = 0;

volatile uint8_t iir_state;
volatile uint8_t io_state;

/// why does it related to nvm??
uint8_t nvm_data[5];
nvmType_t type=0;
nvmErr_t err;

#define DEBUG 1
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif


void put_rc(FRESULT rc)
{
	const char *str =
		"OK\0" "DISK_ERR\0" "INT_ERR\0" "NOT_READY\0" "NO_FILE\0" "NO_PATH\0"
		"INVALID_NAME\0" "DENIED\0" "EXIST\0" "INVALID_OBJECT\0" "WRITE_PROTECTED\0"
		"INVALID_DRIVE\0" "NOT_ENABLED\0" "NO_FILE_SYSTEM\0" "MKFS_ABORTED\0" "TIMEOUT\0"
		"LOCKED\0" "NOT_ENOUGH_CORE\0" "TOO_MANY_OPEN_FILES\0";
	FRESULT i;

	for (i = 0; i != rc && *str; i++) {
		while (*str++) ;
	}
	if (rc != 0 ){
		PRINTF("rc=%u FR_%s\n", (UINT)rc, str);
	}
}

void die (		/* Stop with dying message */
	FRESULT rc	/* FatFs return value */
)
{
	put_rc (rc);
	for (;;) ;
}

DWORD get_fattime (void)
{
	DWORD tmr;

	/* Pack date and time into a DWORD variable */
	tmr =	  (((DWORD)rtcYear - 80) << 25)
			| ((DWORD)rtcMon << 21)
			| ((DWORD)rtcMday << 16)
			| (WORD)(rtcHour << 11)
			| (WORD)(rtcMin << 5)
			| (WORD)(rtcSec >> 1);

	return tmr;
}

void sd_card_open_data_file(void){
	//data_file_number++;
	sprintf(file_str,"test.DAT");
//	PRINTF("\nCreate a new file (DATA%03d.DAT)\n",data_file_number);
	rc = f_open(&fil, &file_str[0], FA_WRITE | FA_CREATE_ALWAYS);
	if (rc) die(rc);
	else {
		system_state = system_state | FILE_OPEN;
//		LED2_ON;
		nvm_data[0] = data_file_number;
		nvm_data[1] = system_state;
		nvm_data[2] = sensor_state;
		nvm_data[3] = system_event;
		nvm_data[4] = system_command;
		//err = nvm_erase(gNvmInternalInterface_c, type, 0x40000000); /* erase sector 30 --- sector 31 is the 'secret zone' */
		err=0;
		if (err != 0 ){
			PRINTF("nvm_erase returned: %x\n",err);
		}
		err = nvm_write(gNvmInternalInterface_c, type, nvm_data, NVM_DATA_ADDRESS, 5);
		if (err != 0 ){
			PRINTF("nvm_write returned: %x\n",err);
		}
	}
}

void sd_card_read_data_file(uint8_t file_number){
	sprintf(file_str,"test.DAT");
	PRINTF("\nOpen a file test.DAT\n");
	rc = f_open(&fil, &file_str[0], FA_READ);
	if (rc) die(rc);


	PRINTF("\nType the file content.\n");
	for (;;) {

		rc = f_read(&fil, buff, 3, &br);	/* Read a chunk of file */
		if (rc || !br) break;			/* Error or end of file */
		printf("%s", buff);
	}
	if (rc) die(rc);
	PRINTF("\nClose the file.\n");
	rc = f_close(&fil);
	if (rc) die(rc);

}

void set_fletcher_ck(uint8_t *buffer, uint8_t len) {
	uint8_t ck_a = 0;
	uint8_t ck_b = 0;
	uint8_t i = 0;
	for (i = 0; i < len; i++ ){
		ck_a = ck_a + buffer[i];
		ck_b = ck_b + ck_a;
	}
	buffer[len] = ck_a;
	buffer[len+1] = ck_b;
}

uint8_t check_fletcher_ck(uint8_t *buffer, uint8_t len ) {
	uint8_t ck_a = 0;
	uint8_t ck_b = 0;
	uint8_t i = 0;
	for (i = 0; i < (len - 4); i++ ){
		ck_a = ck_a + buffer[i+2];
		ck_b = ck_b + ck_a;
	}
	if ((ck_a == buffer[len-2]) && (ck_b == buffer[len-1])) return 1;
	else return 0;
}

void sd_card_close_data_file(void){
//	PRINTF("\nClose the file.\n");
	rc = f_close(&fil);
	if (rc) die(rc);
	system_state = system_state & ~FILE_OPEN;
}

void uSDcard_power_up(void)
{
	SD_PWR_EN_HIGH;								// Switch the SD card power on	
}

void uSDcard_power_down(void)
{
	SD_PWR_EN_LOW;								// Switch the SD card power off	
}

void uSDcard_init(void)
{
	uSDcard_power_up();
	put_rc(disk_initialize(0));		/* Initialise SD Card */
	put_rc(f_mount(0, &fatfs));
}

void test_write_data(void)
{
	char readData[3];

	sd_card_open_data_file();
	f_write(&fil, "abc", 3, &bw);
	if (bw != 3) {
		printf("Error writing data packet, %u bytes writen\n", bw);
	}

	sd_card_close_data_file();
	sd_card_read_data_file(1);
}


