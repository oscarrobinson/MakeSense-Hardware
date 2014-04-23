/*
 * Orisen Ltd.
 * All rights reserved.
 * cfs using fat system - excluding coffee, just use fat system.
 */

#include "cfs/cfs.h"
#include "include/ff.h"
#include "include/ffconf.h"
#include "include/diskio.h"
#include "mc1322x.h"


struct filestate {
  int flag;
#define FLAG_FILE_CLOSED 0
#define FLAG_FILE_OPEN   1
  unsigned int fileno;
};

static FRESULT rc;
static FATFS fatfs;			/* File system object */
static FIL fil;				/* File object */
static DIR dir;				/* Directory object */
static FILINFO fno;			/* File information object */
static UINT bw;
static UINT br;

static BYTE rtcYear = 110, rtcMon = 10, rtcMday = 15, rtcHour, rtcMin, rtcSec;

#define DEBUG 0

#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

/*---------------------------------------------------------------------------*/
static void put_rc (FRESULT rc)
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

/*---------------------------------------------------------------------------*/
static void die (		/* Stop with dying message */
	FRESULT rc	/* FatFs return value */
)
{
	put_rc (rc);
	for (;;) ;
}

/*---------------------------------------------------------------------------*/
int cfs_open(const char *n, int f)
{
	UINT accessmode_app = (FA_OPEN_ALWAYS|FA_WRITE|FA_READ);
	UINT accessmode = (FA_CREATE_NEW|FA_WRITE|FA_READ);
	UINT accessmode_ow = (FA_CREATE_ALWAYS|FA_OPEN_ALWAYS|FA_WRITE|FA_READ);
	UINT file_exist = 0;
	FRESULT rc;
	DWORD ptr;

	PRINTF("DEBUG: OrisenPrime: cfs_open - flags = %d\r\n", f);

	rc = f_open(&fil, n, accessmode_app);
	PRINTF("DEBUG: -OrisenPrime: cfs_open: rc = %d\r\n", rc);
	if (rc == FR_OK) {
		PRINTF("file exists\r\n");
		file_exist = 1;
	}

	f_close(&fil);


	if(f & CFS_READ) {
		rc = f_open(&fil, n, FA_READ);
		PRINTF("DEBUG: --OrisenPrime: cfs_open: rc = %d\r\n", rc);
	}
	if((f & CFS_WRITE) || (f & CFS_APPEND)) {
		if((f & CFS_WRITE) || ((f & CFS_APPEND) && (file_exist == 0)) ){
			PRINTF("File over write=%d\r\n", fil.fptr);
			rc = f_open(&fil, n, accessmode_ow);
		}
		else if((f & CFS_APPEND) && (file_exist == 1))
		{
			//append file
			rc = f_open(&fil, n, accessmode_app);
			PRINTF("File append pointer=%d, size=%d\r\n", fil.fptr, fil.fsize);
			rc |= f_lseek(&fil,fil.fsize);

			PRINTF("File pointer=%d\r\n", fil.fptr);
			rc |=f_sync(&fil);
		}
	}
	PRINTF("DEBUG: ---OrisenPrime: cfs_open: rc = %d\r\n", rc);
	if (rc) {
		//die(rc);
		return -1;
	}
	put_rc(rc);
	return rc;

}

/*---------------------------------------------------------------------------*/
void
cfs_close(int f)
{
	PRINTF("closing file size %d\r\n", fil.fsize);
	rc = f_close(&fil);
}

/*---------------------------------------------------------------------------*/
int
cfs_read(int f, void *buf, unsigned int len)
{
	FRESULT rc;
// Jagun@UCL f_read takes care of this. No need for return code checking
// We will just return the number of bytes read, which could be 0
//	for (;;) {
		rc = f_read(&fil, buf, len, &br);	/* Read a chunk of file */
//		if (rc || !br) break;			/* Error or end of file */
//	}
	//if (rc) die(rc);

// Jagun@UCL If we close the file, br might be affected and
// files should be closed explicitly by cfs_close()
//	rc = f_close(&fil);
	//if (rc) die(rc);
	put_rc(rc);

//Jagun@UCL cfs_read is supposed to return the no. of bytes read
// NOT error code; See the Contiki API
//		return rc;
	return br;
}
/*---------------------------------------------------------------------------*/
int
cfs_write(int f, const void *buf, unsigned int len)
{
	UINT bw;
	char readbuf[5];
	FRESULT rc;

	rc=f_write(&fil, buf, len, &bw);
	if (bw != len) {
		printf("Error writing data packet, %u bytes written\n", bw);
		return -1;
	}
	//rc |= f_read(&fil, readbuf, 0, &bw);
	rc |=f_sync(&fil);
	put_rc(rc);
	// Jagun - the number of bytes written is supposed to be returned
	return 	bw;	
//	return rc;	// rather than error code- see the Contiki API
}
/*---------------------------------------------------------------------------*/
int
cfs_remove(const char *name)
{
  return 0;
}
/*---------------------------------------------------------------------------*/
cfs_offset_t
cfs_seek(int f, cfs_offset_t offset, int whence)
{
  cfs_offset_t new_offset;
	FRESULT rc;

  if (whence == CFS_SEEK_SET) {
    new_offset = offset;
  } else if (whence == CFS_SEEK_END) {
    new_offset = fil.fsize + offset;
  } else if (whence == CFS_SEEK_CUR) {
    new_offset = fil.fptr + offset;
  } else {
    return (cfs_offset_t)-1;
  }

  if (new_offset < 0 || new_offset > fil.fsize) {
    return -1;
  }
 
	PRINTF("File seek pointer=%d, offset=%d, size=%d\r\n", fil.fptr, new_offset, fil.fsize);
	rc = f_lseek(&fil, new_offset);

	return new_offset;
}
/*---------------------------------------------------------------------------*/
int
cfs_opendir(struct cfs_dir *p, const char *n)
{
  return -1;
}
/*---------------------------------------------------------------------------*/
int
cfs_readdir(struct cfs_dir *p, struct cfs_dirent *e)
{
  return -1;
}
/*---------------------------------------------------------------------------*/
void
cfs_closedir(struct cfs_dir *p)
{
}
/*---------------------------------------------------------------------------*/


