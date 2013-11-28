#include "include/ff.h"
#include "include/ffconf.h"
#include "include/diskio.h"
#include "mc1322x.h"
#include "OrisenSpectrum.h"

void put_rc (FRESULT rc);
void die(FRESULT rc);
DWORD get_fattime (void);
void sd_card_open_data_file(void);
void sd_card_close_data_file(void);
void set_fletcher_ck(uint8_t *buffer, uint8_t len);
uint8_t check_fletcher_ck(uint8_t *buffer, uint8_t len );
void sd_card_read_data_file(uint8_t file_number);
uint8_t check_fletcher_ck(uint8_t *buffer, uint8_t len );
void set_fletcher_ck(uint8_t *buffer, uint8_t len);
void uSDcard_init(void);
void uSDcard_power_down(void);


// System State Defines
#define SLEEP_STATE			0x01
#define DATA_STATE			0x02
#define CHARGE_STATE		0x04
#define FILE_OPEN				0x08
#define RADIO_ON				0x10


// Node settings
#define NVM_DATA_ADDRESS	0x1E000

void test_write_data(void);
void uSDcard_init(void);
