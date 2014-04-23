#ifndef __RTC_H
#define __RTC_H

#include "i2c.h"
#include "sys/process.h"

/*****************************************************************************************
 *  Driver defines
 ****************************************************************************************/

// Address
#define M41T62_I2C_ADDR	((0xD0)>>1)

// The registers are overloaded - in some cases, there is more than
// one function per register and in others one function is spread
// across multiple registers. All of which doesn't make for a nice
// clean set of defines
//
#define M41T62_REG_HUNDREDTHS 					0x00
#define M41T62_REG_SECONDS							0x01
#define M41T62_REG_MINUTES							0x02
#define M41T62_REG_HOURS 								0x03
#define M41T62_REG_DAYOFWEEK						0x04
#define M41T62_REG_DAY									0x05
#define M41T62_REG_MONTH 								0x06
#define M41T62_REG_YEAR 								0x07
#define M41T62_REG_CALIBRATION					0x08
#define M41T62_REG_WATCHDOG							0x09
#define M41T62_REG_ALARM_MONTH  				0x0A
#define M41T62_REG_ALARM_DAY    				0x0B
#define M41T62_REG_ALARM_HOUR   				0x0C
#define M41T62_REG_ALARM_MINUTE 				0x0D
#define M41T62_REG_ALARM_SECOND 				0x0E
#define M41T62_REG_FLAGS        				0x0F

#define M41T62_REG_ST		 								0x01
#define M41T62_REG_OFIE		 							0x02
#define M41T62_REG_CENTURY      				0x06
#define M41T62_REG_SQW_FREQ     				0x04
#define M41T62_REG_SQWE       					0x0A
#define M41T62_REG_AFE		       				0x0A
#define M41T62_REG_ALARM_RPT1   				0x0E
#define M41T62_REG_ALARM_RPT2   				0x0D
#define M41T62_REG_ALARM_RPT3   				0x0C
#define M41T62_REG_ALARM_RPT4   				0x0B
#define M41T62_REG_ALARM_RPT5   				0x0B

#define M41T62_HUNDREDTHS_MASK					0x0F
#define M41T62_TENTHS_MASK							0xF0
#define M41T62_SECONDS_MASK							0x0F
#define M41T62_10SECONDS_MASK						0x70
#define M41T62_MINUTES_MASK							0x0F
#define M41T62_10MINUTES_MASK						0x70
#define M41T62_HOURS_MASK								0x0F
#define M41T62_10HOURS_MASK							0x30
#define M41T62_DAYOFWEEK_MASK						0x07
#define M41T62_DAY_MASK									0x0F
#define M41T62_10DAY_MASK  							0x30
#define M41T62_MONTH_MASK								0x0F
#define M41T62_10MONTH_MASK							0x10
#define M41T62_YEAR_MASK								0x0F
#define M41T62_10YEAR_MASK							0xF0
#define M41T62_CENTURY_MASK							0xC0
#define M41T62_ALARM_SECONDS_MASK				0x0F
#define M41T62_ALARM_10SECONDS_MASK			0x70
#define M41T62_ALARM_MINUTES_MASK 			0x0F
#define M41T62_ALARM_10MINUTES_MASK 		0x70
#define M41T62_ALARM_HOUR_MASK					0x0F
#define M41T62_ALARM_10HOUR_MASK				0x30
#define M41T62_ALARM_DAY_MASK 					0x0F
#define M41T62_ALARM_10DAY_MASK 				0x30
#define M41T62_ALARM_MONTH_MASK					0x0F
#define M41T62_ALARM_10MONTH_MASK				0x10
#define M41T62_AF_MASK									0x40
#define M41T62_AFE_MASK									0x80
#define M41T62_AF_MASK									0x40
#define M41T62_OF_MASK									0x04
#define M41T62_OFIE_MASK								0x80
#define M41T62_OUT_MASK									0x80
#define M41T62_RPT1_MASK								0x80
#define M41T62_RPT2_MASK								0x80
#define M41T62_RPT3_MASK								0x80
#define M41T62_RPT4_MASK								0x80
#define M41T62_RPT5_MASK								0x40
#define M41T62_RS_MASK									0xF0
#define M41T62_S_MASK										0x20
#define M41T62_SQWE_MASK								0x40
#define M41T62_ST_MASK									0x80
#define M41T62_WDF_MASK									0x80
#define M41T62_REG_WATCHDOG_RB_MASK			0x83
#define M41T62_REG_WATCHDOG_RB0_MASK		0x01
#define M41T62_REG_WATCHDOG_RB1_MASK		0x02
#define M41T62_REG_WATCHDOG_RB2_MASK		0x80
#define M41T62_REG_WATCHDOG_BMB_MASK		0x7C

#define M41T62_GET_HUNDREDTHS(r)				((r&0x0F))
#define M41T62_GET_TENTHS(r)						((r&0xF0)>>4)
#define M41T62_GET_SECONDS(r)						((((r&0x70)>>4)*10) + (r&0x0F))
#define M41T62_GET_MINUTES(r)						((((r&0x70)>>4)*10) + (r&0x0F))
#define M41T62_GET_HOURS(r)							((((r&0x30)>>4)*10) + (r&0x0F))
#define M41T62_GET_DAYOFWEEK(r)					((r&0x07))
#define M41T62_GET_DAY(r)								((((r&0x30)>>4)*10) + (r&0x0F))
#define M41T62_GET_MONTH(r)							((((r&0x10)>>4)*10) + (r&0x0F))
#define M41T62_GET_YEAR(r)							((((r&0xF0)>>4)*10) + (r&0x0F))
#define M41T62_GET_CENTURY(r)						(((r&0xC0)>>6))
#define M41T62_GET_SQW_FREQ(r)					(((r&0xF0)>>4))
#define M41T62_GET_RB(r)								(((r&0x80)>>5) + (r&0x03))
#define M41T62_GET_BMB(r)								(((r&0x7C)>>2))

#define M41T62_SET_HUNDREDTHS(r)				((uint8_t)r&0x0F)
#define M41T62_SET_TENTHS(r)						(((uint8_t)r<<4)&0xF0)
#define M41T62_SET_SECONDS(r)						(((((uint8_t)r/10)<<4)&0x70) + ((uint8_t)r%10))
#define M41T62_SET_MINUTES(r)						(((((uint8_t)r/10)<<4)&0x70) + ((uint8_t)r%10))
#define M41T62_SET_SECONDS(r)						(((((uint8_t)r/10)<<4)&0x70) + ((uint8_t)r%10))
#define M41T62_SET_MINUTES(r)						(((((uint8_t)r/10)<<4)&0x70) + ((uint8_t)r%10))
#define M41T62_SET_HOURS(r)							(((((uint8_t)r/10)<<4)&0x30) + ((uint8_t)r%10))
#define M41T62_SET_DAYOFWEEK(r)					((r&0x07))
#define M41T62_SET_DAY(r)								(((((uint8_t)r/10)<<4)&0x30) + ((uint8_t)r%10))
#define M41T62_SET_MONTH(r)							(((((uint8_t)r/10)<<4)&0x10) + ((uint8_t)r%10))
#define M41T62_SET_YEAR(r)							(((((uint8_t)r/10)<<4)&0xF0) + ((uint8_t)r%10))
#define M41T62_SET_CENTURY(r)						(((uint8_t)r<<6)&0xC0)
#define M41T62_SET_SQW_FREQ(r)					((((uint8_t)r&0x0F)<<4))
#define M41T62_SET_RB(r)								((((uint8_t)r&0x04)<<5) + ((uint8_t)r&0x03))
#define M41T62_SET_BMB(r)								(((uint8_t)r&0x1F)<<2)
#define M41T62_SET_RPT1(r)							(((uint8_t)r&0x01)<<7)
#define M41T62_SET_RPT2(r)							(((uint8_t)r&0x01)<<7)
#define M41T62_SET_RPT3(r)							(((uint8_t)r&0x01)<<7)
#define M41T62_SET_RPT4(r)							(((uint8_t)r&0x01)<<7)
#define M41T62_SET_RPT5(r)							(((uint8_t)r&0x01)<<6)

#define M41T62_SQW_NONE									0x00
#define M41T62_SQW_32168Hz							0x01
#define M41T62_SQW_8192Hz								0x02
#define M41T62_SQW_4096Hz								0x03
#define M41T62_SQW_2048Hz								0x04
#define M41T62_SQW_1024Hz								0x05
#define M41T62_SQW_512Hz								0x06
#define M41T62_SQW_256Hz								0x07
#define M41T62_SQW_128Hz								0x08
#define M41T62_SQW_64Hz									0x09
#define M41T62_SQW_32Hz									0x0A
#define M41T62_SQW_16Hz									0x0B
#define M41T62_SQW_8Hz									0x0C
#define M41T62_SQW_4Hz									0x0D
#define M41T62_SQW_2Hz									0x0E
#define M41T62_SQW_1Hz									0x0F

#define M41T62_RPT_SECOND 							0x1F
#define M41T62_RPT_MINUTE 							0x1E
#define M41T62_RPT_HOUR   							0x1C
#define M41T62_RPT_DAY    							0x18
#define M41T62_RPT_MONTH  							0x10
#define M41T62_RPT_YEAR   							0x00

#define M41T62_WDT_16Hz									0x00 /* 16Hz    - 0.0625s period */
#define M41T62_WDT_4Hz 									0x01 /* 4Hz     - 0.25s period   */
#define M41T62_WDT_1Hz   								0x02 /* 1 Hz    - 1s period      */
#define M41T62_WDT_1_4Hz   							0x03 /* 1/4 Hz  - 4s period      */
#define M41T62_WDT_1_60Hz 							0x04 /* 1/60 Hz - 60s period     */

// Initial masks to clear all entries that should be
// zero, reset alarm and all flags. This will be
// anded into the register contents so a zero bit
// indicates that it should be cleared
//
#define M41T62_REG0_INIT_MASK						0x00
#define M41T62_REG1_INIT_MASK						0x80	// Leave stop bit alone
#define M41T62_REG2_INIT_MASK						0x00
#define M41T62_REG3_INIT_MASK						0x00
#define M41T62_REG4_INIT_MASK						0x00
#define M41T62_REG5_INIT_MASK						0x00
#define M41T62_REG6_INIT_MASK						0x00
#define M41T62_REG7_INIT_MASK						0x00
#define M41T62_REG8_INIT_MASK						0xBF	// Leave OUT, sign and calibration bits alone
#define M41T62_REG9_INIT_MASK						0x00
#define M41T62_REGA_INIT_MASK						0x00
#define M41T62_REGB_INIT_MASK						0x00
#define M41T62_REGC_INIT_MASK						0x00
#define M41T62_REGD_INIT_MASK						0x00
#define M41T62_REGE_INIT_MASK						0x00
#define M41T62_REGF_INIT_MASK						0x00

struct RTC_time {
	uint8_t		hundredths;
	uint8_t		tenths;
	uint8_t		seconds;
	uint8_t		minutes;
	uint8_t		hours;
	uint8_t		day;
	uint8_t		month;
	uint8_t		year;
};

struct RTC_alarm {
	uint8_t		seconds;
	uint8_t		minutes;
	uint8_t		hours;
	uint8_t		day;
	uint8_t		month;
};

enum {
	ALARM_EXPIRED
};

typedef enum {RPT_YEAR, RPT_MONTH, RPT_DAY, RPT_HOUR, RPT_MINUTE, RPT_SECOND} alarmRepeat;

// Function definitions
//
void RTC_init();
void RTC_getTime(struct RTC_time *t);
void RTC_setTime(struct RTC_time *t);
void RTC_getAlarm(struct RTC_alarm *t);
void RTC_setAlarm(struct RTC_alarm *t, void (*alarm_callback)(void), alarmRepeat rpt);
void RTC_clearAlarm();
void RTC_start();
void RTC_stop();
void RTC_resetRegisters();
void M41T62_readReg(uint8_t reg, uint8_t* data);
void M41T62_readRegN(uint8_t reg, uint8_t* data, uint8_t n);
void M41T62_writeReg(uint8_t writeAddr, uint8_t data);
void M41T62_writeRegN(uint8_t* data, uint8_t n);

PROCESS_NAME(RTC_process);

#endif /* __RTC_H */



