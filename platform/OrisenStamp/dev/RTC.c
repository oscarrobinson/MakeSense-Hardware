/*
 * Driver for the Orisen Stamp real time clock chip
 * This is an M41T62 Serial real-time clock (RTC) with alarm
 *
 * Author: Stephen Hailes
 *
 */

#include <sys/clock.h>
#include "include/RTC.h"
#include "mc1322x.h"
#include <signal.h>


#define DEBUG 0

#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

PROCESS(RTC_process, "RTC process");

static void print_regs(char *str);

static void (*alarmCallback)(void)     = NULL;
static void (*watchdogCallback)(void)  = NULL;
static struct process *alarmProcess    = NULL;
static struct process *watchdogProcess = NULL;

void RTC_init()
{	uint8_t buf;

  // At the point we power up, the initial values for the registers are:
  // ST       = 0			-- we're not stopped, but OF is set because we just powered up
  // OF       = 1			-- attempt to reset this, until 4 seconds have elapsed, then cycle ST
  // OFIE     = 0			-- Oscillator fail interrupt enable.
  // OUT      = 1			-- This will be the value on the IRQ/OUT pin cos OFIE, AFE and WD are all zero
  // AFE      = 0			-- Alarm enable
  // SQWE     = 1			-- Square wave enable
  // RS[3:0]  = 0x01	-- Frequency of square wave = 32.768 kHz
  // WATCHDOG = 0			-- Watchdog off. Leave it off, because we have our own if wanted.

	// To set these registers, we need to make the NON-time related changes before
  // setting the time. Any write to the registers containing time info stops the clock
  //

	// Set stop flag ST to one and back as per datasheet advice to kick the ocscillator
  // First, read the register
	RTC_stop();
  RTC_start();

  // Turn the square wave off for now.
  //
  // First, read the register continaing the SQWE bit into buf
	M41T62_readReg(M41T62_REG_SQWE, &buf);
  //
  // Now clear the bit and send the rest back
	buf &= ~M41T62_SQWE_MASK;
	M41T62_writeReg(M41T62_REG_SQWE, buf);

  // Clear the oscillator fail (OF) flag. As per the datasheet,
	// we try this repeatedly until 4 seconds have elapsed and
  // then we blip the ST bit to one and zero in short succession
	// and try again
  //
  int done = 0;
  while (!done) {
  	int now  = clock_time();

		while ((clock_time() - now) < 4 * CLOCK_CONF_SECOND) {
			// Set the OF bit to zero
		  // First, read the flags register
			M41T62_readReg(M41T62_REG_FLAGS, &buf);
      //
			// If the bit is zero, we're done
			if ((buf & M41T62_OF_MASK) > 0) {
				done = 1;
				break;
			}
			//
			// Now set it to zero
			buf &= ~M41T62_OF_MASK;
			M41T62_writeReg(M41T62_REG_FLAGS, buf);			
		}

		if (done)
      break;

		RTC_stop();
		RTC_start();
	}

  // Clear all the bits in the registers that should be zero
  //
  RTC_resetRegisters();
  print_regs("Initial");

  enable_irq_kbi(5);			// RTC IRQ
}

// Get current time.
//
void RTC_getTime(struct RTC_time *t)
{ uint8_t buf[] = {0, 0, 0, 0, 0, 0, 0, 0};
	M41T62_readRegN(M41T62_REG_HUNDREDTHS, buf, 8);
  t->hundredths = M41T62_GET_HUNDREDTHS(buf[0]);
  t->tenths     = M41T62_GET_TENTHS(buf[0]);
  t->seconds    = M41T62_GET_SECONDS(buf[1]);
  t->minutes    = M41T62_GET_MINUTES(buf[2]);
  t->hours      = M41T62_GET_HOURS(buf[3]);
  t->day        = M41T62_GET_DAY(buf[5]);
  t->month      = M41T62_GET_MONTH(buf[6]);
  t->year       = M41T62_GET_YEAR(buf[7]);
}

void RTC_setTime(struct RTC_time *t)
{ static uint8_t rxbuf[] = {0, 0, 0, 0, 0, 0, 0, 0};
  static uint8_t txbuf[] = {0, 0, 0, 0, 0, 0, 0, 0, 0};

  // Read the registers
	M41T62_readRegN(M41T62_REG_HUNDREDTHS, rxbuf, 8);

  // Clear out the bits we want to change
  rxbuf[0] &= 0;
  rxbuf[1] &= ~(M41T62_10SECONDS_MASK | M41T62_SECONDS_MASK);
  rxbuf[2] &= ~(M41T62_10MINUTES_MASK | M41T62_MINUTES_MASK);
  rxbuf[3] &= ~(M41T62_10HOURS_MASK   | M41T62_HOURS_MASK);
  rxbuf[5] &= ~(M41T62_10DAY_MASK     | M41T62_DAY_MASK);
  rxbuf[6] &= ~(M41T62_10MONTH_MASK   | M41T62_MONTH_MASK);
  rxbuf[7] &= 0;

  // Set the bits as we want them. Note that we
  // must construct a transmit buffer that contains
  // the register number as the first element, followed
  // by the data.
  //
  txbuf[0] = M41T62_REG_HUNDREDTHS;
  txbuf[1] = 0;		// Always reset to zero by chip anyway
  txbuf[2] = rxbuf[1] | M41T62_SET_SECONDS(t->seconds);
  txbuf[3] = rxbuf[2] | M41T62_SET_MINUTES(t->minutes);
  txbuf[4] = rxbuf[3] | M41T62_SET_HOURS(t->hours);
  txbuf[5] = rxbuf[4];
  txbuf[6] = rxbuf[5] | M41T62_SET_DAY(t->day);
  txbuf[7] = rxbuf[6] | M41T62_SET_MONTH(t->month);
  txbuf[8] = rxbuf[7] | M41T62_SET_YEAR(t->year);

  // And write them back
  //
  M41T62_writeRegN(txbuf, 9);
  print_regs("Set time");
}

void RTC_getAlarm(struct RTC_alarm *t)
{ static uint8_t rxbuf[] = {0, 0, 0, 0, 0};
  
  // Read the registers
	M41T62_readRegN(M41T62_REG_ALARM_MONTH, rxbuf, 5);

  t->seconds    = M41T62_GET_SECONDS(rxbuf[4]);
  t->minutes    = M41T62_GET_MINUTES(rxbuf[3]);
  t->hours      = M41T62_GET_HOURS(rxbuf[2]);
  t->day        = M41T62_GET_DAY(rxbuf[1]);
  t->month      = M41T62_GET_MONTH(rxbuf[0]);
}

void RTC_setAlarm(struct RTC_alarm *t,  void (*alarm_callback)(void), alarmRepeat rpt)
{ static uint8_t rxbuf[] = {0, 0, 0, 0, 0};
  static uint8_t txbuf[] = {0, 0, 0, 0, 0, 0};

  alarmCallback = alarm_callback;
	alarmProcess  = PROCESS_CURRENT();

  // Read the registers
	M41T62_readRegN(M41T62_REG_ALARM_MONTH, rxbuf, 5);

  // Clear out the bits we want to change
  rxbuf[0] &= ~(M41T62_ALARM_10MONTH_MASK   | M41T62_ALARM_MONTH_MASK);
  rxbuf[1] &= ~(M41T62_ALARM_10DAY_MASK     | M41T62_ALARM_DAY_MASK);
  rxbuf[2] &= ~(M41T62_ALARM_10HOUR_MASK    | M41T62_ALARM_HOUR_MASK);
  rxbuf[3] &= ~(M41T62_ALARM_10MINUTES_MASK | M41T62_ALARM_MINUTES_MASK);
  rxbuf[4] &= ~(M41T62_ALARM_10SECONDS_MASK | M41T62_ALARM_SECONDS_MASK);

  // Set the bits as we want them. Note that we
  // must construct a transmit buffer that contains
  // the register number as the first element, followed
  // by the data.
  //
  txbuf[0] = M41T62_REG_ALARM_MONTH;
  txbuf[1] = rxbuf[0] | M41T62_SET_MONTH(t->month);
  txbuf[2] = rxbuf[1] | M41T62_SET_DAY(t->day);
  txbuf[3] = rxbuf[2] | M41T62_SET_HOURS(t->hours);
  txbuf[4] = rxbuf[3] | M41T62_SET_MINUTES(t->minutes);
  txbuf[5] = rxbuf[4] | M41T62_SET_SECONDS(t->seconds);

  // Set the alarm repeat bits
  //
  uint8_t rptBits; 
  switch (rpt) {
    case RPT_YEAR:
		  rptBits = M41T62_RPT_YEAR;
			break;
    case RPT_MONTH:
		  rptBits = M41T62_RPT_MONTH;
			break;
    case RPT_DAY:
		  rptBits = M41T62_RPT_DAY;
			break;
    case RPT_HOUR:
		  rptBits = M41T62_RPT_HOUR;
			break;
    case RPT_MINUTE:
		  rptBits = M41T62_RPT_MINUTE;
			break;
    case RPT_SECOND:
		  rptBits = M41T62_RPT_SECOND;
			break;
  }

  txbuf[5] &= ~M41T62_SET_RPT1(rptBits);
  txbuf[5] |= M41T62_SET_RPT1(rptBits);
  rptBits = rptBits >> 1;
  txbuf[4] &= ~M41T62_SET_RPT2(rptBits);
  txbuf[4] |= M41T62_SET_RPT2(rptBits);
  rptBits = rptBits >> 1;
  txbuf[3] &= ~M41T62_SET_RPT3(rptBits);
  txbuf[3] |= M41T62_SET_RPT3(rptBits);
  rptBits = rptBits >> 1;
  txbuf[2] &= ~M41T62_SET_RPT4(rptBits);
  txbuf[2] |= M41T62_SET_RPT4(rptBits);
  rptBits = rptBits >> 1;
  txbuf[2] &= ~M41T62_SET_RPT5(rptBits);
  txbuf[2] |= M41T62_SET_RPT5(rptBits);

  // Set the alarm flag enable bit
  txbuf[1] |= M41T62_AFE_MASK;

  // And write them back
  //
  M41T62_writeRegN(txbuf, 6); 

  // There is a bizarreness in that that alarm will NOT
  // be set if the address is allowed to increment to
  // the flag register, as it does when we do the write
  // above
  //
  // Force the address to another place to enable the
  // alarm
  //
  M41T62_readReg(M41T62_REG_HUNDREDTHS, rxbuf);
  print_regs("Set alarm");
}

void RTC_clearAlarm()
{ static uint8_t rxbuf[] = {0, 0, 0, 0, 0};
  static uint8_t txbuf[] = {0, 0, 0, 0, 0, 0};

  // Delete the callback
  alarmCallback = NULL;
  alarmProcess  = NULL;

  // Now clear the alarm entries
  // Read the registers
	M41T62_readRegN(M41T62_REG_ALARM_MONTH, rxbuf, 5);

  // Clear out the bits representing the alarm
  rxbuf[0] &= ~(M41T62_ALARM_10MONTH_MASK   | M41T62_ALARM_MONTH_MASK);
  rxbuf[1] &= ~(M41T62_ALARM_10DAY_MASK     | M41T62_ALARM_DAY_MASK);
  rxbuf[2] &= ~(M41T62_ALARM_10HOUR_MASK    | M41T62_ALARM_HOUR_MASK);
  rxbuf[3] &= ~(M41T62_ALARM_10MINUTES_MASK | M41T62_ALARM_MINUTES_MASK);
  rxbuf[4] &= ~(M41T62_ALARM_10SECONDS_MASK | M41T62_ALARM_SECONDS_MASK);

  // Set the bits as we want them. Note that we
  // must construct a transmit buffer that contains
  // the register number as the first element, followed
  // by the data.
  //
  txbuf[0] = M41T62_REG_ALARM_MONTH;
  txbuf[1] = rxbuf[0];
  txbuf[2] = rxbuf[1];
  txbuf[3] = rxbuf[2];
  txbuf[4] = rxbuf[3];
  txbuf[5] = rxbuf[4];

  // Clear the alarm flag enable bit
  txbuf[1] &= ~(M41T62_AFE_MASK);

  // And all the RPT values
  //
  txbuf[5] &= ~(M41T62_RPT1_MASK);  
  txbuf[4] &= ~(M41T62_RPT2_MASK);  
  txbuf[3] &= ~(M41T62_RPT3_MASK);  
  txbuf[2] &= ~(M41T62_RPT4_MASK | M41T62_RPT5_MASK);  

  // And write them back
  //
  M41T62_writeRegN(txbuf, 6);
}

void RTC_resetRegisters()
{ static uint8_t txbuf[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  static uint8_t *rxbuf;
	rxbuf = &txbuf[1];

  // Read the registers
	M41T62_readRegN(M41T62_REG_HUNDREDTHS, rxbuf, 16);  

  // Clear out the bits that should be zero
  rxbuf[0x00] &= (M41T62_REG0_INIT_MASK);
  rxbuf[0x01] &= (M41T62_REG1_INIT_MASK);
  rxbuf[0x02] &= (M41T62_REG2_INIT_MASK);
  rxbuf[0x03] &= (M41T62_REG3_INIT_MASK);
  rxbuf[0x04] &= (M41T62_REG4_INIT_MASK);
  rxbuf[0x05] &= (M41T62_REG5_INIT_MASK);
  rxbuf[0x06] &= (M41T62_REG6_INIT_MASK);
  rxbuf[0x07] &= (M41T62_REG7_INIT_MASK);
  rxbuf[0x08] &= (M41T62_REG8_INIT_MASK);
  rxbuf[0x09] &= (M41T62_REG9_INIT_MASK);
  rxbuf[0x0A] &= (M41T62_REGA_INIT_MASK);
  rxbuf[0x0B] &= (M41T62_REGB_INIT_MASK);
  rxbuf[0x0C] &= (M41T62_REGC_INIT_MASK);
  rxbuf[0x0D] &= (M41T62_REGE_INIT_MASK);
  rxbuf[0x0E] &= (M41T62_REGE_INIT_MASK);
  rxbuf[0x0F] &= (M41T62_REGF_INIT_MASK);

  // And write them back
  //
  txbuf[0] = M41T62_REG_HUNDREDTHS;
  
  M41T62_writeRegN(txbuf, 17);
}

void RTC_stop()
{ uint8_t buf;
  // Set the stop bit
	M41T62_readReg(M41T62_REG_ST, &buf);
	buf |= M41T62_ST_MASK;
	M41T62_writeReg(M41T62_REG_ST, buf);
}

void RTC_start()
{ uint8_t buf;
	// Clear the stop bit
	M41T62_readReg(M41T62_REG_ST, &buf);
	buf &= ~M41T62_ST_MASK;
	M41T62_writeReg(M41T62_REG_ST, buf);
}

static void print_regs(char *str)
{ 
#if DEBUG
  static uint8_t rxbuf[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  int i;

  // Read the registers
	M41T62_readRegN(M41T62_REG_HUNDREDTHS, rxbuf, 16);  
	
  if (str != NULL && strcmp(str, ""))
    PRINTF("%15s: ", str);

  for (i = 0; i < 16; i++) {
    PRINTF("%02X ", rxbuf[i]);
  }
  PRINTF("\n");
#endif
}


PROCESS_THREAD(RTC_process, ev, data)
{ uint8_t buf;

  PROCESS_BEGIN();
 
  while(1) {
    PROCESS_YIELD();

    if (ev != PROCESS_EVENT_POLL) {
      continue;
    }

		// Read the flag bits. This also clears the ISR indication
    // We can't do this within the ISR because the i2c blocks
    //
		M41T62_readReg(M41T62_REG_FLAGS, &buf);

    // Let the calling process know that the alarm has expired
    // Both by calling the callback and by posting an event to
    // the process that set up the alarm in the first place.
    //
    if (alarmCallback != NULL && ((buf & M41T62_AF_MASK) != 0))
      alarmCallback();

    if (alarmProcess != NULL)
		  process_post(alarmProcess, ALARM_EXPIRED, NULL);

  }
  
  PROCESS_END();
}

// Interrupts are on kbi5
//
void kbi5_isr(void)
{ 
  process_poll(&RTC_process);

	clear_kbi_evnt(5);
}




// *******************************************************
// Utility functions that read and write from/to registers
// *******************************************************

// Read one byte from a register
//
void M41T62_readReg(uint8_t reg, uint8_t* data)
{
  i2c_transmitinit(M41T62_I2C_ADDR, 1, &reg); 			// Send register to be read
  while(!i2c_transferred());								  			// Wait for transfer
  i2c_receiveinit(M41T62_I2C_ADDR, 1, data);				// Get value back
  while(!i2c_transferred());												// Wait for transfer
}

// Read n bytes from a given register on - useful for autoincrements
//
void M41T62_readRegN(uint8_t reg, uint8_t* data, uint8_t n)
{
  i2c_transmitinit(M41T62_I2C_ADDR, 1, &reg); 			// Send register to be read
  while(!i2c_transferred());								  			// Wait for transfer
  i2c_receiveinit(M41T62_I2C_ADDR, n, data);				// Get the n values back
  while(!i2c_transferred());												// Wait for transfer
}

// Write one byte to a register
// NB The write function requires us NOT to stop between sending the
// register number and the data, which is why we put them into a single
// buffer and then transmit that.
//
void M41T62_writeReg(uint8_t writeAddr, uint8_t data)
{ uint8_t buf[] = {0, 0};
  buf[0] = writeAddr;
  buf[1] = data;

  i2c_transmitinit(M41T62_I2C_ADDR, 2, buf);        // Send data to write
  while(!i2c_transferred());								  			// Wait for transfer
}

// Write n bytes to a set of registers
// NB The write function requires us NOT to stop between sending the
// register number and the data, so we assume that the data is constructed
// with the register in the first byte
//
void M41T62_writeRegN(uint8_t* data, uint8_t n)
{
  i2c_transmitinit(M41T62_I2C_ADDR, n, data);      	// Send data to write
  while(!i2c_transferred());								  			// Wait for transfer
}



