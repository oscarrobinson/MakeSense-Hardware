/*
 * lsm330dlc.c
 *
 *  Created on: 27/1/13
 *      Author: Stephen Hailes and Jagun Kwon
 *
 */

#include <stdio.h>
#include "i2c.h"
#include "include/lsm330dlc.h"

#include "lib/sensors.h"    // for Contiki sensor interface

#define DEBUG 1

#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

extern volatile uint8_t timer_delay;

void lsm330dlc_init(void)
{	uint8_t buf[] = {0,0};

	buf[0] = LSM330DLC_WHO_AM_I_G;
	i2c_transmitinit(LSM330DLC_I2C_GYR_ADDR, 1, buf);
	while(!i2c_transferred());

	i2c_receiveinit(LSM330DLC_I2C_GYR_ADDR, 1, buf);
	while(!i2c_transferred());

  if (buf[0] != LSM330DLC_I_AM_L3GD20)
		PRINTF("LSM330DLC: Not initialised properly: return from WHO AM I is %d\r\n", buf[0]);
	else
		PRINTF("LSM330DLC: Initialised OK: %d\r\n", buf[0]);
}

void lsm330dlc_start(void)
{	uint8_t buf[] = {0,0};

  // Accelerometer setup - enable normal mode and high precision
  buf[0] = LSM330DLC_CTRL_REG1_A;
  buf[1] = LSM330DLC_ODR_10Hz_A | LSM330DLC_ACC_ENABLE_ALL_AXES;
	i2c_transmitinit(LSM330DLC_I2C_ACC_ADDR, 2, buf);
	while(!i2c_transferred());

  buf[0] = LSM330DLC_CTRL_REG4_A;
  buf[1] = LSM330DLC_HR_A;
	i2c_transmitinit(LSM330DLC_I2C_ACC_ADDR, 2, buf);
	while(!i2c_transferred());

  // Gyro setup - enable readings
  buf[0] = LSM330DLC_CTRL_REG1_G;
  buf[1] = LSM330DLC_PD_G | LSM330DLC_ZEN_G | LSM330DLC_YEN_G | LSM330DLC_XEN_G;
	i2c_transmitinit(LSM330DLC_I2C_GYR_ADDR, 2, buf);
	while(!i2c_transferred());
}

void lsm330dlc_stop(void)
{	uint8_t buf[] = {0,0};

  // Accelerometer setup - disable readings
  buf[0] = LSM330DLC_CTRL_REG1_A;
  buf[1] = 0;
	i2c_transmitinit(LSM330DLC_I2C_ACC_ADDR, 2, buf);
	while(!i2c_transferred());

  // Gyro setup - disable readings
  buf[0] = LSM330DLC_CTRL_REG1_G;
  buf[1] = 0;
	i2c_transmitinit(LSM330DLC_I2C_GYR_ADDR, 2, buf);
	while(!i2c_transferred());
}

// Latch data
void lsm330dlc_get_data(uint8_t *buf)
{ int i;
	// Read accelerometer
	buf[0] = I2C_AUTO_INCREMENT | LSM330DLC_OUT_X_L_A;
	i2c_transmitinit(LSM330DLC_I2C_ACC_ADDR, 1, buf);
	while(!i2c_transferred());
	i2c_receiveinit(LSM330DLC_I2C_ACC_ADDR, 6, buf);
	while(!i2c_transferred());

	// Read gyro
	buf[6] = I2C_AUTO_INCREMENT | LSM330DLC_OUT_X_L_G;
	i2c_transmitinit(LSM330DLC_I2C_GYR_ADDR, 1, &(buf[6]));
	while(!i2c_transferred());
	i2c_receiveinit(LSM330DLC_I2C_GYR_ADDR, 6, &(buf[6]));
	while(!i2c_transferred());
}

void lsm330dlc_decode(struct lsm330dlc_data *imu_data , uint8_t *buffer)
{
	imu_data->acc_x =  ((buffer[1]<<8) + buffer[0]);
	imu_data->acc_y =  ((buffer[3]<<8) + buffer[2]);
	imu_data->acc_z =  ((buffer[5]<<8) + buffer[4]);
	imu_data->gyr_x =  ((buffer[7]<<8) + buffer[6]);
	imu_data->gyr_y =  ((buffer[9]<<8) + buffer[8]);
	imu_data->gyr_z = ((buffer[11]<<8) + buffer[10]);
}

// *******************************************************
// Utility functions that read and write from/to registers
// *******************************************************
u8_t LSM330DLC_ReadReg(u8_t deviceAddr, u8_t Reg, u8_t* Data) {
    i2c_transmitinit( deviceAddr, 1, &Reg ) ;
    while(!i2c_transferred()) /* Wait for transfer */ ;
    i2c_receiveinit( deviceAddr, 1, Data) ;
    while(!i2c_transferred()) /* Wait for transfer */ ;
    return MEMS_SUCCESS;
}

u8_t LSM330DLC_WriteReg(u8_t deviceAddr, u8_t WriteAddr, u8_t Data) {

/*
  uint8_t buf[] = {0,0};
  buf[0] = WriteAddr;
  buf[1] = Data; 
  i2c_transmitinit( deviceAddr, 2, buf);
  while(!i2c_transferred()) // Wait for transfer / ;
*/

    i2c_transmitinit( deviceAddr, 1, &WriteAddr ) ; // Register to write
    while(!i2c_transferred()) /* Wait for transfer */  ;
    i2c_transmitinit( deviceAddr, 1, &Data ) ;      // Data to write
    while(!i2c_transferred()) /* Wait for transfer */  ;

	return MEMS_SUCCESS;
}


// Set Sampling Frequencies for the Accelerometer
void lsm330dlc_setFreqAcc(u8_t freq) {
	uint8_t buf[] = {0,0};

	buf[0] = LSM330DLC_CTRL_REG1_A;
	buf[1] = freq | LSM330DLC_ACC_ENABLE_ALL_AXES;
	i2c_transmitinit(LSM330DLC_I2C_ACC_ADDR, 2, buf);
	while(!i2c_transferred());
}

// Set Sampling Frequencies for the Gyroscope
void lsm330dlc_setFreqGyr(u8_t freq) {
  u8_t value;

  LSM330DLC_ReadReg(LSM330DLC_I2C_GYR_ADDR,
				LSM330DLC_CTRL_REG1_G, &value);
  value &= 0x0f;
  value |= freq;
  LSM330DLC_WriteReg(LSM330DLC_I2C_GYR_ADDR,
				LSM330DLC_CTRL_REG1_G, value);
}

// Power Mode : Normal/Low Power Mode
// Set power mode for the accelerometer
u8_t LSM330DLC_SetMode_A(LSM330DLC_Mode_A_t md) {
  u8_t value;
  u8_t value2;
  static   u8_t ODR_old_value;

  LSM330DLC_ReadReg(LSM330DLC_I2C_ACC_ADDR, LSM330DLC_CTRL_REG1_A, &value);
  LSM330DLC_ReadReg(LSM330DLC_I2C_ACC_ADDR,LSM330DLC_CTRL_REG4_A, &value2);

  if((value & 0xF0)==0) value = value | (ODR_old_value & 0xF0); //if it comes from POWERDOWN  

  switch(md) {

  case LSM330DLC_POWER_DOWN_A:
    ODR_old_value = value;
    value &= 0x0F;
    break;

  case LSM330DLC_NORMAL_A:
    value &= 0xF7;
    value |= (MEMS_RESET<<LSM330DLC_LPEN_A);
    value2 &= 0xF7;
    value2 |= (MEMS_SET<<LSM330DLC_HR_A);   //set HighResolution_BIT
    break;

  case LSM330DLC_LOW_POWER_A:
    value &= 0xF7;
    value |=  (MEMS_SET<<LSM330DLC_LPEN_A);
    value2 &= 0xF7;
    value2 |= (MEMS_RESET<<LSM330DLC_HR_A); //reset HighResolution_BIT
    break;

  default:
    return MEMS_ERROR;
  }

  LSM330DLC_WriteReg(LSM330DLC_I2C_ACC_ADDR,LSM330DLC_CTRL_REG1_A, value);

  LSM330DLC_WriteReg(LSM330DLC_I2C_ACC_ADDR,LSM330DLC_CTRL_REG4_A, value2);

  return MEMS_SUCCESS;
}


// Set power mode for the gyroscope
u8_t LSM330DLC_SetMode_G(LSM330DLC_Mode_G_t md) {
  u8_t value;

  LSM330DLC_ReadReg(LSM330DLC_I2C_GYR_ADDR, LSM330DLC_CTRL_REG1_G, &value);

  switch(md) {

  case LSM330DLC_POWER_DOWN_G:
    value &= 0xF7;
    value |= (MEMS_RESET<<LSM330DLC_PD_G);
    break;

  case LSM330DLC_NORMAL_G:
    value &= 0xF7;
    value |= (MEMS_SET<<LSM330DLC_PD_G);
    break;

  case LSM330DLC_SLEEP_G:
    value &= 0xF0;
    value |= ( (MEMS_SET<<LSM330DLC_PD_G) | (MEMS_RESET<<LSM330DLC_ZEN_G) | (MEMS_RESET<<LSM330DLC_YEN_G) | (MEMS_RESET<<LSM330DLC_XEN_G) );
    break;

  default:
    return MEMS_ERROR;
  }

  LSM330DLC_WriteReg(LSM330DLC_I2C_GYR_ADDR, LSM330DLC_CTRL_REG1_G, value);

  return MEMS_SUCCESS;
}


// Sets FIFO Mode Modality for the Accelerometer
u8_t lsm330dlc_FIFOModeEnable_A(LSM330DLC_FifoMode_A_t fm) {
  u8_t value;

  if(fm == LSM330DLC_FIFO_DISABLE_A) {
    if(!LSM330DLC_ReadReg(LSM330DLC_I2C_ACC_ADDR, LSM330DLC_FIFO_CTRL_REG_A, &value) )
      return MEMS_ERROR;

    value &= 0x1f;
    value |= (LSM330DLC_FIFO_BYPASS_MODE_A<<LSM330DLC_FM_A);

    if(!LSM330DLC_WriteReg(LSM330DLC_I2C_ACC_ADDR, LSM330DLC_FIFO_CTRL_REG_A, value))           //fifo mode bypass
      return MEMS_ERROR;
    if( !LSM330DLC_ReadReg(LSM330DLC_I2C_ACC_ADDR, LSM330DLC_CTRL_REG5_A, &value) )
      return MEMS_ERROR;

    value &= 0xBF;

    if( !LSM330DLC_WriteReg(LSM330DLC_I2C_ACC_ADDR, LSM330DLC_CTRL_REG5_A, value) )               //fifo disable
      return MEMS_ERROR;
  }

  if(fm == LSM330DLC_FIFO_BYPASS_MODE_A) {
    if( !LSM330DLC_ReadReg(LSM330DLC_I2C_ACC_ADDR, LSM330DLC_CTRL_REG5_A, &value) )
      return MEMS_ERROR;

    value &= 0xBF;
    value |= LSM330DLC_FIFO_EN_A;

   if( !LSM330DLC_WriteReg(LSM330DLC_I2C_ACC_ADDR, LSM330DLC_CTRL_REG5_A, value) )               //fifo enable
      return MEMS_ERROR;
   if( !LSM330DLC_ReadReg(LSM330DLC_I2C_ACC_ADDR, LSM330DLC_FIFO_CTRL_REG_A, &value) )
      return MEMS_ERROR;

    value &= 0x1f;
    value |= (fm<<LSM330DLC_FM_A);                     //fifo mode configuration

   if( !LSM330DLC_WriteReg(LSM330DLC_I2C_ACC_ADDR, LSM330DLC_FIFO_CTRL_REG_A, value) )
      return MEMS_ERROR;
  }

  if(fm == LSM330DLC_FIFO_MODE_A)   {
    if( !LSM330DLC_ReadReg(LSM330DLC_I2C_ACC_ADDR, LSM330DLC_CTRL_REG5_A, &value) )
      return MEMS_ERROR;

    value &= 0xBF;
    value |= LSM330DLC_FIFO_EN_A;
    if( !LSM330DLC_WriteReg(LSM330DLC_I2C_ACC_ADDR, LSM330DLC_CTRL_REG5_A, value) )               //fifo enable
      return MEMS_ERROR;
    if(!LSM330DLC_ReadReg(LSM330DLC_I2C_ACC_ADDR, LSM330DLC_FIFO_CTRL_REG_A, &value) )
      return MEMS_ERROR;

    value &= 0x1f;
    value |= (fm<<LSM330DLC_FM_A);                      //fifo mode configuration

    if(!LSM330DLC_WriteReg(LSM330DLC_I2C_ACC_ADDR, LSM330DLC_FIFO_CTRL_REG_A, value) )
      return MEMS_ERROR;
  }

  if(fm == LSM330DLC_FIFO_STREAM_MODE_A)   {
    if( !LSM330DLC_ReadReg(LSM330DLC_I2C_ACC_ADDR, LSM330DLC_CTRL_REG5_A, &value) )
      return MEMS_ERROR;

    value &= 0xBF;
    value |= LSM330DLC_FIFO_EN_A;

    if( !LSM330DLC_WriteReg(LSM330DLC_I2C_ACC_ADDR, LSM330DLC_CTRL_REG5_A, value) )               //fifo enable
      return MEMS_ERROR;
    if(!LSM330DLC_ReadReg(LSM330DLC_I2C_ACC_ADDR, LSM330DLC_FIFO_CTRL_REG_A, &value) )
      return MEMS_ERROR;

    value &= 0x1f;
    value |= (fm<<LSM330DLC_FM_A);                      //fifo mode configuration

    if(!LSM330DLC_WriteReg(LSM330DLC_I2C_ACC_ADDR, LSM330DLC_FIFO_CTRL_REG_A, value) )
      return MEMS_ERROR;
  }

  if(fm == LSM330DLC_FIFO_TRIGGER_MODE_A)   {
    if( !LSM330DLC_ReadReg(LSM330DLC_I2C_ACC_ADDR, LSM330DLC_CTRL_REG5_A, &value) )
      return MEMS_ERROR;

    value &= 0xBF;
    value |= LSM330DLC_FIFO_EN_A;
    if( !LSM330DLC_WriteReg(LSM330DLC_I2C_ACC_ADDR, LSM330DLC_CTRL_REG5_A, value) )               //fifo enable
      return MEMS_ERROR;
    if(!LSM330DLC_ReadReg(LSM330DLC_I2C_ACC_ADDR, LSM330DLC_FIFO_CTRL_REG_A, &value) )
      return MEMS_ERROR;

    value &= 0x1f;
    value |= (fm<<LSM330DLC_FM_A);                      //fifo mode configuration

    if(!LSM330DLC_WriteReg(LSM330DLC_I2C_ACC_ADDR, LSM330DLC_FIFO_CTRL_REG_A, value) )
      return MEMS_ERROR;
  }
  return MEMS_SUCCESS;
}

// Sets FIFO Mode Modality for the Gyroscope
u8_t lsm330dlc_FIFOModeEnable_G(LSM330DLC_FifoMode_G_t fm) {
  u8_t value;

  if(fm == LSM330DLC_FIFO_DISABLE_G) {
    if( !LSM330DLC_ReadReg(LSM330DLC_I2C_GYR_ADDR, LSM330DLC_CTRL_REG5_G, &value) )
      return MEMS_ERROR;
    value &= 0xBF;
    if( !LSM330DLC_WriteReg(LSM330DLC_I2C_GYR_ADDR, LSM330DLC_CTRL_REG5_G, value) )
      return MEMS_ERROR;
  }
  else {
    if( !LSM330DLC_ReadReg(LSM330DLC_I2C_GYR_ADDR, LSM330DLC_CTRL_REG5_G, &value) )
      return MEMS_ERROR;
    value &= 0xBF;
    value |= LSM330DLC_FIFO_EN_G;
    if( !LSM330DLC_WriteReg(LSM330DLC_I2C_GYR_ADDR, LSM330DLC_CTRL_REG5_G, value) )
      return MEMS_ERROR;

// Alternative (instead of using WriteReg)
//PRINTF("DEBUG:Before1: FIFO_G REG5 value=0x%02X\n", value);
/*
  uint8_t buf[] = {0,0};
  buf[0] = LSM330DLC_CTRL_REG5_G;
  buf[1] = value; 
  i2c_transmitinit(LSM330DLC_I2C_GYR_ADDR, 2, buf);
  while(!i2c_transferred()) ; // Wait for transfer //  
*/

// DEBUG
LSM330DLC_ReadReg(LSM330DLC_I2C_GYR_ADDR, LSM330DLC_CTRL_REG5_G, &value);
PRINTF("DEBUG:After: FIFO_G REG5 value=0x%02X\n", value);

    if(!LSM330DLC_ReadReg(LSM330DLC_I2C_GYR_ADDR, LSM330DLC_FIFO_CTRL_REG_G, &value) )
      return MEMS_ERROR;
    value &= 0x1f;
    value |= (fm<<LSM330DLC_FM0_G);
    if(!LSM330DLC_WriteReg(LSM330DLC_I2C_GYR_ADDR, LSM330DLC_FIFO_CTRL_REG_G, value) )
      return MEMS_ERROR;

// Alternative (instead of using WriteReg)
//PRINTF("DEBUG:Before1: FIFO_G CRTL_REG value=0x%02X\n", value);
//  uint8_t buf[] = {0,0};
/*
  buf[0] = LSM330DLC_FIFO_CTRL_REG_G;
  buf[1] = value; 
  i2c_transmitinit(LSM330DLC_I2C_GYR_ADDR, 2, buf);
  while(!i2c_transferred()) ; // Wait for transfer //  
*/

// DEBUG
LSM330DLC_ReadReg(LSM330DLC_I2C_GYR_ADDR, LSM330DLC_FIFO_CTRL_REG_G, &value);
PRINTF("DEBUG:After: FIFO_G CTRL_REG value=0x%02X\n", value);
  }
  return MEMS_SUCCESS;
}

// Sets the full scale accelerometer, i.e., 2G, 4G, 8G, or 16G
u8_t LSM330DLC_SetFullScale_A(u8_t fs) {
  u8_t value;

  LSM330DLC_ReadReg(LSM330DLC_I2C_ACC_ADDR,LSM330DLC_CTRL_REG4_A, &value);

  value &= 0xCF;
  value |= fs;

  LSM330DLC_WriteReg(LSM330DLC_I2C_ACC_ADDR,LSM330DLC_CTRL_REG4_A, value);

  return MEMS_SUCCESS;
}

// Sets the full scale gyro
u8_t LSM330DLC_SetFullScale_G(u8_t fs) {
  u8_t value;

  LSM330DLC_ReadReg(LSM330DLC_I2C_GYR_ADDR, LSM330DLC_CTRL_REG4_G, &value);
  value &= 0xCF;
  value |= fs;

// Alternative (instead of using WriteReg)
  uint8_t buf[] = {0,0};
  buf[0] = LSM330DLC_CTRL_REG4_G;
  buf[1] = value; 
  i2c_transmitinit(LSM330DLC_I2C_GYR_ADDR, 2, buf);
  while(!i2c_transferred()) ; // Wait for transfer //  ;

//LSM330DLC_ReadReg(LSM330DLC_I2C_GYR_ADDR, LSM330DLC_CTRL_REG4_G, &value);
//PRINTF("CTFL_REG4_G = 0x%02x\n", value);

//  LSM330DLC_WriteReg(LSM330DLC_I2C_GYR_ADDR, LSM330DLC_CTRL_REG4_G, value);

//LSM330DLC_ReadReg(LSM330DLC_I2C_GYR_ADDR, LSM330DLC_CTRL_REG4_G, &value);
//PRINTF("CTFL_REG4_G = 0x%02x\n", value);

  return MEMS_SUCCESS;
}



// Contiki Sensor Interface
static int
value(int type)
{ static uint8_t buffer[12];
  static struct lsm330dlc_data imu_data;

  switch (type) {
		// Must get acc_x, y, z and gyr_x, y, z after getting the data first
	  // In fact, get data and decode it
    case LSM330DLC_LATCH_DATA:
		default:
    	lsm330dlc_get_data(buffer);
    	lsm330dlc_decode(&imu_data, buffer);
      return 1;

    case LSM330DLC_ACC_X_RAW:
    	return imu_data.acc_x;
    case LSM330DLC_ACC_Y_RAW:
    	return imu_data.acc_y;
    case LSM330DLC_ACC_Z_RAW:
    	return imu_data.acc_z;
    case LSM330DLC_GYR_X_RAW:
    	return imu_data.gyr_x;
    case LSM330DLC_GYR_Y_RAW:
    	return imu_data.gyr_y;
    case LSM330DLC_GYR_Z_RAW:
    	return imu_data.gyr_z;
  }
}

static int
status(int type)
{	uint8_t buf[] = {0};

	buf[0] = LSM330DLC_STATUS_REG_A;
	i2c_transmitinit(LSM330DLC_I2C_ACC_ADDR, 1, buf);
	while(!i2c_transferred());
	i2c_receiveinit(LSM330DLC_I2C_ACC_ADDR, 1, buf);
	while(!i2c_transferred());

  return buf[0];
}

static int
configure(int type, int c)
{
	uint8_t buf[] = {0,0};

	if (c == 1) {		// If activating the sensor
		lsm330dlc_init();
		lsm330dlc_start();
	} else if (c == 0) {	// If deactivating the sensor
		// Disable the sensor
		lsm330dlc_stop();
		LSM330DLC_SetMode_A(LSM330DLC_POWER_DOWN_A);
		LSM330DLC_SetMode_G(LSM330DLC_POWER_DOWN_G);
	}

	// Power Mode Selection for both acc and gyr
	if (c & LSM330DLC_SET_POWER_NORMAL) {
		LSM330DLC_SetMode_A(LSM330DLC_NORMAL_A);
		LSM330DLC_SetMode_G(LSM330DLC_NORMAL_G);
	} else if (c & LSM330DLC_SET_POWER_LOW) {
		LSM330DLC_SetMode_A(LSM330DLC_LOW_POWER_A);
		LSM330DLC_SetMode_G(LSM330DLC_SLEEP_G);
	}

	// Sampling frequencies for the Accelerometer
	if (c & LSM330DLC_SET_ACC_FREQ_1Hz) {
		lsm330dlc_setFreqAcc(LSM330DLC_ODR_1Hz_A);
	} else if (c & LSM330DLC_SET_ACC_FREQ_10Hz) {
		lsm330dlc_setFreqAcc(LSM330DLC_ODR_10Hz_A);
	} else if (c & LSM330DLC_SET_ACC_FREQ_25Hz) {
		lsm330dlc_setFreqAcc(LSM330DLC_ODR_25Hz_A);
	} else if (c & LSM330DLC_SET_ACC_FREQ_50Hz) {
		lsm330dlc_setFreqAcc(LSM330DLC_ODR_50Hz_A);
	} else if (c & LSM330DLC_SET_ACC_FREQ_100Hz) {
		lsm330dlc_setFreqAcc(LSM330DLC_ODR_100Hz_A);
	} else if (c & LSM330DLC_SET_ACC_FREQ_200Hz) {
		lsm330dlc_setFreqAcc(LSM330DLC_ODR_200Hz_A);
	} else if (c & LSM330DLC_SET_ACC_FREQ_400Hz) {
		lsm330dlc_setFreqAcc(LSM330DLC_ODR_400Hz_A);
	} else if (c & LSM330DLC_SET_ACC_FREQ_1620kHz_LP) {
		lsm330dlc_setFreqAcc(LSM330DLC_ODR_1620Hz_LP_A);
	} else if (c & LSM330DLC_SET_ACC_FREQ_1344Hz_NP_5367HZ_LP) {
		lsm330dlc_setFreqAcc(LSM330DLC_ODR_1344Hz_NP_5367HZ_LP_A);
	}

	// Sampling frequencies for the Gyroscope
	// TODO There are bandwidth sub-properties.
	// But, we choose the highest bandwidth for all output rates 
	if (c & LSM330DLC_SET_GYR_FREQ_95Hz) {
		lsm330dlc_setFreqGyr(LSM330DLC_ODR_95Hz_BW_25_G);
	} else if (c & LSM330DLC_SET_GYR_FREQ_190Hz) {
		lsm330dlc_setFreqGyr(LSM330DLC_ODR_190Hz_BW_70_G);
	} else if (c & LSM330DLC_SET_GYR_FREQ_380Hz) {
		lsm330dlc_setFreqGyr(LSM330DLC_ODR_380Hz_BW_110_G);
	} else if (c & LSM330DLC_SET_GYR_FREQ_760Hz) {
		lsm330dlc_setFreqGyr(LSM330DLC_ODR_760Hz_BW_110_G);
	}

	// FIFO Mode Selection
	if (c & LSM330DLC_SET_FIFO_MODE_DISABLE) {
	    lsm330dlc_FIFOModeEnable_A(LSM330DLC_FIFO_DISABLE_A);
	    lsm330dlc_FIFOModeEnable_G(LSM330DLC_FIFO_DISABLE_G);
	} else if (c & LSM330DLC_SET_FIFO_MODE_BYPASS) {
	    lsm330dlc_FIFOModeEnable_A(LSM330DLC_FIFO_BYPASS_MODE_A);
	    lsm330dlc_FIFOModeEnable_G(LSM330DLC_FIFO_BYPASS_MODE_G);
	} else if (c & LSM330DLC_SET_FIFO_MODE_FIFO) {
	    lsm330dlc_FIFOModeEnable_A(LSM330DLC_FIFO_MODE_A);
	    lsm330dlc_FIFOModeEnable_G(LSM330DLC_FIFO_MODE_G);
	} else if (c & LSM330DLC_SET_FIFO_MODE_STREAM) {
	    lsm330dlc_FIFOModeEnable_A(LSM330DLC_FIFO_STREAM_MODE_A);
	    lsm330dlc_FIFOModeEnable_G(LSM330DLC_FIFO_STREAM_MODE_G);
	} else if (c & LSM330DLC_SET_FIFO_MODE_TRIGGER) {
	    lsm330dlc_FIFOModeEnable_A(LSM330DLC_FIFO_TRIGGER_MODE_A);
	} else if (c & LSM330DLC_SET_FIFO_MODE_STREAM_TO_FIFO) {
		// Only for Gyroscope
	    lsm330dlc_FIFOModeEnable_G(LSM330DLC_FIFO_STREAM_TO_FIFO_MODE_G);
	} else if (c & LSM330DLC_SET_FIFO_MODE_BYPASS_TO_STREAM) {
		// Only for Gyroscope
	    lsm330dlc_FIFOModeEnable_G(LSM330DLC_FIFO_BYPASS_TO_STREAM_MODE_G);
	}

	// Full scale selection for the accelerometer: Default value: 2G
	if (c & LSM330DLC_SET_ACC_FULLSCALE_2G) {
		LSM330DLC_SetFullScale_A(LSM330DLC_FULLSCALE_2_A);
	} else if (c & LSM330DLC_SET_ACC_FULLSCALE_4G) {
		LSM330DLC_SetFullScale_A(LSM330DLC_FULLSCALE_4_A);
	} else if (c & LSM330DLC_SET_ACC_FULLSCALE_8G) {
		LSM330DLC_SetFullScale_A(LSM330DLC_FULLSCALE_8_A);
	} else if (c & LSM330DLC_SET_ACC_FULLSCALE_16G) {
		LSM330DLC_SetFullScale_A(LSM330DLC_FULLSCALE_16_A);
	}

	// Full scale selection for the gyroscope
	if (c & LSM330DLC_SET_GYR_FULLSCALE_250) {
		LSM330DLC_SetFullScale_G(LSM330DLC_FS_250_G);
	} else if (c & LSM330DLC_SET_GYR_FULLSCALE_500) {
		LSM330DLC_SetFullScale_G(LSM330DLC_FS_500_G);
	} else if (c & LSM330DLC_SET_GYR_FULLSCALE_2000) {
		LSM330DLC_SetFullScale_G(LSM330DLC_FS_2000_G);
	}

	// High resolution output mode: Default disabled
	if (c & LSM330DLC_SET_HIGHRES_DISABLE) {

	} else if (c & LSM330DLC_SET_HIGHRES_ENABLE) {
		buf[0] = LSM330DLC_CTRL_REG4_A;
		buf[1] = LSM330DLC_HR_A;
		i2c_transmitinit(LSM330DLC_I2C_ACC_ADDR, 2, buf);
		while(!i2c_transferred());
	}
  return 1;
}

// Instantiate the sensor: 
SENSORS_SENSOR(lsm330dlc_sensor, "lsm330dlc-Sensor", value, configure, status);

