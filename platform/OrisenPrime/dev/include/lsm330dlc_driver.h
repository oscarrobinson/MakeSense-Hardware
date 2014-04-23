/******************** (C) COPYRIGHT 2011 STMicroelectronics ********************
* File Name          : LSM330DLC_driver.h
* Author             : MSH Application Team
* Author             : Fabio Tota
* Version            : $Revision:$
* Date               : $Date:$
* Description        : Descriptor Header for lsm330dlc_driver.c driver file
*
* HISTORY:
* Date        | Modification                                | Author
* 27/01/2012  | Initial Revision                            | Fabio Tota
* 27/07/2012  |  Modified to support multiple drivers in the same program                |	Abhishek Anand
* 23/01/2013  : Adapted & modified by Jagun Kwon and Stephen Hailes (UCL-CS)

********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
* THIS SOFTWARE IS SPECIFICALLY DESIGNED FOR EXCLUSIVE USE WITH ST PARTS.
*
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LSM330DLC_DRIVER__H
#define __LSM330DLC_DRIVER__H

/* Includes ------------------------------------------------------------------*/
#include "lib/sensors.h"    // for Contiki sensor interface
#include "i2c.h"

/* Exported types ------------------------------------------------------------*/

//these could change accordingly with the architecture

#ifndef __ARCHDEP__TYPES
#define __ARCHDEP__TYPES

//typedef unsigned char u8_t;
typedef unsigned char uint8_t;
//typedef unsigned short int u16_t;
typedef short int i16_t;
typedef signed char i8_t;

#endif /*__ARCHDEP__TYPES*/

typedef u8_t LSM330DLC_IntPinConf_t;
typedef u8_t LSM330DLC_Axis_t;
typedef u8_t LSM330DLC_Int1Conf_t;


//define structure
#ifndef __SHARED__TYPES
#define __SHARED__TYPES

typedef enum {
  MEMS_SUCCESS                            =		0x01,
  MEMS_ERROR			          =		0x00	
} status_t;

typedef enum {
  MEMS_ENABLE			          =		0x01,
  MEMS_DISABLE			          =		0x00	
} State_t;

typedef struct {
  i16_t AXIS_X;
  i16_t AXIS_Y;
  i16_t AXIS_Z;
} AxesRaw_t;

#endif /*__SHARED__TYPES*/

typedef enum {  
  LSM330DLC_ODR_1Hz_A		        =		0x01,		
  LSM330DLC_ODR_10Hz_A				=		0x02,
  LSM330DLC_ODR_25Hz_A		        =		0x03,
  LSM330DLC_ODR_50Hz_A		        =		0x04,
  LSM330DLC_ODR_100Hz_A		        =		0x05,	
  LSM330DLC_ODR_200Hz_A		        =		0x06,
  LSM330DLC_ODR_400Hz_A		        =		0x07,
  LSM330DLC_ODR_1620Hz_LP_A	        =		0x08,
  LSM330DLC_ODR_1344Hz_NP_5367HZ_LP_A	=		0x09	
} LSM330DLC_ODR_A_t;

typedef enum {
  LSM330DLC_POWER_DOWN_A			=		0x00,
  LSM330DLC_LOW_POWER_A			=		0x01,
  LSM330DLC_NORMAL_A			=		0x02
} LSM330DLC_Mode_A_t;

typedef enum {
  LSM330DLC_HPM_NORMAL_MODE_RES           =               0x00,
  LSM330DLC_HPM_REF_SIGNAL                =               0x01,
  LSM330DLC_HPM_NORMAL_MODE               =               0x02,
  LSM330DLC_HPM_AUTORESET_INT             =               0x03
} LSM330DLC_HPFMode_t;

typedef enum {
  LSM330DLC_HPFCF_0                       =               0x00,
  LSM330DLC_HPFCF_1                       =               0x01,
  LSM330DLC_HPFCF_2                       = 		0x02,
  LSM330DLC_HPFCF_3                       =               0x03,
  LSM330DLC_HPFCF_4                       =               0x04,
  LSM330DLC_HPFCF_5                       =               0x05,
  LSM330DLC_HPFCF_6                       =               0x06,
  LSM330DLC_HPFCF_7                       =               0x07,
  LSM330DLC_HPFCF_8                       =               0x08,
  LSM330DLC_HPFCF_9                       =               0x09,  
} LSM330DLC_HPFCutOffFreq_t;

typedef enum {
  LSM330DLC_FULLSCALE_2_A                   =               0x00,
  LSM330DLC_FULLSCALE_4_A                   =               0x01,
  LSM330DLC_FULLSCALE_8_A                   =               0x02,
  LSM330DLC_FULLSCALE_16_A                  =               0x03
} LSM330DLC_Fullscale_A_t;

typedef enum {
  LSM330DLC_BLE_LSB			=		0x00,
  LSM330DLC_BLE_MSB			=		0x01
} LSM330DLC_Endianess_t;

typedef enum {
  LSM330DLC_FIFO_BYPASS_MODE_A              =               0x00,
  LSM330DLC_FIFO_MODE_A                     =               0x01,
  LSM330DLC_FIFO_STREAM_MODE_A              =               0x02,
  LSM330DLC_FIFO_TRIGGER_MODE_A             =               0x03,
  LSM330DLC_FIFO_DISABLE_A                  =               0x04
} LSM330DLC_FifoMode_A_t;
  //????????????????????
typedef enum {
  LSM330DLC_TRIG_INT1_A                     =		0x00,
  LSM330DLC_TRIG_INT2_A 			=		0x01
} LSM330DLC_TrigInt_A_t;

typedef enum {
  LSM330DLC_SPI_4_WIRE                    =               0x00,
  LSM330DLC_SPI_3_WIRE                    =               0x01
} LSM330DLC_SPIMode_t;

typedef enum {
  LSM330DLC_X_ENABLE_A                      =               0x01,
  LSM330DLC_X_DISABLE_A                     =               0x00,
  LSM330DLC_Y_ENABLE_A                      =               0x02,
  LSM330DLC_Y_DISABLE_A                     =               0x00,
  LSM330DLC_Z_ENABLE_A                      =               0x04,
  LSM330DLC_Z_DISABLE_A                     =               0x00    
} LSM330DLC_AXISenable_A_t;

typedef enum {
  LSM330DLC_INT1_6D_4D_DISABLE_A            =               0x00,
  LSM330DLC_INT1_6D_ENABLE_A                =               0x01,
#define LSM330DLC_ACC_ENABLE_ALL_AXES   (0x07)

  LSM330DLC_INT1_4D_ENABLE_A                =               0x02 
} LSM330DLC_INT_6D_4D_A_t;

typedef enum {
  LSM330DLC_UP_SX_A                         =               0x44,
  LSM330DLC_UP_DX_A                         =               0x42,
  LSM330DLC_DW_SX_A                         =               0x41,
  LSM330DLC_DW_DX_A                         =               0x48,
  LSM330DLC_TOP_A                           =               0x60,
  LSM330DLC_BOTTOM_A                        =               0x50
} LSM330DLC_POSITION_6D_A_t;

typedef enum {
  LSM330DLC_INT_MODE_OR_A                   =               0x00,
  LSM330DLC_INT_MODE_6D_MOVEMENT_A          =               0x01,
  LSM330DLC_INT_MODE_AND_A                  =               0x02,
  LSM330DLC_INT_MODE_6D_POSITION_A          =               0x03  
} LSM330DLC_Int1Mode_A_t;


//interrupt click response [b7-b0]
//  b7 = sign   b6 = double-single  b5, b4, b3 = don't care  
//  b2 = z      b1 = y     b0 = x
typedef enum {
LSM330DLC_DCLICK_Z_P                      =               0x44,
LSM330DLC_DCLICK_Z_N                      =               0xC4,
LSM330DLC_SCLICK_Z_P                      =               0x04,
LSM330DLC_SCLICK_Z_N                      =               0x84,
LSM330DLC_DCLICK_Y_P                      =               0x42,
LSM330DLC_DCLICK_Y_N                      =               0xC2,
LSM330DLC_SCLICK_Y_P                      =               0x02,
LSM330DLC_SCLICK_Y_N                      =               0x82,
LSM330DLC_DCLICK_X_P                      =               0x41,
LSM330DLC_DCLICK_X_N                      =               0xC1,
LSM330DLC_SCLICK_X_P                      =               0x01,
LSM330DLC_SCLICK_X_N                      =               0x81,
LSM330DLC_NO_CLICK                        =               0x00
} LSM330DLC_Click_Responce;




/******************************************************************/
/********************Gyro Typedef*********************************/


typedef enum {  
  LSM330DLC_ODR_95Hz_B0W_12_5_G            =		0x00,
  LSM330DLC_ODR_95Hz_BW_25_G		=		0x01,		
  LSM330DLC_ODR_190Hz_BW_12_5_G           =		0x04,
  LSM330DLC_ODR_190Hz_BW_25_G		=		0x05,
  LSM330DLC_ODR_190Hz_BW_50_G		=		0x06,
  LSM330DLC_ODR_190Hz_BW_70_G		=		0x07,	
  LSM330DLC_ODR_380Hz_BW_20_G		=		0x08,
  LSM330DLC_ODR_380Hz_BW_25_G		=		0x09,
  LSM330DLC_ODR_380Hz_BW_50_G		=		0x0A,
  LSM330DLC_ODR_380Hz_BW_110_G            =		0x0B,	
  LSM330DLC_ODR_760Hz_BW_30_G		=		0x0C,
  LSM330DLC_ODR_760Hz_BW_35_G		=		0x0D,
  LSM330DLC_ODR_760Hz_BW_50_G		=		0x0E,
  LSM330DLC_ODR_760Hz_BW_110_G     	=		0x0F
} LSM330DLC_ODR_G_t;

typedef enum {
  LSM330DLC_POWER_DOWN_G                    =		0x00,
  LSM330DLC_SLEEP_G 			=		0x01,
  LSM330DLC_NORMAL_G			=		0x02
} LSM330DLC_Mode_G_t;


typedef enum {
  LSM330DLC_PUSH_PULL_G                     =               0x00,
  LSM330DLC_OPEN_DRAIN_G                    =               0x01  
} LSM330DLC_IntPinMode_G_t;

typedef u8_t LSM330DLC_Axis_G_t;
typedef u8_t LSM330DLC_Int1PinConf_t;
typedef u8_t LSM330DLC_Int2PinConf_t;

typedef enum {
  LSM330DLC_FULLSCALE_250_G                 =               0x00,
  LSM330DLC_FULLSCALE_500_G                 =               0x01,
  LSM330DLC_FULLSCALE_2000_G                =               0x02	
} LSM330DLC_Fullscale_G_t;

typedef enum {
  LSM330DLC_BLE_LSB_G			=		0x00,
  LSM330DLC_BLE_MSB_G			=		0x01
} LSM330DLC_Endianess_G_t;

typedef enum {
  LSM330DLC_THS_X_G                         =               0x00,
  LSM330DLC_THS_Y_G                         =               0x01,  
  LSM330DLC_THS_Z_G                         =               0x02
} LSM330DLC_IntThsAxis_G;

typedef enum {
  LSM330DLC_FIFO_DISABLE_G                  =               0x05,
  LSM330DLC_FIFO_BYPASS_MODE_G              =               0x00,
  LSM330DLC_FIFO_MODE_G                     =               0x01,
  LSM330DLC_FIFO_STREAM_MODE_G              =               0x02,
  LSM330DLC_FIFO_STREAM_TO_FIFO_MODE_G      =               0x03,
  LSM330DLC_FIFO_BYPASS_TO_STREAM_MODE_G    =               0x04    
} LSM330DLC_FifoMode_G_t;

typedef enum {
  LSM330DLC_NONE_G                          =               0x00,
  LSM330DLC_HPF_G                           =               0x01,
  LSM330DLC_LPF2_G                          =               0x02,
  LSM330DLC_HPFLPF2_G                       =               0x03
} LSM330DLC_HPF_LPF2_Enable_G;

typedef struct{
  i16_t x;
  i16_t y;
  i16_t z;
} LSM330DLC_AngRateRaw_t;

typedef enum {
  LSM330DLC_Z_ENABLE_G                      =               0x04,
  LSM330DLC_Z_DISABLE_G                     =               0x00,
  LSM330DLC_Y_ENABLE_G                      =               0x02,
  LSM330DLC_Y_DISABLE_G                     =               0x00,
  LSM330DLC_X_ENABLE_G                      =               0x01,
  LSM330DLC_X_DISABLE_G                     =               0x00    
} LSM330DLC_AXISenable_G_t;


/************************************************/
/*  General defines                                                         */
/************************************************/
/*
#define LSM330DLC_MEMS_I2C_ADDRESS_A			0x32 //i2c addr. Write Device, SDO_A = 0
#define LSM330DLC_MEMS_I2C_ADDRESS_G            0xD6 //i2c addr. Write Device, SDO_G = 0
*/

#define LSM330DLC_ACC_DEV_NAME      "lsm330dlc_acc"
#define LSM330DLC_GYR_DEV_NAME      "lsm330dlc_gyr"


#define LSM330DLC_SAD0L         (0x00)
#define LSM330DLC_SAD0H         (0x01)

/************************************************/
/*  Accelerometer I2C address                   */
/************************************************/

#define LSM330DLC_ACC_I2C_SADROOT   (0x0C)
/* I2C address if acc SA0 pin to GND */
#define LSM330DLC_ACC_I2C_SAD_L     ((LSM330DLC_ACC_I2C_SADROOT<<1)|LSM330DLC_SAD0L)
/* I2C address if acc SA0 pin to Vdd */
#define LSM330DLC_ACC_I2C_SAD_H     ((LSM330DLC_ACC_I2C_SADROOT<<1)|LSM330DLC_SAD0H)

// This is the Accelerometer I2C address that we use on the Orisen Prime
//
#define LSM330DLC_I2C_ACC_ADDR				LSM330DLC_ACC_I2C_SAD_L
//#define LSM330DLC_MEMS_I2C_ADDRESS_A		((0x30)>>1)//LSM330DLC_I2C_ACC_ADDR
#define LSM330DLC_MEMS_I2C_ADDRESS_A		LSM330DLC_I2C_ACC_ADDR

/************************************************/
/*  Gyroscope I2C address                       */
/************************************************/

#define LSM330DLC_GYR_I2C_SADROOT       (0x35)

/* I2C address if gyr SA0 pin to GND */
#define LSM330DLC_GYR_I2C_SAD_L     ((LSM330DLC_GYR_I2C_SADROOT<<1)|LSM330DLC_SAD0L)
/* I2C address if gyr SA0 pin to Vdd */
#define LSM330DLC_GYR_I2C_SAD_H     ((LSM330DLC_GYR_I2C_SADROOT<<1)|LSM330DLC_SAD0H)
/* to set gpios numb connected to gyro interrupt pins,
 * the unused ones have to be set to -EINVAL
 */

// This is the Gyroscope I2C address that we use on the Orisen Prime
//
#define LSM330DLC_I2C_GYR_ADDR 				LSM330DLC_GYR_I2C_SAD_L
//#define LSM330DLC_MEMS_I2C_ADDRESS_G		((0xD4)>>1)//LSM330DLC_I2C_GYR_ADDR 
#define LSM330DLC_MEMS_I2C_ADDRESS_G		LSM330DLC_I2C_GYR_ADDR 



/* Exported constants ACC--------------------------------------------------------*/
#ifndef __SHARED__CONSTANTS
#define __SHARED__CONSTANTS

#define MEMS_SET                                        0x01
#define MEMS_RESET                                      0x00

#endif /*__SHARED__CONSTANTS*/



//Register Definition

// CONTROL REGISTER 1
#define LSM330DLC_CTRL_REG1_A				0x20
#define LSM330DLC_ODR_BIT_A			        BIT(4)
#define LSM330DLC_LPEN_A					BIT(3)
#define LSM330DLC_ZEN_A					BIT(2)
#define LSM330DLC_YEN_A					BIT(1)
#define LSM330DLC_XEN_A					BIT(0)

#define LSM330DLC_ACC_ENABLE_ALL_AXES   (0x07)

//CONTROL REGISTER 2
#define LSM330DLC_CTRL_REG2_A				0x21
#define LSM330DLC_HPM_A     				BIT(6)
#define LSM330DLC_HPCF_A					BIT(4)
#define LSM330DLC_FDS_A					BIT(3)
#define LSM330DLC_HPCLICK_A				BIT(2)
#define LSM330DLC_HPIS2_A					BIT(1)
#define LSM330DLC_HPIS1_A					BIT(0)

//CONTROL REGISTER 3
#define LSM330DLC_CTRL_REG3_A				0x22
#define LSM330DLC_I1_CLICK_A				BIT(7)
#define LSM330DLC_I1_AOI1_A				BIT(6)
#define LSM330DLC_I1_AOI2_A			        BIT(5)
#define LSM330DLC_I1_DRDY1_A				BIT(4)
#define LSM330DLC_I1_DRDY2_A				BIT(3)
#define LSM330DLC_I1_WTM_A				BIT(2)
#define LSM330DLC_I1_ORUN_A				BIT(1)

//CONTROL REGISTER 6
#define LSM330DLC_CTRL_REG6_A				0x25
#define LSM330DLC_I2_CLICK_A				BIT(7)
#define LSM330DLC_I2_INT1_A				BIT(6)
#define LSM330DLC_I2_BOOT_A         			BIT(4)
#define LSM330DLC_H_LACTIVE_A				BIT(1)

//TEMPERATURE CONFIG REGISTER
#define LSM330DLC_TEMP_CFG_REG_A				0x1F
#define LSM330DLC_ADC_PD_A			        BIT(7)
#define LSM330DLC_TEMP_EN_A				BIT(6)

//CONTROL REGISTER 4
#define LSM330DLC_CTRL_REG4_A				0x23
#define LSM330DLC_BLE_A					BIT(6)
#define LSM330DLC_FS_A					BIT(4)
#define LSM330DLC_HR_A					BIT(3)
#define LSM330DLC_SIM_A					BIT(0)

//CONTROL REGISTER 5
#define LSM330DLC_CTRL_REG5_A				0x24
#define LSM330DLC_BOOT_A                                  BIT(7)
#define LSM330DLC_FIFO_EN_A                               BIT(6)
#define LSM330DLC_LIR_INT1_A                              BIT(3)
#define LSM330DLC_D4D_INT1_A                              BIT(2)

//REFERENCE/DATA_CAPTURE
#define LSM330DLC_REFERENCE_REG_A		                0x26
#define LSM330DLC_REF_A		                	BIT(0)

//STATUS_REG_AXIES
#define LSM330DLC_STATUS_REG_A				0x27
#define LSM330DLC_ZYXOR                                   BIT(7)
#define LSM330DLC_ZOR                                     BIT(6)
#define LSM330DLC_YOR                                     BIT(5)
#define LSM330DLC_XOR                                     BIT(4)
#define LSM330DLC_ZYXDA                                   BIT(3)
#define LSM330DLC_ZDA                                     BIT(2)
#define LSM330DLC_YDA                                     BIT(1)
#define LSM330DLC_XDA                                     BIT(0)


//INTERRUPT 1 CONFIGURATION
#define LSM330DLC_INT1_CFG_A				0x30
#define LSM330DLC_ANDOR_A                                   BIT(7)
#define LSM330DLC_INT_6D_A                                  BIT(6)
#define LSM330DLC_ZHIE_A                                    BIT(5)
#define LSM330DLC_ZLIE_A                                    BIT(4)
#define LSM330DLC_YHIE_A                                    BIT(3)
#define LSM330DLC_YLIE_A                                    BIT(2)
#define LSM330DLC_XHIE_A                                    BIT(1)
#define LSM330DLC_XLIE_A                                    BIT(0)

//FIFO CONTROL REGISTER
#define LSM330DLC_FIFO_CTRL_REG_A                           0x2E
#define LSM330DLC_FM_A                                      BIT(6)
#define LSM330DLC_TR_A                                      BIT(5)
#define LSM330DLC_FTH_A                                     BIT(0)

//CONTROL REG3 bit mask
#define LSM330DLC_CLICK_ON_PIN_INT1_ENABLE_A                0x80
#define LSM330DLC_CLICK_ON_PIN_INT1_DISABLE_A               0x00
#define LSM330DLC_I1_INT1_ON_PIN_INT1_ENABLE_A              0x40
#define LSM330DLC_I1_INT1_ON_PIN_INT1_DISABLE_A             0x00
#define LSM330DLC_I1_INT2_ON_PIN_INT1_ENABLE_A              0x20
#define LSM330DLC_I1_INT2_ON_PIN_INT1_DISABLE_A             0x00
#define LSM330DLC_I1_DRDY1_ON_INT1_ENABLE_A                 0x10
#define LSM330DLC_I1_DRDY1_ON_INT1_DISABLE_A                0x00
#define LSM330DLC_I1_DRDY2_ON_INT1_ENABLE_A                 0x08
#define LSM330DLC_I1_DRDY2_ON_INT1_DISABLE_A                0x00
#define LSM330DLC_WTM_ON_INT1_ENABLE_A                      0x04
#define LSM330DLC_WTM_ON_INT1_DISABLE_A                     0x00
#define LSM330DLC_INT1_OVERRUN_ENABLE_A                     0x02
#define LSM330DLC_INT1_OVERRUN_DISABLE_A                    0x00

//CONTROL REG6 bit mask
#define LSM330DLC_CLICK_ON_PIN_INT2_ENABLE_A                0x80
#define LSM330DLC_CLICK_ON_PIN_INT2_DISABLE_A               0x00
#define LSM330DLC_I2_INT1_ON_PIN_INT2_ENABLE_A              0x40
#define LSM330DLC_I2_INT1_ON_PIN_INT2_DISABLE_A             0x00
#define LSM330DLC_I2_INT2_ON_PIN_INT2_ENABLE_A              0x20
#define LSM330DLC_I2_INT2_ON_PIN_INT2_DISABLE_A             0x00
#define LSM330DLC_I2_BOOT_ON_INT2_ENABLE_A                  0x10
#define LSM330DLC_I2_BOOT_ON_INT2_DISABLE_A                 0x00
#define LSM330DLC_INT_ACTIVE_HIGH_A                         0x00
#define LSM330DLC_INT_ACTIVE_LOW_A                          0x02

//INT1_CFG bit mask
#define LSM330DLC_INT1_AND_A                                0x80
#define LSM330DLC_INT1_OR_A                                 0x00
#define LSM330DLC_INT1_ZHIE_ENABLE_A                        0x20
#define LSM330DLC_INT1_ZHIE_DISABLE_A                       0x00
#define LSM330DLC_INT1_ZLIE_ENABLE_A                        0x10
#define LSM330DLC_INT1_ZLIE_DISABLE_A                       0x00
#define LSM330DLC_INT1_YHIE_ENABLE_A                        0x08
#define LSM330DLC_INT1_YHIE_DISABLE_A                       0x00
#define LSM330DLC_INT1_YLIE_ENABLE_A                        0x04
#define LSM330DLC_INT1_YLIE_DISABLE_A                       0x00
#define LSM330DLC_INT1_XHIE_ENABLE_A                        0x02
#define LSM330DLC_INT1_XHIE_DISABLE_A                       0x00
#define LSM330DLC_INT1_XLIE_ENABLE_A                        0x01
#define LSM330DLC_INT1_XLIE_DISABLE_A                       0x00

//INT1_SRC bit mask
#define LSM330DLC_INT1_SRC_IA_A                             0x40
#define LSM330DLC_INT1_SRC_ZH_A                             0x20
#define LSM330DLC_INT1_SRC_ZL_A                             0x10
#define LSM330DLC_INT1_SRC_YH_A                             0x08
#define LSM330DLC_INT1_SRC_YL_A                             0x04
#define LSM330DLC_INT1_SRC_XH_A                             0x02
#define LSM330DLC_INT1_SRC_XL_A                             0x01

//INT1 REGISTERS
#define LSM330DLC_INT1_THS_A                                0x32
#define LSM330DLC_INT1_DURATION_A                           0x33

//INTERRUPT 1 SOURCE REGISTER
#define LSM330DLC_INT1_SRC_A				0x31

//FIFO Source Register bit Mask
#define LSM330DLC_FIFO_SRC_WTM                            0x80
#define LSM330DLC_FIFO_SRC_OVRUN                          0x40
#define LSM330DLC_FIFO_SRC_EMPTY                          0x20
  
//INTERRUPT CLICK REGISTER
#define LSM330DLC_CLICK_CFG_A				0x38
//INTERRUPT CLICK CONFIGURATION bit mask
#define LSM330DLC_ZD_ENABLE_A                               0x20
#define LSM330DLC_ZD_DISABLE_A                              0x00
#define LSM330DLC_ZS_ENABLE_A                               0x10
#define LSM330DLC_ZS_DISABLE_A                              0x00
#define LSM330DLC_YD_ENABLE_A                               0x08
#define LSM330DLC_YD_DISABLE_A                              0x00
#define LSM330DLC_YS_ENABLE_A                               0x04
#define LSM330DLC_YS_DISABLE_A                              0x00
#define LSM330DLC_XD_ENABLE_A                               0x02
#define LSM330DLC_XD_DISABLE_A                              0x00
#define LSM330DLC_XS_ENABLE_A                               0x01
#define LSM330DLC_XS_DISABLE_A                              0x00

//INTERRUPT CLICK SOURCE REGISTER
#define LSM330DLC_CLICK_SRC_A                               0x39
//INTERRUPT CLICK SOURCE REGISTER bit mask
#define LSM330DLC_IA                                      0x40
#define LSM330DLC_DCLICK                                  0x20
#define LSM330DLC_SCLICK                                  0x10
#define LSM330DLC_CLICK_SIGN                              0x08
#define LSM330DLC_CLICK_Z                                 0x04
#define LSM330DLC_CLICK_Y                                 0x02
#define LSM330DLC_CLICK_X                                 0x01

//Click-click Register
#define LSM330DLC_CLICK_THS_A                               0x3A
#define LSM330DLC_TIME_LIMIT_A                              0x3B
#define LSM330DLC_TIME_LATENCY_A                            0x3C
#define LSM330DLC_TIME_WINDOW_A                             0x3D

//OUTPUT REGISTER
#define LSM330DLC_OUT_X_L_A					0x28
#define LSM330DLC_OUT_X_H_A					0x29
#define LSM330DLC_OUT_Y_L_A					0x2A
#define LSM330DLC_OUT_Y_H_A					0x2B
#define LSM330DLC_OUT_Z_L_A					0x2C
#define LSM330DLC_OUT_Z_H_A					0x2D


//STATUS REGISTER bit mask
#define LSM330DLC_STATUS_REG_ZYXOR                        0x80    // 1	:	new data set has over written the previous one
							// 0	:	no overrun has occurred (default)	
#define LSM330DLC_STATUS_REG_ZOR                          0x40    // 0	:	no overrun has occurred (default)
							// 1	:	new Z-axis data has over written the previous one
#define LSM330DLC_STATUS_REG_YOR                          0x20    // 0	:	no overrun has occurred (default)
							// 1	:	new Y-axis data has over written the previous one
#define LSM330DLC_STATUS_REG_XOR                          0x10    // 0	:	no overrun has occurred (default)
							// 1	:	new X-axis data has over written the previous one
#define LSM330DLC_STATUS_REG_ZYXDA                        0x08    // 0	:	a new set of data is not yet avvious one
                                                        // 1	:	a new set of data is available 
#define LSM330DLC_STATUS_REG_ZDA                          0x04    // 0	:	a new data for the Z-Axis is not availvious one
                                                        // 1	:	a new data for the Z-Axis is available
#define LSM330DLC_STATUS_REG_YDA                          0x02    // 0	:	a new data for the Y-Axis is not available
                                                        // 1	:	a new data for the Y-Axis is available
#define LSM330DLC_STATUS_REG_XDA                          0x01    // 0	:	a new data for the X-Axis is not available

#define LSM330DLC_DATAREADY_BIT                          STATUS_REG_ZYXDA

//FIFO REGISTERS
#define LSM330DLC_FIFO_CTRL_REG_A			        0x2E
#define LSM330DLC_FIFO_SRC_REG_A			        0x2F


/****************************************************************************/
/*************************Gyro Register**************************************/
/* Exported constants Gyro--------------------------------------------------------*/

//Register Definition
#define LSM330DLC_WHO_AM_I_G				(0x0F) // device identification register

// CONTROL REGISTER 1
#define LSM330DLC_CTRL_REG1_G				0x20
#define LSM330DLC_ODRR_G					BIT(4)
#define LSM330DLC_PD_G					BIT(3)
#define LSM330DLC_ZEN_G					BIT(2)
#define LSM330DLC_YEN_G					BIT(1)
#define LSM330DLC_XEN_G					BIT(0)

//CONTROL REGISTER 2
#define LSM330DLC_CTRL_REG2_G				0x21
#define LSM330DLC_HPM_G					BIT(4)
#define LSM330DLC_HPFC3_G					BIT(3)
#define LSM330DLC_HPFC2_G					BIT(2)
#define LSM330DLC_HPFC1_G					BIT(1)
#define LSM330DLC_HPFC0_G					BIT(0)

//CONTROL REGISTER 3
#define LSM330DLC_CTRL_REG3_G				0x22
#define LSM330DLC_I1_INT_G				BIT(7)
#define LSM330DLC_I1_BOOT_G				BIT(6)
#define LSM330DLC_H_LACTIVE_G				BIT(5)
#define LSM330DLC_PP_OD_G					BIT(4)
#define LSM330DLC_I2_DRDY_G				BIT(3)
#define LSM330DLC_I2_WTM_G				BIT(2)
#define LSM330DLC_I2_ORUN_G				BIT(1)
#define LSM330DLC_I2_EMPTY_G				BIT(0)

#define LSM330DLC_I1_ON_PIN_INT1_ENABLE_G                   0x80
#define LSM330DLC_I1_ON_PIN_INT1_DISABLE_G                  0x00
#define LSM330DLC_I1_BOOT_ON_INT1_ENABLE_G                  0x40
#define LSM330DLC_I1_BOOT_ON_INT1_DISABLE_G                 0x00
#define LSM330DLC_INT1_ACTIVE_HIGH_G                        0x00
#define LSM330DLC_INT1_ACTIVE_LOW_G                         0x20

#define LSM330DLC_I2_DRDY_ON_INT2_ENABLE_G                  0x08
#define LSM330DLC_I2_DRDY_ON_INT2_DISABLE_G                 0x00
#define LSM330DLC_WTM_ON_INT2_ENABLE_G                      0x04
#define LSM330DLC_WTM_ON_INT2_DISABLE_G                     0x00
#define LSM330DLC_OVERRUN_ON_INT2_ENABLE_G                  0x02
#define LSM330DLC_OVERRUN_ON_INT2_DISABLE_G                 0x00
#define LSM330DLC_EMPTY_ON_INT2_ENABLE_G                    0x01
#define LSM330DLC_EMPTY_ON_INT2_DISABLE_G                   0x00

//CONTROL REGISTER 4
#define LSM330DLC_CTRL_REG4_G				0x23
#define LSM330DLC_BDU_G					BIT(7)
#define LSM330DLC_BLE_G					BIT(6)
#define LSM330DLC_FS_G					BIT(4)
#define LSM330DLC_SIM_G					BIT(0)

//CONTROL REGISTER 5
#define LSM330DLC_CTRL_REG5_G				0x24
#define LSM330DLC_FIFO_EN_G                                 BIT(6)
#define LSM330DLC_HPEN_G                                    BIT(4)
#define LSM330DLC_INT1_SEL1_G                               BIT(3)
#define LSM330DLC_INT1_SEL0_G                               BIT(2)
#define LSM330DLC_OUT_SEL1_G                                BIT(1)

//INTERRUPT 1 CONFIGURATION
#define LSM330DLC_INT1_CFG_G				0x30
#define LSM330DLC_LIR_G                                     BIT(6)
#define LSM330DLC_ANDOR_G                                   BIT(7)
#define LSM330DLC_ZHIE_G                                    BIT(5)
#define LSM330DLC_ZLIE_G                                    BIT(4)
#define LSM330DLC_YHIE_G                                    BIT(3)
#define LSM330DLC_YLIE_G                                    BIT(2)
#define LSM330DLC_XHIE_G                                    BIT(1)
#define LSM330DLC_XLIE_G                                    BIT(0)


#define LSM330DLC_INT1_AND_G                                0x80
#define LSM330DLC_INT1_OR_G                                 0x00
#define LSM330DLC_INT1_LIR_ENABLE_G                         0x40
#define LSM330DLC_INT1_LIR_DISABLE_G                        0x00
#define LSM330DLC_INT1_ZHIE_ENABLE_G                        0x20
#define LSM330DLC_INT1_ZHIE_DISABLE_G                       0x00
#define LSM330DLC_INT1_ZLIE_ENABLE_G                        0x10
#define LSM330DLC_INT1_ZLIE_DISABLE_G                       0x00
#define LSM330DLC_INT1_YHIE_ENABLE_G                        0x08
#define LSM330DLC_INT1_YHIE_DISABLE_G                       0x00
#define LSM330DLC_INT1_YLIE_ENABLE_G                        0x04
#define LSM330DLC_INT1_YLIE_DISABLE_G                       0x00
#define LSM330DLC_INT1_XHIE_ENABLE_G                        0x02
#define LSM330DLC_INT1_XHIE_DISABLE_G                       0x00
#define LSM330DLC_INT1_XLIE_ENABLE_G                        0x01
#define LSM330DLC_INT1_XLIE_DISABLE_G                       0x00

#define LSM330DLC_INT1_THS_XH_G                             0x32
#define LSM330DLC_INT1_THS_XL_G                             0x33
#define LSM330DLC_INT1_THS_YH_G                             0x34
#define LSM330DLC_INT1_THS_YL_G                             0x35
#define LSM330DLC_INT1_THS_ZH_G                             0x36
#define LSM330DLC_INT1_THS_ZL_G                             0x37

//INTERRUPT 1 SOURCE REGISTER
#define LSM330DLC_INT1_SRC_G				0x31

//FIFO CONTROL REGISTER
#define LSM330DLC_FIFO_CTRL_REG_G                           0x2E
#define LSM330DLC_FM0_G                                     BIT(5)

//OUTPUT REGISTER
#define LSM330DLC_OUT_X_L_G					0x28
#define LSM330DLC_OUT_X_H_G					0x29
#define LSM330DLC_OUT_Y_L_G					0x2A
#define LSM330DLC_OUT_Y_H_G					0x2B
#define LSM330DLC_OUT_Z_L_G					0x2C
#define LSM330DLC_OUT_Z_H_G					0x2D


//STATUS REGISTER
#define LSM330DLC_STATUS_REG_G                            0x27

#define LSM330DLC_I_AM_L3GD20			        0xD4 

//FIFO REGISTERS
#define LSM330DLC_FIFO_CTRL_REG_G			        0x2E
#define LSM330DLC_FIFO_SRC_REG_G			        0x2F

//INT1 REGISTERS
#define LSM330DLC_INT1_TSH_XH_G				0x32
#define LSM330DLC_INT1_TSH_XL_G				0x33
#define LSM330DLC_INT1_TSH_YH_G				0x34
#define LSM330DLC_INT1_TSH_YL_G				0x35
#define LSM330DLC_INT1_TSH_ZH_G				0x36
#define LSM330DLC_INT1_TSH_ZL_G				0x37
#define LSM330DLC_INT1_DURATION_G			        0x38





/* Exported macro ------------------------------------------------------------*/

#ifndef __SHARED__MACROS

#define __SHARED__MACROS
#define ValBit(VAR,Place)         (VAR & (1<<Place))
#define BIT(x) ( (x) )

#endif /*__SHARED__MACROS*/

/* Exported functions Acc--------------------------------------------------------*/
//Generic
u8_t LSM330DLC_ReadReg(u8_t deviceAddr, u8_t Reg, u8_t* Data);
u8_t LSM330DLC_WriteReg(u8_t deviceAddr, u8_t WriteAddr, u8_t Data);

//Sensor Configuration Functions
status_t LSM330DLC_SetODR_A(LSM330DLC_ODR_A_t ov);
status_t LSM330DLC_SetMode_A(LSM330DLC_Mode_A_t md);
status_t LSM330DLC_SetAxis_A(LSM330DLC_AXISenable_A_t axis);
status_t LSM330DLC_SetFullScale_A(LSM330DLC_Fullscale_A_t fs);
status_t LSM330DLC_SetBLE_A(LSM330DLC_Endianess_t ble);

//Filtering Functions
status_t LSM330DLC_HPFClickEnable_A(State_t hpfe);
status_t LSM330DLC_HPFAOI1Enable_A(State_t hpfe);
status_t LSM330DLC_HPFAOI2Enable_A(State_t hpfe);
status_t LSM330DLC_SetHPFMode_A(LSM330DLC_HPFMode_t hpf);
status_t LSM330DLC_SetHPFCutOFF_A(LSM330DLC_HPFCutOffFreq_t hpf);
status_t LSM330DLC_SetFilterDataSel_A(State_t state);

//Interrupt Functions
status_t LSM330DLC_SetInt1Pin_A(LSM330DLC_IntPinConf_t pinConf);
status_t LSM330DLC_SetInt2Pin_A(LSM330DLC_IntPinConf_t pinConf);
status_t LSM330DLC_Int1LatchEnable_A(State_t latch);
status_t LSM330DLC_ResetInt1Latch_A(void);
status_t LSM330DLC_SetIntConfiguration_A(LSM330DLC_Int1Conf_t ic);
status_t LSM330DLC_SetInt1Threshold_A(u8_t ths);
status_t LSM330DLC_SetInt1Duration_A(LSM330DLC_Int1Conf_t id);
status_t LSM330DLC_SetIntMode_A(LSM330DLC_Int1Mode_A_t ic);
status_t LSM330DLC_SetClickCFG_A(u8_t status);
status_t LSM330DLC_SetInt6D4DConfiguration_A(LSM330DLC_INT_6D_4D_A_t ic);
status_t LSM330DLC_GetInt1Src_A(u8_t* val);
status_t LSM330DLC_GetInt1SrcBit_A(u8_t statusBIT);

//FIFO Functions
status_t LSM330DLC_FIFOModeEnable_A(LSM330DLC_FifoMode_A_t fm);
status_t LSM330DLC_SetWaterMark_A(u8_t wtm);
status_t LSM330DLC_SetTriggerInt_A(LSM330DLC_TrigInt_A_t tr);
status_t LSM330DLC_GetFifoSourceReg_A(u8_t* val);
status_t LSM330DLC_GetFifoSourceBit_A(u8_t statusBIT);
status_t LSM330DLC_GetFifoSourceFSS_A(u8_t* val);

//Other Reading Functions
status_t LSM330DLC_GetStatusReg_A(u8_t* val);
status_t LSM330DLC_GetStatusBit_A(u8_t statusBIT);
status_t LSM330DLC_GetAccAxesRaw_A(AxesRaw_t* buff);
status_t LSM330DLC_GetClickResponce_A(u8_t* val);
status_t LSM330DLC_Get6DPosition_A(u8_t* val);



/* Exported functions Gyro-------------------------------------------------------*/
//Sensor Configuration Functions
status_t LSM330DLC_SetODR_G(LSM330DLC_ODR_G_t ov);
status_t LSM330DLC_SetMode_G(LSM330DLC_Mode_G_t md);
//status_t SetAxis(State_t x, State_t y, State_t z);
status_t LSM330DLC_SetAxis_G(LSM330DLC_AXISenable_G_t axis);
status_t LSM330DLC_SetFullScale_G(LSM330DLC_Fullscale_G_t fs);
status_t LSM330DLC_SetBDU_G(State_t bdu);
status_t LSM330DLC_SetBLE_G(LSM330DLC_Endianess_t ble);

//Filtering Functions
status_t LSM330DLC_HPFEnable_G(State_t hpf);
status_t LSM330DLC_SetHPFMode_G(LSM330DLC_HPFMode_t hpf);
status_t LSM330DLC_SetHPFCutOFF_G(LSM330DLC_HPFCutOffFreq_t hpf);
status_t LSM330DLC_SetOutputDataAndFifoFilters_G(LSM330DLC_HPF_LPF2_Enable_G hpf);
status_t LSM330DLC_SetInt1Filters_G(LSM330DLC_HPF_LPF2_Enable_G hpf);

//Interrupt Functions
status_t LSM330DLC_SetIntPinMode_G(LSM330DLC_IntPinMode_G_t pm);
status_t LSM330DLC_SetInt1Pin_G(LSM330DLC_Int1PinConf_t pinConf);
status_t LSM330DLC_SetInt2Pin_G(LSM330DLC_Int2PinConf_t pinConf);
status_t LSM330DLC_Int1LatchEnable_G(State_t latch);
status_t LSM330DLC_ResetInt1Latch_G(void);
///////////////fino a qui////////////////////
status_t LSM330DLC_SetIntConfiguration_G(LSM330DLC_Int1Conf_t ic);
status_t LSM330DLC_SetInt1Threshold_G(LSM330DLC_IntThsAxis_G axis, u16_t ths);
status_t LSM330DLC_SetInt1Duration_G(LSM330DLC_Int1Conf_t id);

//FIFO Functions
status_t LSM330DLC_FIFOModeEnable(LSM330DLC_FifoMode_G_t fm);
status_t LSM330DLC_SetWaterMark(u8_t wtm);

//Reading Functions
status_t LSM330DLC_GetStatusReg(u8_t* buff);
status_t LSM330DLC_GetAngRateRaw(LSM330DLC_AngRateRaw_t* buff);
status_t LSM330DLC_GetInt1Src(u8_t* buff);
status_t LSM330DLC_GetFifoSourceReg(u8_t* buff);
status_t LSM330DLC_GetWHO_AM_I(u8_t* val);

#endif /* __LSM330DLC_H */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/



