#ifndef __LSM330DLC_DRIVER__H
#define __LSM330DLC_DRIVER__H

#include "lib/sensors.h"    // for Contiki sensor interface
#include "i2c.h"

/*****************************************************************************************
 *  Driver defines
 ****************************************************************************************/

// Contiki sensor interface related
extern const struct sensors_sensor lsm330dlc_sensor;

#define LSM330DLC_LATCH_DATA	0	// This must be invoked first
#define LSM330DLC_ACC_X_RAW		1
#define LSM330DLC_ACC_Y_RAW		2
#define LSM330DLC_ACC_Z_RAW		3
#define LSM330DLC_GYR_X_RAW		4
#define LSM330DLC_GYR_Y_RAW		5
#define LSM330DLC_GYR_Z_RAW		6

// Used in conjunction with the Contiki's configure interface
// Power Mode
// 0 and 1 are used by Contiki to avtivate and deactivate a sensor
#define LSM330DLC_SET_POWER_LOW						((1)<<1)
#define LSM330DLC_SET_POWER_NORMAL					((1)<<2)

// Sampling Frequencies
#define LSM330DLC_SET_ACC_FREQ_1Hz						((1)<<3)
#define LSM330DLC_SET_ACC_FREQ_10Hz						((1)<<4)
#define LSM330DLC_SET_ACC_FREQ_25Hz						((1)<<5)
#define LSM330DLC_SET_ACC_FREQ_50Hz						((1)<<6)
#define LSM330DLC_SET_ACC_FREQ_100Hz					((1)<<7)
#define LSM330DLC_SET_ACC_FREQ_200Hz					((1)<<8)
#define LSM330DLC_SET_ACC_FREQ_400Hz					((1)<<9)
#define LSM330DLC_SET_ACC_FREQ_1620kHz_LP				((1)<<10)
#define LSM330DLC_SET_ACC_FREQ_1344Hz_NP_5367HZ_LP		((1)<<11)

// FIFO Mode Configurations
#define LSM330DLC_SET_FIFO_MODE_DISABLE				((1)<<12)
#define LSM330DLC_SET_FIFO_MODE_BYPASS				((1)<<13)
#define LSM330DLC_SET_FIFO_MODE_FIFO				((1)<<14)	
#define LSM330DLC_SET_FIFO_MODE_STREAM				((1)<<15)
#define LSM330DLC_SET_FIFO_MODE_TRIGGER				((1)<<16)
#define LSM330DLC_SET_FIFO_MODE_STREAM_TO_FIFO		((1)<<17)
#define LSM330DLC_SET_FIFO_MODE_BYPASS_TO_STREAM	((1)<<18)

// Full scale selection for the accelerometer: Default value: 2G
#define LSM330DLC_SET_ACC_FULLSCALE_2G				((1)<<19)
#define LSM330DLC_SET_ACC_FULLSCALE_4G				((1)<<20)
#define LSM330DLC_SET_ACC_FULLSCALE_8G				((1)<<21)
#define LSM330DLC_SET_ACC_FULLSCALE_16G				((1)<<22)

// Full scale selection for the gyroscope
#define LSM330DLC_SET_GYR_FULLSCALE_250				((1)<<23)
#define LSM330DLC_SET_GYR_FULLSCALE_500				((1)<<24)
#define LSM330DLC_SET_GYR_FULLSCALE_2000			((1)<<25)

// High resolution output mode: Default disabled
#define LSM330DLC_SET_HIGHRES_DISABLE				((1)<<26)
#define LSM330DLC_SET_HIGHRES_ENABLE				((1)<<27)

// Sampling Frequencies for the gyroscope
// Choose the highest bandwidth by default
#define LSM330DLC_SET_GYR_FREQ_95Hz					((1)<<28)
#define LSM330DLC_SET_GYR_FREQ_190Hz				((1)<<29)
#define LSM330DLC_SET_GYR_FREQ_380Hz				((1)<<30)
#define LSM330DLC_SET_GYR_FREQ_760Hz				((1)<<31)



struct lsm330dlc_data {
	int16_t acc_x ;
	int16_t acc_y ;
	int16_t acc_z ;
	int16_t gyr_x ;
	int16_t gyr_y ;
	int16_t gyr_z ;
} ;

/*****************************************************************************************
 *  General defines
 ****************************************************************************************/
#define LSM330DLC_ACC_DEV_NAME      				"lsm330dlc_acc"
#define LSM330DLC_GYR_DEV_NAME      				"lsm330dlc_gyr"

#define LSM330DLC_SAD0L         					(0x00)
#define LSM330DLC_SAD0H         					(0x01)

#define	I2C_AUTO_INCREMENT							(0x80)

#define MEMS_SUCCESS								1
#define	MEMS_ERROR									0

#define MEMS_SET                                    0x01
#define MEMS_RESET                                  0x00

#define BIT(x) ( (x) )

/*****************************************************************************************
 *  Accelerometer I2C address
 ****************************************************************************************/
#define LSM330DLC_ACC_I2C_SADROOT   				(0x0C)

/* I2C address if acc SA0 pin to GND */
#define LSM330DLC_ACC_I2C_SAD_L     				((LSM330DLC_ACC_I2C_SADROOT<<1)|LSM330DLC_SAD0L)
/* I2C address if acc SA0 pin to Vdd */
#define LSM330DLC_ACC_I2C_SAD_H     				((LSM330DLC_ACC_I2C_SADROOT<<1)|LSM330DLC_SAD0H)

// This is the Accelerometer I2C address that we use on the Orisen Prime
//
#define LSM330DLC_I2C_ACC_ADDR							LSM330DLC_ACC_I2C_SAD_L

/*****************************************************************************************
 *  Gyroscope I2C address                       
 ****************************************************************************************/
#define LSM330DLC_GYR_I2C_SADROOT						(0x35)

/* I2C address if gyr SA0 pin to GND */
#define LSM330DLC_GYR_I2C_SAD_L     				((LSM330DLC_GYR_I2C_SADROOT<<1)|LSM330DLC_SAD0L)
/* I2C address if gyr SA0 pin to Vdd */
#define LSM330DLC_GYR_I2C_SAD_H     				((LSM330DLC_GYR_I2C_SADROOT<<1)|LSM330DLC_SAD0H)

// This is the Gyroscope I2C address that we use on the Orisen Prime
//
#define LSM330DLC_I2C_GYR_ADDR 							LSM330DLC_GYR_I2C_SAD_L


/*****************************************************************************************
 * Accelerometer defines
 *****************************************************************************************/

// TEMP_CFG_REG_A
#define LSM330DLC_TEMP_CFG_REG_A						0x1F

// CTRL_REG1_A
#define LSM330DLC_CTRL_REG1_A							0x20
#define LSM330DLC_ODR_POWER_DOWN_A						0x00
#define LSM330DLC_ODR_1Hz_A		    					0x10
#define LSM330DLC_ODR_10Hz_A							0x20
#define LSM330DLC_ODR_25Hz_A		  					0x30
#define LSM330DLC_ODR_50Hz_A		  					0x40
#define LSM330DLC_ODR_100Hz_A		  					0x50	
#define LSM330DLC_ODR_200Hz_A		  					0x60
#define LSM330DLC_ODR_400Hz_A		  					0x70
#define LSM330DLC_ODR_1620Hz_LP_A	        			0x80
#define LSM330DLC_ODR_1344Hz_NP_5367HZ_LP_A				0x90	
#define LSM330DLC_LPEN_A								0x08
#define LSM330DLC_X_ENABLE_A             				0x01
#define LSM330DLC_Y_ENABLE_A               				0x02
#define LSM330DLC_Z_ENABLE_A               				0x04
#define LSM330DLC_ACC_ENABLE_ALL_AXES   				0x07

// Power mode for the accelerometer
typedef enum {
  LSM330DLC_POWER_DOWN_A        =       0x00,
  LSM330DLC_LOW_POWER_A         =       0x01,
  LSM330DLC_NORMAL_A            =       0x02
} LSM330DLC_Mode_A_t;

// CTRL_REG2_A
#define LSM330DLC_CTRL_REG2_A							0x21
#define LSM330DLC_HPM_NORMAL_MODE_RES       0x00
#define LSM330DLC_HPM_REF_SIGNAL            0x40
#define LSM330DLC_HPM_NORMAL_MODE           0x80
#define LSM330DLC_HPM_AUTORESET_INT         0xC0
#define LSM330DLC_HPFCF_0                   0x00
#define LSM330DLC_HPFCF_1                   0x10
#define LSM330DLC_HPFCF_2                   0x20
#define LSM330DLC_HPFCF_3                   0x30
#define LSM330DLC_FDS                   		0x08
#define LSM330DLC_HPCLICK                   0x04
#define LSM330DLC_HPIS_0                   	0x00
#define LSM330DLC_HPIS_1                   	0x01
#define LSM330DLC_HPIS_2                   	0x02
#define LSM330DLC_HPIS_3                   	0x03

// CTRL_REG3_A
#define LSM330DLC_CTRL_REG3_A								0x22
#define LSM330DLC_I1_CLICK_A								0x80
#define LSM330DLC_I1_AOI1_A             		0x40
#define LSM330DLC_I1_DRDY1_A             		0x10
#define LSM330DLC_I1_DRDY2_A             		0x08
#define LSM330DLC_I1_WTM_A             			0x04
#define LSM330DLC_I1_OVERRUN_A            	0x02

// CTRL_REG4_A
#define LSM330DLC_CTRL_REG4_A								0x23
#define LSM330DLC_BLE_A											0x40
#define LSM330DLC_FULLSCALE_2_A             0x00
#define LSM330DLC_FULLSCALE_4_A             0x10
#define LSM330DLC_FULLSCALE_8_A             0x20
#define LSM330DLC_FULLSCALE_16_A            0x30
#define LSM330DLC_HR_A            					0x08
#define LSM330DLC_SIM_A            					0x01

// CTRL_REG5_A
#define LSM330DLC_CTRL_REG5_A								0x24
#define LSM330DLC_BOOT_A									0x80
#define LSM330DLC_FIFO_EN_A									0x40
#define LSM330DLC_LIR_INT1_A								0x08
#define LSM330DLC_D4D_INT1_A								0x04

// CTRL_REG6_A
#define LSM330DLC_CTRL_REG6_A								0x25
#define LSM330DLC_I2_CLICKen_A								0x80
#define LSM330DLC_I2_INT1_A									0x40
#define LSM330DLC_BOOT_I2_A									0x10
#define LSM330DLC_H_LACTIVE_A								0x02

// REFERENCE/DATACAPTURE_A
#define LSM330DLC_REFERENCE_REG_A							0x26

// STATUS_REG_A
#define LSM330DLC_STATUS_REG_A								0x27
#define LSM330DLC_ZYXOR_A									0x80
#define LSM330DLC_ZOR_A										0x40
#define LSM330DLC_YOR_A										0x20
#define LSM330DLC_XOR_A										0x10
#define LSM330DLC_ZYXDA_A									0x08
#define LSM330DLC_ZDA_A										0x04
#define LSM330DLC_YDA_A										0x02
#define LSM330DLC_XDA_A										0x01
#define LSM330DLC_DATAREADY_BIT_A     			LSM330DLC_ZYXDA_A

// OUTPUT REGISTERS
#define LSM330DLC_OUT_X_L_A									0x28
#define LSM330DLC_OUT_X_H_A									0x29
#define LSM330DLC_OUT_Y_L_A									0x2A
#define LSM330DLC_OUT_Y_H_A									0x2B
#define LSM330DLC_OUT_Z_L_A									0x2C
#define LSM330DLC_OUT_Z_H_A									0x2D

// FIFO_CTRL_REG_A
#define LSM330DLC_FIFO_CTRL_REG_A   						0x2E
#define LSM330DLC_FM_BYPASS_MODE_A							0x00
#define LSM330DLC_FM_FIFO_MODE_A							0x40
#define LSM330DLC_FM_STREAM_MODE_A							0x80
#define LSM330DLC_FM_TRIGGER_MODE_A							0xC0
#define LSM330DLC_TR_A										0x20
#define LSM330DLC_FTHMASK_A									0x1F
#define LSM330DLC_FM_A                                      BIT(6)

// Alternative to the above defs
typedef enum {
  LSM330DLC_FIFO_BYPASS_MODE_A              =               0x00,
  LSM330DLC_FIFO_MODE_A                     =               0x01,
  LSM330DLC_FIFO_STREAM_MODE_A              =               0x02,
  LSM330DLC_FIFO_TRIGGER_MODE_A             =               0x03,
  LSM330DLC_FIFO_DISABLE_A                  =               0x04
} LSM330DLC_FifoMode_A_t;

// FIFO_SRC_REG_A
#define LSM330DLC_FIFO_SRC_REG_A						0x2F
#define LSM330DLC_WTM_A                    	0x80
#define LSM330DLC_OVRUN_A                  	0x40
#define LSM330DLC_EMPTY_A                  	0x20
#define LSM330DLC_FSSMASK_A									0x1F

// INT1_CFG_A
#define LSM330DLC_INT1_CFG_A								0x30
#define LSM330DLC_AOI_A               			0x80
#define LSM330DLC_6D_A                			0x40
#define LSM330DLC_ZHIE_A              			0x20
#define LSM330DLC_ZLIE_A              			0x10
#define LSM330DLC_YHIE_A              			0x08
#define LSM330DLC_YLIE_A              			0x04
#define LSM330DLC_XHIE_A              			0x02
#define LSM330DLC_XLIE_A              			0x01

// INT1_SRC_A
#define LSM330DLC_INT1_SRC_A								0x31
#define LSM330DLC_IA_A                   		0x40
#define LSM330DLC_ZH_A                   		0x20
#define LSM330DLC_ZL_A                   		0x10
#define LSM330DLC_YH_A                   		0x08
#define LSM330DLC_YL_A                   		0x04
#define LSM330DLC_XH_A                   		0x02
#define LSM330DLC_XL_A                   		0x01

// INT1_THS_A
#define LSM330DLC_INT1_THS_A                0x32

// INT1_DURATION_A
#define LSM330DLC_INT1_DURATION_A           0x33

// CLICK_CFG _A
#define LSM330DLC_CLICK_CFG_A								0x38
#define LSM330DLC_ZD_A                     	0x20
#define LSM330DLC_ZS_A                     	0x10
#define LSM330DLC_YD_A                     	0x08
#define LSM330DLC_YS_A                     	0x04
#define LSM330DLC_XD_A                     	0x02
#define LSM330DLC_XS_A                     	0x01

// CLICK_SRC_A
#define LSM330DLC_CLICK_SRC_A               0x39
#define LSM330DLC_IA                        0x40
#define LSM330DLC_DCLICK                    0x20
#define LSM330DLC_SCLICK                    0x10
#define LSM330DLC_CLICK_SIGN                0x08
#define LSM330DLC_CLICK_Z                   0x04
#define LSM330DLC_CLICK_Y                   0x02
#define LSM330DLC_CLICK_X                   0x01

// CLICK_THS_A
#define LSM330DLC_CLICK_THS_A               0x3A

// TIME_LIMIT_A
#define LSM330DLC_TIME_LIMIT_A              0x3B

// TIME_LATENCY_A
#define LSM330DLC_TIME_LATENCY_A            0x3C

// TIME WINDOW_A
#define LSM330DLC_TIME_WINDOW_A             0x3D

// Act_THS
#define LSM330DLC_Act_THS_A             		0x3E

// Act_DUR
#define LSM330DLC_Act_DUR_A             		0x3f

/*****************************************************************************************
 * Gyro defines
*****************************************************************************************/

// WHO_AM_I_G
#define LSM330DLC_WHO_AM_I_G								0x0F
#define LSM330DLC_I_AM_L3GD20			  				0xD4

// CTRL_REG1_G
#define LSM330DLC_CTRL_REG1_G						0x20
#define LSM330DLC_ODR_95Hz_BW_12_5_G       			0x00
#define LSM330DLC_ODR_95Hz_BW_25_G					0x10	
#define LSM330DLC_ODR_190Hz_BW_12_5_G       		0x40
#define LSM330DLC_ODR_190Hz_BW_25_G					0x50
#define LSM330DLC_ODR_190Hz_BW_50_G					0x60
#define LSM330DLC_ODR_190Hz_BW_70_G					0x70
#define LSM330DLC_ODR_380Hz_BW_20_G					0x80
#define LSM330DLC_ODR_380Hz_BW_25_G					0x90
#define LSM330DLC_ODR_380Hz_BW_50_G					0xA0
#define LSM330DLC_ODR_380Hz_BW_110_G       			0xB0
#define LSM330DLC_ODR_760Hz_BW_30_G					0xC0
#define LSM330DLC_ODR_760Hz_BW_35_G					0xD0
#define LSM330DLC_ODR_760Hz_BW_50_G					0xE0
#define LSM330DLC_ODR_760Hz_BW_110_G     			0xF0
#define LSM330DLC_PD_G											0x08
#define LSM330DLC_ZEN_G											0x04
#define LSM330DLC_YEN_G											0x01
#define LSM330DLC_XEN_G											0x02

// Power modes for the gyroscope
typedef enum {
  LSM330DLC_POWER_DOWN_G        =       0x00,
  LSM330DLC_SLEEP_G             =       0x01,
  LSM330DLC_NORMAL_G            =       0x02
} LSM330DLC_Mode_G_t;

//CTRL_REG2_G
#define LSM330DLC_CTRL_REG2_G								0x21
#define LSM330DLC_EXTRen_G									0x80
#define LSM330DLC_LVLen_G										0x40
#define LSM330DLC_HPM_NORMAL_RESET_G				0x00
#define LSM330DLC_HPM_REFERENCE_G						0x10
#define LSM330DLC_HPM_NORMAL_G							0x20
#define LSM330DLC_HPM_AUTORESET_G						0x30
#define LSM330DLC_HPFCMASK_G								0x0F

//CTRL_REG3_G
#define LSM330DLC_CTRL_REG3_G								0x22
#define LSM330DLC_I1_Int1_G									0x80
#define LSM330DLC_I1_Boot_G									0x40
#define LSM330DLC_H_Lactive_G								0x20
#define LSM330DLC_PP_OD_G										0x10
#define LSM330DLC_I2_DRDY_G									0x08
#define LSM330DLC_I2_WTM_G									0x04
#define LSM330DLC_I2_ORun_G									0x02
#define LSM330DLC_I2_Empty_G								0x01

//CTRL_REG4_G
#define LSM330DLC_CTRL_REG4_G								0x23
#define LSM330DLC_BDU_G											0x80
#define LSM330DLC_BLE_G											0x40
#define LSM330DLC_FS_250_G           				0x00
#define LSM330DLC_FS_500_G           				0x10
#define LSM330DLC_FS_2000_G          				0x20	
#define LSM330DLC_SIM_G											0x01

//CTRL_REG5_G
#define LSM330DLC_CTRL_REG5_G								0x24
#define LSM330DLC_BOOT_G         						0x80
#define LSM330DLC_FIFO_EN_G         				0x40
#define LSM330DLC_HPEN_G            				0x10
#define LSM330DLC_INT1_SEL1_G    					  0x08
#define LSM330DLC_INT1_SEL0_G     				  0x04
#define LSM330DLC_OUT_SEL1_G       				 	0x02
#define LSM330DLC_OUT_SEL0_G       				 	0x01

// REFERENCE_G
#define LSM330DLC_REFERENCE_G								0x25

// OUT_TEMP_G
#define LSM330DLC_OUT_TEMP_G								0x26

// STATUS_REG_G
#define LSM330DLC_STATUS_REG_G      						0x27
#define LSM330DLC_ZYXOR_G									0x80
#define LSM330DLC_ZOR_G										0x40
#define LSM330DLC_YOR_G										0x20
#define LSM330DLC_XOR_G										0x10
#define LSM330DLC_ZYXDA_G									0x08
#define LSM330DLC_ZDA_G										0x04
#define LSM330DLC_YDA_G										0x02
#define LSM330DLC_XDA_G										0x01
#define LSM330DLC_DATAREADY_BIT_G     			LSM330DLC_ZYXDA_G

// OUTPUT REGISTERS
#define LSM330DLC_OUT_X_L_G									0x28
#define LSM330DLC_OUT_X_H_G									0x29
#define LSM330DLC_OUT_Y_L_G									0x2A
#define LSM330DLC_OUT_Y_H_G									0x2B
#define LSM330DLC_OUT_Z_L_G									0x2C
#define LSM330DLC_OUT_Z_H_G									0x2D

// FIFO_CTRL_REG_G
#define LSM330DLC_FIFO_CTRL_REG_G   						0x2E
#define LSM330DLC_FM_BYPASS_MODE_G							0x00
#define LSM330DLC_FM_FIFO_MODE_G							0x20
#define LSM330DLC_FM_STREAM_MODE_G							0x40
#define LSM330DLC_FM_STREAM_TO_FIFO_MODE_G					0x60
#define LSM330DLC_FM_BYPASS_TO_STREAM_MODE_G				0x80
#define LSM330DLC_WTMMASK_G									0x1F
#define LSM330DLC_FM0_G                                     BIT(5)

// Alternative to the above defs
typedef enum {
  LSM330DLC_FIFO_DISABLE_G                  =               0x05,
  LSM330DLC_FIFO_BYPASS_MODE_G              =               0x00,
  LSM330DLC_FIFO_MODE_G                     =               0x01,
  LSM330DLC_FIFO_STREAM_MODE_G              =               0x02,
  LSM330DLC_FIFO_STREAM_TO_FIFO_MODE_G      =               0x03,
  LSM330DLC_FIFO_BYPASS_TO_STREAM_MODE_G    =               0x04
} LSM330DLC_FifoMode_G_t;


// FIFO_SRC_REG_G
#define LSM330DLC_FIFO_SRC_REG_G						0x2F
#define LSM330DLC_WTM_G                    	0x80
#define LSM330DLC_OVRUN_G                  	0x40
#define LSM330DLC_EMPTY_G                  	0x20
#define LSM330DLC_FSSMASK_G									0x1F

// INT1_CFG_G
#define LSM330DLC_INT1_CFG_G								0x30
#define LSM330DLC_ANDOR_G              			0x80
#define LSM330DLC_LIR_G                			0x40
#define LSM330DLC_ZHIE_G              			0x20
#define LSM330DLC_ZLIE_G              			0x10
#define LSM330DLC_YHIE_G              			0x08
#define LSM330DLC_YLIE_G              			0x04
#define LSM330DLC_XHIE_G              			0x02
#define LSM330DLC_XLIE_G              			0x01

// INT1_SRC_G
#define LSM330DLC_INT1_SRC_G								0x31
#define LSM330DLC_IA_G                   		0x40
#define LSM330DLC_ZH_G                   		0x20
#define LSM330DLC_ZL_G                   		0x10
#define LSM330DLC_YH_G                   		0x08
#define LSM330DLC_YL_G                   		0x04
#define LSM330DLC_XH_G                   		0x02
#define LSM330DLC_XL_G                   		0x01

// INT1_THS_XH_G
#define LSM330DLC_INT1_THS_XH_G							0x32

// INT1_THS_XL_G
#define LSM330DLC_INT1_THS_XL_G							0x33

// INT1_THS_YH_G
#define LSM330DLC_INT1_THS_YH_G							0x34

// INT1_THS_YL_G
#define LSM330DLC_INT1_THS_YL_G							0x35

// INT1_THS_ZH_G
#define LSM330DLC_INT1_THS_ZH_G							0x36

// INT1_THS_ZL_G
#define LSM330DLC_INT1_THS_ZL_G							0x37

// INT1_DURATION_G
#define LSM330DLC_INT1_DURATION_G						0x38
#define LSM330DLC_WAIT_G										0x80
#define LSM330DLC_DMASK_G										0x7F

#endif /* __LSM330DLC_H */



