/*
 * Copyright (c) 2013, Dominik Beste <dbeste@cs.ucl.ac.uk>
 *
 * Driver for the MPU9150 chip used on the OrisenSpectrum.
 */

#ifndef MPU9150_H_
#define MPU9150_H_

extern const struct sensors_sensor mpu9150_sensor;

#define MPU9150_I2C_ADDR			((0b11010010)>>1)

//Magnetometer Registers
#define MPU9150_RA_MAG_ADDRESS 0x0C
#define MPU9150_RA_MAG_XOUT_L 0x03
#define MPU9150_RA_MAG_XOUT_H 0x04
#define MPU9150_RA_MAG_YOUT_L 0x05
#define MPU9150_RA_MAG_YOUT_H 0x06
#define MPU9150_RA_MAG_ZOUT_L 0x07
#define MPU9150_RA_MAG_ZOUT_H 0x08

#define MPU9150_SMPLRT_DIV 			0x19
#define MPU9150_CONFIG 				0x1A
#define MPU9150_GYRO_CONFIG 			0x1B
#define MPU9150_ACCEL_CONFIG 			0x1C
#define MPU9150_FF_THR 				0x1D
#define MPU9150_FF_DUR 				0x1E
#define MPU9150_MOT_THR 			0x1F
#define MPU9150_MOT_DUR 			0x20
#define MPU9150_ZRMOT_THR 			0x21
#define MPU9150_ZRMOT_DUR 			0x22
#define MPU9150_FIFO_EN 			0x23
#define MPU9150_I2C_MST_CTRL 			0x24
#define MPU9150_I2C_SLV0_ADDR 			0x25
#define MPU9150_I2C_SLV0_REG 			0x26
#define MPU9150_I2C_SLV0_CTRL 			0x27
#define MPU9150_I2C_SLV1_ADDR 			0x28
#define MPU9150_I2C_SLV1_REG 			0x29
#define MPU9150_I2C_SLV1_CTRL 			0x2A
#define MPU9150_I2C_SLV2_ADDR 			0x2B
#define MPU9150_I2C_SLV2_REG 			0x2C
#define MPU9150_I2C_SLV2_CTRL 			0x2D
#define MPU9150_I2C_SLV3_ADDR 			0x2E
#define MPU9150_I2C_SLV3_REG 			0x2F
#define MPU9150_I2C_SLV3_CTRL 			0x30
#define MPU9150_I2C_SLV4_ADDR 			0x31
#define MPU9150_I2C_SLV4_REG 			0x32
#define MPU9150_I2C_SLV4_DO 			0x33
#define MPU9150_I2C_SLV4_CTRL 			0x34
#define MPU9150_I2C_SLV4_DI 			0x35
#define MPU9150_I2C_MST_STATUS			0x36
#define MPU9150_INT_PIN_CFG 			0x37
#define MPU9150_INT_ENABLE 			0x38
#define MPU9150_INT_STATUS 			0x3A
#define MPU9150_ACCEL_XOUT_H 			0x3B
#define MPU9150_ACCEL_XOUT_L 			0x3C
#define MPU9150_ACCEL_YOUT_H 			0x3D
#define MPU9150_ACCEL_YOUT_L 			0x3E
#define MPU9150_ACCEL_ZOUT_H 			0x3F
#define MPU9150_ACCEL_ZOUT_L 			0x40
#define MPU9150_TEMP_OUT_H 			0x41
#define MPU9150_TEMP_OUT_L 			0x42
#define MPU9150_GYRO_XOUT_H 			0x43
#define MPU9150_GYRO_XOUT_L 			0x44
#define MPU9150_GYRO_YOUT_H 			0x45
#define MPU9150_GYRO_YOUT_L 			0x46
#define MPU9150_GYRO_ZOUT_H 			0x47
#define MPU9150_GYRO_ZOUT_L 			0x48
#define MPU9150_EXT_SENS_DATA_00 		0x49
#define MPU9150_EXT_SENS_DATA_01 		0x4A
#define MPU9150_EXT_SENS_DATA_02 		0x4B
#define MPU9150_EXT_SENS_DATA_03		0x4C
#define MPU9150_EXT_SENS_DATA_04 		0x4D
#define MPU9150_EXT_SENS_DATA_05 		0x4E
#define MPU9150_EXT_SENS_DATA_06 		0x4F
#define MPU9150_EXT_SENS_DATA_07 		0x50
#define MPU9150_EXT_SENS_DATA_08 		0x51
#define MPU9150_EXT_SENS_DATA_09 		0x52
#define MPU9150_EXT_SENS_DATA_10 		0x53
#define MPU9150_EXT_SENS_DATA_11 		0x54
#define MPU9150_EXT_SENS_DATA_12 		0x55
#define MPU9150_EXT_SENS_DATA_13 		0x56
#define MPU9150_EXT_SENS_DATA_14 		0x57
#define MPU9150_EXT_SENS_DATA_15 		0x58
#define MPU9150_EXT_SENS_DATA_16 		0x59
#define MPU9150_EXT_SENS_DATA_17 		0x5A
#define MPU9150_EXT_SENS_DATA_18 		0x5B
#define MPU9150_EXT_SENS_DATA_19 		0x5C
#define MPU9150_EXT_SENS_DATA_20 		0x5D
#define MPU9150_EXT_SENS_DATA_21 		0x5E
#define MPU9150_EXT_SENS_DATA_22 		0x5F
#define MPU9150_EXT_SENS_DATA_23 		0x60
#define MPU9150_MOT_DETECT_STATUS 		0x61
#define MPU9150_I2C_SLV0_DO 			0x63
#define MPU9150_I2C_SLV1_DO			0x64
#define MPU9150_I2C_SLV2_DO 			0x65
#define MPU9150_I2C_SLV3_DO			0x66
#define MPU9150_I2C_MST_DELAY_CTRL 		0x67
#define MPU9150_SIGNAL_PATH_RESET 		0x68
#define MPU9150_MOT_DETECT_CTRL 		0x69
#define MPU9150_USER_CTRL			0x6A
#define MPU9150_PWR_MGMT_1			0x6B
#define MPU9150_PWR_MGMT_2 			0x6C
#define MPU9150_FIFO_COUNT_H			0x72
#define MPU9150_FIFO_COUNT_L			0x73
#define MPU9150_FIFO_R_W			0x74
#define MPU9150_WHO_AM_I			0x75


//Define varaibles for sensor addresses, to control which sensors to use:
#define INV_X_GYRO      (0b10000000)
#define INV_Y_GYRO      (0b01000000)
#define INV_Z_GYRO      (0b00100000)
#define INV_XYZ_GYRO    (INV_X_GYRO | INV_Y_GYRO | INV_Z_GYRO)
#define INV_X_ACCEL     (0b10000000)
#define INV_Y_ACCEL     (0b01000000)
#define INV_Z_ACCEL     (0b00100000)
#define INV_XYZ_ACCEL   (INV_X_ACCEL | INV_Y_ACCEL | INV_Z_ACCEL)
//#define INV_XYZ_COMPASS (0x01)
#define INV_TEMP        (0b10000000)



struct mpu9150_data {
	int16_t acc_x ;
	int16_t acc_y ;
	int16_t acc_z ;
	int16_t temp ;
	int16_t gyro_x ;
	int16_t gyro_y ;
	int16_t gyro_z ;
} ;

void mpu9150_init(void);
void mpu9150_stop(void);
void mpu9150_start(void);
void mpu9150_get_data(uint8_t *buffer);
void mpu9150_get_data_size(uint8_t *buffer, int size);
void mpu9150_decode(struct mpu9150_data *const ba , uint8_t *buffer);
void mpu9150_who_am_i(uint8_t *buffer);
void prepareRead(uint8_t slv_adr);
uint8_t lsr(uint8_t x, uint8_t n);
uint8_t lsl(uint8_t x, uint8_t n);
void resetFIFO();

char *int2bin(uint8_t a, char *buffer, int buf_size);

#endif /* MPU9150_H_ */
