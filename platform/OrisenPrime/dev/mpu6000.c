/*
 * mpu6000.c
 *
 *  Created on: 6 Dec 2011
 *      Author: mcphillips
 */


#include "ssi.h"
#include <stdio.h>
#include "include/mpu6000.h"
#include "include/demo_software_defines.h"


uint8_t mpu6000_spi_read(uint8_t reg)
{
  uint8_t return_value = 0;
  uint8_t addr = reg | 0x80; // Set most significant bit
//  MPU6000_CS_ENABLE;
//  return_value = spi_rw_byte(addr);
//  return_value = spi_rw_byte(0x00);
//  MPU6000_CS_DISABLE;

//	PRINTF("Receive FIFO Counter before read: %x\n",SSI->SFCSRbits.RFCNT);
    ssi_tx_data(addr);
//	PRINTF("Receive FIFO Counter before during: %x\n",SSI->SFCSRbits.RFCNT);
    ssi_tx_data(0x00);
//	PRINTF("Receive FIFO Counter before after: %x\n",SSI->SFCSRbits.RFCNT);
//    while(SSI->SFCSRbits.RFCNT == 0)

//    return_value = ssi_rx_data();

  return(return_value);
}

void mpu6000_spi_write(uint8_t reg, uint8_t data)
{
//	  uint8_t clear_rxbuffer;
//	  MPU6000_CS_ENABLE;
//	return_value = spi_rw_byte(reg);
//	return_value = spi_rw_byte(data);
//	  MPU6000_CS_DISABLE;
//		PRINTF("Receive FIFO Counter before write: %x\n",SSI->SFCSRbits.RFCNT);
	 ssi_tx_data(reg);
	 ssi_tx_data(data);
//	 clear_rxbuffer = ssi_rx_data();
//	 clear_rxbuffer = ssi_rx_data();

}

void mpu_init (void) {


}


void mpu_read(void) {

	uint8_t byte_H;
	uint8_t byte_L;
	//Sensor variables
	int16_t temp;

	int16_t accX;
	int16_t accY;
	int16_t accZ;

	int16_t gyroX;
	int16_t gyroY;
	int16_t gyroZ;

  // Read AccelX
	byte_H = mpu6000_spi_read(MPU6000_ACC_XOUT_H);
	byte_L = mpu6000_spi_read(MPU6000_ACC_XOUT_L);
	accX = (byte_H<<8)| byte_L;
	// Read AccelY
	byte_H = mpu6000_spi_read(MPU6000_ACC_YOUT_H);
	byte_L = mpu6000_spi_read(MPU6000_ACC_YOUT_L);
	accY = (byte_H<<8)| byte_L;
	// Read AccelZ
	byte_H = mpu6000_spi_read(MPU6000_ACC_ZOUT_H);
	byte_L = mpu6000_spi_read(MPU6000_ACC_ZOUT_L);
	accZ = (byte_H<<8)| byte_L;

	// Read Temp
	byte_H = mpu6000_spi_read(MPU6000_TEMP_OUT_H);
	byte_L = mpu6000_spi_read(MPU6000_TEMP_OUT_L);
	temp = (byte_H<<8)| byte_L;

	// Read GyroX
	byte_H = mpu6000_spi_read(MPU6000_GYRO_XOUT_H);
	byte_L = mpu6000_spi_read(MPU6000_GYRO_XOUT_L);
	gyroX = (byte_H<<8)| byte_L;
	// Read GyroY
	byte_H = mpu6000_spi_read(MPU6000_GYRO_YOUT_H);
	byte_L = mpu6000_spi_read(MPU6000_GYRO_YOUT_L);
	gyroY = (byte_H<<8)| byte_L;
	// Read GyroZ
	byte_H = mpu6000_spi_read(MPU6000_GYRO_ZOUT_H);
	byte_L = mpu6000_spi_read(MPU6000_GYRO_ZOUT_L);
	gyroZ = (byte_H<<8)| byte_L;
  
	PRINTF("Temp: %d\t",temp);

	PRINTF("Accels: %d\t,%d\t,%d\t",accX,accY,accZ);
	PRINTF("Gyros: %d\t,%d\t,%d\n",gyroX,gyroY,gyroZ);

}
