//==============================================================================
//    S E N S I R I O N   AG,  Laubisruetistr. 50, CH-8712 Staefa, Switzerland
//==============================================================================
// Project   :  SHT2x Sample Code (V1.2)
// File      :  SHT2x.c
// Author    :  MST
// Controller:  NEC V850/SG3 (uPD70F3740)
// Compiler  :  IAR compiler for V850 (3.50A)
// Brief     :  Sensor layer. Functions for sensor access
//==============================================================================

/*
 *  Updated on: 26 Aug 2012 by Jagun Kwon (j.kwon@cs.ucl.ac.uk)
 *  Refactored to comply with Contiki sensor interface structure
 */

//---------- Includes ----------------------------------------------------------
#include "SHT2x.h"
#include "utils.h"
#include "i2c.h"
//#include "math.h"
//#include <stdio.h>

#include "lib/sensors.h"

#if 0
//==============================================================================
u8t SHT2x_CheckCrc(u8t data[], u8t nbrOfBytes, u8t checksum)
//==============================================================================
{
  u8t crc = 0;	
  u8t byteCtr, bit;
  //calculates 8-Bit checksum with given polynomial
  for (byteCtr = 0; byteCtr < nbrOfBytes; ++byteCtr)
  { crc ^= (data[byteCtr]);
    for (bit = 8; bit > 0; --bit)
    { if (crc & 0x80) crc = (crc << 1) ^ POLYNOMIAL;
      else crc = (crc << 1);
    }
  }
  if (crc != checksum) return CHECKSUM_ERROR;
  else return 0;
}


//===========================================================================
u8t SHT2x_ReadUserRegister(u8t *pRegisterValue)
//===========================================================================
{
  //u8t checksum;   //variable for checksum byte
  u8t error=0;    //variable for error code
  //I2c_StartCondition();
  //error |= I2c_WriteByte (I2C_ADR_W);
  //error |= I2c_WriteByte (USER_REG_R);
  i2c_transmitinit( I2C_ADR_W, 1, USER_REG_R ) ;
  while(!i2c_transferred()) /* Wait for transfer */ ;

  i2c_receiveinit( I2C_ADR_R, 1, pRegisterValue ) ;
  while(!i2c_transferred()) /* Wait for transfer */ ;

  //I2c_StartCondition();
  //error |= I2c_WriteByte (I2C_ADR_R);
  //*pRegisterValue = I2c_ReadByte(ACK);
  //checksum=I2c_ReadByte(NO_ACK);
  //error |= SHT2x_CheckCrc (pRegisterValue,1,checksum);
  //I2c_StopCondition();
  return error;
}

//===========================================================================
u8t SHT2x_WriteUserRegister(u8t *pRegisterValue)
//===========================================================================
{
  u8t error=0;   //variable for error code

  //I2c_StartCondition();
  //error |= I2c_WriteByte (I2C_ADR_W);
  //error |= I2c_WriteByte (USER_REG_W);
  //error |= I2c_WriteByte (*pRegisterValue);
  //I2c_StopCondition();
  return error;
}


//===========================================================================
u8t SHT2x_MeasureHM(etSHT2xMeasureType eSHT2xMeasureType, nt16 *pMeasurand)
//===========================================================================
{
  u8t  checksum;   //checksum
  u8t  data[2];    //data array for checksum verification
  u8t  error=0;    //error variable
  u16t i;          //counting variable

  //-- write I2C sensor address and command --
  I2c_StartCondition();
  error |= I2c_WriteByte (I2C_ADR_W); // I2C Adr
  switch(eSHT2xMeasureType)
  { case HUMIDITY: error |= I2c_WriteByte (TRIG_RH_MEASUREMENT_HM); break;
    case TEMP    : error |= I2c_WriteByte (TRIG_T_MEASUREMENT_HM);  break;
    default: assert(0);
  }
  //-- wait until hold master is released --
  I2c_StartCondition();
  error |= I2c_WriteByte (I2C_ADR_R);
  SCL=HIGH;                     // set SCL I/O port as input
  for(i=0; i<1000; i++)         // wait until master hold is released or
  {
	  DelayMicroSeconds(1000);    // a timeout (~1s) is reached
    if (SCL_CONF==1) break;
  }
  //-- check for timeout --
  if(SCL_CONF==0) error |= TIME_OUT_ERROR;

  //-- read two data bytes and one checksum byte --
  pMeasurand->s16.u8H = data[0] = I2c_ReadByte(ACK);
  pMeasurand->s16.u8L = data[1] = I2c_ReadByte(ACK);
  checksum=I2c_ReadByte(NO_ACK);

  //-- verify checksum --
  error |= SHT2x_CheckCrc (data,2,checksum);
  I2c_StopCondition();
  return error;
}
#endif

//===========================================================================
u16t SHT2x_GetTemp(void)
//===========================================================================
{
  u8t  checksum;   //checksum
  u8t  data[2];    //data array for checksum verification
  u8t  error=0;    //error variable
  u16t i;          //counting variable
  nt16 tempVal;
  //-- write I2C sensor address and command --
  //I2c_StartCondition();
  //error |= I2c_WriteByte (I2C_ADR_W); // I2C Adr
  //switch(eSHT2xMeasureType)
  //{ case HUMIDITY: error |= I2c_WriteByte (TRIG_RH_MEASUREMENT_HM); break;
  //  case TEMP    : error |= I2c_WriteByte (TRIG_T_MEASUREMENT_HM);  break;
  //  default: assert(0);
 //}
  //-- wait until hold master is released --
  //I2c_StartCondition();
  //error |= I2c_WriteByte (I2C_ADR_R);
  //SCL=HIGH;                     // set SCL I/O port as input
  //for(i=0; i<1000; i++)         // wait until master hold is released or
  //{
  //	  DelayMicroSeconds(1000);    // a timeout (~1s) is reached
  //  if (SCL_CONF==1) break;
  //}
  //-- check for timeout --
  //if(SCL_CONF==0) error |= TIME_OUT_ERROR;

  //-- read two data bytes and one checksum byte --
  //pMeasurand->s16.u8H = data[0] = I2c_ReadByte(ACK);
  //pMeasurand->s16.u8L = data[1] = I2c_ReadByte(ACK);
  //checksum=I2c_ReadByte(NO_ACK);

  //-- verify checksum --
  //error |= SHT2x_CheckCrc (data,2,checksum);
  //I2c_StopCondition();
  /// end original driver

  clock_delay_msec(4);

  uint8_t request = TRIG_T_MEASUREMENT_HM ;

  //uint8_t precv[] = {0,0} ;

  i2c_transmitinit( SHT2x_I2C_ADDR, 1, &request ) ;
  while(!i2c_transferred()) /* Wait for transfer */ ;

  i2c_receiveinit( SHT2x_I2C_ADDR, 2, data ) ;
  while(!i2c_transferred()) /* Wait for transfer */ ;
  tempVal.s16.u8H = data[0];
  tempVal.s16.u8L = data[1];
  //temp_val = ((data[0]>>2) + (data[1]<<6));

  //if ((temp_val & 0x2000) == 0x2000){
  //	temp_val = (((~(temp_val) & 0x3FFF)+1)*-1) ;
  //}

  //ba->acc_x = temp_val;

  return (u16t)  tempVal.i16;
}

//===========================================================================
u16t SHT2x_GetRH(void)
//===========================================================================
{
  u8t  checksum;   //checksum
  u8t  data[2];    //data array for checksum verification
  u8t  error=0;    //error variable
  u16t i;          //counting variable
  nt16 tempVal;


  clock_delay_msec(4);

  uint8_t request = TRIG_RH_MEASUREMENT_HM ;

  //uint8_t precv[] = {0,0} ;

  i2c_transmitinit( SHT2x_I2C_ADDR, 1, &request ) ;
  while(!i2c_transferred()) /* Wait for transfer */ ;

  i2c_receiveinit( SHT2x_I2C_ADDR, 2, data ) ;
  while(!i2c_transferred()) /* Wait for transfer */ ;
  tempVal.s16.u8H = data[0];
  tempVal.s16.u8L = data[1];
  //temp_val = ((data[0]>>2) + (data[1]<<6));

  //if ((temp_val & 0x2000) == 0x2000){
  //	temp_val = (((~(temp_val) & 0x3FFF)+1)*-1) ;
  //}

  //ba->acc_x = temp_val;

  return (u16t)  tempVal.i16;
}


#if 0
//===========================================================================
u8t SHT2x_MeasurePoll(etSHT2xMeasureType eSHT2xMeasureType, nt16 *pMeasurand)
//===========================================================================
{
  u8t  checksum;   //checksum
  u8t  data[2];    //data array for checksum verification
  u8t  error=0;    //error variable
  u16t i=0;        //counting variable

  //-- write I2C sensor address and command --
  I2c_StartCondition();
  error |= I2c_WriteByte (I2C_ADR_W); // I2C Adr
  switch(eSHT2xMeasureType)
  { case HUMIDITY: error |= I2c_WriteByte (TRIG_RH_MEASUREMENT_POLL); break;
    case TEMP    : error |= I2c_WriteByte (TRIG_T_MEASUREMENT_POLL);  break;
    default: assert(0);
  }
  //-- poll every 10ms for measurement ready. Timeout after 20 retries (200ms)--
  do
  { I2c_StartCondition();
    DelayMicroSeconds(10000);  //delay 10ms
    if(i++ >= 20) break;
  } while(I2c_WriteByte (I2C_ADR_R) == ACK_ERROR);
  if (i>=20) error |= TIME_OUT_ERROR;

  //-- read two data bytes and one checksum byte --
  pMeasurand->s16.u8H = data[0] = I2c_ReadByte(ACK);
  pMeasurand->s16.u8L = data[1] = I2c_ReadByte(ACK);
  checksum=I2c_ReadByte(NO_ACK);

  //-- verify checksum --
  error |= SHT2x_CheckCrc (data,2,checksum);
  I2c_StopCondition();

  return error;
}
#endif

//===========================================================================
u8t SHT2x_SoftReset()
//===========================================================================
{
  u8t  error=0;           //error variable

  uint8_t request = SOFT_RESET ;

    //uint8_t precv[] = {0,0} ;
  i2c_transmitinit( SHT2x_I2C_ADDR, 1, &request ) ;
  while(!i2c_transferred()) /* Wait for transfer */ ;

  clock_delay_msec(15); // wait till sensor has restarted

  return error;
}


//==============================================================================
float SHT2x_CalcRH(u16t u16sRH)
//==============================================================================
{
  ft humidityRH;              // variable for result

  u16sRH &= ~0x0003;          // clear bits [1..0] (status bits)
  //-- calculate relative humidity [%RH] --

  humidityRH = -6.0 + 125.0/65536 * (ft)u16sRH; // RH= -6 + 125 * SRH/2^16
  return humidityRH;
}

//==============================================================================
float SHT2x_CalcTemperatureC(u16t u16sT)
//==============================================================================
{
  ft temperatureC;            // variable for result
  int temp;

  u16sT &= ~0x0003;           // clear bits [1..0] (status bits)
  char *pt;
  pt = (char*)&u16sT;
  //-- calculate temperature [�C] --
  temperatureC= -46.85 + 175.72/65536.0 * (ft)u16sT; //T= -46.85 + 175.72 * ST/2^16

  return temperatureC;
}
#if 0
//==============================================================================
u8t SHT2x_GetSerialNumber(u8t u8SerialNumber[])
//==============================================================================
{
  u8t  error=0;                          //error variable

  //Read from memory location 1
  I2c_StartCondition();
  error |= I2c_WriteByte (I2C_ADR_W);    //I2C address
  error |= I2c_WriteByte (0xFA);         //Command for readout on-chip memory
  error |= I2c_WriteByte (0x0F);         //on-chip memory address
  I2c_StartCondition();
  error |= I2c_WriteByte (I2C_ADR_R);    //I2C address
  u8SerialNumber[5] = I2c_ReadByte(ACK); //Read SNB_3
  I2c_ReadByte(ACK);                     //Read CRC SNB_3 (CRC is not analyzed)
  u8SerialNumber[4] = I2c_ReadByte(ACK); //Read SNB_2
  I2c_ReadByte(ACK);                     //Read CRC SNB_2 (CRC is not analyzed)
  u8SerialNumber[3] = I2c_ReadByte(ACK); //Read SNB_1
  I2c_ReadByte(ACK);                     //Read CRC SNB_1 (CRC is not analyzed)
  u8SerialNumber[2] = I2c_ReadByte(ACK); //Read SNB_0
  I2c_ReadByte(NO_ACK);                  //Read CRC SNB_0 (CRC is not analyzed)
  I2c_StopCondition();

  //Read from memory location 2
  I2c_StartCondition();
  error |= I2c_WriteByte (I2C_ADR_W);    //I2C address
  error |= I2c_WriteByte (0xFC);         //Command for readout on-chip memory
  error |= I2c_WriteByte (0xC9);         //on-chip memory address
  I2c_StartCondition();
  error |= I2c_WriteByte (I2C_ADR_R);    //I2C address
  u8SerialNumber[1] = I2c_ReadByte(ACK); //Read SNC_1
  u8SerialNumber[0] = I2c_ReadByte(ACK); //Read SNC_0
  I2c_ReadByte(ACK);                     //Read CRC SNC0/1 (CRC is not analyzed)
  u8SerialNumber[7] = I2c_ReadByte(ACK); //Read SNA_1
  u8SerialNumber[6] = I2c_ReadByte(ACK); //Read SNA_0
  I2c_ReadByte(NO_ACK);                  //Read CRC SNA0/1 (CRC is not analyzed)
  I2c_StopCondition();

  return error;
}
#endif

// Jagun: 26 Aug 2012: added the following to comply with
// Contiki sensor interface structure

//#include "common.h"

static int
value(int type)
{
	u16_t uT, uRH;
	float translatedT, translatedRH;

    switch(type) {
        case SHT2X_TEMP:
		    uT=SHT2x_GetTemp();
		    translatedT = SHT2x_CalcTemperatureC(uT);
		    return (u16_t)(translatedT*10);

		case SHT2X_HUMIDITY:
		    uRH = SHT2x_GetRH();
		    translatedRH = SHT2x_CalcRH(uRH);
		    return (u16_t)(translatedRH*10);
    }
    return 0;	// It must either be TEMP or HUMIDITY
}

static int
status(int type)
{
    // TODO We don't seem to have an equivalent for this
    return 1;
}

static int
configure(int type, int c)
{
	SHT2x_SoftReset();
    return 1;
}

// Instantiate the sensor 
SENSORS_SENSOR(sht2x_sensor, "SHT2x-Sensor", value, configure, status);

