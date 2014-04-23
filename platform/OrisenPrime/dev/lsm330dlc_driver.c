/******************** (C) COPYRIGHT 2011 STMicroelectronics ********************
* File Name          : LSM330DLC_driver.c
* Author             : MSH Application Team
* Author             : Fabio Tota
* Version            : $Revision:$
* Date               : $Date:$
* Description        : LSM330DLC driver file
*                      
* HISTORY:
* Date        |	Modification                    |	Author
* 27/01/2012  |	Initial Revision                |	Fabio Tota
* 27/07/2012  |  Modified to support multiple drivers in the same program|Abhishek Anand

* Adapted for the Orisen Prime board by Jagun Kwon at UCL CS (jagun.kwon@ucl.ac.uk)
* Last updated: 22/02/2013

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

/* Includes ------------------------------------------------------------------*/
#include "include/lsm330dlc_driver.h"
#include "lib/sensors.h"    // for Contiki sensor interface
#include "i2c.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/*******************************************************************************
* Function Name		: LSM330DLC_ReadReg
* Description		: Generic Reading function. It must be fullfilled with either
*			: I2C or SPI reading functions					
* Input			: Device Address, Register Address
* Output		: Data REad
* Return		: None
*******************************************************************************/
u8_t LSM330DLC_ReadReg(u8_t deviceAddr, u8_t Reg, u8_t* Data) {

	i2c_transmitinit( deviceAddr, 1, &Reg ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;

	i2c_receiveinit( deviceAddr, 1, Data) ;
 	while(!i2c_transferred()) /* Wait for transfer */ ;

	return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name		: LSM330DLC_WriteReg
* Description		: Generic Writing function. It must be fullfilled with either
*			: I2C or SPI writing function
* Input			: Device Address, Register Address, Data to be written
* Output		: None
* Return		: None
*******************************************************************************/
u8_t LSM330DLC_WriteReg(u8_t deviceAddr, u8_t WriteAddr, u8_t Data) {

	i2c_transmitinit( deviceAddr, 1, &WriteAddr ) ;	// Register to write
	while(!i2c_transferred()) /* Wait for transfer */ ;

	i2c_transmitinit( deviceAddr, 1, &Data ) ;		// Data to write
	while(!i2c_transferred()) /* Wait for transfer */ ;

  return MEMS_SUCCESS;
}

//static int lsm330dlc_acc_i2c_read(struct lsm330dlc_acc_status *stat, u8 *buf,
static int lsm330dlc_acc_i2c_read(u8_t deviceAddr, u8_t *buf, int len) {
    int ret;
    u8_t reg = buf[0];
    u8_t cmd = reg;

    if (len > 1) {
		printf("DEBUG: Len > 1 in acc_read\n");
        //cmd = (I2C_AUTO_INCREMENT | reg);		// TODO Commented out for now
	}

	i2c_transmitinit( deviceAddr, sizeof(cmd), &cmd ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;
	printf("DEBUG: ret = %d\n", ret);
    //ret = i2c_master_send(deviceAddr, &cmd, sizeof(cmd));
    //if (ret != sizeof(cmd))
    //    return ret;

	i2c_receiveinit( deviceAddr, len, buf) ;
  	while(!i2c_transferred()) /* Wait for transfer */ ;
	return 0;
    //return i2c_master_recv(deviceAddr, buf, len);
}

static int lsm330dlc_acc_i2c_write(u8_t deviceAddr, u8_t *buf, int len) {
    int ret;
    u8_t reg, value;

    if (len > 1) {
		printf("DEBUG: Len > 1 in acc_write\n");
        //buf[0] = (I2C_AUTO_INCREMENT | buf[0]);
	}

    reg = buf[0];
    value = buf[1];

	i2c_transmitinit( deviceAddr, len+1, buf ) ;
	while(!i2c_transferred()) /* Wait for transfer */ ;
    //ret = i2c_master_send(stat->client, buf, len+1);
    //return (ret == len+1) ? 0 : ret;
	return 0;	// TODO 0 or success for now.
}



/* Private functions ---------------------------------------------------------*/


/*******************************************************************************
* Function Name  : LSM330DLC_SetODR_A
* Description    : Sets LSM330DLC Acc Output Data Rate
* Input          : Output Data Rate
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330DLC_SetODR_A(LSM330DLC_ODR_A_t ov){
  u8_t value;

  if( !LSM330DLC_ReadReg(LSM330DLC_MEMS_I2C_ADDRESS_A,LSM330DLC_CTRL_REG1_A, &value) )
    return MEMS_ERROR;

  value &= 0x0f;
  value |= ov<<LSM330DLC_ODR_BIT_A;

  if( !LSM330DLC_WriteReg(LSM330DLC_MEMS_I2C_ADDRESS_A,LSM330DLC_CTRL_REG1_A, value) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LSM330DLC_SetMode_A
* Description    : Sets LSM330DLC Operating Mode Acc
* Input          : Modality (NORMAL, LOW_POWER, POWER_DOWN)
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330DLC_SetMode_A(LSM330DLC_Mode_A_t md) {
  u8_t value;
  u8_t value2;
  static   u8_t ODR_old_value;
 
  if( !LSM330DLC_ReadReg(LSM330DLC_MEMS_I2C_ADDRESS_A,LSM330DLC_CTRL_REG1_A, &value) )
    return MEMS_ERROR;
  
  if( !LSM330DLC_ReadReg(LSM330DLC_MEMS_I2C_ADDRESS_A,LSM330DLC_CTRL_REG4_A, &value2) )
    return MEMS_ERROR;
  
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
  
  if( !LSM330DLC_WriteReg(LSM330DLC_MEMS_I2C_ADDRESS_A,LSM330DLC_CTRL_REG1_A, value) )
    return MEMS_ERROR;
  
  if( !LSM330DLC_WriteReg(LSM330DLC_MEMS_I2C_ADDRESS_A,LSM330DLC_CTRL_REG4_A, value2) )
    return MEMS_ERROR;  
   
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LSM330DLC_SetAxis_A
* Description    : Enable/Disable LSM330DLC Acc Axis
* Input          : LSM330DLC_X_ENABLE/LSM330DLC_X_DISABLE | LSM330DLC_Y_ENABLE/LSM330DLC_Y_DISABLE | LSM330DLC_Z_ENABLE/LSM330DLC_Z_DISABLE
* Output         : None
* Note           : You MUST use all input variable in the argument, as example
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330DLC_SetAxis_A(LSM330DLC_AXISenable_A_t axis) {
  u8_t value;
  
  if( !LSM330DLC_ReadReg(LSM330DLC_MEMS_I2C_ADDRESS_A,LSM330DLC_CTRL_REG1_A, &value) )
    return MEMS_ERROR;
  value &= 0xF8;
  value |= (0x07 & axis);
   
  if( !LSM330DLC_WriteReg(LSM330DLC_MEMS_I2C_ADDRESS_A,LSM330DLC_CTRL_REG1_A, value) )
    return MEMS_ERROR;   
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LSM330DLC_SetFullScale_A
* Description    : Sets the LSM330DLC Acc FullScale
* Input          : LSM330DLC_FULLSCALE_2/LSM330DLC_FULLSCALE_4/LSM330DLC_FULLSCALE_8/LSM330DLC_FULLSCALE_16
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330DLC_SetFullScale_A(LSM330DLC_Fullscale_A_t fs) {
  u8_t value;
  
  if( !LSM330DLC_ReadReg(LSM330DLC_MEMS_I2C_ADDRESS_A,LSM330DLC_CTRL_REG4_A, &value) )
    return MEMS_ERROR;
                  
  value &= 0xCF;	
  value |= (fs<<LSM330DLC_FS_A);
  
  if( !LSM330DLC_WriteReg(LSM330DLC_MEMS_I2C_ADDRESS_A,LSM330DLC_CTRL_REG4_A, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LSM330DLC_SetBLE_A
* Description    : Set Endianess (MSB/LSB) Acc
* Input          : LSM330DLC_BLE_LSB / LSM330DLC_BLE_MSB
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330DLC_SetBLE_A(LSM330DLC_Endianess_t ble) {
  u8_t value;
  
  if( !LSM330DLC_ReadReg(LSM330DLC_MEMS_I2C_ADDRESS_A,LSM330DLC_CTRL_REG4_A, &value) )
    return MEMS_ERROR;
                  
  value &= 0xBF;	
  value |= (ble<<LSM330DLC_BLE_A);
  
  if( !LSM330DLC_WriteReg(LSM330DLC_MEMS_I2C_ADDRESS_A,LSM330DLC_CTRL_REG4_A, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LSM330DLC_HPFClick_A
* Description    : Enable/Disable High Pass Filter for click Acc
* Input          : MEMS_ENABLE/MEMS_DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330DLC_HPFClickEnable_A(State_t hpfe) {
  u8_t value;
  
  if( !LSM330DLC_ReadReg(LSM330DLC_MEMS_I2C_ADDRESS_A,LSM330DLC_CTRL_REG2_A, &value) )
    return MEMS_ERROR;
                  
  value &= 0xFB;
  value |= (hpfe<<LSM330DLC_HPCLICK_A);
  
  if( !LSM330DLC_WriteReg(LSM330DLC_MEMS_I2C_ADDRESS_A,LSM330DLC_CTRL_REG2_A, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LSM330DLC_HPFAOI1_A
* Description    : Enable/Disable High Pass Filter for AOI on INT_1 Acc
* Input          : MEMS_ENABLE/MEMS_DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330DLC_HPFAOI1Enable_A(State_t hpfe) {
  u8_t value;
  
  if( !LSM330DLC_ReadReg(LSM330DLC_MEMS_I2C_ADDRESS_A, LSM330DLC_CTRL_REG2_A, &value) )
    return MEMS_ERROR;
                  
  value &= 0xFE;
  value |= (hpfe<<LSM330DLC_HPIS1_A);
  
  if( !LSM330DLC_WriteReg(LSM330DLC_MEMS_I2C_ADDRESS_A, LSM330DLC_CTRL_REG2_A, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LSM330DLC_HPFAOI2_A
* Description    : Enable/Disable High Pass Filter for AOI on INT_2  Acc
* Input          : MEMS_ENABLE/MEMS_DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330DLC_HPFAOI2Enable_A(State_t hpfe) {
  u8_t value;
  
  if( !LSM330DLC_ReadReg(LSM330DLC_MEMS_I2C_ADDRESS_A, LSM330DLC_CTRL_REG2_A, &value) )
    return MEMS_ERROR;
                  
  value &= 0xFD;
  value |= (hpfe<<LSM330DLC_HPIS2_A);
  
  if( !LSM330DLC_WriteReg(LSM330DLC_MEMS_I2C_ADDRESS_A, LSM330DLC_CTRL_REG2_A, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LSM330DLC_SetHPFMode_A
* Description    : Set High Pass Filter Modality Acc
* Input          : LSM330DLC_HPM_NORMAL_MODE_RES/LSM330DLC_HPM_REF_SIGNAL/LSM330DLC_HPM_NORMAL_MODE/LSM330DLC_HPM_AUTORESET_INT
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330DLC_SetHPFMode_A(LSM330DLC_HPFMode_t hpm) {
  u8_t value;
  
  if( !LSM330DLC_ReadReg(LSM330DLC_MEMS_I2C_ADDRESS_A, LSM330DLC_CTRL_REG2_A, &value) )
    return MEMS_ERROR;
                  
  value &= 0x3F;
  value |= (hpm<<LSM330DLC_HPM_A);
  
  if( !LSM330DLC_WriteReg(LSM330DLC_MEMS_I2C_ADDRESS_A, LSM330DLC_CTRL_REG2_A, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LSM330DLC_SetHPFCutOFF_A
* Description    : Set High Pass CUT OFF Freq Acc
* Input          : HPFCF [0,3]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330DLC_SetHPFCutOFF_A(LSM330DLC_HPFCutOffFreq_t hpf) {
  u8_t value;
    
  if (hpf > 3)
    return MEMS_ERROR;
  
  if( !LSM330DLC_ReadReg(LSM330DLC_MEMS_I2C_ADDRESS_A, LSM330DLC_CTRL_REG2_A, &value) )
    return MEMS_ERROR;
                  
  value &= 0xCF;
  value |= (hpf<<LSM330DLC_HPCF_A);
  
  if( !LSM330DLC_WriteReg(LSM330DLC_MEMS_I2C_ADDRESS_A, LSM330DLC_CTRL_REG2_A, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
  
}


/*******************************************************************************
* Function Name  : LSM330DLC_SetFilterDataSel_A
* Description    : Set Filter Data Selection bypassed or sent to FIFO OUT register Acc
* Input          : MEMS_SET, MEMS_RESET
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330DLC_SetFilterDataSel_A(State_t state) {
  u8_t value;
  
  if( !LSM330DLC_ReadReg(LSM330DLC_MEMS_I2C_ADDRESS_A, LSM330DLC_CTRL_REG2_A, &value) )
    return MEMS_ERROR;
                  
  value &= 0xF7;
  value |= (state<<LSM330DLC_FDS_A);
  
  if( !LSM330DLC_WriteReg(LSM330DLC_MEMS_I2C_ADDRESS_A, LSM330DLC_CTRL_REG2_A, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
  
}


/*******************************************************************************
* Function Name  : LSM330DLC_SetInt1Pin_A
* Description    : Set Interrupt1 pin Function Acc
* Input          :  LSM330DLC_CLICK_ON_PIN_INT1_ENABLE/DISABLE    | LSM330DLC_I1_INT1_ON_PIN_INT1_ENABLE/DISABLE |              
                    LSM330DLC_I1_INT2_ON_PIN_INT1_ENABLE/DISABLE  | LSM330DLC_I1_DRDY1_ON_INT1_ENABLE/DISABLE    |              
                    LSM330DLC_I1_DRDY2_ON_INT1_ENABLE/DISABLE     | LSM330DLC_WTM_ON_INT1_ENABLE/DISABLE         |           
                    LSM330DLC_INT1_OVERRUN_ENABLE/DISABLE  
* example        : LSM330DLC_SetInt1Pin_A(LSM330DLC_CLICK_ON_PIN_INT1_ENABLE | LSM330DLC_I1_INT1_ON_PIN_INT1_ENABLE |              
                    LSM330DLC_I1_INT2_ON_PIN_INT1_DISABLE | LSM330DLC_I1_DRDY1_ON_INT1_ENABLE | LSM330DLC_I1_DRDY2_ON_INT1_ENABLE     |
                    LSM330DLC_WTM_ON_INT1_DISABLE | LSM330DLC_INT1_OVERRUN_DISABLE   ) 
* Note           : To enable Interrupt signals on INT1 Pad (You MUST use all input variable in the argument, as example)
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330DLC_SetInt1Pin_A(LSM330DLC_IntPinConf_t pinConf) {
  u8_t value;
  
  if( !LSM330DLC_ReadReg(LSM330DLC_MEMS_I2C_ADDRESS_A, LSM330DLC_CTRL_REG3_A, &value) )
    return MEMS_ERROR;
                  
  value &= 0x00;
  value |= pinConf;
  
  if( !LSM330DLC_WriteReg(LSM330DLC_MEMS_I2C_ADDRESS_A, LSM330DLC_CTRL_REG3_A, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LSM330DLC_SetInt2Pin_A
* Description    : Set Interrupt2 pin Function Acc
* Input          : LSM330DLC_CLICK_ON_PIN_INT2_ENABLE/DISABLE   | LSM330DLC_I2_INT1_ON_PIN_INT2_ENABLE/DISABLE |               
                   LSM330DLC_I2_INT2_ON_PIN_INT2_ENABLE/DISABLE | LSM330DLC_I2_BOOT_ON_INT2_ENABLE/DISABLE |                   
                   LSM330DLC_INT_ACTIVE_HIGH/LOW
* example        : LSM330DLC_SetInt2Pin(LSM330DLC_CLICK_ON_PIN_INT2_ENABLE/DISABLE | LSM330DLC_I2_INT1_ON_PIN_INT2_ENABLE/DISABLE |               
                   LSM330DLC_I2_INT2_ON_PIN_INT2_ENABLE/DISABLE | LSM330DLC_I2_BOOT_ON_INT2_ENABLE/DISABLE |                   
                   LSM330DLC_INT_ACTIVE_HIGH/LOW)
* Note           : To enable Interrupt signals on INT2 Pad (You MUST use all input variable in the argument, as example)
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330DLC_SetInt2Pin_A(LSM330DLC_IntPinConf_t pinConf) {
  u8_t value;
  
  if( !LSM330DLC_ReadReg(LSM330DLC_MEMS_I2C_ADDRESS_A, LSM330DLC_CTRL_REG6_A, &value) )
    return MEMS_ERROR;
                  
  value &= 0x00;
  value |= pinConf;
  
  if( !LSM330DLC_WriteReg(LSM330DLC_MEMS_I2C_ADDRESS_A, LSM330DLC_CTRL_REG6_A, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}                       


/*******************************************************************************
* Function Name  : LSM330DLC_SetClickCFG_A
* Description    : Set Click Interrupt config Function  Acc
* Input          : LSM330DLC_ZD_ENABLE/DISABLE | LSM330DLC_ZS_ENABLE/DISABLE  | LSM330DLC_YD_ENABLE/DISABLE  | 
                   LSM330DLC_YS_ENABLE/DISABLE | LSM330DLC_XD_ENABLE/DISABLE  | LSM330DLC_XS_ENABLE/DISABLE 
* example        : LSM330DLC_SetClickCFG( LSM330DLC_ZD_ENABLE | LSM330DLC_ZS_DISABLE | LSM330DLC_YD_ENABLE | 
                               LSM330DLC_YS_DISABLE | LSM330DLC_XD_ENABLE | LSM330DLC_XS_ENABLE)
* Note           : You MUST use all input variable in the argument, as example
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330DLC_SetClickCFG_A(u8_t status) {
  u8_t value;
  
  if( !LSM330DLC_ReadReg(LSM330DLC_MEMS_I2C_ADDRESS_A, LSM330DLC_CLICK_CFG_A, &value) )
    return MEMS_ERROR;
                  
  value &= 0xC0;
  value |= status;
  
  if( !LSM330DLC_WriteReg(LSM330DLC_MEMS_I2C_ADDRESS_A, LSM330DLC_CLICK_CFG_A, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}  


/*******************************************************************************
* Function Name  : LSM330DLC_SetClickTHS_A
* Description    : Set Click Interrupt threshold Acc
* Input          : Click-click Threshold value [0-127]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330DLC_SetClickTHS_A(u8_t ths) {
  
  if(ths>127)     return MEMS_ERROR;
  
  if( !LSM330DLC_WriteReg(LSM330DLC_MEMS_I2C_ADDRESS_A, LSM330DLC_CLICK_THS_A, ths) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
} 


/*******************************************************************************
* Function Name  : LSM330DLC_SetClickLIMIT_A
* Description    : Set Click Interrupt Time Limit Acc
* Input          : Click-click Time Limit value [0-127]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330DLC_SetClickLIMIT_A(u8_t t_limit) {
  
  if(t_limit>127)     return MEMS_ERROR;
  
  if( !LSM330DLC_WriteReg(LSM330DLC_MEMS_I2C_ADDRESS_A, LSM330DLC_TIME_LIMIT_A, t_limit) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
} 


/*******************************************************************************
* Function Name  : LSM330DLC_SetClickLATENCY_A
* Description    : Set Click Interrupt Time Latency Acc
* Input          : Click-click Time Latency value [0-255]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330DLC_SetClickLATENCY_A(u8_t t_latency) {
  
  if(t_latency>255)     return MEMS_ERROR;
  
  if( !LSM330DLC_WriteReg(LSM330DLC_MEMS_I2C_ADDRESS_A, LSM330DLC_TIME_LATENCY_A, t_latency) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
} 


/*******************************************************************************
* Function Name  : LSM330DLC_SetClickWINDOW_A
* Description    : Set Click Interrupt Time Window Acc
* Input          : Click-click Time Window value [0-255]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330DLC_SetClickWINDOW_A(u8_t t_window) {
  
  if(t_window>255)     return MEMS_ERROR;
  
  if( !LSM330DLC_WriteReg(LSM330DLC_MEMS_I2C_ADDRESS_A, LSM330DLC_TIME_WINDOW_A, t_window) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LSM330DLC_GetClickResponce_A
* Description    : Get Click Interrupt Responce by CLICK_SRC REGISTER Acc
* Input          : char to empty by Click Responce Typedef
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330DLC_GetClickResponce_A(u8_t* res) {
  u8_t value;
  
 if( !LSM330DLC_ReadReg(LSM330DLC_MEMS_I2C_ADDRESS_A, LSM330DLC_CLICK_SRC_A, &value) ) 
   return MEMS_ERROR;

 value &= 0x7F;
 
 if((value & LSM330DLC_IA)==0) {        *res = LSM330DLC_NO_CLICK;     return MEMS_SUCCESS;}
 else {
   if (value & LSM330DLC_DCLICK){
     if (value & LSM330DLC_CLICK_SIGN){
        if (value & LSM330DLC_CLICK_Z) {*res = LSM330DLC_DCLICK_Z_N;   return MEMS_SUCCESS;}
        if (value & LSM330DLC_CLICK_Y) {*res = LSM330DLC_DCLICK_Y_N;   return MEMS_SUCCESS;}
        if (value & LSM330DLC_CLICK_X) {*res = LSM330DLC_DCLICK_X_N;   return MEMS_SUCCESS;}
     }
     else{
        if (value & LSM330DLC_CLICK_Z) {*res = LSM330DLC_DCLICK_Z_P;   return MEMS_SUCCESS;}
        if (value & LSM330DLC_CLICK_Y) {*res = LSM330DLC_DCLICK_Y_P;   return MEMS_SUCCESS;}
        if (value & LSM330DLC_CLICK_X) {*res = LSM330DLC_DCLICK_X_P;   return MEMS_SUCCESS;}
     }       
   }
   else{
     if (value & LSM330DLC_CLICK_SIGN){
        if (value & LSM330DLC_CLICK_Z) {*res = LSM330DLC_SCLICK_Z_N;   return MEMS_SUCCESS;}
        if (value & LSM330DLC_CLICK_Y) {*res = LSM330DLC_SCLICK_Y_N;   return MEMS_SUCCESS;}
        if (value & LSM330DLC_CLICK_X) {*res = LSM330DLC_SCLICK_X_N;   return MEMS_SUCCESS;}
     }
     else{
        if (value & LSM330DLC_CLICK_Z) {*res = LSM330DLC_SCLICK_Z_P;   return MEMS_SUCCESS;}
        if (value & LSM330DLC_CLICK_Y) {*res = LSM330DLC_SCLICK_Y_P;   return MEMS_SUCCESS;}
        if (value & LSM330DLC_CLICK_X) {*res = LSM330DLC_SCLICK_X_P;   return MEMS_SUCCESS;}
     }
   }
 }
 return MEMS_ERROR;
} 


/*******************************************************************************
* Function Name  : LSM330DLC_Int1LatchEnable_A
* Description    : Enable Interrupt 1 Latching function Acc
* Input          : ENABLE/DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330DLC_Int1LatchEnable_A(State_t latch) {
  u8_t value;
  
  if( !LSM330DLC_ReadReg(LSM330DLC_MEMS_I2C_ADDRESS_A, LSM330DLC_CTRL_REG5_A, &value) )
    return MEMS_ERROR;
                  
  value &= 0xF7;
  value |= latch<<LSM330DLC_LIR_INT1_A;
  
  if( !LSM330DLC_WriteReg(LSM330DLC_MEMS_I2C_ADDRESS_A, LSM330DLC_CTRL_REG5_A, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LSM330DLC_ResetInt1Latch_A
* Description    : Reset Interrupt 1 Latching function Acc
* Input          : None
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330DLC_ResetInt1Latch_A(void) {
  u8_t value;
  
  if( !LSM330DLC_ReadReg(LSM330DLC_MEMS_I2C_ADDRESS_A, LSM330DLC_INT1_SRC_A, &value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

      
/*******************************************************************************
* Function Name  : LSM330DLC_SetIntConfiguration_A
* Description    : Interrupt 1 Configuration (whitout 6D_INT) Acc
* Input          : LSM330DLC_INT1_AND/OR | LSM330DLC_INT1_ZHIE_ENABLE/DISABLE | LSM330DLC_INT1_ZLIE_ENABLE/DISABLE...
* Output         : None
* Note           : You MUST use all input variable in the argument, as example
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330DLC_SetIntConfiguration_A(LSM330DLC_Int1Conf_t ic) {
  u8_t value;
  
  if( !LSM330DLC_ReadReg(LSM330DLC_MEMS_I2C_ADDRESS_A, LSM330DLC_INT1_CFG_A, &value) )
    return MEMS_ERROR;
  
  value &= 0x40; 
  value |= ic;
  
  if( !LSM330DLC_WriteReg(LSM330DLC_MEMS_I2C_ADDRESS_A, LSM330DLC_INT1_CFG_A, value) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}
      
      
/*******************************************************************************
* Function Name  : LSM330DLC_SetIntMode_A
* Description    : Interrupt 1 Configuration mode (OR, 6D Movement, AND, 6D Position) Acc
* Input          : LSM330DLC_INT_MODE_OR, LSM330DLC_INT_MODE_6D_MOVEMENT, LSM330DLC_INT_MODE_AND, LSM330DLC_INT_MODE_6D_POSITION
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330DLC_SetIntMode_A(LSM330DLC_Int1Mode_A_t int_mode) {
  u8_t value;
  
  if( !LSM330DLC_ReadReg(LSM330DLC_MEMS_I2C_ADDRESS_A, LSM330DLC_INT1_CFG_A, &value) )
    return MEMS_ERROR;
  
  value &= 0x3F; 
  value |= (int_mode<<LSM330DLC_INT_6D_A);
  
  if( !LSM330DLC_WriteReg(LSM330DLC_MEMS_I2C_ADDRESS_A, LSM330DLC_INT1_CFG_A, value) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LSM330DLC_SetInt6D4DConfiguration_A
* Description    : 6D, 4D Interrupt Configuration Acc
* Input          : LSM330DLC_INT1_6D_ENABLE_A, LSM330DLC_INT1_4D_ENABLE_A, LSM330DLC_INT1_6D_4D_DISABLE_A
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330DLC_SetInt6D4DConfiguration_A(LSM330DLC_INT_6D_4D_A_t ic) {
  u8_t value;
  u8_t value2;
  
  if( !LSM330DLC_ReadReg(LSM330DLC_MEMS_I2C_ADDRESS_A, LSM330DLC_INT1_CFG_A, &value) )
    return MEMS_ERROR;
  if( !LSM330DLC_ReadReg(LSM330DLC_MEMS_I2C_ADDRESS_A, LSM330DLC_CTRL_REG5_A, &value2) )
    return MEMS_ERROR;
  
  if(ic == LSM330DLC_INT1_6D_ENABLE_A){
      value &= 0xBF; 
      value |= (MEMS_ENABLE<<LSM330DLC_INT_6D_A);
      value2 &= 0xFB; 
      value2 |= (MEMS_DISABLE<<LSM330DLC_D4D_INT1_A);
  }
  
    if(ic == LSM330DLC_INT1_4D_ENABLE_A){
      value &= 0xBF; 
      value |= (MEMS_ENABLE<<LSM330DLC_INT_6D_A);
      value2 &= 0xFB; 
      value2 |= (MEMS_ENABLE<<LSM330DLC_D4D_INT1_A);
  }
  
    if(ic == LSM330DLC_INT1_6D_4D_DISABLE_A){
      value &= 0xBF; 
      value |= (MEMS_DISABLE<<LSM330DLC_INT_6D_A);
      value2 &= 0xFB; 
      value2 |= (MEMS_DISABLE<<LSM330DLC_D4D_INT1_A);
  }
  
  if( !LSM330DLC_WriteReg(LSM330DLC_MEMS_I2C_ADDRESS_A, LSM330DLC_INT1_CFG_A, value) )
    return MEMS_ERROR;
  if( !LSM330DLC_WriteReg(LSM330DLC_MEMS_I2C_ADDRESS_A, LSM330DLC_CTRL_REG5_A, value2) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LSM330DLC_Get6DPosition_A
* Description    : 6D, 4D Interrupt Position Detect Acc
* Input          : Byte to empty by POSITION_6D_t Typedef
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330DLC_Get6DPosition_A(u8_t* val){
  u8_t value;
  
  if( !LSM330DLC_ReadReg(LSM330DLC_MEMS_I2C_ADDRESS_A, LSM330DLC_INT1_SRC_A, &value) )
    return MEMS_ERROR;

  value &= 0x7F;
  
  switch (value){
  case LSM330DLC_UP_SX_A:   *val = LSM330DLC_UP_SX_A;    break;
  case LSM330DLC_UP_DX_A:   *val = LSM330DLC_UP_DX_A;    break;
  case LSM330DLC_DW_SX_A:   *val = LSM330DLC_DW_SX_A;    break;
  case LSM330DLC_DW_DX_A:   *val = LSM330DLC_DW_DX_A;    break;
  case LSM330DLC_TOP_A:     *val = LSM330DLC_TOP_A;      break;
  case LSM330DLC_BOTTOM_A:  *val = LSM330DLC_BOTTOM_A;   break;
  }
  
return MEMS_SUCCESS;  
}


/*******************************************************************************
* Function Name  : LSM330DLC_SetInt1Threshold_A
* Description    : Sets Interrupt 1 Threshold Acc
* Input          : Threshold = [0,31]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330DLC_SetInt1Threshold_A(u8_t ths) {
  if (ths > 127)
    return MEMS_ERROR;
  
      if( !LSM330DLC_WriteReg(LSM330DLC_MEMS_I2C_ADDRESS_A, LSM330DLC_INT1_THS_A, ths) )
        return MEMS_ERROR;    

  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LSM330DLC_SetInt1Duration_A
* Description    : Sets Interrupt 1 Duration Acc
* Input          : Duration value
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330DLC_SetInt1Duration_A(LSM330DLC_Int1Conf_t id) {
 
  if (id > 127)
    return MEMS_ERROR;

  if( !LSM330DLC_WriteReg(LSM330DLC_MEMS_I2C_ADDRESS_A, LSM330DLC_INT1_DURATION_A, id) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

      
/*******************************************************************************
* Function Name  : LSM330DLC_FIFOModeEnable_A
* Description    : Sets Fifo Modality Acc
* Input          : LSM330DLC_FIFO_DISABLE, LSM330DLC_FIFO_BYPASS_MODE, LSM330DLC_FIFO_MODE, LSM330DLC_FIFO_STREAM_MODE, LSM330DLC_FIFO_TRIGGER_MODE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330DLC_FIFOModeEnable_A(LSM330DLC_FifoMode_A_t fm) {
  u8_t value;  
  
  if(fm == LSM330DLC_FIFO_DISABLE_A) { 
     if( !LSM330DLC_ReadReg(LSM330DLC_MEMS_I2C_ADDRESS_A, LSM330DLC_FIFO_CTRL_REG_A, &value) )
      return MEMS_ERROR;
    
    value &= 0x1f;
    value |= (LSM330DLC_FIFO_BYPASS_MODE_A<<LSM330DLC_FM_A);                     
    
    if( !LSM330DLC_WriteReg(LSM330DLC_MEMS_I2C_ADDRESS_A, LSM330DLC_FIFO_CTRL_REG_A, value) )           //fifo mode bypass
      return MEMS_ERROR;   
    if( !LSM330DLC_ReadReg(LSM330DLC_MEMS_I2C_ADDRESS_A, LSM330DLC_CTRL_REG5_A, &value) )
      return MEMS_ERROR;
                    
    value &= 0xBF;    
    
    if( !LSM330DLC_WriteReg(LSM330DLC_MEMS_I2C_ADDRESS_A, LSM330DLC_CTRL_REG5_A, value) )               //fifo disable
      return MEMS_ERROR;   
  }
  
  if(fm == LSM330DLC_FIFO_BYPASS_MODE_A)   {  
    if( !LSM330DLC_ReadReg(LSM330DLC_MEMS_I2C_ADDRESS_A, LSM330DLC_CTRL_REG5_A, &value) )
      return MEMS_ERROR;
                    
    value &= 0xBF;
    value |= MEMS_SET<<LSM330DLC_FIFO_EN_A;
    
    if( !LSM330DLC_WriteReg(LSM330DLC_MEMS_I2C_ADDRESS_A, LSM330DLC_CTRL_REG5_A, value) )               //fifo enable
      return MEMS_ERROR;  
    if( !LSM330DLC_ReadReg(LSM330DLC_MEMS_I2C_ADDRESS_A, LSM330DLC_FIFO_CTRL_REG_A, &value) )
      return MEMS_ERROR;
    
    value &= 0x1f;
    value |= (fm<<LSM330DLC_FM_A);                     //fifo mode configuration
    
    if( !LSM330DLC_WriteReg(LSM330DLC_MEMS_I2C_ADDRESS_A, LSM330DLC_FIFO_CTRL_REG_A, value) )
      return MEMS_ERROR;
  }
  
  if(fm == LSM330DLC_FIFO_MODE_A)   {
    if( !LSM330DLC_ReadReg(LSM330DLC_MEMS_I2C_ADDRESS_A, LSM330DLC_CTRL_REG5_A, &value) )
      return MEMS_ERROR;
                    
    value &= 0xBF;
    value |= MEMS_SET<<LSM330DLC_FIFO_EN_A;
    
    if( !LSM330DLC_WriteReg(LSM330DLC_MEMS_I2C_ADDRESS_A, LSM330DLC_CTRL_REG5_A, value) )               //fifo enable
      return MEMS_ERROR;  
    if( !LSM330DLC_ReadReg(LSM330DLC_MEMS_I2C_ADDRESS_A, LSM330DLC_FIFO_CTRL_REG_A, &value) )
      return MEMS_ERROR;
    
    value &= 0x1f;
    value |= (fm<<LSM330DLC_FM_A);                      //fifo mode configuration
    
    if( !LSM330DLC_WriteReg(LSM330DLC_MEMS_I2C_ADDRESS_A, LSM330DLC_FIFO_CTRL_REG_A, value) )
      return MEMS_ERROR;
  }
  
  if(fm == LSM330DLC_FIFO_STREAM_MODE_A)   {  
    if( !LSM330DLC_ReadReg(LSM330DLC_MEMS_I2C_ADDRESS_A, LSM330DLC_CTRL_REG5_A, &value) )
      return MEMS_ERROR;
                    
    value &= 0xBF;
    value |= MEMS_SET<<LSM330DLC_FIFO_EN_A;
    
    if( !LSM330DLC_WriteReg(LSM330DLC_MEMS_I2C_ADDRESS_A, LSM330DLC_CTRL_REG5_A, value) )               //fifo enable
      return MEMS_ERROR;   
    if( !LSM330DLC_ReadReg(LSM330DLC_MEMS_I2C_ADDRESS_A, LSM330DLC_FIFO_CTRL_REG_A, &value) )
      return MEMS_ERROR;
    
    value &= 0x1f;
    value |= (fm<<LSM330DLC_FM_A);                      //fifo mode configuration
    
    if( !LSM330DLC_WriteReg(LSM330DLC_MEMS_I2C_ADDRESS_A, LSM330DLC_FIFO_CTRL_REG_A, value) )
      return MEMS_ERROR;
  }
  
  if(fm == LSM330DLC_FIFO_TRIGGER_MODE_A)   {  
    if( !LSM330DLC_ReadReg(LSM330DLC_MEMS_I2C_ADDRESS_A, LSM330DLC_CTRL_REG5_A, &value) )
      return MEMS_ERROR;
                    
    value &= 0xBF;
    value |= MEMS_SET<<LSM330DLC_FIFO_EN_A;
    
    if( !LSM330DLC_WriteReg(LSM330DLC_MEMS_I2C_ADDRESS_A, LSM330DLC_CTRL_REG5_A, value) )               //fifo enable
      return MEMS_ERROR;    
    if( !LSM330DLC_ReadReg(LSM330DLC_MEMS_I2C_ADDRESS_A, LSM330DLC_FIFO_CTRL_REG_A, &value) )
      return MEMS_ERROR;
    
    value &= 0x1f;
    value |= (fm<<LSM330DLC_FM_A);                      //fifo mode configuration
    
    if( !LSM330DLC_WriteReg(LSM330DLC_MEMS_I2C_ADDRESS_A, LSM330DLC_FIFO_CTRL_REG_A, value) )
      return MEMS_ERROR;
  }
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LSM330DLC_SetTriggerInt_A
* Description    : Trigger event liked to trigger signal INT1/INT2 Acc
* Input          : LSM330DLC_TRIG_INT1/LSM330DLC_TRIG_INT2
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330DLC_SetTriggerInt_A(LSM330DLC_TrigInt_A_t tr) {
  u8_t value;  
  
  if( !LSM330DLC_ReadReg(LSM330DLC_MEMS_I2C_ADDRESS_A, LSM330DLC_FIFO_CTRL_REG_A, &value) )
      return MEMS_ERROR;
    
    value &= 0xDF;
    value |= (tr<<LSM330DLC_TR_A); 

  if( !LSM330DLC_WriteReg(LSM330DLC_MEMS_I2C_ADDRESS_A, LSM330DLC_FIFO_CTRL_REG_A, value) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LSM330DLC_SetWaterMark_A
* Description    : Sets Watermark Value Acc
* Input          : Watermark = [0,31]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330DLC_SetWaterMark_A(u8_t wtm) {
  u8_t value;
  
  if(wtm > 31)
    return MEMS_ERROR;  
  
  if( !LSM330DLC_ReadReg(LSM330DLC_MEMS_I2C_ADDRESS_A, LSM330DLC_FIFO_CTRL_REG_A, &value) )
    return MEMS_ERROR;
                  
  value &= 0xE0;
  value |= wtm; 
  
  if( !LSM330DLC_WriteReg(LSM330DLC_MEMS_I2C_ADDRESS_A, LSM330DLC_FIFO_CTRL_REG_A, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LSM330DLC_GetStatusReg_A
* Description    : Read the status register Acc
* Input          : char to empty by Status Reg Value
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330DLC_GetStatusReg_A(u8_t* val) {

  if( !LSM330DLC_ReadReg(LSM330DLC_MEMS_I2C_ADDRESS_A, LSM330DLC_STATUS_REG_A, val) )
      return MEMS_ERROR;
  
  return MEMS_SUCCESS;  
}


/*******************************************************************************
* Function Name  : LSM330DLC_GetStatusBIT_A
* Description    : Read the status register BIT Acc
* Input          : LSM330DLC_STATUS_REG_ZYXOR, LSM330DLC_STATUS_REG_ZOR, LSM330DLC_STATUS_REG_YOR, LSM330DLC_STATUS_REG_XOR,
                   LSM330DLC_STATUS_REG_ZYXDA, LSM330DLC_STATUS_REG_ZDA, LSM330DLC_STATUS_REG_YDA, LSM330DLC_STATUS_REG_XDA, LSM330DLC_DATAREADY_BIT
* Output         : status register BIT
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330DLC_GetStatusBit_A(u8_t statusBIT) {
  u8_t value;  
  
  if( !LSM330DLC_ReadReg(LSM330DLC_MEMS_I2C_ADDRESS_A, LSM330DLC_STATUS_REG_A, &value) )
      return MEMS_ERROR;
 
  switch (statusBIT){
    case LSM330DLC_STATUS_REG_ZYXOR:     if(value &= LSM330DLC_STATUS_REG_ZYXOR) return MEMS_SUCCESS;
										 else  return MEMS_ERROR; 
    case LSM330DLC_STATUS_REG_ZOR:       if(value &= LSM330DLC_STATUS_REG_ZOR) return MEMS_SUCCESS;
                                         else  return MEMS_ERROR;
    case LSM330DLC_STATUS_REG_YOR:       if(value &= LSM330DLC_STATUS_REG_YOR) return MEMS_SUCCESS;
										 else  return MEMS_ERROR;                               
    case LSM330DLC_STATUS_REG_XOR:       if(value &= LSM330DLC_STATUS_REG_XOR) return MEMS_SUCCESS;
                                         else  return MEMS_ERROR;   
    case LSM330DLC_STATUS_REG_ZYXDA:     if(value &= LSM330DLC_STATUS_REG_ZYXDA) return MEMS_SUCCESS;
                                         else  return MEMS_ERROR; 
    case LSM330DLC_STATUS_REG_ZDA:       if(value &= LSM330DLC_STATUS_REG_ZDA) return MEMS_SUCCESS;
                                         else  return MEMS_ERROR; 
    case LSM330DLC_STATUS_REG_YDA:       if(value &= LSM330DLC_STATUS_REG_YDA) return MEMS_SUCCESS;
                                         else  return MEMS_ERROR; 
    case LSM330DLC_STATUS_REG_XDA:       if(value &= LSM330DLC_STATUS_REG_XDA) return MEMS_SUCCESS;
                                         else  return MEMS_ERROR;                                
    
  }
return MEMS_ERROR;
}


/*******************************************************************************
* Function Name  : LSM330DLC_GetAccAxesRaw Acc
* Description    : Read the Acceleration Values Output Registers Acc
* Input          : buffer to empty by AccAxesRaw_t Typedef
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330DLC_GetAccAxesRaw_A(AxesRaw_t* buff) {
  u8_t valueL;
  u8_t valueH;
  u8_t reg, coeff;
  //check HR bit
  if( !LSM330DLC_ReadReg(LSM330DLC_MEMS_I2C_ADDRESS_A, LSM330DLC_CTRL_REG4_A, &reg) )
      return MEMS_ERROR;
  //if High Resolution enabled 
  if(reg & 0x08){ coeff = 16; }
  //if High Resolution disabled
  else {coeff = (u8_t)256;} 
  
  if( !LSM330DLC_ReadReg(LSM330DLC_MEMS_I2C_ADDRESS_A, LSM330DLC_OUT_X_L_A, &valueL) )
      return MEMS_ERROR;
  
  if( !LSM330DLC_ReadReg(LSM330DLC_MEMS_I2C_ADDRESS_A, LSM330DLC_OUT_X_H_A, &valueH) )
      return MEMS_ERROR;
  
  buff->AXIS_X = (i16_t)( (valueH << 8) | valueL )/coeff;
  
  if( !LSM330DLC_ReadReg(LSM330DLC_MEMS_I2C_ADDRESS_A, LSM330DLC_OUT_Y_L_A, &valueL) )
      return MEMS_ERROR;
  
  if( !LSM330DLC_ReadReg(LSM330DLC_MEMS_I2C_ADDRESS_A, LSM330DLC_OUT_Y_H_A, &valueH) )
      return MEMS_ERROR;
  
  buff->AXIS_Y = (i16_t)( (valueH << 8) | valueL )/coeff;
  
   if( !LSM330DLC_ReadReg(LSM330DLC_MEMS_I2C_ADDRESS_A, LSM330DLC_OUT_Z_L_A, &valueL) )
      return MEMS_ERROR;
  
  if( !LSM330DLC_ReadReg(LSM330DLC_MEMS_I2C_ADDRESS_A, LSM330DLC_OUT_Z_H_A, &valueH) )
      return MEMS_ERROR;
  
  buff->AXIS_Z = (i16_t)( (valueH << 8) | valueL )/coeff;
  
  return MEMS_SUCCESS;  
}


/*******************************************************************************
* Function Name  : LSM330DLC_GetInt1Src_A
* Description    : Reset Interrupt 1 Latching function Acc
* Input          : Char to empty by Int1 source value
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330DLC_GetInt1Src_A(u8_t* val) {
  
  if( !LSM330DLC_ReadReg(LSM330DLC_MEMS_I2C_ADDRESS_A, LSM330DLC_INT1_SRC_A, val) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LSM330DLC_GetInt1SrcBit_A
* Description    : Reset Interrupt 1 Latching function Acc
* Input          : LSM330DLC_INT1_SRC_IA, LSM330DLC_INT1_SRC_ZH, LSM330DLC_INT1_SRC_ZL, LSM330DLC_INT1_SRC_YH .....
* Output         : None
* Return         : Status of BIT [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330DLC_GetInt1SrcBit_A(u8_t statusBIT) {
  u8_t value;  
  
  if( !LSM330DLC_ReadReg(LSM330DLC_MEMS_I2C_ADDRESS_A, LSM330DLC_INT1_SRC_A, &value) )
      return MEMS_ERROR;
 
  if(statusBIT == LSM330DLC_INT1_SRC_IA_A){
    if(value &= LSM330DLC_INT1_SRC_IA_A)    return MEMS_SUCCESS;
    else  return MEMS_ERROR;  
  }
  
  if(statusBIT == LSM330DLC_INT1_SRC_ZH_A){
    if(value &= LSM330DLC_INT1_SRC_ZH_A)    return MEMS_SUCCESS;
    else  return MEMS_ERROR;  
  }

  if(statusBIT == LSM330DLC_INT1_SRC_ZL_A){
    if(value &= LSM330DLC_INT1_SRC_ZL_A)    return MEMS_SUCCESS;
    else  return MEMS_ERROR;  
  }
  
  if(statusBIT == LSM330DLC_INT1_SRC_YH_A){
    if(value &= LSM330DLC_INT1_SRC_YH_A)    return MEMS_SUCCESS;
    else  return MEMS_ERROR;  
  }

  if(statusBIT == LSM330DLC_INT1_SRC_YL_A){
    if(value &= LSM330DLC_INT1_SRC_YL_A)    return MEMS_SUCCESS;
    else  return MEMS_ERROR;  
  }

  if(statusBIT == LSM330DLC_INT1_SRC_XH_A){
    if(value &= LSM330DLC_INT1_SRC_XH_A)    return MEMS_SUCCESS;
    else  return MEMS_ERROR;  
  }

  if(statusBIT == LSM330DLC_INT1_SRC_XL_A){
    if(value &= LSM330DLC_INT1_SRC_XL_A)    return MEMS_SUCCESS;
    else  return MEMS_ERROR;  
  } 
return MEMS_ERROR;
}


/*******************************************************************************
* Function Name  : LSM330DLC_GetFifoSourceReg_A
* Description    : Read Fifo source Register Acc
* Input          : Byte to empty by FIFO source register value
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330DLC_GetFifoSourceReg_A(u8_t* val) {
  
  if( !LSM330DLC_ReadReg(LSM330DLC_MEMS_I2C_ADDRESS_A, LSM330DLC_FIFO_SRC_REG_A, val) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LSM330DLC_GetFifoSourceBit_A
* Description    : Read Fifo WaterMark source bit Acc
* Input          : LSM330DLC_FIFO_SRC_WTM, LSM330DLC_FIFO_SRC_OVRUN, LSM330DLC_FIFO_SRC_EMPTY
* Output         : None
* Return         : Status of BIT [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330DLC_GetFifoSourceBit_A(u8_t statusBIT){
  u8_t value;  
  
  if( !LSM330DLC_ReadReg(LSM330DLC_MEMS_I2C_ADDRESS_A, LSM330DLC_FIFO_SRC_REG_A, &value) )
      return MEMS_ERROR;
 
  if(statusBIT == LSM330DLC_FIFO_SRC_WTM){
    if(value &= LSM330DLC_FIFO_SRC_WTM)     return MEMS_SUCCESS;
    else  return MEMS_ERROR;  
  }
  
  if(statusBIT == LSM330DLC_FIFO_SRC_OVRUN){
    if(value &= LSM330DLC_FIFO_SRC_OVRUN)   return MEMS_SUCCESS;
    else  return MEMS_ERROR;  
  }

  if(statusBIT == LSM330DLC_FIFO_SRC_EMPTY){
    if(value &= LSM330DLC_FIFO_SRC_EMPTY)   return MEMS_SUCCESS;
    else  return MEMS_ERROR;  
  } 
return MEMS_ERROR;
}


/*******************************************************************************
* Function Name  : LSM330DLC_GetFifoSourceFSS_A
* Description    : Read current number of unread samples stored in FIFO Acc
* Input          : Byte to empty by FIFO unread sample value
* Output         : None
* Return         : Status [value of FSS]
*******************************************************************************/
status_t LSM330DLC_GetFifoSourceFSS_A(u8_t* val){
  u8_t value;
  
  if( !LSM330DLC_ReadReg(LSM330DLC_MEMS_I2C_ADDRESS_A, LSM330DLC_FIFO_SRC_REG_A, &value) )
    return MEMS_ERROR;
 
  value &= 0x1F;
  
  *val = value;
  
  return MEMS_SUCCESS;
}

      
/*******************************************************************************
* Function Name  : LSM330DLC_SetSPIInterface_A
* Description    : Set SPI mode: 3 Wire Interface OR 4 Wire Interface Acc
* Input          : LSM330DLC_SPI_3_WIRE, LSM330DLC_SPI_4_WIRE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330DLC_SetSPIInterface_A(LSM330DLC_SPIMode_t spi) {
  u8_t value;
  
  if( !LSM330DLC_ReadReg(LSM330DLC_MEMS_I2C_ADDRESS_A, LSM330DLC_CTRL_REG4_A, &value) )
    return MEMS_ERROR;
                  
  value &= 0xFE;
  value |= spi<<LSM330DLC_SIM_A;
  
  if( !LSM330DLC_WriteReg(LSM330DLC_MEMS_I2C_ADDRESS_A, LSM330DLC_CTRL_REG4_A, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/******************************************************************/
/*************function Gyro****************************************/

/*******************************************************************************
* Function Name  : LSM330DLC_SetODR_G
* Description    : Sets LSM330DLC Output Data Rate Gyro
* Input          : Output Data Rate
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330DLC_SetODR_G(LSM330DLC_ODR_G_t ov){
  u8_t value;

  if( !LSM330DLC_ReadReg(LSM330DLC_MEMS_I2C_ADDRESS_G, LSM330DLC_CTRL_REG1_G, &value) )
    return MEMS_ERROR;

  value &= 0x0f;
  value |= ov<<4;

  if( !LSM330DLC_WriteReg(LSM330DLC_MEMS_I2C_ADDRESS_G, LSM330DLC_CTRL_REG1_G, value) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LSM330DLC_GetWHO_AM_I
* Description    : Read identification code by WHO_AM_I register
* Input          : Char to empty by Device identification Value
* Output         : None
* Return         : Status [value of FSS]
*******************************************************************************/
status_t LSM330DLC_GetWHO_AM_I(u8_t* val){
  
    // Contact WHO_AM_I_G register 0x0F in Gyro - expected answer 0xD4
	/* Equiv.
    uint8_t buf[] = {0,0,0,0,0,0};
    buf[0] = LSM330DLC_WHO_AM_I_G;
    i2c_transmitinit(LSM330DLC_MEMS_I2C_ADDRESS_G, 1, buf);
    while(!i2c_transferred());

    i2c_receiveinit(LSM330DLC_MEMS_I2C_ADDRESS_G, 1, buf);
    while(!i2c_transferred());
	printf("WHO_AM_I = 0x%02X\n", buf[0]);
	*/

	if( !LSM330DLC_ReadReg(LSM330DLC_MEMS_I2C_ADDRESS_G, LSM330DLC_WHO_AM_I_G, val) )
		return MEMS_ERROR;
	return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LSM330DLC_SetMode_G
* Description    : Sets LSM330DLC Operating Mode gyro
* Input          : Modality (NORMAL, SLEEP, POWER_DOWN)
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330DLC_SetMode_G(LSM330DLC_Mode_G_t md) {
  u8_t value;
  
  if( !LSM330DLC_ReadReg(LSM330DLC_MEMS_I2C_ADDRESS_G, LSM330DLC_CTRL_REG1_G, &value) )
    return MEMS_ERROR;
                  
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
  
  if( !LSM330DLC_WriteReg(LSM330DLC_MEMS_I2C_ADDRESS_G, LSM330DLC_CTRL_REG1_G, value) )
    return MEMS_ERROR;
                  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LSM330DLC_SetAxis_G
* Description    : Enable/Disable LSM330DLC Axis Gyro
* Input          : LSM330DLC_X_ENABLE/DISABLE | LSM330DLC_Y_ENABLE/DISABLE | LSM330DLC_Z_ENABLE/DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330DLC_SetAxis_G(LSM330DLC_AXISenable_G_t axis) {
  u8_t value;
  
  if( !LSM330DLC_ReadReg(LSM330DLC_MEMS_I2C_ADDRESS_G, LSM330DLC_CTRL_REG1_G, &value) )
    return MEMS_ERROR;
    
  value &= 0xf8;
  value |= axis;
  
  if( !LSM330DLC_WriteReg(LSM330DLC_MEMS_I2C_ADDRESS_G, LSM330DLC_CTRL_REG1_G, value) )
    return MEMS_ERROR;  
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LSM330DLC_SetFullScale_G
* Description    : Sets the LSM330DLC FullScale Gyro
* Input          : LSM330DLC_FS_250/LSM330DLC_FS_500/LSM330DLC_FS_2000
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330DLC_SetFullScale_G(LSM330DLC_Fullscale_G_t fs) {
  u8_t value;
  
  if( !LSM330DLC_ReadReg(LSM330DLC_MEMS_I2C_ADDRESS_G, LSM330DLC_CTRL_REG4_G, &value) )
    return MEMS_ERROR;
                  
  value &= 0xCF;	
  value |= (fs<<LSM330DLC_FS_G);
  
  if( !LSM330DLC_WriteReg(LSM330DLC_MEMS_I2C_ADDRESS_G, LSM330DLC_CTRL_REG4_G, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LSM330DLC_SetBDU_G
* Description    : Enable/Disable Block Data Update Functionality Gyro
* Input          : ENABLE/DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330DLC_SetBDU_G(State_t bdu) {
  u8_t value;
  
  if( !LSM330DLC_ReadReg(LSM330DLC_MEMS_I2C_ADDRESS_G, LSM330DLC_CTRL_REG4_G, &value) )
    return MEMS_ERROR;
 
  value &= 0x7F;
  value |= (bdu<<LSM330DLC_BDU_G);

  if( !LSM330DLC_WriteReg(LSM330DLC_MEMS_I2C_ADDRESS_G, LSM330DLC_CTRL_REG4_G, value) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LSM330DLC_SetBLE_G
* Description    : Set Endianess (MSB/LSB) Gyro
* Input          : LSM330DLC_BLE_LSB / LSM330DLC_BLE_MSB
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330DLC_SetBLE_G(LSM330DLC_Endianess_t ble) {
  u8_t value;
  
  if( !LSM330DLC_ReadReg(LSM330DLC_MEMS_I2C_ADDRESS_G, LSM330DLC_CTRL_REG4_G, &value) )
    return MEMS_ERROR;
                  
  value &= 0xBF;	
  value |= (ble<<LSM330DLC_BLE_G);
  
  if( !LSM330DLC_WriteReg(LSM330DLC_MEMS_I2C_ADDRESS_G, LSM330DLC_CTRL_REG4_G, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LSM330DLC_HPFEnable_G
* Description    : Enable/Disable High Pass Filter Gyro
* Input          : ENABLE/DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330DLC_HPFEnable_G(State_t hpf) {
  u8_t value;
  
  if( !LSM330DLC_ReadReg(LSM330DLC_MEMS_I2C_ADDRESS_G, LSM330DLC_CTRL_REG5_G, &value) )
    return MEMS_ERROR;
                  
  value &= 0xEF;
  value |= (hpf<<LSM330DLC_HPEN_G);
  
  if( !LSM330DLC_WriteReg(LSM330DLC_MEMS_I2C_ADDRESS_G, LSM330DLC_CTRL_REG5_G, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LSM330DLC_SetHPFMode_G
* Description    : Set High Pass Filter Modality gyro
* Input          : LSM330DLC_HPM_NORMAL_MODE_RES/LSM330DLC_HPM_REF_SIGNAL/LSM330DLC_HPM_NORMAL_MODE/LSM330DLC_HPM_AUTORESET_INT
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330DLC_SetHPFMode_G(LSM330DLC_HPFMode_t hpf) {
  u8_t value;
  
  if( !LSM330DLC_ReadReg(LSM330DLC_MEMS_I2C_ADDRESS_G, LSM330DLC_CTRL_REG2_G, &value) )
    return MEMS_ERROR;
                  
  value &= 0xCF;
  value |= (hpf<<LSM330DLC_HPM_G);
  
  if( !LSM330DLC_WriteReg(LSM330DLC_MEMS_I2C_ADDRESS_G, LSM330DLC_CTRL_REG2_G, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LSM330DLC_SetHPFCutOFF_G
* Description    : Set High Pass CUT OFF Freq Gyro
* Input          : LSM330DLC_HPFCF_0,LSM330DLC_HPFCF_1,LSM330DLC_HPFCF_2... See Table 27 of the datasheet
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330DLC_SetHPFCutOFF_G(LSM330DLC_HPFCutOffFreq_t hpf) {
  u8_t value;
  
  if( !LSM330DLC_ReadReg(LSM330DLC_MEMS_I2C_ADDRESS_G, LSM330DLC_CTRL_REG2_G, &value) )
    return MEMS_ERROR;
                  
  value &= 0xF0;
  value |= (hpf<<LSM330DLC_HPFC0_G);
  
  if( !LSM330DLC_WriteReg(LSM330DLC_MEMS_I2C_ADDRESS_G, LSM330DLC_CTRL_REG2_G, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
  
}


/*******************************************************************************
* Function Name  : LSM330DLC_SetIntPinMode_G
* Description    : Set Interrupt Pin Modality (push pull or Open drain) gyro
* Input          : PUSH_PULL/OPEN_DRAIN
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330DLC_SetIntPinMode_G(LSM330DLC_IntPinMode_G_t pm) {
  u8_t value;
  
  if( !LSM330DLC_ReadReg(LSM330DLC_MEMS_I2C_ADDRESS_G, LSM330DLC_CTRL_REG3_G, &value) )
    return MEMS_ERROR;
                  
  value &= 0xEF;
  value |= (pm<<LSM330DLC_PP_OD_G);
  
  if( !LSM330DLC_WriteReg(LSM330DLC_MEMS_I2C_ADDRESS_G, LSM330DLC_CTRL_REG3_G, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LSM330DLC_SetInt1Pin_G
* Description    : Set Interrupt1 pin Function Gyro
* Input          : LSM330DLC_I1_ON_PIN_INT1_ENABLE | LSM330DLC_I1_BOOT_ON_INT1 | LSM330DLC_INT1_ACTIVE_HIGH
* example        : LSM330DLC_SetInt1Pin(LSM330DLC_I1_ON_PIN_INT1_ENABLE | LSM330DLC_I1_BOOT_ON_INT1_ENABLE | LSM330DLC_INT1_ACTIVE_LOW) 
* to enable Interrupt 1 or Bootstatus on interrupt 1 pin
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330DLC_SetInt1Pin_G(LSM330DLC_Int1PinConf_t pinConf) {
  u8_t value;
  
  if( !LSM330DLC_ReadReg(LSM330DLC_MEMS_I2C_ADDRESS_G, LSM330DLC_CTRL_REG3_G, &value) )
    return MEMS_ERROR;
                  
  value &= 0x1F;
  value |= pinConf;
  
  if( !LSM330DLC_WriteReg(LSM330DLC_MEMS_I2C_ADDRESS_G, LSM330DLC_CTRL_REG3_G, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LSM330DLC_SetInt2Pin_G
* Description    : Set Interrupt2 pin Function Gyro
* Input          : LSM330DLC_I2_DRDY_ON_INT2_ENABLE/DISABLE | 
                   LSM330DLC_WTM_ON_INT2_ENABLE/DISABLE | 
                   LSM330DLC_OVERRUN_ON_INT2_ENABLE/DISABLE | 
                   LSM330DLC_EMPTY_ON_INT2_ENABLE/DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330DLC_SetInt2Pin_G(LSM330DLC_Int2PinConf_t pinConf) {
  u8_t value;
  
  if( !LSM330DLC_ReadReg(LSM330DLC_MEMS_I2C_ADDRESS_G, LSM330DLC_CTRL_REG3_G, &value) )
    return MEMS_ERROR;
                  
  value &= 0xF0;
  value |= pinConf;
  
  if( !LSM330DLC_WriteReg(LSM330DLC_MEMS_I2C_ADDRESS_G, LSM330DLC_CTRL_REG3_G, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;  
}


/*******************************************************************************
* Function Name  : LSM330DLC_Int1LatchEnable_G
* Description    : Enable Interrupt 1 Latching function Gyro
* Input          : ENABLE/DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330DLC_Int1LatchEnable_G(State_t latch) {
  u8_t value;
  
  if( !LSM330DLC_ReadReg(LSM330DLC_MEMS_I2C_ADDRESS_G, LSM330DLC_INT1_CFG_G, &value) )
    return MEMS_ERROR;
                  
  value &= 0xBF;
  value |= latch<<LSM330DLC_LIR_G;
  
  if( !LSM330DLC_WriteReg(LSM330DLC_MEMS_I2C_ADDRESS_G, LSM330DLC_INT1_CFG_G, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LSM330DLC_ResetInt1Latch_G
* Description    : Reset Interrupt 1 Latching function Gyro
* Input          : None
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330DLC_ResetInt1Latch_G(void) {
  u8_t value;
  
  if( !LSM330DLC_ReadReg(LSM330DLC_MEMS_I2C_ADDRESS_G, LSM330DLC_INT1_SRC_G, &value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LSM330DLC_SetIntConfiguration_G
* Description    : Interrupt 1 Configuration Gyro
* Input          : LSM330DLC_AND/OR, LSM330DLC_INT1_LIR ZHIE_ENABLE/DISABLE | LSM330DLC_INT1_ZLIE_ENABLE/DISABLE...
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330DLC_SetIntConfiguration_G(LSM330DLC_Int1Conf_t ic) {
  u8_t value;
  
  value = ic;

  if( !LSM330DLC_WriteReg(LSM330DLC_MEMS_I2C_ADDRESS_G, LSM330DLC_INT1_CFG_G, value) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LSM330DLC_SetInt1Threshold_G
* Description    : Sets Interrupt 1 Threshold Gyro
* Input          : Threshold = [0,31]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330DLC_SetInt1Threshold_G(LSM330DLC_IntThsAxis_G axis, u16_t ths) {
  u8_t value;
  
  switch (axis) {
    
    case LSM330DLC_THS_X_G:
      //write the threshold LSB
      value = (u8_t)( ths & 0x00ff); 
      if( !LSM330DLC_WriteReg(LSM330DLC_MEMS_I2C_ADDRESS_G, LSM330DLC_INT1_THS_XL_G, value) )
        return MEMS_ERROR;
      
      //write the threshold LSB
      value = (u8_t)( ths >> 8); 
      if( !LSM330DLC_WriteReg(LSM330DLC_MEMS_I2C_ADDRESS_G, LSM330DLC_INT1_THS_XH_G, value) )
        return MEMS_ERROR;
      
      break;
      
    case LSM330DLC_THS_Y_G:
      //write the threshold LSB
      value = (u8_t)( ths & 0x00ff); 
      if( !LSM330DLC_WriteReg(LSM330DLC_MEMS_I2C_ADDRESS_G, LSM330DLC_INT1_THS_YL_G, value) )
        return MEMS_ERROR;
      
      //write the threshold LSB
      value = (u8_t)( ths >> 8); 
      if( !LSM330DLC_WriteReg(LSM330DLC_MEMS_I2C_ADDRESS_G, LSM330DLC_INT1_THS_YH_G, value) )
        return MEMS_ERROR;
      
      break;
      
    case LSM330DLC_THS_Z_G:
      //write the threshold LSB
      value = (u8_t)( ths & 0x00ff); 
      if( !LSM330DLC_WriteReg(LSM330DLC_MEMS_I2C_ADDRESS_G, LSM330DLC_INT1_THS_ZL_G, value) )
        return MEMS_ERROR;
      
      //write the threshold LSB
      value = (u8_t)( ths >> 8); 
      if( !LSM330DLC_WriteReg(LSM330DLC_MEMS_I2C_ADDRESS_G, LSM330DLC_INT1_THS_ZH_G, value) )
        return MEMS_ERROR;
      
      break;     

        
  }

  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LSM330DLC_SetInt1Duration_G
* Description    : Sets Interrupt 1 Duration Gyro
* Input          : Duration value
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330DLC_SetInt1Duration_G(LSM330DLC_Int1Conf_t id) {
 
  if (id > 127)
    return MEMS_ERROR;

  if( !LSM330DLC_WriteReg(LSM330DLC_MEMS_I2C_ADDRESS_G, LSM330DLC_INT1_DURATION_G, id) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LSM330DLC_FIFOModeEnable_G
* Description    : Sets Fifo Modality Gyro
* Input          : 
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330DLC_FIFOModeEnable_G(LSM330DLC_FifoMode_G_t fm) {
  u8_t value;  
  
  if(fm == LSM330DLC_FIFO_DISABLE_G) {
    
    if( !LSM330DLC_ReadReg(LSM330DLC_MEMS_I2C_ADDRESS_G, LSM330DLC_CTRL_REG5_G, &value) )
      return MEMS_ERROR;
                    
    value &= 0xBF;    
    
    if( !LSM330DLC_WriteReg(LSM330DLC_MEMS_I2C_ADDRESS_G, LSM330DLC_CTRL_REG5_G, value) )
      return MEMS_ERROR;
    
  }
  else {
    
    if( !LSM330DLC_ReadReg(LSM330DLC_MEMS_I2C_ADDRESS_G, LSM330DLC_CTRL_REG5_G, &value) )
      return MEMS_ERROR;
                    
    value &= 0xBF;
    value |= MEMS_SET<<LSM330DLC_FIFO_EN_G;
    
    if( !LSM330DLC_WriteReg(LSM330DLC_MEMS_I2C_ADDRESS_G, LSM330DLC_CTRL_REG5_G, value) )
      return MEMS_ERROR;
    
    
    if( !LSM330DLC_ReadReg(LSM330DLC_MEMS_I2C_ADDRESS_G, LSM330DLC_FIFO_CTRL_REG_G, &value) )
      return MEMS_ERROR;
    
    value &= 0x1f;
    value |= (fm<<LSM330DLC_FM0_G);
    
    if( !LSM330DLC_WriteReg(LSM330DLC_MEMS_I2C_ADDRESS_G, LSM330DLC_FIFO_CTRL_REG_G, value) )
      return MEMS_ERROR;
  }

  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LSM330DLC_SetWaterMark_G
* Description    : Sets Watermark Value Gyro
* Input          : Watermark = [0,31]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330DLC_SetWaterMark_G(u8_t wtm) {
  u8_t value;
  
  if(wtm > 31)
    return MEMS_ERROR;  
  
  if( !LSM330DLC_ReadReg(LSM330DLC_MEMS_I2C_ADDRESS_G, LSM330DLC_FIFO_CTRL_REG_G, &value) )
    return MEMS_ERROR;
                  
  value &= 0xE0;
  value |= wtm; 
  
  if( !LSM330DLC_WriteReg(LSM330DLC_MEMS_I2C_ADDRESS_G, LSM330DLC_FIFO_CTRL_REG_G, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LSM330DLC_GetStatusReg_G
* Description    : Read the status register Gyro
* Input          : None
* Output         : status register buffer
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330DLC_GetStatusReg(u8_t* buff) {
  if( !LSM330DLC_ReadReg(LSM330DLC_MEMS_I2C_ADDRESS_G, LSM330DLC_STATUS_REG_G, buff) )
      return MEMS_ERROR;
  
  return MEMS_SUCCESS;  
}


/*******************************************************************************
* Function Name  : LSM330DLC_GetAngRateRaw
* Description    : Read the Angular Rate Registers Gyro
* Input          : None
* Output         : Angular Rate Registers buffer
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330DLC_GetAngRateRaw(LSM330DLC_AngRateRaw_t* buff) {
  u8_t valueL;
  u8_t valueH;
  

  if( !LSM330DLC_ReadReg(LSM330DLC_MEMS_I2C_ADDRESS_G, LSM330DLC_OUT_X_L_G, &valueL) )
      return MEMS_ERROR;
  
  if( !LSM330DLC_ReadReg(LSM330DLC_MEMS_I2C_ADDRESS_G, LSM330DLC_OUT_X_H_G, &valueH) )
      return MEMS_ERROR;
  
  buff->x = (i16_t)( (valueH << 8) | valueL );
  
  if( !LSM330DLC_ReadReg(LSM330DLC_MEMS_I2C_ADDRESS_G, LSM330DLC_OUT_Y_L_G, &valueL) )
      return MEMS_ERROR;
  
  if( !LSM330DLC_ReadReg(LSM330DLC_MEMS_I2C_ADDRESS_G, LSM330DLC_OUT_Y_H_G, &valueH) )
      return MEMS_ERROR;
  
  buff->y = (i16_t)( (valueH << 8) | valueL );
  
   if( !LSM330DLC_ReadReg(LSM330DLC_MEMS_I2C_ADDRESS_G, LSM330DLC_OUT_Z_L_G, &valueL) )
      return MEMS_ERROR;
  
  if( !LSM330DLC_ReadReg(LSM330DLC_MEMS_I2C_ADDRESS_G, LSM330DLC_OUT_Z_H_G, &valueH) )
      return MEMS_ERROR;
  
  buff->z = (i16_t)( (valueH << 8) | valueL );
  
  return MEMS_SUCCESS;  
}


/*******************************************************************************
* Function Name  : LSM330DLC_GetInt1Src_G
* Description    : Reset Interrupt 1 Latching function Gyro
* Input          : None
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330DLC_GetInt1Src_G(u8_t* buff) {
  
  if( !LSM330DLC_ReadReg(LSM330DLC_MEMS_I2C_ADDRESS_G, LSM330DLC_INT1_SRC_G, buff) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LSM330DLC_GetFifoSourceReg_G
* Description    : Read Fifo source Register gyro
* Input          : None
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330DLC_GetFifoSourceReg_G(u8_t* buff) {
  
  if( !LSM330DLC_ReadReg(LSM330DLC_MEMS_I2C_ADDRESS_G, LSM330DLC_FIFO_SRC_REG_G, buff) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LSM330DLC_SetOutputDataAndFifoFilters_G
* Description    : ENABLE/DISABLE HIGH PASS and LOW PASS filters applied to output and fifo registers Gyro
*                : See Table 8 of AN3393 for more details
* Input          : LSM330DLC_NONE, LSM330DLC_HPH, LSM330DLC_LPF2, LSM330DLC_HPFLPF2
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330DLC_SetOutputDataAndFifoFilters_G(LSM330DLC_HPF_LPF2_Enable_G hpf){
  u8_t value;
  
  //HPF
  if( !LSM330DLC_ReadReg(LSM330DLC_MEMS_I2C_ADDRESS_G, LSM330DLC_CTRL_REG5_G, &value) )
    return MEMS_ERROR;
  
  switch(hpf) {
    
  case LSM330DLC_NONE_G:
    value &= 0xfc;
    value |= 0x00; //hpen = x, Out_sel_1 = 0, Out_sel_0 = 0
    break;
    
  case LSM330DLC_HPF_G:
    value &= 0xfc;
    value |= 0x01; //hpen = x, Out_sel_1 = 0, Out_sel_0 = 1
    break;

  case LSM330DLC_LPF2_G:
    value &= 0xed;
    value |= 0x02; //hpen = 0, Out_sel_1 = 1, Out_sel_0 = x
    break;    
   
  case LSM330DLC_HPFLPF2_G:
    value &= 0xed;
    value |= 0x12; //hpen = 1, Out_sel_1 = 1, Out_sel_0 = x
    break;    
  }
  
  if( !LSM330DLC_WriteReg(LSM330DLC_MEMS_I2C_ADDRESS_G, LSM330DLC_CTRL_REG5_G, value) )
    return MEMS_ERROR;
  
  
  return MEMS_SUCCESS;
  
}


/*******************************************************************************
* Function Name  : LSM330DLC_SetInt1Filters_G
* Description    : ENABLE/DISABLE HIGH PASS and LOW PASS filters applied to Int1 circuitery Gyro
*                : See Table 9 of AN3393 for more details
* Input          : LSM330DLC_NONE, LSM330DLC_HPH, LSM330DLC_LPF2, LSM330DLC_HPFLPF2
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330DLC_SetInt1Filters_G(LSM330DLC_HPF_LPF2_Enable_G hpf){
  u8_t value;
  
  //HPF
  if( !LSM330DLC_ReadReg(LSM330DLC_MEMS_I2C_ADDRESS_G, LSM330DLC_CTRL_REG5_G, &value) )
    return MEMS_ERROR;
  
  switch(hpf) {
    
  case LSM330DLC_NONE_G:
    value &= 0xf3;
    value |= 0x00<<LSM330DLC_INT1_SEL0_G; //hpen = x, Int1_sel_1 = 0, Int1_sel_0 = 0
    break;
    
  case LSM330DLC_HPF_G:
    value &= 0xf3;
    value |= 0x01<<LSM330DLC_INT1_SEL0_G; //hpen = x, Int1_sel_1 = 0, Int1_sel_0 = 1
    break;

  case LSM330DLC_LPF2_G:
    value &= 0xe7;
    value |= 0x02<<LSM330DLC_INT1_SEL0_G; //hpen = 0, Int1_sel_1 = 1, Int1_sel_0 = x
    break;    
   
  case LSM330DLC_HPFLPF2_G:
    value &= 0xe7;
    value |= 0x01<<LSM330DLC_HPEN_G;
    value |= 0x02<<LSM330DLC_INT1_SEL0_G; //hpen = 1, Int1_sel_1 = 1, Int1_sel_0 = x
    break;    
  }
  
  if( !LSM330DLC_WriteReg(LSM330DLC_MEMS_I2C_ADDRESS_G, LSM330DLC_CTRL_REG5_G, value) )
    return MEMS_ERROR;
  
  
  return MEMS_SUCCESS;
  
}


/*******************************************************************************
* Function Name  : LSM330DLC_SetSPIInterface_G
* Description    : Set SPI mode: 3 Wire Interface OR 4 Wire Interface Gyro
* Input          : LSM330DLC_SPI_3_WIRE, LSM330DLC_SPI_4_WIRE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330DLC_SetSPIInterface_G(LSM330DLC_SPIMode_t spi) {
  u8_t value;
  
  if( !LSM330DLC_ReadReg(LSM330DLC_MEMS_I2C_ADDRESS_G, LSM330DLC_CTRL_REG4_G, &value) )
    return MEMS_ERROR;
                  
  value &= 0xFE;
  value |= spi<<LSM330DLC_SIM_G;
  
  if( !LSM330DLC_WriteReg(LSM330DLC_MEMS_I2C_ADDRESS_G, LSM330DLC_CTRL_REG4_G, value) )
    return MEMS_ERROR;
  
  
  return MEMS_SUCCESS;
}

// The following functions have been added by Jagun Kwon @ UCL CS (jagun.kwon@ucl.ac.uk)
// Initialise the LSM330DLC module
int LSM330DLC_Init() {

  uint8_t buffer[50]; 
  uint8_t response; 
  int len = 0;

	LSM330DLC_GetWHO_AM_I(buffer);
	printf("DEBUG: WHO AM I: 0x%02X\n", buffer[0]);

 //set ACC ODR (turn ON device)
 response = LSM330DLC_SetODR_A(LSM330DLC_ODR_100Hz_A);
 if(response==1){  //debug print response
        len = sprintf((char*)buffer,"SET_ODR_OK ACC   \n\r\0");
		printf("DEBUG: LSM330DLC Init: %s", buffer);
  }
 
 //set ACC PowerMode 
 response = LSM330DLC_SetMode_A(LSM330DLC_NORMAL_A);
 if(response==1){  //debug print response
        len = sprintf((char*)buffer,"SET_MODE_OK ACC    \n\r\0");
		printf("DEBUG: LSM330DLC Init: %s", buffer);
  }
 //set ACC Fullscale
 response = LSM330DLC_SetFullScale_A(LSM330DLC_FULLSCALE_2_A);
 if(response==1){  //debug print response
        len = sprintf((char*)buffer,"SET_FULLSCALE_OK ACC\n\r\0");
		printf("DEBUG: LSM330DLC Init: %s", buffer);
  }
 //set ACC axis Enable
 response = LSM330DLC_SetAxis_A(LSM330DLC_X_ENABLE_A | LSM330DLC_Y_ENABLE_A | LSM330DLC_Z_ENABLE_A);
 if(response==1){     //debug print response
        len = sprintf((char*)buffer,"SET_AXIS_OK ACC    \n\r\0");
		printf("DEBUG: LSM330DLC Init: %s", buffer);
  }

  //set GYRO ODR (turn ON device)
 response = LSM330DLC_SetODR_G(LSM330DLC_ODR_95Hz_BW_25_G);
 if(response==1){  //debug print response
        len = sprintf((char*)buffer,"SET_ODR_OK GYRO   \n\r\0");
		printf("DEBUG: LSM330DLC Init: %s", buffer);
  }
 
 //set GYRO PowerMode 
 response = LSM330DLC_SetMode_G(LSM330DLC_NORMAL_G);
 if(response==1){  //debug print response
        len = sprintf((char*)buffer,"SET_MODE_OK GYRO    \n\r\0");
		printf("DEBUG: LSM330DLC Init: %s", buffer);
  }
 //set GYRO Fullscale
 response = LSM330DLC_SetFullScale_G(LSM330DLC_FULLSCALE_500_G);
 if(response==1){  //debug print response
        len = sprintf((char*)buffer,"SET_FULLSCALE_OK GYRO\n\r\0");
		printf("DEBUG: LSM330DLC Init: %s", buffer);
  }
 //set GYRO axis Enable
 response = LSM330DLC_SetAxis_G(LSM330DLC_X_ENABLE_G | LSM330DLC_Y_ENABLE_G | LSM330DLC_Z_ENABLE_G);
 if(response==1){     //debug print response
        len = sprintf((char*)buffer,"SET_AXIS_OK GYRO    \n\r\0");
		printf("DEBUG: LSM330DLC Init: %s", buffer);
  }
 
}
