/*
 * @Author: Wang Bing
 * @Date: 2021-10-22 14:26:45
 * @LastEditTime: 2023-02-08 10:27:10
 * @LastEditors: Wangbingbin bing.wang2@bshg.com
 * @Description: IIC_SEN communication with OLD module
 * @FilePath: \PUc:\Users\WangBingBin\AppData\Local\WangBing\Projects\EXTERNAL\Pressure\Pressure_16F18346.X\Pressure_16F18346.X\Application\PhyDriver\IIC_Sensor.c
 */

/***************************************************************************************
*                                   includes                                           *
 **************************************************************************************/ 
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

#include "hal_gpio.h" 
#include "hal_rcc.h"
#include "IICInterface.h"
#include "drv_iic.h"


/***************************************************************************************
*                                   variables                                          *
 **************************************************************************************/ 

 /**************************************************************************************
*                                   Functions                                          *
 **************************************************************************************/ 

/***************************************************************************************
*                                  IIC_SEN module transmission function                    *
 **************************************************************************************/
 
/**
 * @description: switch SDA to input mode
 * @param {*}
 * @return {*}
 */
void IIC_vSwitchToInputMode(void)
{
	DRV_vIICSDAConfigure(eINPUT_MODE);
}

/**
 * @description: witch SDA to output mode
 * @param {*}
 * @return {*}
 */
void IIC_vSwitchToOutputMode(void)
{
	DRV_vIICSDAConfigure(eOUTPUT_MODE);
}

//start signal
//keep SCL high level, set SDA to low level from high level as starting up signal.
void IIC_SEN_start(void)
{
	IIC_vSwitchToOutputMode();

	IIC_SEN_SDA_1;
	
	IIC_SEN_SCK_1;

	__NOP();
	__NOP();
					   //All operation finish, release SCL	
	IIC_SEN_SDA_0;

	__NOP();
	__NOP();
	
	IIC_SEN_SCK_0;
        
}

//stop signal
//Keep SCL high level. set SDA to high level from low level as the stop signal.
void IIC_SEN_stop(void)
{
	IIC_vSwitchToOutputMode();
    
	IIC_SEN_SCK_0;

	IIC_SEN_SDA_0;
	
	__NOP();
	__NOP();

	IIC_SEN_SCK_1;
	
	IIC_SEN_SDA_1;

	__NOP();
	__NOP();
	
}

void IIC_vMasterAck(void)
{
	IIC_SEN_SCK_0;

	IIC_vSwitchToOutputMode();

	IIC_SEN_SDA_0;

	__NOP();
	__NOP();

	IIC_SEN_SCK_1;

	__NOP();
	__NOP();

	IIC_SEN_SCK_0;
}

void IIC_vMasterNoAck(void)
{
	IIC_SEN_SCK_0;

	IIC_vSwitchToOutputMode();	

	IIC_SEN_SDA_1;

	__NOP();
	__NOP();

	IIC_SEN_SCK_1;

	__NOP();
	__NOP();

	IIC_SEN_SCK_0;

}

uint8_t IIC_vMasterWaitSlaveResponse(void)
{
	uint8_t u8TimeOut = 0;

	IIC_vSwitchToInputMode();	

	IIC_SEN_SDA_1;

	__NOP();
	__NOP();

	IIC_SEN_SCK_1;

	__NOP();
	__NOP();

	while(IIC_u8ReadSDA() == 1)
	{
		u8TimeOut ++;
		if(u8TimeOut > 250)
		{
			IIC_SEN_stop();

			return 1;
		}
	}
	IIC_SEN_SCK_0;
	return 0;

}

//writing, finally set SDA to high so that wait for response from slave device.
void IIC_SEN_writeByte(uint8_t data)
{
	uint8_t    i = 0;

	uint8_t temp = 0;

	temp = data;

	IIC_vSwitchToOutputMode();	

	IIC_SEN_SCK_0;

	for(i=0; i<8; i++)
	{	
    		
		if ((temp&0x80)==0)
		{
			IIC_SEN_SDA_0;
		}  
		else 
		{
			IIC_SEN_SDA_1;
		}
			temp = temp << 1;

			__NOP();
			__NOP();

			//at least 250ns delay
			IIC_SEN_SCK_1;

			__NOP();
			__NOP();

			IIC_SEN_SCK_0;

			__NOP();
			__NOP();

		}
	
}

//writing, finally set SDA to high so that wait for response from slave device.
uint8_t IIC_SEN_readByte(void)
{
	uint8_t    i = 0;
	uint8_t temp = 0;

	IIC_vSwitchToInputMode();

	for(i=0; i<8; i++)
	{
		IIC_SEN_SCK_0;

		__NOP();
		__NOP();

		IIC_SEN_SCK_1;

		temp = temp << 1;

		if (IIC_u8ReadSDA()==0)
		{
		temp = temp|0x00;
		}  
		else 
		{
		temp = temp|0x01;
		}

		__NOP();
		__NOP();

	}

	return temp;
	
}


void IIC_vWriteAdd(uint8_t u8Add, uint8_t u8Data)
{
	
	IIC_SEN_start();
	IIC_SEN_writeByte(PSR_IIC_WR_ADD);
	IIC_vMasterWaitSlaveResponse();
	IIC_SEN_writeByte(u8Add);
	IIC_vMasterWaitSlaveResponse();
	IIC_SEN_writeByte(u8Data);
	IIC_vMasterWaitSlaveResponse();
	IIC_SEN_stop();

}

uint8_t IIC_vReadAdd(uint8_t u8Add)
{
	
	uint8_t u8Data = 0;
	IIC_SEN_start();
	IIC_SEN_writeByte(PSR_IIC_WR_ADD);
	IIC_vMasterWaitSlaveResponse();
	IIC_SEN_writeByte(u8Add);
	IIC_vMasterWaitSlaveResponse();
	IIC_SEN_start();
	IIC_SEN_writeByte(PSR_IIC_RD_ADD);
	IIC_vMasterWaitSlaveResponse();
	u8Data = IIC_SEN_readByte();
	IIC_SEN_stop();
	return u8Data;
}


uint8_t IIC_u8ReadSDA(void)
{
	return (uint8_t)GPIO_ReadInputDataBit(GPIOA, GPIO_IIC_SDA);
}




