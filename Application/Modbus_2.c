/*
 * @Author: Wang Bing
 * @Date: 2021-09-06 12:44:09
 * @LastEditTime: 2023-03-16 00:21:26
 * @LastEditors: Wangbingbin bing.wang2@bshg.com
 * @Description: Modbus logic
 * @FilePath: \VE_CPM_CP_SMM_SW_S50PUc:\Users\WangBingBin\AppData\Local\WangBing\Projects\EXTERNAL\Pressure\Pressure_221206_16F18346.X\Application\Logic\Modbus\Modbus.c
 */

/****************************************************************************************
*                                   INCLUDE                                             *
 ***************************************************************************************/ 
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>


//#include "sysTick.h"
#include "Modbus.h"
#include "Modbus_2.h"
#include "drv_inc.h"
#include "hal_uart.h"
#include "ModbusCRC.h"
#include "mm32_it.h"


/****************************************************************************************
*                          declaration                                                  *
 ***************************************************************************************/ 
extern void MOS2_vTriggerEvent(uint8_t u8RegisterAddrss);
extern void MOS2_vEventHandler(void);
extern void MOS2_vRegisterHandler(uint8_t RegisterID);
extern void MOS2_vConfigureSlaveAddress(void);
extern void MOS2_vConfigurePassword(void);
extern void MOS2_vModbusProtocalConfig(void);

/****************************************************************************************
*                          definitions                                                  *
 ***************************************************************************************/ 

#define MOS2_READ_ALL_LEN          5

#define MOS2_WRITE_ALL_LEN          6

/****************************************************************************************
*                          Variables                                                    *
 ***************************************************************************************/ 
TModbusData MOS2_tModbusData                     = {{0},0,0,0,{0},0,0};
static TModbusPara MOS2_tModbusPara              = {0,{0,0},0,0,0};
static uint16_t MOS2_u8EventRegister             = 0;
static uint8_t  MOS2_u8IntervalTime              = BR_INTERVAL_9600;

static bool     MOS2_bInitFinished               = false;
static bool     MOS2_bTransmissionEn             = true;


static E_MODBUS2STATE  MOS2_u8Statemachine       = E_MODBUS2_INIT;
static uint8_t MOS2_u8ReadRegisterAdd            = 0;

/*
bandrate is 9600,
one bit time is 1000000us/9600bit/s = 104us, one byte is 104*10 = 1040us.
Modbus confirm frame finish time is >3.5*1040us = 3.64ms, set it to 10ms.
*/

uint8_t MOS2_aucReadAllData[] = {0x01,0x04,0x02,0x00,0x00,0x00,0x00};

uint8_t MOS2_aucWriteData[] = {0x01,0x06,0x00,0x00,0x00,0x00,0x00,0x00};
//static uint8_t ucTestinc;

uint16_t RegData2[] = {        //Data
                0x0000,
                0x0000,
                0x0000,
                0x0000,
                0x0000,
                0x0000,

            };

const uint8_t MOS2_auModebusIntervalTime[7] = {BR_INTERVAL_1200, BR_INTERVAL_2400, BR_INTERVAL_4800, BR_INTERVAL_9600, BR_INTERVAL_19200,BR_INTERVAL_38400,BR_INTERVAL_DEFAULT};


/****************************************************************************************
*                          Private function                                             *
 ***************************************************************************************/ 
/*
 * description: register the triggered event
 * 
 * parameter:uint8_t u8RegisterAddrss
 * event ID
 * 
 * return: void
 * 
 * 
 * */
void MOS2_vTriggerEvent(uint8_t u8RegisterAddrss)
{
	uint16_t u16RegisterEvent = 0x0001;
	u16RegisterEvent = u16RegisterEvent<<u8RegisterAddrss;
	MOS2_u8EventRegister = MOS2_u8EventRegister|u16RegisterEvent;
}

/**
 * description: Event searching based on modbus function6
 * 
 * parameter:void
 * 
 * return: void
 * 
 * 
 * */
void MOS2_vEventHandler(void)
{
	uint16_t u16RegisterIndex = 0x0001;
	uint8_t i = 0;
	for(i=0; i< 16; i++)
	{
		u16RegisterIndex = 0x0001;
		u16RegisterIndex = u16RegisterIndex<<i;

		if((MOS2_u8EventRegister & u16RegisterIndex) != 0)
		{
			MOS2_u8EventRegister = MOS2_u8EventRegister&(~u16RegisterIndex);
			MOS2_vRegisterHandler(i);
			break;
		}

	}
}

void MOS2_vWriteDataIntoEEP(void)
{

}

/**
 * description: detailed event handling slave address update
 * 
 * parameter:void
 * 
 * return: void
 * 
 * 
 * */
void MOS2_vSlaveAddressUpdate(void)
{

    MOS2_vWriteDataIntoEEP();
	RegData2[DATA2_SLAVE_ADD] = 0x0001;

}


void MOS2_vCallbackInit(void)
{

		/*Register callback function*/
	STK_vSetGenericTimerCallbackFuncUART3(MOS2_vTimeoutInterrupt);
	UART_vSetUartReceiveCallbackFuncUART3(MOS2_vDataReceiveInterrupt);

}


/**
 * description��registered event handling based on function 6
 * 
 * parameter:void
 * 
 * return: void
 * 
 * 
 * */
void MOS2_vRegisterHandler(uint8_t u8RegisterID)
{
	switch(u8RegisterID)
	{
		case DATA2_TARGET_SPD:
		{

			break;
		}

		case DATA2_ACTUAL_SPD:
		{

			break;
		}
		
		case DATA2_SLAVE_ADD:
		{
			MOS2_vSlaveAddressUpdate();
			break;
		}

		case DATA2_RESERVED1:
		{
			
			break;
		}

		case DATA2_RESERVED2:
		{
	
			break;
		}

		case DATA2_RESERVED3:
		{

			break;
		}
	}
}

/****************************************************************************************
*                          Public function                                             *
 ***************************************************************************************/ 
/**
 * description��modbus function module init
 * 
 * parameter:void
 * 
 * return: void
 * 
 * 
 * */
void MOS2_vModbusInit(void)
{
	MOS2_vCallbackInit();

	MOS2_vModbusProtocalConfig();//brandrate configure

	MOS2_vConfigureSlaveAddress();

	MOS2_tModbusData.u8TimeoutStart = 0; //MODbus timer stop

	MOS2_bInitFinished = true;

}


/**
 * description��modbus transtion control
 * 
 * parameter:void
 * 
 * return: void
 * 
 * 
 * */
void MOS2_vModbusTransmitEnalbe(void)
{

}


void MOS2_vModbusProtocalConfig(void)
{
	RegData2[DATA2_SLAVE_ADD] = 0x01;

}


/**
 * description: configure slave address
 * 
 * parameter:void
 * 
 * return: void
 * 
 * 
 * */
void MOS2_vConfigureSlaveAddress(void)
{
	MOS2_tModbusPara.u8DeviceAddress = RegData2[DATA2_SLAVE_ADD];
}



/***********************************************New Protocal for Customer************************************************/
bool MOS2_vCheckWriteDataCommand(void)
{
	uint8_t bReturnValue = false;
	if((MOS2_tModbusData.auReceiveBuff[1] == MOS2_aucWriteData[1])&&(MOS2_tModbusData.auReceiveBuff[1] == 0x06))//check function code
	{
		bReturnValue = true;
	}
	else
	{
		bReturnValue = false;
	}
	return bReturnValue;
}

/**
 * description: modbus function 6, write parameter
 * 
 * parameter:void
 * 
 * return: void
 * 
 * 
 * */
void MOS2_vModbudFun6(void)  //function code 6
{
  	uint16_t u16RegisterAdd = 0;
	uint16_t u16value = 0;

  	u16RegisterAdd=MOS2_tModbusData.auReceiveBuff[2]*256+MOS2_tModbusData.auReceiveBuff[3];  //get address to modify 
	u16value=MOS2_tModbusData.auReceiveBuff[4]*256+MOS2_tModbusData.auReceiveBuff[5];     //the value to be write
	RegData2[u16RegisterAdd]=u16value;  //update registers

	MOS2_vTriggerEvent(u16RegisterAdd);//trigger handler	

}


bool MOS2_vCheckReadAllDataCommand(void)
{
	uint8_t bReturnValue = false;
	if((MOS2_tModbusData.auReceiveBuff[1] == MOS2_aucReadAllData[1])&&(MOS2_tModbusData.auReceiveBuff[1] == 0x04))//check function code
	{
		bReturnValue = true;
	}
	else
	{
		bReturnValue = false;
	}
	return bReturnValue;
}

/**
 * description:modbus function 4, read pressure data
 * 
 * parameter:void
 * 
 * return: void
 * 
 * 
 * */
void MOS2_vModbusFun4(void)
{
  	uint16_t u16RegisterAdd = 0;
	uint16_t u16RegisterLen = 0;
	uint16_t j = 0;

	u16RegisterLen = MOS2_tModbusData.auReceiveBuff[2]/2;
	u16RegisterAdd = MOS2_u8ReadRegisterAdd;
	for(j=0;j<u16RegisterLen;j++)
	{
		RegData2[u16RegisterAdd] = MOS2_tModbusData.auReceiveBuff[3+2*j]*256+MOS2_tModbusData.auReceiveBuff[3+2*j+1];		
	}
}

/**
 * description: monitor command from master and response to master
 * 
 * parameter:void
 * 
 * return: void
 * 
 * 
 * */
void MOS2_vReadWriteDataCommon(void)
{

	if(MOS2_tModbusData.auReceiveBuff[0] == MOS2_tModbusPara.u8DeviceAddress)  //frame is to this slave? 
	{
		switch(MOS2_tModbusData.auReceiveBuff[1])  //function code analysis
		{

			case FUN_CODE_4://read data     
			{
				MOS2_vModbusFun4();
				break;
			}
			case FUN_CODE_6://write data     
			{
				MOS2_vModbudFun6();
				break;
			}

			
			default:
			{
				break;
			}				
		}
	}
}

/**
 * description modbus transition msg self 
 * 
 * parameter:uint16_t u16RegisterAddr
 * 
 * parameter:uint8_t u8Length
 * 
 * return: void
 * 
 * 
 * 
 * */
void MOS2_vReadRegister(uint16_t u16RegisterAddr, uint8_t u8Length)
{
  	uint16_t u16RegisterAdd = 0;
	uint16_t u16RegisterLen = 0;
	uint16_t i = 0;
	uint16_t j = 0;
	uint16_t u16TransmitCRC = 0;

	u16RegisterAdd=u16RegisterAddr;  //get register address to read
	u16RegisterLen=u8Length;  //register number
	MOS2_u8ReadRegisterAdd = u16RegisterAdd;
	
	MOS2_tModbusData.auTransmitBuff[i++]=MOS2_tModbusPara.u8DeviceAddress;//this device address
    MOS2_tModbusData.auTransmitBuff[i++]=0x04;        //function code      

	MOS2_tModbusData.auTransmitBuff[i++]=(uint8_t)(u16RegisterAddr/256);
	MOS2_tModbusData.auTransmitBuff[i++]=(uint8_t)(u16RegisterAddr%256);

	MOS2_tModbusData.auTransmitBuff[i++]=(uint8_t)(u16RegisterLen/256);
	MOS2_tModbusData.auTransmitBuff[i++]=(uint8_t)(u16RegisterLen%256);

	u16TransmitCRC=crc16(MOS2_tModbusData.auTransmitBuff,i);
	MOS2_tModbusData.auTransmitBuff[i++]=(uint8_t)(u16TransmitCRC/256);
	MOS2_tModbusData.auTransmitBuff[i++]=(uint8_t)(u16TransmitCRC%256);
	
	//MOS_vModbusTransmitEnalbe();  //Transmission enable
	
	for(j=0; j< i ; j++)
	{
	 UART_vWriteDataUART3(MOS2_tModbusData.auTransmitBuff[j]);
	}

	while(UART_bUARTTXBUFEMPTUART3()==RESET)
	{

	}

	//Enable TX
	//UART2->GCR = UART2->GCR|UART_GCR_TX;
	//Disable TX
	UART_vModbusTxDisableUART3();
	//MOS2_vModbusReceiveEnalbe();


}


/**
 * description modbus transition msg self 
 * 
 * parameter:uint16_t u16RegisterAddr
 * 
 * parameter:uint8_t u8Length
 * 
 * return: void
 * 
 * 
 * 
 * */
void MOS2_vWriteRegister(uint16_t u16RegisterAddr, uint16_t u16Content)
{
  	uint16_t u16RegisterAdd = 0;
	uint16_t i = 0;
	uint16_t j = 0;
	uint16_t u16TransmitCRC = 0;

	u16RegisterAdd=u16RegisterAddr;  //get register address to read
	
	MOS2_tModbusData.auTransmitBuff[i++]=MOS2_tModbusPara.u8DeviceAddress;//this device address
    MOS2_tModbusData.auTransmitBuff[i++]=0x06;        //function code      

	MOS2_tModbusData.auTransmitBuff[i++]=(uint8_t)(u16RegisterAddr/256);
	MOS2_tModbusData.auTransmitBuff[i++]=(uint8_t)(u16RegisterAddr%256);

	MOS2_tModbusData.auTransmitBuff[i++]=(uint8_t)(u16Content/256);
	MOS2_tModbusData.auTransmitBuff[i++]=(uint8_t)(u16Content%256);

	u16TransmitCRC=crc16(MOS2_tModbusData.auTransmitBuff,i);
	MOS2_tModbusData.auTransmitBuff[i++]=(uint8_t)(u16TransmitCRC/256);
	MOS2_tModbusData.auTransmitBuff[i++]=(uint8_t)(u16TransmitCRC%256);
	
	//MOS_vModbusTransmitEnalbe();  //Transmission enable
	
	for(j=0; j< i ; j++)
	{
	 UART_vWriteDataUART3(MOS2_tModbusData.auTransmitBuff[j]);
	}

	while(UART_bUARTTXBUFEMPTUART3()==RESET)
	{

	}

	//Enable TX
	//UART2->GCR = UART2->GCR|UART_GCR_TX;
	//Disable TX
	UART_vModbusTxDisableUART3();
	//MOS2_vModbusReceiveEnalbe();


}

/********************************************Interface function**********************************************/

/**
 * description: monitor command from master and response to master
 * 
 * parameter:void
 * 
 * return: void
 * 
 * 
 * */
uint8_t MOS2_u8ModBusRecHandle(void)
{
	uint8_t u8CommandType = 0;
	static uint16_t u16CalculateCRC = 0;
	static uint16_t u16ReceivedCRC = 0;

	uint8_t u8ReturnCode = E_REC_EMPTY_ERR;

	if(0 == MOS2_tModbusData.u8OneFrameFinish)  //no modbus frame
	{
		return E_REC_EMPTY_ERR;
	}


	else if(MOS2_vCheckReadAllDataCommand() == true)
	{
		u8CommandType = 2;
	}
	else if(MOS2_vCheckWriteDataCommand() == true)
	{
		u8CommandType = 3;
	}
	else
	{
		u8CommandType = 4;
	}

//Check CRC or Checksum
	if(u8CommandType == 2)//read data
	{
		u16CalculateCRC= crc16(&MOS2_tModbusData.auReceiveBuff[0], MOS2_READ_ALL_LEN);       //check crc code
		u16ReceivedCRC= MOS2_tModbusData.auReceiveBuff[MOS2_READ_ALL_LEN]*256;
		u16ReceivedCRC = u16ReceivedCRC  + MOS2_tModbusData.auReceiveBuff[MOS2_READ_ALL_LEN + 1];  //received crc code
		if(u16CalculateCRC == u16ReceivedCRC)
		{
            MOS2_vReadWriteDataCommon();
			u8ReturnCode = E_REC_COR_4;
		}
		else
		{
			u8ReturnCode = E_REC_CRC_ERR;
		}

	}

	if(u8CommandType ==3)//change parameters
	{
		u16CalculateCRC= crc16(&MOS2_tModbusData.auReceiveBuff[0], MOS2_WRITE_ALL_LEN);       //check crc code
		u16ReceivedCRC= MOS2_tModbusData.auReceiveBuff[MOS2_WRITE_ALL_LEN]*256;
		u16ReceivedCRC = u16ReceivedCRC  + MOS2_tModbusData.auReceiveBuff[MOS2_WRITE_ALL_LEN + 1];  //received crc code
		if(u16CalculateCRC == u16ReceivedCRC)
		{
            MOS2_vReadWriteDataCommon();
			u8ReturnCode = E_REC_COR_4;
		}
		else
		{
			u8ReturnCode = E_REC_CRC_ERR;
		}

	}

	MOS2_tModbusData.u8ReceivedByteCount = 0;   
    MOS2_tModbusData.u8OneFrameFinish = 0;	

	MOS2_vEventHandler();

	return u8ReturnCode;
}


void MOS2_vTimeoutInterrupt(void)
{
	if(MOS2_tModbusData.u8TimeoutStart != 0)
	{
		MOS2_tModbusData.u8TimeoutCount++; 
		if(MOS2_tModbusData.u8TimeoutCount>= MOS2_u8IntervalTime)  
		{
			MOS2_tModbusData.u8TimeoutStart=0;
			MOS2_tModbusData.u8OneFrameFinish=1;  
		}
	}
}

void MOS2_vDataReceiveInterrupt(void)
{

	uint8_t ucReceivedByte = 0;
	ucReceivedByte = UART_vGetUARTDataUART3();
	if(MOS2_tModbusData.u8OneFrameFinish ==1)  
	{
		return ;
	}

	MOS2_tModbusData.auReceiveBuff[MOS2_tModbusData.u8ReceivedByteCount ++] = ucReceivedByte;
	MOS2_tModbusData.u8TimeoutCount = 0;  //Clean timeout

	if(MOS2_tModbusData.u8ReceivedByteCount ==1)  
	{
		MOS2_tModbusData.u8TimeoutStart=1; 
	}
}

bool MOS2_bGetInitStatus(void)
{

	return MOS2_bInitFinished;

}




void MOS2_vGetTargetSpeed(void)
{
	MOS2_vReadRegister(DATA2_TARGET_SPD, 1);
}


void MOS2_vWriteCurrentSpeed(void)
{
	MOS2_vReadRegister(DATA2_ACTUAL_SPD, RegData2[DATA2_ACTUAL_SPD]);
}





