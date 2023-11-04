/*
 * @Author: Wang Bing
 * @Date: 2021-09-06 12:44:09
 * @LastEditTime: 2023-10-30 13:44:05
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
#include "drv_inc.h"
#include "hal_uart.h"
#include "ModbusCRC.h"
#include "mm32_it.h"

/****************************************************************************************
*                          declaration                                                  *
 ***************************************************************************************/ 
extern void MOS_vTriggerEvent(uint8_t u8RegisterAddrss);
extern void MOS_vEventHandler(void);
extern void MOS_vRegisterHandler(uint8_t RegisterID);
extern void MOS_vConfigureSlaveAddress(void);
extern void MOS_vConfigurePassword(void);
extern void MOS_vModbusProtocalConfig(void);
/****************************************************************************************
*                          Variables                                                    *
 ***************************************************************************************/ 
TModbusData MOS_tModbusData                     = {{0},0,0,0,{0},0,0};
static TModbusPara MOS_tModbusPara              = {0,{0,0},0,0,0};
static uint16_t MOS_u8EventRegister             = 0;
static uint8_t  MOS_u8IntervalTime              = BR_INTERVAL_9600;

static bool     MOS_bInitFinished               = false;
// static uint8_t MOS_auDataTransmitBuff[50];
/*
bandrate is 9600,
one bit time is 1000000us/9600bit/s = 104us, one byte is 104*10 = 1040us.
Modbus confirm frame finish time is >3.5*1040us = 3.64ms, set it to 10ms.
*/


/********************************water level test protocal for customer******************************/
typedef struct _TFixPart
{
	uint8_t  u8FrameHeader;
	uint8_t u16CMDAttributionHigh;
	uint8_t u16CMDAttributionLow;
	uint8_t  u8FixByte;
}TFixPart;


typedef struct _THostCommand
{
	TFixPart tFixPart;
	uint8_t  u8CtrlCmd;
	uint8_t  u8Address;
	uint8_t u16DataLenHigh;
	uint8_t u16DataLenLow;
	uint8_t  u8CheckSum;
	uint8_t  u8EndCod;
}THostCommand;


typedef struct _TSlaveResponse
{
	TFixPart tFixPart;
	uint8_t  u8CtrlCmd;
	uint8_t  u8Address;
	uint8_t u16DataLenHigh;
	uint8_t u16DataLenLow;
	uint8_t  u8ActWorkStatus;
	uint8_t  u8BallStatus;
	uint8_t u16WaterLevelHigh;
	uint8_t u16WaterLevelLow;
	uint8_t u8FixData[7];
	uint8_t  u8CheckSum;
	uint8_t  u8EndCod;
}TSlaveResponse;


#define MOS_FRAME_HEADER          0x68        //fixed frame header
#define MOS_COMMAND_ATTRIBUTION   0x66      //fixed attributon
#define MOS_FIXED_BYTE            0x68        //fixed byte
#define MOS_CONTROL_CMD           0x4B        //control command
#define MOS_RESPONSE_CMD          0x8B        //slave response
#define MOS_END_CODE              0x16

#define MOS_UPDATE_LEN            71

#define MOS_READ_ALL_LEN          6

#define MOS_WRITE_ALL_LEN          6

#define MOS_GET_DATA               8

#define MOS_TRANS_LEN             21

#define MOS_PRE_DEF_SUM         0x23B

#define MOS_DEEP_VALUE        300//cm
#define MOS_ORIGINAL_VALUE    0x01
#define MOS_TARGET_VALUE      0x02

uint8_t MOS_aucReadAllData[] = {0x01,0x04,0x00,0x00,0x00,0x01,0x00,0x00};

uint8_t MOS_aucWriteData[] = {0x01,0x06,0x00,0x00,0x00,0x01,0x00,0x00};
//static uint8_t ucTestinc;

uint16_t Reg[]={      //Parameter
                0x0000,   
                0x0000,
                0x0000,
                0x0000,
                0x0000,
                0x0000,
                0x0000,
                0x0000,
                0x0000,
                0x0000,
                0x0000,
								0x0000,		
            };

uint16_t RegData[] = {        //Data
                0x0002,
                0x0000,
                0x0000,
                0x0000,
                0x0000,
                0x0000,
                0x0000,
                0x0000,
                0x0000,
                0x0000,
                0x0000,
                0x0000,
            };

const uint8_t MOS_auModebusIntervalTime[7] = {BR_INTERVAL_1200, BR_INTERVAL_2400, BR_INTERVAL_4800, BR_INTERVAL_9600, BR_INTERVAL_19200,BR_INTERVAL_38400,BR_INTERVAL_DEFAULT};
/*************************************water level test protocalend**********************************************/


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
void MOS_vTriggerEvent(uint8_t u8RegisterAddrss)
{
	uint16_t u16RegisterEvent = 0x0001;
	u16RegisterEvent = u16RegisterEvent<<u8RegisterAddrss;
	MOS_u8EventRegister = MOS_u8EventRegister|u16RegisterEvent;
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
void MOS_vEventHandler(void)
{
	uint16_t u16RegisterIndex = 0x0001;
	uint8_t i = 0;
	for(i=0; i< 16; i++)
	{
		u16RegisterIndex = 0x0001;
		u16RegisterIndex = u16RegisterIndex<<i;

		if((MOS_u8EventRegister & u16RegisterIndex) != 0)
		{
			MOS_u8EventRegister = MOS_u8EventRegister&(~u16RegisterIndex);
			MOS_vRegisterHandler(i);
			break;
		}

	}
}

void MOS_vWriteDataIntoEEP(void)
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
void MOS_vSlaveAddressUpdate(void)
{

    MOS_vWriteDataIntoEEP();
	RegData[DATA_SLAVE_ADD] = 0x0001;

}


void MOS_vCallbackInit(void)
{

		/*Register callback function*/
	STK_vSetGenericTimerCallbackFunc(MOS_vTimeoutInterrupt);
	UART_vSetUartReceiveCallbackFunc(MOS_vDataReceiveInterrupt);

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
void MOS_vRegisterHandler(uint8_t u8RegisterID)
{
	switch(u8RegisterID)
	{
		case DATA_MODE_SELECT:
		{

			break;
		}

		case DATA_TARGET_SPD:
		{

			break;
		}

		case DATA_SLAVE_ADD:
		{
			MOS_vSlaveAddressUpdate();
			break;
		}

		case DATA_RESERVED2:
		{
			
			break;
		}

		case DATA_RESERVED3:
		{
	
			break;
		}

		case DATA_RESERVED4:
		{

			break;
		}
		case DATA_RESERVED5:
		{

			break;
		}
		case DATA_RESERVED6:
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
void MOS_vModbusInit(void)
{
	MOS_vCallbackInit();

	MOS_vModbusProtocalConfig();//brandrate configure

	MOS_vConfigureSlaveAddress();

	MOS_tModbusData.u8TimeoutStart = 0; //MODbus timer stop

	MOS_bInitFinished = true;

}

/**
 * description��modbus receiving control
 * 
 * parameter:void
 * 
 * return: void
 * 
 * 
 * */
/*void MOS_vModbusReceiveEnalbe(void)
{

}
*/

/**
 * description��modbus transtion control
 * 
 * parameter:void
 * 
 * return: void
 * 
 * 
 * */
void MOS_vModbusTransmitEnalbe(void)
{

}


void MOS_vModbusProtocalConfig(void)
{
	RegData[DATA_SLAVE_ADD] = 0x01;

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
void MOS_vConfigureSlaveAddress(void)
{
	MOS_tModbusPara.u8DeviceAddress = RegData[DATA_SLAVE_ADD];
}



/**
 * @description: interface to transmit temperatuer information message.
 * @param {*}
 * @return {*}
 */
/*void MOS_vRegularMsgTrans(void)
{
	MOS_vTransmitMessage(DATA_PRESSURE, 1);
}
*/

/***********************************************New Protocal for Customer************************************************/
bool MOS_vCheckWriteDataCommand(void)
{
	uint8_t bReturnValue = false;
	if((MOS_tModbusData.auReceiveBuff[1] == MOS_aucWriteData[1])&&(MOS_tModbusData.auReceiveBuff[1] == 0x06))//check function code
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
void MOS_vModbudFun6(void)  //function code 6
{
  	uint16_t u16RegisterAdd = 0;
	uint16_t u16value = 0;
	uint16_t i = 0;
	uint16_t u16TransmitCRC = 0;
	uint16_t j = 0;

  	u16RegisterAdd=MOS_tModbusData.auReceiveBuff[2]*256+MOS_tModbusData.auReceiveBuff[3];  //get address to modify 
	u16value=MOS_tModbusData.auReceiveBuff[4]*256+MOS_tModbusData.auReceiveBuff[5];     //the value to be write
	RegData[u16RegisterAdd]=u16value;  //update registers

	MOS_vTriggerEvent(u16RegisterAdd);//trigger handler

	//slave return
	
	MOS_tModbusData.auTransmitBuff[i++]=MOS_tModbusPara.u8DeviceAddress;//slave address
  	MOS_tModbusData.auTransmitBuff[i++]=0x06;        //function code 
  	MOS_tModbusData.auTransmitBuff[i++]=(uint8_t)(u16RegisterAdd/256);
	MOS_tModbusData.auTransmitBuff[i++]=(uint8_t)(u16RegisterAdd%256);
	MOS_tModbusData.auTransmitBuff[i++]=(uint8_t)(u16value/256);
	MOS_tModbusData.auTransmitBuff[i++]=(uint8_t)(u16value%256);
	u16TransmitCRC=crc16(MOS_tModbusData.auTransmitBuff,i);
	MOS_tModbusData.auTransmitBuff[i++]=(uint8_t)(u16TransmitCRC/256);  //
	MOS_tModbusData.auTransmitBuff[i++]=(uint8_t)(u16TransmitCRC%256);
	
	//MOS_vModbusTransmitEnalbe();  //set to receive state
	
	for(j=0; j< i ; j++)
	{
	 UART_vWriteData(MOS_tModbusData.auTransmitBuff[j]);
	}

	while(UART_bUARTTXBUFEMPT()==RESET)
	{

	}
	//Enable TX
	//UART2->GCR = UART2->GCR|UART_GCR_TX;
	//Diable TX
	UART_vModbusTxDisable();
	//MOS_vModbusReceiveEnalbe();
}


bool MOS_vCheckReadAllDataCommand(void)
{
	uint8_t bReturnValue = false;
	if((MOS_tModbusData.auReceiveBuff[1] == MOS_aucReadAllData[1])&&(MOS_tModbusData.auReceiveBuff[1] == 0x04))//check function code
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
void MOS_vModbusFun4(void)
{
  	uint16_t u16RegisterAdd = 0;
	uint16_t u16RegisterLen = 0;
	uint16_t u16TransDataLen = 0;
	uint16_t i = 0;
	uint16_t j = 0;
	uint16_t u16TransmitCRC = 0;

	u16RegisterAdd=MOS_tModbusData.auReceiveBuff[2]*256+MOS_tModbusData.auReceiveBuff[3];  //get register address
	u16RegisterLen=MOS_tModbusData.auReceiveBuff[4]*256+MOS_tModbusData.auReceiveBuff[5];  //get Nr. of register
	
	MOS_tModbusData.auTransmitBuff[i++]=MOS_tModbusPara.u8DeviceAddress;//slave address
    MOS_tModbusData.auTransmitBuff[i++]=0x04;        //function code      
  	u16TransDataLen=u16RegisterLen*2;   //byte length = number*2

	MOS_tModbusData.auTransmitBuff[i++]=(uint8_t)(u16TransDataLen%256);
	
	for(j=0;j<u16RegisterLen;j++)
	{
        MOS_tModbusData.auTransmitBuff[i++]=(uint8_t)(RegData[u16RegisterAdd+j]/256);
		MOS_tModbusData.auTransmitBuff[i++]=(uint8_t)(RegData[u16RegisterAdd+j]%256);		
	}
	u16TransmitCRC=crc16(MOS_tModbusData.auTransmitBuff,i);
	MOS_tModbusData.auTransmitBuff[i++]=(uint8_t)(u16TransmitCRC/256);
	MOS_tModbusData.auTransmitBuff[i++]=(uint8_t)(u16TransmitCRC%256);
	
	//MOS_vModbusTransmitEnalbe();  //set to receive state
	
	for(j=0; j< i ; j++)
	{
	 UART_vWriteData(MOS_tModbusData.auTransmitBuff[j]);
	
	}

	while(UART_bUARTTXBUFEMPT()==RESET)
	{

	}

	//Enable TX
	//UART2->GCR = UART2->GCR|UART_GCR_TX;
	//Disable TX
	UART_vModbusTxDisable();
	//MOS_vModbusReceiveEnalbe();
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
void MOS_vReadWriteDataCommon(void)
{

	if(MOS_tModbusData.auReceiveBuff[0] == MOS_tModbusPara.u8DeviceAddress)  //frame is to this slave? 
	{
		switch(MOS_tModbusData.auReceiveBuff[1])  //function code analysis
		{

			case 4:     
			{
				MOS_vModbusFun4();
				break;
			}
			case 6:     
			{
				MOS_vModbudFun6();
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
 * description: monitor command from master and response to master
 * 
 * parameter:void
 * 
 * return: void
 * 
 * 
 * */
void MOS_vModBusHandler(void)
{
	uint8_t u8CommandType = 0;
	static uint16_t u16CalculateCRC = 0;
	static uint16_t u16ReceivedCRC = 0;

	uint16_t u16CalculateCheckSum;
	uint16_t u16ReceivedCheckSum;


	if(0 == MOS_tModbusData.u8OneFrameFinish)  //no modbus frame
	{
		return;
	}


	else if(MOS_vCheckReadAllDataCommand() == true)
	{
		u8CommandType = 2;
	}
	else if(MOS_vCheckWriteDataCommand() == true)
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
		u16CalculateCRC= crc16(&MOS_tModbusData.auReceiveBuff[0], MOS_READ_ALL_LEN);       //check crc code
		u16ReceivedCRC= MOS_tModbusData.auReceiveBuff[MOS_READ_ALL_LEN]*256;
		u16ReceivedCRC = u16ReceivedCRC  + MOS_tModbusData.auReceiveBuff[MOS_READ_ALL_LEN + 1];  //received crc code
		if(u16CalculateCRC == u16ReceivedCRC)
		{
            MOS_vReadWriteDataCommon();
		}

	}

	if(u8CommandType ==3)//change parameters
	{
		u16CalculateCRC= crc16(&MOS_tModbusData.auReceiveBuff[0], MOS_WRITE_ALL_LEN);       //check crc code
		u16ReceivedCRC= MOS_tModbusData.auReceiveBuff[MOS_WRITE_ALL_LEN]*256;
		u16ReceivedCRC = u16ReceivedCRC  + MOS_tModbusData.auReceiveBuff[MOS_WRITE_ALL_LEN + 1];  //received crc code
		if(u16CalculateCRC == u16ReceivedCRC)
		{
            MOS_vReadWriteDataCommon();
		}

	}

	MOS_tModbusData.u8ReceivedByteCount = 0;   
  MOS_tModbusData.u8OneFrameFinish = 0;	

	MOS_vEventHandler();
}


void MOS_vTimeoutInterrupt(void)
{
	if(MOS_tModbusData.u8TimeoutStart != 0)
	{
		MOS_tModbusData.u8TimeoutCount++; 
		if(MOS_tModbusData.u8TimeoutCount>= MOS_u8IntervalTime)  
		{
			MOS_tModbusData.u8TimeoutStart=0;
			MOS_tModbusData.u8OneFrameFinish=1;  
		}
	}
}

void MOS_vDataReceiveInterrupt(void)
{

	uint8_t ucReceivedByte = 0;
	ucReceivedByte = UART_vGetUARTData();
	if(MOS_tModbusData.u8OneFrameFinish ==1)  
	{
		return ;
	}

	MOS_tModbusData.auReceiveBuff[MOS_tModbusData.u8ReceivedByteCount ++] = ucReceivedByte;
	MOS_tModbusData.u8TimeoutCount = 0;  //Clean timeout

	if(MOS_tModbusData.u8ReceivedByteCount ==1)  
	{
		MOS_tModbusData.u8TimeoutStart=1; 
	}
}

bool MOS_bGetInitStatus(void)
{

	return MOS_bInitFinished;

}





