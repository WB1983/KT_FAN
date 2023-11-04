/*
 * @Author: Wang Bing
 * @Date: 2021-09-06 12:44:09
 * @LastEditTime: 2023-03-16 00:18:54
 * @LastEditors: Wangbingbin bing.wang2@bshg.com
 * @Description: header file of the modbus
 * @FilePath: \VE_CPM_CP_SMM_SW_S50PUc:\Users\WangBingBin\AppData\Local\WangBing\Projects\EXTERNAL\Pressure\Pressure_221206_16F18346.X\Application\Logic\Modbus\Modbus.h
 */

#ifndef _MODBUS2_H
#define _MODBUS2_H

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

#define MOS2_CRC_LEN                2
#define MOS2_WR_NR                  5




extern uint16_t RegData2[];

extern const uint8_t MOS2_auModebusIntervalTime[7];

typedef enum _MOS2_DATA_REGISTER_ENUM
{
    DATA2_SLAVE_ADD,
    DATA2_TARGET_SPD,//read
    DATA2_ACTUAL_SPD,//read
    DATA2_RESERVED1,
    DATA2_RESERVED2,
    DATA2_RESERVED3,

}MOS2_DATA_REGISTER_ENUM;

typedef enum _E_MODBUS2STATE
{
	E_MODBUS2_INIT,
	E_MODBUS2_TRANS,
	E_MODBUS2_REC,
	E_MODBUS2_ERR,
	E_MODBUS2_IDLE,

}E_MODBUS2STATE;

typedef enum _E_MODBUS2RECSTATE
{
	E_REC_EMPTY_ERR,
	E_REC_CRC_ERR,
	E_REC_COR_4,
	E_REC_COR_6,
}E_MODBUS2RECSTATE;
	


extern void MOS2_vModbusInit(void);

extern void MOS2_vGetTargetSpeed(void);

extern void MOS2_vWriteCurrentSpeed(void);

extern uint8_t MOS2_u8ModBusRecHandle(void);

extern void MOS2_vTimeoutInterrupt(void);

extern void MOS2_vDataReceiveInterrupt(void);


#endif


