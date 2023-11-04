/*
 * @Author: Wang Bing
 * @Date: 2021-09-06 12:44:09
 * @LastEditTime: 2023-03-16 00:18:54
 * @LastEditors: Wangbingbin bing.wang2@bshg.com
 * @Description: header file of the modbus
 * @FilePath: \VE_CPM_CP_SMM_SW_S50PUc:\Users\WangBingBin\AppData\Local\WangBing\Projects\EXTERNAL\Pressure\Pressure_221206_16F18346.X\Application\Logic\Modbus\Modbus.h
 */

#ifndef _MODBUS
#define _MODBUS

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

#define MOS_CRC_LEN                2
#define MOS_BROADCAST_ADD       0xFF
#define MOS_CONFIRM_PWD       0x08AE
#define MOS_WR_NR                  5

extern uint16_t Reg[];
extern uint16_t RegData[];
/*
typedef enum _REGISTER_ENUM
{
    // LOCK_VALUE,//pressure value
    // RESERVED1,
    // RESERVED2,
    // FILTER_VALUE,//Filter value
    // COM_ADD_VALUE,//communcaiton address value
    // COM_FORMAT_VALUE,//communication format value
    // RESERVED3,
    // //DATA_MODE_SWITCH,
    // WORK_MODE,
    // RESERVED4, 
}REGISTER_ENUM;
*/
typedef enum _E_FUNCTION_CODE
{
	FUN_CODE_4 = 4,
	FUN_CODE_6 = 6,


}E_FUNCTION_CODE;


typedef enum _BANDRATE_LIST
{
    BR_1200,
    BR_2400,
    BR_4800,
    BR_9600,
    BR_19200,
    BR_38400,
    BR_DEFAULT
}BANDRATE_LIST;

typedef enum _BANDRATE_MODEBUS_INTERVAL
{
    BR_INTERVAL_1200 = 64,
    BR_INTERVAL_2400 = 32,
    BR_INTERVAL_4800 = 16,
    BR_INTERVAL_9600 = 8,
    BR_INTERVAL_19200 = 4,
    BR_INTERVAL_38400 = 2,
    BR_INTERVAL_DEFAULT = 4
}BANDRATE_MODEBUS_INTERVAL;

typedef enum _eTEMP_UPDATE_INTERVAL
{
    UPDATE_INTERVAL_1S = 1,
    UPDATE_INTERVAL_2S,
    UPDATE_INTERVAL_3S,
    UPDATE_INTERVAL_4S,
    UPDATE_INTERVAL_5S,
    UPDATE_INTERVAL_10S = 10,
}eTEMP_UPDATE_INTERVAL;


extern const uint8_t MOS_auModebusIntervalTime[7];

typedef struct _TModbusData
{
    uint8_t auReceiveBuff[20];
    uint8_t u8OneFrameFinish;
    uint8_t u8ReceivedByteCount;
    uint8_t u8TimeoutStart;
    uint8_t auTransmitBuff[20];
    uint8_t u8TimeoutCount;//uint is ms
    uint8_t u8OneFrameThresholdTime;
}TModbusData;

typedef struct _TModbusConfigure
{
    uint8_t u8ModbusBandrate;
    uint8_t u8ModbusFrameFormat;
}TModbusConfigure;

typedef struct _TModbusPara
{
    uint8_t u8DeviceAddress;
    TModbusConfigure tModbusConfigure;
    uint8_t u8FilterValue;
    uint16_t u16LockValue;
    uint8_t u8PressueMeasMode;

}TModbusPara;

typedef void (* EEPWriteCallbackFunc)(void);

typedef struct _TEEPREG
{
    uint16_t u16RegAdd;
    uint16_t u16EEPAdd;
    EEPWriteCallbackFunc ptCallBack;
}TEEPREG;

typedef enum _DATA_REGISTER_ENUM
{
    // DATA_FIL_PRE,
    // DATA_INPUT_ANA_VALUE,
    // DATA_INPUT_DIGI_VALUE,//Input value digital
    // DATA_VOUT_VALUE,
    // DATA_TEMP_VALUE,
    // DATA_PRESSURE,
    // DATA_VOUT_CALI,
    // DATA_DIGITAL_CALI,
    // DATA_VOUT_PRESSURE,
    // DATA_FIL_FAC_DIG,
    // DATA_FIL_FAC_ANA,
    // DATA_MODE_SWITCH
    DATA_MODE_SELECT,//read
    DATA_TARGET_SPD,//read
    DATA_ACTUAL_SPD,//read
    DATA_POWER,//read
    DATA_SLAVE_ADD,
    DATA_RESERVED2,
    DATA_RESERVED3,
    DATA_RESERVED4,
    DATA_RESERVED5,
    DATA_RESERVED6,
    DATA_RESERVED7,
    DATA_RESERVED8,
}DATA_REGISTER_ENUM;

// typedef enum _FRAME_CONFIGURE
// {
//     FRAME_MODE1 = 0x00, //N,8,1
//     FRAME_MODE2 = 0x01, //N,8,2
//     FRAME_MODE3 = 0x02, //E,8,1
//     FRAME_MODE4 = 0x03  //O,8,1
// }FRAME_CONFIGURE;

extern void MOS_vModBusHandler(void);

extern TModbusData * MOS_PtGetModbusObject(void);

//extern void MOS_vModbusReceiveEnalbe(void);

extern void MOS_vModbusTransmitEnalbe(void);

extern void MOS_vModbusInit(void);

extern void MOS_vConfigureBandrate(void);

extern uint8_t * MOS_puGetModbusIntervalTime(void);

extern void MOS_vTransmitMessage(uint16_t u16RegisterAddr, uint8_t u8Length);

extern uint8_t MOS_vSetEvenParityCheck(uint8_t u8CheckType, uint8_t u8InputData);

extern void MOS_vModBusHandler(void);

extern void MOS_vTimeoutInterrupt(void);

extern void MOS_vDataReceiveInterrupt(void);

extern bool MOS_bGetInitStatus(void);

#endif

