#ifndef __ERROR_REACT_H
#define __ERROR_REACT_H

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

/***********************************definition*******************************************/
//Level 1
#define MC_FAULT_I_DCBUS_OVER                1  //can recovery to right value, but need keep 30 back to previous state
#define MC_FAULT_U_DCBUS_UNDER               2  //can recovery to right value, but need keep 30 back to previous state
#define MC_FAULT_U_DCBUS_OVER                3  //can recovery to right value, but need keep 30 back to previous state

//Level 2
#define MC_FAULT_LOAD_OVER                   4  //NR
#define MC_FAULT_SPEED_OVER                  5  //NR
#define MC_FAULT_ROTOR_BLOCKED               6  //can't recovery after 60 seconds. turn on motor again to try.
#define MC_FAULT_PHASELESS                   7 	//can't recovery after 60 seconds. turn on motor again to try.
#define MC_FAULT_SPEED_INFOR                 8  //can't recovery after 60 seconds. turn on motor again to try.
#define MC_FAULT_STARTUP_LOSS_SYN            9  //can't recovery after 60 seconds. turn on motor again to try.

//Level 1
#define ERT_FAUYLT_SAFE_TIME_OUTOF_RANGE     10
#define ERT_FAUYLT_15V_VOL_HIGH              11//can recovery to right value, but need keep 30 back to previous state
#define ERT_FAUYLT_INT_REF_VOL_HIGH          12//can recovery to right value, but need keep 30 back to previous state
#define ERT_FAUYLT_15V_VOL_LOW               13//can recovery to right value, but need keep 30 back to previous state
#define ERT_FAUYLT_INT_REF_VOL_LOW           14//can recovery to right value, but need keep 30 back to previous state


//Level 2
#define ERT_FAUYLT_NTC_TEMP_HIGH             15//can recovery to right value, but need keep 30 back to previous state
#define ERT_FAUYLT_MCU_TEMP_HIGH             16//can recovery to right value, but need keep 30 back to previous state
#define ERT_FAUYLT_MCU_TEMP_LOW              17//can recovery to right value, but need keep 30 back to previous state

//Level 3
#define MCU_ERT_FAUYLT_RAM_TEST_CALLEE       18//keep to current state, then reset?          
#define MCU_ERT_FAUYLT_RAM_TEST_COMPLETED    19
#define MCU_ERT_FAUYLT_SYSTICK_INIT_CALLEE   20
#define MCU_ERT_FAUYLT_CLOCK_TEST_CALLER     21
#define MCU_ERT_FAUYLT_CPU_TEST_CALLER       22
#define MCU_ERT_FAUYLT_STACK_OVERFLOW_TEST   23
#define MCU_ERT_FAUYLT_DELTA_MAIN            24
#define MCU_ERT_FAUYLT_FLASH_TEST_CALLER     25
#define MCU_ERT_FAUYLT_LAST_DELTA_MAIN       26

//Level 3
#define MCU_ERT_FAUYLT_WDG_TEST_CALLER       27                  
#define MCU_ERT_FAUYLT_CRC32_TEST_CALLER     28
#define MCU_ERT_FAUYLT_CHECKPOINT1           29
#define MCU_ERT_FAUYLT_CHECKPOINT2           30

#define ERT_TOTAL_ERROR_NR                   30


//MCU Error


typedef enum _EERRORTYPE
{
	E_ERROR_LEVEL_1 = 1,//after short time, it can recover.
	E_ERROR_LEVEL_2,//after long time, it can recover.
	E_ERROR_LEVEL_3,//don't enter main loop. stay in current loop.
	
}EERRORTYPE;

typedef struct _TERRORRECORD
{
	uint64_t u64ErrorIDMark;
	uint64_t u64RecordInfor;
}TERRORRECORD;

typedef struct _TERRORREAD
{
	uint64_t u64ReadOutErrorDebug;
	uint64_t u64ReadOutInforDebug;
}TERRORREAD;



typedef struct _TERRORData
{
	uint8_t      u8TaskState;
	uint8_t      u8PriorityErrorID;
	uint64_t     u64PreErrorIDMark;
	uint8_t      u8ErrorCnt;
	bool         bRecoveryEnable;
	
	TERRORRECORD tErrorRecord;
	
	TERRORREAD   tErrorRead;
}TERRORDATA;

typedef struct _TERRORTABLEELEMENT
{
	uint32_t u8ErrorIDMASK;
	uint8_t u8ErrorType;
}TERRORTABLEELEMENT;


//Level 1
#define MC_FAULT_I_DCBUS_OVER_MASK                0x0000000000000001  /* OverCurrent fault flag */
#define MC_FAULT_U_DCBUS_UNDER_MASK               0x0000000000000002 /* Undervoltage fault flag */
#define MC_FAULT_U_DCBUS_OVER_MASK                0x0000000000000004/* Overvoltage fault flag */

//Level 2
#define MC_FAULT_LOAD_OVER_MASK                   0x0000000000000008   /* Overload fault flag */
#define MC_FAULT_SPEED_OVER_MASK                  0x0000000000000010   /* Over speed fault flag */
#define MC_FAULT_ROTOR_BLOCKED_MASK               0x0000000000000020/* Blocked rotor fault flag */
#define MC_FAULT_PHASELESS_MASK                   0x0000000000000040			/* Phasee less fault flag */
#define MC_FAULT_SPEED_INFOR_MASK                 0x0000000000000080  /* Speed inforamtion error */
#define MC_FAULT_STARTUP_LOSS_SYN_MASK            0x0000000000000100/* under startup Lost synchronization error */

//Level 1
#define ERT_SAFE_TIME_OUTOF_RANGE_MASK            0x0000000000000200
#define ERT_15V_VOL_HIGH_MASK                     0x0000000000000400
#define ERT_INT_REF_VOL_HIGH_MASK                 0x0000000000000800
#define ERT_15V_VOL_LOW_MASK               		  0x0000000000001000
#define ERT_INT_REF_VOL_LOW_MASK           		  0x0000000000002000

//Level 1
#define ERT_NTC_TEMP_HIGH_MASK                    0x0000000000004000
#define ERT_MCU_TEMP_HIGH_MASK                    0x0000000000008000
#define ERT_MCU_TEMP_LOW_MASK                     0x0000000000010000 

//Level 3
#define MCU_ERT_FAUYLT_RAM_TEST_CALLEE_MASK       0x0000000000020000          
#define MCU_ERT_FAUYLT_RAM_TEST_COMPLETED_MASK    0x0000000000040000
#define MCU_ERT_FAUYLT_SYSTICK_INIT_CALLEE_MASK   0x0000000000080000
#define MCU_ERT_FAUYLT_CLOCK_TEST_CALLER_MASK     0x0000000000100000
#define MCU_ERT_FAUYLT_CPU_TEST_CALLER_MASK       0x0000000000200000
#define MCU_ERT_FAUYLT_STACK_OVERFLOW_TEST_MASK   0x0000000000400000
#define MCU_ERT_FAUYLT_DELTA_MAIN_MASK            0x0000000000800000
#define MCU_ERT_FAUYLT_FLASH_TEST_CALLER_MASK     0x0000000001000000
#define MCU_ERT_FAUYLT_LAST_DELTA_MAIN_MASK       0x0000000002000000

//Level 3
#define MCU_ERT_FAUYLT_WDG_TEST_CALLER_MASK       0x0000000004000000              
#define MCU_ERT_FAUYLT_CRC32_TEST_CALLER_MASK     0x0000000008000000
#define MCU_ERT_FAUYLT_CHECKPOINT1_MASK           0x0000000010000000
#define MCU_ERT_FAUYLT_CHECKPOINT2_MASK           0x0000000020000000


extern void ERT_vErrorCodeAccess(void);

extern void ERT_vSetErrorCode(uint32_t u32ErrorCode);

extern void ERT_vResetErrorCode(uint32_t u32ErrorCode);

extern void ERT_vErrorLEDDisplay(void);

extern void ERT_vErrorReaction(void);

extern TERRORDATA * ERT_tGetErrorCode(void);

extern void ERT_vErrorReport(uint8_t u8ErrorCode);

extern void ERT_vErrorReactionRecovery(void);

extern void ERT_vReadErrorCode(void);

extern void ERT_vResetErrorCount(void);


#endif
