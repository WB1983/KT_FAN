#ifndef __ERRORHANDLE_H
#define __ERRORHANDLE_H

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

typedef struct _TERRORData
{
	uint8_t  u8TaskState;
	uint32_t u32ErrorIDMark;
	uint32_t u32PreErrorIDMark;

}TERRORDATA;

#define EHE_SAFE_TIME_OUTOF_RANGE     0x80000000
#define EHE_15V_VOL_HIGH              0x04000000
#define EHE_INT_REF_VOL_HIGH          0x00200000
#define EHE_NTC_TEMP_HIGH             0x00010000
#define EHE_MCU_TEMP_HIGH             0x00008000
#define EHE_MCU_TEMP_LOW              0x00000400


extern TERRORDATA EHE_tErrorData;

extern void EHE_vErrorHandleTask(void);

extern void EHE_vSetErrorCode(uint32_t u32ErrorCode);

extern void EHE_vResetErrorCode(uint32_t u32ErrorCode);



#endif
