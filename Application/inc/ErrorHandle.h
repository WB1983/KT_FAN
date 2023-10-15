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

extern TERRORDATA EHE_tErrorData;

extern void EHE_vErrorHandleTask(void);

extern void EHE_vSetErrorCode(uint32_t u32ErrorCode);

extern void EHE_vResetErrorCode(uint32_t u32ErrorCode);



#endif
