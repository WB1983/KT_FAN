#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

#include "ErrorHandle.h"
#include "SIMEEP.h"
#include "drv_inc.h"

#define EHE_BYTE_CNT            4


int32_t EHE_u32ErrorIDMark    = 0;
int32_t EHE_u32PreErrorIDMark = 0;

typedef enum _E_ERRORTASKID
{
	E_ERROR_INIT,
	E_ERROR_RUN,	
}E_ERRORTASKID;


TERRORDATA EHE_tErrorData;

static uint8_t * EHE_pu8Buffer = NULL;

void EHE_vErrorHandleTask(void)
{
	
	switch(EHE_tErrorData.u8TaskState)
		{
			case E_ERROR_INIT:
				{
					//load data from flash
					EHE_pu8Buffer = (uint8_t *)&EHE_tErrorData.u32ErrorIDMark;
					EEPROM_Read(EHE_pu8Buffer,EHE_BYTE_CNT);
					//EHE_tErrorData.u32ErrorIDMark = *(uint32_t *)EHE_pu8Buffer;
					EHE_tErrorData.u32PreErrorIDMark = EHE_tErrorData.u32ErrorIDMark;
					EHE_tErrorData.u8TaskState = E_ERROR_RUN;
					break;

				}
			case E_ERROR_RUN:
				{
					if(EHE_tErrorData.u32ErrorIDMark != EHE_tErrorData.u32PreErrorIDMark)
					{
						EHE_tErrorData.u32PreErrorIDMark = EHE_tErrorData.u32ErrorIDMark;

						//write to Flash
						EHE_pu8Buffer = (uint8_t *)&EHE_tErrorData.u32ErrorIDMark;					
						SMP_vWriteWordN(EHE_pu8Buffer, EHE_BYTE_CNT);				
					}
					break;

				}
			
			default:
				{
				}


		}


}

void EHE_vSetErrorCode(uint32_t u32ErrorCode)
{
	EHE_tErrorData.u32PreErrorIDMark |=u32ErrorCode;

}

void EHE_vResetErrorCode(uint32_t u32ErrorCode)
{
	EHE_tErrorData.u32PreErrorIDMark &=(~u32ErrorCode);

}



