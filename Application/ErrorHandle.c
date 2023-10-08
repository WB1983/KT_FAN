#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

#include "ErrorHandler.h"
#include "SIMEEP.h"
#include "drv.eep.h"

void EHE_vWriteError(uint16_t u16ErrorID)
{

	SMP_vWriteWord(&u16ErrorID);


}


uint16_t EHE_vReadLastError(void)
{
    uint16_t u16LastErrorId;
	
	SMP_vReadWord(&u16LastErrorId);

	return u16LastErrorId;

}

uint8_t * EHE_vReadErrorN(uint8_t ErrorNumber)
{
    uint8_t pu8Buffer[2*ErrorNumber] = 0;
	
	EEPROM_Read(pu8Buffer,2*ErrorNumber);

	return pu8Buffer[0];

}


