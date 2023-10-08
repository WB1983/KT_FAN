#ifndef __ERRORHANDLE_H
#define __ERRORHANDLE_H

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

extern void EHE_vWriteError(uint16_t u16ErrorID);

extern uint16_t EHE_vReadLastError(void);

extern uint8_t * EHE_vReadErrorN(uint8_t ErrorNumber);


#endif
