#ifndef __APP_STATEMACHINE_H
#define __APP_STATEMACHINE_H

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

#include "stateMachineCom.h"


#define MOS2_EVENT_STARTUP        0
#define MOS2_EVENT_TRANS_MSG      1
#define MOS2_EVENT_RECEP_MSG      2
#define MOS2_EVENT_RECEP_SUC      3
#define MOS2_EVENT_RECEP_TIMOUT   4
#define MOS2_EVENT_RECEP_ERR      5
#define MOS2_EVENT_ERROR_FIN      6


#define MOS2_INIT   0
#define MOS2_TRANS  1
#define MOS2_RECEP  2
#define MOS2_ERROR  3
#define MOS2_IDLE   4


extern void MOS2_vInit_entry(void); 
extern void MOS2_vInit_do(void); 
extern void MOS2_vInit_exit(void);
extern void MOS2_vTrans_entry(void);
extern void MOS2_vTrans_do(void);
extern void MOS2_vTrans_exit(void);
extern void MOS2_vRecep_entry(void);
extern void MOS2_vRecep_do(void);
extern void MOS2_vRecep_exit(void);
extern void MOS2_vError_entry(void);
extern void MOS2_vError_do(void);
extern void MOS2_vError_exit(void);
extern void MOS2_vIdle_entry(void);
extern void MOS2_vIdle_do(void);
extern void MOS2_vIdle_exit(void);
		

extern void AMS_vModbus2Statemachine(void);	

#endif

