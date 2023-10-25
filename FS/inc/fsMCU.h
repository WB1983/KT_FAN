#ifndef __FSMCU_H
#define __FSMCU_H
#include "fsParameter.h"
#include "LibDefines.h"
#include "Math.h"
//#include "systick.h"
/**
 * \brief Type definition of programmable voltage detector
 */
typedef enum _TPVDEnable{		//enable/disable pre-programmable voltage detector
	DISABLE_PVD,
	ENABLE_PVD
}TPVDEnable;

/**
 * \brief	Type definition of programmable voltage detector
 */
typedef enum _TPVDLocked{
	UNLOCK_PVD,					// O INTERRUPT FROM TIM 1/8/15/16/17 DISCONNECTED
	LOCK_PVD					// O INTERRUPT FROM TIM 1/8/15/16/17 CONNECTED
}TPVDLocked;



/**
 * \brief	Type definition of clock sources for clock compare function
 */
typedef enum _TClockSourceType {
	HSI_CLK,
	HSE_CLK,
	NO_OF_CLK
}TClockSourceType;
	

/**
 * \brief	Type definition of clock sources for systick clock source
 */
typedef enum _TSystickClockSource {
	EXTERNAL_CLOCK,
	PROCESSOR_CLOCK,
	NO_OF_CLOCK
}TSystickClockSource;

//extern void FMC_vRTCConfig(void);
//extern void FMC_vClockSourceCompare(TClockSourceType TClock);
//extern void FMC_vInitIWWDG(void);
//extern void FMC_vPVDInit(void);
extern void FMC_vSetSystickTimerRegisters();
extern void FMC_vEnDisAbleSystickTimer(bool bEnDisAbleTimer);
extern uint32_t FMC_u32GetSystickTimerCounter(void);



#endif

