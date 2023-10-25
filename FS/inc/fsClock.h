#ifndef __FSCLOCK_H
#define __FSCLOCK_H

#include "parameter.h"

/********************************************************************************************************************/
/* GLOBAL TYPES *****************************************************************************************************/
/********************************************************************************************************************/
typedef unsigned long long TSafeTime;			/**< Type of the safe timestamps */


/********************************************************************************************************************/
/* GLOBAL MACROS ****************************************************************************************************/
/********************************************************************************************************************/
/**
	\brief	Resolution of the reference system time base in [100ns] units
			Resolution is here 100us because the system time is triggered by the motor
			PWM timer interrupt (PWM = 10kHz   ->  one tick = 1/10kHz = 1000/10us).
*/
#define STK_TIME_RESOLUTION_100NS       1000UL

/**
	\brief	Support macro that can be used to define times in [ms] unit
	\remark
*/
#define STK_DEF_TIME_MS(Time_ms)        ((TSafeTime)((PWMFREQ * (Time_ms)) / STK_TIME_RESOLUTION_100NS))//10KHZ

/********************************************************************************************************************/
/* GLOBAL FUNCTIONS DECLARATIONS ************************************************************************************/
/********************************************************************************************************************/
/**
	\brief	STK_vTriggerSafeTime() function trigger the reference system time counter in tick units
			This function has to be triggered by a cyclical timer interrupt. This
			timer interrupt is generated for each PWM pulse.
	\return	void
*/
extern void STK_vTriggerSafeTime(void);

/**
	\brief	STK_tGetSafeTime() function is the interface to the reference system time
	\return	TSafeTime Actual reference system time value in ticks
*/
extern TSafeTime STK_tGetSafeTime(void);

/**
	\brief	Interface to initialize the STK module
	\return	void
	\note	System timer was already configured to calibrate  LSI in 'FMS_bCalcClockRatio()'
			for clock compare but now comfigured
*/
extern void STK_vInitSafeTime(void);

extern void FCK_vInitSafeTime(void);




#endif
