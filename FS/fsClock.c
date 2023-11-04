#include "fsClock.h"
#include "libdefines.h"
#include "fsMCU.h"
#include "fsParameter.h"
#include "common.h"
#include "ErrorReact.h"

static TSafeTime	STK_tSafeTime				= 0;			///< Safe time counter
static uint32_t		STK_u32SystickOld			= 0;			///< Variable holds System timer value till next FST_vTriggerSafeTime() call
																///< to calculate difference
static uint32_t		STK_u32SystickDifference	= 0;			///< difference of the System timer between two FST_vTriggerSafeTime() calls
static uint32_t		STK_u32WaitSystemInit		= FSP_FST_NBR_OF_INIT_TICKS;			///< wait few tickes till system get stable after reset/waking up
static uint32_t		STK_u32NbrOfTicksErrorCheck	= FSP_FST_NBR_OF_TICKS_ERROR_CHECK;		///< check error every 10 mili seconds
static uint32_t		STK_u32TickTimeErrorCounter	= 0;			///< Time latency error counter

/*****************************************************************************************************************
 * GLOBAL FUNCTIONS **********************************************************************************************
 *****************************************************************************************************************/
/*
* simulated system tick
*/
void STK_vTriggerSafeTime(void)
{
	// compute tick time
	if(FMC_u32GetSystickTimerCounter() < STK_u32SystickOld)
	{
		STK_u32SystickDifference = 0xFFFF - STK_u32SystickOld + FMC_u32GetSystickTimerCounter();
	}
	else
	{
		STK_u32SystickDifference	= (0xFFFF & (FMC_u32GetSystickTimerCounter() - STK_u32SystickOld));
	}
	
	STK_u32SystickOld			= FMC_u32GetSystickTimerCounter();		// remember current time for next tick time check

	if(STK_u32WaitSystemInit > 0)
	{
		// wait till filter get stable
		STK_u32WaitSystemInit--;
	}
	else
	{
		// check time difference for limits
		if(STK_u32SystickDifference > FSP_FST_MAX_TIME_DIFF_LIMIT)
		{
			STK_u32TickTimeErrorCounter++;
		}
		if(STK_u32SystickDifference < FSP_FST_MIN_TIME_DIFF_LIMIT)
		{
			STK_u32TickTimeErrorCounter++;
		}
	}

	if(STK_u32NbrOfTicksErrorCheck > 0)		// do error check over 'STK_u32NbrOfTicksErrorCheck' ticks, delay(filter) 100*100 = 10ms.
	{
		STK_u32NbrOfTicksErrorCheck--;
	}
	else
	{
		if(STK_u32TickTimeErrorCounter > FSP_FST_WRONG_TICK_TIME_NBR_LIMIT)
		{
         //Error, implement corresponding action.interrupt is not implmented on time!
         //Turn off the IPM output
		 ERT_vErrorReport(ERT_FAUYLT_SAFE_TIME_OUTOF_RANGE);
		}
		else
		{
			STK_u32NbrOfTicksErrorCheck	= FSP_FST_NBR_OF_TICKS_ERROR_CHECK;
			STK_u32TickTimeErrorCounter = 0;
		}
	}
	STK_tSafeTime++;	// increment motor control tick time. 100us
}

TSafeTime STK_tGetSafeTime(void)
{
	return STK_tSafeTime;
}

void FCK_vInitSafeTime(void)
{
	STK_u32WaitSystemInit		= FSP_FST_NBR_OF_INIT_TICKS;
	STK_u32NbrOfTicksErrorCheck	= FSP_FST_NBR_OF_TICKS_ERROR_CHECK;
	FMC_vSetSystickTimerRegisters();
}


