#include "libdefines.h"
#include "fsMCU.h"
#include "Math.h"
#include "fsParameter.h"
#include "Drv_Counter.h"
/********************************************************************************************************************/
/* LOCAL FUNCTION PROTOTYPES ****************************************************************************************/
/********************************************************************************************************************/
//static BOOL FMC_bCalcClockRatio(TClockSourceType FMC_TClockType);

/********************************************************************************************************************/
/* LOCAL VARIABLES **************************************************************************************************/
/********************************************************************************************************************/

/********************************************************************************************************************/
/* LOCAL FUNCTION DEFINITION ****************************************************************************************/
/********************************************************************************************************************/
/* System timer section section (start)																	*/
/**
 * \brief	Systick timer configuration
 * 			Systick timer serve to obtain clock of MCU core, to compare
 * 			with other clock source (LSI).
 */
void FMC_vSetSystickTimerRegisters(void)
	{
		Drv_Tim16_Counter_Init(DRV_PWM_PRESCALER, DRV_PWM_PERIOD);
	}

/*
 * \brief	Systick timer enable interface
 *
 * \param	BOOL TRUE - enabled, FALSE - disabled.
 * \return	void
 */
void FMC_vEnDisAbleSystickTimer(bool bEnDisAbleTimer)
	{

		Drv_Tim16_EnDisable(bEnDisAbleTimer);

	}

uint32_t FMC_u32GetSystickTimerCounter(void)
{
	return Drv_u32GetTim16CountVal();
}

