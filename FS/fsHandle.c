#include "SpeedMonitor.h"
#include "fsRotorLock.h"
#include "fsPhaseloss.h"
#include "fsCommon.h"

typedef enum _ECALLID
{
	E_SPEED_MONITOR = 0,
	E_MOTOR_BLOCK,
	E_PHASE_LOSS,
	E_TOTAL_PHASE
}ECALLID;

ECALLID FHE_eCurrentCallId = E_SPEED_MONITOR;


void FHE_vHandleTask(void)
{
	FHE_eCurrentCallId ++;
	if(FHE_eCurrentCallId >= E_TOTAL_PHASE)
	{
		FHE_eCurrentCallId = E_SPEED_MONITOR;
	}
	else
	{

	}

#if(FS_COMMON_ENABLE == OPTION_ACTIVE)
	FCN_vGetPhaseVoltage();
#endif

#if(FS_SPEED_MONITOR == OPTION_ACTIVE)
	if(FHE_eCurrentCallId == E_SPEED_MONITOR)
	{
		SMR_vSpeedEvaluation();
	}
#endif

#if(FS_MOTOR_BLOCK == OPTION_ACTIVE)
	if(FHE_eCurrentCallId == E_MOTOR_BLOCK)
	{
		FRK_vLockedRotorEvaluation();
	}
#endif

#if(FS_MOTOR_PHASE_LOSS == OPTION_ACTIVE)
	if(FHE_eCurrentCallId == E_PHASE_LOSS)
	{
		FPL_vPhaseLossEvaluation();
	}
#endif




}


/**
Handle in the interrupt


**/
void FHE_vFunctionSafetyCheck(void)
{
#if(FS_SPEED_MONITOR == OPTION_ACTIVE)
	SMR_vCheckSpeedMonitor();	
#endif

#if(FS_MOTOR_BLOCK == OPTION_ACTIVE)
	FRK_vCheckMotorLock();
#endif

#if(FS_MOTOR_PHASE_LOSS == OPTION_ACTIVE)
	FPL_vPhaseLossCheck();
#endif


}

