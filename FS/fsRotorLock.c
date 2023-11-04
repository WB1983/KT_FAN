#include "drv_inc.h"
#include "StateMachine.h"
#include "parameter.h"
#include "fsparameter.h"

#include "FOC_PMSM.h"
#include "Compile.h"
#include "Ramp.h"
#include "main.h"
#include "Current.h"
#include "Voltage.h"
#include "SpeedMonitor.h"		/* Include motor error module							*/
#include "fsCommon.h"
#include "errorReact.h"

#define FSF_NGISMV1101_LROTOR_ACTUAL_IMPEDANCE_TABLE	{																	\
				/*	speed								 	  	actual impedance */											\
					{0UL,								    		345UL},			/* 0. speed range 0 	- 359 	*/		\
					{360UL,								 			406UL},			/* 1. speed range 360 	- 787	*/		\
					{788UL,											487UL},			/* 2. speed range 788 	- 1215	*/		\
					{1216UL,										568UL},			/* 3. speed range 1216 	- 1643 	*/		\
					{1644UL,	   									644UL},			/* 4. speed range 1644 	- 2071 	*/		\
					{2072UL,										722UL},			/* 5. speed range 2072 	- 2285 	*/		\
					{2286UL,										761UL},			/* 6. speed range 2286 	- 2499	*/		\
					{2500UL,										800UL}			/* 7. Maximum speed 			*/		\
}

/**
 * [ms] Task handler time base
 * */
#define FRK_TASK_TIME_BASE_MS                (10)

/* CONVERSION of the config values to working values */

/**
 *  [SafeTime] Task handler time base
 * */
#define FRK_TASK_TIME_BASE                  STK_DEF_TIME_MS(FRK_TASK_TIME_BASE_MS)

#define FRK_LROTOR_MINBMF_CURRENT_RMS_BUFFER_SAMPLE_COUNTER  1024//origitnal value is 4096


typedef struct _TDataRotorLock
{
	int32_t tCurrentVectorBuff;
	int32_t tVoltageVectorBuff;
	int32_t tDCLinkVoltageBuff;
	int32_t tCurrentVectorRMS;
	int32_t tVoltageVectorRMS;
	int32_t tDCLinkVoltageRMS;
	int32_t tBemfVectorBuff;
	int32_t tBemfVectorAVG;
	int16_t   u16MinBEMF;
	int16_t   u16VDC_80PERCENT;
	TSafeTime tTaskEntryTime;
	TSafeTime tLastTimeSliceTime;
	int16_t   u16ErrorCounter;
	TThreePhase tThreePhaseCur;
	TThreePhase tThreePhaseVol;
	int16_t s16ActualSpeed;
		
}TDataRotorLock;


typedef struct _TParamRotorLock
{
	uint16_t  u16MinBEMFRatioHigh;//Q12
	uint16_t  u16MinBEMFRatioLow;//Q12
	uint16_t  u16MinBEMFSwitchSpd;
 	uint16_t  u16MaxErrorCount;
		
}TParamRotorLock;


/** \brief    Locked rotor - Actual impedance LOOKUP table data type
 *
 *  \details
 */
typedef struct _TFSF_MINBMF_ActImpTBL {
	uint32_t					Speed_rpm;						/* Speed range with the assigned impedance		*/
	uint32_t					ActualImpedance;				/* Actual impedance valid for the speed range	*/
}TFSF_MINBMF_ActImpTBL;



static TDataRotorLock FRK_tDataRotorLock;

static TParamRotorLock FRK_tParaRotorLock = {FSF_LROTOR_MIN_BEMF_PER_SPEED_RATIO_Q12_HS, FSF_LROTOR_MIN_BEMF_PER_SPEED_RATIO_Q12_LS,FSF_LROTOR_MIN_BEMF_PER_SPEED_SWITCH_SPEED ,80};

static TFSF_MINBMF_ActImpTBL	ActualImpedanceTable[8U] = FSF_NGISMV1101_LROTOR_ACTUAL_IMPEDANCE_TABLE;

static uint32_t FRK_u32CurrentImpedance = 9800;//568UL;

#define FRK_SIMPLE_ROTOR_LOCK_MOR 1
/**********************************************simple solution***********************************************/
typedef struct _TSimpleRotorLockData
{
	int32_t s32BEMFPerSpeedRatio;
	int32_t BemfMag;
	bool    bBemfCheckState;
	int32_t BemfMagVirtual;
	
}TSimpleRotorLockData;

typedef struct _TSimpleRotorLockPara
{
	int32_t s32BEMFLimitVal;	

	
}TSimpleRotorLockPara;

TSimpleRotorLockData FRK_tSimpleRotorLockData = {0,0,0,0};
TSimpleRotorLockPara FRK_tSimpleRotorLockPara = {300};

/***************************************************************************************/

static uint32_t FSF_LROTOR_getActualImpedance(void)
{

	const uint32_t Speed_rpm = Motor_1st.FOC.SMO.s16Speed;
	const uint8_t LastIndex = (uint8_t)((uint8_t)(sizeof(ActualImpedanceTable) / sizeof(ActualImpedanceTable[0U])) - 1U);
	uint8_t FieldIndex;
	int32_t XValue = 0L;
	int32_t XValueNext = 1L;
	int32_t YValue = 0L;
	int32_t YValueNext = 1L;
	uint32_t Ret;
	int32_t TempVal;

	/* Check maximum speed */
	if(Speed_rpm >= ActualImpedanceTable[LastIndex].Speed_rpm)
	{
		Ret = ActualImpedanceTable[LastIndex].ActualImpedance;
	}
	else
	{
		/* Search the matching index in the LUT */
		for (FieldIndex = 0U; FieldIndex < LastIndex; FieldIndex++)
		{
			if((Speed_rpm >= ActualImpedanceTable[FieldIndex].Speed_rpm)
			&& (Speed_rpm < ActualImpedanceTable[FieldIndex + 1U].Speed_rpm))
			{
				YValue = (int32_t)ActualImpedanceTable[FieldIndex].ActualImpedance;
				YValueNext = (int32_t)ActualImpedanceTable[FieldIndex + 1U].ActualImpedance;
				XValue = (int32_t)ActualImpedanceTable[FieldIndex].Speed_rpm;
				XValueNext = (int32_t)ActualImpedanceTable[FieldIndex + 1U].Speed_rpm;
				break;// Matching index has been found - terminate for loop => stop searching and continue interpolation

			}
		}

		/* Interpolate between two points in the LUT ==> Yinterpol = dx * DeltaY/DeltaX + Y0 */
		TempVal = (int32_t)Speed_rpm - XValue;			/* dx */
		TempVal *= (YValueNext - YValue);				/* DeltaY */
		if(XValueNext != XValue)						/* Avoid division by 0 */
		{
			TempVal /= (XValueNext - XValue);			/* DeltaX */
		}
		TempVal += YValue;								/* Y0 */

		Ret = (uint32_t)TempVal;						/* Assign the gained result to return value */
	}

	return Ret;
}


void FRK_vcalcCurrentVector(void)
{

	TThreePhaseCur * tThreePhaseCurrent = NULL;
	tThreePhaseCurrent = &FRK_tDataRotorLock.tThreePhaseCur;
	CUR_vGetThreePhaseCur(tThreePhaseCurrent);


	/* get positive value only */
	if(tThreePhaseCurrent->a < 0)
	{
		tThreePhaseCurrent->a *= (-1L);
	}

	if(tThreePhaseCurrent->b < 0)
	{
		tThreePhaseCurrent->b *= (-1L);
	}

	if(tThreePhaseCurrent->c < 0)
	{
		tThreePhaseCurrent->c *= (-1L);
	}

	/* Calc current vector AVG */
	FRK_tDataRotorLock.tCurrentVectorBuff -= Division(FRK_tDataRotorLock.tCurrentVectorBuff,FRK_LROTOR_MINBMF_CURRENT_RMS_BUFFER_SAMPLE_COUNTER);

	FRK_tDataRotorLock.tCurrentVectorBuff += (tThreePhaseCurrent->a + tThreePhaseCurrent->b +tThreePhaseCurrent->c);

	FRK_tDataRotorLock.tCurrentVectorRMS = Division(FRK_tDataRotorLock.tCurrentVectorBuff,FRK_LROTOR_MINBMF_CURRENT_RMS_BUFFER_SAMPLE_COUNTER);
	

	/* calculate AVG */
	FRK_tDataRotorLock.tDCLinkVoltageBuff -= Division(FRK_tDataRotorLock.tDCLinkVoltageBuff,FRK_LROTOR_MINBMF_CURRENT_RMS_BUFFER_SAMPLE_COUNTER);

	FRK_tDataRotorLock.tDCLinkVoltageBuff += Motor_1st.FOC.s16VbusAvg;
	
	FRK_tDataRotorLock.tDCLinkVoltageRMS = Division(FRK_tDataRotorLock.tDCLinkVoltageBuff,FRK_LROTOR_MINBMF_CURRENT_RMS_BUFFER_SAMPLE_COUNTER);
}


void FRK_vcalcVoltageVector(void)
{

    TThreePhase * tPhaseVoltage = NULL;
	tPhaseVoltage = &FRK_tDataRotorLock.tThreePhaseVol;
	FCN_vGetThreePhaseVol(tPhaseVoltage);

	/* get positive value only */
	if(tPhaseVoltage->a < 0)
	{
		tPhaseVoltage->a = tPhaseVoltage->a * (-1L);
	}

	if(tPhaseVoltage->b < 0)
	{
		tPhaseVoltage->b  = tPhaseVoltage->b  * (-1L);
	}

	if(tPhaseVoltage->c < 0L)
	{
		tPhaseVoltage->c = tPhaseVoltage->c * (-1L);
	}

	FRK_tDataRotorLock.tVoltageVectorBuff -= Division(FRK_tDataRotorLock.tVoltageVectorBuff,FRK_LROTOR_MINBMF_CURRENT_RMS_BUFFER_SAMPLE_COUNTER);

	FRK_tDataRotorLock.tVoltageVectorBuff += tPhaseVoltage->a + tPhaseVoltage->b + tPhaseVoltage->c;

	FRK_tDataRotorLock.tVoltageVectorRMS = Division(FRK_tDataRotorLock.tVoltageVectorBuff,FRK_LROTOR_MINBMF_CURRENT_RMS_BUFFER_SAMPLE_COUNTER);

}

void FRK_vcalcBemf(void)
{
	int32_t ActualBEMFSample = 0;
	if((eM1_MainState == MainState_Run)&&(eM1_RunSubState == RunState_Spin))//after close loop
		{
			FRK_vcalcCurrentVector();
			FRK_vcalcVoltageVector();

			/* BEMF calculation */
			ActualBEMFSample = FRK_tDataRotorLock.tVoltageVectorRMS;								/* Calc BEMF via formula Ubemf = Vs - Is*Z */
			ActualBEMFSample -= MLIB_Mul_Q15(FRK_tDataRotorLock.tCurrentVectorRMS, FRK_u32CurrentImpedance);

			
			FRK_tDataRotorLock.tBemfVectorBuff -= Division(FRK_tDataRotorLock.tBemfVectorBuff,FRK_LROTOR_MINBMF_CURRENT_RMS_BUFFER_SAMPLE_COUNTER);

			FRK_tDataRotorLock.tBemfVectorBuff += ActualBEMFSample;

			FRK_tDataRotorLock.tBemfVectorAVG = Division(FRK_tDataRotorLock.tBemfVectorBuff,FRK_LROTOR_MINBMF_CURRENT_RMS_BUFFER_SAMPLE_COUNTER);
			
			if(FRK_tDataRotorLock.tBemfVectorAVG < 0)
			{
				FRK_tDataRotorLock.tBemfVectorAVG = 0;
			}
		}

}

void FRK_vLockedRotorEvaluation(void)
{

#if (FS_MOTOR_PHASE_LOSS_SIMPLE == OPTION_ACTIVE)

	
#else

	FRK_vcalcBemf();

#endif
}

void FRK_vCheckMotorLockComplex(void)
{
	if((eM1_MainState == MainState_Run)&&(eM1_RunSubState == RunState_Spin))//after close loop
	{
	    //FSF_LROTOR_getActualImpedance();
		FRK_tDataRotorLock.tTaskEntryTime = STK_tGetSafeTime();
		if ((FRK_tDataRotorLock.tTaskEntryTime - FRK_tDataRotorLock.tLastTimeSliceTime) >= FRK_TASK_TIME_BASE)
		{
			FRK_tDataRotorLock.tLastTimeSliceTime = FRK_tDataRotorLock.tTaskEntryTime;

			FRK_tDataRotorLock.u16VDC_80PERCENT = Motor_1st.FOC.s16VbusAvg*8/10;

			FRK_tParaRotorLock.u16MinBEMFSwitchSpd = Q15(FSF_LROTOR_MIN_SUPPORTED_SPEED/SPEED_MAX);

			FRK_tDataRotorLock.s16ActualSpeed = (Motor_1st.FOC.s16SpdFilt*SPEED_MAX)/32768;
				
			if(Motor_1st.FOC.SMO.s16Speed > FRK_tParaRotorLock.u16MinBEMFSwitchSpd)
				{
					FRK_tDataRotorLock.u16MinBEMF = (FRK_tDataRotorLock.s16ActualSpeed*FRK_tParaRotorLock.u16MinBEMFRatioHigh)/1000;
				}
			else
				{
					FRK_tDataRotorLock.u16MinBEMF = (FRK_tDataRotorLock.s16ActualSpeed*FRK_tParaRotorLock.u16MinBEMFRatioLow)/1000;

				}
		
			if(FRK_tDataRotorLock.u16MinBEMF > FRK_tDataRotorLock.u16VDC_80PERCENT)			 
			{
				FRK_tDataRotorLock.u16MinBEMF = FRK_tDataRotorLock.u16VDC_80PERCENT;
			}

			//don't consider braking status
			if(FRK_tDataRotorLock.tBemfVectorAVG  < FRK_tDataRotorLock.u16MinBEMF)
				{
				
					FRK_tDataRotorLock.u16ErrorCounter ++;

				}
			else
				{
					if(FRK_tDataRotorLock.u16ErrorCounter > 0)
						{
							FRK_tDataRotorLock.u16ErrorCounter --;
						}
				}

			if(FRK_tDataRotorLock.u16ErrorCounter > FRK_tParaRotorLock.u16MaxErrorCount)
				{
					//throw error
					ERT_vErrorReport(MC_FAULT_ROTOR_BLOCKED);
				}

			}
	}
}

/*-------------------------------------------------------------------------------------------------
 *	BOOL VEC_bMonitorLockedRotor(const TMotObj* ptMot)
 *	Checks rotor locked condition
 *------------------------------------------------------------------------------------------------- */

BOOL FRK_bCheckLockedRotorSimple(void)
{
	/*
	 * Locked rotor condition is only checked while the motor control is
	 * in synchronized mode.
	 * */
	if((eM1_MainState == MainState_Run)&&(eM1_RunSubState == RunState_Spin))//after close loop
	{
	    int32_t s32CalcTempVal = 0;
		s32CalcTempVal = Motor_1st.FOC.SMO.s16Ealpha_est*Motor_1st.FOC.SMO.s16Ealpha_est + Motor_1st.FOC.SMO.s16Ebeta_est*Motor_1st.FOC.SMO.s16Ebeta_est;
		FRK_tSimpleRotorLockData.BemfMag = Hw_Sqrt(s32CalcTempVal);
		
 		FRK_tSimpleRotorLockData.BemfMagVirtual = FRK_tSimpleRotorLockData.BemfMag*SPEED_MAX;
			
		/* Check speed depending EMF value */
		FRK_tSimpleRotorLockData.s32BEMFPerSpeedRatio = Division(FRK_tSimpleRotorLockData.BemfMagVirtual, Motor_1st.FOC.s16SpdFilt);
																											/* As motor runs in Tq mode 'tFreqTargetQ' is not reliable source of information */
		if(Motor_1st.USER.s16VspCmd > 0)
		{
			if (FRK_tSimpleRotorLockData.s32BEMFPerSpeedRatio < FRK_tSimpleRotorLockPara.s32BEMFLimitVal)
			{
				/*
				 * Control lost synchronization
				 * => throw error code
				 * */
				if ((eM1_MainState == MainState_Run)&&(eM1_RunSubState == RunState_Spin))//close loop
				{
					//ERM_vSetMotorError(ptMot, ERM_MOT_CONTROL_LOST_SYNCH);
					FRK_tSimpleRotorLockData.bBemfCheckState = FALSE;
					ERT_vErrorReport(MC_FAULT_ROTOR_BLOCKED);
					//FSV_vInitModelCalculation();
				}
			}
		}
		else
		{
			if (FRK_tSimpleRotorLockData.s32BEMFPerSpeedRatio > FRK_tSimpleRotorLockPara.s32BEMFLimitVal)
			{
				/*
				 * Control lost synchronization
				 * => throw error code
				 * */
				if ((eM1_MainState == MainState_Run)&&(eM1_RunSubState == RunState_Spin) )
				{
					ERT_vErrorReport( MC_FAULT_ROTOR_BLOCKED);
				}
			}
		}
	}
	return true;
}

void FRK_vCheckMotorLock(void)
{
#if (FS_MOTOR_PHASE_LOSS_SIMPLE == OPTION_ACTIVE)

	FRK_bCheckLockedRotorSimple();

#else

	FRK_vCheckMotorLockComplex();

#endif

}


