#include "drv_inc.h"
#include "StateMachine.h"
#include "parameter.h"
#include "FOC_PMSM.h"
#include "Compile.h"
#include "Ramp.h"
#include "main.h"
#include "Current.h"
#include "Voltage.h"
#include "SpeedMonitor.h"
#include "fsCommon.h"
#include "errorReact.h"

#define PHASEL_MPHW_AVG_BUFFER_SAMPLE_COUNTER 16384

#define RMP_INCREASE_CUR_HI                    Q15(0.5/I_MAX)
#define RMP_INCREASE_CUR_LO                    Q15(0.4/I_MAX)


#define RMP_ADC_CURRENTHIGHLIM				(2040)		/* adc value limits for detecting zero current */
#define RMP_ADC_CURRENTLOWLIM				(2000)		/* adc value limits for detecting zero current */


#define RMP_PHASE_OPEN_FILTER				(5)	

/**
 * [ms] Task handler time base
 * */
#define FPL_TASK_TIME_BASE_MS                (10)

/* CONVERSION of the config values to working values */

/**
 *  [SafeTime] Task handler time base
 * */
#define FPL_TASK_TIME_BASE              STK_DEF_TIME_MS(FPL_TASK_TIME_BASE_MS)


static TFp RMP_tPhaseOpenPresentCount      = 0;

typedef struct _TDataPhaseLoss
{
	int32_t PhaseABuffCur;
	int32_t PhaseBBuffCur;
	int32_t PhaseCBuffCur;
	int32_t PhaseABuffVolt;
	int32_t PhaseBBuffVolt;
	int32_t PhaseCBuffVolt;

	int32_t tBemfVectorBuff;
	int32_t tBemfVectorAVG;
	int16_t u16MinBEMF;
	int16_t u16VDC_80PERCENT;
	TSafeTime tTaskEntryTime;
	TSafeTime tLastTimeSliceTime;
	int16_t u16ErrorCounter;

	TThreePhaseCur tThreePhaseCur;
	TThreePhaseCur tThreePhaseVol;

	TThreePhaseCur tThreePhaseCurRMS;
	TThreePhaseCur tThreePhaseVolRMS;

	bool bStaticVectorFlag;//true, positive/negative not ballance
	bool bVoltageAsymmetryFlag;//false, not symmetry
	bool bCurrentAsymmetryFlag;//false, not symmetry
	bool bAsymmetrySampleValid;
	int32_t s32MaxRuntimeDiffVolt;
	int32_t s32MaxRuntimeDiffCurr;
	bool bPhaseCorrupted;
	bool bPhaseLost;
	bool bPhaseAsymmetrical;//true, phase not symmetry
	uint16_t u16MaxRuntimeErrorCounter;


}TDataPhaseLoss;

typedef struct _TParaPhaseLoss
{
	int32_t s32VoltAsymmetryThreshold;
	int32_t s32CurrAsymmetryThreshold;
	int32_t s32CurrentNoiseFilter;
	int32_t s32VoltageNoiseFilter;
	uint16_t u16ErrorCntLimit;
}TParaPhaseLoss;

static TDataPhaseLoss FPL_tPhaseLossData;
static TParaPhaseLoss FPL_tPhaseLossPara = {20,20,600,400,700};//voltage about 5V, Current about 60mA. 700*10=7 seconds, to detected phase lose.

/*************************************************Simple Solution*******************************************************/
typedef struct _TPhaseLossDataSimple
{
	TThreePhaseCur tThreePhaseCurRaw;
	TThreePhaseCur tThreePhaseCurRawOffset;
	uint16_t       s16OverCurrentRawHi;
	uint16_t       s16OverCurrentRawLo;

}TPhaseLossDataSimple;

typedef struct _TPhaseLossParaSimple
{
	uint16_t      s16CurrentRawHigh;
	uint16_t      s16CurrentRawLow;

}TPhaseLossParaSimple;


static TPhaseLossDataSimple FPL_tPhaseLossDataSimple = {{0,0,0},0,0};
static TPhaseLossParaSimple FPL_tPhaseLossParaSimple = {RMP_INCREASE_CUR_HI,RMP_INCREASE_CUR_LO};

static TCurBufferCal * FPL_ptCurBufferCal = NULL;

/************************************static function**********************************************/

/** \brief    Evaluate if a motor phase current asymmetry is present.
 *
 *  \details  Detects the asymmetry by comparing of current rms  values.
 *
 *  \param    TFSF_MPHW_Data* ptPHASEL_Data
 *  \param    const TFSF_MPHW_Drive_Para* ptPHASEL_Para
 *
 *  \return   bool
 */

static bool FPL_vEvaluateStaticVector(void)
{
	bool ReturnValue = false;

	uint32_t Temp;
	uint32_t positive;
	uint32_t negative;

	bool isPhaseA_static = false;
	bool isPhaseB_static = false;
	bool isPhaseC_static = false;

	FPL_ptCurBufferCal = CUR_tGetPhaseCurrentCalPointer();

	positive = FPL_ptCurBufferCal->buffer_IaP_Avg;
	negative = FPL_ptCurBufferCal->buffer_IaN_Avg;

	Temp = (positive + negative) / 2UL;
	/* get 75% value */
	positive = (positive / 2UL) + (positive / 4UL);
	negative = (negative / 2UL) + (negative / 4UL);

	if((Temp < positive) || (Temp < negative))
	{
		isPhaseA_static = true;
	}

	positive = FPL_ptCurBufferCal->buffer_IbP_Avg;
	negative = FPL_ptCurBufferCal->buffer_IbN_Avg;


	Temp = (positive + negative) / 2UL;
	/* get 75% value */
	positive = (positive / 2UL) + (positive / 4UL);
	negative = (negative / 2UL) + (negative / 4UL);

	if((Temp < positive) || (Temp < negative))
	{
		isPhaseB_static = true;
	}

	positive = FPL_ptCurBufferCal->buffer_IcP_Avg;
	negative = FPL_ptCurBufferCal->buffer_IcN_Avg;


	Temp = (positive + negative) / 2UL;
	/* get 75% value */
	positive = (positive / 2UL) + (positive / 4UL);
	negative = (negative / 2UL) + (negative / 4UL);

	if((Temp < positive) || (Temp < negative))
	{
		isPhaseC_static = true;
	}

	/* 2 phases need to be valid, to avoid 0 degree vector */
	if((isPhaseA_static == true) && (isPhaseB_static == true))
	{
		ReturnValue = true;
	}
	/* 2 phases need to be valid, to avoid 0 degree vector */
	if((isPhaseB_static == true) && (isPhaseC_static == true))
	{
		ReturnValue = true;
	}
	/* 2 phases need to be valid, to avoid 0 degree vector */
	if((isPhaseC_static == true) && (isPhaseA_static == true))
	{
		ReturnValue = true;
	}

	return ReturnValue;
}


static bool FPL_vEvaluateVoltageAsymmetry(void)
{
	int32_t difference;
	int32_t threshold;

	int32_t diff_A;
	int32_t	diff_B;

	bool isVoltageSymmetrical = true;

	/* get static vector run threshold */
	threshold = FPL_tPhaseLossPara.s32VoltAsymmetryThreshold;

	/* calculate the percentage value */
	diff_A = FPL_tPhaseLossData.tThreePhaseVolRMS.a;
	diff_B = FPL_tPhaseLossData.tThreePhaseVolRMS.b;
	if(diff_A > diff_B)
	{
		difference = diff_A - diff_B;
		difference = (difference * 100L) / diff_A;
	}
	else
	{
		difference = diff_B - diff_A;
		difference = (difference * 100L) / diff_B;
	}

	if(difference > threshold)
	{
		isVoltageSymmetrical = false;
	}

	if(difference > FPL_tPhaseLossData.s32MaxRuntimeDiffVolt)
	{
		FPL_tPhaseLossData.s32MaxRuntimeDiffVolt = difference;
	}

	/* calculate the percentage value */
	diff_A = FPL_tPhaseLossData.tThreePhaseVolRMS.b;
	diff_B = FPL_tPhaseLossData.tThreePhaseVolRMS.c;
	if(diff_A > diff_B)
	{
		difference = diff_A - diff_B;
		difference = (difference * 100L) / diff_A;
	}
	else
	{
		difference = diff_B - diff_A;
		difference = (difference * 100L) / diff_B;
	}

	if(difference > threshold)
	{
		isVoltageSymmetrical = false;
	}

	if(difference > FPL_tPhaseLossData.s32MaxRuntimeDiffVolt)
	{
		FPL_tPhaseLossData.s32MaxRuntimeDiffVolt = difference;
	}

	/* calculate the percentage value */
	diff_A = FPL_tPhaseLossData.tThreePhaseVolRMS.c;
	diff_B = FPL_tPhaseLossData.tThreePhaseVolRMS.a;
	if(diff_A > diff_B)
	{
		difference = diff_A - diff_B;
		difference = (difference * 100L) / diff_A;
	}
	else
	{
		difference = diff_B - diff_A;
		difference = (difference * 100L) / diff_B;
	}

	if(difference > threshold)
	{
		isVoltageSymmetrical = false;
	}

	if(difference > FPL_tPhaseLossData.s32MaxRuntimeDiffVolt)
	{
		FPL_tPhaseLossData.s32MaxRuntimeDiffVolt = difference;
	}

	return isVoltageSymmetrical;
}


/** \brief    Evaluate if a motor phase current asymmetry is present.
 *
 *  \details  Detects the asymmetry by comparing of current rms  values.
 *
 *  \param    TFSF_MPHW_Data* ptPHASEL_Data
 *  \param    const TFSF_MPHW_Drive_Para* ptPHASEL_Para
 *
 *  \return   bool
 */
static bool FPL_vEvaluateCurrentAsymmetry(void)
{
	int32_t difference;
	int32_t threshold;

	int32_t diff_A;
	int32_t	diff_B;

	int32_t	phA;
	int32_t	phB;
	int32_t	phC;

	bool isCurrentSymmetrical = true;

	/* get asymmetry threshold */
	threshold = FPL_tPhaseLossPara.s32CurrAsymmetryThreshold;

	/* filter low current value noise  */
	if(FPL_tPhaseLossData.tThreePhaseCurRMS.a < FPL_tPhaseLossPara.s32CurrentNoiseFilter)
	{
		phA = 1L;
	}
	else
	{
		phA = FPL_tPhaseLossData.tThreePhaseCurRMS.a;
	}

	/* filter low current value noise  */
	if(FPL_tPhaseLossData.tThreePhaseCurRMS.b < FPL_tPhaseLossPara.s32CurrentNoiseFilter)
	{
		phB = 1L;
	}
	else
	{
		phB = FPL_tPhaseLossData.tThreePhaseCurRMS.b;
	}

	/* filter low current value noise */
	if(FPL_tPhaseLossData.tThreePhaseCurRMS.c < FPL_tPhaseLossPara.s32CurrentNoiseFilter)
	{
		phC = 1L;
	}
	else
	{
		phC = FPL_tPhaseLossData.tThreePhaseCurRMS.c;
	}

	if(((phA + phB + phC)/3L ) > FPL_tPhaseLossPara.s32CurrentNoiseFilter)
	{
		FPL_tPhaseLossData.bAsymmetrySampleValid = true;
	}
	else
	{
		FPL_tPhaseLossData.bAsymmetrySampleValid = false;
	}

	/* calculate the percentage value */
	diff_A = phA;
	diff_B = phB;
	if(diff_A > diff_B)
	{
		difference = diff_A - diff_B;
		difference = (difference * 100L) / diff_A;
	}
	else
	{
		difference = diff_B - diff_A;
		difference = (difference * 100L) / diff_B;
	}

	if(difference > threshold)
	{
		isCurrentSymmetrical = false;
	}

	if(difference > FPL_tPhaseLossData.s32MaxRuntimeDiffCurr)
	{
		FPL_tPhaseLossData.s32MaxRuntimeDiffCurr = difference;
	}

	/* calculate the percentage value */
	diff_A = phB;
	diff_B = phC;
	if(diff_A > diff_B)
	{
		difference = diff_A - diff_B;
		difference = (difference * 100L) / diff_A;
	}
	else
	{
		difference = diff_B - diff_A;
		difference = (difference * 100L) / diff_B;
	}

	if(difference > threshold)
	{
		isCurrentSymmetrical = false;
	}

	if(difference > FPL_tPhaseLossData.s32MaxRuntimeDiffCurr)
	{
		FPL_tPhaseLossData.s32MaxRuntimeDiffCurr = difference;
	}

	/* calculate the percentage value */
	diff_A = phC;
	diff_B = phA;
	if(diff_A > diff_B)
	{
		difference = diff_A - diff_B;
		difference = (difference * 100L) / diff_A;
	}
	else
	{
		difference = diff_B - diff_A;
		difference = (difference * 100L) / diff_B;
	}

	if(difference > threshold)
	{
		isCurrentSymmetrical = false;
	}

	if(difference > FPL_tPhaseLossData.s32MaxRuntimeDiffCurr)
	{
		FPL_tPhaseLossData.s32MaxRuntimeDiffCurr = difference;
	}

	return isCurrentSymmetrical;
}
/*************************************************************************************************/
void FPL_vPhaselossprepareSamples(void)
{

	int32_t	temp_phA;
	int32_t	temp_phB;
	int32_t	temp_phC;
	
	TThreePhaseCur * tThreePhaseCurrent = NULL;
	tThreePhaseCurrent = &FPL_tPhaseLossData.tThreePhaseCur;
	CUR_vGetThreePhaseCur(tThreePhaseCurrent);
	
	TThreePhase * tPhaseVoltage = NULL;
	tPhaseVoltage = &FPL_tPhaseLossData.tThreePhaseVol;
	FCN_vGetThreePhaseVol(tPhaseVoltage);

	//TThreePhase * tPhaseVoltage = NULL;
	if((eM1_MainState == MainState_Run)&&(eM1_RunSubState == RunState_Spin))//after close loop
	{

		/* get positive value only */
		temp_phA = (int32_t)tThreePhaseCurrent->a;
		if(temp_phA < 0L)
		{
			temp_phA = -temp_phA;
		}

		temp_phB = (int32_t)tThreePhaseCurrent->b;
		if(temp_phB < 0L)
		{
			temp_phB = -temp_phB;
		}

		temp_phC = (int32_t)tThreePhaseCurrent->c;
		if(temp_phC < 0L)
		{
			temp_phC = -temp_phC;
		}

		/* erase one sample */
		FPL_tPhaseLossData.PhaseABuffCur -= Division(FPL_tPhaseLossData.PhaseABuffCur , PHASEL_MPHW_AVG_BUFFER_SAMPLE_COUNTER);
			/* add new sample */
		FPL_tPhaseLossData.PhaseABuffCur += temp_phA;

		/* erase one sample */
		FPL_tPhaseLossData.PhaseBBuffCur -= Division(FPL_tPhaseLossData.PhaseBBuffCur , PHASEL_MPHW_AVG_BUFFER_SAMPLE_COUNTER);
		/* add new sample */
		FPL_tPhaseLossData.PhaseBBuffCur += temp_phB;

		/* erase one sample */
		FPL_tPhaseLossData.PhaseCBuffCur-= Division(FPL_tPhaseLossData.PhaseCBuffCur , PHASEL_MPHW_AVG_BUFFER_SAMPLE_COUNTER);
		/* add new sample */
		FPL_tPhaseLossData.PhaseCBuffCur += temp_phC;

		/* get positive value only */
		temp_phA = tPhaseVoltage->a;
		if(temp_phA < 0L)
		{
			temp_phA = -temp_phA;
		}

		temp_phB = tPhaseVoltage->b;
		if(temp_phB < 0L)
		{
			temp_phB = -temp_phB;
		}

		temp_phC = tPhaseVoltage->c;
		if(temp_phC < 0L)
		{
			temp_phC = -temp_phC;
		}

		/* erase one sample */
		FPL_tPhaseLossData.PhaseABuffVolt -= Division(FPL_tPhaseLossData.PhaseABuffVolt , PHASEL_MPHW_AVG_BUFFER_SAMPLE_COUNTER);
		/* add new sample */
		FPL_tPhaseLossData.PhaseABuffVolt += temp_phA;

		/* erase one sample */
		FPL_tPhaseLossData.PhaseBBuffVolt -= Division(FPL_tPhaseLossData.PhaseBBuffVolt , PHASEL_MPHW_AVG_BUFFER_SAMPLE_COUNTER);
		/* add new sample */
		FPL_tPhaseLossData.PhaseBBuffVolt += temp_phB;

		/* erase one sample */
		FPL_tPhaseLossData.PhaseCBuffVolt -= Division(FPL_tPhaseLossData.PhaseCBuffVolt , PHASEL_MPHW_AVG_BUFFER_SAMPLE_COUNTER);
		/* add new sample */
		FPL_tPhaseLossData.PhaseCBuffVolt += temp_phC;
	}

}

void FPL_vPhaseLossEvaluation(void)
{

#if (FS_MOTOR_BLOCK_SIMPLE == OPTION_ACTIVE)
	

#else
	if((eM1_MainState == MainState_Run)&&(eM1_RunSubState == RunState_Spin))
		{
			FPL_vPhaselossprepareSamples();
			CUR_updatePhaseCurrentBuffer();
		}

#endif

}

/**
 * \brief
 * This function is needed to be called during the startup to check for disconnected phase, during the ramp up we are loading the phase with current, 
 * if there is a open phase present the adc reads zero current, if the amount of zero current value overflows the filter there is open phase present.
 * */
static void RMP_vOpenPhaseCheckSimpleImplementation(void)
{

	TThreePhaseCur * RMP_ptThreePhaRaw = NULL;
	RMP_ptThreePhaRaw = &(FPL_tPhaseLossDataSimple.tThreePhaseCurRaw);
	CUR_vGetThreePhaseRawValue(RMP_ptThreePhaRaw);

	FPL_tPhaseLossDataSimple.tThreePhaseCurRawOffset.a = Motor_1st.FOC.sIabc_offset.s16A >> 3;
	FPL_tPhaseLossDataSimple.tThreePhaseCurRawOffset.b = Motor_1st.FOC.sIabc_offset.s16B >> 3;
	FPL_tPhaseLossDataSimple.tThreePhaseCurRawOffset.c = Motor_1st.FOC.sIabc_offset.s16C >> 3;

	FPL_tPhaseLossParaSimple.s16CurrentRawHigh = RMP_INCREASE_CUR_HI/16;

	FPL_tPhaseLossParaSimple.s16CurrentRawLow  = RMP_INCREASE_CUR_LO/16;

	FPL_tPhaseLossDataSimple.s16OverCurrentRawHi = FPL_tPhaseLossParaSimple.s16CurrentRawHigh;

	FPL_tPhaseLossDataSimple.s16OverCurrentRawLo = FPL_tPhaseLossParaSimple.s16CurrentRawLow;

	//if(((RMP_ptThreePhaRaw->a < (FPL_tPhaseLossDataSimple.tThreePhaseCurRawOffset.a + (TFp)FPL_tPhaseLossDataSimple.s16OverCurrentRawHi)) && (RMP_ptThreePhaRaw->a > (FPL_tPhaseLossDataSimple.tThreePhaseCurRawOffset.a + (TFp)FPL_tPhaseLossDataSimple.s16OverCurrentRawLo))) || ((RMP_ptThreePhaRaw->b < (FPL_tPhaseLossDataSimple.tThreePhaseCurRawOffset.b + (TFp)FPL_tPhaseLossDataSimple.s16OverCurrentRawHi)) && (RMP_ptThreePhaRaw->b > (FPL_tPhaseLossDataSimple.tThreePhaseCurRawOffset.b + (TFp)FPL_tPhaseLossDataSimple.s16OverCurrentRawLo))))
	//if((RMP_ptThreePhaRaw->a > (FPL_tPhaseLossDataSimple.tThreePhaseCurRawOffset.a + (TFp)FPL_tPhaseLossDataSimple.s16OverCurrentRawLo)) || (RMP_ptThreePhaRaw->b > (FPL_tPhaseLossDataSimple.tThreePhaseCurRawOffset.b + (TFp)FPL_tPhaseLossDataSimple.s16OverCurrentRawLo)))
	if((RMP_ptThreePhaRaw->c < (FPL_tPhaseLossDataSimple.tThreePhaseCurRawOffset.c + (TFp)FPL_tPhaseLossDataSimple.s16OverCurrentRawLo)) || (RMP_ptThreePhaRaw->b > (FPL_tPhaseLossDataSimple.tThreePhaseCurRawOffset.b + (TFp)FPL_tPhaseLossDataSimple.s16OverCurrentRawLo)))
	{ 
		RMP_tPhaseOpenPresentCount = 	RMP_tPhaseOpenPresentCount + 1;
	}	
	
	if(RMP_tPhaseOpenPresentCount > RMP_PHASE_OPEN_FILTER)					/** the amount of zero current overflowed the filter , there is a open phase present **/
	{
		//PWM_vDisablePwmOutputs(ptHwMapLcl->tHwChannel);
		//ERM_vSetMotorError(ptMot, ERM_MOT_PHASE_OPEN);					
		//ERM_vSetMotorError(ERM_SYSTEM_ERROR, ERM_SYS_MOT_APP_ERROR);
		ERT_vErrorReport(MC_FAULT_PHASELESS);
		
		RMP_tPhaseOpenPresentCount = 0;
		RMP_ptThreePhaRaw->a = 0;
		RMP_ptThreePhaRaw->b = 0;
		RMP_ptThreePhaRaw->c = 0;
	
	}

}

void RMP_vOpenPhaseCheckSimple(void)
{
	if((eM1_MainState == MainState_Run)&&(eM1_RunSubState == RunState_Align))
		{
			if(Motor_1st.FOC.sIdq_cmd.s16Q >= Q15(IQ_ALIGN / I_MAX))
				{
					RMP_vOpenPhaseCheckSimpleImplementation();
				}
		}
}

void RMP_vOpenPhaseCheckComplex(void)
{
	bool isPhaseA_Zero = false;
	bool isPhaseB_Zero = false;
	bool isPhaseC_Zero = false;

	FPL_tPhaseLossData.tThreePhaseCurRMS.a = FPL_tPhaseLossData.PhaseABuffCur/PHASEL_MPHW_AVG_BUFFER_SAMPLE_COUNTER;
	FPL_tPhaseLossData.tThreePhaseCurRMS.b = FPL_tPhaseLossData.PhaseBBuffCur/PHASEL_MPHW_AVG_BUFFER_SAMPLE_COUNTER;
	FPL_tPhaseLossData.tThreePhaseCurRMS.c = FPL_tPhaseLossData.PhaseCBuffCur/PHASEL_MPHW_AVG_BUFFER_SAMPLE_COUNTER;

	FPL_tPhaseLossData.tThreePhaseVolRMS.a = FPL_tPhaseLossData.PhaseABuffVolt/PHASEL_MPHW_AVG_BUFFER_SAMPLE_COUNTER;
	FPL_tPhaseLossData.tThreePhaseVolRMS.b = FPL_tPhaseLossData.PhaseBBuffVolt/PHASEL_MPHW_AVG_BUFFER_SAMPLE_COUNTER;
	FPL_tPhaseLossData.tThreePhaseVolRMS.c = FPL_tPhaseLossData.PhaseCBuffVolt/PHASEL_MPHW_AVG_BUFFER_SAMPLE_COUNTER;
	if((eM1_MainState == MainState_Run)&&(eM1_RunSubState == RunState_Spin))//after close loop
	{

		FPL_tPhaseLossData.tTaskEntryTime = STK_tGetSafeTime();
		if ((FPL_tPhaseLossData.tTaskEntryTime - FPL_tPhaseLossData.tLastTimeSliceTime) >= FPL_TASK_TIME_BASE)
		{
		
			FPL_tPhaseLossData.tLastTimeSliceTime = FPL_tPhaseLossData.tTaskEntryTime;

			//Calculate every 10ms
			CUR_vGetPhaseCurrentBufferSamples();

			FPL_tPhaseLossData.bStaticVectorFlag         = FPL_vEvaluateStaticVector();
			FPL_tPhaseLossData.bCurrentAsymmetryFlag     = FPL_vEvaluateCurrentAsymmetry();
			FPL_tPhaseLossData.bVoltageAsymmetryFlag	 = FPL_vEvaluateVoltageAsymmetry();

			
		}

		if(FPL_tPhaseLossData.tThreePhaseVolRMS.a > FPL_tPhaseLossPara.s32VoltageNoiseFilter)
			{
				/* the pwm duty voltage generator is expecting a current flow */
				if(FPL_tPhaseLossData.tThreePhaseCurRMS.a < FPL_tPhaseLossPara.s32CurrentNoiseFilter)
				{
					isPhaseA_Zero = true;
				}
			}

			if(FPL_tPhaseLossData.tThreePhaseVolRMS.b > FPL_tPhaseLossPara.s32VoltageNoiseFilter)
			{
				/* the pwm duty voltage generator is expecting a current flow */
				if(FPL_tPhaseLossData.tThreePhaseCurRMS.b < FPL_tPhaseLossPara.s32CurrentNoiseFilter)
				{
					isPhaseB_Zero = true;
				}
			}

			if(FPL_tPhaseLossData.tThreePhaseVolRMS.c > FPL_tPhaseLossPara.s32VoltageNoiseFilter)
			{
				/* the pwm duty voltage generator is expecting a current flow */
				if(FPL_tPhaseLossData.tThreePhaseCurRMS.c < FPL_tPhaseLossPara.s32CurrentNoiseFilter)
				{
					isPhaseC_Zero = true;
				}
			}


			if((FPL_tPhaseLossData.bVoltageAsymmetryFlag == false) || (FPL_tPhaseLossData.bCurrentAsymmetryFlag == false))
			{
				/* the current needs to be high enough to validated asymmetry */
				if((FPL_tPhaseLossData.bAsymmetrySampleValid == true) && (FPL_tPhaseLossData.bStaticVectorFlag == false))
				{
					FPL_tPhaseLossData.bPhaseCorrupted = true;
			
					/* find single phase */
					if((isPhaseA_Zero == true) || (isPhaseB_Zero == true) || (isPhaseC_Zero == true))
					{
						FPL_tPhaseLossData.bPhaseLost = true;
					}
					else
					{
						FPL_tPhaseLossData.bPhaseAsymmetrical = true;
					}
				}
			}


			if(FPL_tPhaseLossData.bPhaseCorrupted == false)
			{
				FPL_tPhaseLossData.bPhaseLost = false;
				FPL_tPhaseLossData.bPhaseAsymmetrical = false;
			}

				/* Save last entry time */
			
			if(FPL_tPhaseLossData.bPhaseCorrupted == true)
			{
				FPL_tPhaseLossData.u16ErrorCounter++; 													/* Increment error counter if the asymmetry deviation is out of allowed limit */
			}
			else
			{
				if (FPL_tPhaseLossData.u16ErrorCounter > 0UL)
				{
					FPL_tPhaseLossData.u16ErrorCounter--; 												/* If the asymmetry deviation is in allowed range - decrement the error counter */
				}
			}
			/* for debug
			if(FPL_tPhaseLossData.u16ErrorCounter > FPL_tPhaseLossData.u16MaxRuntimeErrorCounter)
			{
				FPL_tPhaseLossData.u16MaxRuntimeErrorCounter = FPL_tPhaseLossData.u16ErrorCounter;
			}
			*/
			if(FPL_tPhaseLossData.u16ErrorCounter > FPL_tPhaseLossPara.u16ErrorCntLimit)
			{
				//Error Handle
				ERT_vErrorReport(MC_FAULT_PHASELESS);
			}
	}

}


void FPL_vPhaseLossCheck(void)
{
#if (FS_MOTOR_BLOCK_SIMPLE == OPTION_ACTIVE)

	RMP_vOpenPhaseCheckSimple();

#else

	RMP_vOpenPhaseCheckComplex();

#endif

}



