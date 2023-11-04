#include "drv_inc.h"
#include "StateMachine.h"
#include "parameter.h"
#include "ParamMot.h"

#include "fsparameter.h"
#include "ParamMot.h"
#include "FOC_PMSM.h"
#include "Compile.h"
#include "Ramp.h"
#include "main.h"
#include "Current.h"
#include "Voltage.h"
#include "SpeedMonitor.h"
#include "Filter.h"
#include "Transfer.h"
#include "fsCommon.h"
#include "ErrorReact.h"

/* VARIABLES ******************************************************************************************/
static TFilterData	    SMR_tFluxAlfa          = {0,0,0};			///< Flux component psi alfa
static TFilterData		SMR_tFluxBeta          = {0,0,0};			///< Flux component psi beta
static TCartNum			SMR_tPsiCart           = {0,0};			    ///< Filt.cart.flux psialfa,psibeta
static TPolNum			SMR_tPsiPol            = {0,0};			    ///< Filt.polar comp. MaxFlux, psi
static TAngle			SMR_tFluxVoltPsiCalc   = 0;	                ///< Actual psi
static TAngle			SMR_tFluxVoltPsiLast   = 0;              	///< Previous psi
static TFilterData 		SMR_tStepDeltaPsiActu  = {0,0,0};	        ///< Filtered delta psi value

static TFp              SMR_tSpeedDiff         = 0;					///< instantaneous speed difference of the models

static uint32_t SMR_u32SpeedErrorCounter	   = 0;	                ///< speed difference overflow counter

static TSafeTime SMR_tCycleStartTime		   = 0;	                ///< start time stamp for the current cycle
static TSafeTime SMR_tActualTime			   = 0;             	///< actual time for the current cycle

static uint32_t SMR_u32CycleTimeCheck	       = FSP_SMR_MOTOR_MODEL_CYCLE_TIME_CHECK;	///< Speed difference cycle time check - 1s

static uint32_t SMR_u32SpeedErrorLimit	       = FSP_SMR_MOTOR_MODEL_MAX_SPEED_ERRORS;	///< Max. speed errors per one cycle check

static TFilterCoeff SMR_tPT1CoefFlx			   = MPM_OBR_FLX_FILCOE;				///< Motor model PT1 filter flux calculation
static TFilterCoeff SMR_tFltCoefDPsi		   = MOTPAR_DPSI_FLT_COEF;				///< Motor model PT1 filter flux calculation

static TFp SMR_tSpeedDiffLimitPos		       = (TFp)FSP_SMR_MOTOR_MODEL_MAX_SPEED_DIFF;	///< Max. positive speed difference
static TFp SMR_tSpeedDiffLimitNeg		       = -(TFp)FSP_SMR_MOTOR_MODEL_MAX_SPEED_DIFF;	///< Max. negative speed difference
/******************************************************************************************************/
/*Second solution*/
/******************************************************************************************************/
typedef struct _TSpeedDeviation
{
	TFp  s32UplimitDeviation;
	TFp  s32DownlimitDeviation;
}TSpeedDeviation;

typedef struct _TSMRParaSecond
{
	int32_t u32MinSpeed;
	TSpeedDeviation tSpeedDeviation;
	int32_t u32ErrorCountMax;
	uint16_t u16MaxErrorCnt;
}TSMRParaSecond;

#define SMR_PWM_RESOLUTION_CALC             32768

/**
 * [ms] Task handler time base
 * */
#define SMR_TASK_TIME_BASE_MS                (10)

/* CONVERSION of the config values to working values */

/**
 *  [SafeTime] Task handler time base
 * */
#define SMR_TASK_TIME_BASE                  STK_DEF_TIME_MS(SMR_TASK_TIME_BASE_MS)

//#define SMR_Angle_DIFF_2_SPD_PU           PWMFREQ*15/10000/FCN_MONITOR_PERIOD
#define SMR_Angle_DIFF_2_SPD_PU             5

static TFilterCoeff        SMR_tFilterCoeEmf	= FIR_tCalcFilterConst(0.065,5);
static TFilterCoeff        SMR_tFilterCoeSpd	= FIR_tCalcFilterConst(0.07,100);

static TSMRDataSecond 	   SMR_tFSVDataSecond   = {0,0,0,0,0,0,{0,0},0,0};
static TSMRParaSecond      SMR_tFSVParaSecond   = {150,{20,-20},0,150};

static TSmrModelInputData BackupModel           ={0,0,0,0,0};

/*****************************test variables********************************************/
static uint16_t SMR_u16TestValue;

static uint32_t SMR_s32Re;
static uint32_t SMR_s32Im;

static int16_t SMR_s16Ls = 0;
static int32_t SMR_s32Temp1 = 0;
static int32_t SMR_s32Temp2 = 0;
static int32_t SMR_s32DeltaTime = 0;
/* LOCAL FUNCTION PROTOTYPES **************************************************************************/

/* LOCAL FUNCTION DEFINITION **************************************************************************/

/* GLOBAL FUNCTION DEFINITION *************************************************************************/

/*************************************************************First Solution**************************************************/
void SMR_vInitModelCalculation(void)
{
	SMR_u32SpeedErrorCounter = 0;
	SMR_tSpeedDiff = 0;
}


/*
**this function should be called in the 10khz carrier wave interrupt
**
*/
void SMR_vMotorModelCalculation(const TSmrModelInputData* ptModel) {
	int32_t	siPsiAlfaQ;		/* Alfa component of flux */
	int32_t	siPsiBetaQ;		/* Beta component of flux */
	TAngle	tDeltaPsiLcl;		/* local variable for speed calculation */
	/* ptModel->tUAlpha   :   flux component */
	siPsiAlfaQ = ptModel->tUAlpha;															/* ualfa             */
	/* siPsiBetaQ  = ptModel->tUBeta; ->  Actual flux compenent = beta */
	siPsiBetaQ = ptModel->tUBeta ;															/* ubeta             */

	SMR_tFluxAlfa.tFilterInputValue = siPsiAlfaQ;											/* Assign flux alfa filter input */
	SMR_tFluxBeta.tFilterInputValue = siPsiBetaQ;											/* Assign flux beta filter input */
	FIR_vCalcFilter(&SMR_tFluxAlfa, &SMR_tPT1CoefFlx);									/* Calc.alfa filter */
	FIR_vCalcFilter(&SMR_tFluxBeta, &SMR_tPT1CoefFlx);									/* Calc.beta filter */

	SMR_tPsiCart.tRe = FIR_tGetFilterOutput(&SMR_tFluxAlfa);								/* Assign filt. val. to cartesian components */
	SMR_tPsiCart.tIm = FIR_tGetFilterOutput(&SMR_tFluxBeta);

	FPM_vCart2Pol(&SMR_tPsiCart, &SMR_tPsiPol);						/* Calculate flux polar components FluxAmp & psi */
	SMR_tFluxVoltPsiCalc = SMR_tPsiPol.siAngle;						/* Assign actual flux angle psi (used for transformations ) */

	tDeltaPsiLcl = SMR_tFluxVoltPsiCalc;
	tDeltaPsiLcl -= SMR_tFluxVoltPsiLast;
	SMR_tFluxVoltPsiLast = SMR_tFluxVoltPsiCalc;					/* Shift actual value into history */
	SMR_tStepDeltaPsiActu.tFilterInputValue = (TFp)tDeltaPsiLcl;		/* LPF */
	FIR_vCalcFilter(&SMR_tStepDeltaPsiActu, &SMR_tFltCoefDPsi);

	/* evaluate difference between two models */
	SMR_tSpeedDiff = FIR_tGetFilterOutput(&SMR_tStepDeltaPsiActu) - ptModel->tDeltaPsi;

	/* Check models speed difference for limits */
	if(SMR_tSpeedDiff > SMR_tSpeedDiffLimitPos) {
		SMR_u32SpeedErrorCounter++;
	}
	else
	{
		if(SMR_tSpeedDiff < SMR_tSpeedDiffLimitNeg)
		{
			SMR_u32SpeedErrorCounter++;
		}
	}

	SMR_tActualTime = STK_tGetSafeTime();
	/* Is time up for speed error number limit? */
	if((SMR_tActualTime - SMR_tCycleStartTime) >= SMR_u32CycleTimeCheck) {
		SMR_tCycleStartTime = SMR_tActualTime;
		/* Check error counter for limit */
		if(SMR_u32SpeedErrorCounter > SMR_u32SpeedErrorLimit) {
			//Stop the motor.
			ERT_vErrorReport(MC_FAULT_SPEED_INFOR);
			//ERROR
		}
		SMR_u32SpeedErrorCounter = 0;
	}
}

/***********************************************second solution************************************************/       
void SMR_tgetPWMDutyValues(void)
{
	static TDutyCycleConfig* ptTemp = NULL;
	static TDutyCycleConfig Temp = {0U,0U,0U};
	const TIM_TypeDef * ptATmrReg   = NULL;            /* Timer */
	uint32_t TempPWMCalc = 0UL;
	uint32_t TempArr = 0UL;

	if(ADC_GetSoftwareStartConvStatus(ADC1) == SET)//ADC interrupt enable
	{
		ptTemp = &SMR_tFSVDataSecond.tDutyCycle;
		ptATmrReg =  (TIM_TypeDef*)TIM1_BASE;

		TempArr = ptATmrReg->ARR;
		/* check division by 0 */
		if(TempArr > 0UL)
		{
			TempPWMCalc = (uint32_t)((uint32_t)SMR_PWM_RESOLUTION_CALC * (uint32_t)ptATmrReg->CCR1);
			TempPWMCalc = TempPWMCalc / TempArr;

			ptTemp->uiDutyCyclePhaseU = (uint16_t)TempPWMCalc;

			TempPWMCalc = (uint32_t)((uint32_t)SMR_PWM_RESOLUTION_CALC * (uint32_t)ptATmrReg->CCR2);
			TempPWMCalc = TempPWMCalc / TempArr;

			ptTemp->uiDutyCyclePhaseV = (uint16_t)TempPWMCalc;

			TempPWMCalc = (uint32_t)((uint32_t)SMR_PWM_RESOLUTION_CALC * (uint32_t)ptATmrReg->CCR3);
			TempPWMCalc = TempPWMCalc / TempArr;

			ptTemp->uiDutyCyclePhaseW = (uint16_t)TempPWMCalc;
		}
		else
		{
			*ptTemp = Temp;
		}
	}
	else
	{
		ptTemp->uiDutyCyclePhaseU = 0;
		ptTemp->uiDutyCyclePhaseV = 0;
		ptTemp->uiDutyCyclePhaseW = 0;
	}

	//return ptTemp;

}

void SMR_vGetPhaseVoltage(void)
{
	
	//SMR_tFSVDataSecond.tDutyCycle = SMR_tgetPWMDutyValues();
	const uint16_t VDC_temp = VOL_tGetDclVolRawSampleQ();																/* Assign VDC to local variable and use it for next calculations */

	/* Calc phase voltages from duty cycles Q15*/
	SMR_tFSVDataSecond.tDutyCycle.uiDutyCyclePhaseU = Motor_1st.FOC.SVM.s16DutyA;
	SMR_tFSVDataSecond.tDutyCycle.uiDutyCyclePhaseV = Motor_1st.FOC.SVM.s16DutyB;
	SMR_tFSVDataSecond.tDutyCycle.uiDutyCyclePhaseW = Motor_1st.FOC.SVM.s16DutyC;

	SMR_u16TestValue = (((2L*SMR_tFSVDataSecond.tDutyCycle.uiDutyCyclePhaseU) - (SMR_tFSVDataSecond.tDutyCycle.uiDutyCyclePhaseV + SMR_tFSVDataSecond.tDutyCycle.uiDutyCyclePhaseW)))/3L;
	SMR_tFSVDataSecond.tConstructVoltage.tVoltPhase.a = MLIB_Mul_Q15((((2L*SMR_tFSVDataSecond.tDutyCycle.uiDutyCyclePhaseU) - (SMR_tFSVDataSecond.tDutyCycle.uiDutyCyclePhaseV + SMR_tFSVDataSecond.tDutyCycle.uiDutyCyclePhaseW)))/3L, Motor_1st.FOC.s16VbusAvg);
	SMR_tFSVDataSecond.tConstructVoltage.tVoltPhase.b = MLIB_Mul_Q15((((2L*SMR_tFSVDataSecond.tDutyCycle.uiDutyCyclePhaseV) - (SMR_tFSVDataSecond.tDutyCycle.uiDutyCyclePhaseU + SMR_tFSVDataSecond.tDutyCycle.uiDutyCyclePhaseW)))/3L, Motor_1st.FOC.s16VbusAvg);
	SMR_tFSVDataSecond.tConstructVoltage.tVoltPhase.c = MLIB_Mul_Q15((((2L*SMR_tFSVDataSecond.tDutyCycle.uiDutyCyclePhaseW) - (SMR_tFSVDataSecond.tDutyCycle.uiDutyCyclePhaseU + SMR_tFSVDataSecond.tDutyCycle.uiDutyCyclePhaseV)))/3L, Motor_1st.FOC.s16VbusAvg);

}

//void SMR_vGetPhaseCurrent(void)
//{
//	CUR_vGetThreePhaseCur(&SMR_tFSVDataSecond.tCurrent.tIThree);
//}

void SMR_vIndependentMotorModelCalculation(void)
{
	//get phase a,b,c current
	//SMR_vGetPhaseCurrent();
	CUR_vGetThreePhaseCur(&SMR_tFSVDataSecond.tCurrent.tIThree);

	//get phase a,b,c voltage
	//SMR_vGetPhaseVoltage();
	FCN_vGetThreePhaseVol(&SMR_tFSVDataSecond.tConstructVoltage.tVoltPhase);

	/* Transform currents and voltages into orthogonal system */
	TRR_vClarkeTrans(&SMR_tFSVDataSecond.tCurrent.tIThree,&SMR_tFSVDataSecond.tCurrent.tICart);	
	TRR_vClarkeTrans(&SMR_tFSVDataSecond.tConstructVoltage.tVoltPhase,&SMR_tFSVDataSecond.tConstructVoltage.tVoltCart);	
	//MCFLIB_Clark_Sat_S16(&SMR_tFSVDataSecond.tCurrent.tIThree, &SMR_tFSVDataSecond.tCurrent.tICart);
	//MCFLIB_Clark_Sat_S16((MCFLIB_3_ABC_T_S16 *)(&SMR_tFSVDataSecond.tConstructVoltage.tVoltPhase), (MCFLIB_2_ALBE_T_S16 *)(&SMR_tFSVDataSecond.tConstructVoltage.tVoltCart));

	/*Vs-R*I*/
	SMR_tFSVDataSecond.tConstructVoltage.tVoltAlpha.tFilterInputValue = SMR_tFSVDataSecond.tConstructVoltage.tVoltCart.tRe;
	SMR_tFSVDataSecond.tConstructVoltage.tVoltAlpha.tFilterInputValue -= MLIB_Mul_Q15(SMR_tFSVDataSecond.tCurrent.tICart.tRe, MOTPAR_RS_Q);

	SMR_tFSVDataSecond.tConstructVoltage.tVolBelta.tFilterInputValue= SMR_tFSVDataSecond.tConstructVoltage.tVoltCart.tIm;
	SMR_tFSVDataSecond.tConstructVoltage.tVolBelta.tFilterInputValue -= MLIB_Mul_Q15(SMR_tFSVDataSecond.tCurrent.tICart.tIm, MOTPAR_RS_Q);

	/*Filter*/
	FIR_vCalcFilter(&SMR_tFSVDataSecond.tConstructVoltage.tVoltAlpha, &SMR_tFilterCoeEmf);
	FIR_vCalcFilter(&SMR_tFSVDataSecond.tConstructVoltage.tVolBelta, &SMR_tFilterCoeEmf);



	SMR_tFSVDataSecond.tConstructVoltage.tFluxVolt.tRe = SMR_tFSVDataSecond.tConstructVoltage.tVoltAlpha.tRealFilterOutputValue;
	SMR_tFSVDataSecond.tConstructVoltage.tFluxVolt.tIm = SMR_tFSVDataSecond.tConstructVoltage.tVolBelta.tRealFilterOutputValue;

	//SMR_s16Ls = MOTPAR_LS_Q;
	//SMR_s32Temp1 = SMR_tFSVDataSecond.tCurrent.tICart.tRe;
	//SMR_s32Temp2 = SMR_tFSVDataSecond.tCurrent.tICart.tIm;
	SMR_s32Re = MLIB_Mul_Q15(SMR_tFSVDataSecond.tCurrent.tICart.tRe,MOTPAR_LS_Q);
	SMR_s32Im =	MLIB_Mul_Q15(SMR_tFSVDataSecond.tCurrent.tICart.tIm,MOTPAR_LS_Q);

	/*Calculate Vs-R*I-L*I*/
	SMR_tFSVDataSecond.tConstructVoltage.tFluxVolt.tRe += -SMR_s32Re;
	SMR_tFSVDataSecond.tConstructVoltage.tFluxVolt.tIm += -SMR_s32Im;

	/*Calcualte Bemf angle and magnitude*/
	FPM_vCart2Pol((TCartNum *)&SMR_tFSVDataSecond.tConstructVoltage.tFluxVolt, &SMR_tFSVDataSecond.tConstructVoltage.tPolarVolt);

	/*Angle difference*/
	SMR_tFSVDataSecond.tConstructVoltage.tAngleDiff = SMR_tFSVDataSecond.tConstructVoltage.tPolarVolt.siAngle - SMR_tFSVDataSecond.tConstructVoltage.tLastAngle;

		/*Update last angle*/
	SMR_tFSVDataSecond.tConstructVoltage.tLastAngle = SMR_tFSVDataSecond.tConstructVoltage.tPolarVolt.siAngle;

	/*Filter*/
	SMR_tFSVDataSecond.tSpeedObserver.tObserverSpeedFilter.tFilterInputValue = SMR_tFSVDataSecond.tConstructVoltage.tAngleDiff;

	/* Check the symmetry of phase voltages - when they equals, that means the motor is not supplied */
	if ((SMR_tFSVDataSecond.tConstructVoltage.tVoltPhase.a == SMR_tFSVDataSecond.tConstructVoltage.tVoltPhase.b) &&
		(SMR_tFSVDataSecond.tConstructVoltage.tVoltPhase.b == SMR_tFSVDataSecond.tConstructVoltage.tVoltPhase.c))
	{
		SMR_tFSVDataSecond.tSpeedObserver.s32ObserverSpd = 0L;
		SMR_tFSVDataSecond.tObserverSpdPU = Q15(SMR_tFSVDataSecond.tSpeedObserver.s32ObserverSpd/SPEED_MAX);
		FIR_vCalcFilter(&SMR_tFSVDataSecond.tSpeedObserver.tObserverSpeedFilter, 0L);
	}
	else
	{

		/*low pass filter*/
		FIR_vCalcFilter(&SMR_tFSVDataSecond.tSpeedObserver.tObserverSpeedFilter, &SMR_tFilterCoeSpd);
#ifdef USE_COMPLEX_PROCESS
		/*Calculate real speed*/
		SMR_tFSVDataSecond.tSpeedObserver.s32ObserverSpd = SMR_tFSVDataSecond.tSpeedObserver.tObserverSpeedFilter.tRealFilterOutputValue*PWMFREQ*60/4/65536;
		
		/* Prevent rotation direction dependency */
		if (SMR_tFSVDataSecond.tSpeedObserver.s32ObserverSpd  < 0) 		/* If the result of speed calculation is negative, calculate the absolute value */
		{
			SMR_tFSVDataSecond.tSpeedObserver.s32ObserverSpd *= (-1);		/* The motor speed should be always a positive number */
		}
		
		/* Filter out not reliable speeds caused by phase current noise i.e. in idle or DC current states */
		if(SMR_tFSVDataSecond.tSpeedObserver.s32ObserverSpd < SMR_tFSVParaSecond.u32MinSpeed)
		{
			SMR_tFSVDataSecond.tSpeedObserver.s32ObserverSpd = 0L;
		}
		SMR_tFSVDataSecond.tObserverSpdPU = Q15(SMR_tFSVDataSecond.tSpeedObserver.s32ObserverSpd/SPEED_MAX);
#else
		SMR_tFSVDataSecond.tObserverSpdPU = SMR_tFSVDataSecond.tSpeedObserver.tObserverSpeedFilter.tRealFilterOutputValue*SMR_Angle_DIFF_2_SPD_PU;
		if(SMR_tFSVDataSecond.tObserverSpdPU < /*Q15(SMR_tFSVParaSecond.u32MinSpeed/SPEED_MAX)*/1967)
		{
			SMR_tFSVDataSecond.tObserverSpdPU = 0L;
		}
				/* Prevent rotation direction dependency */
		if (SMR_tFSVDataSecond.tObserverSpdPU < 0) 		/* If the result of speed calculation is negative, calculate the absolute value */
		{
			SMR_tFSVDataSecond.tObserverSpdPU *= (-1);		/* The motor speed should be always a positive number */
		}
#endif
	}

}

TSMRDataSecond * SMR_tGetFSVData(void)
{
	return &SMR_tFSVDataSecond;
}

/******************************************************************************************************
void SMR_vSpeedEvaluation(void)
it should be called in the interrupt. calling period is adjustable
******************************************************************************************************/
void SMR_vSpeedEvaluationSimple(void)
{
    
	int32_t s32TempValue = 0;
	if((eM1_MainState == MainState_Run)&&(eM1_RunSubState == RunState_Spin))//after close loop
	{
		BackupModel.tIAlpha   = Motor_1st.FOC.sIAlBe.s16Alpha;
		BackupModel.tIBeta    = Motor_1st.FOC.sIAlBe.s16Beta;
		BackupModel.tUAlpha   = Motor_1st.FOC.sVAlBe.s16Alpha;
		BackupModel.tUBeta    = Motor_1st.FOC.sVAlBe.s16Beta;
		//BackupModel.tDeltaPsi = Motor_1st.FOC.SMO.s16Speed;
		//convert to angle diff from virtual speed.
		s32TempValue  = Motor_1st.FOC.SMO.s16Speed*POLEPAIRS/60;
		s32TempValue  = s32TempValue*65536/PWMFREQ;
		s32TempValue  = s32TempValue*5000/32768;
		BackupModel.tDeltaPsi = s32TempValue;
		SMR_vMotorModelCalculation(&BackupModel);
	}

}

void SMR_vSpeedEvaluationComplex(void)
{
	if ((eM1_MainState == MainState_Run)&&(eM1_RunSubState == RunState_Spin))
		{
			/*calculate motor speed according to voltage balance equation */
			SMR_vIndependentMotorModelCalculation();
		}
}

void SMR_vSpeedEvaluation(void)
{
#if(FS_SPEED_MONITOR_SIMPLE == OPTION_ACTIVE)

	SMR_vSpeedEvaluationSimple();

#else

	SMR_vSpeedEvaluationComplex();

#endif

}

/****************************************************************************************************
speed error detection
****************************************************************************************************/
void SMR_vCheckSpeedMonitorImplement(void)
{
	if ((eM1_MainState == MainState_Run)&&(eM1_RunSubState == RunState_Spin))
	{
		SMR_tFSVDataSecond.tTaskEntryTime = STK_tGetSafeTime();
		if ((SMR_tFSVDataSecond.tTaskEntryTime - SMR_tFSVDataSecond.tLastTimeSliceTime) >= SMR_TASK_TIME_BASE)
		{
			SMR_s32DeltaTime = SMR_tFSVDataSecond.tTaskEntryTime - SMR_tFSVDataSecond.tLastTimeSliceTime;
			//SMR_tFSVDataSecond.tLastTimeSliceTime = SMR_tFSVDataSecond.tTaskEntryTime;
			
			/**/
			SMR_tFSVDataSecond.tSpdMonitorSpeedLimit.s32UpLimitSpeed = Motor_1st.FOC.SMO.s16Speed*(SMR_tFSVParaSecond.tSpeedDeviation.s32UplimitDeviation + 100)/100;
			SMR_tFSVDataSecond.tSpdMonitorSpeedLimit.s32DownLimitSpeed = Motor_1st.FOC.SMO.s16Speed*(SMR_tFSVParaSecond.tSpeedDeviation.s32DownlimitDeviation + 100)/100;

			if((SMR_tFSVDataSecond.tObserverSpdPU > SMR_tFSVDataSecond.tSpdMonitorSpeedLimit.s32UpLimitSpeed)
				||(SMR_tFSVDataSecond.tObserverSpdPU < SMR_tFSVDataSecond.tSpdMonitorSpeedLimit.s32DownLimitSpeed))
				{
					SMR_tFSVDataSecond.u32ErrorCount++;
				}
				else
				{
					if(SMR_tFSVDataSecond.u32ErrorCount > 0)
					SMR_tFSVDataSecond.u32ErrorCount--;
				}

			if(SMR_tFSVDataSecond.u32ErrorCount >= SMR_tFSVParaSecond.u16MaxErrorCnt)
				{
					/*lose synchronization*/
					//Turn off the motor
					ERT_vErrorReport(MC_FAULT_SPEED_INFOR);
				}
		}

	}
}

void SMR_vCheckSpeedMonitor(void)
{

#if (FS_SPEED_MONITOR_SIMPLE == OPTION_ACTIVE)

#else

	SMR_vCheckSpeedMonitorImplement();

#endif
}

