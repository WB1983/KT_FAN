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
#include "drv_inc.h"
#include "Common.h"


typedef struct _TTHREEPHASEVOL
{
	TDutyCycleConfig       	tDutyCycle;
	TConstructVoltage     	tConstructVoltage;
	TCurrents 				tCurrent;
}TTHREEPHASEVOL;
static TTHREEPHASEVOL FCN_tThreePhaseVolCal;

static TThreePhaseCur       FCN_tThreePhaseCur = {0,0,0};
static TThreePhaseVolt      FCN_tThreePhaseVol = {0,0,0};

void FCN_vGetPhaseVoltage(void)
{
    int16_t Temp_A = 0;
    int16_t Temp_B = 0;
    int16_t Temp_C = 0;

	const uint16_t VDC_temp = VOL_tGetDclVolRawSampleQ();/* Assign VDC to local variable and use it for next calculations */

	/* Calc phase voltages from duty cycles Q15*/
	FCN_tThreePhaseVolCal.tDutyCycle.uiDutyCyclePhaseU = Motor_1st.FOC.SVM.s16DutyA;
	FCN_tThreePhaseVolCal.tDutyCycle.uiDutyCyclePhaseV = Motor_1st.FOC.SVM.s16DutyB;
	FCN_tThreePhaseVolCal.tDutyCycle.uiDutyCyclePhaseW = Motor_1st.FOC.SVM.s16DutyC;

    Temp_A = (int16_t)Division((int32_t)((2L*FCN_tThreePhaseVolCal.tDutyCycle.uiDutyCyclePhaseU) - (FCN_tThreePhaseVolCal.tDutyCycle.uiDutyCyclePhaseV + FCN_tThreePhaseVolCal.tDutyCycle.uiDutyCyclePhaseW)),3L);
    Temp_B = (int16_t)Division((int32_t)((2L*FCN_tThreePhaseVolCal.tDutyCycle.uiDutyCyclePhaseV) - (FCN_tThreePhaseVolCal.tDutyCycle.uiDutyCyclePhaseU + FCN_tThreePhaseVolCal.tDutyCycle.uiDutyCyclePhaseW)),3L);
    Temp_C = (int16_t)Division((int32_t)((2L*FCN_tThreePhaseVolCal.tDutyCycle.uiDutyCyclePhaseW) - (FCN_tThreePhaseVolCal.tDutyCycle.uiDutyCyclePhaseU + FCN_tThreePhaseVolCal.tDutyCycle.uiDutyCyclePhaseV)),3L);

	FCN_tThreePhaseVolCal.tConstructVoltage.tVoltPhase.a = MLIB_Mul_Q15(Temp_A, Motor_1st.FOC.s16VbusAvg);
	FCN_tThreePhaseVolCal.tConstructVoltage.tVoltPhase.b = MLIB_Mul_Q15(Temp_B, Motor_1st.FOC.s16VbusAvg);
	FCN_tThreePhaseVolCal.tConstructVoltage.tVoltPhase.c = MLIB_Mul_Q15(Temp_C, Motor_1st.FOC.s16VbusAvg);

    FCN_tThreePhaseVol.a = FCN_tThreePhaseVolCal.tConstructVoltage.tVoltPhase.a;
    FCN_tThreePhaseVol.b = FCN_tThreePhaseVolCal.tConstructVoltage.tVoltPhase.b;
    FCN_tThreePhaseVol.c = FCN_tThreePhaseVolCal.tConstructVoltage.tVoltPhase.c;
}


void FCN_vGetThreePhaseVol(TThreePhaseVolt* ptThreePha)
{
	ptThreePha->a = FCN_tThreePhaseVol.a;
	ptThreePha->b = FCN_tThreePhaseVol.b;
	ptThreePha->c = FCN_tThreePhaseVol.c;
}