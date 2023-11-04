#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>

#include "mm32_it.h"
#include "StateMachine.h"
#include "Ramp.h"
#include "parameter.h"
#include "Current.h"
#include "Voltage.h"
#include "fsTemperature.h"
#include "ErrorReact.h"

#define	    MIT_NULLEVENT       0x00
#define		MIT_1MSEVENT        0x01
#define		MIT_10MSEVENT       0x02
#define		MIT_100MSEVENT      0x04

#define     TSE_MAX_TASK        0x08

typedef enum _tEventFunctionID
{
	TSE_1MSEVENT   = 0,
	TSE_10MSEVENT  = 1,
	TSE_100MSEVENT = 2,

}tEventFunctionID;

static uint16_t TSE_u16EventFlag = 0;

void TSE_vAdaptInterfaceFromFlag2TaskCall(void)
{
	if(Motor_1st.USER.bSlowLoopFlag == true)
	{
		Motor_1st.USER.bSlowLoopFlag =  0;
		TSE_u16EventFlag |= MIT_1MSEVENT;
	}

	if(Motor_1st.USER.bSlowLoopFlag2 == true)
	{
		Motor_1st.USER.bSlowLoopFlag2 =  0;
		TSE_u16EventFlag |= MIT_10MSEVENT;
	}
}

void TSE_vRegisterHandler(uint8_t u8EventFunction)
{
	switch(u8EventFunction)
		{

          case TSE_1MSEVENT:
          	{
          		//add your function
				VOL_vTrackVoltageValues();
				FTE_vTemperatureCal();
				/* Slow Loop Statemachine */
				#if(MOTOR_ALTERNATIVE_START_MODE == OPTION_PASSIVE)
					s_STATE_SLOW[eM1_MainState]();
				#endif
          		break;
          	}

		case TSE_10MSEVENT:
			{
				//add your function
				#if(MOTOR_ALTERNATIVE_START_MODE == OPTION_ACTIVE)
          		s_STATE_SLOW[eM1_MainState]();
				//RMP_vCalTrapezoidalCurve(RMP_ALIGN_CUR_STEP, &(Motor_1st.FOC.sIdq_cmd.s16Q), &(Motor_1st.USER.s16AlignIqCmd));

				//Motor_1st.FOC.s16SpeedRamp = RMP_s16CalcSmootherSinusoidalSpeedRampCurve(RMP_RAMP_UP_GRADIENT, &(Motor_1st.FOC.s16SpdCmd));
				#endif
			break;
			}

	    case TSE_100MSEVENT:
	      	{
	      	   //add your function

	      		break;
	      	}
		default:
			{
				break;
			}

		}



}


void TSE_vSliceTaskHandle(void)
{
	uint16_t u16RegisterIndex = 0x0001;
	uint8_t i = 0;

	TSE_vAdaptInterfaceFromFlag2TaskCall();

	if(TSE_u16EventFlag != 0)
	{
		for(i=0; i< TSE_MAX_TASK; i++)
		{
			u16RegisterIndex = 0x0001;
			u16RegisterIndex = u16RegisterIndex<<i;

			if((TSE_u16EventFlag & u16RegisterIndex) != 0)
			{
				TSE_u16EventFlag = TSE_u16EventFlag&(~u16RegisterIndex);
				TSE_vRegisterHandler(i);
			}

		}

	}


}



