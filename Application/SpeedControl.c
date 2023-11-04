#include "LibDefines.h"
#include "Common.h"
#include "hal_misc.h"
#include "drv_inc.h"
#include "board.h"
#include "mm32_it.h"
#include "parameter.h"
#include "ramp.h"
#include "Filter.h"
#include "Modbus.h"
#include "Modbus_2.h"
#include "Statemachine.h"
#include "user_function.h"
#include "statemachineCom.h"
#include "Appmodbus2stm.h"
#include "speedcontrol.h"

#define MIN_DUTY_PWM            Q15(0.15)//Q15(0.05)
#define MAX_DUTY_PWM            Q15(0.65)//Q15(0.45)
#define DUTY_RANG               MAX_DUTY_PWM - MIN_DUTY_PWM

#define MIN_SPD                 STOP_TO_RUN_SPEED
#define MAX_SPD                 M1_MAX_SPEED
#define SPEED_RANGE             MAX_SPD - MIN_SPD

//#define TRANSFER_FACTOR         275//27.5*10, SPEED_RANGE/DUTY_RANG
#define TRANSFER_FACTOR         220//27.5*10, SPEED_RANGE/DUTY_RANG


#define SCL_bGetDirection()   GPIO_ReadInputDataBit(DIRECTION_PORT, DIRECTION_PIN)

#define SCL_POSITIVE_VAL        1
#define SCL_NEGATIVE_VAL       -1

#define SCL_DIR_DET_CNT        10
#define SCL_DIR_DECISION_CNT   7

typedef struct _TSpeedCtrlCommand
{
	int16_t      s16TargetSpeed;
	TDirection   tCurrentDirection;

}TSpeedCtrlCommand;

typedef enum _TSpeedCtrlMode
{
	E_PWM_CONTROL,
	E_MODEBUS_SLAVE_CONTROL,
	E_DEBUG_CONTROL,
	E_MODEBUS_MASTER_CONTROL,

}TSpeedCtrlMode;

typedef struct _TSpeedControlData
{
	TSpeedCtrlMode     u8ControlMode;
	uint8_t     u8State;
	int16_t     s16TargetSpeedPU;
	int16_t     s16TargetSpeedActual;
	uint8_t     u8FilterCount;
	uint32_t    u32DutyHighSum;
	uint32_t    u32DutyPeriodSum;
	uint32_t    u32DutyHighAvg;
	uint32_t    u32DutyPeriodAvg;
	bool        bDataValid;
	bool        bFilterDone;
	int16_t     s16ActualSpeed;
	int16_t     s16ActualSpeedPu;
	uint16_t    u16PercentValue;
	uint16_t    u16PercentValueComplementation;
	uint16_t    u16Factor;
	uint8_t     u8DirectionDetCnt;
	uint8_t     u8PositiveCnt;
	uint8_t     u8NegativeCnt;
	TDirection  bMotorDirection;
	int8_t      s8SpeedPolarity;
}TSpeedControlData;

typedef struct _TSpeedControlPara
{
	uint16_t u16AppMaxSpeedActual;
	uint16_t u16AppMaxSpeedPU;
	uint8_t  u8SampleCount;
	uint8_t  u8DirectDetCnt;
	uint8_t  u8DesisionCnt;

}TSpeedControlPara;

typedef enum _SpeedControlState
{
	SCL_INIT,
	SCL_CAL,
	SCL_OUTPUT,
	SCL_IDLE

}SpeedControlState;


static TSpeedControlData        SCL_tSpeedControlData = {E_PWM_CONTROL, SCL_INIT,0,0,0,0,0,0,0,true,false,0,0,0,0,TRANSFER_FACTOR,0,0,0,FORWARD,SCL_POSITIVE_VAL};
static TInputCapture         *  SCL_tInputCaptureData = NULL;
static TSpeedControlPara        SCL_tSpeedControlPara = {MAX_SPD, M1_MAX_SPEED_PU,8,SCL_DIR_DET_CNT, SCL_DIR_DECISION_CNT};

static TFilterCoeff	       SCL_tTargetSpeedFilterCoef = FIR_tCalcFilterConst(1.0,3.2);
static TFilterData              SCL_tTargetSpeedFilter;

static bool                SCL_bDirectionDetAvailable = false;
void SCL_vMotorDirectionCheck(void)
{
	if((eM1_MainState <= MainState_Run)&&(eM1_RunSubState <= RunState_Ready))//after close loop
		{
			if(SCL_tSpeedControlData.u8DirectionDetCnt < SCL_tSpeedControlPara.u8DirectDetCnt )
				{
					SCL_tSpeedControlData.u8DirectionDetCnt ++;
					if( SCL_bGetDirection() == Bit_SET)
						{
							SCL_tSpeedControlData.u8NegativeCnt ++;

						}
					else
						{
							SCL_tSpeedControlData.u8PositiveCnt ++;
				
						}
				}
			else
				{
					if(SCL_tSpeedControlData.u8PositiveCnt >= SCL_tSpeedControlPara.u8DesisionCnt)
						{
							SCL_tSpeedControlData.bMotorDirection = FORWARD;
							SCL_tSpeedControlData.s8SpeedPolarity = SCL_POSITIVE_VAL;

						}
					else if(SCL_tSpeedControlData.u8NegativeCnt >= SCL_tSpeedControlPara.u8DesisionCnt)
						{
							SCL_tSpeedControlData.bMotorDirection = BACKWARD;
							SCL_tSpeedControlData.s8SpeedPolarity = SCL_NEGATIVE_VAL;

						}
					else
						{
						}
					SCL_tSpeedControlData.u8DirectionDetCnt = 0;
					SCL_tSpeedControlData.u8PositiveCnt     = 0;
					SCL_tSpeedControlData.u8NegativeCnt     = 0;
					
				}
		}

}

bool SCL_bDataFilter(void)
{
	
	//Average filter
	if(SCL_tSpeedControlData.u8FilterCount < SCL_tSpeedControlPara.u8SampleCount)
	{
		
		SCL_tSpeedControlData.u8FilterCount ++;
		SCL_tSpeedControlData.u32DutyHighSum   += SCL_tInputCaptureData->u16HighCount;
		SCL_tSpeedControlData.u32DutyPeriodSum += SCL_tInputCaptureData->u16PeriodCount;
	}
	else if(SCL_tSpeedControlData.u8FilterCount == SCL_tSpeedControlPara.u8SampleCount)
	{
		SCL_tSpeedControlData.u8FilterCount    = 0;
		SCL_tSpeedControlData.u32DutyHighAvg   =  SCL_tSpeedControlData.u32DutyHighSum	 >> 3;
		SCL_tSpeedControlData.u32DutyPeriodAvg =  SCL_tSpeedControlData.u32DutyPeriodSum >> 3;

		SCL_tSpeedControlData.u32DutyHighSum   = 0;
		SCL_tSpeedControlData.u32DutyPeriodSum = 0;
		
		SCL_tSpeedControlData.bFilterDone = true;
	
	}
	
	return SCL_tSpeedControlData.bFilterDone;

}

void SCL_vCalculateActualSpeedFromPU(void)
{

	SCL_tSpeedControlData.s16ActualSpeedPu = Motor_1st.FOC.s16SpdFilt;
	
	SCL_tSpeedControlData.s16ActualSpeed = SCL_tSpeedControlData.s16ActualSpeedPu*SPEED_MAX/32768;
	
	Drv_vSetOutputFreq(abs(SCL_tSpeedControlData.s16ActualSpeed));

}

void SCL_vCalcualteTargetSpeed(void)
{
	uint32_t u32TempValue = 0;

	if(SCL_tSpeedControlData.u32DutyPeriodAvg == 0)
		{
			if(GPIO_ReadInputDataBit(TIM3_CH1_PORT, TIM3_CH1_PIN) == true)//High Level, max speed
				{
					SCL_tSpeedControlData.s16TargetSpeedActual = 0;
					SCL_tSpeedControlData.s16TargetSpeedPU = 0;
					
				}
			else if(GPIO_ReadInputDataBit(TIM3_CH1_PORT, TIM3_CH1_PIN) == false)
				{
					SCL_tSpeedControlData.s16TargetSpeedActual = SCL_tSpeedControlPara.u16AppMaxSpeedActual;
					SCL_tSpeedControlData.s16TargetSpeedPU =    SCL_tSpeedControlPara.u16AppMaxSpeedPU;
				}
		}
	else
		{
			//Duty cycle(Q15(0.05) to Q15(0.45))
			SCL_tSpeedControlData.u16PercentValue = 0x8000*SCL_tSpeedControlData.u32DutyHighAvg/SCL_tSpeedControlData.u32DutyPeriodAvg;
			SCL_tSpeedControlData.u16PercentValueComplementation = 0x8000 - SCL_tSpeedControlData.u16PercentValue;
			if(SCL_tSpeedControlData.u16PercentValueComplementation < MIN_DUTY_PWM)
				{
					SCL_tSpeedControlData.s16TargetSpeedActual = 0;

				}
			else if(SCL_tSpeedControlData.u16PercentValueComplementation > MAX_DUTY_PWM)
				{
					SCL_tSpeedControlData.s16TargetSpeedActual = SCL_tSpeedControlData.s8SpeedPolarity*MAX_SPD;
				}
			else
				{
					//SCL_tSpeedControlData.s16TargetSpeedActual =  100*MLIB_Mul_Q15(SCL_tSpeedControlData.u16Factor,(SCL_tSpeedControlData.u16PercentValueComplementation - MIN_DUTY_PWM))/10 + MIN_SPD;
					SCL_tSpeedControlData.s16TargetSpeedActual =  SCL_tSpeedControlData.s8SpeedPolarity*(MLIB_Mul_Q15(SCL_tSpeedControlData.u16Factor,(SCL_tSpeedControlData.u16PercentValueComplementation - MIN_DUTY_PWM))*10 + MIN_SPD);
				}
			
		//Transfer to PU value and direction update
			SCL_tSpeedControlData.s16TargetSpeedPU = Q15(SCL_tSpeedControlData.s16TargetSpeedActual/SPEED_MAX);
			if(SCL_tSpeedControlData.s16TargetSpeedPU > M1_MAX_SPEED_PU)
			{
				SCL_tSpeedControlData.s16TargetSpeedPU = M1_MAX_SPEED_PU;
			}
			else if(SCL_tSpeedControlData.s16TargetSpeedPU < -M1_MAX_SPEED_PU)
			{
				SCL_tSpeedControlData.s16TargetSpeedPU = -M1_MAX_SPEED_PU;
			}

		//filter
		
		// SCL_tTargetSpeedFilter.tFilterInputValue = SCL_tSpeedControlData.s16TargetSpeedPU;
		// FIR_vCalcFilter(&SCL_tTargetSpeedFilter, (TFilterCoeff*)(SCL_tTargetSpeedFilterCoef));
		}
	Motor_1st.USER.s16VspCmd = SCL_tSpeedControlData.s16TargetSpeedPU;
}


void SCL_vSpeedControlPWMHandle(void)
{
	SCL_vMotorDirectionCheck();

	switch(SCL_tSpeedControlData.u8State)
		{
			case SCL_INIT:
				{
					SCL_tInputCaptureData = MIT_ptGetCaptureData();
					SCL_tSpeedControlData.u8State = SCL_CAL;
					break;

				}

			case SCL_CAL:
				{
					if(SCL_bDataFilter() == true)
						{
							SCL_tSpeedControlData.bFilterDone = false;
							if(SCL_tSpeedControlData.bDataValid == true)
								{
									//Calculate Targe speed
									SCL_vCalcualteTargetSpeed();
									SCL_tSpeedControlData.u8State = SCL_OUTPUT;
								}

						}
					break;
				}

			case SCL_OUTPUT:
				{
					SCL_vCalculateActualSpeedFromPU();
					SCL_tSpeedControlData.u8State = SCL_CAL;
					
					break;
				}

			default:
				{
					break;

				}

		}


}

void SCL_vSpeedControlModbusHandle(void) 
{
	if(abs(SCL_tSpeedControlData.s16TargetSpeedActual) < MIN_SPD)
		{

			SCL_tSpeedControlData.s16TargetSpeedActual  = 0;
			SCL_tSpeedControlData.s16TargetSpeedPU  	= 0;
		}
	else if(abs(SCL_tSpeedControlData.s16TargetSpeedActual) > MAX_SPD)
		{
			if(SCL_tSpeedControlData.s16TargetSpeedActual > MAX_SPD)
				{
					SCL_tSpeedControlData.s16TargetSpeedActual = MAX_SPD;
					SCL_tSpeedControlData.s16TargetSpeedPU     = M1_MAX_SPEED_PU;
				}
			else if(SCL_tSpeedControlData.s16TargetSpeedActual < -MAX_SPD)
				{
					SCL_tSpeedControlData.s16TargetSpeedActual = -MAX_SPD;
					SCL_tSpeedControlData.s16TargetSpeedPU     = -M1_MAX_SPEED_PU;
				}
		}
	else
		{
			SCL_tSpeedControlData.s16TargetSpeedPU     = Q15(SCL_tSpeedControlData.s16TargetSpeedActual/SPEED_MAX);
		}
	Motor_1st.USER.s16VspCmd = SCL_tSpeedControlData.s16TargetSpeedPU;
	//update actual speed
	SCL_vCalculateActualSpeedFromPU();
	
}

void SCL_vSpeedControlMasterModbusHandle(void) 
{
	if(abs(SCL_tSpeedControlData.s16TargetSpeedPU) < M1_MIN_SPEED_PU)
		{

			SCL_tSpeedControlData.s16TargetSpeedActual  = 0;
			SCL_tSpeedControlData.s16TargetSpeedPU  	= 0;
		}
	else if(abs(SCL_tSpeedControlData.s16TargetSpeedPU) > M1_MAX_SPEED_PU)
		{
			if(SCL_tSpeedControlData.s16TargetSpeedPU > M1_MAX_SPEED_PU)
				{
					SCL_tSpeedControlData.s16TargetSpeedActual = MAX_SPD;
					SCL_tSpeedControlData.s16TargetSpeedPU     = M1_MAX_SPEED_PU;
				}
			else if(SCL_tSpeedControlData.s16TargetSpeedPU < -M1_MAX_SPEED_PU)
				{
					SCL_tSpeedControlData.s16TargetSpeedActual = -MAX_SPD;
					SCL_tSpeedControlData.s16TargetSpeedPU     = -M1_MAX_SPEED_PU;
				}
		}
	else
		{
			SCL_tSpeedControlData.s16TargetSpeedActual     = SCL_tSpeedControlData.s16TargetSpeedPU*SPEED_MAX/32768;
		}
	Motor_1st.USER.s16VspCmd = SCL_tSpeedControlData.s16TargetSpeedPU;
	//update actual speed
	SCL_tSpeedControlData.s16ActualSpeedPu = Motor_1st.FOC.s16SpdFilt;
	
	SCL_tSpeedControlData.s16ActualSpeed = SCL_tSpeedControlData.s16ActualSpeedPu*SPEED_MAX/32768;
	RegData2[DATA_ACTUAL_SPD] = SCL_tSpeedControlData.s16ActualSpeedPu;
	
}

void SCL_vSpeedControlHandle(void)
{
	//Read mode
	SCL_tSpeedControlData.u8ControlMode    = RegData[DATA_MODE_SELECT];

	//Update Speed information
	RegData[DATA_MODE_SELECT] = SCL_tSpeedControlData.u8ControlMode;
	RegData[DATA_ACTUAL_SPD]  = SCL_tSpeedControlData.s16ActualSpeed;
	
	if(SCL_tSpeedControlData.u8ControlMode == E_PWM_CONTROL)
	{
		SCL_vSpeedControlPWMHandle();
	}
	else if(SCL_tSpeedControlData.u8ControlMode == E_MODEBUS_SLAVE_CONTROL)
	{
		SCL_tSpeedControlData.s16TargetSpeedActual = RegData[DATA_TARGET_SPD];
		SCL_vSpeedControlModbusHandle();
	}
	else if(SCL_tSpeedControlData.u8ControlMode == E_MODEBUS_MASTER_CONTROL)
	{
		AMS_vModbus2Statemachine();
		SCL_tSpeedControlData.s16TargetSpeedPU = RegData2[DATA_TARGET_SPD];
		SCL_vSpeedControlMasterModbusHandle();
		
	}
	else if(SCL_tSpeedControlData.u8ControlMode == E_DEBUG_CONTROL)
	{

	}

	if(Motor_1st.USER.s16VspCmd  == 0)
		{

			SCL_bDirectionDetAvailable = true;
		}

	if(SCL_bDirectionDetAvailable == true)
		{
			if(Motor_1st.USER.s16VspCmd > 0)
				{

					SCL_bDirectionDetAvailable = false;
					UFN_vForwardParamStartupInit();
				}
			else if(Motor_1st.USER.s16VspCmd < 0)
				{
					SCL_bDirectionDetAvailable = false;
					UFN_vBackwardParamStartupInit();
				}

		}

}


TDirection SCL_bGetMotorDirection(void)
{
	return SCL_tSpeedControlData.bMotorDirection;
} 

