/**
 * @file     STATEMACHINE.c
 * @author   MindMotion Motor Team : Wesson
 * @brief    This file provides the STATEMACHINE functions and test samples.
 *
 * @attention
 *
 * THE EXISTING FIRMWARE IS ONLY FOR REFERENCE, WHICH IS DESIGNED TO PROVIDE
 * CUSTOMERS WITH CODING INFORMATION ABOUT THEIR PRODUCTS SO THEY CAN SAVE
 * TIME. THEREFORE, MINDMOTION SHALL NOT BE LIABLE FOR ANY DIRECT, INDIRECT OR
 * CONSEQUENTIAL DAMAGES ABOUT ANY CLAIMS ARISING OUT OF THE CONTENT OF SUCH
 * HARDWARE AND/OR THE USE OF THE CODING INFORMATION CONTAINED HEREIN IN
 * CONNECTION WITH PRODUCTS MADE BY CUSTOMERS.
 *
 * <H2><CENTER>&COPY; COPYRIGHT MINDMOTION </CENTER></H2>
 */

/** Define to prevent recursive inclusion */
#define _STATEMACHINE_C_
/** Files includes */

#include "drv_inc.h"
#include "StateMachine.h"
#include "parameter.h"
#include "FOC_PMSM.h"
#include "Compile.h"
#include "Ramp.h"
#include "WindDetection.h"
#include "Current.h"
#include "ErrorReact.h"
/*******************************************************************************
 * Variables
 ******************************************************************************/
/*! @brief M1 structure */
m1_app_mainstate_t eM1_MainState = MainState_Init;                 /* Main State */
m1_run_substate_t eM1_RunSubState = RunState_Calib;                /* Run Sub State */

Q15_t    s16PosElEst_SMO;

/*! @brief Application sub-state function field - fast */
const pfcn_void_void s_STATE_FAST[4] = {M1_Fault_Fast, M1_Init_Fast, M1_Stop_Fast, M1_Run_Fast};

/*! @brief Application sub-state function field - slow */
const pfcn_void_void s_STATE_SLOW[4] = {M1_Fault_Slow, M1_Init_Slow, M1_Stop_Slow, M1_Run_Slow};


/*! @brief Application sub-state function field - fast */
static const pfcn_void_void s_M1_STATE_RUN_TABLE_FAST[8] = {M1_RunCalibFast, M1_RunReadyFast,
															M1_RunWindDetFast,  M1_RunBrakeFast,
                                                            M1_RunAlignFast, M1_RunStartupFast,
                                                            M1_RunSpinFast,  M1_RunFreewheelFast
                                                           };

/*! @brief Application sub-state function field - slow */
static const pfcn_void_void s_M1_STATE_RUN_TABLE_SLOW[8] = {M1_RunCalibSlow, M1_RunReadySlow,
															M1_RunWindDetSlow,  M1_RunBrakeSlow,
                                                            M1_RunAlignSlow, M1_RunStartupSlow,
                                                            M1_RunSpinSlow,  M1_RunFreewheelSlow
                                                           };
static uint8_t Motor_u8UserCommand = 0;
static uint8_t Motor_u8UserCommand2 = 0;
static uint8_t ucErrorCount = 0;
static uint8_t Motor_u8UnderErrorTest = 0;
static volatile int32_t Motor_s32DebugAngle1 = 0;
static volatile int32_t Motor_s32DebugAngle2 = 0;
	

static volatile bool Motor_bStartClose =true;

static int16_t Motor_s16DeltaAngle = 0;


#define MOT_OBR_0_05_ANGLE 0.05*32768/180

static uint8_t STO_u8Test = 0;

static TRmpData * MOT_ptAlternativeRampData = NULL;

static TERRORDATA * MOT_ptErrorData = NULL;


extern void MOT_vAngleSmoothSwitch(void);
/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * @brief Fault state called in fast state machine
 *
 * @param void  No input parameter
 *
 * @return None
 */
void M1_Fault_Fast(void)
{
    DISABLE_PWMOUT();
    M1_FaultDetection();
}

/*!
 * @brief State initialization routine called in fast state machine
 *
 * @param void  No input parameter
 *
 * @return None
 */
void M1_Init_Fast(void)
{
	/* Init error data pointer */
	MOT_ptErrorData = ERT_tGetErrorCode();

    DISABLE_PWMOUT();
    variable_reset(&Motor_1st);
    /*Switch from Init to Stop state*/
    M1_SwitchInitStop();
}

/*!
 * @brief Stop state routine called in fast state machine
 *
 * @param void  No input parameter
 *
 * @return None
 */
void M1_Stop_Fast(void)
{
    DISABLE_PWMOUT();
    Motor_1st.FOC.s16VbusAvg = Motor_1st.FOC.s16Vbus;
    M1_FaultDetection();

}


/*!
 * @brief Run state routine called in fast state machine
 *
 * @param void  No input parameter
 *
 * @return None
 */
void M1_Run_Fast(void)
{
    /*Run fast state machine*/
    s_M1_STATE_RUN_TABLE_FAST[eM1_RunSubState]();
    M1_FaultDetection();

}




/*!
 * @brief Fault state routine called in slow state machine
 *
 * @param void  No input parameter
 *
 * @return None
 */
void M1_Fault_Slow(void)
{

    Var_Filt(&Motor_1st);
    ERT_vErrorLEDDisplay();
	ERT_vErrorReactionRecovery();
    /* after fault condition ends wait defined time to clear fault state */

}

/*!
 * @brief Fault state routine called in slow state machine
 *
 * @param void  No input parameter
 *
 * @return None
 */
void M1_Init_Slow(void)
{
	
    Var_Filt(&Motor_1st);
#if (MOTOR_ALTERNATIVE_START_MODE ==	OPTION_PASSIVE)

#else
	RMP_vConfigureStartupAlgorithm(FORWARD);
	MOT_ptAlternativeRampData = RMP_ptGetRampDataPointer();

#endif
}

/*!
 * @brief Stop state routine called in slow state machine
 *
 * @param void  No input parameter
 *
 * @return None
 */
void M1_Stop_Slow(void)
{
    /* Get the speed command*/
    Motor_1st.FOC.s16SpdCmd = Motor_1st.USER.s16VspCmd;
    if(abs(Motor_1st.FOC.s16SpdCmd) > Q15(STOP_TO_RUN_SPEED / SPEED_MAX))
    {
        /* Switch from Stop to Run state*/
        M1_SwitchStopRun();
    }
    Var_Filt(&Motor_1st);
}



/*!
 * @brief Run state routine called in slow state machine
 *
 * @param void  No input parameter
 *
 * @return None
 */
void M1_Run_Slow(void)
{
    /* Get the speed command*/
    Motor_1st.FOC.s16SpdCmd = Motor_1st.USER.s16VspCmd;
    /* Run sub-state function */
    s_M1_STATE_RUN_TABLE_SLOW[eM1_RunSubState]();
}



/*!
 * @brief Switch from Fault to Stop state
 *
 * @param void  No input parameter
 *
 * @return None
 */
void M1_SwitchFaultStop(void)
{
    /* Type the code to do when going from the FAULT to the INIT state */
    eM1_MainState = MainState_Stop;
}


/*!
 * @brief Switch from Init to Stop state
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M1_SwitchInitStop(void)
{
    /* type the code to do when going from the INIT to the FAULT state */
    eM1_MainState = MainState_Stop;
}

/*!
 * @brief Transition from Stop to Fault state
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M1_SwitchStopFault(void)
{
    /* type the code to do when going from the STOP to the FAULT state */
    eM1_MainState = MainState_Fault;
}


/*!
 * @brief Switch from Stop to Run state
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M1_SwitchStopRun(void)
{
    /* type the code to do when going from the STOP to the RUN state */

    eM1_RunSubState = RunState_Calib;
    eM1_MainState = MainState_Run;

}


/*!
 * @brief Switch from Run to Fault state
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M1_SwitchRunFault(void)
{
    /* type the code to do when going from the RUN to the FAULT state */
    DISABLE_PWMOUT();
    eM1_MainState = MainState_Fault;
    eM1_RunSubState = RunState_Calib;
    variable_reset(&Motor_1st);
}

/*!
 * @brief Switch from Run to Stop state
 *
 * @param void  No input parameter
 *
 * @return None
 */
//static void M1_SwitchRunStop(void)
//{
/* type the code to do when going from the RUN to the STOP state */

//}

/*!
 * @brief Calibration process called in fast state machine as Run sub state
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M1_RunCalibFast(void)
{
    /* Type the code to do when in the RUN CALIB sub-state
       performing ADC offset calibration */
    {
        M1_SwitchRunCalibReady();
    }
}

/*!
 * @brief Ready state called in fast state machine as Run sub state
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M1_RunReadyFast(void)
{
	TDutyCycleConfig tWindDetectInitDuty = {PWM_DUTYCYCLE_0_PERCENT,PWM_DUTYCYCLE_0_PERCENT,PWM_DUTYCYCLE_0_PERCENT};
    /* Type the code to do when in the RUN READY sub-state */
    if(abs(Motor_1st.FOC.s16SpdCmd) > Q15(STOP_TO_RUN_SPEED / SPEED_MAX))
    {
        /* PreCharge Duty 50%*/
        ENABLE_PWMOUT();
        /*Charge BootCap*/
        if(Motor_1st.USER.u16PreChargeTime++ >= Motor_1st.USER.u16PreChargeTimeCmd)
        {

#if (WDN_WIND_DETECTION_METHOD_1 == OPTION_ACTIVE)
			//Brake
			
			TIM_bSetOutputConfig(PWM_CH_ALL_ON, tWindDetectInitDuty);

			M1_SwitchRunReadyWindDet();

			CUR_vCurRMSFilterInit();//filter init

#elif (WDN_WIND_DETECTION_METHOD_2 == OPTION_ACTIVE)

			WDN_vInitWindDetection();

			M1_SwitchRunReadyWindDet();

#else
			M1_SwitchRunReadyAlign();
#endif
        }
    }
    else
    {
        DISABLE_PWMOUT();
    }
    Motor_1st.FOC.SVM.s16U_Alpha = 0;
    Motor_1st.FOC.SVM.s16U_Beta = 0;

    MCF_Svm_7(&Motor_1st.FOC.SVM);

    PWM_Update(&Motor_1st);
}

/*!
 * @brief Alignment process called in fast state machine as Run sub state
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M1_RunAlignFast(void)
{
    /* type the code to do when in the RUN ALIGN sub-state */

    MCFLIB_Clark_Sat_S16(&Motor_1st.FOC.sIabc, &Motor_1st.FOC.sIAlBe);
    Motor_1st.FOC.s16CmdTheta = 0;

    Motor_1st.FOC.s16CmdTheta_Update = Motor_1st.FOC.s16CmdTheta;
    FOC_Current_Controller(&Motor_1st);
    Motor_1st.FOC.SVM.s16U_Alpha = Motor_1st.FOC.sVAlBe_cmd.s16Alpha;
    Motor_1st.FOC.SVM.s16U_Beta = Motor_1st.FOC.sVAlBe_cmd.s16Beta;
    /*SVPWM*/
    MCF_Svm_7(&Motor_1st.FOC.SVM);

    PWM_Update(&Motor_1st);
}

/*!
 * @brief Start-up process called in fast state machine as Run sub state
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M1_RunStartupFast(void)
{
    static    int16_t s16PosElEst;
    static    int16_t s16ThetaErr;
    static    sAngle_Trig     sAngle_temp;
    /* type the code to do when in the RUN STARTUP sub-state */
    /* Clark transform */
    MCFLIB_Clark_Sat_S16(&Motor_1st.FOC.sIabc, &Motor_1st.FOC.sIAlBe);
    /* open loop startup angle calcation*/
    Motor_1st.FOC.s16CmdTheta +=  MLIB_Mul_Q15(Motor_1st.FOC.s16SpeedRamp, SPEED_TO_THETA);
    /* no angle compensation*/
    Motor_1st.FOC.s16CmdTheta_Update = Motor_1st.FOC.s16CmdTheta;
    FOC_Current_Controller(&Motor_1st);
    Motor_1st.FOC.SVM.s16U_Alpha = Motor_1st.FOC.sVAlBe_cmd.s16Alpha;
    Motor_1st.FOC.SVM.s16U_Beta = Motor_1st.FOC.sVAlBe_cmd.s16Beta;

    Motor_1st.FOC.SVM.u16sector = MCF_Svm_7(&Motor_1st.FOC.SVM);

    PWM_Update(&Motor_1st);

    Motor_1st.FOC.SMO.s16Speed = Motor_1st.FOC.sPLL.s32Speed >> 16;
    /*use forced direction for startup */
    if(Motor_1st.FOC.s16SpdCmd > 0)
    {
        if(Motor_1st.FOC.SMO.s16Speed < 0)
        {
            Motor_1st.FOC.SMO.s16Speed = 1;
        }
    }
    else
    {
        if(Motor_1st.FOC.SMO.s16Speed > 0)
        {
            Motor_1st.FOC.SMO.s16Speed = -1;
        }
    }
    Motor_1st.FOC.SMO.s16Ialpha = Motor_1st.FOC.sIAlBe.s16Alpha;
    Motor_1st.FOC.SMO.s16Ibeta = Motor_1st.FOC.sIAlBe.s16Beta;
    Motor_1st.FOC.SMO.s16Valpha = Motor_1st.FOC.sVAlBe_cmd.s16Alpha;
    Motor_1st.FOC.SMO.s16Vbeta = Motor_1st.FOC.sVAlBe_cmd.s16Beta;

    MLIB_SMO(&Motor_1st.FOC.SMO);

    sAngle_temp = MLIB_SinCos(s16PosElEst);
    /* PLL calculation */
    s16ThetaErr = MLIB_Mul_Q15(-Motor_1st.FOC.SMO.s16Ealpha_est, sAngle_temp.s16Cos) - MLIB_Mul_Q15(Motor_1st.FOC.SMO.s16Ebeta_est, sAngle_temp.s16Sin);
    s16PosElEst = MLIB_PLL(s16ThetaErr, &Motor_1st.FOC.sPLL);

    s16PosElEst_SMO = s16PosElEst;

	//Calculate difference between open loop and monitor, angle synchronization
	Motor_s16DeltaAngle = Motor_1st.FOC.s16CmdTheta - s16PosElEst_SMO;

	
    //debug
    Motor_s32DebugAngle1 = Motor_1st.FOC.s16CmdTheta;
	Motor_s32DebugAngle2 = s16PosElEst_SMO;
}

/*!
 * @brief Spin state called in fast state machine as Run sub state
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M1_RunSpinFast(void)
{
    /* Type the code to do when in the RUN SPIN sub-state */
    /* Clark transform */
    MCFLIB_Clark_Sat_S16(&Motor_1st.FOC.sIabc, &Motor_1st.FOC.sIAlBe);

    PMSM_Obs(&Motor_1st);

	MOT_vAngleSmoothSwitch();//Angle synchronization

    FOC_Current_Controller(&Motor_1st);

    Motor_1st.FOC.SVM.s16U_Alpha = Motor_1st.FOC.sVAlBe_cmd.s16Alpha;
    Motor_1st.FOC.SVM.s16U_Beta = Motor_1st.FOC.sVAlBe_cmd.s16Beta;

    Motor_1st.FOC.SVM.u16sector = MCF_Svm_7(&Motor_1st.FOC.SVM);

    PWM_Update(&Motor_1st);

}

/*!
 * @brief Free-wheel process called in fast state machine as Run sub state
 *
 * @param void  No input parameter、
 *
 * @return None
 */
static void M1_RunFreewheelFast(void)
{
    DISABLE_PWMOUT();
    /* Clark transform */
    MCFLIB_Clark_Sat_S16(&Motor_1st.FOC.sIabc, &Motor_1st.FOC.sIAlBe);
    /* Type the code to do when in the RUN FREEWHEEL sub-state */
    Motor_1st.FOC.sIdq_cmd.s16Q = 0;

    Motor_1st.FOC.SVM.s16U_Alpha = 0;
    Motor_1st.FOC.SVM.s16U_Beta = 0;

    Motor_1st.FOC.SVM.u16sector = MCF_Svm_7(&Motor_1st.FOC.SVM);

    PWM_Update(&Motor_1st);

	//clean error count!
	ERT_vResetErrorCount();
}


/*!
 * @brief Calibration process called in slow state machine as Run sub state
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M1_RunCalibSlow(void)
{
    Var_Filt(&Motor_1st);
}

/*!
 * @brief Ready state called in slow state machine as Run sub state
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M1_RunReadySlow(void)
{
    Var_Filt(&Motor_1st);
}

/*!
 * @brief Alignment process called in slow state machine as Run sub state
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M1_RunAlignSlow(void)
{
    /* Iq command for align */
    Motor_1st.USER.u16AlignTimeCnt++;
#if (MOTOR_ALTERNATIVE_START_MODE == OPTION_PASSIVE)
    if(Motor_1st.FOC.s16SpdCmd > 0)
    {
        Motor_1st.FOC.sIdq_cmd.s16Q   += Motor_1st.USER.s16AlignCurAcc;
        if(Motor_1st.FOC.sIdq_cmd.s16Q > Motor_1st.USER.s16AlignIqCmd)
        {
            Motor_1st.FOC.sIdq_cmd.s16Q = Motor_1st.USER.s16AlignIqCmd;
        }
    }
    else if(Motor_1st.FOC.s16SpdCmd < 0)
    {
        Motor_1st.FOC.sIdq_cmd.s16Q     -= Motor_1st.USER.s16AlignCurAcc;
        if(Motor_1st.FOC.sIdq_cmd.s16Q < -Motor_1st.USER.s16AlignIqCmd)
        {
            Motor_1st.FOC.sIdq_cmd.s16Q = -Motor_1st.USER.s16AlignIqCmd;
        }
    }

#else

    if(Motor_1st.FOC.s16SpdCmd > 0)
    {
    
		RMP_vCalTrapezoidalCurve(&(MOT_ptAlternativeRampData->u16CurrentGradient), &(Motor_1st.FOC.sIdq_cmd.s16Q), &(Motor_1st.USER.s16AlignIqCmd));
		
        if(Motor_1st.FOC.sIdq_cmd.s16Q > Motor_1st.USER.s16AlignIqCmd)
        {
            Motor_1st.FOC.sIdq_cmd.s16Q = Motor_1st.USER.s16AlignIqCmd;
        }
    }
    else if(Motor_1st.FOC.s16SpdCmd < 0)
    {
        RMP_vCalTrapezoidalCurve(&(MOT_ptAlternativeRampData->u16CurrentGradient), &(Motor_1st.FOC.sIdq_cmd.s16Q), &(Motor_1st.USER.s16AlignIqCmd));
		
        if(Motor_1st.FOC.sIdq_cmd.s16Q < -Motor_1st.USER.s16AlignIqCmd)
        {
            Motor_1st.FOC.sIdq_cmd.s16Q = -Motor_1st.USER.s16AlignIqCmd;
        }
    }

#endif

    Var_Filt(&Motor_1st);
    Motor_1st.FOC.s16SpdFilt = 0;

    if(abs(Motor_1st.FOC.s16SpdCmd) < Q15(FREEWHEEL_SPEED / SPEED_MAX))
    {
        /* Switch from Align to Ready state*/
        M1_SwitchRunAlignReady();
    }

#if (MOTOR_ALTERNATIVE_START_MODE == OPTION_PASSIVE)

#else
	Motor_1st.USER.u16AlignTimeCmd = RMP_ALIGN_TIME;
#endif
    if(Motor_1st.USER.u16AlignTimeCnt > Motor_1st.USER.u16AlignTimeCmd)
    {
        /*Switch from Align to Startup state*/
        M1_SwitchRunAlignStartup();
        Motor_1st.USER.u16AlignTimeCnt = 0;
    }
}

/*!
 * @brief Start-up process called in slow state machine as Run sub state
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M1_RunStartupSlow(void)
{
    Motor_1st.USER.u16StartupTimeCnt++;

    /* Iq command for startup */
    if(Motor_1st.FOC.s16SpdCmd > 0)
    {
        Motor_1st.FOC.sIdq_cmd.s16Q = Motor_1st.USER.s16StartupIqCmd;
    }
    else if(Motor_1st.FOC.s16SpdCmd < 0)
    {
        Motor_1st.FOC.sIdq_cmd.s16Q = -Motor_1st.USER.s16StartupIqCmd;
    }
    /* acclelerate for startup speed command */
	
  #if(MOTOR_ALTERNATIVE_START_MODE == OPTION_PASSIVE)
     Motor_1st.FOC.s32SpeedRamp =   MLIB_Ramp_Q31(MLIB_Conv_Q31s(Motor_1st.FOC.s16SpdCmd), &Motor_1st.USER.sSpeedRamp);
     if (Motor_1st.FOC.s32SpeedRamp > MLIB_Conv_Q31s(Motor_1st.USER.s16StartupSpdCmdMax))
     {
       Motor_1st.FOC.s32SpeedRamp = MLIB_Conv_Q31s(Motor_1st.USER.s16StartupSpdCmdMax);
     }
     else if (Motor_1st.FOC.s32SpeedRamp < -MLIB_Conv_Q31s(Motor_1st.USER.s16StartupSpdCmdMax))
     {
       Motor_1st.FOC.s32SpeedRamp = -MLIB_Conv_Q31s(Motor_1st.USER.s16StartupSpdCmdMax);
     }        
      Motor_1st.FOC.s16SpeedRamp = MLIB_Conv_Q15l(Motor_1st.FOC.s32SpeedRamp);
   #else
	RMP_vCalcSmootherSinusoidalSpeedRampCurve(&(MOT_ptAlternativeRampData->u16SpeedStartupGradient),&Motor_1st.FOC.s16SpeedRamp,&Motor_1st.FOC.s16SpdCmd);
     if (Motor_1st.FOC.s16SpeedRamp > Motor_1st.USER.s16StartupSpdCmdMax)
     {
       	Motor_1st.FOC.s16SpeedRamp = Motor_1st.USER.s16StartupSpdCmdMax;
     }
	 else if (Motor_1st.FOC.s16SpeedRamp <- Motor_1st.USER.s16StartupSpdCmdMax)
 	 {

	 	Motor_1st.FOC.s16SpeedRamp = -Motor_1st.USER.s16StartupSpdCmdMax;
 	 }
   #endif
      
    Var_Filt(&Motor_1st);
    /* if speed command lower than freewheel speed , it will switch startup to freewheel*/
    if(abs(Motor_1st.FOC.s16SpdCmd) < Q15(FREEWHEEL_SPEED / SPEED_MAX))
    {
        M1_SwitchRunStartupFreewheel();
    }
    /* if speed filter higher than STARTUP_TO_SPIN_SPEED and speed command in the right direction , it will switch startup to spin*/
    if(((Motor_1st.FOC.s16SpdFilt >= Motor_1st.USER.u16OPToCLSwitchSpd) && (Motor_1st.FOC.s16SpdCmd > 0)) || ((Motor_1st.FOC.s16SpdFilt <= -Motor_1st.USER.u16OPToCLSwitchSpd) && (Motor_1st.FOC.s16SpdCmd < 0)))
    {
        if(Motor_1st.USER.u16StartupTimeCnt > Motor_1st.USER.u16StartupTimeCmd)
        {
        	if(Motor_bStartClose == true)
        		{
            		M1_SwitchRunStartupSpin();
        		}
            Motor_1st.USER.u16StartupTimeCnt = 0;

        }
    }
	else//add for project
		{
			if(Motor_1st.USER.u16StartupTimeCnt > Motor_1st.USER.u16StartupTimeCmd)
	        {                
				//write error code and Stop the Motor because starting up failure
				ERT_vErrorReport(MC_FAULT_STARTUP_LOSS_SYN);
				//jump to error state
				/* Switches to the FAULT state */
        		M1_SwitchRunFault();
	        }
		
		}
	
}


/*!
 * @brief Spin state called in slow state machine as Run sub state
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M1_RunSpinSlow(void)
{
    Var_Filt(&Motor_1st);
    /* Speed PI compute the Iq reference*/
    Motor_1st.FOC.sIdq_cmd.s16Q = MLIB_PIControl_Q15((Motor_1st.FOC.s16SpeedRamp - (Motor_1st.FOC.s16SpdFilt)), &Motor_1st.USER.sSpeed_PI);
#if(MOTOR_ALTERNATIVE_START_MODE == OPTION_PASSIVE)
	/* Execute the speed ramp*/
    Motor_1st.FOC.s32SpeedRamp =   MLIB_Ramp_Q31(MLIB_Conv_Q31s(Motor_1st.FOC.s16SpdCmd), &Motor_1st.USER.sSpeedRamp);
    Motor_1st.FOC.s16SpeedRamp = MLIB_Conv_Q15l(Motor_1st.FOC.s32SpeedRamp);
#else
	RMP_vCalcSmootherSinusoidalSpeedRampCurve(&(MOT_ptAlternativeRampData->u16SpeedRunGradient),&(Motor_1st.FOC.s16SpeedRamp) &(Motor_1st.FOC.s16SpdCmd));
#endif

    //if(abs(Motor_1st.FOC.s16SpdCmd) < Q15(FREEWHEEL_SPEED / SPEED_MAX))
    //{
        /*Switch from Startup to Free-wheel state*/
    //    M1_SwitchRunSpinFreewheel();
    //}

    if(abs(Motor_1st.FOC.s16SpeedRamp) < Q15(FREEWHEEL_SPEED / SPEED_MAX))
    {
        /*Switch from Startup to Free-wheel state*/
        M1_SwitchRunSpinFreewheel();
    }
}

/*!
 * @brief Free-wheel process called in slow state machine as Run sub state
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M1_RunFreewheelSlow(void)
{
    static    uint16_t u16FreeWheelTimeCnt;
    u16FreeWheelTimeCnt++;
    Var_Filt(&Motor_1st);
    if(u16FreeWheelTimeCnt > Motor_1st.USER.u16FreeWheelTimeCmd)
    {
        /* Switch from Spin to Free-wheel state */
        M1_SwitchRunFreewheelReady();
        u16FreeWheelTimeCnt = 0;
    }

	if(abs(Motor_1st.FOC.s16SpdCmd) > Q15(STOP_TO_RUN_SPEED / SPEED_MAX))
	{
		M1_SwitchRunFreewheelReady();
		u16FreeWheelTimeCnt = 0;
	}
}



/*!
 * @brief Switch from Calib to Ready state
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M1_SwitchRunCalibReady(void)
{
    /* Type the code to do when going from the RUN CALIB to the RUN READY sub-state */
    Motor_1st.USER.u16PreChargeTime  = 0;
    eM1_RunSubState = RunState_Ready;
}

/*!
 * @brief Switch from Ready to Align state
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M1_SwitchRunReadyAlign(void)
{
    /* Type the code to do when going from the RUN kRunState_Ready to the RUN kRunState_Align sub-state */
    variable_reset(&Motor_1st);

    eM1_RunSubState = RunState_Align;
}

/*!
 * @brief Switch from Align to Ready state
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M1_SwitchRunAlignReady(void)
{
    /* Type the code to do when going from the RUN kRunState_Align to the RUN kRunState_Ready sub-state */
    DISABLE_PWMOUT();
    Motor_1st.USER.u16PreChargeTime  = 0;
    eM1_RunSubState = RunState_Ready;
}

/*!
 * @brief Switch from Align to Startup state
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M1_SwitchRunAlignStartup(void)
{
    /* Type the code to do when going from the RUN kRunState_Align to the RUN kRunState_Startup sub-state */
    Motor_1st.FOC.sPLL.s32Theta = Motor_1st.FOC.s16CmdTheta << 16;
    if(Motor_1st.USER.s16VspCmd > 0)//forward
    {
        Motor_1st.USER.sSpeedRamp.s32RampUp         = Q31(STARTUP_SPEED_RAMP / SPEED_MAX / SLOWLOOP_FREQ);
        Motor_1st.USER.sSpeedRamp.s32RampDown       = Q31(STARTUP_SPEED_RAMP / SPEED_MAX / SLOWLOOP_FREQ);
    }
    else if(Motor_1st.USER.s16VspCmd < 0)//backward
    {
        Motor_1st.USER.sSpeedRamp.s32RampUp         = Q31(RAMP_UP_BACK / SPEED_MAX / SLOWLOOP_FREQ);
        Motor_1st.USER.sSpeedRamp.s32RampDown       = Q31(RAMP_DOWN_BACK / SPEED_MAX / SLOWLOOP_FREQ);  
    }
    eM1_RunSubState = RunState_Startup;
}


/*!
 * @brief Switch from Startup to Spin state
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M1_SwitchRunStartupSpin(void)
{
    /* Type the code to do when going from the RUN kRunState_Startup to the RUN kRunState_Spin sub-state */
    if(Motor_1st.USER.s16VspCmd > 0)//Forward
    {
        Motor_1st.USER.sSpeedRamp.s32RampUp        = Q31(RAMP_UP / SPEED_MAX / SLOWLOOP_FREQ);
        Motor_1st.USER.sSpeedRamp.s32RampDown      = Q31(RAMP_DOWN / SPEED_MAX / SLOWLOOP_FREQ);
    }
    else if(Motor_1st.USER.s16VspCmd < 0)//backward
    {
        Motor_1st.USER.sSpeedRamp.s32RampUp         = Q31(RAMP_UP_BACK / SPEED_MAX / SLOWLOOP_FREQ);
        Motor_1st.USER.sSpeedRamp.s32RampDown       = Q31(RAMP_DOWN_BACK / SPEED_MAX / SLOWLOOP_FREQ); 
    }
    Motor_1st.USER.sSpeedRamp.s32State   = Motor_1st.FOC.s16SpdFilt << 16;
    Motor_1st.USER.sSpeed_PI.s32IAccK_1 = Motor_1st.FOC.sIdq_cmd.s16Q << 16;
    eM1_RunSubState = RunState_Spin;
}

/*!
 * @brief Switch from Startup to Free-wheel state
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M1_SwitchRunStartupFreewheel(void)
{
    eM1_RunSubState = RunState_Freewheel;
}

/*!
 * @brief Switch from Spin to Free-wheel state
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M1_SwitchRunSpinFreewheel(void)
{
    /* Type the code to do when going from the RUN SPIN to the RUN FREEWHEEL sub-state */
    DISABLE_PWMOUT();
    eM1_RunSubState = RunState_Freewheel;
}

/*!
 * @brief Switch from Free-wheel to Ready state
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M1_SwitchRunFreewheelReady(void)
{
    /* Type the code to do when going from the RUN kRunState_FreeWheel to the RUN kRunState_Ready sub-state */
    variable_reset(&Motor_1st);
    Motor_1st.USER.u16PreChargeTime  = 0;
    eM1_RunSubState = RunState_Ready;
}

/*****************************************update by WB**************************************************/

/*!
 * @brief Switch from Stop to Run state
 *
 * @param void  No input parameter
 *
 * @return None
 */
 void M1_SwitchRunReadyWindDet(void)
{
    /* type the code to do when going from the STOP to the Wind Det state */

	eM1_RunSubState = RunState_WindDet;


}

/*!
 * @brief Switch from Stop to Run state
 *
 * @param void  No input parameter
 *
 * @return None
 */
 void M1_SwitchRunWindDetBrake(void)
{
    /* type the code to do when going from the Wind Det to the RUN state */

    eM1_RunSubState = RunState_Brake;

}

/*!
 * @brief Switch from brake to stop state
 *
 * @param void  No input parameter
 *
 * @return None
 */
 void M1_SwitchRunWindDetAlign(void)
{
    /* type the code to do when going from the INIT to the FAULT state */
    eM1_RunSubState = RunState_Align;


}

/*!
 * @brief Switch from brake to Run state
 *
 * @param void  No input parameter
 *
 * @return None
 */
 void M1_SwitchRunBrakeReady(void)
{
    /* type the code to do when going from the STOP to the Wind Det state */

    /* type the code to do when going from the Wind Det to the RUN state */

    eM1_RunSubState = RunState_Ready;

}

/*!
 * @brief Switch from brake to Run state
 *
 * @param void  No input parameter
 *
 * @return None
 */

void M1_SwitchRunWindDetReady(void)
{
	eM1_RunSubState = RunState_Ready;

}


/*!
 * @brief Switch from brake to Run state
 *
 * @param void  No input parameter
 *
 * @return None
 */
 void M1_SwitchRunWindDetSpin(void)
{
    /* type the code to do when going from the STOP to the Wind Det state */

    /* type the code to do when going from the Wind Det to the RUN state */


    /* Type the code to do when going from the RUN kRunState_Startup to the RUN kRunState_Spin sub-state */
    Motor_1st.USER.sSpeedRamp.s32RampUp        = Q31(RAMP_UP / SPEED_MAX / SLOWLOOP_FREQ);
    Motor_1st.USER.sSpeedRamp.s32RampDown      = Q31(RAMP_DOWN / SPEED_MAX / SLOWLOOP_FREQ);
    Motor_1st.USER.sSpeedRamp.s32State   = Motor_1st.FOC.s16SpdFilt << 16;
    Motor_1st.USER.sSpeed_PI.s32IAccK_1 = Motor_1st.FOC.sIdq_cmd.s16Q << 16;

    eM1_RunSubState = RunState_Spin;

}


/*!
 * @brief Stop state routine called in slow state machine
 *
 * @param void  No input parameter
 *
 * @return None
 */
void M1_RunWindDetSlow(void)
{
	#if (WDN_WIND_DETECTION_METHOD_1 == OPTION_ACTIVE)
	STO_u8Test ++;
    /* Get the speed command*/
    Motor_1st.FOC.s16SpdCmd = Motor_1st.USER.s16VspCmd;
    if(abs(Motor_1st.FOC.s16SpdCmd) > Q15(STOP_TO_RUN_SPEED / SPEED_MAX))
    {
    	/* Implement Wind detection*/
		if(WDN_bEstimateStartupScenario() == true)
			{
				WDN_vAnalyzeStartPower();
				/* Switch from Stop to Run state*/
				M1_SwitchRunWindDetAlign();
				
		
			}
    }
	else
		{
			//add your code
			M1_SwitchRunWindDetReady();
		}
	#elif (WDN_WIND_DETECTION_METHOD_2 == OPTION_ACTIVE)
	
	#endif
    Var_Filt(&Motor_1st);
}

/*!
 * @brief Stop state routine called in slow state machine
 *
 * @param void  No input parameter
 *
 * @return None
 */
void M1_RunWindDetFast(void)
{
	#if (WDN_WIND_DETECTION_METHOD_1 == OPTION_ACTIVE)
    /* Get the speed command*/
    /*Motor_1st.FOC.s16SpdCmd = Motor_1st.USER.s16VspCmd;
    if(abs(Motor_1st.FOC.s16SpdCmd) > Q15(STOP_TO_RUN_SPEED / SPEED_MAX))
    {

    }*/
	#elif (WDN_WIND_DETECTION_METHOD_2 == OPTION_ACTIVE)
	    /* Get the speed command*/
    Motor_1st.FOC.s16SpdCmd = Motor_1st.USER.s16VspCmd;
    if(abs(Motor_1st.FOC.s16SpdCmd) > Q15(STOP_TO_RUN_SPEED / SPEED_MAX))
    {
    
		WDN_vWindDetection();

    }
	else
	{
		//add your code
		M1_SwitchRunWindDetReady();

	}

	#endif
    Var_Filt(&Motor_1st);
}

/*!
 * @brief Run state routine called in fast state machine
 *
 * @param void  No input parameter
 *
 * @return None
 */
void M1_RunBrakeFast(void)
{
	if(TIM_bBrakeUpdate() == true)//brake is finished
		{
			//check if target speed is bigger than threshold
			if(abs(Motor_1st.FOC.s16SpdCmd) > Q15(STOP_TO_RUN_SPEED / SPEED_MAX))
				{
					//switch to align
					M1_SwitchRunWindDetAlign();
				}
			else
				{

				}
		}

}


/*!
 * @brief Run state routine called in slow state machine
 *
 * @param void  No input parameter
 *
 * @return None
 */
void M1_RunBrakeSlow(void)
{

}

/*!
 * @brief Run state routine called in slow state machine
 *
 * @param void  No input parameter
 *
 * @return None
 */
void M1_SwitchForceToStop(void)
{
    DISABLE_PWMOUT();
    eM1_MainState = MainState_Stop;
    eM1_RunSubState = RunState_Calib;
    variable_reset(&Motor_1st);

}

/*!
 * @brief Switch from Run to Fault state
 *
 * @param void  No input parameter
 *
 * @return None
 */
void M1_SwitchForceFault(void)
{
    /* type the code to do when going from the RUN to the FAULT state */
    DISABLE_PWMOUT();
    eM1_MainState = MainState_Fault;
    eM1_RunSubState = RunState_Calib;
    variable_reset(&Motor_1st);
}



/*!
 * @brief Fault detention routine - check various faults
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M1_FaultDetection(void)
{
    static uint16_t u16OvCntr;
	static uint16_t u16UvCntr;
    /* Clearing actual faults before detecting them again  */
    /* Clear all faults */
    
		ERT_vResetErrorCode(MC_FAULT_U_DCBUS_UNDER|MC_FAULT_U_DCBUS_UNDER|MC_FAULT_I_DCBUS_OVER);

    if(Motor_1st.FOC.s16VbusAvg > Motor_1st.USER.s16OVPCmd)
    {
        u16OvCntr++;
        if(u16OvCntr > 100)  //0.01s
        {
            
            ERT_vErrorReport(MC_FAULT_U_DCBUS_UNDER);
        }
    }
    else
    {
        u16OvCntr = 0;

    }

    if(Motor_1st.FOC.s16VbusAvg < Motor_1st.USER.s16UVPCmd)
    {
        u16UvCntr ++;
        if(u16UvCntr > 100)  //0.01s
        {
            ERT_vErrorReport(MC_FAULT_U_DCBUS_UNDER);
        }
    }
    else
    {
        u16UvCntr = 0;
    }
    /*  Trigger hardware I_DCBUS overcurrent protection */
    if(Motor_1st.PWMx.Timer->SR & TIM_IT_Break)
    {
        Motor_u8UnderErrorTest ++;
        TIM_ITConfig((Motor_1st.PWMx.Timer), TIM_IT_Break, DISABLE);
        Motor_1st.PWMx.Timer->SR = ~(TIM_IT_Break);

        ERT_vErrorReport(MC_FAULT_I_DCBUS_OVER);
    }

	//Collect other error information

}

/**
 * @brief Updata PWM value to compare register
 * @param None
 * @retval None
 */
void PWM_Update(Motor_TypeDef *Motor)
{
    Motor->PWMx.Timer->CCR1 = MLIB_Mul_Q15((INT16_MAX - Motor->FOC.SVM.s16DutyA), Motor->USER.u16PWMPeriod);
    Motor->PWMx.Timer->CCR2 = MLIB_Mul_Q15((INT16_MAX - Motor->FOC.SVM.s16DutyB), Motor->USER.u16PWMPeriod);
    Motor->PWMx.Timer->CCR3 = MLIB_Mul_Q15((INT16_MAX - Motor->FOC.SVM.s16DutyC), Motor->USER.u16PWMPeriod);
}

void MOT_vAngleSmoothSwitch(void)
{
	if(Motor_s16DeltaAngle > MOT_OBR_0_05_ANGLE)
		{
			Motor_s16DeltaAngle -= MOT_OBR_0_05_ANGLE;
		}
	else if(Motor_s16DeltaAngle < -MOT_OBR_0_05_ANGLE)
		{
			Motor_s16DeltaAngle += MOT_OBR_0_05_ANGLE;
		}

	Motor_1st.FOC.s16CmdTheta = Motor_1st.FOC.s16CmdTheta + Motor_s16DeltaAngle;
}



