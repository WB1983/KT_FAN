/**
 * @file     user_function.c
 * @author   MindMotion Motor Team : Wesson
 * @brief    This file provides all the functions for the board level support package.
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
#define _USER_FUNCTION_C_

/** Files includes */
#include "user_function.h"
#include "parameter.h"

/**
  * @brief  Initializes the Motor control parameters 
  * @retval None
  */

void Init_Parameter(Motor_TypeDef *Motor)
{

    /* PWM setting */
    Motor->PWMx.Timer                                                   = TIM1;      /*Select timer of PWM */

    Motor->USER.u16PWMFreq                                              = PWMFREQ;    /* PWM  Frequence*/
    Motor->USER.u16DeadTime                                             = SYSCLK_HSI_96MHz / 1000000 * DEAD_TIME;  /* Dead time */

    Motor->USER.sSpeed_Filt.u16Sh                                       = 5;
    Motor->USER.sSpeed_Filt.s32Acc                                      = 0;
	
    Motor->USER.sVbus_Filt.u16Sh                                        = 3;
    Motor->USER.sVbus_Filt.s32Acc                                       = 0;
    Motor->USER.sSpeedRamp.s32RampUp                                    = Q31(RAMP_UP / SPEED_MAX / SLOWLOOP_FREQ);
    Motor->USER.sSpeedRamp.s32RampDown                                  = Q31(RAMP_DOWN / SPEED_MAX / SLOWLOOP_FREQ);

    /* Protection  */
    Motor->USER.s16OVPCmd                                               = Q15(DCBUS_OVER / UDC_MAX);
    Motor->USER.s16UVPCmd                                               = Q15(DCBUS_UNDER / UDC_MAX);

    /* Iq Limit  */
    Motor->USER.s16IqLimitMax                                           = Q15(SPD_PI_LIMIT / I_MAX);

    /* Precharge */
    Motor->USER.u16PreChargeTimeCmd                                     = PWMFREQ / 1000.0 * PRECHARGE_TIME;

    /* Alignment setting */
    Motor->USER.s16StartupSpdCmdMax                                     = Q15(MAXSTARTUP_SPEED / SPEED_MAX);
    //Motor->USER.s16AlignIqCmd                                           = Q15(IQ_ALIGN / I_MAX);
    Motor->USER.u16AlignTimeCmd                                         = ALIGN_TIME * SLOWLOOP_FREQ;
    //Motor->USER.s16StartupIqCmd                                         = Q15(IQ_STARTUP / I_MAX);
    Motor->USER.u16StartupTimeCmd                                       = STARTUP_TIME * SLOWLOOP_FREQ;

    Motor->USER.u16FreeWheelTimeCmd                                     = FREEWHEEL_TIME * SLOWLOOP_FREQ;
    Motor->USER.u16FaultReleaseTimeCmd                                  = FAULTRELEASE_TIME * SLOWLOOP_FREQ;
    Motor->USER.u16OPToCLSwitchSpd                                        = Q15(STARTUP_TO_SPIN_SPEED / SPEED_MAX);

    /* VS Limit */
    Motor->USER.s16VqLimitMax                                           = Q15(Q_PI_LIMIT / U_MAX);
    Motor->USER.s16VdLimitMax                                           = Q15(D_PI_LIMIT / U_MAX);
    Motor->USER.s16DutyLimitCmd                                         = Q15(MAX_DUTY/1.15);//Q15((2*MAX_DUTY-1)/1.15);

    Motor->USER.u16PWMPeriod                                            = SYSCLK_HSI_96MHz / 2 / Motor->USER.u16PWMFreq;
    Motor->USER.u16SlowLoopDiv                                          = PWMFREQ / SLOWLOOP_FREQ;

    Motor->USER.sSpeed_PI.s16Kp                                         = M1_SPEED_KP;
		Motor->USER.sSpeed_PI.i16Kp_Shift                                   = M1_SPEED_KP_SHIFT;
		
    Motor->USER.sSpeed_PI.s16Ki                                         = M1_SPEED_KI;
		Motor->USER.sSpeed_PI.i16Ki_Shift                                   = M1_SPEED_KI_SHIFT;

    Motor->USER.sSpeed_PI.s16LowerLim                                   = MLIB_Neg_Q15(Motor->USER.s16IqLimitMax);
    Motor->USER.sSpeed_PI.s16UpperLim                                   = Motor->USER.s16IqLimitMax;
    Motor->USER.sSpeed_PI.s32IAccK_1                                    = 0;

    Motor->USER.sId_PI.s16Kp                                            = M1_ID_KP;
    Motor->USER.sId_PI.s16Ki                                            = M1_ID_KI;
		Motor->USER.sId_PI.i16Kp_Shift                                      = M1_ID_KP_SHIFT;
    Motor->USER.sId_PI.i16Ki_Shift                                      = M1_ID_KI_SHIFT;
    Motor->USER.sId_PI.s16LowerLim                                      = MLIB_Neg_Q15(Motor->USER.s16VdLimitMax);
    Motor->USER.sId_PI.s16UpperLim                                      = Motor->USER.s16VdLimitMax;
    Motor->USER.sId_PI.s32IAccK_1                                       = 0;

    Motor->USER.sIq_PI.s16Kp                                            = M1_IQ_KP;
    Motor->USER.sIq_PI.s16Ki                                            = M1_IQ_KI;
		Motor->USER.sIq_PI.i16Kp_Shift                                      = M1_IQ_KP_SHIFT;
    Motor->USER.sIq_PI.i16Ki_Shift                                      = M1_IQ_KI_SHIFT;
    Motor->USER.sIq_PI.s16LowerLim                                      = MLIB_Neg_Q15(Motor->USER.s16VqLimitMax);
    Motor->USER.sIq_PI.s16UpperLim                                      = Motor->USER.s16VqLimitMax;
    Motor->USER.sIq_PI.s32IAccK_1                                       = 0;

    /* SMO setting */
    Motor->FOC.SMO.s16Fs                                                = Q15(1.0 - Rs / Ls / PWMFREQ);
    Motor->FOC.SMO.s16Gs                                                = Q15(U_MAX / Ls / I_MAX / PWMFREQ);
    Motor->FOC.SMO.s16ErrMax                                            = SMO_ERR_MAX;
    Motor->FOC.SMO.s16Kslide                                            = SLIDE_GIAN;
		Motor->FOC.SMO.s16IerrGain                                          = IERR_GIAN;
    Motor->FOC.SMO.i16IerrGainShift                                     = IERR_GIAN_SHIFT;

    Motor->FOC.SMO.s16Ialpha_est                                        = 0;
    Motor->FOC.SMO.s16Ibeta_est                                         = 0;
    Motor->FOC.SMO.s16Ealpha_est                                        = 0;
    Motor->FOC.SMO.s16Ebeta_est                                         = 0;
    Motor->FOC.SMO.s16Speed                                             = 0;
}

/**
  * @brief  Reset the Motor control parameters 
  * @retval None
  */


void variable_reset(Motor_TypeDef *Motor)
{

    Motor_1st.USER.sSpeed_PI.s32IAccK_1                                 = 0;
    Motor->USER.sId_PI.s32IAccK_1                                       = 0;
    Motor->USER.sIq_PI.s32IAccK_1                                       = 0;
    Motor->USER.sSpeedRamp.s32State                                     = 0;
    Motor->FOC.sVdq.s16Q                                                = 0;
    Motor->FOC.sVdq.s16D                                                = 0;
    Motor->FOC.sIdq_cmd.s16Q                                            = 0;
    Motor->FOC.sIdq_cmd.s16D                                            = 0;
    Motor->USER.sSpeed_Filt.s32Acc                                      = 0;


    /*SMO observer and pll parameter*/
    Motor_1st.FOC.sPLL.s16PGain                                         = M1_TO_KP_GAIN;
    Motor_1st.FOC.sPLL.i16PGainSh                                       = M1_TO_KP_SHIFT;
    Motor_1st.FOC.sPLL.s16IGain                                         = M1_TO_KI_GAIN;
    Motor_1st.FOC.sPLL.i16IGainSh                                       = M1_TO_KI_SHIFT;
    Motor_1st.FOC.sPLL.s16ThGain                                        = M1_TO_THETA_GAIN;
    Motor_1st.FOC.sPLL.i16ThGainSh                                      = M1_TO_THETA_SHIFT;
    Motor_1st.FOC.sPLL.s32I_1                                           = 0;
    Motor_1st.FOC.sPLL.s32Speed                                         = 0;
		
    Motor_1st.FOC.SMO.s16Speed                                          = 0;
    Motor_1st.FOC.s16SpeedRamp                                          = 0;
    Motor_1st.FOC.s32SpeedRamp                                          = 0;
    Motor_1st.FOC.s16SpdFilt                                            = 0;
    Motor_1st.USER.u16AlignTimeCnt                                      = 0;


    Motor_1st.USER.s16AlignCurAcc                                       = Q15(ALIGN_CURRENT_RAMP / I_MAX / SLOWLOOP_FREQ);

    Motor->FOC.s16CmdTheta                                              = 0;

    Motor_1st.FOC.SVM.s16DutyA                                          = Q15(0.5);
    Motor_1st.FOC.SVM.s16DutyB                                          = Q15(0.5);
    Motor_1st.FOC.SVM.s16DutyC                                          = Q15(0.5);

}

/**
* @brief Filter Motor  parameters : motor speed ,bus voltage  eg 
  * @retval None
  */


void Var_Filt(Motor_TypeDef *Motor)
{
    Motor->FOC.s16VbusAvg = MLIB_FilterMA_Q15(Motor->FOC.s16Vbus, &Motor->USER.sVbus_Filt);
    Motor_1st.FOC.s16SpdFilt = MLIB_FilterMA_Q15(Motor_1st.FOC.SMO.s16Speed, &Motor->USER.sSpeed_Filt);
}

/**
 * setting backwarding running parameters
*/

void UFN_vBackwardParamStartupInit(void)
{
    Motor_1st.USER.sSpeedRamp.s32RampUp                                    = Q31(RAMP_UP_BACK / SPEED_MAX / SLOWLOOP_FREQ);
    Motor_1st.USER.sSpeedRamp.s32RampDown                                  = Q31(RAMP_DOWN_BACK / SPEED_MAX / SLOWLOOP_FREQ);

    /* Alignment setting */
    //Motor->USER.s16StartupSpdCmdMax                                     = Q15(MAXSTARTUP_SPEED / SPEED_MAX);
    Motor_1st.USER.s16AlignIqCmd                                           = Q15(IQ_ALIGN_BACK / I_MAX);
    //Motor->USER.u16AlignTimeCmd                                         = ALIGN_TIME * SLOWLOOP_FREQ;
    Motor_1st.USER.s16StartupIqCmd                                         = Q15(IQ_STARTUP_BACK / I_MAX);
    //Motor->USER.u16StartupTimeCmd                                       = STARTUP_TIME * SLOWLOOP_FREQ;

    Motor_1st.USER.u16FreeWheelTimeCmd                                     = FREEWHEEL_TIME_BACK * SLOWLOOP_FREQ;
    //Motor->USER.u16FaultReleaseTimeCmd                                  = FAULTRELEASE_TIME * SLOWLOOP_FREQ;
    //Motor->USER.u16OPToCLSwitchSpd                                        = Q15(STARTUP_TO_SPIN_SPEED / SPEED_MAX);

    /* VS Limit */
    // Motor->USER.s16VqLimitMax                                           = Q15(Q_PI_LIMIT / U_MAX);
    // Motor->USER.s16VdLimitMax                                           = Q15(D_PI_LIMIT / U_MAX);
    // Motor->USER.s16DutyLimitCmd                                         = Q15(MAX_DUTY/1.15);//Q15((2*MAX_DUTY-1)/1.15);

    // Motor->USER.u16PWMPeriod                                            = SYSCLK_HSI_96MHz / 2 / Motor->USER.u16PWMFreq;
    // Motor->USER.u16SlowLoopDiv                                          = PWMFREQ / SLOWLOOP_FREQ;

    Motor_1st.USER.sSpeed_PI.s16Kp                                         = M1_SPEED_KP_BACK;
	Motor_1st.USER.sSpeed_PI.i16Kp_Shift                                   = M1_SPEED_KP_SHIFT_BACK;
		
    Motor_1st.USER.sSpeed_PI.s16Ki                                         = M1_SPEED_KI_BACK;
	Motor_1st.USER.sSpeed_PI.i16Ki_Shift                                   = M1_SPEED_KI_SHIFT_BACK;

    // Motor->USER.sSpeed_PI.s16LowerLim                                   = MLIB_Neg_Q15(Motor->USER.s16IqLimitMax);
    // Motor->USER.sSpeed_PI.s16UpperLim                                   = Motor->USER.s16IqLimitMax;
    // Motor->USER.sSpeed_PI.s32IAccK_1                                    = 0;

    Motor_1st.USER.sId_PI.s16Kp                                            = M1_ID_KP_BACK;
    Motor_1st.USER.sId_PI.s16Ki                                            = M1_ID_KI_BACK;
	Motor_1st.USER.sId_PI.i16Kp_Shift                                      = M1_ID_KP_SHIFT_BACK;
    Motor_1st.USER.sId_PI.i16Ki_Shift                                           = M1_ID_KI_SHIFT_BACK;
    // Motor->USER.sId_PI.s16LowerLim                                      = MLIB_Neg_Q15(Motor->USER.s16VdLimitMax);
    // Motor->USER.sId_PI.s16UpperLim                                      = Motor->USER.s16VdLimitMax;
    // Motor->USER.sId_PI.s32IAccK_1                                       = 0;

    Motor_1st.USER.sIq_PI.s16Kp                                            = M1_IQ_KP_BACK;
    Motor_1st.USER.sIq_PI.s16Ki                                            = M1_IQ_KI_BACK;
    Motor_1st.USER.sIq_PI.i16Kp_Shift                                      = M1_IQ_KP_SHIFT_BACK;
    Motor_1st.USER.sIq_PI.i16Ki_Shift                                      = M1_IQ_KI_SHIFT_BACK;
    // Motor->USER.sIq_PI.s16LowerLim                                      = MLIB_Neg_Q15(Motor->USER.s16VqLimitMax);
    // Motor->USER.sIq_PI.s16UpperLim                                      = Motor->USER.s16VqLimitMax;
    // Motor->USER.sIq_PI.s32IAccK_1                                       = 0;

    /* SMO setting */
    // Motor->FOC.SMO.s16Fs                                                = Q15(1.0 - Rs / Ls / PWMFREQ);
    // Motor->FOC.SMO.s16Gs                                                = Q15(U_MAX / Ls / I_MAX / PWMFREQ);
    // Motor->FOC.SMO.s16ErrMax                                            = SMO_ERR_MAX;
    // Motor->FOC.SMO.s16Kslide                                            = SLIDE_GIAN;
		// Motor->FOC.SMO.s16IerrGain                                          = IERR_GIAN;
    // Motor->FOC.SMO.i16IerrGainShift                                     = IERR_GIAN_SHIFT;

    // Motor->FOC.SMO.s16Ialpha_est                                        = 0;
    // Motor->FOC.SMO.s16Ibeta_est                                         = 0;
    // Motor->FOC.SMO.s16Ealpha_est                                        = 0;
    // Motor->FOC.SMO.s16Ebeta_est                                         = 0;
    // Motor->FOC.SMO.s16Speed                                             = 0;

	    /*SMO observer and pll parameter*/
    Motor_1st.FOC.sPLL.s16PGain                                         = M1_TO_KP_GAIN_BACK;
    //Motor_1st.FOC.sPLL.i16PGainSh                                       = M1_TO_KP_SHIFT_BACK;
    Motor_1st.FOC.sPLL.s16IGain                                         = M1_TO_KI_GAIN_BACK;
    //Motor_1st.FOC.sPLL.i16IGainSh                                       = M1_TO_KI_SHIFT_BACK;
    Motor_1st.FOC.sPLL.s16ThGain                                        = M1_TO_THETA_GAIN_BACK;
    //Motor_1st.FOC.sPLL.i16ThGainSh                                      = M1_TO_THETA_SHIFT_BACK;
}


void UFN_vForwardParamStartupInit(void)
{
  	Motor_1st.USER.sSpeedRamp.s32RampUp									   = Q31(RAMP_UP / SPEED_MAX / SLOWLOOP_FREQ);
  	Motor_1st.USER.sSpeedRamp.s32RampDown 								   = Q31(RAMP_DOWN / SPEED_MAX / SLOWLOOP_FREQ);

  /* Alignment setting */
    // Motor->USER.s16StartupSpdCmdMax                                     = Q15(MAXSTARTUP_SPEED / SPEED_MAX);
     Motor_1st.USER.s16AlignIqCmd                                           = Q15(IQ_ALIGN / I_MAX);
    // Motor->USER.u16AlignTimeCmd                                         = ALIGN_TIME * SLOWLOOP_FREQ;
     Motor_1st.USER.s16StartupIqCmd                                         = Q15(IQ_STARTUP / I_MAX);
    // Motor->USER.u16StartupTimeCmd                                       = STARTUP_TIME * SLOWLOOP_FREQ;

     Motor_1st.USER.u16FreeWheelTimeCmd                                     = FREEWHEEL_TIME * SLOWLOOP_FREQ;
    // Motor->USER.u16FaultReleaseTimeCmd                                  = FAULTRELEASE_TIME * SLOWLOOP_FREQ;
    // Motor->USER.u16OPToCLSwitchSpd                                        = Q15(STARTUP_TO_SPIN_SPEED / SPEED_MAX);

    // /* VS Limit */
    // Motor->USER.s16VqLimitMax                                           = Q15(Q_PI_LIMIT / U_MAX);
    // Motor->USER.s16VdLimitMax                                           = Q15(D_PI_LIMIT / U_MAX);
    // Motor->USER.s16DutyLimitCmd                                         = Q15(MAX_DUTY/1.15);//Q15((2*MAX_DUTY-1)/1.15);

    // Motor->USER.u16PWMPeriod                                            = SYSCLK_HSI_96MHz / 2 / Motor->USER.u16PWMFreq;
    // Motor->USER.u16SlowLoopDiv                                          = PWMFREQ / SLOWLOOP_FREQ;

     Motor_1st.USER.sSpeed_PI.s16Kp                                         = M1_SPEED_KP;
	 Motor_1st.USER.sSpeed_PI.i16Kp_Shift                                   = M1_SPEED_KP_SHIFT;
		
     Motor_1st.USER.sSpeed_PI.s16Ki                                         = M1_SPEED_KI;
	 Motor_1st.USER.sSpeed_PI.i16Ki_Shift                                   = M1_SPEED_KI_SHIFT;

    // Motor->USER.sSpeed_PI.s16LowerLim                                   = MLIB_Neg_Q15(Motor->USER.s16IqLimitMax);
    // Motor->USER.sSpeed_PI.s16UpperLim                                   = Motor->USER.s16IqLimitMax;
    // Motor->USER.sSpeed_PI.s32IAccK_1                                    = 0;

     Motor_1st.USER.sId_PI.s16Kp                                            = M1_ID_KP;
     Motor_1st.USER.sId_PI.s16Ki                                            = M1_ID_KI;
	 Motor_1st.USER.sId_PI.i16Kp_Shift                                      = M1_ID_KP_SHIFT;
     Motor_1st.USER.sId_PI.i16Ki_Shift                                      = M1_ID_KI_SHIFT;
    // Motor->USER.sId_PI.s16LowerLim                                      = MLIB_Neg_Q15(Motor->USER.s16VdLimitMax);
    // Motor->USER.sId_PI.s16UpperLim                                      = Motor->USER.s16VdLimitMax;
    // Motor->USER.sId_PI.s32IAccK_1                                       = 0;

    Motor_1st.USER.sIq_PI.s16Kp                                            = M1_IQ_KP;
    Motor_1st.USER.sIq_PI.s16Ki                                            = M1_IQ_KI;
	Motor_1st.USER.sIq_PI.i16Kp_Shift                                      = M1_IQ_KP_SHIFT;
    Motor_1st.USER.sIq_PI.i16Ki_Shift                                      = M1_IQ_KI_SHIFT;
    // Motor->USER.sIq_PI.s16LowerLim                                      = MLIB_Neg_Q15(Motor->USER.s16VqLimitMax);
    // Motor->USER.sIq_PI.s16UpperLim                                      = Motor->USER.s16VqLimitMax;
    // Motor->USER.sIq_PI.s32IAccK_1                                       = 0;

    // /* SMO setting */
    // Motor->FOC.SMO.s16Fs                                                = Q15(1.0 - Rs / Ls / PWMFREQ);
    // Motor->FOC.SMO.s16Gs                                                = Q15(U_MAX / Ls / I_MAX / PWMFREQ);
    // Motor->FOC.SMO.s16ErrMax                                            = SMO_ERR_MAX;
    // Motor->FOC.SMO.s16Kslide                                            = SLIDE_GIAN;
	// Motor->FOC.SMO.s16IerrGain                                          = IERR_GIAN;
    // Motor->FOC.SMO.i16IerrGainShift                                     = IERR_GIAN_SHIFT;

    // Motor->FOC.SMO.s16Ialpha_est                                        = 0;
    // Motor->FOC.SMO.s16Ibeta_est                                         = 0;
    // Motor->FOC.SMO.s16Ealpha_est                                        = 0;
    // Motor->FOC.SMO.s16Ebeta_est                                         = 0;
    // Motor->FOC.SMO.s16Speed                                             = 0;

		    /*SMO observer and pll parameter*/
    Motor_1st.FOC.sPLL.s16PGain                                         = M1_TO_KP_GAIN;
    //Motor_1st.FOC.sPLL.i16PGainSh                                       = M1_TO_KP_SHIFT;
    Motor_1st.FOC.sPLL.s16IGain                                         = M1_TO_KI_GAIN;
    //Motor_1st.FOC.sPLL.i16IGainSh                                       = M1_TO_KI_SHIFT;
    Motor_1st.FOC.sPLL.s16ThGain                                        = M1_TO_THETA_GAIN;
    //Motor_1st.FOC.sPLL.i16ThGainSh                                      = M1_TO_THETA_SHIFT;
}
