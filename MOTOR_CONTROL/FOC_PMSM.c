/**
 * @file     FOC_PMSM.c
 * @author   MindMotion Motor Team : Wesson
 * @brief    This file provides the FOC functions and test samples.
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
#define _FOC_PMSM_C_

/** Files includes */

#include "FOC_PMSM.h"
#include "parameter.h"
#include "drv_inc.h"
#include "board.h"
#include "Voltage.h"

__IO int16_t			MOT_s16SpdAdcValue = 0;					/* the speed commond of VSP */

/**
  * @brief Correct PWM duty cycle outputcalculation, based on the measured DC-bus voltage. 
  * @param sVAlBeIn  s16Vbus
  * @retval sVAlBe_Out
  */
void DcBus_Ripple_suppress(MCFLIB_2_ALBE_T_S16 *sVAlBeIn, Q15_t s16Vbus, MCFLIB_2_ALBE_T_S16 *sVAlBe_Out)
{
    int32_t temp32;
    temp32 = ((int32_t)(sVAlBeIn->s16Alpha)) * ((int32_t)(Q15(0.866))); 
    temp32 = Division(temp32, s16Vbus) << 1;     //0.866*2 = sqrt3
    temp32 = (temp32 > INT16_MAX)  ? INT16_MAX : temp32; /* high saturation */
    temp32 = (temp32 < INT16_MIN)  ? INT16_MIN : temp32; /* low saturation */

    sVAlBe_Out->s16Alpha = (int16_t)(temp32);
    temp32 = ((int32_t)(sVAlBeIn->s16Beta)) * ((int32_t)(Q15(0.866))); 
    temp32 = Division(temp32, s16Vbus) << 1;     //0.866*2 = sqrt3
    temp32 = (temp32 > INT16_MAX)  ? INT16_MAX : temp32; /* high saturation */
    temp32 = (temp32 < INT16_MIN)  ? INT16_MIN : temp32; /* low saturation */

    sVAlBe_Out->s16Beta = (int16_t)(temp32);

}

/**
  * @brief Get the ADC result 
  * @param  Motor
  * @retval None
  */
void Get_ADC_Result(Motor_TypeDef *Motor)
{
    int16_t s16Ib_Temp, s16Ia_Temp;
    /*get the two phase current */
    s16Ia_Temp = (int16_t)GET_ADC1_VALUE(IR_U_RANK) << 3; // Read ADC register for phase B Sensing current   adc is 12 bit (<< 3) for Q15 format
    s16Ib_Temp = (int16_t)GET_ADC2_VALUE(IR_V_RANK) << 3; // Read ADC register for phase C Sensing current   adc is 12 bit (<< 3) for Q15 format
    Motor->FOC.sIabc.s16A = (-s16Ia_Temp + Motor->FOC.sIabc_offset.s16A)<<1;   // (<< 1)  VDD/2  ~ Imax
    Motor->FOC.sIabc.s16B = (-s16Ib_Temp + Motor->FOC.sIabc_offset.s16B)<<1;   // (<< 1)  VDD/2  ~ Imax
    Motor->FOC.sIabc.s16C = -Motor->FOC.sIabc.s16A - Motor->FOC.sIabc.s16B;    // IC= -IA - IB
    Motor->FOC.s16Vbus = (int16_t)GET_ADC1_VALUE(VBUS_RANK) << 3;  // adc is 12 bit  (<< 3) for Q15 format
    //Motor->USER.s16VspCmd = (int16_t)GET_ADC2_VALUE(VR_RANK) << 3; // adc is 12 bit  (<< 3) for Q15 format
    
    VOL_tReadVoltageValue();
}
/**
  * @brief  Sensorless SMO observer and calls
  *         PLL with the purpose of computing a new speed estimation and
  *         updating the estimated electrical angle
  * @param  Motor
  * @retval None
  */

void PMSM_Obs(Motor_TypeDef *Motor)
{
    int16_t s16ThetaErr;

    /* SMO observer in AB system */
    Motor->FOC.SMO.s16Ialpha = Motor->FOC.sIAlBe.s16Alpha;
    Motor->FOC.SMO.s16Ibeta = Motor->FOC.sIAlBe.s16Beta;
    Motor->FOC.SMO.s16Valpha = Motor->FOC.sVAlBe.s16Alpha;
    Motor->FOC.SMO.s16Vbeta = Motor->FOC.sVAlBe.s16Beta;
    MLIB_SMO(&Motor->FOC.SMO);    
    /* Calculate the input of PLL   */
    s16ThetaErr = MLIB_Mul_Q15(-Motor->FOC.SMO.s16Ealpha_est, Motor->FOC.sAngle_cmd.s16Cos) - MLIB_Mul_Q15(Motor->FOC.SMO.s16Ebeta_est, Motor->FOC.sAngle_cmd.s16Sin);
	  /* update the motor angle  */
    Motor->FOC.s16CmdTheta = MLIB_PLL(s16ThetaErr, &Motor->FOC.sPLL);
    /* PLL calculation */
    Motor->FOC.s16CmdTheta_Update = Motor->FOC.s16CmdTheta + MLIB_Mul_Q15(Motor->FOC.s16SpdFilt, SPEED_TO_THETA);
    Motor->FOC.SMO.s16Speed = Motor->FOC.sPLL.s32Speed >> 16;
}

/**
  * @brief It executes the core of FOC drive that is the controllers for Iqd
  *        currents regulation. It must be called periodically
  *        when new motor currents have been converted
  * @param Motor.
  * @retval None
  */

void FOC_Current_Controller(Motor_TypeDef *Motor)
{
    int32_t s32Temp;
    int16_t s16Temp;
	  /* Calculate the sincos of  s16CmdTheta */
    Motor->FOC.sAngle_cmd = MLIB_SinCos(Motor->FOC.s16CmdTheta);
	  /* PARK  transform */
    MCFLIB_Park_S16(&Motor->FOC.sIAlBe, &Motor->FOC.sAngle_cmd, &Motor->FOC.sIdq);
	 /* Calculate the PI saturation for Vd    */
    s16Temp = MLIB_Mul_Q15(Motor->USER.s16DutyLimitCmd, Motor->FOC.s16VbusAvg);
    Motor->FOC.s16VMAX = MLIB_Mul_Q15(s16Temp, Q15(0.6667)); //Amplitude invariant transformation
    Motor->USER.sId_PI.s16UpperLim = Motor->FOC.s16VMAX;
    Motor->USER.sId_PI.s16LowerLim = MLIB_Neg_Q15(Motor->USER.sId_PI.s16UpperLim);
	  /* PI regulator for Vd*/
    Motor->FOC.sIdq_err.s16D = Motor->FOC.sIdq_cmd.s16D - Motor->FOC.sIdq.s16D;
    Motor->FOC.sVdq.s16D = MLIB_PIControl_Q15(Motor->FOC.sIdq_err.s16D, &Motor->USER.sId_PI);
    /* Calculate the PI saturation for Vq    */	
    s32Temp = Motor->FOC.s16VMAX * Motor->FOC.s16VMAX - Motor->FOC.sVdq.s16D * Motor->FOC.sVdq.s16D;
    Motor->USER.sIq_PI.s16UpperLim   = Hw_Sqrt(s32Temp);
    Motor->USER.sIq_PI.s16LowerLim = MLIB_Neg_Q15(Motor->USER.sIq_PI.s16UpperLim);
	  /* PI regulator for Vq*/
    Motor->FOC.sIdq_err.s16Q = Motor->FOC.sIdq_cmd.s16Q - Motor->FOC.sIdq.s16Q;
    Motor->FOC.sVdq.s16Q = MLIB_PIControl_Q15(Motor->FOC.sIdq_err.s16Q, &Motor->USER.sIq_PI);
  	/* Update the sincos for invpark */
		Motor->FOC.sAngle_cmd_update = Motor->FOC.sAngle_cmd;
		/* Inv PARK transform*/
    MCFLIB_InvPark_S16(&Motor->FOC.sVdq, &Motor->FOC.sAngle_cmd_update, &Motor->FOC.sVAlBe);
		/* Elimination of the DC-bus voltage ripple  */
    DcBus_Ripple_suppress(&Motor->FOC.sVAlBe, Motor->FOC.s16VbusAvg, &Motor->FOC.sVAlBe_cmd);
}

uint16_t MOT_u16GetVariantResistorDigitalValue(void)
{

	return MOT_s16SpdAdcValue;

}
