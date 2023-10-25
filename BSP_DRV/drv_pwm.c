/** 
 * @file     drv_pwm.c
 * @author   MindMotion Motor Team : Wesson
 * @brief    This file provides all the driver functions for the PWM.
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
#define _DRV_PWM_C_

/** Files includes */
#include "drv_pwm.h"
#include "main.h"
#include "statemachine.h"
#include "Common.h"
/**
 * @addtogroup MM32_Hardware_Driver_Layer
 * @{
 */

/**
 * @addtogroup Drv_PWM
 * @{
 */

typedef struct _TBrakeData
{
	uint16_t u16BrakeTime;

}TBrakeData;

typedef struct _TBrakePara
{
	uint16_t u16BrakeLimitVal;

}TBrakePara;

/* CAP COM TIM1 and TIM8 */
#define HMT_HIN1_EN     TIM_CCER_CC1EN     /**< High side channel 1 (PhaseU) enable flag */
#define HMT_LIN1_EN     TIM_CCER_CC1NEN    /**< Low side  channel 1 (PhaseU) enable flag */
#define HMT_HIN2_EN     TIM_CCER_CC2EN     /**< High side channel 2 (PhaseU) enable flag */
#define HMT_LIN2_EN     TIM_CCER_CC2NEN    /**< Low side  channel 2 (PhaseU) enable flag */
#define HMT_HIN3_EN     TIM_CCER_CC3EN     /**< High side channel 3 (PhaseU) enable flag */
#define HMT_LIN3_EN     TIM_CCER_CC3NEN    /**< Low side  channel 3 (PhaseU) enable flag */
#define HMT_ALL_OUT    (HMT_HIN1_EN | HMT_LIN1_EN | HMT_HIN2_EN | HMT_LIN2_EN | HMT_HIN3_EN | HMT_LIN3_EN)

static const uint32_t PWM_ucChannelConfig[] = {
    (uint32_t)0x00,                                                     /**< All PWM channels off */
    (uint32_t)(HMT_ALL_OUT),                                            /**< All PWM channels on */
    (uint32_t)(HMT_HIN1_EN | HMT_LIN1_EN),                              /**< PWM channel phase U on (high and low) */
    (uint32_t)(HMT_HIN2_EN | HMT_LIN2_EN),                              /**< PWM channel phase V on (high and low) */
    (uint32_t)(HMT_HIN3_EN | HMT_LIN3_EN),                              /**< PWM channel phase W on (high and low) */
    (uint32_t)(HMT_HIN1_EN | HMT_LIN1_EN | HMT_HIN2_EN | HMT_LIN2_EN),  /**< PWM channel phase U and V on (high and low) */
    (uint32_t)(HMT_HIN2_EN | HMT_LIN2_EN | HMT_HIN3_EN | HMT_LIN3_EN),  /**< PWM channel phase V and W on (high and low) */
    (uint32_t)(HMT_HIN3_EN | HMT_LIN3_EN | HMT_HIN1_EN | HMT_LIN1_EN),  /**< PWM channel phase W and U on (high and low) */
    (uint32_t)(HMT_HIN1_EN),                                            /**< PWM channel phase U high side on */
    (uint32_t)(HMT_HIN2_EN),                                            /**< PWM channel phase V high side on */
    (uint32_t)(HMT_HIN3_EN),                                            /**< PWM channel phase W high side on */
    (uint32_t)(HMT_LIN1_EN),                                            /**< PWM channel phase U low side on */
    (uint32_t)(HMT_LIN2_EN),                                            /**< PWM channel phase V low side on */
    (uint32_t)(HMT_LIN3_EN),                                            /**< PWM channel phase W low side on */
    (uint32_t)0x00                                                      /**< All PWM channels off => never used */
};


static uint16_t TIM_u16BrakeDutyCycle = PWM_DUTYCYCLE_50_PERCENT;

static TBrakeData MOT_tBrakeData = {0};
static TBrakePara MOT_tBrakePara = {0};


void Drv_Pwm_Init(TIM_TypeDef * pTim, uint16_t u16Period,uint16_t u16DeadTime)
{
    /** Define the struct of THE PWM configuration */
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_BDTRInitTypeDef TIM_BDTRInitStruct;
    TIM_OCInitTypeDef  TIM_OCInitStructure;

    /** Enable the TIM1 clock */
    if (TIM1 == pTim)
    {
        RCC_APB2PeriphClockCmd(RCC_APB2ENR_TIM1, ENABLE);
    }
    else if (TIM8 == pTim)
    {
        RCC_APB2PeriphClockCmd(RCC_APB2ENR_TIM8, ENABLE);
    }
    else
    {
        return;
    }
    /**
     * Sets the value of the automatic reload register Period for the next update event load activity  
     * Set the Prescaler value used as the divisor of the TIMx clock frequency
     * Set clock split :TDTS = TIM_CKD_DIV1
     * TIM center aligned mode1  
     */
    TIM_TimeBaseStructure.TIM_Period        = u16Period;
    TIM_TimeBaseStructure.TIM_Prescaler     = 0;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_CenterAligned2;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 1;
    
    TIM_TimeBaseInit(pTim, &TIM_TimeBaseStructure);

    /**
     * Enable state selection in running mode  
     * Enable state selection in idle mode  
     * Software error lock configuration: lock closed without protection
     * DTG[7:0] dead zone generator configuration (dead zone time DT)  
     */
     /**
     * TDTS = 125nS(8MHz)
     * DTG[7: 5] = 0xx => DT = DTG[7: 0] * Tdtg, Tdtg = TDTS;
     * DTG[7: 5] = 10x => DT =(64+DTG[5: 0]) * Tdtg, Tdtg = 2 * TDTS;
     * DTG[7: 5] = 110 => DT =(32+DTG[4: 0]) * Tdtg, Tdtg = 8 * TDTS;
     * DTG[7: 5] = 111=> DT =(32 + DTG[4: 0]) *  Tdtg, Tdtg = 16 * TDTS;
     */
    TIM_BDTRInitStruct.TIM_OSSRState    = TIM_OSSRState_Enable;
    TIM_BDTRInitStruct.TIM_OSSIState    = TIM_OSSIState_Enable;
    TIM_BDTRInitStruct.TIM_LOCKLevel    = TIM_LOCKLevel_OFF;
    TIM_BDTRInitStruct.TIM_DeadTime     = u16DeadTime;

    /**
     * Brake configuration: enable brake
     * Brake input polarity: active in low level   
     * Auto output enable configuration: Disable MOE bit hardware control
     */
    TIM_BDTRInitStruct.TIM_Break            = TIM_Break_Enable;
    TIM_BDTRInitStruct.TIM_BreakPolarity    = TIM_BreakPolarity_Low;
    TIM_BDTRInitStruct.TIM_AutomaticOutput  = TIM_AutomaticOutput_Disable;
    TIM_BDTRConfig(pTim, &TIM_BDTRInitStruct);
    
    /**
     * Mode configuration: Output compare mode: PWM2
     * Output status setting: enable output  
     * Complementary channel output status setting: enable output
     * Sets the pulse value to be loaded into the capture comparison register
     * Output polarity is high
     * N Output polarity is high
     */
     
    TIM_OCInitStructure.TIM_OCMode          = TIM_OCMode_PWM2;//because HV/LV is inverted.
    TIM_OCInitStructure.TIM_OutputState     = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState    = TIM_OutputNState_Enable;
    TIM_OCInitStructure.TIM_Pulse           = 0;
    TIM_OCInitStructure.TIM_OCPolarity      = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCNPolarity     = TIM_OCNPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState     = TIM_OCIdleState_Reset;
    TIM_OCInitStructure.TIM_OCNIdleState    = TIM_OCNIdleState_Reset;
    /*Initializes the specified parameters in the init_struct*/
    TIM_OC1Init(pTim, &TIM_OCInitStructure);
    TIM_OC2Init(pTim, &TIM_OCInitStructure);
    TIM_OC3Init(pTim, &TIM_OCInitStructure);    
    /** Initialize the CCR4 trigger point */
    TIM_OCInitStructure.TIM_Pulse           = 10;
    TIM_OCInitStructure.TIM_OutputState     = TIM_OutputState_Disable;
    TIM_OCInitStructure.TIM_OutputNState    = TIM_OutputNState_Disable;
    
    TIM_OC4Init(pTim, &TIM_OCInitStructure);

    /** Enable CH1, 2, and 3 to be preloaded */
    TIM_OC1PreloadConfig(pTim, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(pTim, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(pTim, TIM_OCPreload_Enable);
    
    /** Enable TIMx's preloaded register on ARR */
    TIM_ARRPreloadConfig(pTim, ENABLE);

    /** Enable the TIM1 */
    TIM_Cmd(pTim, ENABLE);
    /** Main Output Enable:Disable the MOE bit */
    TIM_CtrlPWMOutputs(pTim, DISABLE);
}





/*---------------------------------------------------------------------------------------------------------*/
static uint16_t TIM_vDutyCycleToCompare(uint16_t uiDutyCycle) {
	uint32_t uiCompareVal;

	/* Transform given duty cycle into compare values */
	uiCompareVal  = (uint32_t)uiDutyCycle;                   /* 0..100% = 0..32768 (Q15) */
	uiCompareVal *= (uint32_t)(Motor_1st.USER.u16PWMPeriod);          /* Timer max value          */
	uiCompareVal /= (uint32_t)PWM_DUTYCYCLE_100_PERCENT;     /* 100% q15 = 32768         */

	/* Check range of calculated compare values */
	if (uiCompareVal > (uint32_t)(Motor_1st.USER.u16PWMPeriod)){
		/*
		 * Limit compare value to max. value (timer peak value)
		 * */
		uiCompareVal = (uint32_t)(Motor_1st.USER.u16PWMPeriod);
	}

	return (uint16_t)uiCompareVal; /* Return calculated compare value  */
}

/*---------------------------------------------------------------------------------------------------------*/
void TIM_vSetCompareValues(TDutyCycleConfig tCompareValue)
{
	uint16_t	uiRegU;
	uint16_t	uiRegV;
	uint16_t	uiRegW;

	uiRegU = Motor_1st.USER.u16PWMPeriod;
	uiRegW = Motor_1st.USER.u16PWMPeriod;
	uiRegV = Motor_1st.USER.u16PWMPeriod;
	uiRegU -= tCompareValue.uiDutyCyclePhaseU;
	uiRegV -= tCompareValue.uiDutyCyclePhaseV;
	uiRegW -= tCompareValue.uiDutyCyclePhaseW;
	Motor_1st.PWMx.Timer->CCR1  = uiRegU;
	Motor_1st.PWMx.Timer->CCR2  = uiRegV;
	Motor_1st.PWMx.Timer->CCR3  = uiRegW;
/*
	Motor_1st.PWMx.Timer->CCR1 = MLIB_Mul_Q15((INT16_MAX - Motor_1st.FOC.SVM.s16DutyA), Motor_1st.USER.u16PWMPeriod);
    Motor_1st.PWMx.Timer->CCR2 = MLIB_Mul_Q15((INT16_MAX - Motor_1st.FOC.SVM.s16DutyB), Motor_1st.USER.u16PWMPeriod);
    Motor_1st.PWMx.Timer->CCR3 = MLIB_Mul_Q15((INT16_MAX - Motor_1st.FOC.SVM.s16DutyC), Motor_1st.USER.u16PWMPeriod);
*/
}

/*---------------------------------------------------------------------------------------------------------*/
void TIM_vSetOutputs(uint32_t tSwitches, FunctionalState tStatus)
{

		if (tStatus == ENABLE)
		{
			/*
			 * Enable selected switches and
			 * set master output enable bit to activate output (main switch)
			 * */
			TIM1->CCER = tSwitches; 
			ENABLE_PWMOUT();
		}
		else
		{
			/*
			 * Disable selected switches
			 * If all switches have been disabled, clear the master output enable bit too
			 * */
			TIM1->CCER &= ~tSwitches;
			if((TIM1->CCER & HMT_ALL_OUT) == 0)
			{
				DISABLE_PWMOUT();
			}
		}

}

/*
Param1:  TChannels            tActiveChannels
		select the channel of IGBT
Param2:  TDutyCycleConfig     tCompareValue
		configure the PWM duty cycle. range: 0...32768


*/
BOOL TIM_bSetOutputConfig(TChannels            tActiveChannels, TDutyCycleConfig tCompareValue)
{
	  TDutyCycleConfig tPWMCompareValue = {0,0,0};

	  __disable_irq(); 						/* Configure compare values - Always start from the all disabled state */
      /* Set compare values to 50 %*/
      tPWMCompareValue.uiDutyCyclePhaseU = TIM_vDutyCycleToCompare(tCompareValue.uiDutyCyclePhaseU);
      tPWMCompareValue.uiDutyCyclePhaseV = TIM_vDutyCycleToCompare(tCompareValue.uiDutyCyclePhaseV);
      tPWMCompareValue.uiDutyCyclePhaseW = TIM_vDutyCycleToCompare(tCompareValue.uiDutyCyclePhaseW);
      TIM_vSetCompareValues(tPWMCompareValue); 					/* Set compare values */
      TIM_vSetOutputs(HMT_ALL_OUT,DISABLE); /* Disable outputs    */
      TIM_vSetOutputs(PWM_ucChannelConfig[tActiveChannels],ENABLE);
      __enable_irq();
      return TRUE;


}

/****************************************************/
void TIM_vBrakeInit(uint16_t u16BrakeTime)
{

	TDutyCycleConfig TIM_tBrakeDutyCycle = {PWM_DUTYCYCLE_50_PERCENT,PWM_DUTYCYCLE_50_PERCENT,PWM_DUTYCYCLE_50_PERCENT};
	TIM_bSetOutputConfig(PWM_CH_ALL_ON,TIM_tBrakeDutyCycle);

	MOT_tBrakePara.u16BrakeLimitVal = u16BrakeTime;
	
	M1_SwitchRunWindDetBrake();
}


/****************************************************/
bool TIM_bBrakeUpdate(void)
{
	bool bReturnValue = false;
	TDutyCycleConfig TIM_tBrakeDutyCycleUpdate;
	if(MOT_tBrakeData.u16BrakeTime < MOT_tBrakePara.u16BrakeLimitVal)
	{
		MOT_tBrakeData.u16BrakeTime ++;

		if(TIM_u16BrakeDutyCycle > 0)
		{
			TIM_u16BrakeDutyCycle -= 2;
		}
		TIM_tBrakeDutyCycleUpdate.uiDutyCyclePhaseU = TIM_u16BrakeDutyCycle;
		TIM_tBrakeDutyCycleUpdate.uiDutyCyclePhaseV = TIM_u16BrakeDutyCycle;
		TIM_tBrakeDutyCycleUpdate.uiDutyCyclePhaseW = TIM_u16BrakeDutyCycle;
		TIM_vSetCompareValues(TIM_tBrakeDutyCycleUpdate);
	}
	else
	{
		bReturnValue = true;
	}
	
	return bReturnValue;

}


/**
  * @}
*/

/**
  * @}
*/
