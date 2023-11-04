#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>

#include "hal_rcc.h"
#include "hal_misc.h"
#include "hal_conf.h"

#include "drv_inc.h"
/**
 * @brief    : Set TIM1 as PWM output.
 * @param    : u16Prescaler
 * The frequency division factor of TIM is (u16Prescaler + 1)
 * @param    : u16Period
 * Set the maximum count value of TIM to u16Period
 * @retval   : None
 */
void Drv_Tim17_PWM_Out_Init(uint16_t u16Prescaler, uint16_t u16Period)
{
    /** Define the struct of THE PWM configuration */
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;

    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_OCStructInit(&TIM_OCInitStructure);
    /** Enable the TIM1 clock */
    RCC_APB2PeriphClockCmd(RCC_APB2ENR_TIM17, ENABLE);

    /** Reser TIM1 */
    TIM_DeInit(TIM17);
    /**
     * Sets the value of the automatic reload register Period for the next update event load activity
     * Set the Prescaler value used as the divisor of the TIMx clock frequency
     * Set clock split :TDTS = TIM_CKD_DIV1
     * Up counting mode
     */
    TIM_TimeBaseStructure.TIM_Period        = u16Period;
    TIM_TimeBaseStructure.TIM_Prescaler     = u16Prescaler;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

    TIM_TimeBaseInit(TIM17, &TIM_TimeBaseStructure);

    /**
     * Mode configuration: PWM mode 1
     * Output status setting: enable output
     * Complementary channel output status setting: disable output
     * Sets the pulse value to be loaded into the capture comparison register
     * Output polarity is high
     */
    TIM_OCInitStructure.TIM_OCMode          = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState     = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState    = TIM_OutputNState_Disable;

    TIM_OCInitStructure.TIM_Pulse           = u16Period / 2;
    TIM_OCInitStructure.TIM_OCPolarity      = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState     = TIM_OCIdleState_Reset;

    TIM_OC1Init(TIM17, &TIM_OCInitStructure);

    /** Enable CH1 to be preloaded */
    TIM_OC1PreloadConfig(TIM17, TIM_OCPreload_Enable);

    /** Enable TIMx's preloaded register on ARR */
    TIM_ARRPreloadConfig(TIM17, ENABLE);

    /** Enable the TIM1 */
    TIM_Cmd(TIM17, DISABLE);
    /** Main Output Enable:Disable the MOE bit */
    TIM_CtrlPWMOutputs(TIM17, DISABLE);
}

void Drv_vPwmOutputInit(void)
{
	//Drv_Tim17_PWM_Out_Init(DRV_PWM_PRESCALER, DRV_PWM_PERIOD);
	Drv_Tim17_PWM_Out_Init(DRV_PWM_OUTPUT_PRESCALER, DRV_PWM_OUTPUT_PERIOD);//Output low level
	TIM_CtrlPWMOutputs(TIM17, ENABLE);
    TIM_Cmd(TIM17, ENABLE);
}

void Drv_vSetOutputFreq(uint16_t u16Frequency)
{
	if(u16Frequency == 0)
		{
			//Output low level
			//GPIO_ResetBits(TIM17_CH1_PORT, TIM17_CH1_PIN);
			TIM_SetAutoreload(TIM17, DRV_PWM_PERIOD_OUT_LO);
		}
	else
		{

			TIM_SetAutoreload(TIM17, DRV_PWM_BASE_FRQ/u16Frequency);

		}

}

void Drv_vEnablePWMOutput(void)
{

	TIM_CtrlPWMOutputs(TIM17, ENABLE);

}
