#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>

#include "hal_rcc.h"
#include "hal_misc.h"
#include "hal_conf.h"
#include "board.h"
#include "drv_inc.h"

#define CAP_NORMAL_MODE
static uint16_t u16BUFFER;
/**
 * @brief    : Set TIM3 as the slave timer.
 * @param    : u16Prescaler
 * The frequency division factor of TIM is (u16Prescaler + 1)
 * @param    : u16Period
 * Set the maximum count value of TIM to u16Period
 * @retval   : None
 */
void Drv_Tim3_Pwm_In_Init(uint16_t u16Prescaler, uint16_t u16Period)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_ICInitTypeDef  TIM_ICInitStructure;
    NVIC_InitTypeDef NVIC_StructInit;

    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

    /** Configure timer 2 interrupt channel and priority */
    NVIC_StructInit.NVIC_IRQChannel             = TIM3_IRQn;
    NVIC_StructInit.NVIC_IRQChannelPriority     = TIM3_INTERRUPT;
    NVIC_StructInit.NVIC_IRQChannelCmd          = ENABLE;
    NVIC_Init(&NVIC_StructInit);

    TIM_DeInit(TIM3);

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
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

    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
    
//channel 1
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_Channel     = TIM_Channel_1;
    TIM_ICInitStructure.TIM_ICPolarity  = TIM_ICPolarity_Rising;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter    = 0x0;
	//Configure PWM
    TIM_PWMIConfig(TIM3, &TIM_ICInitStructure);
	//Select the valid input
    TIM_SelectInputTrigger(TIM3, TIM_TS_TI1FP1);
    //Configuration in master-slave reset mode
    TIM_SelectSlaveMode(TIM3, TIM_SlaveMode_Reset);

    TIM_ITConfig(TIM3, TIM_IT_CC1, ENABLE);
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
    TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
    TIM_ClearITPendingBit(TIM3, TIM_IT_Update);

	TIM_Cmd(TIM3, ENABLE);

}

void Drv_vCaptureInit(void)
{

	Drv_Tim3_Pwm_In_Init(DRV_CAP_PWM_PRESCALER, DRV_CAP_PWM_PERIOD);

}


