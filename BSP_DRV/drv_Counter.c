#include "hal_rcc.h"
#include "hal_misc.h"

#include "drv_inc.h"
/**
 * @brief    : Set TIM1 as PWM output.
 * @param    : u16Prescaler
 * The frequency division factor of TIM is (u16Prescaler + 1)
 * @param    : u16Period
 * Set the maximum count value of TIM to u16Period
 * @retval   : None
 */
void Drv_Tim16_Counter_Init(uint16_t u16Prescaler, uint16_t u16Period)
{
    /** Define the struct of THE PWM configuration */
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;

    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_OCStructInit(&TIM_OCInitStructure);
    /** Enable the TIM1 clock */
    RCC_APB2PeriphClockCmd(RCC_APB2ENR_TIM16, ENABLE);

    /** Reser TIM1 */
    TIM_DeInit(TIM16);
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

    TIM_TimeBaseInit(TIM16, &TIM_TimeBaseStructure);

    /** Enable TIMx's preloaded register on ARR */
    TIM_ARRPreloadConfig(TIM16, ENABLE);

    /** Enable the TIM1 */
    TIM_Cmd(TIM16, ENABLE);
}


void Drv_Tim16_EnDisable(bool benable)
{
    if(benable == true)
    {
        TIM_Cmd(TIM16, ENABLE);
    }
    else
    {
        TIM_Cmd(TIM16, DISABLE);
    }

}

uint32_t Drv_u32GetTim16CountVal(void)
{

    return TIM_GetCounter(TIM16);
}