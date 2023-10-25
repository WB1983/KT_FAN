////////////////////////////////////////////////////////////////////////////////
/// @file    beeper.c
/// @author  NJ ClassB TEAM
/// @brief   THIS FILE PROVIDES ALL THE SYSTEM FUNCTIONS.
////////////////////////////////////////////////////////////////////////////////
/// @attention
///
/// THE EXISTING FIRMWARE IS ONLY FOR REFERENCE, WHICH IS DESIGNED TO PROVIDE
/// CUSTOMERS WITH CODING INFORMATION ABOUT THEIR PRODUCTS SO THEY CAN SAVE
/// TIME. THEREFORE, MINDMOTION SHALL NOT BE LIABLE FOR ANY DIRECT, INDIRECT OR
/// CONSEQUENTIAL DAMAGES ABOUT ANY CLAIMS ARISING OUT OF THE CONTENT OF SUCH
/// HARDWARE AND/OR THE USE OF THE CODING INFORMATION CONTAINED HEREIN IN
/// CONNECTION WITH PRODUCTS MADE BY CUSTOMERS.
///
/// <H2><CENTER>&COPY; COPYRIGHT MINDMOTION </CENTER></H2>
////////////////////////////////////////////////////////////////////////////////

// Define to prevent recursive inclusion
#define _BEEPER_C_

#include "key_classb.h"
#include "beeper.h"

////////////////////////////////////////////////////////////////////////////////
extern u32 KEYDBCFlags;
////////////////////////////////////////////////////////////////////////////////
void BEEPER_Config(void)
{
    GPIO_InitTypeDef  GPIO_InitStruct;
    TIM_TypeDef TIM_InitStruct;

    RCC_GPIO_ClockCmd(GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2ENR_TIM1,ENABLE);                //ENALBE TIM1 CLOCK

    //GPIO initial
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_2);            //PA8->TIM1_CH1

    GPIO_InitStruct.GPIO_Pin  =  GPIO_Pin_8;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*
    TIM1->CR1=TIM_CR1_UDIS;                                         //UPDATE DISABLE,UP COUNTER,EDGE ALIGNED
    TIM1->CR2=0;
    TIM1->SMCR=0;                                                   //CK_INT(8Mhz),
    TIM1->PSC=0;                                                    //8MHZ/1
    TIM1->ARR=16000;                                                //8MHZ/16000/=0.5KHZ=fpwm
    TIM1->CCR1=1000;                                                 //duty cycle
    TIM1->CCMR1=0x60;                                               //COMPARATOR
    TIM1->CCER=TIM_CCER_CC1EN;
    TIM1->BDTR=TIM_BDTR_MOEN;
    */
    TIM_UpdateDisableConfig(TIM1,DISABLE);
//    TIM_PrescalerConfig(TIM1, 0, TIM_PSCReloadMode_Immediate);
    TIM_PrescalerConfig(TIM1, 12, TIM_PSCReloadMode_Immediate);
    TIM_SetAutoreload(TIM1,16000);
    TIM_SetCompare1(TIM1,200);
    TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_OCMode_PWM1);
    TIM_CCxCmd(TIM1,TIM_Channel_1,TIM_CCx_Enable);
    TIM_CtrlPWMOutputs(TIM1,ENABLE);

}

void BEEPER_Polling()
{
    if(KEYDBCFlags&0xf)
    {
        //TIM1->CR1|=(u32)(TIM_CR1_CEN);
        TIM_Cmd(TIM1, ENABLE);
    }
    else
    {
        //TIM1->CR1&=~(u32)(TIM_CR1_CEN);
        TIM_Cmd(TIM1, DISABLE);
    }
}
