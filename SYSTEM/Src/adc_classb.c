////////////////////////////////////////////////////////////////////////////////
/// @file    adc.c
/// @author  NJ ClassB TEAM
/// @brief   In window comparator mode,The transformation results are detected
///          Set the threshold value from 0 to 3V.
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
#define _ADC_C_

// Files includes
#include <stdio.h>
#include "delay.h"
#include "adc_classb.h"
#include "IEC60730_B_param.h"

#define  VDD_MOST   3800
#define  VDD_LEAST  600 //user can config to pass the test
#define  TS_OFFSET  0x1FFFF7F6
#define  TS_V25     0x1FFFF7FC
#define  REF_OFFSET 0x1FFFF7E0
#define  ADC_CHECK  ((uint32_t)90)
////////////////////////////////////////////////////////////////////////////////
/// @addtogroup MM32_Hardware_Abstract_Layer
/// @{

////////////////////////////////////////////////////////////////////////////////
/// @addtogroup ADC_AWDG
/// @{

////////////////////////////////////////////////////////////////////////////////
/// @addtogroup ADC_Exported_Functions
/// @{
////////////////////////////////////////////////////////////////////////////////
extern u32 KEYDBCFlags;
////////////////////////////////////////////////////////////////////////////////
/// @brief  Configure the ADC1 single conversion mode to correspond to the PIN
/// @note   It must be careful of the different config for each chipset.
/// @param  GPIOn: The sampling GPIO corresponds to the port
/// @param  gpio_pin: The sampling gpio_pin corresponds to the pin.
/// @retval None.
////////////////////////////////////////////////////////////////////////////////
void GPIO_PinConfigAnalog(GPIO_TypeDef* GPIOn, u16 gpio_pin)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_StructInit(&GPIO_InitStruct);
    RCC_GPIO_ClockCmd(GPIOn, ENABLE);
    GPIO_InitStruct.GPIO_Pin  =  gpio_pin;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOn, &GPIO_InitStruct);
}

////////////////////////////////////////////////////////////////////////////////
/// @brief  Configure ADC1 single conversion mode, external interrupt source
///         interrupt priority
/// @note   Configure parameters according to requirements.
/// @param  adc_channel_x: The sampling channel
/// @retval None.
////////////////////////////////////////////////////////////////////////////////
void ADC1SingleChannelInit(ADCCHANNEL_TypeDef adc_channel_x)
{
    ADC_InitTypeDef  ADC_InitStruct;
    ADC_StructInit(&ADC_InitStruct);
    //Enable ADC clock
    RCC_ADC_ClockCmd(ADC1, ENABLE);
    ADC_Calibration(ADC1);
    ADC_InitStruct.ADC_Resolution = ADC_Resolution_12b;
    //ADC prescale factor
    ADC_InitStruct.ADC_PRESCARE = ADC_PCLK2_PRESCARE_16;
    //Set ADC mode to continuous conversion mode
    ADC_InitStruct.ADC_Mode = ADC_CR_IMM;
    //AD data right-justified
    ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStruct.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
    ADC_Init(ADC1, &ADC_InitStruct);
    //Enable the channel
    ADC_RegularChannelConfig(ADC1, adc_channel_x, 0, (u32)ADC_Samctl_240_5);

    //Enable
    ADC_Cmd(ADC1, ENABLE);
    //ADC_ANY_Cmd(ADC1, ENABLE);
}
////////////////////////////////////////////////////////////////////////////////
/// @brief
/// @note
///
/// @param  None.
/// @retval None.
////////////////////////////////////////////////////////////////////////////////
void ADC_CHANNELxENcmd(ADC_TypeDef* adc,ADCCHANNEL_TypeDef channel, FunctionalState state)
{
    (state) ? (adc->ADCHS |=(1<<channel)) : (adc->ADCHS &= ~(1<<channel));
}
////////////////////////////////////////////////////////////////////////////////
/// @brief  Configure sampling GPIO with single conversion mode for ADC1
/// @note   Note the corresponding channel selection.
/// @param  adc_channel_x: The sampling channel
/// @retval None.
////////////////////////////////////////////////////////////////////////////////
void ADC_Config_2(ADCCHANNEL_TypeDef adc_channel)
{
    if( ADC_Channel_0 == adc_channel ) {
        GPIO_PinConfigAnalog(GPIOA, GPIO_Pin_0);
    }
    else if( ADC_Channel_1 == adc_channel ) {
        GPIO_PinConfigAnalog(GPIOA, GPIO_Pin_1);
    }
    else if( ADC_Channel_2 == adc_channel ) {
        GPIO_PinConfigAnalog(GPIOA, GPIO_Pin_2);
    }
    else if( ADC_Channel_3 == adc_channel ) {
        GPIO_PinConfigAnalog(GPIOA, GPIO_Pin_3);
    }
    else if( ADC_Channel_4 == adc_channel ) {
        GPIO_PinConfigAnalog(GPIOA, GPIO_Pin_4);
    }
    else if( ADC_Channel_5 == adc_channel ) {
        GPIO_PinConfigAnalog(GPIOA, GPIO_Pin_5);
    }
    else if( ADC_Channel_6 == adc_channel ) {
        GPIO_PinConfigAnalog(GPIOA, GPIO_Pin_6);
    }
    else if( ADC_Channel_7 == adc_channel ) {
        GPIO_PinConfigAnalog(GPIOA, GPIO_Pin_7);
    }
    else if( ADC_Channel_8 == adc_channel ) {
        GPIO_PinConfigAnalog(GPIOB, GPIO_Pin_0);
    }
    else if( ADC_Channel_9 == adc_channel ) {
        GPIO_PinConfigAnalog(GPIOB, GPIO_Pin_1);
    }
    else {
    }
    ADC1SingleChannelInit(adc_channel);
    ADC_CHANNELxENcmd(ADC1,adc_channel,DISABLE);
}

////////////////////////////////////////////////////////////////////////////////
/// @brief
/// @note
///
/// @param  None.
/// @retval None.
////////////////////////////////////////////////////////////////////////////////
void ADC_Config()
{
    ADC_Config_2(ADC_Channel_14);
    ADC_TempSensorVrefintCmd(ENABLE);
}
////////////////////////////////////////////////////////////////////////////////
/// @brief
/// @note
///
/// @param  None.
/// @retval None.
////////////////////////////////////////////////////////////////////////////////
void IEC60730_ADCInternalChannelCheck(void)
{

    u16 adresult=0;
    u32 tmp0=0;
    u32 Vref_calc=0;
    u32 VDDA=0;
    ADC_Config();
    ADC_CHANNELxENcmd(ADC1,ADC_Channel_14,ENABLE);
    ADC_SoftwareStartConvCmd(ADC1,ENABLE);
    while(!((ADC1->DR)&ADC_DR_VALID));
    adresult=(u16)(ADC1->DR);
    ADC_CHANNELxENcmd(ADC1,ADC_Channel_14,DISABLE);
    tmp0=adresult*3300;
    tmp0/=4096;
    Vref_calc = (*(__IO uint16_t *)(REF_OFFSET));
    VDDA = Vref_calc*3300/adresult;
#ifdef ClassB_VERBOSE_POR
    printf("OnBoard VDDA  is %04d mV\r\n", VDDA);
#endif
    if((VDDA > VDD_MOST)|| (VDDA < VDD_LEAST))
    {
#ifdef ClassB_VERBOSE
        printf("\n\r ADC REF CHECK ERROR  \n\r");
#endif  /* ClassB_VERBOSE */
        FailSafePOR(ADC_CHECK);
    }

}
////////////////////////////////////////////////////////////////////////////////
/// @brief
/// @note
///
/// @param  None.
/// @retval None.
////////////////////////////////////////////////////////////////////////////////

void ADC_Polling()
{
    u16 adresult=0;
//    u16 rltarr[6];
    u32 tmp0=0;
    u32 VDDA=0;
    u32   Vref_calc=0;
    static u32 adcflags=0;

    if(KEYDBCFlags&0x1)
    {
        if(!(adcflags&(u32)1))
        {
            ADC_CHANNELxENcmd(ADC1,ADC_Channel_1,ENABLE);
            ADC_SoftwareStartConvCmd(ADC1,ENABLE);
            while(!((ADC1->DR)&ADC_DR_VALID));
            adresult=(u16)(ADC1->DR);
            ADC_CHANNELxENcmd(ADC1,ADC_Channel_1,DISABLE);

            tmp0=adresult*3300;
            tmp0/=4096;
#ifdef ClassB_VERBOSE_POR
            printf("ADC CH1 Volt is %04d mV\r\n", tmp0);
#endif
            adcflags|=((u32)1);
        }
    }
    else
    {
        adcflags&=~((u32)1);
    }

    if(KEYDBCFlags&0x2)
    {
        if(!(adcflags&(u32)2))
        {
            ADC_CHANNELxENcmd(ADC1,ADC_Channel_4,ENABLE);
            ADC_SoftwareStartConvCmd(ADC1,ENABLE);
            while(!((ADC1->DR)&ADC_DR_VALID));
            adresult=(u16)(ADC1->DR);
            ADC_CHANNELxENcmd(ADC1,ADC_Channel_4,DISABLE);
            tmp0=adresult*3300;
            tmp0/=4096;
#ifdef ClassB_VERBOSE_POR
            printf("ADC CH4 Volt is %04d mV\r\n", tmp0);
#endif
            adcflags|=((u32)2);
        }
    }
    else
    {
        adcflags&=~((u32)2);
    }

    if(KEYDBCFlags&0x4)
    {
        if(!(adcflags&(u32)4))
        {
            ADC_CHANNELxENcmd(ADC1,ADC_Channel_15,ENABLE);
            ADC_SoftwareStartConvCmd(ADC1,ENABLE);
            while(!((ADC1->DR)&ADC_DR_VALID));
            adresult=(u16)(ADC1->DR);
            ADC_CHANNELxENcmd(ADC1,ADC_Channel_15,DISABLE);
            tmp0=adresult*3300;
            tmp0/=4096;
            Vref_calc = (*(__IO uint16_t *)(REF_OFFSET));
            VDDA = Vref_calc*4096/adresult;
#ifdef ClassB_VERBOSE_POR
            printf("OnBoard VDDA  is %04d mV\r\n", VDDA);
#endif
            if(VDDA > VDD_MOST)
            {
#ifdef ClassB_VERBOSE
                printf("\n\r ADC REF CHECK ERROR  \n\r");
#endif  /* ClassB_VERBOSE */
            }
            else
            {
                if(VDDA < VDD_LEAST)
                {
#ifdef ClassB_VERBOSE
                    printf("\n\r ADC REF CHECK ERROR  \n\r");
#endif  /* ClassB_VERBOSE */
                }
            }
            adcflags|=((u32)4);
        }
    }
    else
    {
        adcflags&=~((u32)4);
    }


    if(KEYDBCFlags&0x8)
    {
        if(!(adcflags&(u32)8))
        {
            ADC_CHANNELxENcmd(ADC1,ADC_Channel_15,ENABLE);
            ADC_SoftwareStartConvCmd(ADC1,ENABLE);
            while(!((ADC1->DR)&ADC_DR_VALID));
            adresult=(u16)(ADC1->DR);
            ADC_CHANNELxENcmd(ADC1,ADC_Channel_15,DISABLE);
            tmp0=adresult*3300;
            tmp0/=4096;
            Vref_calc = (*(__IO uint16_t *)(REF_OFFSET));
            VDDA = Vref_calc*4096/adresult;
#ifdef ClassB_VERBOSE_POR
            printf("OnBoard VDDA  is %04d mV\r\n", VDDA);
#endif
            if(VDDA > VDD_MOST)
            {
#ifdef ClassB_VERBOSE
                printf("\n\r ADC REF CHECK ERROR  \n\r");
#endif  /* ClassB_VERBOSE */
            }
            else
            {
                if(VDDA < VDD_LEAST)
                {
#ifdef ClassB_VERBOSE
                    printf("\n\r ADC REF CHECK ERROR  \n\r");
#endif  /* ClassB_VERBOSE */
                }
            }
            adcflags|=((u32)8);
        }
    }
    else
    {
        adcflags&=~((u32)8);
    }
}

/// @}

/// @}

/// @}
