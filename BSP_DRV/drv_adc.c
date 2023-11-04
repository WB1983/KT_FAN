/** 
 * @file     drv_adc.c
 * @author   MindMotion Motor Team : Wesson
 * @brief    This file provides all the driver functions for the ADC.
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
#define _DRV_ADC_C_

/** Files includes */
#include "hal_rcc.h"
#include "hal_misc.h"

#include "drv_adc.h"
/** 
 * @addtogroup MM32_Hardware_Driver_Layer
 * @{
 */

/** 
 * @addtogroup Drv_ADC
 * @{
 */
 
void Drv_Adc_Basic_Init(ADC_TypeDef* pAdc, EXTERTRIG_TypeDef ADC_ExternalTrigConv)
{
    /** Define the struct of ADC configuration */
    ADC_InitTypeDef  ADC_InitStructure;
    ADC_StructInit(&ADC_InitStructure);

    /** Enable the ADC1 clock */
    if (ADC1 == pAdc)
    {
        RCC_APB2PeriphClockCmd(RCC_APB2ENR_ADC1, ENABLE);
    }
    else if (ADC2 == pAdc)
    {
        RCC_APB2PeriphClockCmd(RCC_APB2ENR_ADC2, ENABLE);
    }
    else
    {
        return;
    }
    /** Calibration ADC1 */
    ADC_Calibration(pAdc);
    /**
     * Initialize the ADC to 12bit
     * ADC clock 6 frequency division
     * Single period mode
     * ADC conversion is triggered by TIM1_CCR4
     * Data right aligned
     */
    ADC_InitStructure.ADC_Resolution            = ADC_Resolution_12b;
    ADC_InitStructure.ADC_PRESCARE              = ADC_PCLK2_PRESCARE_2;
    ADC_InitStructure.ADC_Mode                  = ADC_Mode_Single_Period;
    ADC_InitStructure.ADC_ExternalTrigConv      = ADC_ExternalTrigConv;
    ADC_InitStructure.ADC_DataAlign             = ADC_DataAlign_Right;
    
    ADC_Init(pAdc, &ADC_InitStructure);

    /** External triggering was enabled */
    ADC_ExternalTrigConvCmd(pAdc, ENABLE);
    
    ADC_Cmd(pAdc, DISABLE);
}

void Drv_Adc_Channel_Init(ADC_TypeDef* pAdc, ADC_Channel_TypeDef* pAdcChannel,uint32_t s32SampleTime)
{
    uint8_t u8Temp = 0;
    if ((pAdc == NULL) || (pAdcChannel == NULL))
    {
        return ;
    }
    do
    {
        /** Sample time selection to s32SampleTime */
        ADC_RegularChannelConfig(pAdc, pAdcChannel->sAdcChannel, 0, s32SampleTime);
        if (u8Temp < pAdcChannel->u8Rank)
        {
            /**Configures the adc any channels conversion Max rank number by macro definition */
            ADC_ANY_NUM_Config(pAdc, pAdcChannel->u8Rank);
        }
        /** Configures the adc any channels conversion rank and channelby macro definition */
        ADC_ANY_CH_Config(pAdc, pAdcChannel->u8Rank, pAdcChannel->sAdcChannel);

        u8Temp = pAdcChannel->u8Rank;
        
        pAdcChannel = pAdcChannel->pNext;
    } while(pAdcChannel != NULL);
    /** Enables the ANY channel converter */
    ADC_ANY_Cmd(pAdc, ENABLE);
}


/**************************************************************Vref Inject convertion***************************************************/
void Drv_Adc_Inject_Channel_Init(void)
{

	/* Select the ADC sample time*/
	ADC_RegularChannelConfig(ADC1, ADC_Channel_VoltReference, 0, ADC_SMPR_SAMCTL_240_5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_TempSensor, 0, ADC_SMPR_SAMCTL_240_5);
	/* Auto injection mode */
	ADC_AutoInjectedConvCmd(ADC1, ADC_ANY_CR_JAUTO);
	/* ADC_Inject_Sequence_Length the sequencer length for injected channels*/
	ADC_InjectedSequencerLengthConfig(ADC1, ADC_Inject_Seqen_Len2);
	/* select ISUM_CHANNEL as Injection channel	*/
	ADC_InjectedSequencerChannelConfig(ADC1, ADC_InjectedChannel_1, ADC_Channel_VoltReference);  
	ADC_InjectedSequencerChannelConfig(ADC1, ADC_InjectedChannel_2, ADC_Channel_TempSensor); 
	/* Injection enable */
	ADC_InjectedConvCmd(ADC1, ADC_ANY_CR_JCEN);
	//ADC1->ANYCR |= ADC_ANY_CR_JEOSIE;


}




/**
  * @}
*/

/**
  * @}
*/
