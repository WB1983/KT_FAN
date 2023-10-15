/** 
 * @file     drv_adc.h
 * @author   Motor TEAM
 * @brief    This file provides all the driver functions prototypes for the ADC.
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
#ifndef __DRV_ADC_H
#define __DRV_ADC_H

/** Files includes */
#include <stdio.h>
#include "hal_adc.h"

/** 
 * @addtogroup MM32_Hardware_Driver_Layer
 * @{
 */

/** 
 * @addtogroup Drv_ADC
 * @{
 */
 

 
typedef struct ADC_Channel
{
    uint8_t u8Rank;
    ADCCHANNEL_TypeDef sAdcChannel;
    struct ADC_Channel * pNext;
}ADC_Channel_TypeDef;

 
#define READ_ADC2_EOC_FLAG()      READ_BIT(ADC2->ADSTA, ADC_IT_EOC)
#define CLEAN_ADC2_EOC_FLAG()     SET_BIT(ADC2->ADSTA, ADC_IT_EOC)

/** ADC Sequential sampling */
#if 0
#define GET_ADC_VALUE(Channel)      (READ_REG(ADC1->DR##Channel) & 0xFFF)
#endif
/** ADC Injection pattern */
#if 0
#define GET_ADC1_VALUE(rank)         (READ_REG(ADC1->ADDR##rank) & 0xFFF)
#define GET_ADC2_VALUE(rank)         (READ_REG(ADC2->ADDR##rank) & 0xFFF)
#endif

#define GET_ADC1_VALUE(rank)         (READ_REG(*(&(ADC1->ADDR0) + rank)) & 0xFFF)
#define GET_ADC2_VALUE(rank)         (READ_REG(*(&(ADC2->ADDR0) + rank)) & 0xFFF)

#define GET_VREF_VALUE()            (READ_REG(*(&(ADC1->JDR0))) & 0xFFF)   
#define GET_VTEMP_VALUE()           (READ_REG(*(&(ADC1->JDR1))) & 0xFFF)

extern void Drv_Adc_Basic_Init(ADC_TypeDef* pAdc, EXTERTRIG_TypeDef ADC_ExternalTrigConv);
extern void Drv_Adc_Channel_Init(ADC_TypeDef* pAdc, ADC_Channel_TypeDef* pAdcChannel,uint32_t s32SampleTime);

extern void Drv_Adc_Inject_Channel_Init(void);

/**
  * @}
*/

/**
  * @}
*/


#endif
