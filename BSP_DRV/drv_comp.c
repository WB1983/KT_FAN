/** 
 * @file     drv_comp.c
 * @author   MindMotion Motor Team : Wesson
 * @brief    This file provides all the driver functions for the COMP.
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
#define _DRV_COMP_C_

/** Files includes */
#include "drv_comp.h"

/** 
 * @addtogroup MM32_Hardware_Driver_Layer
 * @{
 */

/** 
 * @addtogroup Drv_COMP
 * @{
 */

/**
* @brief    : This function describes the underlying configuration of the comparator.
* @param    : None
* @retval   : None
*/
void Drv_Comp_Init(COMP_Selection_TypeDef sSelection,COMP_Input_TypeDef * pCompInput)
{
    COMP_InitTypeDef COMP_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2ENR_COMP, ENABLE);

    COMP_DeInit(sSelection);
    COMP_InitStructure.COMP_NonInvertingInput = pCompInput->sCompNonInvertingInput;
    COMP_InitStructure.COMP_InvertingInput    = pCompInput->sCompInvertingInput;
    COMP_InitStructure.COMP_Output            = COMP_Output_TIM1BKIN;
    COMP_InitStructure.COMP_OutputPol         = COMP_Inverted;
    COMP_InitStructure.COMP_Hysteresis        = COMP_Hysteresis_Medium;
    COMP_InitStructure.COMP_Mode              = COMP_Mode_MediumSpeed;
    COMP_InitStructure.COMP_Filter            = COMP_Filter_64_Period;

    COMP_Init(sSelection, &COMP_InitStructure);
    if (COMP_InvertingInput_CRV == pCompInput->sCompInvertingInput)
    {
        COMP_SetCrv(COMP_CRV_SRC_AVDD, pCompInput->u8CompCrvSelect);
    }
    COMP_Cmd(sSelection, ENABLE);
}

/**
  * @}
*/

/**
  * @}
*/
