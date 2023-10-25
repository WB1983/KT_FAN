/**
 * @file     drv_dma.c
 * @author   Motor TEAM
 * @brief    This file provides all the driver functions for the DMA.
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
#define _DRV_DMA_C_

/** Files includes */
#include "drv_dma.h"

/**
 * @addtogroup MM32_Hardware_Driver_Layer
 * @{
 */

/**
 * @addtogroup Drv_DMA
 * @{
 */

/**
 * @brief    : Configure DMA channel 4 to transfer data from memory to TIM1CCR4.
 * @param    : *pDmaData
 * The memory address to be transferred by DMA
 * @retval   : None
 */
void DMA_Channel4_Init(uint16_t *pDmaData)
{
    DMA_InitTypeDef  DMA_InitStruct;
    RCC_AHBPeriphClockCmd(RCC_AHBENR_DMA1, ENABLE);

    DMA_DeInit(DMA1_Channel4);
    DMA_StructInit(&DMA_InitStruct);
    /** Transfer register address */
    DMA_InitStruct.DMA_PeripheralBaseAddr   = (uint32_t) & (TIM3 ->CCR1);
    /** Transfer memory address */
    DMA_InitStruct.DMA_MemoryBaseAddr       = (uint32_t) pDmaData;
    /** Transfer direction, from register to memory */
    DMA_InitStruct.DMA_DIR                  = DMA_DIR_PeripheralSRC;
    DMA_InitStruct.DMA_BufferSize           = 1;
    DMA_InitStruct.DMA_PeripheralInc        = DMA_PeripheralInc_Disable;
    /** Transfer completed memory address increment */
    DMA_InitStruct.DMA_MemoryInc            = DMA_MemoryInc_Disable;
    DMA_InitStruct.DMA_PeripheralDataSize   = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStruct.DMA_MemoryDataSize       = DMA_MemoryDataSize_HalfWord;
    DMA_InitStruct.DMA_Mode                 = DMA_Mode_Normal;
    DMA_InitStruct.DMA_Priority             = DMA_Priority_High;
    DMA_InitStruct.DMA_M2M                  = DMA_M2M_Disable;
    DMA_InitStruct.DMA_Auto_reload          = DMA_Auto_Reload_Enable;
    DMA_Init(DMA1_Channel4, &DMA_InitStruct);

	
}

/**
  * @}
*/

/**
  * @}
*/

