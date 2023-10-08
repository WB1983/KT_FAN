////////////////////////////////////////////////////////////////////////////////
/// @file    delay.c
/// @author  AE TEAM
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
#define _DELAY_C_

// Files includes
#include <stdio.h>
#include "delay.h"
#include "hal_device.h"
#include "IEC60730_B_param.h"
#include "IEC60730_B_lib.h"

////////////////////////////////////////////////////////////////////////////////
/// @addtogroup MM32_Hardware_Abstract_Layer
/// @{

////////////////////////////////////////////////////////////////////////////////
/// @addtogroup DELAY
/// @{

////////////////////////////////////////////////////////////////////////////////
/// @addtogroup DELAY_Exported_Functions
/// @{

#define USE_SYSTICK_DELAY 1
//1 = use systick as the delay,
//0 = use NOP loop as the delay
#if USE_SYSTICK_DELAY
extern u32 SystemCoreClock;
static vu32 sTimingDelay;
extern __IO uint32_t uwTick;

////////////////////////////////////////////////////////////////////////////////
/// @brief  Initialize systick for delay function
/// @note   This function should affected by chip version /8 or /1.
/// @param  None.
/// @retval None.
////////////////////////////////////////////////////////////////////////////////
void DELAY_Init(void)
{
    if (SysTick_Config(SystemCoreClock / 1000))
    {
        while (1);
    }
    NVIC_SetPriority(SysTick_IRQn, 0x0);
}

//__weak uint32_t HAL_GetTick(void)
//{
//    return uwTick;
//}


////////////////////////////////////////////////////////////////////////////////
/// @brief  Set the count as delay counter, it is break until counter goes to 0.
/// @note   use sTimingDelay.
/// @param  count specifies the delay tick number
/// @retval None.
////////////////////////////////////////////////////////////////////////////////
void DELAY_Ms(vu32 count)
{
    sTimingDelay = count;

    while(sTimingDelay != 0);
}
#else
static vu32 sDelayNopNumber;
////////////////////////////////////////////////////////////////////////////////
/// @brief  Initialize for delay function
/// @note   None.
/// @param  None.
/// @retval None.
////////////////////////////////////////////////////////////////////////////////
void DELAY_Init(void)
{
    //Custom
    sDelayNopNumber = 100;
}
////////////////////////////////////////////////////////////////////////////////
/// @brief  Set the count as delay counter, it is break until counter goes to 0.
/// @note   None.
/// @param  count specifies the delay tick number
/// @retval None.
////////////////////////////////////////////////////////////////////////////////
void DELAY_Us(vu32 count)
{
    u32 i = 0;
    while(count--)
    {
        i = sDelayNopNumber;
        while(i--);
    }
}
////////////////////////////////////////////////////////////////////////////////
/// @brief  Set the count as delay counter, it is break until counter goes to 0.
/// @note   None.
/// @param  count specifies the delay tick number
/// @retval None.
////////////////////////////////////////////////////////////////////////////////
void DELAY_Ms(vu32 count)
{
    u32 i = 0;
    while(count--)
    {
        i = sDelayNopNumber * 1000;
        while(i--);
    }
}
#endif


/// @}

/// @}

/// @}
