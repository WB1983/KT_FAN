/** 
 * @file     mm32_it.h
 * @author   Motor TEAM
 * @brief    This file provides all the functions prototypes for the Interrupt interface.
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

#ifndef __MM32_IT_H
#define __MM32_IT_H

/** Files includes */
#include "main.h"
#include "Common.h"
/** 
 * @addtogroup MM32_User_Layer
 * @{
 */

typedef struct _TInputCapture
{
	uint32_t u16LowCount;
	uint32_t u16HighCount;
	uint32_t u16PeriodCount;
}TInputCapture;

/** 
 * @addtogroup User_MM32_IT
 * @{
 */

extern CallbackFunc2 STK_pfGeneralTimerCallback;
extern void Interrupt_Init(void);

extern void STK_vSetGenericTimerCallbackFunc(CallbackFunc2  pfCallbackfunction);

extern void UART_vSetUartReceiveCallbackFunc(CallbackFunc2  pfCallbackfunction);

extern void STK_vSetGenericTimerCallbackFuncUART3(CallbackFunc2  pfCallbackfunction);

extern void UART_vSetUartReceiveCallbackFuncUART3(CallbackFunc2  pfCallbackfunction);

extern TInputCapture * MIT_ptGetCaptureData(void);
/**
  * @}
*/

/**
  * @}
*/


#endif
