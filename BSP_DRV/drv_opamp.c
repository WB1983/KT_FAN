/** 
 * @file     drv_opamp.c
 * @author   MindMotion Motor Team : Wesson
 * @brief    This file provides all the driver functions for the OPAMP.
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
#define _DRV_OPAMP_C_

/** Files includes */
#include "drv_opamp.h"

/** 
 * @addtogroup MM32_Hardware_Driver_Layer
 * @{
 */

/** 
 * @addtogroup Drv_OPAMP
 * @{
 */

/**
* @brief    : This function describes the underlying configuration of the op-amp.
* @param    : None
* @retval   : None
*/
void Drv_Opamp_Init(void)
{
	 /* op-amp Clock Enable */
    RCC_APB2PeriphClockCmd(RCC_APB2ENR_OPAMP, ENABLE);
    /*Enable the specified OPAMP peripheral*/
    OPAMP_Configure(OPAMP1,ENABLE);
    OPAMP_Configure(OPAMP3,ENABLE);
#if THREE_SHUNT_R
    /** For Motor-DK IR_U */
    OPAMP_Configure(OPAMP2,ENABLE);
#endif
}
/**
  * @}
*/

/**
  * @}
*/
