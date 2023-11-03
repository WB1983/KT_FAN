/** 
 * @file     drv_inc.h
 * @author   Motor TEAM
 * @brief    This file is used to include the driver header files used in this project
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
#ifndef __DRV_INC_H
#define __DRV_INC_H

/** Files includes */
#include <stdint.h>
#include "hal_conf.h"
#include "hal_device.h"
//#include "systick.h"
#include "board.h"
#include "drv_adc.h"
#include "drv_comp.h"
#include "drv_opamp.h"
#include "drv_pwm.h"
#include "drv_led.h"
#include "drv_iwdg.h"
#include "drv_div.h"
#include "drv_sqrt.h"
#include "mm32_it.h"

#include "drv_sci.h"
#include "drv_Output.h"
#include "drv_Capture.h"
#include "drv_Counter.h"
#include "drv_dma.h"
#include "drv_eep.h"
/** 
 * @addtogroup MM32_User_Layer
 * @{
 */

/** 
 * @addtogroup User_Main
 * @{
 */

/**
  * @}
*/

/**
  * @}
*/

#endif
