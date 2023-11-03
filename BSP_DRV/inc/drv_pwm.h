/** 
 * @file     drv_pwm.h
 * @author   Motor TEAM
 * @brief    This file provides all the driver functions prototypes for the PWM.
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
#ifndef __DRV_PWM_H
#define __DRV_PWM_H

/** Files includes */
#include "HAL_device.h"
#include "HAL_conf.h"
#include "common.h"
/** 
 * @addtogroup MM32_Hardware_Driver_Layer
 * @{
 */

/** 
 * @addtogroup Drv_PWM
 * @{
 */
/**
 * \brief	Enumerated type definition of the supported active output channel configurations
 * */
typedef enum _TChannels {
    PWM_CH_ALL_OFF,   /**< 0  All PWM channels off */
    PWM_CH_ALL_ON,    /**< 1  All PWM channels on */
    PWM_CH_U,         /**< 2  PWM channel phase U on (high and low) */
    PWM_CH_V,         /**< 3  PWM channel phase V on (high and low) */
    PWM_CH_W,         /**< 4  PWM channel phase W on (high and low) */
    PWM_CH_U_V,       /**< 5  PWM channel phase U and V on (high and low) */
    PWM_CH_V_W,       /**< 6  PWM channel phase V and W on (high and low) */
    PWM_CH_W_U,       /**< 7  PWM channel phase W and U on (high and low) */
    PWM_CH_HIN_U,     /**< 8  PWM channel phase U high side on */
    PWM_CH_HIN_V,     /**< 9  PWM channel phase V high side on */
    PWM_CH_HIN_W,     /**< 10 PWM channel phase W high side on */
    PWM_CH_LIN_U,     /**< 11 PWM channel phase U low side on */
    PWM_CH_LIN_V,     /**< 12 PWM channel phase V low side on */
    PWM_CH_LIN_W,     /**< 13 PWM channel phase W low side on */
    PWM_NO_OF_PATTERNS
}TChannels;


#define SET_CCR1_VAL(Value)         WRITE_REG(TIM1->CCR1, Value)
#define SET_CCR2_VAL(Value)         WRITE_REG(TIM1->CCR2, Value)
#define SET_CCR3_VAL(Value)         WRITE_REG(TIM1->CCR3, Value)
#define SET_CCR4_VAL(Value)         WRITE_REG(TIM1->CCR4, Value)

#define DISABLE_PWMOUT()            CLEAR_BIT(TIM1->BDTR, TIM_BDTR_MOEN)
#define ENABLE_PWMOUT()             SET_BIT(TIM1->BDTR, TIM_BDTR_MOEN)

#define READ_TIM1_UPDATE_FLAG()     READ_BIT(TIM1->SR, TIM_IT_Update)
#define CLEAN_TIM1_UPDATE_FLAG()    CLEAR_BIT(TIM1->SR, TIM_IT_Update)

#define TIM1_BREAK_ENABLE()         SET_BIT(TIM1->BDTR,TIM_BDTR_BKE)
#define TIM1_BREAK_DISABLE()        CLEAR_BIT(TIM1->BDTR,TIM_BDTR_BKE)
#define READ_TIM1_BREAK_FLAG()      READ_BIT(TIM1->SR, TIM_IT_Break)
#define CLEAN_TIM1_BREAK_FLAG()     CLEAR_BIT(TIM1->SR, TIM_IT_Break)

#define PWM_DUTYCYCLE_0_PERCENT             0       /**< 50% Duty cycle */
#define PWM_DUTYCYCLE_50_PERCENT        16384       /**< 50% Duty cycle */
#define PWM_DUTYCYCLE_100_PERCENT       32768       /**< 100% Duty cycle => High side static on */

#define PWM_THREE_DUTY_ZERO {PWM_DUTYCYCLE_0_PERCENT,PWM_DUTYCYCLE_0_PERCENT,PWM_DUTYCYCLE_0_PERCENT}

extern void Drv_Pwm_Init(TIM_TypeDef * pTim, uint16_t u16Period, uint16_t u16DeadTime);

extern bool TIM_bBrakeUpdate(void);
extern void TIM_vBrakeInit(uint16_t u16BrakeTime);

extern BOOL TIM_bSetOutputConfig(TChannels            tActiveChannels, TDutyCycleConfig tCompareValue);
/**
  * @}
*/

/**
  * @}
*/

#endif
