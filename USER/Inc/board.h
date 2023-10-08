/**
 * @file     board.h
 * @author   MindMotion Motor Team : Wesson
 * @brief    This file provides all the functions prototypes for the board level support package.
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
#ifndef __BOARD_H
#define __BOARD_H

/** Files includes */
#include <stdio.h>
#include "mm32_device.h"
#include "hal_conf.h"

/** Interrupt priorities */
/**
 * The order of priority decreases from small to large
 * 0 : highest priority-first
 */
#define SYSTICK_INTERRUPT           (1)
#define TIM1_UPDATE_INTERRUPT       (2)
#define ADC2_INTERRUPT              (0)
#define UART3_RX_INTERRUPT          (3)
#define TIM17_INTERRUPT             (3)
#define TIM3_INTERRUPT              (0)

/** ADC interface */
//ADC1
#define IR_U_CHANNEL                ADC_Channel_6   //ADC1 CH6
#define IR_U_RANK                   (0)

#define VBUS_CHANNEL                ADC_Channel_0   //ADC1 CH0
#define VBUS_RANK                   (1)

#define VL_15_CHANNEL               ADC_Channel_1   //ADC1 CH1
#define VF_RANK                     (2)

#define IN_VL_CHANNEL               ADC_Channel_VoltReference   //ADC1 CH14
#define IV_RANK                     (3)

#define IN_TP_CHANNEL               ADC_Channel_TempSensor   //ADC1 CH15
#define IP_RANK                     (4)

//ADC2
#define IR_V_CHANNEL                ADC_Channel_11   //ADC2 CH11
#define IR_V_RANK                   (0)

#define ISUM_CHANNEL                ADC_Channel_7   //ADC2 CH12
#define ISUM_RANK                   (1)

#define NTC_CHANNEL                 ADC_Channel_0   //ADC2 CH3
#define NTC_RANK                    (2)

#define SO_1R_CHANNEL               ADC_Channel_12   //ADC2 CH12
#define SO_RANK                     (3)

#if THREE_SHUNT_R
#define IR_W_CHANNEL                ADC_Channel_10   //ADC2 CH10
#define IR_W_RANK                   (4)
#endif

#define ADC_GPIO_CLK               (RCC_AHBENR_GPIOA | RCC_AHBENR_GPIOB)
#define NTC_PORT                     GPIOA
#define NTC_PIN                      GPIO_Pin_10

#define VBUS_PORT                   GPIOA
#define VBUS_PIN                    GPIO_Pin_0
/** Without opamp */
#define ISUM_PORT                   GPIOA
#define ISUM_PIN                    GPIO_Pin_7
/** Used opamp */
#if THREE_SHUNT_R
#define IR_W_PORT                   GPIOB
#define IR_W_PIN                    GPIO_Pin_2
#endif
#define IR_U_PORT                   GPIOA
#define IR_U_PIN                    GPIO_Pin_6

#define IR_V_PORT                   GPIOB
#define IR_V_PIN                    GPIO_Pin_10

#define SO_C_PORT                   GPIOA
#define SO_C_PIN                    GPIO_Pin_9

#define VOL_15_PORT                 GPIOA
#define VOL_15_PIN                  GPIO_Pin_1



/** COMP interface */
#define USE_CRV                     1
#define COMP_NUMBER                 COMP5
#define COMP_NON_INVERTING          COMP_NonInvertingInput_IO0                                      /** for PA11 */

#if USE_CRV
#define COMP_INVERTING              COMP_InvertingInput_CRV
#define COMP_CRV_VOLTAGE_SELECT     COMP_CRV_2_20//change to 7_20 from 3_20
#else
#define COMP_INVERTING              COMP_InvertingInput_IO2                                         /** for PA12 */
#define COMP_INM_PORT               GPIOA
#define COMP_INM_PIN                GPIO_Pin_12
#endif
#define COMP_GPIO_CLK               (RCC_AHBENR_GPIOA)

#define COMP_INP_PORT               GPIOA
#define COMP_INP_PIN                GPIO_Pin_11

/** OPAMP interface */
#define OPAMP_GPIO_CLK              (RCC_AHBENR_GPIOA | RCC_AHBENR_GPIOB)
#define OPAMP1_INM_PORT             GPIOA
#define OPAMP1_INM_PIN              GPIO_Pin_5
#define OPAMP1_INP_PORT             GPIOA
#define OPAMP1_INP_PIN              GPIO_Pin_4
#define OPAMP2_INM_PORT             GPIOB
#define OPAMP2_INM_PIN              GPIO_Pin_1
#define OPAMP2_INP_PORT             GPIOB
#define OPAMP2_INP_PIN              GPIO_Pin_0
#define OPAMP3_INM_PORT             GPIOB
#define OPAMP3_INM_PIN              GPIO_Pin_11
#define OPAMP3_INP_PORT             GPIOB
#define OPAMP3_INP_PIN              GPIO_Pin_12
#define OPAMP4_INM_PORT             GPIOA
#define OPAMP4_INM_PIN              GPIO_Pin_8
#define OPAMP4_INP_PORT             GPIOB
#define OPAMP4_INP_PIN              GPIO_Pin_15

/** LED interface */
#define LED_RCC_CLOCKGPIO           (RCC_AHBENR_GPIOA)

//#define LED1_PORT                   GPIOC
//#define LED1_PIN                    GPIO_Pin_5

#define LED2_PORT                   GPIOA
#define LED2_PIN                    GPIO_Pin_15

//#define LED1_ON()                   LED1_PORT->BRR = LED1_PIN
//#define LED1_OFF()                  LED1_PORT->BSRR = LED1_PIN
#define LED2_ON()                   LED2_PORT->BRR = LED2_PIN
#define LED2_OFF()                  LED2_PORT->BSRR = LED2_PIN
//#define LED1_TOGGLE()               LED1_PORT->ODR ^= LED1_PIN
#define LED2_TOGGLE()               LED2_PORT->ODR ^= LED2_PIN

/** PWM interface */
#define PWM_L_USE_IO   0
#define PWM_L_USE_TIM  1

#define BLDC1_GPIO_CLK               (/*RCC_AHBENR_GPIOA | */RCC_AHBENR_GPIOB)

#define BLDC1_UH_PORT                GPIOB
#define BLDC1_UH_PIN                 GPIO_Pin_8
#define BLDC1_VH_PORT                GPIOB
#define BLDC1_VH_PIN                 GPIO_Pin_5
#define BLDC1_WH_PORT                GPIOB
#define BLDC1_WH_PIN                 GPIO_Pin_7

#define BLDC1_UL_PORT                GPIOB
#define BLDC1_UL_PIN                 GPIO_Pin_6
#define BLDC1_VL_PORT                GPIOB
#define BLDC1_VL_PIN                 GPIO_Pin_3
#define BLDC1_WL_PORT                GPIOB
#define BLDC1_WL_PIN                 GPIO_Pin_4

//#define BLDC1_BKP_PORT               GPIOA
//#define BLDC1_BKP_PIN                GPIO_Pin_6

#define BLDC1_UH_PIN_SRC             GPIO_PinSource8
#define BLDC1_VH_PIN_SRC             GPIO_PinSource5
#define BLDC1_WH_PIN_SRC             GPIO_PinSource7
#define BLDC1_UL_PIN_SRC             GPIO_PinSource6
#define BLDC1_VL_PIN_SRC             GPIO_PinSource3
#define BLDC1_WL_PIN_SRC             GPIO_PinSource4
//#define BLDC1_BKP_PIN_SRC            GPIO_PinSource6

#define BLDC1_UH_PIN_AF              GPIO_AF_3
#define BLDC1_VH_PIN_AF              GPIO_AF_7
#define BLDC1_WH_PIN_AF              GPIO_AF_6
#define BLDC1_UL_PIN_AF              GPIO_AF_7
#define BLDC1_VL_PIN_AF              GPIO_AF_6
#define BLDC1_WL_PIN_AF              GPIO_AF_6
//#define BLDC1_BKP_PIN_AF             GPIO_AF_7

/*****************************************capture/output IO****************************************/
#define TIM_OUT_PWM_GPIO_CLK        RCC_AHBENR_GPIOB
#define TIM_CAPTURE_GPIO_CLK        RCC_AHBENR_GPIOD


#define TIM17_CH1_PORT              GPIOB
#define TIM17_CH1_PIN               GPIO_Pin_13

#define TIM3_CH1_PORT               GPIOD
#define TIM3_CH1_PIN                GPIO_Pin_7

#define TIM17_CH1_PIN_SRC           GPIO_PinSource13
#define TIM3_CH1_PIN_SRC            GPIO_PinSource7

#define TIM17_CH1_PIN_AF            GPIO_AF_6
#define TIM3_CH1_PIN_AF             GPIO_AF_5


/*****************************************Direction io****************************************/
#define DIR_RCC_CLOCKGPIO           RCC_AHBENR_GPIOC
#define DIRECTION_PORT              GPIOC
#define DIRECTION_PIN               GPIO_Pin_13


/**
 * @addtogroup MM32_Hardware_Driver_Layer
 * @{
 */

/**
 * @addtogroup Bsp
 * @{
 */


extern void Bsp_Gpio_Init(void);
extern void Peripheral_Init(void);
/**
  * @}
*/

/**
  * @}
*/


#endif
