
////////////////////////////////////////////////////////////////////////////////
/// @file    Interrupts.c
/// @author  NJ ClassB TEAM
/// @brief   
///          
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
/* Define to prevent recursive inclusion -------------------------------------------------------------------*/
#ifndef __DIGITALIO_CLASSB_H
#define __DIGITALIO_CLASSB_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------------------------------------*/
#include "hal_gpio.h"

/* Settings ------------------------------------------------------------------------------------------------*/
/* Exported typedef ----------------------------------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------------------------------------*/
void IEC60730_GPIO_OutputInit(GPIO_TypeDef* GPIOx, u16 GPIO_PIN, BitAction DefaultStatus);
ErrorStatus IEC60730_GPIO_WriteOutBits(GPIO_TypeDef* GPIOx_OUT, u16 GPIO_PIN_OUT, BitAction Status);
ErrorStatus IEC60730_GPIO_OutputCheck(GPIO_TypeDef* GPIOx_OUT, u16 GPIO_PIN_OUT);
ErrorStatus IEC60730_GPIO_InputCheck(GPIO_TypeDef* GPIOx_IN, u16 GPIO_PIN_IN,
                                   GPIO_TypeDef* GPIOx_CMP, u16 GPIO_PIN_CMP);
ErrorStatus IEC60730_GPIO_InputValueCheck(GPIO_TypeDef* GPIOx_IN, u16 GPIO_PIN_IN, BitAction Status);
void IEC60730_DIOCheck(void);

#ifdef __cplusplus
}
#endif

#endif
