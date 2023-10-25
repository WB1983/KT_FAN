////////////////////////////////////////////////////////////////////////////////
/// @file     key.h
/// @author   AE TEAM
/// @brief    THIS FILE CONTAINS ALL THE FUNCTIONS PROTOTYPES FOR THE SERIES OF
///           MM32 FIRMWARE LIBRARY.
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

#ifndef __KEY_H
#define __KEY_H

// Files includes
// Files includes
#include <string.h>
#include "mm32_device.h"
#include "hal_conf.h"

#define KEY1_GPIO_Port                  GPIOB
#define KEY1_Pin                        GPIO_Pin_1
#define KEY2_GPIO_Port                  GPIOB
#define KEY2_Pin                        GPIO_Pin_2
#define KEY3_GPIO_Port                  GPIOB
#define KEY3_Pin                        GPIO_Pin_10
#define KEY4_GPIO_Port                  GPIOB
#define KEY4_Pin                        GPIO_Pin_11
#define WK_UP_GPIO_Port                 GPIOA
#define WK_UP_Pin                       GPIO_Pin_0

#define KEY1                            GPIO_ReadInputDataBit(KEY1_GPIO_Port,KEY1_Pin)  //read key1
#define KEY2                            GPIO_ReadInputDataBit(KEY2_GPIO_Port,KEY2_Pin)  //read key2
#define KEY3                            GPIO_ReadInputDataBit(KEY3_GPIO_Port,KEY3_Pin)  //read key3
#define KEY4                            GPIO_ReadInputDataBit(KEY4_GPIO_Port,KEY4_Pin)  //read key4
#define WK_UP                           GPIO_ReadInputDataBit(WK_UP_GPIO_Port,WK_UP_Pin)//read wk_up

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void KEY_Config(void);
void KEY_Polling(void);


#endif
