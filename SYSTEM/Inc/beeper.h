////////////////////////////////////////////////////////////////////////////////
/// @file    uart.h
/// @author  AE TEAM
/// @brief   THIS FILE PROVIDES ALL THE SYSTEM FIRMWARE FUNCTIONS.
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
#ifndef __BEAPER_H
#define __BEAPER_H
// Files includes
#include "mm32_device.h"


////////////////////////////////////////////////////////////////////////////////
#define BEEPER_Port  GPIOA
#define BEEPER_Pin   GPIO_Pin_8

#define BEEPER_ON()  GPIO_ResetBits(BEEPER_Port,BEEPER_Pin)
#define BEEPER_OFF()  GPIO_SetBits(BEEPER_Port,BEEPER_Pin)
////////////////////////////////////////////////////////////////////////////////

void BEEPER_Config(void);
void BEEPER_Polling(void);

#endif
////////////////////////////////////////////////////////////////////////////////

