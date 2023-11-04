////////////////////////////////////////////////////////////////////////////////
/// @file    KEY.c
/// @author  NJ ClassB TEAM
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
#define _KEY_C_

// Files includes
#include <stdio.h>
#include "key_classb.h"
#include "led_classb.h"
#include "IEC60730_B_param.h"

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
u32 KEYDBCFlags=0;
void Key1_Down()
{
    static u32 dbccnt=0;
    if(KEY1==Bit_SET)
    {
        if(dbccnt++>80)
        {
            dbccnt=82;
        }
        else if(dbccnt>70)
        {
            LED1_TOGGLE();
#ifdef ClassB_VERBOSE_POR
            printf("KEY1 put down!\r\n");
#endif
            dbccnt=82;
            KEYDBCFlags|=((u32)1);
        }
    }
    else
    {
        dbccnt=0;
        KEYDBCFlags&=~((u32)1);
    }
}

void Key2_Down()
{
    static u32 dbccnt=0;
    if(KEY2==Bit_RESET)
    {
        if(dbccnt++>80)
        {
            dbccnt=82;
        }
        else if(dbccnt>70)
        {
            LED2_TOGGLE();
#ifdef ClassB_VERBOSE_POR
            printf("KEY2 put down!\r\n");
#endif
            dbccnt=82;
            KEYDBCFlags|=((u32)2);
        }
    }
    else
    {
        dbccnt=0;
        KEYDBCFlags&=~((u32)2);
    }
}

void Key3_Down()
{

    static u32 dbccnt=0;
    if(KEY3==Bit_RESET)
    {
        if(dbccnt++>80)
        {
            dbccnt=82;
        }
        else if(dbccnt>70)
        {
            LED3_TOGGLE();
#ifdef ClassB_VERBOSE_POR
            printf("KEY3 put down!\r\n");
#endif
            dbccnt=82;
            KEYDBCFlags|=((u32)4);
        }
    }
    else
    {
        dbccnt=0;
        KEYDBCFlags&=~((u32)4);
    }
}

void Key4_Down()
{
    static u32 dbccnt=0;
    if(KEY4==Bit_RESET)
    {
        if(dbccnt++>80)
        {
            dbccnt=82;
        }
        else if(dbccnt>70)
        {
            LED4_TOGGLE();
#ifdef ClassB_VERBOSE_POR
            printf("KEY4 put down!\r\n");
#endif
            KEYDBCFlags|=((u32)8);
            dbccnt=82;
        }
    }
    else
    {
        dbccnt=0;
        KEYDBCFlags&=~((u32)8);
    }

}
void KEY_Polling()
{
    Key1_Down();
    Key2_Down();
    Key3_Down();
    Key4_Down();
}
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void KEY_Config()
{
    GPIO_InitTypeDef GPIO_InitStruct;
    RCC_GPIO_ClockCmd(GPIOA, ENABLE);
    RCC_GPIO_ClockCmd(GPIOB, ENABLE);
    RCC_GPIO_ClockCmd(GPIOC, ENABLE);

    GPIO_StructInit(&GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin  = KEY1_Pin;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(KEY1_GPIO_Port, &GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin  = KEY2_Pin;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(KEY2_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Pin  = KEY3_Pin;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(KEY3_GPIO_Port, &GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin  = KEY4_Pin;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(KEY4_GPIO_Port, &GPIO_InitStruct);
}
