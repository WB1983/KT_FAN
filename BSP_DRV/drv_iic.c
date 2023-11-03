#include "drv_iic.h"

/***IIC GPIO configuration***/
void DRV_vIICSDAConfigure(SDADirection u8Direction)
{
    if(u8Direction == eINPUT_MODE)
    {
        GPIO_InitTypeDef  GPIO_InitStruct;
        RCC_AHBPeriphClockCmd(RCC_AHBENR_GPIOD, ENABLE);
        GPIO_StructInit(&GPIO_InitStruct);

        GPIO_InitStruct.GPIO_Pin  =  GPIO_Pin_2;
        GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
        GPIO_Init(GPIOD, &GPIO_InitStruct);
    }
    else if(u8Direction == eOUTPUT_MODE)
    {
        GPIO_InitTypeDef  GPIO_InitStruct;
        RCC_AHBPeriphClockCmd(RCC_AHBENR_GPIOD, ENABLE);
        GPIO_StructInit(&GPIO_InitStruct);

        GPIO_InitStruct.GPIO_Pin  =  GPIO_Pin_2;
        GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
        GPIO_Init(GPIOD, &GPIO_InitStruct);
    }
}

