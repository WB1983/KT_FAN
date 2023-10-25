/*
 * @Author: Wang Bing
 * @Date: 2021-11-01 15:21:13
 * @LastEditTime: 2023-02-08 09:32:54
 * @LastEditors: Wangbingbin bing.wang2@bshg.com
 * @Description: IIC_SEN communication with converter
 * @FilePath: \PUc:\Users\WangBingBin\AppData\Local\WangBing\Projects\EXTERNAL\Pressure\Pressure_16F18346.X\Pressure_16F18346.X\Application\PhyDriver\IIC_Sensor.h
 */


#ifndef _IIC_SEN
#define _IIC_SEN

 /**************************************************************************************
*                                   Includes                                           *
 **************************************************************************************/ 
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

#include "mm32_device.h"


 /**************************************************************************************
*                                   definitions                                        *
 **************************************************************************************/ 

#define GPIO_IIC_CLK     GPIO_Pin_7
#define GPIO_IIC_SDA     GPIO_Pin_6



#define GPIO_SEN_IIC_CLK_OFF()  GPIO_ResetBits(GPIOC,GPIO_IIC_CLK)
#define GPIO_SEN_IIC_CLK_ON()   GPIO_SetBits(GPIOC,GPIO_IIC_CLK)
#define GPIO_SEN_IIC_SDA_OFF()  GPIO_ResetBits(GPIOC,GPIO_Pin_1)
#define GPIO_SEN_IIC_SDA_ON()   GPIO_SetBits(GPIOC,GPIO_Pin_1)


#define IIC_SEN_SCK_0           GPIO_SEN_IIC_CLK_OFF()       //set RC4 to low     
#define IIC_SEN_SCK_1           GPIO_SEN_IIC_CLK_ON()        //set RC4 to high
#define IIC_SEN_SDA_0           GPIO_SEN_IIC_SDA_OFF()       //set RC3 to low       
#define IIC_SEN_SDA_1           GPIO_SEN_IIC_SDA_OFF()       //set RC3 to High


#define PSR_IIC_WR_ADD          0xDA  //Write data to IIC
#define PSR_IIC_RD_ADD          0xDB  //Read data from IIC
 /**************************************************************************************
*                                   functions                                          *
 **************************************************************************************/ 
extern void IIC_vWriteAdd(uint8_t u8Add, uint8_t u8Data);

extern uint8_t IIC_vReadAdd(uint8_t u8Add);

extern uint8_t IIC_u8ReadSDA(void);

#endif
