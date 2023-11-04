/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2023-03-13 23:23:37
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2023-03-14 22:39:27
 * @FilePath: \Pressure_221206_16F18346.Xc:\Users\WangBingBin\AppData\Local\WangBing\Projects\MCU_MindMotion\Project\Meter_Keil\Meter_Keil\Application\Common\SIMEEP.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef __SMP_H
#define __SMP_H

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

extern void SMP_vWriteWord(uint8_t * pu8Buffer);	

extern void SMP_vReadWord(uint8_t * pu8Buffer);	

extern void SMP_vWriteWordN(uint8_t * pu8Buffer, uint8_t u8Length);	

extern void SMP_vReadWordN(uint8_t * pu8Buffer, uint8_t u8Length);		
#endif