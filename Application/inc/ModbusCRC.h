/*
 * @Author: Wang Bing
 * @Date: 2021-09-06 12:44:09
 * @LastEditTime: 2023-02-08 09:34:47
 * @LastEditors: Wangbingbin bing.wang2@bshg.com
 * @Description: CRC16 check
 * @FilePath: \PUc:\Users\WangBingBin\AppData\Local\WangBing\Projects\EXTERNAL\Projects\Temperature_16F18346.X\Temperature_16F18346.X\Application\PhisicalDriver\ModbusCRC.h
 */

#ifndef _MODBUS_CRC_H_
#define _MODBUS_CRC_H_

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

/**
 * @description: crc16 checking
 * @param {*}
 * @return {*}
 */
 uint16_t crc16( uint8_t *puchMsg, uint16_t usDataLen );

#endif

