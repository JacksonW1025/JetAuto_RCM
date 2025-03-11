/**
 * @file checksum.h
 * @author Lu Yongping (Lucas@hiwonder.com)
 * @brief 几种校验算法函数声明
 * @version 0.1
 * @date 2023-05-20
 *
 * @copyright Copyright (c) 2023
 *
 */
 
#ifndef __CHECKSUM_H__
#define __CHECKSUM_H__

#include <stdio.h>
#include <stdint.h>

/**
*@defgroup Checksum Checksum
* @{
*/

/**
* @brief 累加和校验
* @param buf 要进行计算的数据
* @param len 要进行计算的数据的字节数
* @return 累加和结果, 只有低八位有效
*/
uint16_t checksum_sum(const uint8_t *buf, uint16_t len);

/**
 * @brief 异或校验 
 * @details 计算所有字节的异或结果
 * @param buf 要进行计算的数据
 * @param len 要进行计算的数据的字节数
 * @return 异或校验结果, 只有低八位有效
 */
uint16_t checksum_xor(const uint8_t *buf, uint16_t len);  

/**
 * @brief CRC8校验
 * @details 对输入数据进行CRC8校验并返回校验结果
 * @param buf 要进行计算的数据
 * @param len 要进行计算的数据的字节数
 * @return CRC8校验结果, 只有低8位有效
 */
uint16_t checksum_crc8(const uint8_t *buf, uint16_t len);  /* CRC8 校验 */


/**
 * @brief CRC16校验
 * @details 对输入数据进行CRC16校验并返回校验结果
 * @param buf 要进行计算的数据
 * @param len  要进行计算的数据的字节数
 * @return CRC16校验结果 
 */
uint16_t checksum_crc16(const uint8_t *buf, uint16_t len); /* CRC16 校验 */

/**
 * @} 
 */

#endif /* __CHECK_H__ */
