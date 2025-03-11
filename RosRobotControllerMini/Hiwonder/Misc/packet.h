/**
* @file protocol.h
* @brief 与上位机的通信协议处理
* @author LuYongPing
* @date 20 MAY 2023
*/

#ifndef __PROTOCOL_H_
#define __PROTOCOL_H_

#include <stdint.h>
#include <cmsis_os2.h>
#include "lwrb.h"
#include "lwmem.h"

#define PROTO_CONST_STARTBYTE1 0xAAu
#define PROTO_CONST_STARTBYTE2 0x55u
#define PACKET_PARSE_BUFFER_SIZE 64

#pragma pack(1)
struct PacketRawFrame {
    uint8_t start_byte1;
    uint8_t start_byte2;
    uint8_t function;
    uint8_t data_length;
    uint8_t data_and_checksum[257];
};
#pragma pack()

/**
 * @brief 解析器状态机状态枚举
 *
 */
enum PacketControllerState {
    PACKET_CONTROLLER_STATE_STARTBYTE1, /**< 正在寻找帧头标记1 */
    PACKET_CONTROLLER_STATE_STARTBYTE2, /**< 正在寻找帧头标记2 */
    PACKET_CONTROLLER_STATE_FUNCTION, /**< 正在处理帧功能号 */
    PACKET_CONTROLLER_STATE_LENGTH, /**< 正在处理帧长度 */
    PACKET_CONTROLLER_STATE_DATA, /**< 正在处理帧数据 */
    PACKET_CONTROLLER_STATE_CHECKSUM, /** 正在处理数据校验 */
};

/**
 * @brief 帧功能号枚举
 *
 */
enum PACKET_FUNCTION {
    PACKET_FUNC_SYS = 0,
    PACKET_FUNC_LED,
    PACKET_FUNC_BUZZER,
    PACKET_FUNC_MOTOR,
    PACKET_FUNC_PWM_SERVO,
    PACKET_FUNC_BUS_SERVO,
    PACKET_FUNC_KEY,
    PACKET_FUNC_IMU,
    PACKET_FUNC_GAMEPAD,
    PACKET_FUNC_SBUS,
	PACKET_FUNC_OLED,
    PACKET_FUNC_NONE,
};

typedef void(*packet_handle)(struct PacketRawFrame *);

/**
 * @brief 协议解析器
 * @details 协议解析器, 存储了解析器的工作状态、状态机状态等
 */
struct PacketController {
    enum PacketControllerState state;        /**< 解析器状态机当前状态 */
    struct PacketRawFrame frame;             /**< 解析器正在处理的帧 */
    packet_handle handles[PACKET_FUNC_NONE]; /**< 解析操作列表 */
    int data_index;

    uint8_t *rx_dma_buffers[2]; /**< DMA缓存列表 */
    size_t rx_dma_buffer_size; /**< 单个DMA缓存的大小 */
    volatile int rx_dma_buffer_index;    /**< 当前正在接收的DMA缓存索引号 */

    uint8_t *rx_fifo_buffer;    /**< 接收FIFO缓存 */
    lwrb_t *rx_fifo;            /**< 接收缓存FIFO对象 */

    int (*send_packet)(struct PacketController *self, struct PacketRawFrame *frame);
    struct PacketRawFrame* tx_dma_buffer; /**< 正在发送的DMA缓存*/
};


/**
 * @brief 串口命令回调注册
 * @param self 协议实例
 * @param func 功能ID
 * @param handle 回调函数
 * @retval None
*/
void packet_register_callback(struct PacketController *self, enum PACKET_FUNCTION func, packet_handle p);


/**
 * @brief 串口协议接收处理
 * @details 传入数据，根据状态机状态和数据切换状态机状态，完成协议的接收解析
 *          解析出的数据帧会压入接收帧队列
 * @param self 协议实例
 * @retval None
 */
void packet_recv(struct PacketController *self);

/**
 * @brief  串口协议发送处理
 *
 * @param self  协议实例
 * @param func  功能号
 * @param data  数据段
 * @param data_len  数据段长度
 * @retval  ==0 成功
 * @retval  !=0 失败
 */
int packet_transmit(struct PacketController *self, uint8_t func, void* data, size_t data_len);

#endif
