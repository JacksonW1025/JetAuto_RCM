#ifndef __PACKET_HANDLE
#define __PACKET_HANDLE

#include "packet.h"
#include "global.h"
#include "led.h"
#include "buzzer.h"


/**
* @brief 串口命令回调处理
* @param frame 数据帧
* @retval void
*/
void packet_led_handle(struct PacketRawFrame *frame);

/**
* @brief 串口命令回调处理
* @param frame 数据帧
* @retval void
*/
void packet_buzzer_handle(struct PacketRawFrame *frame);

/**
* @brief 串口命令回调处理
* @param frame 数据帧
* @retval void
*/

void packet_motor_handle(struct PacketRawFrame *frame);


void packet_handle_init(void) ;

#endif