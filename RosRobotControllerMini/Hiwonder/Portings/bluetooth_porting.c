/**
 * @file bluetooth_porting.c
 * @author Lu Yongping (Lucas@hiwonder.com)
 * @brief 蓝牙串口协议收发接口实现
 * @version 0.1
 * @date 2023-05-23
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "global.h"
#include "lwrb.h"
#include "usart.h"
#include "packet.h"
#include "cmsis_os2.h"
#include <stdio.h>
#include "lwmem_porting.h"

#if ENABLE_BLUETOOTH

#define BLUETOOTH_DMA_BUFFER_SIZE 16   /* 单个DMA缓存长度 */

extern osMessageQueueId_t moving_ctrl_queueHandle;  /* 运动控制的控制队列 */
extern osMessageQueueId_t bluetooth_tx_queueHandle; /* 蓝牙数据发送队列 */
extern osSemaphoreId_t bluetooth_tx_idleHandle;     /* 蓝牙发送空闲信号量 */

static uint8_t rx_dma_buffers[2][BLUETOOTH_DMA_BUFFER_SIZE];  /* 蓝牙接收缓存 */
static uint32_t buffer_index = 0;  /* 蓝牙接收缓存索引，用于控制使用哪个缓存进行DMA接收 */

static void bluetooth_dma_rx_event_callback(UART_HandleTypeDef *huart, uint16_t length); /* 蓝牙接收事件回调函数 */
static void bluetooth_dma_tx_complete_callback(UART_HandleTypeDef *huart);  /* 蓝牙发送完成回调函数 */
	

/**
 * @brief  蓝牙任务入口
 * @param argument 保留
 * @retval void
 */
void bluetooth_task_entry(void *argument)
{
	/* 接收处理依靠接收中断触发，不使用任务
	   使用 ReceiveToIdle_DMA 进行接收， 该函数会在DMA缓存满时中断或在接收空闲时中断并触发接收事件回调 
	   接收数据的处理过程在回调中完成接收事件回调中完成
	*/
    HAL_UART_RegisterRxEventCallback(&huart2, bluetooth_dma_rx_event_callback); /* 注册接收事件回调 */
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_dma_buffers[buffer_index], BLUETOOTH_DMA_BUFFER_SIZE); /* 开始接收 */
	
	/* 发送处理，通过队列触发发送 */
	uint8_t tx_buffer[8];
    for(;;) {
	    osSemaphoreAcquire(bluetooth_tx_idleHandle, osWaitForever); /* 等待发送空闲信号 */
        osStatus_t status = osMessageQueueGet(bluetooth_tx_queueHandle, tx_buffer, NULL, osWaitForever); /* 从发送队列中取出数据 */
        if(osOK == status) {
            HAL_UART_RegisterCallback(&huart2, HAL_UART_TX_COMPLETE_CB_ID, bluetooth_dma_tx_complete_callback); /* 注册 DMA 接收完成回调 */
			/* 消息的第一个字节作为要发送的数据的长度, 第二字节开始为要发送的数据 */
            HAL_UART_Transmit_DMA(&huart2, &tx_buffer[1], tx_buffer[0] > 7 ? 7 : tx_buffer[0]);  /* 触发 DMA 发送*/
        }
    }	
}

/**
 * @brief  蓝牙接收事件回调
 * @detials 将接收到的数据逐个字节作为消息压入到消息队列等待主逻辑处理
 * @param huart 串口实例
 * @param length 接收到的字节数
 * @retval None.
 */
static void bluetooth_dma_rx_event_callback(UART_HandleTypeDef *huart, uint16_t length)
{
    uint32_t cur_index = buffer_index; /* 取得当前DMA缓存的索引号 */
    if(length < BLUETOOTH_DMA_BUFFER_SIZE) {
        HAL_UART_AbortReceive(&huart2);
    }
    buffer_index ^= 1;
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_dma_buffers[buffer_index], BLUETOOTH_DMA_BUFFER_SIZE);
	for(int i = 0; i < length; ++i) {
		osMessageQueuePut(moving_ctrl_queueHandle, &rx_dma_buffers[cur_index][i], 0, 0);
	}
}

/**
 * @brief 蓝牙 DMA 发送完成回调
 * @param huart 串口实例
 * @retval None.
 */
static void bluetooth_dma_tx_complete_callback(UART_HandleTypeDef *huart)
{
    osSemaphoreRelease(bluetooth_tx_idleHandle);
}

#endif
