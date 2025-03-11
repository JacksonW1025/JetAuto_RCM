/**
 * @file lwmem_porting.c
 * @author Lu Yongping (Lucas@hiwonder.com)
 * @brief SBUS 协议接收的硬件相关处理
 * @version 0.1
 * @date 2023-05-18
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <stdio.h>
#include "SBus.h"
#include "lwmem_porting.h"
#include "lwrb.h"
#include "usart.h"
#include "global.h"
#include "gui_guider.h"
#include "packet_reports.h"

#define SBUS_RX_DMA_BUFFER_SIZE 32 /* SBUS DMA 接收缓存长度 */
#define SBUS_RX_FIFO_BUFFER_SIZE 512 /* SBUS FIFO 缓存长度 */

/* SBUS1 相关变量 */
static uint8_t *subs1_rx_dma_buffers[2];
static uint32_t sbus1_rx_dma_buffer_index;
static uint8_t *sbus1_rx_fifo_buffer;
static lwrb_t *sbus1_rx_fifo;
static void sbus_dma_receive_event_callback(UART_HandleTypeDef *huart, uint16_t length);

/* SBUS 对外暴露变量 */
SBusStatusObjectTypeDef *sbus1_status;

/* SBUS1 需要的外部变量 */
extern osSemaphoreId_t sbus_data_readyHandle; /* subs数据就绪信号量Handle */
extern osSemaphoreId_t sbus_data_ready_01_Handle;
extern osEventFlagsId_t sbus_data_ready_event_Handle;


/**
 * @brief  SBUS初始化
 *  初始化 SBUS 相关内存、变量并开始SBUS 接收
 * @retval None.
 */
void sbus_init(void)
{
    sbus1_status = LWMEM_CCM_MALLOC(sizeof(SBusStatusObjectTypeDef));
    sbus1_status->type_id = OBJECT_TYPE_ID_GAMEPAD_STATUS;

    /* DMA 接收缓存初始化 */
    subs1_rx_dma_buffers[0] = LWMEM_RAM_MALLOC(SBUS_RX_DMA_BUFFER_SIZE); /* DMA 缓存不能放在 CCMRAM 上 */
    subs1_rx_dma_buffers[1] = LWMEM_RAM_MALLOC(SBUS_RX_DMA_BUFFER_SIZE);
    sbus1_rx_dma_buffer_index = 0;

    /* 接收 FIFO 初始化 */
    sbus1_rx_fifo_buffer  = LWMEM_CCM_MALLOC(SBUS_RX_FIFO_BUFFER_SIZE);
    sbus1_rx_fifo = LWMEM_CCM_MALLOC(sizeof(lwrb_t));
    lwrb_init(sbus1_rx_fifo, sbus1_rx_fifo_buffer, SBUS_RX_FIFO_BUFFER_SIZE);

}


/**
 * @brief  SBUS接收事件回调
 *  在 DMA 接收缓存满后或者空闲时触发本函数将接收到的数据压入接收FIFO缓存，再由接收任务完成解析和处理
 *  因为SBUS协议的发送最小间隔为4ms, 而DMA缓存大小设为了32, 所以总是触发空闲事件而不触发DMA满事件
 *  @attention 需要注意, 不是所有遥控接收器都会一次性完成一帧的发送，所以一帧数据可能会触发多次空闲事件, 不能直接使用空闲事件进行帧分割。
 *             因为接收开始位置不一定是帧头位置，所以也不能直接使用DMA接收长度进行分割。
 *             如果一定要使用DMA长度/空闲事件进行帧分割就必须先完成一次帧同步，即接收到一个完整、正确的帧再开始DMA接收确保DMA接收的首字节为帧头。
 *             在本实现中，将所有接收到的数据汇聚为字节流，再通过帧头、帧尾的固定值进行分割。
 * @param huart 串口实例
 * @param Pos 接收到的字节数
 * @retval None.
 */
static void sbus_dma_receive_event_callback(UART_HandleTypeDef *huart, uint16_t length)
{
    uint32_t cur_index = sbus1_rx_dma_buffer_index; /* 取得当前DMA缓存的索引号 */
    sbus1_rx_dma_buffer_index ^= 1;
    if(length < SBUS_RX_DMA_BUFFER_SIZE) {
        HAL_UART_AbortReceive(&huart5);
    }
    HAL_UARTEx_ReceiveToIdle_DMA(&huart5, (uint8_t*)subs1_rx_dma_buffers[sbus1_rx_dma_buffer_index], SBUS_RX_DMA_BUFFER_SIZE);
    lwrb_write(sbus1_rx_fifo, subs1_rx_dma_buffers[cur_index], length); /* 将接收到的数据写入fifo ring */
    if(lwrb_get_full(sbus1_rx_fifo) >= 25) { /* SBUS 每帧数据大小为25个字节， 每一次超过25个字节就进行一次解析 */
        osSemaphoreRelease(sbus_data_ready_01_Handle); /* 置位 SBUS 数据就绪信号量 */
    }
}


/**
 * @brief  SBUS接收任务入口函数
 * @param argument 入口参数
 * @retval None.
 */
#if ENABLE_SBUS
void sbus_rx_task_entry(void *argument)
{
    extern osMessageQueueId_t lvgl_event_queueHandle;
    extern SBusStatusObjectTypeDef sbus_status_disp;

    static uint8_t buf_temp[32];

    sbus_init();

    /* 开始接收 */
start_sbus_receive:
    HAL_UART_AbortReceive(&huart5);
    HAL_UART_RegisterRxEventCallback(&huart5, sbus_dma_receive_event_callback); /* 注册接收事件回调 */
    /* 使用 ReceiveToIdle_DMA 进行接收， 该函数会在DMA缓存满时中断或在接收空闲时中断并触发接收事件回调 */
    HAL_UARTEx_ReceiveToIdle_DMA(&huart5, (uint8_t*)subs1_rx_dma_buffers[sbus1_rx_dma_buffer_index], SBUS_RX_DMA_BUFFER_SIZE);
    /* 解析循环 */
    for(;;) {
        if(osSemaphoreAcquire(sbus_data_ready_01_Handle, 500) != osOK) { /* 等待数据就绪 */
            goto start_sbus_receive;
        }
        for(;;) {
            memset(buf_temp, 0, 25);
            if(lwrb_peek(sbus1_rx_fifo, 0, buf_temp, 25) == 25) {  /* 从缓存中取出25个字节 */
                if(sbus_decode_frame(buf_temp, sbus1_status) == 0) { /* 尝试解析直到字节不够 */
                    lwrb_skip(sbus1_rx_fifo, 25);
                    PacketReportSBusTypeDef report;
                    for(int i = 0; i < 16; ++i) {
                        report.channels[i] = sbus1_status->channels[i];
                    }
                    report.ch17 = sbus1_status->ch17;
                    report.ch18 = sbus1_status->ch18;
                    report.signal_loss = sbus1_status->signal_loss;
                    report.fail_safe = sbus1_status->fail_safe;
                    packet_transmit(&packet_controller, PACKET_FUNC_SBUS, &report, sizeof(PacketReportSBusTypeDef));
                } else {
                    lwrb_skip(sbus1_rx_fifo, 1);
                }
            } else {
                break;
            }
        }
    }
}
#endif

