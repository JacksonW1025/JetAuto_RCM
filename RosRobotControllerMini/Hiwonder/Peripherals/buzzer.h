/**
 * @file buzzer.h
 * @author Lu Yongping (Lucas@hiwonder.com)
 * @brief 实现硬件&系统无关的蜂鸣器控制的函数、数据结构的声明
 * @version 0.1
 * @date 2023-05-18
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef __BUZZER_H_
#define __BUZZER_H_

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

typedef enum {
    BUZZER_STAGE_START_NEW_CYCLE,
    BUZZER_STAGE_WATTING_OFF,
    BUZZER_STAGE_WATTING_PERIOD_END,
    BUZZER_STAGE_IDLE,
} BuzzerStageEnum;


/**
  *@defgroup Buzzer Buzzer
  * @{
*/


/**
  * @brief 蜂鸣器控制参数结构体
  *
*/
typedef struct {
    uint16_t freq;      /**< @brief 蜂鸣器频率 */
    uint32_t ticks_on;  /**< @brief 周期内蜂鸣器响起时长，毫秒 */
    uint32_t ticks_off; /**< @brief 周期内蜂鸣器静音时长，毫秒 */
    uint16_t repeat;    /**< @brief 蜂鸣器"嘀"响重复次数 */
} BuzzerCtrlTypeDef;


/**
  * @brief 蜂鸣器对象结构体
  *
*/
typedef struct BuzzerObject BuzzerObjectTypeDef;
struct BuzzerObject {
	uint32_t id;

    BuzzerStageEnum stage;   /**< @brief 蜂鸣器控制状态机的当前状态 */
    uint32_t ticks_count;    /**< @brief 毫秒计数，用于累计毫秒数切换状态机状态 */
    BuzzerCtrlTypeDef ctrl_structure; /**< @brief 蜂鸣器控制参数 */

    // 移植接口
    /**
      * @brief 从队列中取出控制块
      * @attention 此函数必须为非阻塞的
      * @param [out] p 存放取出数据的内存指针
      * @retval 0  成功
      * @retval !=0 失败
    */
    int (*get_ctrl_block)(BuzzerObjectTypeDef *self, BuzzerCtrlTypeDef *p);
    /**
      * @brief 向队列压入一个控制块
      * @param p 要压入的控制块的指针
      * @retval 0  成功
      * @retval !=0 失败
    */
    int (*put_ctrl_block)(BuzzerObjectTypeDef *self, BuzzerCtrlTypeDef *p);

    /**
      * @brief 设置蜂鸣器相应IO口的反正频率
      * @param freq 新的翻转频率
      * @retval None.
     */
    void (*set_pwm)(BuzzerObjectTypeDef *self, uint32_t freq);
};

/**
  * @brief 以默认数据初始化一个 BuzzerObjectTypeDef 实例的内存空间
  * @param self 要初始化的对象指针
  * @retval None.
*/
void buzzer_object_init(BuzzerObjectTypeDef *self);

/**
  * @brief Buzzer任务处理函数
  * @attention 用户必须定时调用此函数以处理蜂鸣器的操作
  * @param self Buzzer对象指针
  * @param period 本次调用距离上次调用间隔的毫秒数
  * @retval None.
*/
void buzzer_task_handler(BuzzerObjectTypeDef *self, uint32_t period);

/**
  * @brief 使蜂鸣器长鸣
  * @param self Buzzer对象指针
  * @param freq 蜂鸣器频率
  * @retval 0 成功
  * @retval !=0 失败
*/
int buzzer_on(BuzzerObjectTypeDef *self, uint32_t freq);

/**
  * @brief 使蜂鸣器静音
  * @param self Buzzer对象指针
  * @retval 0 成功
  * @retval !=0 失败
*/
int buzzer_off(BuzzerObjectTypeDef *self);

/**
  * @brief 使蜂鸣器以固定周期和占空发出嘀嘀响
  * @param self Buzzer对象指针
  * @param freq 蜂鸣器频率
  * @param ticks_on 周期内蜂鸣器响起时长，毫秒
  * @param ticks_off 周期内蜂鸣器静音时长，毫秒
  * @param repeat 嘀嘀响重复次数， 当给定的重复次数为0时会一直嘀嘀响
  * @retval 0 成功
  * @retval !=0 失败
 */
int buzzer_didi(BuzzerObjectTypeDef *self, uint32_t freq, uint32_t ticks_on, uint32_t ticks_off, uint32_t repeat);

/** @} */
#endif

