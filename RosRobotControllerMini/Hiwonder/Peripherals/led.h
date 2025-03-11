/**
 * @file led.h
 * @author Lu Yongping (Lucas@hiwonder.com)
 * @brief 硬件无关的LED灯闪烁控制的函数即数据结构声明
 * @version 0.1
 * @date 2023-05-13
 *
 * @copyright Copyright (c) 2023
 *
 */


#ifndef __LED_H_
#define __LED_H_

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

typedef enum {
    LED_STAGE_START_NEW_CYCLE,
    LED_STAGE_WATTING_OFF,
    LED_STAGE_WATTING_PERIOD_END,
    LED_STAGE_IDLE,
} LEDStageEnum;



/**
  *@defgroup LED LED
  * @{
*/


/**
  * @brief LED控制的参数结构体
  *
*/
typedef struct {
    uint32_t ticks_on;  /**< @brief 周期内LED亮起时长，毫秒 */
    uint32_t ticks_off; /**< @brief 周期内LED熄灭时长，毫秒 */
    uint16_t repeat;    /**< @brief LED闪烁重复次数 */
} LEDCtrlTypeDef;


/**
  * @brief LED灯对象结构体
  *
*/
typedef struct LEDObjectObject LEDObjectTypeDef;
struct LEDObjectObject {
	uint32_t id;
	
    LEDStageEnum stage;   /**< @brief LED控制状态机的当前状态 */
    uint32_t ticks_count;    /**< @brief 毫秒计数，用于累计毫秒数切换状态机状态 */
    LEDCtrlTypeDef ctrl_structure; /**< @brief LED控制参数 */

    // 移植接口
    /**
      * @brief 从队列中取出控制块
      * @attention 此函数必须为非阻塞的
      * @param [out] p 存放取出数据的内存指针
      * @retval 0  成功
      * @retval !=0 失败
    */
    int (*get_ctrl_block)(LEDObjectTypeDef *self, LEDCtrlTypeDef *p);
    /**
      * @brief 向队列压入一个控制块
      * @param p 要压入的控制块的指针
      * @retval 0  成功
      * @retval !=0 失败
    */
    int (*put_ctrl_block)(LEDObjectTypeDef *self, LEDCtrlTypeDef *p);

    /**
      * @brief 设置 LED 灯亮灭
      * @param new_state LED 灯新的状态 @attention 无论实际电路是高电平亮起还是低电平亮起，此函数都必须实现写1时亮起，写0时熄灭
      * @retval None.
     */
    void (*set_pin)(LEDObjectTypeDef *self, uint32_t new_state);
};



/**
  * @brief 以默认数据初始化一个 LEDObjectTypeDef 实例的内存空间
  * @param self 要初始化的对象指针
  * @retval None.
*/
void led_object_init(LEDObjectTypeDef *self);

/**
  * @brief LED 任务处理函数
  * @attention 用户必须定时调用此函数以处理 LED 的操作
  * @param self LED 对象指针
  * @param period 本次调用距离上次调用间隔的毫秒数
  * @retval None.
*/
void led_task_handler(LEDObjectTypeDef *self, uint32_t period);

/**
  * @brief 使LED长亮
  * @param self LED 对象指针
  * @retval 0 成功
  * @retval !=0 失败
*/
int led_on(LEDObjectTypeDef *self);

/**
  * @brief 使LED熄灭
  * @param self LED 对象指针
  * @retval 0 成功
  * @retval !=0 失败
*/
int led_off(LEDObjectTypeDef *self);


/**
  * @brief 使LED以固定周期和占空比闪烁
  * @param self LED 对象指针
  * @param ticks_on 周期内 LED 亮起起时长，毫秒
  * @param ticks_off 周期内 LED 熄灭时长，毫秒
  * @param repeat 闪烁重复次数， 当给定的重复次数为0时会一直闪烁
  * @retval 0 成功
  * @retval !=0 失败
 */
int led_flash(LEDObjectTypeDef *self, uint32_t ticks_on, uint32_t ticks_off, uint32_t repeat);

/** @} */
#endif

