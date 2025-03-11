/**
 * @file button.h
 * @author Lu Yongping (Lucas@hiwonder.com)
 * @brief 硬件无关的按键检测的相关程序和数结构什么
 * @version 0.1
 * @date 2023-05-08
 *
 * @copyright Copyright (c) 2023
 *
 */
#ifndef __BUTTON_H_
#define __BUTTON_H_

#include <stdio.h>
#include <stdint.h>

typedef enum {
    BUTTON_STAGE_NORMAL,
    BUTTON_STAGE_PRESS,
    BUTTON_STAGE_LONGPRESS,
} ButtonStageEnum;

/**
  *@defgroup Button Button
  * @{
*/

/**
  * @brief 按钮事件枚举
  *
*/
typedef enum {
    BUTTON_EVENT_PRESSED = 0x01,           /**< @brief 按钮被按下 */
    BUTTON_EVENT_LONGPRESS = 0x02,         /**< @brief 按钮被长按 */
    BUTTON_EVENT_LONGPRESS_REPEAT = 0x04,  /**< @brief 按钮长按重触发 */
    BUTTON_EVENT_RELEASE_FROM_LP = 0x08,   /**< @brief 按钮从长按中松开 */
    BUTTON_EVENT_RELEASE_FROM_SP = 0x10,   /**< @brief 按钮从短按中松开 */
    BUTTON_EVENT_CLICK = 0x20,             /**< @brief 按钮被点击 */
    BUTTON_EVENT_DOUBLE_CLICK = 0x40,      /**< @brief 按钮被双击 */
    BUTTON_EVENT_TRIPLE_CLICK = 0x80,      /**< @brief 按钮被三连击 */
} ButtonEventIDEnum;



typedef struct ButtonObject ButtonObjectTypeDef;

/**
  * @brief 按钮事件对象结构体
  *
*/
typedef struct {
    ButtonObjectTypeDef *button; /**< @brief 触发事件的按钮对象指针 */
    ButtonEventIDEnum event; /**< @brief 按钮事件ID */
} ButtonEventObjectTypeDef;

/**
  * @brief 按钮事件回调函数类型
  *
*/
typedef void (*ButtonEventCallbackFuncTypeDef)(ButtonObjectTypeDef *self,  ButtonEventIDEnum event);

/**
  * @brief 按钮对象结构体
  *
*/
struct ButtonObject {
	uint32_t id;

    ButtonStageEnum stage;  /**< @brief 按键扫描状态机当前状态 */
    uint32_t last_pin_raw; /**< @brief 上次读到的原始IO口状态 */
	uint32_t last_pin_filtered; /**< @brief 上次经过消抖的IO口状态 */
    uint32_t combin_counter; /**< @brief 连击计数 */
	uint32_t ticks_count; /**< @brief 毫秒计时变量 */

    /* config */
    uint32_t combin_th; /**< @brief 连按最大间隔 毫秒*/
    uint32_t lp_th;     /**< @brief 长按阈值 毫秒 */
    uint32_t repeat_th; /**< @brief 长按重触发间隔 毫秒 */
    ButtonEventCallbackFuncTypeDef event_callback; /**< @brief 事件回调函数指针 */

    /* 移植接口 */
    /**
      * @brief 读取按钮IO状态
      * @attention 用户实现此接口时其返回值必须为按下时为1,松开时为0
      * @retval 0 按钮未被按下
      * @retval 1 按钮已被按下
    */
    uint32_t (*read_pin)(ButtonObjectTypeDef *self);

};


/**
  * @brief 以默认数据初始化一个 ButtonObjectTypeDef 实例的内存空间
  * @param self 要初始化的对象指针
  * @retval None.
*/
void button_object_init(ButtonObjectTypeDef *self);

/**
  * @brief 按钮任务处理函数
  * @attention 用户必须定时调用此函数以扫描
  * @param self 按钮对象指针
  * @param period 本次调用距离上次调用间隔的毫秒数
  * @retval None.
*/
void button_task_handler(ButtonObjectTypeDef *self, uint32_t period);

/**
  * @brief 按钮事件回调注册
  * @param self 按钮对象指针
  * @param callback 回调函数指针
  * @retval None.
*/
void button_register_callback(ButtonObjectTypeDef *self, ButtonEventCallbackFuncTypeDef callback);


/**
  * @brief 按钮的默认事件回调
  * @param self 触发回调的按钮对象
  * @param event 触发此次回调的事件
  */
void button_defalut_event_callback(ButtonObjectTypeDef *self,  ButtonEventIDEnum event);

/** @} */
#endif

