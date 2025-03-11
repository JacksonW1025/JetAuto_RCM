/**
 * @file pwm_servo.h
 * @author Lu Yongping (Lucas@hiwonder.com)
 * @brief st7735显示屏驱动
 * @version 0.1
 * @date 2023-05-05
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef __PWM_SERVO_H_
#define __PWM_SERVO_H_

#include <stdint.h>
#include <stdbool.h>

typedef struct PWMServoObject  PWMServoObjectTypeDef;

/**
 * @brief PWM舵机对象结构体
 */
struct PWMServoObject {
    int id;     /**< @brief 舵机ID */
    int offset; /**< @brief 舵机偏差 */
    int target_duty;  /**< @brief 目标脉宽 */
    int current_duty; /**< @brief 当前脉宽 */
    int duty_raw;     /**< @brief 机器脉宽，最终写入定时器的脉宽，包含了offset， == current_duty + offset */

    /* 速度控制需要的变量 */
    uint32_t duration;    /**< @brief 舵机从当前角度运动到指定角度的时间，也就是控制速度 单位:ms */
    float duty_inc; /**< @brief 每次位置更新的脉宽增量 */
    int  inc_times; /**< @brief 需要递增的次数 */
    bool is_running;  /**< @brief 舵机是否在转动过程中 */
    bool duty_changed; /**< @brief 脉宽是否被改变 */

    /* 外部提供，硬件抽象接口 */
    void (*write_pin)(uint32_t new_state); /* IO口电平设置 */
};

/**
 * @brief 舵机对象初始化
 * @param object 要进行初始化舵机对象指针
 * @retval None.
*/
void pwm_servo_object_init(PWMServoObjectTypeDef *object);

/**
 * @brief 舵机脉宽控制
 * @detials 计算脉宽变化、当前速度所需要的脉宽并实现控制，需要50ms调用一次
 * @param self 需要控制的舵机对象指针
 * @retval None.
*/
void pwm_servo_duty_compare(PWMServoObjectTypeDef *self);

/**
 * @brief 设置舵机角度
 * @detials 用于设置舵机的角度，实际只是设置舵机对象的几个内部变量，不会马上实际控制舵机，实际控制舵机是由 pwm_servo_duty_compare来计算并控制
 * @param self 需要控制的舵机对象指针
 * @param 舵机新的脉宽 500~2500间的一个整数值
 * @param None.
*/
void pwm_servo_set_position (PWMServoObjectTypeDef *self, uint32_t duty, uint32_t duration);


/**
 * @brief 设置舵机偏差
 * @param self 需要控制的舵机对象指针
 * @param 新的舵机偏差
 * @param None.
*/
void pwm_servo_set_offset(PWMServoObjectTypeDef *self, int offset);

#endif

