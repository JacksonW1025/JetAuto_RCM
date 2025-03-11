/**
 * @file pwm_servo.c
 * @author Lu Yongping (Lucas@hiwonder.com)
 * @brief PWM舵机驱动与硬件无关的代码
 * @version 0.1
 * @date 2023-05-07
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "pwm_servo.h"
#include <string.h>

void pwm_servo_duty_compare(PWMServoObjectTypeDef *self)   //脉宽变化比较及速度控制
{
    // 根据新设置的目标重新计算舵机控制参数
    if(self->duty_changed) {
        self->duty_changed = false;
        self->inc_times = self->duration / 20; // 计算需要递增的次数
        if(self->target_duty > self->current_duty) { /* 计算总的位置变换量 */
            self->duty_inc = (float)(-(self->target_duty - self->current_duty));
        } else {
            self->duty_inc = (float)(self->current_duty - self->target_duty);
        }
        self->duty_inc /= (float)self->inc_times; /* 计算每控制周期位置增量 */
        self->is_running = true;  // 舵机开始动作
    }
		
		// 需要控制舵机转动以到达新的位置
    if(self->is_running) {
        --self->inc_times;
        if(self->inc_times == 0) {
            self->current_duty = self->target_duty;   //最后一次递增就直接将设定值赋给当前值，保证最总位置正确
            self->is_running = false; //到达设定位置，舵机停止运动
        } else {
            self->current_duty = self->target_duty + (int)(self->duty_inc * self->inc_times);
        }
    }
    self->duty_raw = self->current_duty + self->offset; // 动作要加上舵机偏差
}

void pwm_servo_set_position (PWMServoObjectTypeDef *self, uint32_t duty, uint32_t duration)
{
    duration = duration < 20 ? 20 : (duration > 30000 ? 30000 : duration); /* 限制最短/最长运动时间 */
	duty = duty > 2500 ? 2500 : (duty < 500 ? 500 : duty); /* 限制脉宽最大/最小值 */
    self->target_duty = duty;
    self->duration = duration;
    self->duty_changed = true; /* 标记目标位置发送变换, 让 pwm_servo_duty_compare 计算新的运动参数 */
}

void pwm_servo_set_offset(PWMServoObjectTypeDef *self, int offset)
{
    offset = offset < -100 ? -100 : (offset > 100 ? 100 : offset); /* 限制最小/最大偏差, 不同舵机极限会不同， 但是100是一个不错的选择*/
    self->offset = offset;
}

void pwm_servo_object_init(PWMServoObjectTypeDef *obj)
{
    memset(obj, 0, sizeof(PWMServoObjectTypeDef));
    obj->current_duty = 1500; /* 默认位置 */
    obj->duty_raw = 1500;     /* 默认实际脉宽 */
}
