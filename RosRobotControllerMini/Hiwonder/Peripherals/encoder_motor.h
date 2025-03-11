/**
 * @file encoder_motor.h
 * @author Lu Yongping (Lucas@hiwonder.com)
 * @brief 编码器电机控制头文件
 * @version 0.1
 * @date 2023-05-12
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef __ENCODER_MOTOR_H_
#define __ENCODER_MOTOR_H_

#include <stdint.h>
#include "pid.h"

typedef struct EncoderMotorObject EncoderMotorObjectTypeDef;

/**
 * @brief 编码器电机对象结构体
*/
struct EncoderMotorObject{
    int64_t counter;        /**< @brief 总计数值, 64bit 认为不会溢出 */
    int64_t overflow_num;   /**< @brief 溢出计数 */
    int32_t ticks_overflow; /**< @brief 计数溢出值 */
    float tps;              /**< @brief ticks per second 计数器频率 */
    float rps;              /**< @brief revolutions per second 输出轴转速 转每秒 */
    int current_pulse;      /**< @brief 当前输出的PWM值, 有符号对应正反转 */
    PID_ControllerTypeDef pid_controller; /**< @brief PID 控制器 */

    /** porting 可移植硬件接口 **/
    int32_t ticks_per_circle; /**< @brief 电机输出轴旋转一圈产生的计数个数, 根据电机实际情况填写 */
	float rps_limit;  /**< @brief 电机地转速极限，一般会取比100% PWM占空比时地转速稍小的值以确保PID控制器对速度的控制 */
    /**
      * @brief 设置电机速度  -1000 ~ 1000
      * @param self 编码器电机对象
      * @param pulse 新的PWM值, -1000~1000, 对应正反转0~100%转速
      * @retval None.
      */
    void (*set_pulse)(EncoderMotorObjectTypeDef *self, int pulse);
};


/**
 * @breif 编码器电机对象初始化
 * @param self 编码器电机对象指针
 * @retval None.
*/
void encoder_motor_object_init(EncoderMotorObjectTypeDef *self);


/**
 * @brief 编码器电机速度测量更新
 * @detials
 * @param self 编码器电机对象
 * @param period 本次调用与上次调用的时间间隔，单位为秒
 * @param counter 编码器当前计数值
 * @retval None.
*/
void encoder_update(EncoderMotorObjectTypeDef *self, float period, int64_t new_counter);


/**
 * @brief 编码器电机速度控制任务
 * @detials 编码器电机速度PID控制任务,需要定时指定以完成PID控制更新
 * @param self 编码器电机对象
 * @param period 当前更新距离上次更新的时间间隔(更新周期), 单位 sec
 * @retval None.
*/
void encoder_motor_control(EncoderMotorObjectTypeDef *self, float period);


/**
 * @brief 编码器电机设置PID控制目标速度
 * @param rps 目标速度， 单位转每秒
 * @retval None.
*/
void encoder_motor_set_speed(EncoderMotorObjectTypeDef *self, float rps);

#endif

