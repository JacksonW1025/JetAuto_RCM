/**
 * @file pid.h
 * @author Lu Yongping (Lucas@hiwonder.com)
 * @brief PID相关数据结构及方法声明
 * @version 0.1
 * @date 2023-07-12
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef _PID_H
#define _PID_H

#include <stdint.h>

/**
  * @brief PID 控制器结构体
  *
  */
typedef struct {
	float set_point; /**< @brief 目标值 */
	float kp;        /**< @brief 比例增益 */
	float ki;        /**< @brief 积分增益 */
	float kd;        /**< @brief 微分增益 */
	
	float previous_0_err; /**< @brief 上次误差 */
	float previous_1_err; /**< @brief 上上次误差 */
	
	float output; /**< @brief PID输出 */
}PID_ControllerTypeDef;



/**
 * @brief PID控制更新
 * @param self PID控制器对象指针
 * @param actual 当前的实际值
 * @param 距离上次更新的时间间隔
 * @retval None.
 */
void pid_controller_update(PID_ControllerTypeDef *self, float actual, float time_delta);



/**
 * @brief 初始化PID控制器
 * @param self 要初始化的PID控制器指针
 * @param kp 比例增益
 * @param ki 积分增益
 * @param kd 微分增益
 * @retval None.
 */
void pid_controller_init(PID_ControllerTypeDef *self, float kp, float ki, float kd);

#endif
