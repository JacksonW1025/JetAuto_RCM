/**
 * @file pid.c
 * @author Lu Yongping (Lucas@hiwonder.com)
 * @brief PID实现
 * @version 0.1
 * @date 2023-07-12
 *
 * @copyright Copyright (c) 2023
 *
 */


#include "pid.h"

void pid_controller_update(PID_ControllerTypeDef *self, float actual, float time_delta) {
	float err = self->set_point - actual;
	float proportion = err - self->previous_0_err;
	
	float integral = err * time_delta;
	float derivative = (err - 2 * self->previous_1_err + self->previous_0_err) / time_delta;
	
	self->output = (self->kp * err) + (self->ki * integral) + (self->kd * derivative);
	self->previous_1_err = self->previous_0_err;
	self->previous_0_err = err;

}


void pid_controller_init(PID_ControllerTypeDef *self, float kp, float ki, float kd) {
	self->set_point = 0;
	self->kp = kp;
	self->ki = ki;
	self->kd = kd;
	self->previous_0_err = 0;
	self->previous_1_err = 0;
	self->output = 0;
}

