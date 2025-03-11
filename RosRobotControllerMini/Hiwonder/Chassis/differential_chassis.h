#ifndef __DIFFERENTIAL_CHASSIS_H
#define __DIFFERENTIAL_CHASSIS_H
#include "chassis.h"


typedef struct {
	ChassisTypeDef base;
	float wheelbase;
	float shaft_length;
	float wheel_diameter;
	float correction_factor;
	void (*set_motors)(void *self, float rps_l, float rps_r);
}DifferentialChassisTypeDef;
void diff_chassis_object_init(DifferentialChassisTypeDef *self);

#endif

