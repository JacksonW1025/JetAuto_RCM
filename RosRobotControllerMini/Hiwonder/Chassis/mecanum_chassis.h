#ifndef __MECANUM_CHASSIS_H
#define __MECANUM_CHASSIS_H
#include "chassis.h"

typedef struct {
	ChassisTypeDef base;
	float wheelbase;
	float shaft_length;
	float wheel_diameter;
	float correction_factor;
	void (*set_motors)(void *self, float l_head, float l_tail, float r_head, float r_tail);
}MecanumChassisTypeDef;

void mecanum_chassis_object_init(MecanumChassisTypeDef *self);


#endif

