#include "mecanum_chassis.h"
#include <math.h>
#include <stdio.h>
#include <stdbool.h>

#define PI 3.141592654f

static inline float linear_speed_to_rps(MecanumChassisTypeDef *self,  float speed)
{
    return speed / (PI * self->wheel_diameter) * self->correction_factor;
}

void mecanum_chassis_set_velocity(MecanumChassisTypeDef *self, float speed, float direction, float angular_rate)
{
    /**
          Use polar coordinates to control moving
                      x
          v1 motor3|  ↑  |motor1 v2
            +  y - |     |
          v4 motor4|     |motor2 v3
          :param speed: mm/s
          :param direction: Moving direction 0~2pi, 1/2pi<--- ↑ ---> 3/2pi
          :param angular_rate:  The speed at which the chassis rotates rad/sec
          :param fake:
          :return:
          """
    */
    float vx = speed * sinf(direction);
    float vy = speed * cosf(direction);
    float vp = angular_rate * (self->wheelbase + self->shaft_length);
    float v1 = vy - vx - vp;
    float v2 = vy + vx + vp;
    float v3 = vy - vx + vp;
    float v4 = vy + vx - vp;
    v1 = linear_speed_to_rps(self, v1);
    v4 = linear_speed_to_rps(self, v4);
    v2 = linear_speed_to_rps(self, v2);
    v3 = linear_speed_to_rps(self, v3);
    self->set_motors(self, v1, v4, v2, v3);
}

void mecanum_chassis_set_xy(MecanumChassisTypeDef *self, float vx, float vy, float angular_rate)
{
    /**
          Use polar coordinates to control moving
                      x
          v1 motor3|  ↑  |motor1 v2
            +  y - |     |
          v4 motor4|     |motor2 v3
          :param speed: mm/s
          :param direction: Moving direction 0~2pi, 1/2pi<--- ↑ ---> 3/2pi
          :param angular_rate:  The speed at which the chassis rotates rad/sec
          :param fake:
          :return:
          """
    */
    float vp = angular_rate * (self->wheelbase + self->shaft_length);
    float v1 = vx - vy - vp;
    float v2 = vx + vy + vp;
    float v3 = vx - vy + vp;
    float v4 = vx + vy - vp;
    v1 = linear_speed_to_rps(self, v1);
    v4 = linear_speed_to_rps(self, v4);
    v2 = linear_speed_to_rps(self, v2);
    v3 = linear_speed_to_rps(self, v3);
    self->set_motors(self, v1, v4, v2, v3);
}
static void stop(void *self)
{
    ((MecanumChassisTypeDef*)self)->set_motors(self, 0, 0, 0, 0);
}

static void set_velocity(void *self, float vx, float vy, float angular_rate)
{
    mecanum_chassis_set_xy(self, vx, vy, angular_rate);
}

static void set_velocity_radius(void *self, float linear, float r, bool insitu)
{
	if(insitu) {
		float angular_rate = linear / r;
		mecanum_chassis_set_xy(self, 0, 0, angular_rate);
	}else{
		float angular_rate = linear / r;
		mecanum_chassis_set_xy(self, linear, 0, angular_rate);
	}
}


void mecanum_chassis_object_init(MecanumChassisTypeDef *self)
{
    self->base.stop = stop;
    self->base.set_velocity = set_velocity;
	self->base.set_velocity_radius = set_velocity_radius;
}
