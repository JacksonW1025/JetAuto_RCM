#include <stdio.h>
#include "global_conf.h"
#include "differential_chassis.h"

#define PI 3.141592654f
static inline float linear_speed_to_rps(DifferentialChassisTypeDef *self,  float speed)
{
    return speed / (PI * self->wheel_diameter) * self->correction_factor;
}

/*vx mm/s
  angule_rate rad/s
*/
void diff_chassis_move(DifferentialChassisTypeDef *self, float vx, float angule_rate)
{
    float v_l = vx - angule_rate * ((DifferentialChassisTypeDef*)self)->shaft_length / 2.0f;
    float v_r = vx + angule_rate * ((DifferentialChassisTypeDef*)self)->shaft_length / 2.0f;
    float rps_l = linear_speed_to_rps(self, v_l);
    float rps_r = linear_speed_to_rps(self, v_r);
    self->set_motors(self, rps_l, rps_r);
}

static void stop(void *self)
{
    ((DifferentialChassisTypeDef*)self)->set_motors(self, 0, 0);
}

static void set_velocity(void *self, float vx, float vy, float angular_rate)
{
    diff_chassis_move(self, vx, angular_rate);
}

static void set_velocity_radius(void* self, float linear, float r,  bool insitu)
{
    if(insitu) {
		float angule_rate = linear / r;
        diff_chassis_move(self, 0, angule_rate);
    } else {
        float angule_rate = linear / r;
        diff_chassis_move(self, linear, angule_rate);
    }
}

void diff_chassis_object_init(DifferentialChassisTypeDef *self)
{
    self->base.stop = stop;
    self->base.set_velocity = set_velocity;
    self->base.set_velocity_radius = set_velocity_radius;
}

