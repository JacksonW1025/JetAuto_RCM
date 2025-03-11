#ifndef __CHASSIS_H_
#define __CHASSIS_H_
#include <stdbool.h>

typedef enum {
	CHASSIS_TYPE_JETAUTO,
	CHASSIS_TYPE_JETTANK,
	CHASSIS_TYPE_TI4WD,
	CHASSIS_TYPE_TANKBLACK,
}ChassisTypeEnum;

typedef struct {
	ChassisTypeEnum chassis_type;
	void (*set_velocity)(void *self, float vx, float vy, float angular_rate);
	void (*set_velocity_radius)(void *self, float linear, float r, bool insitu);
	void (*stop)(void *self);
} ChassisTypeDef;




#define JETTANK_WHEEL_DIAMETER 54.0 /* mm */
#define JETTANK_CORRECITION_FACTOR 1.0 /* mm */
#define JETTANK_SHAFT_LENGTH 203.8 /* mm */

#define TANKBLACK_WHEEL_DIAMETER 54.0 /* mm */
#define TANKBLACK_CORRECITION_FACTOR 1.0 /* mm */
#define TANKBLACK_SHAFT_LENGTH 152.8 /* mm */

#define JETAUTO_WHEEL_DIAMETER 96.5 /* mm */
#define JETAUTO_CORRECITION_FACTOR 1.0 /* mm */
#define JETAUTO_SHAFT_LENGTH 218.0 /* mm */
#define JETAUTO_WHEELBASE  195.0 /* mm */

#define TI4WD_WHEEL_DIAMETER 45.0 /* mm */
#define TI4WD_CORRECITION_FACTOR 1.0 /* mm */
#define TI4WD_SHAFT_LENGTH 129.0 /* mm */
#define TI4WD_WHEELBASE  145.0 /* mm */
#endif

