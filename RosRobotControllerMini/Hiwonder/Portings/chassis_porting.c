#include "global.h"
#include "differential_chassis.h"
#include "mecanum_chassis.h"
#include "encoder_motor.h"
#include "chassis.h"
#include "motors_param.h"

DifferentialChassisTypeDef jetank;
DifferentialChassisTypeDef tank_black;
DifferentialChassisTypeDef ti4wd;
MecanumChassisTypeDef      jetauto;


static void jetank_set_motors(void* self, float rps_l, float rps_r)
{
    encoder_motor_set_speed(motors[0], rps_r);
    encoder_motor_set_speed(motors[3], -rps_l);
}


static void tankblack_set_motors(void* self, float rps_l, float rps_r)
{
    encoder_motor_set_speed(motors[0], -rps_l);
    encoder_motor_set_speed(motors[1], rps_r);
}

static void ti4wd_set_motors(void* self, float rps_l, float rps_r)
{
    encoder_motor_set_speed(motors[3], rps_r);
    encoder_motor_set_speed(motors[2], -rps_l);
    encoder_motor_set_speed(motors[0], -rps_l);
    encoder_motor_set_speed(motors[1], rps_r);
}

static void jetauto_set_motors(void* self, float rps_lh, float rps_lt, float rps_rh, float rps_rt)
{
    encoder_motor_set_speed(motors[1], rps_lh);
    encoder_motor_set_speed(motors[0], rps_lt);
    encoder_motor_set_speed(motors[2], -rps_rh);
    encoder_motor_set_speed(motors[3], -rps_rt);
}


ChassisTypeDef *chassis = (ChassisTypeDef*)&jetauto;

void chassis_init(void)
{
    diff_chassis_object_init(&jetank);
    jetank.base.chassis_type = CHASSIS_TYPE_JETTANK;
    jetank.correction_factor = JETTANK_CORRECITION_FACTOR;
    jetank.wheel_diameter = JETTANK_WHEEL_DIAMETER;
    jetank.shaft_length = JETTANK_SHAFT_LENGTH;
    jetank.set_motors = jetank_set_motors;

    diff_chassis_object_init(&tank_black);
    tank_black.base.chassis_type = CHASSIS_TYPE_JETTANK;
    tank_black.correction_factor = JETTANK_CORRECITION_FACTOR;
    tank_black.wheel_diameter = JETTANK_WHEEL_DIAMETER;
    tank_black.shaft_length = JETTANK_SHAFT_LENGTH;
    tank_black.set_motors = tankblack_set_motors;

    diff_chassis_object_init(&ti4wd);
    ti4wd.base.chassis_type = CHASSIS_TYPE_TI4WD;
    ti4wd.correction_factor = TI4WD_CORRECITION_FACTOR;
    ti4wd.wheel_diameter = TI4WD_WHEEL_DIAMETER;
    ti4wd.shaft_length = TI4WD_SHAFT_LENGTH;
    ti4wd.set_motors = ti4wd_set_motors;

    mecanum_chassis_object_init(&jetauto);
    jetauto.base.chassis_type = CHASSIS_TYPE_JETAUTO;
    jetauto.correction_factor = JETAUTO_CORRECITION_FACTOR;
    jetauto.wheel_diameter = JETAUTO_WHEEL_DIAMETER;
    jetauto.shaft_length = JETAUTO_SHAFT_LENGTH;
    jetauto.wheelbase = JETAUTO_WHEELBASE;
    jetauto.set_motors = jetauto_set_motors;

}

void set_chassis_type(uint8_t chassis_type)
{
    switch(chassis_type) {
        case CHASSIS_TYPE_JETAUTO:
            chassis = (ChassisTypeDef*)&jetauto;
            set_motor_type(motors[0], MOTOR_TYPE_JGB520);
            set_motor_type(motors[1], MOTOR_TYPE_JGB520);
            set_motor_type(motors[2], MOTOR_TYPE_JGB520);
            set_motor_type(motors[3], MOTOR_TYPE_JGB520);
            break;

        case CHASSIS_TYPE_JETTANK:
            chassis = (ChassisTypeDef*)&jetank;
            set_motor_type(motors[0], MOTOR_TYPE_JGB528);
            set_motor_type(motors[1], MOTOR_TYPE_JGB528);
            set_motor_type(motors[2], MOTOR_TYPE_JGB528);
            set_motor_type(motors[3], MOTOR_TYPE_JGB528);
            break;
        case CHASSIS_TYPE_TI4WD:
            chassis = (ChassisTypeDef*)&ti4wd;
            set_motor_type(motors[0], MOTOR_TYPE_JGA27);
            set_motor_type(motors[1], MOTOR_TYPE_JGA27);
            set_motor_type(motors[2], MOTOR_TYPE_JGA27);
            set_motor_type(motors[3], MOTOR_TYPE_JGA27);
			break;
        case CHASSIS_TYPE_TANKBLACK:
            chassis = (ChassisTypeDef*)&tank_black;
            set_motor_type(motors[0], MOTOR_TYPE_JGB37);
            set_motor_type(motors[1], MOTOR_TYPE_JGB37);
            set_motor_type(motors[2], MOTOR_TYPE_JGB37);
            set_motor_type(motors[3], MOTOR_TYPE_JGB37);
			break;
        default:
            break;
    }
}

