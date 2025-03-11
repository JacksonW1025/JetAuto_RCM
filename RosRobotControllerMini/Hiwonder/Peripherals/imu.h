/**
 * @file imu.h
 * @author Lu Yongping (Lucas@hiwonder.com)
 * @brief imu 相关数据定义和函数声明
 * @version 0.1
 * @date 2023-05-06
 *
 * @copyright Copyright (c) 2023
 *
 */
#ifndef _IMU_H_
#define _IMU_H_


#include <stdint.h>
#include <stdio.h>
/**
 * @defgroup IMU
 * @{
*/

/**
* @brief IMU中对于可能出现的各个轴的枚举
*
*/
typedef enum {
    IMU_AXIS_X, /**< @brief IMU X轴 */
    IMU_AXIS_Y, /**< @brief IMU Y轴 */
    IMU_AXIS_Z, /**< @brief IMU Z轴 */
    IMU_AXIS_W, /**< @brief IMU W轴(四元数) */
} IMU_AxisEnum;


typedef struct IMU_Object IMU_ObjectTypeDef; /**< @brief IMU 对象的类型定义声明*/

/**
* @brief IMU 对象的结构体
*
*/
struct IMU_Object {
    /**
      * @brief 复位imu
    */
    void (*reset)(IMU_ObjectTypeDef *self);


    /**
      * @brief 以当前姿态为基准校准imu
      */
    void (*calibrat)(IMU_ObjectTypeDef *self);

    /**
      * @brief 从传感器更新数据
    */
    int (*update)(IMU_ObjectTypeDef *self);

    /**
     * @brief 获取当前姿态欧拉角,
     * @param [out] rpy 用于返回结果数值的长度为3的float数组的指针
     * @retval 0 成功, rpy数值有效
     * @retval !=0 失败, rpy数值无效
    */
    int (*get_euler)(IMU_ObjectTypeDef *self, float *rpy);


    /**
      * @brief 获取当前姿态四元数,
      * @param [out] quat 用于返回结果数值的长度为4的float数组的指针
      * @retval 0 成功, quat数值有效
      * @retval !=0 失败, quat数值无效
    */
    int (*get_quat)(IMU_ObjectTypeDef *self, float *quat);


    /**
      * @brief 获取当前各个轴的加速度G值,
      * @param [out] xyz 用于返回结果数值的长度为3的float数组的指针
      * @retval 0 成功, xyz数值有效
      * @retval !=0 失败, xyz数值无效
    */
    int (*get_accel)(IMU_ObjectTypeDef *self, float *xyz);


    /**
      * @brief 获取当前各个轴的角速度值,
      * @param [out] xyz 用于返回结果数值的长度为3的float数组的指针
      * @retval 0 成功, xyz数值有效
      * @retval !=0 失败, xyz数值无效
    */
    int (*get_gyro)(IMU_ObjectTypeDef *self, float *xyz);


    /**
      * @brief 获取当前某个轴的加速度G值,
      * @param axis 指定要获取的轴
      * @param [out] accel 用于返回结果加速度的指针
      * @retval 0 成功, accel数值有效
      * @retval !=0 失败, accel数值无效
    */
    int (*get_accel_axis)(IMU_ObjectTypeDef *self, IMU_AxisEnum axis, float *accel);


    /**
      * @brief 获取当前某个轴的角速度值,
      * @param axis 指定要获取的轴
      * @param [out] gyro 用于返回结果角速度的指针
      * @retval 0 成功, gyro数值有效
      * @retval !=0 失败, gyro数值无效
    */
    int (*get_gyro_axis)(IMU_ObjectTypeDef *self, IMU_AxisEnum axis, float *gyro);
};

/** @} */
#endif

