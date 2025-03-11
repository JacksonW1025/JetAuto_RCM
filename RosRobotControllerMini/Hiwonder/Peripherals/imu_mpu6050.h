/**
 * @file imu_mpu6050.h
 * @author Lu Yongping (Lucas@hiwonder.com)
 * @brief mpu6050的寄存器列表、函数、数据结构等
 * @version 0.1
 * @date 2023-05-08
 *
 * @copyright Copyright (c) 2023
 *
 */
 
#ifndef __IMU_MPU6050_H_
#define __IMU_MPU6050_H_

#include "imu.h"
#include <stdbool.h>
#include "Fusion.h"

/**
*   @defgroup IMU 
*   @{
*   @defgroup MPU6050
*   @{
*/


/**
*@defgroup MPU6050常量
* @{
*/
#define MPU6050_GYRO_FSR_250DPS  0  /**< MPU6050 陀螺仪量程为 ±250°/s */
#define MPU6050_GYRO_FSR_500DPS  1  /**< MPU6050 陀螺仪量程为 ±500°/s */
#define MPU6050_GYRO_FSR_1000DPS 2  /**< MPU6050 陀螺仪量程为 ±1000°/s */
#define MPU6050_GYRO_FSR_2000DPS 3  /**< MPU6050 陀螺仪量程为 ±2000°/s */

#define MPU6050_ACCEL_FSR_2G       0  /**< MPU6050 加速度量程为 ±2g */
#define MPU6050_ACCEL_FSR_4G       1  /**< MPU6050 加速度量程为 ±4g */
#define MPU6050_ACCEL_FSR_8G       2  /**< MPU6050 加速度量程为 ±8g */
#define MPU6050_ACCEL_FSR_16G      3  /**< MPU6050 加速度量程为 ±16g */
/** @} */

/**
*@defgroup MPU6050器件地址
* @{
*/
#define MPU6050_DEV_ADDR_1 0x68 /**< mpu6050器件地址 */
#define MPU6050_DEV_ADDR_2 0x69 /**< mpu6050器件地址 */
/** @} */

/**
* @defgroup MPU6050寄存器地址
* @{
*/
#define MPU6050_SMPLRT_DIV   0x19  /**< 陀螺仪采样率，典型值：0x07(125Hz) */
#define MPU6050_CONFIG       0x1A  /**< 低通滤波频率，典型值：0x06(5Hz)  */
#define MPU6050_GYRO_CONFIG  0x1B  /**< 陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s) */
#define MPU6050_ACCEL_CONFIG 0x1C  /**< 加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz) */

#define MPU6050_ACCEL_XOUT_H 0x3B  /**< X轴加速度高位数据 */
#define MPU6050_ACCEL_XOUT_L 0x3C  /**< X轴加速度低位数据 */
#define MPU6050_ACCEL_YOUT_H 0x3D  /**< Y轴加速度高位数据 */
#define MPU6050_ACCEL_YOUT_L 0x3E  /**< Y轴加速度低位数据 */
#define MPU6050_ACCEL_ZOUT_H 0x3F  /**< Z轴加速度高位数据 */
#define MPU6050_ACCEL_ZOUT_L 0x40  /**< Z轴加速度低位数据 */

#define MPU6050_TEMP_OUT_H 0x41  /**< 传感器温度高位数据 */
#define MPU6050_TEMP_OUT_L 0x42  /**< 传感器温度低位数据 */

#define MPU6050_GYRO_XOUT_H 0x43  /**< X轴陀螺仪高位数据 */
#define MPU6050_GYRO_XOUT_L 0x44  /**< X轴陀螺仪低位数据 */
#define MPU6050_GYRO_YOUT_H 0x45  /**< Y轴陀螺仪高位数据 */
#define MPU6050_GYRO_YOUT_L 0x46  /**< Y轴陀螺仪低位数据 */
#define MPU6050_GYRO_ZOUT_H 0x47  /**< Z轴陀螺仪高位数据 */
#define MPU6050_GYRO_ZOUT_L 0x48  /**< Z轴陀螺仪低位数据 */

#define MPU6050_FIFO_EN_REG 0x23 /**< FIFO使能寄存器 */
#define MPU6050_USER_CTRL   0x6A /**< FIFO 控制寄存器 */
#define MPU6050_PWR_MGMT_1 0x6B  /**< 电源管理，典型值：0x00(正常启用) */
#define MPU6050_WHO_AM_I 0x75    /**< IIC地址寄存器(默认数值0x68，只读) */
#define MPU6050_INT_EN_REG  0x38 /**< 中断使能寄存器 */

#define MPU6050_INT_PIN_CFG 0x37 /**< 中断引脚配置寄存器 */


/** @} */


typedef struct MPU6050Object  MPU6050ObjectTypeDef;
struct MPU6050Object {
    IMU_ObjectTypeDef base;

    FusionAhrs ahrs;
    FusionEuler euler;
    FusionQuaternion quat;
    FusionOffset offset; /**< 陀螺仪零偏调整器，当连续上时间静止时，自动校正陀螺仪零偏 */

    FusionMatrix accel_misalignment; /**< 加速度传感器轴向偏差 */
    FusionMatrix accel_sensitivity; /**< 加速度传感器刻度缩放因子 */
    FusionMatrix accel_offset; /**< 加速度传感器零偏 */

    FusionMatrix gyro_misalignment; /**< 陀螺仪轴向偏差 */
    FusionMatrix gyro_sensitivity; /**< 陀螺仪刻度缩放因子 */
    FusionMatrix gyro_offset; /**< 陀螺仪零偏 */

    uint8_t dev_addr; /**< 本实例对应的器件地址 */
    bool data_ready; /**< 数据就绪？外部中断里面 */
    float gyro_sf; /**< 陀螺仪数值缩放因子(每1°对应的数值)*/
    float accel_sf;  /**< 加速度缩放因子(每1g对应的数值) */
    float accel[3]; /**< 最后一次更新到的三轴加速度 */
    float gyro[3]; /**< 最后一次更新到的三轴角速度 */
    float temperature; /**< 最后一个更新到的器件温度 */

    /**
    * @brief 延时x毫秒
    * @param ms 要延时的毫秒数
    */
    void (*sleep_ms)(uint32_t ms);

    /**
     * @brief 向I2C设备指定寄存器写入数据
    * @param self mpu6050对象示例指针
    * @param reg_addr 要写入的寄存器起始地址
    * @param len 要写入的寄存器个数
    * @param data 要写入的数据指针
    * @retval 0 读取成功
        * @retval !=0 读取失败
    */
    int (*i2c_write_byte_to_mem)(MPU6050ObjectTypeDef *self, uint8_t reg_addr, uint8_t data);

    /**
     * @brief 从I2C设备指定寄存器读取数据
     * @param self mpu6050对象示例指针
     * @param reg_addr 要读取的寄存器起始地址
     * @param len 要读取的寄存器个数
     * @param data 存放结果数据的指针
     * @retval 0 读取成功
     * @retval !=0 读取失败
    */
    int (*i2c_read_from_mem)(MPU6050ObjectTypeDef *self, uint8_t reg_addr, uint32_t len, uint8_t *data);
};



int mpu6050_set_gyro_fsr(MPU6050ObjectTypeDef *self, uint32_t fsr);
int mpu6050_set_accel_fsr(MPU6050ObjectTypeDef *self, uint32_t fsr);
int mpu6050_set_lpf(MPU6050ObjectTypeDef *self, uint32_t lpf);
int mpu6050_set_rate(MPU6050ObjectTypeDef *self, uint32_t rate);
int mpu6050_get_temperature(MPU6050ObjectTypeDef *self, float *temp);
int mpu6050_get_accel(MPU6050ObjectTypeDef *self, float *x, float *y, float *z);
int mpu6050_get_gyro(MPU6050ObjectTypeDef *self, float *gx, float *gy, float *gz);
int mpu6050_get_all(MPU6050ObjectTypeDef *self, float *accel, float *temp, float *gyro);
void mpu6050_object_init(MPU6050ObjectTypeDef *obj, uint8_t dev_addr);

/** @} */
/** @} */

#endif
