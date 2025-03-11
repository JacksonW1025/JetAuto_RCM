/**
 * @file imu_mpu6050.c
 * @author Lu Yongping (Lucas@hiwonder.com)
 * @brief mpu6050驱动的实现
 * @version 0.1
 * @date 2023-05-08
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "imu_mpu6050.h"
#include <string.h>
#include "Fusion.h"

/**
*   @ingroup IMU 
*   @{
*   @addtogroup MPU6050
*   @{
*/

/**
    * @brief 设置mpu6050陀螺仪传感器满量程范围
    * @param self mpu6050对象实例指针
    * @param fsr @li 0 ±250dps
    *            @li 1 ±500dps
    *            @li 2 ±1000dps
    *            @li 3 ±2000dps
    * @retval  0 设置成功
    * @retval !=0 设置失败
    */
int mpu6050_set_gyro_fsr(MPU6050ObjectTypeDef *self, uint32_t fsr)
{
    /* 无论量程多少输出数值都是16位有符号数，根据量程确定每1°对应的数值是多少(查芯片手册) */
    switch(fsr) {
    case MPU6050_GYRO_FSR_250DPS:
        self->gyro_sf = 131.0f;
        break;
    case MPU6050_GYRO_FSR_500DPS:
        self->gyro_sf = 65.5f;
        break;
    case MPU6050_GYRO_FSR_1000DPS:
        self->gyro_sf = 32.8f;
        break;
    case MPU6050_GYRO_FSR_2000DPS:
        self->gyro_sf = 16.4f;
        break;
    default:
        return -2;
    }
    return self->i2c_write_byte_to_mem(self, MPU6050_GYRO_CONFIG, (uint8_t)(fsr << 3));
}

/**
* @brief 设置mpu6050加速度传感器满量程范围
    * @param self mpu6050对象实例指针
    * @param fsr @li 0 ±2g
    *            @li 1 ±4g
    *            @li 2 ±8g
    *            @li 3 ±16g
    * @retval  0 设置成功
    * @retval !=0 设置失败
    */
int mpu6050_set_accel_fsr(MPU6050ObjectTypeDef *self, uint32_t fsr)
{
    switch(fsr) {
    case MPU6050_ACCEL_FSR_2G:
        self->accel_sf = 16384.0f;
        break;
    case MPU6050_ACCEL_FSR_4G:
        self->accel_sf = 8192.0f;
        break;
    case MPU6050_ACCEL_FSR_8G:
        self->accel_sf = 4096.0f;
        break;
    case MPU6050_ACCEL_FSR_16G:
        self->accel_sf = 2048.0f;
        break;
    default:
        return -2;
    }
    return self->i2c_write_byte_to_mem(self, MPU6050_ACCEL_CONFIG, (uint8_t)(fsr << 3));
}

/**
* @brief 设置mpu6050的数字低通滤波器
* @param self mpu6050对象实例指针
* @param lpf 数字低通滤波器频率(Hz)
* @retval 0 设置成功
* @retval !=0 设置失败
*/
int mpu6050_set_lpf(MPU6050ObjectTypeDef *self, uint32_t lpf)
{
    uint8_t data = 0;
    if(lpf > 188) {
        data = 1;
    } else if (lpf > 98) {
        data = 2;
    } else if (lpf > 42) {
        data = 3;
    } else if (lpf > 20) {
        data = 4;
    } else if(lpf > 10) {
        data = 5;
    } else {
        data = 6;
    }
    return self->i2c_write_byte_to_mem(self, MPU6050_CONFIG, data);
}

/**
* @brief 设置mpu6050采样率
* @param self mpu6050对象实例指针
* @param rate 新的采样率 4~1000(Hz)
* @retval 0 设置成功
* @retval != 设置失败
*/
int mpu6050_set_rate(MPU6050ObjectTypeDef *self, uint32_t rate)
{
    uint32_t data = 0;
    if(rate > 1000) {
        rate = 1000;
    }
    if(rate < 4) {
        rate = 4;
    }
    data = 1000 / rate - 1;

    if(self->i2c_write_byte_to_mem(self, MPU6050_SMPLRT_DIV, data) != 0) { // 设置采样率
        return -2;
    }

    if(mpu6050_set_lpf(self, rate / 2) != 0) { // 将数字低通滤波器设为采样率的一半
        return -1;
    }
    return 0;
}

/**
* @brief 读取mpu6050温度传感器数值
* @param self mpu6050对象实例指针
* @param temp 温度结果指针
* @retval 0 读取成功
* @retval !=0 读取失败
*/
int mpu6050_get_temperature(MPU6050ObjectTypeDef *self, float *temp)
{
    uint8_t buf[2] = {0};
    uint16_t raw = 0;
    if(self->i2c_read_from_mem(self, MPU6050_TEMP_OUT_H, 2, buf) != 0) {
        return -1;
    }
    raw = (((uint16_t)buf[0]) << 8) | ((uint16_t)buf[1]);
    printf("0x%.4X\r\n", raw);
    *temp = ((float)((int16_t)raw)) / 340.0f + 36.53f;
    return 0;
}

/**
* @brief 读取mpu6050加速度传感器数值
* @param self mpu6050对象实例指针
* @param x x轴结果存储指针
* @param y y轴结果存储指针
* @param z z轴结果存储指针
* @retval 0 读取成功
* @retval != 读取失败
*/
int mpu6050_get_accel(MPU6050ObjectTypeDef *self, float *x, float *y, float *z)
{
    uint8_t buf[6] = {0};
    if(self->i2c_read_from_mem(self, MPU6050_ACCEL_XOUT_H, 6, buf) != 0) {
        return -1;
    }
    int16_t acc_x_raw = ((uint16_t)buf[0] << 8) | (uint16_t)buf[1];
    int16_t acc_y_raw = ((uint16_t)buf[2] << 8) | (uint16_t)buf[3];
    int16_t acc_z_raw = ((uint16_t)buf[4] << 8) | (uint16_t)buf[5];
    *x = (float)((int)acc_x_raw) / self->accel_sf;
    *y = (float)((int)acc_y_raw) / self->accel_sf;
    *z = (float)((int)acc_z_raw) / self->accel_sf;
    return 0;
}

/**
* @brief 读取mpu6050陀螺仪数值
* @param self mpu6050对象实例指针
* @param gx x轴结果存储指针
* @param gy y轴结果存储指针
* @param gz z轴结果存储指针
* @retval 0 读取成功
* @retval != 读取失败
*/
int mpu6050_get_gyro(MPU6050ObjectTypeDef *self, float *gx, float *gy, float *gz)
{
    uint8_t buf[6] = {0};
    if(self->i2c_read_from_mem(self, MPU6050_GYRO_XOUT_H, 6, buf) != 0) {
        return -1;
    }

    int16_t gyro_x_raw = ((uint16_t)buf[0] << 8) | (uint16_t)buf[1];
    int16_t gyro_y_raw = ((uint16_t)buf[2] << 8) | (uint16_t)buf[3];
    int16_t gyro_z_raw = ((uint16_t)buf[4] << 8) | (uint16_t)buf[5];
    *gx = (float)((int)gyro_x_raw) / self->gyro_sf;
    *gy = (float)((int)gyro_y_raw) / self->gyro_sf;
    *gz = (float)((int)gyro_z_raw) / self->gyro_sf;
    return 0;
}

/**
* @brief 读取mpu6050 加速度、温度、陀螺仪数值
* @param self mpu6050对象实例指针
* @param accel xyz三轴加速度结果存储指针
* @param gyro  xyz三轴角速度结果存储指针
* @retval 0 读取成功
* @retval !=0 读取失败
*/
int mpu6050_get_all(MPU6050ObjectTypeDef *self, float *accel, float *temp, float *gyro)
{
    static uint8_t buf[14] = {0};
    if(self->i2c_read_from_mem(self, MPU6050_ACCEL_XOUT_H, 14, buf) != 0) {
        return -1;
    }

    int16_t acc_x_raw = ((uint16_t)buf[0] << 8) | (uint16_t)buf[1];
    int16_t acc_y_raw = ((uint16_t)buf[2] << 8) | (uint16_t)buf[3];
    int16_t acc_z_raw = ((uint16_t)buf[4] << 8) | (uint16_t)buf[5];
    int16_t temp_raw = ((uint16_t)buf[6] << 8) | (uint16_t)buf[7];
    int16_t gyro_x_raw = ((uint16_t)buf[8] << 8) | (uint16_t)buf[9];
    int16_t gyro_y_raw = ((uint16_t)buf[10] << 8) | (uint16_t)buf[11];
    int16_t gyro_z_raw = ((uint16_t)buf[12] << 8) | (uint16_t)buf[13];

    *accel++ = (float)((int)acc_x_raw) / self->accel_sf;
    *accel++ = (float)((int)acc_y_raw) / self->accel_sf;
    *accel++ = (float)((int)acc_z_raw) / self->accel_sf;
    *temp = ((float)((int)temp_raw)) / 340.0f + 36.53f;
    *gyro++ = (float)((int)gyro_x_raw) / self->gyro_sf;
    *gyro++ = (float)((int)gyro_y_raw) / self->gyro_sf;
    *gyro++ = (float)((int)gyro_z_raw) / self->gyro_sf;

    return 0;
}


/**
* @brief 复位mpu6050设备
* @param self mpu6050对象实例指针
* @retval None
*/
static void export_mpu6050_reset(IMU_ObjectTypeDef *self_base)
{
	MPU6050ObjectTypeDef *self = (MPU6050ObjectTypeDef*)self_base;
    self->i2c_write_byte_to_mem(self, MPU6050_PWR_MGMT_1, 0x80); // 复位mpu6050
    self->i2c_write_byte_to_mem(self, MPU6050_PWR_MGMT_1, 0x80); // 复位mpu6050
    self->sleep_ms(50);
    self->i2c_write_byte_to_mem(self, MPU6050_PWR_MGMT_1, 0x00); // 唤醒mpu6050
    self->i2c_write_byte_to_mem(self, MPU6050_PWR_MGMT_1, 0x00); // 唤醒mpu6050

    mpu6050_set_accel_fsr(self, MPU6050_ACCEL_FSR_4G); // 设置加速度量程为±4G
    mpu6050_set_gyro_fsr(self, MPU6050_GYRO_FSR_2000DPS); // 设置角速度量程为±2000°/s

    self->i2c_write_byte_to_mem(self, MPU6050_INT_EN_REG, 0x00); //关闭中断
    self->i2c_write_byte_to_mem(self, MPU6050_USER_CTRL, 0x00);  // 关闭I2C主模式
    self->i2c_write_byte_to_mem(self, MPU6050_FIFO_EN_REG, 0x00); // 关闭FIFO
    self->i2c_write_byte_to_mem(self, MPU6050_INT_PIN_CFG, 0x00); // 中断为高电平触发
    mpu6050_set_rate(self, 100); // 设置采样率为100SPS
    self->i2c_write_byte_to_mem(self, MPU6050_INT_EN_REG, 0x01); // 开启数据就绪中断
}

/**
* @brief 设置mpu6050 FIFO 中断开关
* @param self mpu6050对象实例指针
* @param enable @li true 开启FIFO中断
*               @li false 关闭fifo中断
* @retval 0 设置成功
* @retval !=0 设置失败
*/
int mpu6050_enable_int(MPU6050ObjectTypeDef *self, bool enable)
{
    if(enable) {
        if(self->i2c_write_byte_to_mem(self, MPU6050_INT_EN_REG, 0x01) != 0) {
            return -1;
        }
    } else {
        if(self->i2c_write_byte_to_mem(self, MPU6050_INT_EN_REG, 0x00) != 0) {
            return -1;
        }
    }
    return 0;
}


/**
* @brief 从mpu6050读取所有传感器数值并更新实例成员
* @param self_ mpu6050对象实例指针
* @retval 0 更新成功
* @retval !=0 更新失败
*/
static int export_mpu6050_update(IMU_ObjectTypeDef *self_base)
{
	MPU6050ObjectTypeDef *self = (MPU6050ObjectTypeDef*)self_base;
	//HAL_GPIO_WritePin(LED_SYS_GPIO_Port, LED_SYS_Pin, GPIO_PIN_SET); // 测量采集计算时间上升沿
    if(mpu6050_get_all(self, self->accel, &self->temperature, self->gyro) != 0) {
        return -1;
    }
    FusionVector gyroscope = {.axis.x= self->gyro[0], .axis.y=self->gyro[1], .axis.z = self->gyro[2]};
    FusionVector accelerometer = {.axis.x=self->accel[0], .axis.y = self->accel[1], .axis.z = self->accel[2]};
    gyroscope = FusionOffsetUpdate(&self->offset, gyroscope);
    if(self->offset.cal_count > 0) {
        return 0;
    }

    FusionAhrsUpdateNoMagnetometer(&self->ahrs, gyroscope, accelerometer, 0.01);
    const FusionQuaternion quat = FusionAhrsGetQuaternion(&self->ahrs);
    const FusionEuler euler = FusionQuaternionToEuler(quat);
    memcpy(&self->quat, &quat, sizeof(FusionQuaternion));
    memcpy(&self->euler, &euler, sizeof(FusionEuler));
	//HAL_GPIO_WritePin(LED_SYS_GPIO_Port, LED_SYS_Pin, GPIO_PIN_RESET); // 测量采集计算时间下降沿

    //printf("Roll %0.1f, Pitch %0.1f, Yaw %0.1f\n", euler.angle.roll, euler.angle.pitch, euler.angle.yaw);
    return 0;
}


static int export_mpu6050_get_euler(IMU_ObjectTypeDef *self, float *rpy)
{
	memcpy(rpy, &((MPU6050ObjectTypeDef*)self)->euler, sizeof(FusionEuler));
	return 0;
}


static int export_mpu6050_get_quat(IMU_ObjectTypeDef *self, float *quat)
{
	memcpy(quat, &((MPU6050ObjectTypeDef*)self)->quat, sizeof(FusionQuaternion));
	return 0;
}

static int export_mpu6050_get_accel(IMU_ObjectTypeDef *self, float *accel)
{
	memcpy(accel, &((MPU6050ObjectTypeDef*)self)->accel, sizeof(float) * 3);
	return 0;
}

static int export_mpu6050_get_gyro(IMU_ObjectTypeDef *self, float *gyro)
{
	memcpy(gyro, &((MPU6050ObjectTypeDef*)self)->gyro, sizeof(float) * 3);
	return 0;
}
/**
  * @brief 初始化mpu6050对象内存
  * @param obj 需要初始化内对象指针
  * @param dev_addr 器件地址
  * @retval None
  *
*/
void mpu6050_object_init(MPU6050ObjectTypeDef *obj, uint8_t dev_addr)
{
    memset(obj, 0, sizeof(MPU6050ObjectTypeDef));
    FusionAhrsInitialise(&obj->ahrs);
    FusionOffsetInitialise(&obj->offset, 100);
    obj->dev_addr = dev_addr;
    obj->base.reset = export_mpu6050_reset;
    obj->base.update = export_mpu6050_update;
	obj->base.get_euler = export_mpu6050_get_euler;
	obj->base.get_quat = export_mpu6050_get_quat;
	obj->base.get_accel = export_mpu6050_get_accel;
	obj->base.get_gyro = export_mpu6050_get_gyro;
}

/** @} */
/** @} */
