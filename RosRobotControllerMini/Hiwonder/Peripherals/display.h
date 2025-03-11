/**
 * @file display.h
 * @author Lu Yongping (Lucas@hiwonder.com)
 * @brief 显示设备基础接口
 * @version 0.1
 * @date 2023-05-11
 *
 * @copyright Copyright (c) 2023
 *
 */


#ifndef __DISPLAY_H__
#define __DISPLAY_H__

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>


/**
  * @defgroup Display Display
  * @{
  * @defgroup base
  * @{
 */


typedef struct DisplayObject DisplayObjectTypeDef;

/**
  * @brief 所有显示屏设备的基类
  *
 */
struct DisplayObject {
    uint16_t width;            /**< @brief 屏幕像素宽度 */
    uint16_t height;           /**< @brief 屏幕像素高度 */
	
	/* 接口 */
    /**
      * @brief 屏幕复位接口
      * @param self 屏幕对象指针
	  * @retval None.
    */
    void (*reset)(DisplayObjectTypeDef *self);

    /**
      * @brief 屏幕画点接口
      * @param self 屏幕对象指针
      * @param x 要画的点的X轴坐标
      * @param y 要画的点的Y轴坐标
      * @param color 要画的点的颜色
	  * @retval None.
    */
    void (*draw_pixel)(DisplayObjectTypeDef *self, int x, int y, uint32_t color);

    /**
      * @brief 屏幕画位图接口
      * @param self 屏幕对象指针
      * @param x1 位图左上角X轴坐标
      * @param y1 位图左上角Y轴坐标
      * @param x2 位图右下角X轴坐标
      * @param y2 位图右下角Y轴坐标
      * @param data 要画的位图数据
	  * @retval None.
    */
    void (*draw_bitmap)(DisplayObjectTypeDef *self, int x1, int y1, int x2, int y2, uint8_t *data);

    /**
      * @brief 打开屏幕显示
      * @param self 屏幕对象指针
	  * @retval None.
    */
    void (*display_on)(DisplayObjectTypeDef *self);

    /**
      * @brief 关闭屏幕显示
      * @param self 屏幕对象指针
	  * @retval None.
    */
    void (*display_off)(DisplayObjectTypeDef *self);

    /**
      * @brief 背光设置接口
      * @param self ST7789实例指针
      * @param brightness 亮度 (0~100)
      * @retval None.
    */
    void (*set_backlight)(DisplayObjectTypeDef *self, uint32_t brightness);
};

/**
  * @}
  * @}
 */

#endif
