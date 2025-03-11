/**
 * @file display_st7735.h
 * @author Lu Yongping (Lucas@hiwonder.com)
 * @brief ST7735 显示屏驱动头文件
 * @version 0.1
 * @date 2023-05-11
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef __DISPLAY_ST7735_H
#define __DISPLAY_ST7735_H

#include "display.h"
#include "display_st7735_reg.h"
/**
  * @addtogroup Display
  * @{
  * @defgroup ST7735 ST7735
  * @{
 */

typedef struct ST7735Object ST7735ObjectTypeDef;
/**
  * @brief ST7735 对象结构体
  *
*/
struct ST7735Object{
	DisplayObjectTypeDef base; /* 显示设备基类 */
	
	/* config */
	int x_offset; /**< @brief X轴显示偏移 */
	int y_offset; /**< @brief Y轴显示偏移 */
	
	/* 移植接口 */
	/**
	  * @brief SPI写入接口
	  * @param self ST7735实例指针
	  * @param data 写入数据指针
	  * @param len 写入数据长度
	  * @retval None.
	*/
	void (*spi_write)(ST7735ObjectTypeDef *self, uint8_t *data, size_t len);  
	/**
	  * @brief 数据/命令、片选引脚设置接口
	  * @param self ST7735实例指针
	  * @param new_dc 数据/命令引脚电平
	  * @param new_cs 片选引脚电平
	  * @retval None.
	*/
	void (*set_dc_cs)(ST7735ObjectTypeDef *self, uint32_t new_dc, uint32_t new_cs);
	/**
	  * @brief 数据/命令、片选引脚设置接口
	  * @param self ST7735实例指针
	  * @param new_dc 数据/命令引脚电平
	  * @param new_cs 片选引脚电平
	  * @retval None.
	*/
	void (*set_res)(ST7735ObjectTypeDef *self, uint32_t new_res);
	
	/**
	  * @brief 背光设置接口
	  * @param self ST7735实例指针
	  * @param brightness 亮度 (0~100)
	  * @retval None.
	*/
	void (*set_backlight)(ST7735ObjectTypeDef *self, uint32_t brightness);
	/**
	  * @brief 延时接口
	  * @param self ST7735实例指针
	  * @param ms 延时事件， 单位毫秒
	  * @retval None.
	*/
	void (*sleep_ms)(ST7735ObjectTypeDef *self, uint32_t ms);
};


void st7735_object_init(ST7735ObjectTypeDef *self);

/** 
  * @}
  * @}
*/

#endif
