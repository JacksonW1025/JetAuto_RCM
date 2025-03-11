/**
 * @file lvgl_handle.c
 * @author Lu Yongping (Lucas@hiwonder.com)
 * @brief lvgl界面的数据、信息、显示内容更新
 * @version 0.1
 * @date 2023-06-12
 *
 * @copyright Copyright (c) 2023
 *
 */
#include <stdio.h>
#include "cmsis_os2.h"
#include "object.h"
#include "lwmem_porting.h"
#include "global_conf.h"
#include "u8g2_porting.h"



#if ENABLE_OLED
char oled_l1[24] = "---.---.---.---";
char oled_l2[24] = "---.---.---.---";
int oled_battery = 0;
char oled_msg[24];
extern osMutexId_t oled_mutexHandle;
void oled_task_entry(void *arg)
{
	u8g2_init();
    for(;;) {
		u8g2_ClearBuffer(u8g2);
		u8g2_SetFontMode(u8g2, 1);
		u8g2_SetFontDirection(u8g2, 0);
		u8g2_SetFont(u8g2, u8g2_font_synchronizer_nbp_tf      );
		osMutexAcquire(oled_mutexHandle, osWaitForever);
		u8g2_DrawStr(u8g2, 0, 9, oled_l1);
		u8g2_DrawStr(u8g2, 0, 20, oled_l2);
		if (oled_battery != 0) {
			sprintf(oled_msg, "BAT:%dmv", oled_battery);
			u8g2_DrawStr(u8g2, 0, 31, oled_msg);
		}
		u8g2_SendBuffer(u8g2);
		osMutexRelease(oled_mutexHandle);
		osDelay(500);
	}
}
#endif


