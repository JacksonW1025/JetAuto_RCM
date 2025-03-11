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

#include "cmsis_os2.h"

#include "lvgl/lvgl.h"
#include "lv_port_disp.h"

#include "gui_guider.h"
#include "object.h"
#include "button.h"
#include "sbus.h"
#include "lwmem_porting.h"
#include "global_conf.h"


lv_ui guider_ui;

SBusStatusObjectTypeDef sbus_status_disp;
void update_sbus_view(void)
{
    if(guider_ui.screen_sbus_del == true) {
        lv_label_set_text_fmt(guider_ui.screen_sbus_label_lx, "LX:%4d", sbus_status_disp.channels[2]);
        lv_label_set_text_fmt(guider_ui.screen_sbus_label_ly, "LY:%4d", sbus_status_disp.channels[3]);
        lv_label_set_text_fmt(guider_ui.screen_sbus_label_rx, "RX:%4d", sbus_status_disp.channels[0]);
        lv_label_set_text_fmt(guider_ui.screen_sbus_label_ry, "RY:%4d", sbus_status_disp.channels[1]);
        lv_label_set_text_fmt(guider_ui.screen_sbus_label_sw5, "SWA:%4d", sbus_status_disp.channels[4]);
        lv_label_set_text_fmt(guider_ui.screen_sbus_label_sw6, "SWB:%4d", sbus_status_disp.channels[5]);
        lv_label_set_text_fmt(guider_ui.screen_sbus_label_sw1, "SWD:%4d", sbus_status_disp.channels[7]);
        lv_label_set_text_fmt(guider_ui.screen_sbus_label_sw2, "SWC:%4d", sbus_status_disp.channels[6]);
    }
}


void lvgl_timer_callback(void *argument)
{
    extern osMessageQueueId_t lvgl_event_queueHandle;
    ObjectTypeDef object;
    object.structure.type_id = OBJECT_TYPE_ID_LVGL_UPDATE;
    osMessageQueuePut(lvgl_event_queueHandle, &object, 0, 10);
}

#if ENABLE_LVGL
void gui_task_entry(void *arg)
{
    extern osMessageQueueId_t lvgl_event_queueHandle;
    extern osTimerId_t lvgl_timerHandle;

    lv_init();
    lv_port_disp_init();

    setup_ui(&guider_ui);
    setup_scr_screen_startup(&guider_ui);
    setup_scr_screen_sys(&guider_ui);
    setup_scr_screen_imu(&guider_ui);
    setup_scr_screen_ps2(&guider_ui);
    setup_scr_screen_sbus(&guider_ui);

    lv_event_send(guider_ui.screen_empty, LV_EVENT_CLICKED, NULL);
    for(int i = 0; i < 10; ++i) {
        lv_task_handler();
        osDelay(10);
    }
	
	osTimerStart(lvgl_timerHandle, 40);
    lv_event_send(guider_ui.screen_startup, LV_EVENT_CLICKED, NULL);

    ObjectTypeDef event;
    uint8_t msg_prio = 0;
    for(;;) {
        osMessageQueueGet(lvgl_event_queueHandle, &event, &msg_prio, osWaitForever);
        switch(event.structure.type_id) {
            case OBJECT_TYPE_ID_BATTERY_VOLTAGE: {
                lv_label_set_text_fmt(guider_ui.screen_sys_label_volt, "%dmv", *((uint16_t*)(event.structure.data)));
                break;
            }
            case OBJECT_TYPE_ID_LVGL_UPDATE: {
                lv_task_handler();
                break;
            }
            default:
                break;
        }
    }
}
#endif


