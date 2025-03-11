#include "usb_host.h"
#include "usbh_core.h"
#include "usbh_hid.h"
#include "usbh_hid_gamepad.h"
#include "cmsis_os2.h"
#include "packet.h"
#include "global.h"
#include "packet_reports.h"

#define ATC_THRESHOLD 60

static char A_T_C(int8_t analog_x, int8_t analog_y);


void USBH_HID_EventCallback(USBH_HandleTypeDef *phost)
{
    extern osMessageQueueId_t moving_ctrl_queueHandle;
    static HID_GAMEPAD_Info_TypeDef last_info;
    static char last_direction_msg = 'I';
    static char last_button = 'R';

    switch(USBH_HID_GetDeviceType(phost)) {
        case 0xFF: {/* 手柄数据 */
                HID_GAMEPAD_Info_TypeDef *info = USBH_HID_GetGamepadInfo(phost);
                if(info == NULL) {
                    break;
                }
                PacketReportGamepadTypeDef report;
                report.buttons = info->buttons;
                report.hat = info->hat;
                report.lx = info->lx;
                report.ly = info->ly;
                report.rx = info->rx;
                report.ry = info->ry;
                packet_transmit(&packet_controller, PACKET_FUNC_GAMEPAD, &report, sizeof(PacketReportGamepadTypeDef));
                memcpy(&last_info, info, sizeof(HID_GAMEPAD_Info_TypeDef));
                break;
            }
        default:
            break;
    }
}


static char A_T_C(int8_t analog_x, int8_t analog_y)
{
    char result = ' ';
    if(analog_x < -ATC_THRESHOLD) {
        if(analog_y < -ATC_THRESHOLD) {
            result = 'D';
        } else if(analog_y > ATC_THRESHOLD) {
            result = 'B';
        } else {
            result = 'C';
        }
    } else if(analog_x > ATC_THRESHOLD) {
        if(analog_y < -ATC_THRESHOLD) {
            result = 'F';
        } else if(analog_y > ATC_THRESHOLD) {
            result = 'H';
        } else {
            result = 'G';
        }
    } else {
        if(analog_y < -ATC_THRESHOLD) {
            result = 'E';
        } else if(analog_y > ATC_THRESHOLD) {
            result = 'A';
        } else {
            result = 'I';
        }
    }
    return result;
}

