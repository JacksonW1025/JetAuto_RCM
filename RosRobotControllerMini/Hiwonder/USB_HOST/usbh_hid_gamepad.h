
#ifndef __USBH_HID_GAMEPAD_H
#define __USBH_HID_GAMEPAD_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "usbh_hid.h"
#include <stdbool.h>

typedef struct {
    uint16_t buttons;
	uint8_t hat;
    int8_t lx;
    int8_t ly;
    int8_t rx;
    int8_t ry;
} HID_GAMEPAD_Info_TypeDef;

USBH_StatusTypeDef USBH_HID_GamepadInit(USBH_HandleTypeDef *phost);
HID_GAMEPAD_Info_TypeDef *USBH_HID_GetGamepadInfo(USBH_HandleTypeDef *phost);

#define GAMEPAD_BUTTON_MASK_L2        0x0001u
#define GAMEPAD_BUTTON_MASK_R2        0x0002u
#define GAMEPAD_BUTTON_MASK_SELECT    0x0004u
#define GAMEPAD_BUTTON_MASK_START     0x0008u
#define GAMEPAD_BUTTON_MASK_L3        0x0020u
#define GAMEPAD_BUTTON_MASK_R3        0x0040u
#define GAMEPAD_BUTTON_MASK_CROSS     0x0100u
#define GAMEPAD_BUTTON_MASK_CIRCLE    0x0200u
#define GAMEPAD_BUTTON_MASK_SQUARE    0x0800u
#define GAMEPAD_BUTTON_MASK_TRIANGLE  0x1000u
#define GAMEPAD_BUTTON_MASK_L1        0x4000u
#define GAMEPAD_BUTTON_MASK_R1        0x8000u


#define GAMEPAD_GET_BUTTON(gi, km) (((gi)->buttons & km) ? true : false)


#ifdef __cplusplus
}
#endif

#endif /* __USBH_HID_GAMEPAD_H */
