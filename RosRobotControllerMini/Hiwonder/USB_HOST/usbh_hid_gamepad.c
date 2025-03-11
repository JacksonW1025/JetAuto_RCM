
#include "usbh_hid_gamepad.h"
#include "usbh_hid_parser.h"



static USBH_StatusTypeDef USBH_HID_GamepadDecode(USBH_HandleTypeDef *phost);


HID_GAMEPAD_Info_TypeDef    gamepad_info;
uint8_t                  gamepad_report_data[32];
uint8_t                  gamepad_rx_report_buf[32];



/** @defgroup USBH_HID_GAMEPAD_Private_Functions
  * @{
  */

/**
  * @brief  USBH_HID_GamepadInit
  *         The function init the HID gamepad.
  * @param  phost: Host handle
  * @retval USBH Status
  */
USBH_StatusTypeDef USBH_HID_GamepadInit(USBH_HandleTypeDef *phost)
{
    uint32_t i;
    HID_HandleTypeDef *HID_Handle = (HID_HandleTypeDef *) phost->pActiveClass->pData;

    gamepad_info.buttons = 0;
	gamepad_info.lx = 0;
	gamepad_info.ly = 0;
	gamepad_info.rx = 0;
	gamepad_info.ry = 0;

    for (i = 0U; i < sizeof(gamepad_report_data); i++) {
        gamepad_report_data[i] = 0U;
        gamepad_rx_report_buf[i] = 0U;
    }
	
    if (HID_Handle->length > sizeof(gamepad_report_data)) {
        HID_Handle->length = (uint16_t)sizeof(gamepad_report_data);
    }
    HID_Handle->pData = (uint8_t *)(void *)gamepad_rx_report_buf;
    USBH_HID_FifoInit(&HID_Handle->fifo, phost->device.Data, (uint16_t)(HID_QUEUE_SIZE * sizeof(gamepad_report_data)));

    return USBH_OK;
}

/**
  * @brief  USBH_HID_GetGamepadInfo
  *         The function return gamepad information.
  * @param  phost: Host handle
  * @retval gamepad information
  */
HID_GAMEPAD_Info_TypeDef *USBH_HID_GetGamepadInfo(USBH_HandleTypeDef *phost)
{
    if (USBH_HID_GamepadDecode(phost) == USBH_OK) {
        return &gamepad_info;
    } else {
        return NULL;
    }
}

/**
  * @brief  USBH_HID_GamepadDecode
  *         The function decode gamepad data.
  * @param  phost: Host handle
  * @retval USBH Status
  */
static USBH_StatusTypeDef USBH_HID_GamepadDecode(USBH_HandleTypeDef *phost)
{
    HID_HandleTypeDef *HID_Handle = (HID_HandleTypeDef *) phost->pActiveClass->pData;

    if (HID_Handle->length == 0U) {
        return USBH_FAIL;
    }
    /*Fill report */
    if (USBH_HID_FifoRead(&HID_Handle->fifo, &gamepad_report_data, HID_Handle->length) ==  HID_Handle->length) {
		if(gamepad_report_data[20] == 0x02) {
			return USBH_FAIL;
		}
        /*Decode report */
		gamepad_info.buttons = ((uint16_t)gamepad_report_data[6]) << 8 | ((uint16_t)gamepad_report_data[7]);
		gamepad_info.hat = 0x0F & (~gamepad_report_data[5]);
		gamepad_info.lx = ((int)gamepad_report_data[1]) - 128;
		gamepad_info.ly = (127 -(int)gamepad_report_data[2]);
		gamepad_info.rx = ((int)gamepad_report_data[3]) - 128;
		gamepad_info.ry = (127 -(int)gamepad_report_data[4]);
		//printf("buttons:%0.4X, hat:%0.1X, lx:%d, ly:%d, rx:%d, ry:%d\r\n", gamepad_info.buttons, gamepad_info.hat, gamepad_info.lx, gamepad_info.ly, gamepad_info.rx, gamepad_info.ry);
        return USBH_OK;
    }
    return   USBH_FAIL;
}
