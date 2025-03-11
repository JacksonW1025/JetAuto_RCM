/**
  ******************************************************************************
  * @file    usbh_hid.h
  * @author  MCD Application Team
  * @brief   This file contains all the prototypes for the usbh_hid.c
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2015 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                      www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Define to prevent recursive  ----------------------------------------------*/
#ifndef __USBH_HIWONDER_HID_H
#define __USBH_HIWONDER_HID_H


/* Includes ------------------------------------------------------------------*/
#include "usbh_core.h"
#include "usbh_hid.h"

extern USBH_ClassTypeDef  HIWONDER_HID_CLASS;
#define USBH_HIWONDER_HID_CLASS    &HIWONDER_HID_CLASS

#define HID_GAMEPAD 0x00


HID_TypeTypeDef HIWONDER_USBH_HID_GetDeviceType(USBH_HandleTypeDef *phost)


#endif
