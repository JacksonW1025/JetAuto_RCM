#include "usbh_hid.h"
#include "usbh_hid_parser.h"
#include "usbh_hid_gamepad.h"

static USBH_StatusTypeDef USBH_HID_InterfaceInit(USBH_HandleTypeDef *phost);
static USBH_StatusTypeDef USBH_HID_InterfaceDeInit(USBH_HandleTypeDef *phost);
static USBH_StatusTypeDef USBH_HID_ClassRequest(USBH_HandleTypeDef *phost);
static USBH_StatusTypeDef USBH_HID_Process(USBH_HandleTypeDef *phost);
static USBH_StatusTypeDef USBH_HID_SOFProcess(USBH_HandleTypeDef *phost);
static void  USBH_HID_ParseHIDDesc(HID_DescTypeDef *desc, uint8_t *buf);

extern USBH_StatusTypeDef USBH_HID_GamepadInit(USBH_HandleTypeDef *phost);

USBH_ClassTypeDef  HIWONDER_HID_CLASS = {
    "HID",
    USB_HID_CLASS,
    USBH_HID_InterfaceInit,
    USBH_HID_InterfaceDeInit,
    USBH_HID_ClassRequest,
    USBH_HID_Process,
    USBH_HID_SOFProcess,
    NULL,
};


/** @defgroup USBH_HID_CORE_Private_Functions
* @{
*/


/**
  * @brief  USBH_HID_InterfaceInit
  *         The function init the HID class.
  * @param  phost: Host handle
  * @retval USBH Status
  */
static USBH_StatusTypeDef USBH_HID_InterfaceInit(USBH_HandleTypeDef *phost)
{
    USBH_StatusTypeDef status;
    HID_HandleTypeDef *HID_Handle;
    uint8_t max_ep;
    uint8_t num = 0U;
    uint8_t interface;

    interface = USBH_FindInterface(phost, phost->pActiveClass->ClassCode, 0U, 0xFFU);

    if ((interface == 0xFFU) || (interface >= USBH_MAX_NUM_INTERFACES)) { /* No Valid Interface */
        USBH_DbgLog("Cannot Find the interface for %s class.", phost->pActiveClass->Name);
        return USBH_FAIL;
    }

    status = USBH_SelectInterface(phost, interface);

    if (status != USBH_OK) {
        return USBH_FAIL;
    }

    phost->pActiveClass->pData = (HID_HandleTypeDef *)USBH_malloc(sizeof(HID_HandleTypeDef));
    HID_Handle = (HID_HandleTypeDef *) phost->pActiveClass->pData;

    if (HID_Handle == NULL) {
        USBH_DbgLog("Cannot allocate memory for HID Handle");
        return USBH_FAIL;
    }

    /* Initialize hid handler */
    USBH_memset(HID_Handle, 0, sizeof(HID_HandleTypeDef));

    HID_Handle->state = HID_ERROR;

    /*Decode Bootclass Protocol: Mouse or Keyboard*/
    if (phost->device.CfgDesc.Itf_Desc[interface].bInterfaceProtocol == HID_KEYBRD_BOOT_CODE) {
        USBH_UsrLog("KeyBoard device found!");
        HID_Handle->Init = USBH_HID_KeybdInit;
    } else if (phost->device.CfgDesc.Itf_Desc[interface].bInterfaceProtocol  == HID_MOUSE_BOOT_CODE) {
        USBH_UsrLog("Mouse device found!");
        HID_Handle->Init = USBH_HID_MouseInit;
    } else if((phost->device.DevDesc.idProduct == 0x0526 || phost->device.DevDesc.idProduct == 0x0575) && phost->device.DevDesc.idVendor == 0x2563) {
        USBH_UsrLog("USB Wireless Gamepad found!");
        HID_Handle->Init = USBH_HID_GamepadInit;

    } else {
        USBH_UsrLog("Protocol not supported.");
        return USBH_FAIL;
    }

    HID_Handle->state     = HID_INIT;
    HID_Handle->ctl_state = HID_REQ_INIT;
    HID_Handle->ep_addr   = phost->device.CfgDesc.Itf_Desc[interface].Ep_Desc[0].bEndpointAddress;
    HID_Handle->length    = phost->device.CfgDesc.Itf_Desc[interface].Ep_Desc[0].wMaxPacketSize;
    HID_Handle->poll      = phost->device.CfgDesc.Itf_Desc[interface].Ep_Desc[0].bInterval;


    if (HID_Handle->poll  < 50) {
        HID_Handle->poll = 50;
    }

    /* Check fo available number of endpoints */
    /* Find the number of EPs in the Interface Descriptor */
    /* Choose the lower number in order not to overrun the buffer allocated */
    max_ep = ((phost->device.CfgDesc.Itf_Desc[interface].bNumEndpoints <= USBH_MAX_NUM_ENDPOINTS) ?
              phost->device.CfgDesc.Itf_Desc[interface].bNumEndpoints : USBH_MAX_NUM_ENDPOINTS);


    /* Decode endpoint IN and OUT address from interface descriptor */
    for (num = 0U; num < max_ep; num++) {
        if (phost->device.CfgDesc.Itf_Desc[interface].Ep_Desc[num].bEndpointAddress & 0x80U) {
            HID_Handle->InEp = (phost->device.CfgDesc.Itf_Desc[interface].Ep_Desc[num].bEndpointAddress);
            HID_Handle->InPipe = USBH_AllocPipe(phost, HID_Handle->InEp);

            /* Open pipe for IN endpoint */
            USBH_OpenPipe(phost, HID_Handle->InPipe, HID_Handle->InEp, phost->device.address,
                          phost->device.speed, USB_EP_TYPE_INTR, HID_Handle->length);

            USBH_LL_SetToggle(phost, HID_Handle->InPipe, 0U);
        } else {
//      HID_Handle->OutEp = (phost->device.CfgDesc.Itf_Desc[interface].Ep_Desc[num].bEndpointAddress);
//      HID_Handle->OutPipe  = USBH_AllocPipe(phost, HID_Handle->OutEp);

//      /* Open pipe for OUT endpoint */
//      USBH_OpenPipe(phost, HID_Handle->OutPipe, HID_Handle->OutEp, phost->device.address,
//                    phost->device.speed, USB_EP_TYPE_INTR, HID_Handle->length);

//      USBH_LL_SetToggle(phost, HID_Handle->OutPipe, 0U);
        }
    }

    return USBH_OK;
}

/**
  * @brief  USBH_HID_InterfaceDeInit
  *         The function DeInit the Pipes used for the HID class.
  * @param  phost: Host handle
  * @retval USBH Status
  */
static USBH_StatusTypeDef USBH_HID_InterfaceDeInit(USBH_HandleTypeDef *phost)
{
    HID_HandleTypeDef *HID_Handle = (HID_HandleTypeDef *) phost->pActiveClass->pData;

    if (HID_Handle->InPipe != 0x00U) {
        USBH_ClosePipe(phost, HID_Handle->InPipe);
        USBH_FreePipe(phost, HID_Handle->InPipe);
        HID_Handle->InPipe = 0U;     /* Reset the pipe as Free */
    }

    if (HID_Handle->OutPipe != 0x00U) {
        USBH_ClosePipe(phost, HID_Handle->OutPipe);
        USBH_FreePipe(phost, HID_Handle->OutPipe);
        HID_Handle->OutPipe = 0U;     /* Reset the pipe as Free */
    }

    if (phost->pActiveClass->pData) {
        USBH_free(phost->pActiveClass->pData);
        phost->pActiveClass->pData = 0U;
    }

    return USBH_OK;
}

/**
  * @brief  USBH_HID_ClassRequest
  *         The function is responsible for handling Standard requests
  *         for HID class.
  * @param  phost: Host handle
  * @retval USBH Status
  */
static USBH_StatusTypeDef USBH_HID_ClassRequest(USBH_HandleTypeDef *phost)
{

    USBH_StatusTypeDef status         = USBH_BUSY;
    USBH_StatusTypeDef classReqStatus = USBH_BUSY;
    HID_HandleTypeDef *HID_Handle = (HID_HandleTypeDef *) phost->pActiveClass->pData;

    /* Switch HID state machine */
    switch (HID_Handle->ctl_state) {
        case HID_REQ_INIT:
        case HID_REQ_GET_HID_DESC:

            USBH_HID_ParseHIDDesc(&HID_Handle->HID_Desc, phost->device.CfgDesc_Raw);

            HID_Handle->ctl_state = HID_REQ_GET_REPORT_DESC;

            break;
        case HID_REQ_GET_REPORT_DESC:

            /* Get Report Desc */
            classReqStatus = USBH_HID_GetHIDReportDescriptor(phost, HID_Handle->HID_Desc.wItemLength);
            if (classReqStatus == USBH_OK) {
                /* The descriptor is available in phost->device.Data */
                HID_Handle->ctl_state = HID_REQ_SET_IDLE;

            } else if (classReqStatus == USBH_NOT_SUPPORTED) {
                USBH_ErrLog("Control error: HID: Device Get Report Descriptor request failed");
                status = USBH_FAIL;
            } else {
                /* .. */
            }

            break;

        case HID_REQ_SET_IDLE:
            /* set Idle */
            classReqStatus = USBH_HID_SetIdle(phost, 50U, 0U);
            if (classReqStatus == USBH_OK) {
                HID_Handle->ctl_state = HID_REQ_SET_PROTOCOL;
            } else {
                if (classReqStatus == USBH_NOT_SUPPORTED) {
                    HID_Handle->ctl_state = HID_REQ_SET_PROTOCOL;
                }
            }
            break;

        case HID_REQ_SET_PROTOCOL:
            /* set protocol */
            classReqStatus = USBH_HID_SetProtocol(phost, 0U);

            if (classReqStatus == USBH_OK) {
                HID_Handle->ctl_state = HID_REQ_IDLE;

                /* all requests performed*/
                phost->pUser(phost, HOST_USER_CLASS_ACTIVE);
                status = USBH_OK;
            } else if (classReqStatus == USBH_NOT_SUPPORTED) {
                USBH_ErrLog("Control error: HID: Device Set protocol request failed");
                status = USBH_FAIL;
            } else {
                /* .. */
            }
            break;

        case HID_REQ_IDLE:
        default:
            break;
    }

    return status;
}

/**
  * @brief  USBH_HID_Process
  *         The function is for managing state machine for HID data transfers
  * @param  phost: Host handle
  * @retval USBH Status
  */
static USBH_StatusTypeDef USBH_HID_Process(USBH_HandleTypeDef *phost)
{
    USBH_StatusTypeDef status = USBH_OK;
    HID_HandleTypeDef *HID_Handle = (HID_HandleTypeDef *) phost->pActiveClass->pData;
    uint32_t XferSize;
    USBH_URBStateTypeDef urb;
    switch (HID_Handle->state) {
        case HID_INIT:
            HID_Handle->Init(phost);
            HID_Handle->state = HID_IDLE;

#if (USBH_USE_OS == 1U)
            phost->os_msg = (uint32_t)USBH_URB_EVENT;
#if (osCMSIS < 0x20000U)
            (void)osMessagePut(phost->os_event, phost->os_msg, 0U);
#else
            (void)osMessageQueuePut(phost->os_event, &phost->os_msg, 0U, NULL);
#endif
#endif
            break;

        case HID_IDLE:
            status = USBH_HID_GetReport(phost, 0x01U, 0U, HID_Handle->pData, (uint8_t)HID_Handle->length);
            if (status == USBH_OK) {
                HID_Handle->state = HID_SYNC;
            } else if (status == USBH_BUSY) {
                HID_Handle->state = HID_IDLE;
                status = USBH_OK;
            } else if (status == USBH_NOT_SUPPORTED) {
                HID_Handle->state = HID_SYNC;
                status = USBH_OK;
            } else {
                HID_Handle->state = HID_ERROR;
                status = USBH_FAIL;
            }

#if (USBH_USE_OS == 1U)
            phost->os_msg = (uint32_t)USBH_URB_EVENT;
#if (osCMSIS < 0x20000U)
            (void)osMessagePut(phost->os_event, phost->os_msg, 0U);
#else
            (void)osMessageQueuePut(phost->os_event, &phost->os_msg, 0U, NULL);
#endif
#endif
            break;

        case HID_SYNC:
            /* Sync with start of Even Frame */
            if (phost->Timer & 1U) {
                HID_Handle->state = HID_GET_DATA;
            }

#if (USBH_USE_OS == 1U)
            phost->os_msg = (uint32_t)USBH_URB_EVENT;
#if (osCMSIS < 0x20000U)
            (void)osMessagePut(phost->os_event, phost->os_msg, 0U);
#else
            (void)osMessageQueuePut(phost->os_event, &phost->os_msg, 0U, NULL);
#endif
#endif
            break;

        case HID_GET_DATA:
            USBH_InterruptReceiveData(phost, HID_Handle->pData,
                                      (uint8_t)HID_Handle->length,
                                      HID_Handle->InPipe);

            HID_Handle->state = HID_POLL;
            HID_Handle->timer = phost->Timer;
            HID_Handle->DataReady = 0U;
            break;

        case HID_POLL:
            urb = USBH_LL_GetURBState(phost, HID_Handle->InPipe) ;
            if (urb == USBH_URB_DONE) {
                XferSize = USBH_LL_GetLastXferSize(phost, HID_Handle->InPipe);

                if ((HID_Handle->DataReady == 0U) && (XferSize != 0U)) {
                    USBH_HID_FifoWrite(&HID_Handle->fifo, HID_Handle->pData, HID_Handle->length);
                    HID_Handle->DataReady = 1U;
                    USBH_HID_EventCallback(phost);

#if (USBH_USE_OS == 1U)
                    phost->os_msg = (uint32_t)USBH_URB_EVENT;
#if (osCMSIS < 0x20000U)
                    (void)osMessagePut(phost->os_event, phost->os_msg, 0U);
#else
                    (void)osMessageQueuePut(phost->os_event, &phost->os_msg, 0U, NULL);
#endif
#endif
                }
            } else {
                /* IN Endpoint Stalled */
                if (USBH_LL_GetURBState(phost, HID_Handle->InPipe) == USBH_URB_STALL) {
                    /* Issue Clear Feature on interrupt IN endpoint */
                    if (USBH_ClrFeature(phost, HID_Handle->ep_addr) == USBH_OK) {
                        /* Change state to issue next IN token */
                        HID_Handle->state = HID_GET_DATA;
                    }
                }
            }
            break;

        default:
            break;
    }

    return status;
}

/**
  * @brief  USBH_HID_SOFProcess
  *         The function is for managing the SOF Process
  * @param  phost: Host handle
  * @retval USBH Status
  */
static USBH_StatusTypeDef USBH_HID_SOFProcess(USBH_HandleTypeDef *phost)
{
    HID_HandleTypeDef *HID_Handle = (HID_HandleTypeDef *) phost->pActiveClass->pData;

    if (HID_Handle->state == HID_POLL) {
        if ((phost->Timer - HID_Handle->timer) >= HID_Handle->poll) {
            HID_Handle->state = HID_GET_DATA;

#if (USBH_USE_OS == 1U)
            phost->os_msg = (uint32_t)USBH_URB_EVENT;
#if (osCMSIS < 0x20000U)
            (void)osMessagePut(phost->os_event, phost->os_msg, 0U);
#else
            (void)osMessageQueuePut(phost->os_event, &phost->os_msg, 0U, NULL);
#endif
#endif
        }
    }
    return USBH_OK;
}


/**
  * @brief  USBH_ParseHIDDesc
  *         This function Parse the HID descriptor
  * @param  desc: HID Descriptor
  * @param  buf: Buffer where the source descriptor is available
  * @retval None
  */
static void  USBH_HID_ParseHIDDesc(HID_DescTypeDef *desc, uint8_t *buf)
{
    USBH_DescHeader_t *pdesc = (USBH_DescHeader_t *)buf;
    uint16_t CfgDescLen;
    uint16_t ptr;

    CfgDescLen = LE16(buf + 2U);

    if (CfgDescLen > USB_CONFIGURATION_DESC_SIZE) {
        ptr = USB_LEN_CFG_DESC;

        while (ptr < CfgDescLen) {
            pdesc = USBH_GetNextDesc((uint8_t *)pdesc, &ptr);

            if (pdesc->bDescriptorType == USB_DESC_TYPE_HID) {
                desc->bLength = *(uint8_t *)((uint8_t *)pdesc + 0U);
                desc->bDescriptorType = *(uint8_t *)((uint8_t *)pdesc + 1U);
                desc->bcdHID = LE16((uint8_t *)pdesc + 2U);
                desc->bCountryCode = *(uint8_t *)((uint8_t *)pdesc + 4U);
                desc->bNumDescriptors = *(uint8_t *)((uint8_t *)pdesc + 5U);
                desc->bReportDescriptorType = *(uint8_t *)((uint8_t *)pdesc + 6U);
                desc->wItemLength = LE16((uint8_t *)pdesc + 7U);
                break;
            }
        }
    }
}



/**
  * @brief  HIWONDER_USBH_HID_GetDeviceType
  *         Return Device function.
  * @param  phost: Host handle
  * @retval HID function: HID_MOUSE / HID_KEYBOARD
  */
#define HID_GAMEPAD 0x00
HID_TypeTypeDef HIWONDER_USBH_HID_GetDeviceType(USBH_HandleTypeDef *phost)
{
    HID_TypeTypeDef   type = HID_UNKNOWN;
    uint8_t InterfaceProtocol;

    if (phost->gState == HOST_CLASS) {
        InterfaceProtocol = phost->device.CfgDesc.Itf_Desc[phost->device.current_interface].bInterfaceProtocol;
        if (InterfaceProtocol == HID_KEYBRD_BOOT_CODE) {
            type = HID_KEYBOARD;
        } else if (InterfaceProtocol == HID_MOUSE_BOOT_CODE) {
            type = HID_MOUSE;
        } else {
            if((phost->device.DevDesc.idProduct == 0x0526 || phost->device.DevDesc.idProduct == 0x0575) && phost->device.DevDesc.idVendor == 0x2563) {
                type = (HID_TypeTypeDef)HID_GAMEPAD;
            }
        }
    }
    return type;
}
