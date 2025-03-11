/**
 * @file sbus.h
 * @author Lu Yongping (Lucas@hiwonder.com)
 * @brief SBUS 协议的类型、函数声明
 * @version 0.1
 * @date 2023-05-18
 *
 * @copyright Copyright (c) 2023
 *
 */


#ifndef __SBUS_H_
#define __SBUS_H_

#include <stdint.h>
#include <stdbool.h>
#include <object.h>

#define SBUS_FRAME_LENGTH 25U
#define SBUS_FRAME_STARTBYTE 0x0FU
#define SBUS_FRAME_ENDBYTE 0x00U


typedef struct {
	ObjectTypeIDEnum type_id;
    int16_t channels[16];
		bool ch17;
		bool ch18;
    bool signal_loss;
    bool fail_safe;
}SBusStatusObjectTypeDef;

int sbus_decode_frame(uint8_t *buf, SBusStatusObjectTypeDef *status);
void sbus_print_status(SBusStatusObjectTypeDef *status);

#endif

