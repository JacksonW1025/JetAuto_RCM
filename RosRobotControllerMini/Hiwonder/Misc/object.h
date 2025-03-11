#ifndef __OBJECT_H
#define __OBJECT_H

#include <stdint.h>

typedef enum {
	OBJECT_TYPE_ID_TYPE = 0x00,
	OBJECT_TYPE_ID_KEY_EVENT = 0x01,
	OBJECT_TYPE_ID_SBUS_STATUS,
	OBJECT_TYPE_ID_GAMEPAD_STATUS,
	OBJECT_TYPE_ID_BATTERY_VOLTAGE,
	OBJECT_TYPE_ID_LVGL_UPDATE,
}ObjectTypeIDEnum;


typedef union {
	uint8_t raw[32];
	struct {
		ObjectTypeIDEnum type_id;
		uint8_t data[];
	} structure;
	
} ObjectTypeDef;
#endif


