/// 实现硬件无关的按键检测


#ifndef __KEY_H_
#define __KEY_H_

#include <stdio.h>
#include <stdint.h>
#include <object.h>

typedef enum {
	KEY_STATE_NORMAL,
	KEY_STATE_PRESS,
	KEY_STATE_LONGPRESS,
	KEY_STATE_REPEAT,
} KeyStateEnum;

typedef enum {
	KEY_EVENT_PRESSED = 0x01,
	KEY_EVENT_LONGPRESS = 0x02,
	KEY_EVENT_LONGPRESS_REPEAT = 0x04,
	KEY_EVENT_RELEASE_FROM_LP = 0x08,
	KEY_EVENT_RELEASE_FROM_SP = 0x10,
	KEY_EVENT_CLICK = 0x20,
	KEY_EVENT_DOUBLE_CLICK = 0x40,
	KEY_EVENT_TRIPLE_CLICK = 0x80,
}KeyEventEnum;

typedef struct{
	ObjectTypeDef base;
	int key_id;
	KeyEventEnum event;
}KeyEventObjectTypeDef;

typedef struct _KeyObjectTypeDef KeyObjectTypeDef;
struct _KeyObjectTypeDef {
	KeyStateEnum state;  // 当前按键状态机状态

	uint8_t last_level; // 上次直接读到的引脚电平
	uint32_t stamp; // 
	int combin_counter;

	/* config */
	int combin_th; /**< 连按最大间隔 毫秒*/
	int lp_th;     /**< 长按阈值 毫秒 */
	int repeat_th; /**< 长按重触发间隔 毫秒 */

	// 内部暴露接口
	void (*refresh)(KeyObjectTypeDef *self); //刷新按键状态
	uint32_t (*get_ticks)(void); // 获取当前毫秒时间戳
	
	// 移植接口
	char *name;  //按钮名称
	uint8_t level_press; // 按钮按下时 IO 电平
	uint8_t (*read_pin)(void); //读取按键状态
	void (*event_callback)(KeyObjectTypeDef *key, KeyEventEnum event);
};


void key_obj_init(KeyObjectTypeDef *self);
#endif

