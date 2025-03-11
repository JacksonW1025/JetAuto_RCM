/**
 * @file button.c
 * @author Lu Yongping (Lucas@hiwonder.com)
 * @brief 实现硬件无关的按键检测
 * @version 0.1
 * @date 2023-05-08
 *
 * @copyright Copyright (c) 2023
 *
 */
#include "button.h"


void button_task_handler(ButtonObjectTypeDef *self, uint32_t period)
{
    self->ticks_count += period;

    uint32_t pin = self->read_pin(self);
    if(pin != self->last_pin_raw)  { /* 前后连续的两次IO状态不同认为按钮状态还不稳定，保存新的IO状态然后返回 */
        self->last_pin_raw = pin;
        return;
    }
	
	/* 按钮状态没有改变, 即状态机状态不会发生转移, 直接返回 */
    if(self->last_pin_filtered == self->last_pin_raw && self->stage != BUTTON_STAGE_PRESS && self->stage != BUTTON_STAGE_LONGPRESS) { 
        return;
    }

    self->last_pin_filtered = self->last_pin_raw; /* 保存新的按钮状态 */
    switch(self->stage) {
        case BUTTON_STAGE_NORMAL: {
            if(self->last_pin_filtered) {
                self->event_callback(self, BUTTON_EVENT_PRESSED); /* 触发按键按下事件 */
                if(self->ticks_count < self->combin_th && self->combin_counter > 0) { /* 只有在连击计数不为零时连击才起作用 */
                    self->combin_counter += 1;
                    if(self->combin_counter == 2) {  /* 双击回调 */
                        self->event_callback(self, BUTTON_EVENT_DOUBLE_CLICK);
                    }
                    if(self->combin_counter == 3) {  /* 三连击回调 */
                        self->event_callback(self, BUTTON_EVENT_TRIPLE_CLICK);
                    }
                }
                self->ticks_count = 0;
                self->stage = BUTTON_STAGE_PRESS;
            } else {
                if(self->ticks_count > self->combin_th && self->combin_counter != 0) {
                    self->combin_counter = 0;
                    self->ticks_count = 0;
                }
            }
            break;
		}
        case BUTTON_STAGE_PRESS: {
            if(self->last_pin_filtered) {
                if(self->ticks_count > self->lp_th) { /* 超过长按触发时间 */
                    self->event_callback(self, BUTTON_EVENT_LONGPRESS); /* 触发长按事件 */
                    self->ticks_count = 0;
                    self->stage = BUTTON_STAGE_LONGPRESS; /* 状态转为长按 */
                }
            } else { /* 按钮松开 */
                self->event_callback(self, BUTTON_EVENT_RELEASE_FROM_SP); /* 触发短按松开事件 */
                self->event_callback(self, BUTTON_EVENT_CLICK);  /* 触发点击松开事件 */
                self->combin_counter = self->combin_counter == 0 ? 1 : self->combin_counter; /* 只有在连击计数不为零时连击才起作用 */
                self->stage = BUTTON_STAGE_NORMAL;
            }
            break;
		}
        case BUTTON_STAGE_LONGPRESS: {
            if(self->last_pin_filtered) {
                if(self->ticks_count > self->repeat_th)  {
                    self->event_callback(self, BUTTON_EVENT_LONGPRESS_REPEAT); /* 触发长按重复重复事件 */
                    self->ticks_count = 0; /* 重新计时下一次重复触发 */
                }
            } else { /* 按钮松开 */
                self->event_callback(self, BUTTON_EVENT_RELEASE_FROM_LP);  /* 触发长按松开事件 */
                self->combin_counter = 0;                /* 长按不可连击, 连击计数为0时连击计数不生效 */
                self->ticks_count = self->combin_th + 1; /* 长按不可连击, 让连击计时直接超时 */
                self->stage = BUTTON_STAGE_NORMAL;
            }
            break;
		}
    }
}


void button_register_callback(ButtonObjectTypeDef *self, ButtonEventCallbackFuncTypeDef callback)
{
    if(NULL == callback) {
        return;
    }
    self->event_callback = callback;
}

void button_defalut_event_callback(ButtonObjectTypeDef *self,  ButtonEventIDEnum event)
{
}

void button_object_init(ButtonObjectTypeDef *self)
{
    self->stage = BUTTON_STAGE_NORMAL;
    self->last_pin_raw = 0;
	self->last_pin_filtered = 0;
    self->combin_counter = 0;
    self->ticks_count = 0;

    /* config */
    self->combin_th = 400; 
    self->lp_th = 2000;  
    self->repeat_th = 500;
    self->event_callback = button_defalut_event_callback;
}

