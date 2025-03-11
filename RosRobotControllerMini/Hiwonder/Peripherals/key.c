/// 实现硬件无关的按键检测
#include "global.h"
#include "key.h"

static void default_callback(KeyObjectTypeDef *key, KeyEventEnum event)
{
//    const char events[8][64] = {
//        "KEY_EVENT_PRESSED",
//        "KEY_EVENT_LONGPRESS",
//        "KEY_EVENT_LONGPRESS_REPEAT",
//        "KEY_EVENT_RELEASE_FROM_LP",
//        "KEY_EVENT_RELEASE_FROM_SP",
//        "KEY_EVENT_CLICK",
//        "KEY_EVENT_DOUBLE_CLICK",
//        "KEY_EVENT_TRIPLE_CLICK",
//    };
//    printf("event: %s\r\n", (char*)&events[event]);
}

void key_refresh(KeyObjectTypeDef *self)
{
    uint8_t level = self->read_pin();
    uint32_t current_tick = self->get_ticks();
    if(level == self->last_level) { // 两次原始引脚状态一致认为是按键状态稳定非误触发
        switch(self->state) {
        case KEY_STATE_NORMAL:
            if(level == self->level_press) {
                /* 触发按键按下事件 */
                if(NULL != self->event_callback) {
                    self->event_callback(self, KEY_EVENT_PRESSED);
                }
                if((current_tick - self->stamp) < self->combin_th) {
                    self->combin_counter += 1;
                    if(NULL != self->event_callback) {
                        if(self->combin_counter == 1) {
                            self->event_callback(self, KEY_EVENT_DOUBLE_CLICK);
                        }
                        if(self->combin_counter == 2) {
                            self->event_callback(self, KEY_EVENT_TRIPLE_CLICK);
                        }
                    }
                }
                self->stamp = current_tick; /* 记下这一次按下的时间 */
                self->state = KEY_STATE_PRESS;
            } else {
                if((current_tick - self->stamp) > self->combin_th && self->combin_counter != 0) {
                    self->combin_counter = 0;
                }
            }
            break;
        case KEY_STATE_PRESS:
            if(level != self->level_press) { // 如果按键没有按下，就是松开了
                /* 触发短按松开事件 */
                if(NULL != self->event_callback) {
                    self->event_callback(self, KEY_EVENT_RELEASE_FROM_SP);
                    self->event_callback(self, KEY_EVENT_CLICK);
                }
                self->state = KEY_STATE_NORMAL;
            } else {
                /* 超过长按触发时间，触发长按事件，状态转为长按*/
                if((current_tick - self->stamp) > self->lp_th) {
                    self->event_callback(self, KEY_EVENT_LONGPRESS);
                    self->state = KEY_STATE_LONGPRESS;
                    self->combin_counter = 0;
                    self->stamp = current_tick; /* 记下第一次长按触发时间 */
                }
            }
            break;
        case KEY_STATE_LONGPRESS:
            if(level != self->level_press) { // 如果按键没有按下，就是松开了
                /* 触发长按松开事件 */
                if(NULL != self->event_callback) {
                    self->event_callback(self, KEY_EVENT_RELEASE_FROM_LP);
                    self->stamp = 0;
                }
                self->state = KEY_STATE_NORMAL;
            } else {
                if((current_tick - self->stamp) > self->repeat_th)  {
                    self->event_callback(self, KEY_EVENT_LONGPRESS_REPEAT);
                    self->stamp = current_tick; /* 记下这次重触发时间 */
                }
            }
            break;
        default:
            while(1) {
                printf("KEY REFRESH ERROR!!!\r\n");
            }
        }
    }
    self->last_level = level;
}


void key_obj_init(KeyObjectTypeDef *self)
{
    self->state = KEY_STATE_NORMAL;
    self->last_level = !self->level_press;
    self->refresh = key_refresh;
    self->event_callback = default_callback;
}

