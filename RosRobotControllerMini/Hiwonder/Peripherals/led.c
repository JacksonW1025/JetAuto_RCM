/**
 * @file led.c
 * @author Lu Yongping (Lucas@hiwonder.com)
 * @brief 实现硬件无关的LED灯闪烁控制
 * @version 0.1
 * @date 2023-05-13
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "led.h"

void led_task_handler(LEDObjectTypeDef *self, uint32_t period)
{
    /* 尝试从队列中取的新的控制数据， 如果成功取出则重置状态机重新开始一个控制循环 */
    if(self->get_ctrl_block(self, &self->ctrl_structure) == 0) {
        self->stage = LED_STAGE_START_NEW_CYCLE;
    }
    /* 状态机处理 */
    switch(self->stage) {
        case LED_STAGE_START_NEW_CYCLE: {
            if(self->ctrl_structure.ticks_on > 0) {
                self->set_pin(self, 1);
                if(self->ctrl_structure.ticks_off > 0) { /* 熄灭时间不为 0 即为 闪烁否则为长亮 */
                    self->ticks_count = 0;
                    self->stage = LED_STAGE_WATTING_OFF; /* 等待 LED 灯亮起时间结束 */
                }else{
					self->stage = LED_STAGE_IDLE; /* 长亮， 转入空闲 */
				}
            } else { /* 只要亮起时间为 0 即为长灭 */
                self->set_pin(self, 0);
				self->stage = LED_STAGE_IDLE; /* 长灭， 转入空闲 */
            }
            break;
        }
        case LED_STAGE_WATTING_OFF: {
            self->ticks_count += period;
            if(self->ticks_count >= self->ctrl_structure.ticks_on) { /* LED 亮起时间结束 */
                self->set_pin(self, 0);
                self->stage = LED_STAGE_WATTING_PERIOD_END;
            }
            break;
        }
        case LED_STAGE_WATTING_PERIOD_END: { /* 等待周期结束 */
            self->ticks_count += period;
            if(self->ticks_count >= (self->ctrl_structure.ticks_off + self->ctrl_structure.ticks_on)) {
				 self->ticks_count -= (self->ctrl_structure.ticks_off + self->ctrl_structure.ticks_on);
                if(self->ctrl_structure.repeat == 1) { /* 剩余重复次数为1时就可以结束此次控制任务 */
                    self->set_pin(self, 0);
                    self->stage = LED_STAGE_IDLE;  /* 重复次数用完， 转入空闲 */
                } else {
                    self->set_pin(self, 1);
                    self->ctrl_structure.repeat = self->ctrl_structure.repeat == 0 ? 0 : self->ctrl_structure.repeat - 1;
                    self->stage = LED_STAGE_WATTING_OFF;
                }
            }
            break;
        }
        case LED_STAGE_IDLE: {
            break;
        }
        default:
            break;
    }
}

int led_on(LEDObjectTypeDef *self)
{
    LEDCtrlTypeDef ctrl_structure = {
        .ticks_on = 1,
        .ticks_off = 0,
        .repeat = 0,
    };
    return self->put_ctrl_block(self, &ctrl_structure);
}

int led_off(LEDObjectTypeDef *self)
{
    LEDCtrlTypeDef ctrl_structure = {
        .ticks_on = 0,
        .ticks_off = 0,
        .repeat = 0,
    };
    return self->put_ctrl_block(self, &ctrl_structure);
}

int led_flash(LEDObjectTypeDef *self, uint32_t ticks_on, uint32_t ticks_off, uint32_t repeat)
{
    LEDCtrlTypeDef ctrl_structure = {
        .ticks_on = ticks_on,
        .ticks_off = ticks_off,
        .repeat = repeat,
    };
    return self->put_ctrl_block(self, &ctrl_structure);
}


void led_object_init(LEDObjectTypeDef *self)
{
    self->stage = LED_STAGE_IDLE;
}


