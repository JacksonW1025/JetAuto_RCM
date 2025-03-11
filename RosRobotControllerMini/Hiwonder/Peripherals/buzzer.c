/**
 * @file buzzer.c
 * @author Lu Yongping (Lucas@hiwonder.com)
 * @brief 实现硬件&系统无关的蜂鸣器控制
 * @version 0.1
 * @date 2023-05-18
 *
 * @copyright Copyright (c) 2023
 *
 */
#include "buzzer.h"

void buzzer_task_handler(BuzzerObjectTypeDef *self, uint32_t period)
{
    /* 尝试从队列中取的新的控制数据， 如果成功取出则重置状态机重新开始一个控制循环 */
    if(self->get_ctrl_block(self, &self->ctrl_structure) == 0) {
        self->stage = BUZZER_STAGE_START_NEW_CYCLE;
    }
    /* 状态机处理 */
    switch(self->stage) {
        case BUZZER_STAGE_START_NEW_CYCLE: {
            if(self->ctrl_structure.ticks_on > 0 && self->ctrl_structure.freq > 0) {
                self->set_pwm(self, self->ctrl_structure.freq); /* 鸣响蜂鸣器 */
                if(self->ctrl_structure.ticks_off > 0) {/* 静音时间不为 0 即为 嘀嘀响 否则就是长鸣 */
                    self->ticks_count = 0;
                    self->stage = BUZZER_STAGE_WATTING_OFF; /* 等到鸣响时间结束 */
                }else{
					self->stage = BUZZER_STAGE_IDLE; /* 长鸣，转入空闲 */
				}
            } else { /* 只要鸣响时间为 0 即为静音 */
                self->set_pwm(self, 0);
				self->stage = BUZZER_STAGE_IDLE;  /* 长静音，转入空闲 */
            }
            break;
        }
        case BUZZER_STAGE_WATTING_OFF: {
            self->ticks_count += period;
            if(self->ticks_count >= self->ctrl_structure.ticks_on) { /* 鸣响时间结束 */
                self->set_pwm(self, 0);
                self->stage = BUZZER_STAGE_WATTING_PERIOD_END;
            }
            break;
        }
        case BUZZER_STAGE_WATTING_PERIOD_END: { /* 等待周期结束 */
            self->ticks_count += period;
            if(self->ticks_count >= (self->ctrl_structure.ticks_off + self->ctrl_structure.ticks_on)) {
                self->ticks_count -= (self->ctrl_structure.ticks_off + self->ctrl_structure.ticks_on);
                if(self->ctrl_structure.repeat == 1) { /* 剩余重复次数为1时就可以结束此次控制任务 */
                    self->set_pwm(self, 0);
                    self->stage = BUZZER_STAGE_IDLE;
                } else {
                    self->set_pwm(self, self->ctrl_structure.freq);
                    self->ctrl_structure.repeat = self->ctrl_structure.repeat == 0 ? 0 : self->ctrl_structure.repeat - 1;
                    self->stage = BUZZER_STAGE_WATTING_OFF;
                }
            }
            break;
        }
        case BUZZER_STAGE_IDLE: {
            break;
        }
        default:
            break;
    }
}


int buzzer_on( BuzzerObjectTypeDef *self, uint32_t freq)
{
    BuzzerCtrlTypeDef ctrl_structure = {
        .freq = freq,
        .ticks_on = 1,  
        .ticks_off = 0, /* 静音时间为0 长鸣 */
        .repeat = 0,
    };
    return self->put_ctrl_block(self, &ctrl_structure);
}

int buzzer_off(BuzzerObjectTypeDef *self)
{
    BuzzerCtrlTypeDef ctrl_structure = {
        .freq = 0,
        .ticks_on = 0, /* 鸣响时间为0就是静音 */
        .ticks_off = 0,
        .repeat = 0,
    };
    return self->put_ctrl_block(self, &ctrl_structure);
}

int buzzer_didi(BuzzerObjectTypeDef *self, uint32_t freq, uint32_t ticks_on, uint32_t ticks_off, uint32_t repeat)
{
    BuzzerCtrlTypeDef ctrl_structure = {
        .freq = freq,
        .ticks_on = ticks_on,
        .ticks_off = ticks_off,
        .repeat = repeat,
    };
    return self->put_ctrl_block(self, &ctrl_structure);
}


void buzzer_object_init(BuzzerObjectTypeDef *self)
{
    self->stage = BUZZER_STAGE_IDLE;
}

