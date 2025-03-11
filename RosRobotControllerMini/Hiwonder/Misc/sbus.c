#include "sbus.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>

int sbus_decode_frame(uint8_t *buf, SBusStatusObjectTypeDef *status)
{
    if(SBUS_FRAME_STARTBYTE != buf[0]) { /* 帧头不符合协议要求 */
        return -1;
    }
    if(SBUS_FRAME_ENDBYTE != buf[24]) { /* 帧尾不符合协议要求 */
        return -2;
    }
	status->type_id = OBJECT_TYPE_ID_GAMEPAD_STATUS;
    status->channels[ 0] = ((uint16_t)buf[1] >> 0 | ((uint16_t)buf[2] << 8 )) & 0x07FF;
    status->channels[ 1] = ((uint16_t)buf[2] >> 3 | ((uint16_t)buf[3] << 5 )) & 0x07FF;
    status->channels[ 2] = ((uint16_t)buf[3] >> 6 | ((uint16_t)buf[4] << 2 )  | (uint16_t)buf[5] << 10 ) & 0x07FF;
    status->channels[ 3] = ((uint16_t)buf[5] >> 1 | ((uint16_t)buf[6] << 7 )) & 0x07FF;
    status->channels[ 4] = ((uint16_t)buf[6] >> 4 | ((uint16_t)buf[7] << 4 )) & 0x07FF;
    status->channels[ 5] = ((uint16_t)buf[7] >> 7 | ((uint16_t)buf[8] << 1 )  | (uint16_t)buf[9] <<  9 ) & 0x07FF;
    status->channels[ 6] = ((uint16_t)buf[9] >> 2 | ((uint16_t)buf[10] << 6 )) & 0x07FF;
    status->channels[ 7] = ((uint16_t)buf[10] >> 5 | ((uint16_t)buf[11] << 3 )) & 0x07FF;

    status->channels[ 8] = ((uint16_t)buf[12] << 0 | ((uint16_t)buf[13] << 8 )) & 0x07FF;
    status->channels[ 9] = ((uint16_t)buf[13] >> 3 | ((uint16_t)buf[14] << 5 )) & 0x07FF;
    status->channels[10] = ((uint16_t)buf[14] >> 6 | ((uint16_t)buf[15] << 2 )  | (uint16_t)buf[16] << 10 ) & 0x07FF;
    status->channels[11] = ((uint16_t)buf[16] >> 1 | ((uint16_t)buf[17] << 7 )) & 0x07FF;
    status->channels[12] = ((uint16_t)buf[17] >> 4 | ((uint16_t)buf[18] << 4 )) & 0x07FF;
    status->channels[13] = ((uint16_t)buf[18] >> 7 | ((uint16_t)buf[19] << 1 )  | (uint16_t)buf[20] <<  9 ) & 0x07FF;
    status->channels[14] = ((uint16_t)buf[20] >> 2 | ((uint16_t)buf[21] << 6 )) & 0x07FF;
    status->channels[15] = ((uint16_t)buf[21] >> 5 | ((uint16_t)buf[22] << 3 )) & 0x07FF;

    status->ch17 = (bool)(buf[23] & 0x80u); // bit7 --- 数字通道17的数据
    status->ch18 = (bool)(buf[23] & 0x40u); // bit6 --- 数字通道18的数据
    status->signal_loss = (bool)(buf[23] & 0x20u); // bit5 --- 丢帧信息
    status->fail_safe = (bool)(buf[23] & 0x10u); // bit4 --- 失效保护使能
	
	/* 这部分代码是针对性的，针对HotRC遥控接收器, 该遥控器在序号丢失时不会置位 signal_loss, 前四个channel的值变为固定特定值，这里加入判读 */
	/* 此处加入对上一帧数据的判断，看数据是否从其他数据突变为固定值，但是这样还要加上对开机默认状态的处理且刚好摇杆的四个值都对上的概率已经很小就不多加判断了 */
	if(status->channels[0] == 992 && status->channels[1] == 992 && status->channels[2] == 32 && status->channels[3] == 992) {
		status->signal_loss = true;
	}
	/* 特定代码结束 */
    return 0;
}


void sbus_print_status(SBusStatusObjectTypeDef *status)
{
    for(int i = 0; i < 16; ++i) {
        printf("%04d  ", status->channels[i]);
    }
    printf(status->ch17 ? "X " : "_ ");
    printf(status->ch18 ? "X " : "_ ");
    printf("  sl: ");
    printf(status->signal_loss ? "true" : "false");
    printf("  fs: ");
    printf(status->fail_safe ? "true" : "false");
    printf("\r\n");
}

