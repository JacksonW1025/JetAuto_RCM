#ifndef  __U8G2_PORTING_H
#define  __U8G2_PORTING_H

#include "u8g2.h"

#define MAX_LEN         128
#define OLED_ADDRESS    0x78
#define OLED_CMD        0x00
#define OLED_DATA       0x40
#define OLED_WRITE_CMD	0x00
#define OLED_WRITE_DATA	0x40

extern u8g2_t *u8g2;

void u8g2_init(void);


#endif //__U8G2_PORTING_H
