/**
 * @file lwmem_porting.c
 * @author Lu Yongping (Lucas@hiwonder.com)
 * @brief lwmem 接口移植及内存空间定义
 * @version 0.1
 * @date 2023-06-02
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "lwmem.h"

uint8_t lwmem_ram[1024 * 38];

lwmem_region_t lwmem_regions[] = {
    { (void*)0x10000000, 1024 * 64 }, /* 顺序不能变， ccmram地址比ram地址低，一定要放前面否则无法正常初始化内存*/
    { (void*)lwmem_ram,  1024 * 38 },
    { NULL, 0}
};
