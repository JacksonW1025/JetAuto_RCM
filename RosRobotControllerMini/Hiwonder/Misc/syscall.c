/**
 * @file syscall.c
 * @author Lu Yongping (Lucas@hiwonder.com)
 * @brief 标准库桩函数的重定向
 * @version 0.1
 * @date 2023-05-12
 *
 * @copyright Copyright (c) 2023
 *
 */


#include <stdio.h>
#include "usart.h"
//#include "SEGGER_RTT.h"1	

//#if __ARMCC_VERSION >= 6000000
//    __asm(".global __use_no_semihosting");
//#elif __ARMCC_VERSION >= 5000000
//    #pragma import(__use_no_semihosting)
//#else
//    #error Unsupported compiler
//#endif


//char *_sys_command_string(char *cmd, int len){
//    return NULL;
//}


//void _sys_exit(int return_code) {
//    while (1)
//        ;
//}

//void ttywrch (int ch) {
//}

//int stdin_getchar (void)
//{
//    return 0;
//}

//int stderr_putchar (int ch)
//{
//  //  HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, 0xFFFF);
//    SEGGER_RTT_Write(0, &ch, 1);
//    return ch;
//}

//int stdout_putchar (int ch)
//{
//  //  HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, 0xFFFF);
//    SEGGER_RTT_Write(0, &ch, 1);
//    return (ch);
//}


