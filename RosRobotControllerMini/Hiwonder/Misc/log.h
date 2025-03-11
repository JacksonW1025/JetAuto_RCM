/**
 * @file log.h
 * @author Lu Yongping (Lucas@hiwonder.com)
 * @brief LOG 相关定义和声明
 * @version 0.1
 * @date 2023-05-12
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef __LOG_H__
#define __LOG_H__

#include <stdio.h>
#include <stdint.h>

#define LOG_LEVEL_DEFAULT LOG_LEVEL_DEBUG
#define LOG_PRINT_FILE_NAME 1
#define LOG_ENABLE 1
#define LOG_USE_RTT 0



enum log_level{ 
	LOG_LEVEL_DEBUG,
	LOG_LEVEL_INFO,
	LOG_LEVEL_WARN,
	LOG_LEVEL_ERROR,
	LOG_LEVEL_NONE,
};

// Define LOG_PRINT_FILE_NAME to print file name and line no in the log line.
#if LOG_PRINT_FILE_NAME
#define log_ap(fmt, ...)                                                    \
	"(%s:%d) " fmt, strrchr("/" __FILE__, '/') + 1, __LINE__, __VA_ARGS__
#else
#define log_ap(fmt, ...) fmt, __VA_ARGS__
#endif

#if LOG_USE_RTT
#define LOG_INIT()	do{log_level = LOG_LEVEL_DEFAULT; SEGGER_RTT_Init(); }while(0)
#else
#define LOG_INIT()	do{log_level = LOG_LEVEL_DEFAULT; }while(0)
#endif

#define LOG_PRINTF(...) 	printf(__VA_ARGS__)

#if LOG_ENABLE
//conditional output
#define LOG_DEBUG(...)              {if(log_level <= LOG_LEVEL_DEBUG) LOG_PRINTF(__VA_ARGS__);}
#define LOG_INFO(...)               {if(log_level <= LOG_LEVEL_INFO) LOG_PRINTF(__VA_ARGS__);}
#define LOG_WARN(...)               {if(log_level <= LOG_LEVEL_WARN) LOG_PRINTF(__VA_ARGS__);}
#define LOG_ERROR(...)              {if(log_level <= LOG_LEVEL_ERROR) LOG_PRINTF(__VA_ARGS__);}

#else
#define LOG_DEBUG(...)    do{;}while(0)
#define LOG_INFO(...)     do{;}while(0)
#define LOG_WARN(...)     do{;}while(0)
#define LOG_ERROR(...)    do{;}while(0)

#endif


extern enum log_level log_level;
#endif
