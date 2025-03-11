/**
 * @file log.c
 * @author Lu Yongping (Lucas@hiwonder.com)
 * @brief LOG实现
 * @version 0.1
 * @date 2023-05-12
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "log.h"
#include <stdint.h>
#include "global.h"

enum log_level log_level = LOG_LEVEL_DEBUG;

