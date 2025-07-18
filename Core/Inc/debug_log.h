// debug_log.h
#ifndef DEBUG_LOG_H
#define DEBUG_LOG_H

#include "stm32g0xx_hal.h"
#include <string.h>
#include <stdio.h>

// Externally declare the UART handler
extern UART_HandleTypeDef huart2;

static inline void log_debug(const char *msg) {
    char buffer[200];
    snprintf(buffer, sizeof(buffer), "[DEBUG] %s\r\n", msg);
    HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
}

#endif  // DEBUG_LOG_H
