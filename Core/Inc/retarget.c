#include "stm32g0xx_hal.h"  // or the appropriate header for your MCU

extern UART_HandleTypeDef huart2;  // USART2 handle from CubeMX generated code

int __io_putchar(int ch) {
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}
