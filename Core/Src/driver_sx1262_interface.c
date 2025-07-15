/* driver_sx1262_interface.c - STM32 HAL-based implementation */

#include "driver_sx1262_interface.h"
#include "stm32g0xx_hal.h"
#include <stdio.h>
#include <stdarg.h>

extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart2;

#define NSS_PORT GPIOA
#define NSS_PIN  GPIO_PIN_4
#define RESET_PORT GPIOB
#define RESET_PIN  GPIO_PIN_1
#define BUSY_PORT  GPIOB
#define BUSY_PIN   GPIO_PIN_0

void sx1262_interface_debug_print(const char *const fmt, ...)
{
    char buffer[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);
    HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
}

void sx1262_interface_delay_ms(uint32_t ms)
{
    HAL_Delay(ms);
}

uint8_t sx1262_interface_spi_init(void)
{
    return 0; // already initialized by CubeMX
}

uint8_t sx1262_interface_spi_deinit(void)
{
    return 0; // not needed
}

uint8_t sx1262_interface_spi_write_read(uint8_t *tx_buf, uint32_t tx_len, uint8_t *rx_buf, uint32_t rx_len)
{
    char msg[64];
    snprintf(msg, sizeof(msg), "SPI TX: 0x%02X", tx_buf[0]);
    sx1262_interface_debug_print(msg);
    return (HAL_SPI_TransmitReceive(&hspi1, tx_buf, rx_buf, tx_len, HAL_MAX_DELAY) == HAL_OK) ? 0 : 1;
}

uint8_t sx1262_interface_reset_gpio_init(void)
{
    return 0;
}

uint8_t sx1262_interface_reset_gpio_deinit(void)
{
    return 0;
}

uint8_t sx1262_interface_reset_gpio_write(uint8_t value)
{
    HAL_GPIO_WritePin(RESET_PORT, RESET_PIN, value ? GPIO_PIN_SET : GPIO_PIN_RESET);
    char msg[64];
    snprintf(msg, sizeof(msg), "RESET pin set to %d", value);
    sx1262_interface_debug_print(msg);
    return 0;
}

uint8_t sx1262_interface_busy_gpio_init(void)
{
    return 0;
}

uint8_t sx1262_interface_busy_gpio_deinit(void)
{
    return 0;
}

uint8_t sx1262_interface_busy_gpio_read(uint8_t *value)
{
    *value = HAL_GPIO_ReadPin(BUSY_PORT, BUSY_PIN) == GPIO_PIN_SET ? 1 : 0;
    char msg[64];
    snprintf(msg, sizeof(msg), "BUSY pin read: %s", *value ? "HIGH" : "LOW");
    sx1262_interface_debug_print(msg);
    return 0;
}
