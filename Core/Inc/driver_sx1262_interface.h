#ifndef DRIVER_SX1262_INTERFACE_H
#define DRIVER_SX1262_INTERFACE_H

#include <stdint.h>
#include <stdarg.h>
#include <string.h>

#ifdef __cplusplus
extern "C"{
#endif

uint8_t sx1262_interface_spi_init(void);
uint8_t sx1262_interface_spi_deinit(void);
uint8_t sx1262_interface_spi_write_read(uint8_t *tx_buf, uint32_t tx_len, uint8_t *rx_buf, uint32_t rx_len);

uint8_t sx1262_interface_reset_gpio_init(void);
uint8_t sx1262_interface_reset_gpio_deinit(void);
uint8_t sx1262_interface_reset_gpio_write(uint8_t value);

uint8_t sx1262_interface_busy_gpio_init(void);
uint8_t sx1262_interface_busy_gpio_deinit(void);
uint8_t sx1262_interface_busy_gpio_read(uint8_t *value);

void sx1262_interface_delay_ms(uint32_t ms);
void sx1262_interface_debug_print(const char *const fmt, ...);

#ifdef __cplusplus
}
#endif

#endif // DRIVER_SX1262_INTERFACE_H
