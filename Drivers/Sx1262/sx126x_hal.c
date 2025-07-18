#include "sx126x_hal.h"
#include "sx126x_stm32_hal.h"
#include "debug_log.h"

#define BUSY_TIMEOUT_MS 100

static sx126x_hal_status_t wait_on_busy(const Radio_t* radio) {
    uint32_t start = HAL_GetTick();
    while (HAL_GPIO_ReadPin(radio->busy_port, radio->busy_pin) == GPIO_PIN_SET) {
        if (HAL_GetTick() - start > BUSY_TIMEOUT_MS) {
            log_debug("ERROR: BUSY pin timeout (>100ms)");
            return SX126X_HAL_STATUS_ERROR;
        }
    }
    return SX126X_HAL_STATUS_OK;
}

sx126x_hal_status_t sx126x_hal_reset(const void* context) {
    const Radio_t* radio = (const Radio_t*) context;
    log_debug("SX1262 Reset");
    HAL_GPIO_WritePin(radio->reset_port, radio->reset_pin, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(radio->reset_port, radio->reset_pin, GPIO_PIN_SET);
    HAL_Delay(10);
    return SX126X_HAL_STATUS_OK;
}

sx126x_hal_status_t sx126x_hal_wakeup(const void* context) {
    const Radio_t* radio = (const Radio_t*) context;
    log_debug("SX1262 Wakeup");

    HAL_GPIO_WritePin(radio->nss_port, radio->nss_pin, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(radio->nss_port, radio->nss_pin, GPIO_PIN_SET);

    return wait_on_busy(radio);
}

sx126x_hal_status_t sx126x_hal_write(const void* context,
    const uint8_t* cmd, uint16_t cmd_length,
    const uint8_t* data, uint16_t data_length) {

    const Radio_t* radio = (const Radio_t*) context;
    log_debug("SX1262 Write start");

    if (wait_on_busy(radio) != SX126X_HAL_STATUS_OK)
        return SX126X_HAL_STATUS_ERROR;

    HAL_GPIO_WritePin(radio->nss_port, radio->nss_pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef status = HAL_SPI_Transmit(radio->hspi, (uint8_t*)cmd, cmd_length, HAL_MAX_DELAY);
    if (status == HAL_OK && data_length > 0) {
        status = HAL_SPI_Transmit(radio->hspi, (uint8_t*)data, data_length, HAL_MAX_DELAY);
    }
    HAL_GPIO_WritePin(radio->nss_port, radio->nss_pin, GPIO_PIN_SET);

    log_debug("SX1262 Write end");
    return (status == HAL_OK ? SX126X_HAL_STATUS_OK : SX126X_HAL_STATUS_ERROR);
}

sx126x_hal_status_t sx126x_hal_read(const void* context,
    const uint8_t* cmd, uint16_t cmd_length,
    uint8_t* data, uint16_t data_length) {

    const Radio_t* radio = (const Radio_t*) context;
    log_debug("SX1262 Read start");

    if (wait_on_busy(radio) != SX126X_HAL_STATUS_OK)
        return SX126X_HAL_STATUS_ERROR;

    HAL_GPIO_WritePin(radio->nss_port, radio->nss_pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef status = HAL_SPI_Transmit(radio->hspi, (uint8_t*)cmd, cmd_length, HAL_MAX_DELAY);

    // 🧠 Add this before trying to receive:
    if (wait_on_busy(radio) != SX126X_HAL_STATUS_OK) {
        log_debug("BUSY timeout before SPI receive");
        HAL_GPIO_WritePin(radio->nss_port, radio->nss_pin, GPIO_PIN_SET);
        return SX126X_HAL_STATUS_ERROR;
    }

    if (status == HAL_OK) {
        status = HAL_SPI_Receive(radio->hspi, data, data_length, HAL_MAX_DELAY);
    }

    HAL_GPIO_WritePin(radio->nss_port, radio->nss_pin, GPIO_PIN_SET);
    log_debug("SX1262 Read end");

    return (status == HAL_OK ? SX126X_HAL_STATUS_OK : SX126X_HAL_STATUS_ERROR);
}
