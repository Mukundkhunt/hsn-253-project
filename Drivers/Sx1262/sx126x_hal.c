#include "sx126x_hal.h"

sx126x_hal_status_t sx126x_hal_reset(const void* context) {
    const Radio_t* radio = (const Radio_t*) context;
    HAL_GPIO_WritePin(radio->reset_port, radio->reset_pin, GPIO_PIN_RESET);
    HAL_Delay(1);                 // at least 100 µs low pulse
    HAL_GPIO_WritePin(radio->reset_port, radio->reset_pin, GPIO_PIN_SET);
    HAL_Delay(5);                 // wait for the chip to reboot (~ few ms)
    return SX126X_HAL_STATUS_OK;
}

sx126x_hal_status_t sx126x_hal_wakeup(const void* context) {
    const Radio_t* radio = (const Radio_t*) context;
    // Make sure BUSY is low before toggling (if chip was in sleep, BUSY will be high until awake)
    if(HAL_GPIO_ReadPin(radio->busy_port, radio->busy_pin) == GPIO_PIN_SET) {
        // Optionally, wait or toggle reset if needed. In practice, toggling NSS is enough.
    }
    HAL_GPIO_WritePin(radio->nss_port, radio->nss_pin, GPIO_PIN_RESET);
    HAL_Delay(1);  // > 0.5 µs required
    HAL_GPIO_WritePin(radio->nss_port, radio->nss_pin, GPIO_PIN_SET);
    // After waking, wait until BUSY goes low indicating readiness
    while(HAL_GPIO_ReadPin(radio->busy_port, radio->busy_pin) == GPIO_PIN_SET) { /* spin */ }
    return SX126X_HAL_STATUS_OK;
}

sx126x_hal_status_t sx126x_hal_write(const void* context,
    const uint8_t* cmd, uint16_t cmd_length,
    const uint8_t* data, uint16_t data_length) {
const Radio_t* radio = (const Radio_t*) context;
// Wait until radio is not busy
while(HAL_GPIO_ReadPin(radio->busy_port, radio->busy_pin) == GPIO_PIN_SET) { }
HAL_GPIO_WritePin(radio->nss_port, radio->nss_pin, GPIO_PIN_RESET);
HAL_StatusTypeDef status = HAL_SPI_Transmit(radio->hspi, (uint8_t*)cmd, cmd_length, HAL_MAX_DELAY);
if(status == HAL_OK && data_length > 0) {
status = HAL_SPI_Transmit(radio->hspi, (uint8_t*)data, data_length, HAL_MAX_DELAY);
}
HAL_GPIO_WritePin(radio->nss_port, radio->nss_pin, GPIO_PIN_SET);
return (status == HAL_OK ? SX126X_HAL_STATUS_OK : SX126X_HAL_STATUS_ERROR);
}

sx126x_hal_status_t sx126x_hal_read(const void* context,
    const uint8_t* cmd, uint16_t cmd_length,
    uint8_t* data, uint16_t data_length) {
const Radio_t* radio = (const Radio_t*) context;
while(HAL_GPIO_ReadPin(radio->busy_port, radio->busy_pin) == GPIO_PIN_SET) { }
HAL_GPIO_WritePin(radio->nss_port, radio->nss_pin, GPIO_PIN_RESET);
HAL_StatusTypeDef status = HAL_SPI_Transmit(radio->hspi, (uint8_t*)cmd, cmd_length, HAL_MAX_DELAY);
if(status == HAL_OK) {
status = HAL_SPI_Receive(radio->hspi, data, data_length, HAL_MAX_DELAY);
}
HAL_GPIO_WritePin(radio->nss_port, radio->nss_pin, GPIO_PIN_SET);
return (status == HAL_OK ? SX126X_HAL_STATUS_OK : SX126X_HAL_STATUS_ERROR);
}


