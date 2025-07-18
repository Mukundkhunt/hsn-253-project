// sx126x_stm32_hal.h
#ifndef SX126X_STM32_HAL_H
#define SX126X_STM32_HAL_H

#include "stm32g0xx_hal.h"  // HAL types (SPI_HandleTypeDef, GPIO_TypeDef, etc.)

typedef struct {
    SPI_HandleTypeDef* hspi;

    GPIO_TypeDef* nss_port;
    uint16_t      nss_pin;

    GPIO_TypeDef* reset_port;
    uint16_t      reset_pin;

    GPIO_TypeDef* busy_port;
    uint16_t      busy_pin;

    GPIO_TypeDef* dio1_port;
    uint16_t      dio1_pin;
} Radio_t;

#endif // SX126X_STM32_HAL_H
