#ifndef __RETARGET_H__
#define __RETARGET_H__

#include "stm32g0xx_hal.h"
#include <stdio.h>

void RetargetInit(UART_HandleTypeDef *huart);

#endif // __RETARGET_H__
