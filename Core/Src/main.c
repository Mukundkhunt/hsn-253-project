/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "SX1262.h"
#include "bme68x.h"
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "stm32g0xx_hal.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// SX1262 LoRa HAT Pin definitions (updated as per your wiring!)
#define LORA_NSS_PORT        GPIOB
#define LORA_NSS_PIN         GPIO_PIN_0
#define LORA_RST_PORT        GPIOB
#define LORA_RST_PIN         GPIO_PIN_1
#define LORA_BUSY_PORT       GPIOB
#define LORA_BUSY_PIN        GPIO_PIN_2
#define LORA_DIO1_PORT       GPIOA
#define LORA_DIO1_PIN        GPIO_PIN_4

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
#define BME68X_I2C_ADDR 0x77
struct bme68x_dev gas_sensor;
struct bme68x_conf conf;
struct bme68x_heatr_conf heatr_conf;
struct bme68x_data data[3];
uint8_t n_fields = 0;
int8_t rslt;

static int8_t user_i2c_read(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr);
static int8_t user_i2c_write(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr);
static void user_delay_us(uint32_t period, void *intf_ptr);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void user_delay_us(uint32_t period, void *intf_ptr)
{
    uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) * 1000 < period);
}

static int8_t user_i2c_read(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr)
{
    I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef *)intf_ptr;
    if (HAL_I2C_Mem_Read(hi2c, BME68X_I2C_ADDR << 1, reg_addr, I2C_MEMADD_SIZE_8BIT, data, len, HAL_MAX_DELAY) != HAL_OK) {
        char err[] = "I2C read failed\r\n";
        HAL_UART_Transmit(&huart2, (uint8_t*)err, strlen(err), HAL_MAX_DELAY);
        return -1;
    }
    return 0;
}

static int8_t user_i2c_write(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr)
{
    I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef *)intf_ptr;
    if (HAL_I2C_Mem_Write(hi2c, BME68X_I2C_ADDR << 1, reg_addr, I2C_MEMADD_SIZE_8BIT, (uint8_t*)data, len, HAL_MAX_DELAY) != HAL_OK)
        return -1;
    return 0;
}

void log_debug(const char *msg) {
    bool isDebug = true;
    char buffer[200];
    if(isDebug)
    {
        snprintf(buffer, sizeof(buffer), "[DEBUG] %s\r\n", msg);
        HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
    }
}

void log_info(const char *msg) {
    char buffer[200];
    snprintf(buffer, sizeof(buffer), "%s\r\n", msg);
    HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
}

// --- SX1262 TX Done Callback via DIO1 Interrupt ---
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == LORA_DIO1_PIN) {
        log_debug("SX1262 TX complete (DIO1 interrupt detected)");
        // You can set a flag here for your application logic if needed
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  log_debug("System Initialization Started");
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  // --- SX1262 LoRa HAT Initialization ---
  SX1262_Get_st()->Busy_Pin  = LORA_BUSY_PIN;
  SX1262_Get_st()->Busy_Port = LORA_BUSY_PORT;
  SX1262_Get_st()->NSS_Pin   = LORA_NSS_PIN;
  SX1262_Get_st()->NSS_Port  = LORA_NSS_PORT;
  SX1262_Get_st()->Reset_Pin = LORA_RST_PIN;
  SX1262_Get_st()->Reset_Port= LORA_RST_PORT;
  SX1262_Get_st()->SPI = hspi1;

  log_debug("Initializing SX1262...");
  SX1262_Init();

  uint8_t status = SX1262_getstatus();
  char msg[64];
  snprintf(msg, sizeof(msg), "SX1262 status: 0x%02X\r\n", status);
  log_debug(msg);

  // --- Test Transmit ---
  uint8_t hello[] = "HELLO";
  SX1262_Transmit(hello, sizeof(hello));
  log_debug("SX1262 transmit called (no status)");

  log_debug("Initializing BME68x...");

  // Set up BME68x sensor interface
  gas_sensor.intf = BME68X_I2C_INTF;
  gas_sensor.read = user_i2c_read;
  gas_sensor.write = user_i2c_write;
  gas_sensor.delay_us = user_delay_us;
  gas_sensor.intf_ptr = &hi2c1;

  rslt = bme68x_init(&gas_sensor);
  if (rslt != BME68X_OK) {
      log_debug("Sensor initialization failed. Halting.");
      while (1);
  }

  uint8_t chip_id = 0;
  rslt = bme68x_get_regs(BME68X_REG_CHIP_ID, &chip_id, 1, &gas_sensor);
  if (rslt == BME68X_OK) {
      char id_msg[64];
      snprintf(id_msg, sizeof(id_msg), "BME68x Chip ID: 0x%02X (Expected: 0x%02X)", chip_id, BME68X_CHIP_ID);
      log_debug(id_msg);
  } else {
      log_debug("Failed to read Chip ID");
  }

  log_debug("Configuring BME68x sensor...");

  // Oversampling and filter config
  conf.os_hum = BME68X_OS_2X;
  conf.os_temp = BME68X_OS_8X;
  conf.os_pres = BME68X_OS_4X;
  conf.filter = BME68X_FILTER_SIZE_3;

  if (bme68x_set_conf(&conf, &gas_sensor) != BME68X_OK)
      log_debug("Failed to apply sensor config");

  // Heater configuration
  heatr_conf.enable = BME68X_ENABLE;
  heatr_conf.heatr_temp = 300;
  heatr_conf.heatr_dur = 100;

  if (bme68x_set_heatr_conf(BME68X_FORCED_MODE, &heatr_conf, &gas_sensor) != BME68X_OK)
      log_debug("Failed to apply heater config");

  log_debug("BME68x configuration complete.");

  const char *intro_msg =
    "Enter a command in the format:\r\n"
    "\r\n=== BME68x Sensor Terminal ===\r\n"
    "  <duration> <mode>\r\n"
    "Where:\r\n"
    "  duration - Number of seconds to print (e.g., 5)\r\n"
    "  mode     - One of the following:\r\n"
    "             temp  - Show Temperature in Celsius\r\n"
    "             humi  - Show Humidity in %%\r\n"
    "             press - Show Pressure in hPa\r\n"
    "Examples:\r\n"
    "  10 temp   - Show temperature every second for 10 seconds\r\n"
    "  5 humi    - Show humidity for 5 seconds\r\n"
    "\r\nArithmetic Operations:\r\n"
    "  add a b [c ...]  - Add numbers\r\n"
    "  sub a b [c ...]  - Subtract numbers\r\n"
    "  mul a b [c ...]  - Multiply numbers\r\n"
    "  div a b [c ...]  - Divide numbers (left to right)\r\n"
    "  e.g., add 2 3 4 \r\n"
    "\r\nType 'help' to show this message again.\r\n"
    "===============================\r\n";
   HAL_UART_Transmit(&huart2, (uint8_t *)intro_msg, strlen(intro_msg), HAL_MAX_DELAY);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      char rx_buffer[32] = {0};
      uint8_t idx = 0;
      char ch;

      while (1) {
          HAL_UART_Receive(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
          HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY); // echo

          if (ch == '\r' || ch == '\n') {
              rx_buffer[idx] = '\0';
              break;
          } else if (idx < sizeof(rx_buffer) - 1) {
              rx_buffer[idx++] = ch;
          }
      }

      // Skip if empty input
      if (strlen(rx_buffer) == 0) {
          continue;
      }

      int duration = 0;
      char mode[16] = {0};

      if (strcmp(rx_buffer, "help") == 0) {
          HAL_UART_Transmit(&huart2, (uint8_t *)intro_msg, strlen(intro_msg), HAL_MAX_DELAY);
          continue;
      }

      if (strcmp(rx_buffer, "help") == 0) {
        HAL_UART_Transmit(&huart2, (uint8_t *)intro_msg, strlen(intro_msg), HAL_MAX_DELAY);
        continue;
    }
    
    char copy[64];
    strncpy(copy, rx_buffer, sizeof(copy));
    char *token = strtok(copy, " ");
    if (!token) continue;
    
    char op[4];
    strncpy(op, token, sizeof(op));
    
    if (strcmp(op, "add") == 0 || strcmp(op, "sub") == 0 || strcmp(op, "mul") == 0 || strcmp(op, "div") == 0) {
        int result = 0, value;
        bool first = true;
        while ((token = strtok(NULL, " ")) != NULL) {
            if (sscanf(token, "%d", &value) != 1) {
                HAL_UART_Transmit(&huart2, (uint8_t *)"Invalid number\r\n", 17, HAL_MAX_DELAY);
                break;
            }
    
            if (first) {
                result = value;
                first = false;
                continue;
            }
    
            if (strcmp(op, "add") == 0) result += value;
            else if (strcmp(op, "sub") == 0) result -= value;
            else if (strcmp(op, "mul") == 0) result *= value;
            else if (strcmp(op, "div") == 0) {
                if (value == 0) {
                    HAL_UART_Transmit(&huart2, (uint8_t *)"Divide by 0\r\n", 14, HAL_MAX_DELAY);
                }
                result /= value;
            }
        }
    
        char result_msg[64];
        snprintf(result_msg, sizeof(result_msg), "Result: %d\r\n", result);
        HAL_UART_Transmit(&huart2, (uint8_t *)result_msg, strlen(result_msg), HAL_MAX_DELAY);
        continue;
    }

      if (sscanf(rx_buffer, "%d %15s", &duration, mode) != 2) {
          const char *msg = "Invalid format. Use: <duration> <mode>\r\n";
          HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
          continue;
      }

      char echo[64];
      snprintf(echo, sizeof(echo), "\r\n> Duration: %d sec | Mode: %s\r\n", duration, mode);
      HAL_UART_Transmit(&huart2, (uint8_t *)echo, strlen(echo), HAL_MAX_DELAY);

      bool dataReceived = false;
      for (int t = 0; t < duration; t++) {
          rslt = bme68x_set_op_mode(BME68X_FORCED_MODE, &gas_sensor);
          if (rslt != BME68X_OK) {
              HAL_UART_Transmit(&huart2, (uint8_t *)"Set op mode failed\r\n", 21, HAL_MAX_DELAY);
              break;
          }

          uint32_t meas_dur = bme68x_get_meas_dur(BME68X_FORCED_MODE, &conf, &gas_sensor);
          gas_sensor.delay_us(meas_dur + 10000, gas_sensor.intf_ptr);

          rslt = bme68x_get_data(BME68X_FORCED_MODE, data, &n_fields, &gas_sensor);

          if (rslt == BME68X_OK && n_fields > 0) {
              for (uint8_t i = 0; i < n_fields; i++) {
                  dataReceived = true;
                  char msg[64];
                  if (strcmp(mode, "temp") == 0) {
                      snprintf(msg, sizeof(msg), "Temperature: %.2f °C\r\n", data[i].temperature);
                  } else if (strcmp(mode, "humi") == 0) {
                      snprintf(msg, sizeof(msg), "Humidity: %.2f %%\r\n", data[i].humidity);
                  } else if (strcmp(mode, "press") == 0) {
                      snprintf(msg, sizeof(msg), "Pressure: %.2f hPa\r\n", data[i].pressure);
                  } else {
                        char err[64];
                        snprintf(err, sizeof(err), "Unknown mode: %s\r\n", mode);
                        HAL_UART_Transmit(&huart2, (uint8_t *)err, strlen(err), HAL_MAX_DELAY);
                        t = duration;
                  }
                  // Transmit over UART (debug/log)
                  HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);

                  // Transmit over LoRa
//                  SX1262_Transmit((uint8_t *)msg, strlen(msg));
              }
          } else if(dataReceived) {
              HAL_UART_Transmit(&huart2, (uint8_t *)"Sensor read error\r\n", 20, HAL_MAX_DELAY);
          }

          HAL_Delay(1000); // wait 1 second
      }
  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00503D58;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2, GPIO_PIN_RESET);

  // Configure DIO1_LORA (PA4) as EXTI (Interrupt) Input
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // Configure NSS_LORA (PB0), RESET (PB1), BUSY (PB2) as output
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
