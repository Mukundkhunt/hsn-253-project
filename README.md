# Embedded Smart Systems Project

![STM32+BME680](https://img.shields.io/badge/Platform-STM32-blue) 
![Sensor-BME680](https://img.shields.io/badge/Sensor-BME680-green)

A project demonstrating how to interface the BME680 environmental sensor with STM32 microcontrollers using STM32CubeIDE.

## Features
- Measures temperature, humidity, and barometric pressure
- I2C interface implementation
- UART output for sensor data
- Error handling and sensor validation

## Hardware Requirements
| Component               | Specification          |
|-------------------------|------------------------|
| Microcontroller         | STM32 (e.g., STM32G071RB) |
| Sensor Module           | BME680 (CJMCU-680)     |
| Interface               | I2C                    |
| Power Supply            | 3.3V                   |

## Pin Configuration
### I2C Connection
| BME680 Pin | STM32 Pin  | Function   |
|------------|------------|------------|
| VCC        | 3.3V       | Power      |
| GND        | GND        | Ground     |
| SDA        | PB9        | I2C1_SDA   |
| SCL        | PB8        | I2C1_SCL   |
| CS         | 3.3V       | I2C Enable |

### UART Connection (Debug Output)
| STM32 Pin | Function    |
|-----------|-------------|
| PA2       | USART2_TX   |
| PA3       | USART2_RX   |

## Software Requirements
- STM32CubeIDE
- STM32 HAL Libraries
- BME680 Driver Library
- Serial Terminal (PuTTY, CoolTerm, etc.)

## Getting Started

### 1. Clone the Repository
```bash
git clone https://github.com/yourusername/stm32-bme680.git
