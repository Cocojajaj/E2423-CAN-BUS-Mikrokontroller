/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MCU_SPI3_CS1_Pin GPIO_PIN_13
#define MCU_SPI3_CS1_GPIO_Port GPIOC
#define MCU_SPI3_CS2_Pin GPIO_PIN_14
#define MCU_SPI3_CS2_GPIO_Port GPIOC
#define MCU_SPI3_CS3_CAN_STBY_Pin GPIO_PIN_15
#define MCU_SPI3_CS3_CAN_STBY_GPIO_Port GPIOC
#define A_IN_01_Pin GPIO_PIN_0
#define A_IN_01_GPIO_Port GPIOC
#define A_IN_02_Pin GPIO_PIN_1
#define A_IN_02_GPIO_Port GPIOC
#define A_IN_03_Pin GPIO_PIN_2
#define A_IN_03_GPIO_Port GPIOC
#define A_IN_04_Pin GPIO_PIN_3
#define A_IN_04_GPIO_Port GPIOC
#define A_IN_05_Pin GPIO_PIN_0
#define A_IN_05_GPIO_Port GPIOA
#define A_IN_06_Pin GPIO_PIN_1
#define A_IN_06_GPIO_Port GPIOA
#define A_IN_07_Pin GPIO_PIN_2
#define A_IN_07_GPIO_Port GPIOA
#define A_IN_08_Pin GPIO_PIN_3
#define A_IN_08_GPIO_Port GPIOA
#define A_IN_09_DAC_1_Pin GPIO_PIN_4
#define A_IN_09_DAC_1_GPIO_Port GPIOA
#define D_IN_01_Pin GPIO_PIN_5
#define D_IN_01_GPIO_Port GPIOA
#define A_IN_10_Pin GPIO_PIN_6
#define A_IN_10_GPIO_Port GPIOA
#define D_IN_02_Pin GPIO_PIN_7
#define D_IN_02_GPIO_Port GPIOA
#define D_IN_03_Pin GPIO_PIN_4
#define D_IN_03_GPIO_Port GPIOC
#define D_IN_04_Pin GPIO_PIN_5
#define D_IN_04_GPIO_Port GPIOC
#define A_IN_11_Pin GPIO_PIN_0
#define A_IN_11_GPIO_Port GPIOB
#define A_IN_12_Pin GPIO_PIN_1
#define A_IN_12_GPIO_Port GPIOB
#define D_IN_05_Pin GPIO_PIN_2
#define D_IN_05_GPIO_Port GPIOB
#define D_IN_06_Pin GPIO_PIN_10
#define D_IN_06_GPIO_Port GPIOB
#define D_IN_07_Pin GPIO_PIN_11
#define D_IN_07_GPIO_Port GPIOB
#define D_IN_08_Pin GPIO_PIN_12
#define D_IN_08_GPIO_Port GPIOB
#define D_OUT_01_Pin GPIO_PIN_13
#define D_OUT_01_GPIO_Port GPIOB
#define D_OUT_02_Pin GPIO_PIN_14
#define D_OUT_02_GPIO_Port GPIOB
#define D_OUT_03_Pin GPIO_PIN_15
#define D_OUT_03_GPIO_Port GPIOB
#define D_OUT_04_Pin GPIO_PIN_6
#define D_OUT_04_GPIO_Port GPIOC
#define D_OUT_05_Pin GPIO_PIN_7
#define D_OUT_05_GPIO_Port GPIOC
#define D_OUT_06_Pin GPIO_PIN_8
#define D_OUT_06_GPIO_Port GPIOC
#define D_OUT_07_Pin GPIO_PIN_9
#define D_OUT_07_GPIO_Port GPIOC
#define D_OUT_08_Pin GPIO_PIN_8
#define D_OUT_08_GPIO_Port GPIOA
#define D_SW_OUT_01_Pin GPIO_PIN_9
#define D_SW_OUT_01_GPIO_Port GPIOA
#define D_SW_OUT_02_Pin GPIO_PIN_10
#define D_SW_OUT_02_GPIO_Port GPIOA
#define D_SW_OUT_03_Pin GPIO_PIN_11
#define D_SW_OUT_03_GPIO_Port GPIOA
#define D_SW_OUT_04_Pin GPIO_PIN_12
#define D_SW_OUT_04_GPIO_Port GPIOA
#define MCU_SPI3_CS_ADC_Pin GPIO_PIN_2
#define MCU_SPI3_CS_ADC_GPIO_Port GPIOD
#define MCU_EXT_CLK_ADC_Pin GPIO_PIN_5
#define MCU_EXT_CLK_ADC_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
