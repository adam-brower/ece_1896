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
#include "stm32wbxx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BATT_DETECT_Pin GPIO_PIN_1
#define BATT_DETECT_GPIO_Port GPIOA
#define GAUGE_IO_Pin GPIO_PIN_2
#define GAUGE_IO_GPIO_Port GPIOA
#define GYRO_INT_1_Pin GPIO_PIN_3
#define GYRO_INT_1_GPIO_Port GPIOA
#define LED_RED_Pin GPIO_PIN_4
#define LED_RED_GPIO_Port GPIOA
#define LED_GREEN_Pin GPIO_PIN_5
#define LED_GREEN_GPIO_Port GPIOA
#define LED_BLUE_Pin GPIO_PIN_6
#define LED_BLUE_GPIO_Port GPIOA
#define IR_OUT_1_Pin GPIO_PIN_7
#define IR_OUT_1_GPIO_Port GPIOA
#define IR_OUT_2_Pin GPIO_PIN_8
#define IR_OUT_2_GPIO_Port GPIOA
#define TFT_DC_Pin GPIO_PIN_0
#define TFT_DC_GPIO_Port GPIOB
#define TFT_RST_Pin GPIO_PIN_1
#define TFT_RST_GPIO_Port GPIOB
#define TFT_BL_Pin GPIO_PIN_4
#define TFT_BL_GPIO_Port GPIOE
#define USB_DETECT_Pin GPIO_PIN_6
#define USB_DETECT_GPIO_Port GPIOC
#define VIBE_EN_PWM_Pin GPIO_PIN_10
#define VIBE_EN_PWM_GPIO_Port GPIOA
#define BMS_PG_Pin GPIO_PIN_15
#define BMS_PG_GPIO_Port GPIOA
#define BMS_CHRG_COMPLETE_Pin GPIO_PIN_10
#define BMS_CHRG_COMPLETE_GPIO_Port GPIOC
#define BMS_LOWBATT_Pin GPIO_PIN_11
#define BMS_LOWBATT_GPIO_Port GPIOC
#define BMS_nTE_Pin GPIO_PIN_12
#define BMS_nTE_GPIO_Port GPIOC
#define BUTTON_2_Pin GPIO_PIN_0
#define BUTTON_2_GPIO_Port GPIOD
#define BUTTON_1_Pin GPIO_PIN_1
#define BUTTON_1_GPIO_Port GPIOD
#define PULSE_CHRG_CURRSEL_Pin GPIO_PIN_4
#define PULSE_CHRG_CURRSEL_GPIO_Port GPIOB
#define BMS_CE_Pin GPIO_PIN_5
#define BMS_CE_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
