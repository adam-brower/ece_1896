/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    main.h
  * @author  MCD Application Team
  * @brief   Header for main.c module
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2019-2021 STMicroelectronics.
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
#include "app_conf.h"
#include "app_entry.h"
#include "app_common.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32wbxx_nucleo.h"
#include "stm32wbxx_hal_gpio.h"
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
#define PULSE_CHRG_SEL_Pin GPIO_PIN_13
#define PULSE_CHRG_SEL_GPIO_Port GPIOC
#define SD__CARD_DETECT_Pin GPIO_PIN_8
#define SD__CARD_DETECT_GPIO_Port GPIOB
#define SD_CS_Pin GPIO_PIN_9
#define SD_CS_GPIO_Port GPIOB
#define ADC_FLOW_SIG_IN_Pin GPIO_PIN_0
#define ADC_FLOW_SIG_IN_GPIO_Port GPIOA
#define ADC_TEMP_SIG_IN_Pin GPIO_PIN_1
#define ADC_TEMP_SIG_IN_GPIO_Port GPIOA
#define GYRO_EXTI_Pin GPIO_PIN_2
#define GYRO_EXTI_GPIO_Port GPIOA
#define TFT_LITE_Pin GPIO_PIN_3
#define TFT_LITE_GPIO_Port GPIOA
#define DIS_CS_Pin GPIO_PIN_4
#define DIS_CS_GPIO_Port GPIOA
#define ENCODER_CH1_Pin GPIO_PIN_8
#define ENCODER_CH1_GPIO_Port GPIOA
#define ENCODER_CH2_Pin GPIO_PIN_9
#define ENCODER_CH2_GPIO_Port GPIOA
#define TFT_RST_Pin GPIO_PIN_4
#define TFT_RST_GPIO_Port GPIOC
#define TFT_WAIT_Pin GPIO_PIN_5
#define TFT_WAIT_GPIO_Port GPIOC
#define LED_BLUE_Pin GPIO_PIN_2
#define LED_BLUE_GPIO_Port GPIOB
#define LED_YELLOW_Pin GPIO_PIN_10
#define LED_YELLOW_GPIO_Port GPIOB
#define GAIN_3dB_Pin GPIO_PIN_0
#define GAIN_3dB_GPIO_Port GPIOB
#define Gain_6dB_Pin GPIO_PIN_1
#define Gain_6dB_GPIO_Port GPIOB
#define Gain_12dB_Pin GPIO_PIN_4
#define Gain_12dB_GPIO_Port GPIOE
#define GAIN_15dB_Pin GPIO_PIN_14
#define GAIN_15dB_GPIO_Port GPIOB
#define I2S_AMP_SD_Pin GPIO_PIN_6
#define I2S_AMP_SD_GPIO_Port GPIOC
#define PULSE_CHRG_EN_Pin GPIO_PIN_10
#define PULSE_CHRG_EN_GPIO_Port GPIOA
#define EXTI_ROT_SWT_Pin GPIO_PIN_15
#define EXTI_ROT_SWT_GPIO_Port GPIOA
#define EXTI_BUTTON_1_Pin GPIO_PIN_10
#define EXTI_BUTTON_1_GPIO_Port GPIOC
#define EXTI_BUTTON_2_Pin GPIO_PIN_11
#define EXTI_BUTTON_2_GPIO_Port GPIOC
#define EXTI_BUTTON_3_Pin GPIO_PIN_12
#define EXTI_BUTTON_3_GPIO_Port GPIOC
#define EXTI_PWR_SWT_STAT_Pin GPIO_PIN_0
#define EXTI_PWR_SWT_STAT_GPIO_Port GPIOD
#define EXTI_PRESSURE_ALARM_Pin GPIO_PIN_4
#define EXTI_PRESSURE_ALARM_GPIO_Port GPIOB
#define BOOST_EN_Pin GPIO_PIN_5
#define BOOST_EN_GPIO_Port GPIOB
#define GAUGE_IO_Pin GPIO_PIN_6
#define GAUGE_IO_GPIO_Port GPIOB
#define BATT_DETECT_Pin GPIO_PIN_7
#define BATT_DETECT_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define SD_SPI_HANDLE hspi2
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
