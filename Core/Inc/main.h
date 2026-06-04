/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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
#define LCD_SCL_Pin GPIO_PIN_2
#define LCD_SCL_GPIO_Port GPIOE
#define LCD_CS_Pin GPIO_PIN_4
#define LCD_CS_GPIO_Port GPIOE
#define LCD_SI_Pin GPIO_PIN_6
#define LCD_SI_GPIO_Port GPIOE
#define GM_COUNT_HI_Pin GPIO_PIN_0
#define GM_COUNT_HI_GPIO_Port GPIOA
#define BUZZER_Pin GPIO_PIN_1
#define BUZZER_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_2
#define LED1_GPIO_Port GPIOA
#define LED2_Pin GPIO_PIN_3
#define LED2_GPIO_Port GPIOA
#define ETH_CS_Pin GPIO_PIN_4
#define ETH_CS_GPIO_Port GPIOA
#define ETH_SCLK_Pin GPIO_PIN_5
#define ETH_SCLK_GPIO_Port GPIOA
#define ETH_MISO_Pin GPIO_PIN_6
#define ETH_MISO_GPIO_Port GPIOA
#define ETH_MOSI_Pin GPIO_PIN_7
#define ETH_MOSI_GPIO_Port GPIOA
#define ETH_RST_Pin GPIO_PIN_4
#define ETH_RST_GPIO_Port GPIOC
#define ETH_INT_Pin GPIO_PIN_5
#define ETH_INT_GPIO_Port GPIOC
#define ETH_INT_EXTI_IRQn EXTI9_5_IRQn
#define GM_COUNT_LO_Pin GPIO_PIN_7
#define GM_COUNT_LO_GPIO_Port GPIOE
#define SW_DOWN_Pin GPIO_PIN_9
#define SW_DOWN_GPIO_Port GPIOE
#define SW_DOWN_EXTI_IRQn EXTI9_5_IRQn
#define SW_UP_Pin GPIO_PIN_10
#define SW_UP_GPIO_Port GPIOE
#define SW_UP_EXTI_IRQn EXTI15_10_IRQn
#define SW_SHIFT_Pin GPIO_PIN_11
#define SW_SHIFT_GPIO_Port GPIOE
#define SW_SHIFT_EXTI_IRQn EXTI15_10_IRQn
#define SW_SET_Pin GPIO_PIN_12
#define SW_SET_GPIO_Port GPIOE
#define SW_SET_EXTI_IRQn EXTI15_10_IRQn
#define LAMP_RED_Pin GPIO_PIN_3
#define LAMP_RED_GPIO_Port GPIOD
#define BT_ON_Pin GPIO_PIN_4
#define BT_ON_GPIO_Port GPIOD
#define EE_WP_Pin GPIO_PIN_5
#define EE_WP_GPIO_Port GPIOB
#define EE_SCL_Pin GPIO_PIN_6
#define EE_SCL_GPIO_Port GPIOB
#define EE_SDA_Pin GPIO_PIN_7
#define EE_SDA_GPIO_Port GPIOB
#define LCD_DC_Pin GPIO_PIN_9
#define LCD_DC_GPIO_Port GPIOB
#define LCD_RST_Pin GPIO_PIN_0
#define LCD_RST_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
