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
#include "stm32l4xx_hal.h"

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
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOH
#define LIMIT_WRIST_PITCH_MIN_Pin GPIO_PIN_0
#define LIMIT_WRIST_PITCH_MIN_GPIO_Port GPIOC
#define LIMIT_WRIST_PITCH_MAX_Pin GPIO_PIN_1
#define LIMIT_WRIST_PITCH_MAX_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define SMPS_EN_Pin GPIO_PIN_4
#define SMPS_EN_GPIO_Port GPIOA
#define SMPS_V1_Pin GPIO_PIN_5
#define SMPS_V1_GPIO_Port GPIOA
#define SMPS_PG_Pin GPIO_PIN_6
#define SMPS_PG_GPIO_Port GPIOA
#define SMPS_SW_Pin GPIO_PIN_7
#define SMPS_SW_GPIO_Port GPIOA
#define PWM3_Pin GPIO_PIN_11
#define PWM3_GPIO_Port GPIOB
#define CS1_Pin GPIO_PIN_7
#define CS1_GPIO_Port GPIOC
#define CS2_Pin GPIO_PIN_8
#define CS2_GPIO_Port GPIOC
#define CS3_Pin GPIO_PIN_9
#define CS3_GPIO_Port GPIOC
#define PWM1_Pin GPIO_PIN_9
#define PWM1_GPIO_Port GPIOA
#define PWM2_Pin GPIO_PIN_10
#define PWM2_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define LIMIT_WRIST_PITCH_MAXA15_Pin GPIO_PIN_15
#define LIMIT_WRIST_PITCH_MAXA15_GPIO_Port GPIOA
#define LIMIT_WRIST_PITCH_MAXA15_EXTI_IRQn EXTI15_10_IRQn
#define PWM1B3_Pin GPIO_PIN_3
#define PWM1B3_GPIO_Port GPIOB
#define CYTRON_DIR_1_Pin GPIO_PIN_5
#define CYTRON_DIR_1_GPIO_Port GPIOB
#define SPI2_CS_D8_Pin GPIO_PIN_6
#define SPI2_CS_D8_GPIO_Port GPIOB
#define DIR2_Pin GPIO_PIN_7
#define DIR2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
