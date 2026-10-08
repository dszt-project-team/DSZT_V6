/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define IMU_IST_DRDY_Pin GPIO_PIN_3
#define IMU_IST_DRDY_GPIO_Port GPIOE
#define IMU_IST_RST_Pin GPIO_PIN_2
#define IMU_IST_RST_GPIO_Port GPIOE
#define SPARE_UART8_TX_Pin GPIO_PIN_1
#define SPARE_UART8_TX_GPIO_Port GPIOE
#define SPARE_UART8_RX_Pin GPIO_PIN_0
#define SPARE_UART8_RX_GPIO_Port GPIOE
#define IMU_MPU_DRDY_Pin GPIO_PIN_8
#define IMU_MPU_DRDY_GPIO_Port GPIOB
#define IMU_HEATER_PWM_Pin GPIO_PIN_5
#define IMU_HEATER_PWM_GPIO_Port GPIOB
#define SPARE_USART6_TX_Pin GPIO_PIN_14
#define SPARE_USART6_TX_GPIO_Port GPIOG
#define SPARE_USART2_RX_Pin GPIO_PIN_6
#define SPARE_USART2_RX_GPIO_Port GPIOD
#define STEER_LEFT_DIR_Pin GPIO_PIN_7
#define STEER_LEFT_DIR_GPIO_Port GPIOI
#define STEER_LEFT_FG_Pin GPIO_PIN_6
#define STEER_LEFT_FG_GPIO_Port GPIOI
#define STEER_LEFT_PWM_Pin GPIO_PIN_5
#define STEER_LEFT_PWM_GPIO_Port GPIOI
#define SPARE_USART6_RX_Pin GPIO_PIN_9
#define SPARE_USART6_RX_GPIO_Port GPIOG
#define SPARE_USART2_TX_Pin GPIO_PIN_5
#define SPARE_USART2_TX_GPIO_Port GPIOD
#define STEER_LEFT_BRAKE_Pin GPIO_PIN_2
#define STEER_LEFT_BRAKE_GPIO_Port GPIOI
#define POWER1_CTRL_Pin GPIO_PIN_2
#define POWER1_CTRL_GPIO_Port GPIOH
#define WS2812_LEFT_DATA_Pin GPIO_PIN_8
#define WS2812_LEFT_DATA_GPIO_Port GPIOA
#define POWER2_CTRL_Pin GPIO_PIN_3
#define POWER2_CTRL_GPIO_Port GPIOH
#define POWER3_CTRL_Pin GPIO_PIN_4
#define POWER3_CTRL_GPIO_Port GPIOH
#define POWER4_CTRL_Pin GPIO_PIN_5
#define POWER4_CTRL_GPIO_Port GPIOH
#define IMU_CS_Pin GPIO_PIN_6
#define IMU_CS_GPIO_Port GPIOF
#define MT6826S_RIGHT_PWM_Pin GPIO_PIN_11
#define MT6826S_RIGHT_PWM_GPIO_Port GPIOH
#define STEER_RIGHT_BRAKE_Pin GPIO_PIN_15
#define STEER_RIGHT_BRAKE_GPIO_Port GPIOD
#define METAL_EVENT_IN_Pin GPIO_PIN_2
#define METAL_EVENT_IN_GPIO_Port GPIOC
#define METAL_EN_OUT_Pin GPIO_PIN_3
#define METAL_EN_OUT_GPIO_Port GPIOC
#define BUZZER_PWM_Pin GPIO_PIN_6
#define BUZZER_PWM_GPIO_Port GPIOH
#define STEER_RIGHT_DIR_Pin GPIO_PIN_14
#define STEER_RIGHT_DIR_GPIO_Port GPIOD
#define STEER_RIGHT_FG_Pin GPIO_PIN_13
#define STEER_RIGHT_FG_GPIO_Port GPIOD
#define MT6826S_LEFT_PWM_Pin GPIO_PIN_0
#define MT6826S_LEFT_PWM_GPIO_Port GPIOA
#define STEER_RIGHT_PWM_Pin GPIO_PIN_12
#define STEER_RIGHT_PWM_GPIO_Port GPIOD
#define SPARE_UART7_TX_Pin GPIO_PIN_8
#define SPARE_UART7_TX_GPIO_Port GPIOE
#define LED_RED_Pin GPIO_PIN_11
#define LED_RED_GPIO_Port GPIOE
#define WS2812_RIGHT_DATA_Pin GPIO_PIN_14
#define WS2812_RIGHT_DATA_GPIO_Port GPIOE
#define OID_RS485_RX_Pin GPIO_PIN_9
#define OID_RS485_RX_GPIO_Port GPIOD
#define OID_RS485_TX_Pin GPIO_PIN_8
#define OID_RS485_TX_GPIO_Port GPIOD
#define FC_MAIN2_PWM_Pin GPIO_PIN_1
#define FC_MAIN2_PWM_GPIO_Port GPIOB
#define FC_MAIN1_PWM_Pin GPIO_PIN_0
#define FC_MAIN1_PWM_GPIO_Port GPIOB
#define LED_GREEN_Pin GPIO_PIN_14
#define LED_GREEN_GPIO_Port GPIOF
#define SPARE_UART7_RX_Pin GPIO_PIN_7
#define SPARE_UART7_RX_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
