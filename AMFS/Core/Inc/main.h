/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#define TM6_PRESCALER 50
#define TM6_PERIOD 1000
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define S1_Pin GPIO_PIN_1
#define S1_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define S2_Pin GPIO_PIN_4
#define S2_GPIO_Port GPIOA
#define D1_Pin GPIO_PIN_5
#define D1_GPIO_Port GPIOA
#define D2_Pin GPIO_PIN_6
#define D2_GPIO_Port GPIOA
#define D3_Pin GPIO_PIN_7
#define D3_GPIO_Port GPIOA
#define S3_Pin GPIO_PIN_0
#define S3_GPIO_Port GPIOB
#define display_SFTCLK_Pin GPIO_PIN_8
#define display_SFTCLK_GPIO_Port GPIOA
#define display_SDI_Pin GPIO_PIN_9
#define display_SDI_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define display_LCHCLK_Pin GPIO_PIN_5
#define display_LCHCLK_GPIO_Port GPIOB
#define D4_Pin GPIO_PIN_6
#define D4_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
