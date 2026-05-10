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
#define SOC_Pin GPIO_PIN_0
#define SOC_GPIO_Port GPIOC
#define SOB_Pin GPIO_PIN_1
#define SOB_GPIO_Port GPIOC
#define AUX_TEMP_Pin GPIO_PIN_5
#define AUX_TEMP_GPIO_Port GPIOA
#define VBUS_Pin GPIO_PIN_6
#define VBUS_GPIO_Port GPIOA
#define MOS_TEMP_Pin GPIO_PIN_5
#define MOS_TEMP_GPIO_Port GPIOC
#define TEST_GPIO_Pin GPIO_PIN_2
#define TEST_GPIO_GPIO_Port GPIOB
#define DRV_EN_Pin GPIO_PIN_12
#define DRV_EN_GPIO_Port GPIOB
#define INLA_Pin GPIO_PIN_13
#define INLA_GPIO_Port GPIOB
#define INLB_Pin GPIO_PIN_14
#define INLB_GPIO_Port GPIOB
#define INLC_Pin GPIO_PIN_15
#define INLC_GPIO_Port GPIOB
#define INHA_Pin GPIO_PIN_8
#define INHA_GPIO_Port GPIOA
#define INHB_Pin GPIO_PIN_9
#define INHB_GPIO_Port GPIOA
#define INHC_Pin GPIO_PIN_10
#define INHC_GPIO_Port GPIOA
#define USB_DM_Pin GPIO_PIN_11
#define USB_DM_GPIO_Port GPIOA
#define USB_DP_Pin GPIO_PIN_12
#define USB_DP_GPIO_Port GPIOA
#define nFAULT_Pin GPIO_PIN_2
#define nFAULT_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
