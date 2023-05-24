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
#include "stm32f1xx_hal.h"

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
#define LUZ_TECHO_A2_Pin GPIO_PIN_2
#define LUZ_TECHO_A2_GPIO_Port GPIOA
#define LUZ_CABECERA_A3_Pin GPIO_PIN_3
#define LUZ_CABECERA_A3_GPIO_Port GPIOA
#define LUZ_MESA_A4_Pin GPIO_PIN_4
#define LUZ_MESA_A4_GPIO_Port GPIOA
#define LUZ_TECHO_Pin GPIO_PIN_5
#define LUZ_TECHO_GPIO_Port GPIOA
#define LUZ_CABECERA_Pin GPIO_PIN_6
#define LUZ_CABECERA_GPIO_Port GPIOA
#define CANTONERA_Pin GPIO_PIN_7
#define CANTONERA_GPIO_Port GPIOA
#define LUZ_NCABINA_Pin GPIO_PIN_0
#define LUZ_NCABINA_GPIO_Port GPIOB
#define LUZ_MESA_Pin GPIO_PIN_1
#define LUZ_MESA_GPIO_Port GPIOB
#define PUERTA_INT_Pin GPIO_PIN_3
#define PUERTA_INT_GPIO_Port GPIOB
#define PUERTA_INT_EXTI_IRQn EXTI3_IRQn
#define PANIC_INT_Pin GPIO_PIN_4
#define PANIC_INT_GPIO_Port GPIOB
#define PANIC_INT_EXTI_IRQn EXTI4_IRQn
#define ID_CAN_3_Pin GPIO_PIN_5
#define ID_CAN_3_GPIO_Port GPIOB
#define ID_CAN_4_Pin GPIO_PIN_6
#define ID_CAN_4_GPIO_Port GPIOB
#define ID_CAN_5_Pin GPIO_PIN_7
#define ID_CAN_5_GPIO_Port GPIOB
#define ID_CAN_2_Pin GPIO_PIN_8
#define ID_CAN_2_GPIO_Port GPIOB
#define ID_CAN_1_Pin GPIO_PIN_9
#define ID_CAN_1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
