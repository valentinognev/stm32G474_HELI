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
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32g431xx.h"
#include "stm32g4xx_ll_tim.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_cortex.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_system.h"
#include "stm32g4xx_ll_utils.h"
#include "stm32g4xx_ll_pwr.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_dma.h"

#include "stm32g4xx_ll_exti.h"
#include "stm32g4xx_ll_spi.h"
#include "stm32g4xx_ll_tim.h"
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
#define PSCALERSERVO_1 100
#define PSCALERSERVO_2 100
#define PSCALERSERVO_3 100
#define PSCALERMOTOR_MAIN 100
#define SERVO_2_Pin GPIO_PIN_0
#define SERVO_2_GPIO_Port GPIOA
#define AS5047_CS_Pin GPIO_PIN_4
#define AS5047_CS_GPIO_Port GPIOA
#define AS5047_SCK_Pin GPIO_PIN_5
#define AS5047_SCK_GPIO_Port GPIOA
#define AS5047_MISO_Pin GPIO_PIN_6
#define AS5047_MISO_GPIO_Port GPIOA
#define AS5047_MOSI_Pin GPIO_PIN_7
#define AS5047_MOSI_GPIO_Port GPIOA
#define DSHOT_Pin GPIO_PIN_14
#define DSHOT_GPIO_Port GPIOB
#define SERVO_3_Pin GPIO_PIN_6
#define SERVO_3_GPIO_Port GPIOC
#define SERVO_1_Pin GPIO_PIN_8
#define SERVO_1_GPIO_Port GPIOA
#define AS5047_NSS_Pin GPIO_PIN_15
#define AS5047_NSS_GPIO_Port GPIOA
#define MOTOR_MAIN_Pin GPIO_PIN_6
#define MOTOR_MAIN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define INTERNAL_TEMPSENSOR_AVGSLOPE   ((int32_t) 2500)        /* Internal temperature sensor, parameter Avg_Slope (unit: uV/DegCelsius). Refer to device datasheet for min/typ/max values. */
#define INTERNAL_TEMPSENSOR_V25        ((int32_t)  760)        /* Internal temperature sensor, parameter V25 (unit: mV). Refer to device datasheet for min/typ/max values. */
#define INTERNAL_TEMPSENSOR_V25_TEMP   ((int32_t)   25)
#define INTERNAL_TEMPSENSOR_V25_VREF   ((int32_t) 3300)

#define TIMSERVO_1 TIM2
#define TIMSERVO_2  TIM2
#define TIMSERVO_3   TIM3
#define TIMTAIL_MOTOR  TIM4
#define TIMMAIN_MOTOR  TIM4
#define PWMNUMVAL 100

#define min(a,b) ((a) < (b) ? (a) : (b))
#define max(a,b) ((a) > (b) ? (a) : (b))

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
