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

#include "stm32g4xx_ll_adc.h"
#include "stm32g4xx_ll_spi.h"
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32g474xx.h"
#include "stm32g4xx_ll_spi.h"
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
#define PHASE_Pin GPIO_PIN_0
#define PHASE_GPIO_Port GPIOA
#define AVGSPEEED_Pin GPIO_PIN_1
#define AVGSPEEED_GPIO_Port GPIOA
#define AMPSPEED_Pin GPIO_PIN_2
#define AMPSPEED_GPIO_Port GPIOA
#define AS5047_CS_Pin GPIO_PIN_4
#define AS5047_CS_GPIO_Port GPIOA
#define AS5047_SCK_Pin GPIO_PIN_5
#define AS5047_SCK_GPIO_Port GPIOA
#define AS5047_MISO_Pin GPIO_PIN_6
#define AS5047_MISO_GPIO_Port GPIOA
#define AS5047_MOSI_Pin GPIO_PIN_7
#define AS5047_MOSI_GPIO_Port GPIOA
#define MOTOR_TAIL_Pin GPIO_PIN_2
#define MOTOR_TAIL_GPIO_Port GPIOB
#define SERVO_3RD_Pin GPIO_PIN_6
#define SERVO_3RD_GPIO_Port GPIOC
#define SERVO_PITCH_Pin GPIO_PIN_8
#define SERVO_PITCH_GPIO_Port GPIOA
#define SERVO_ROLL_Pin GPIO_PIN_15
#define SERVO_ROLL_GPIO_Port GPIOA
#define DSHOT_Pin GPIO_PIN_5
#define DSHOT_GPIO_Port GPIOB
#define MOTOR_MAIN_Pin GPIO_PIN_6
#define MOTOR_MAIN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define TIMCLOCK (100000000)
#define PWM_CHARACTERISTIC_FREQUENCY (400) // Hz PWMNUMVAL
#undef PSCALARSERVO_ROLL
#define PSCALARSERVO_ROLL (TIMCLOCK/(PWM_CHARACTERISTIC_FREQUENCY*0xFFFFFFFF/PWMNUMVAL)-1) //(170-1)
#undef PSCALARSERVO_PITCH
#define PSCALARSERVO_PITCH (TIMCLOCK/(PWM_CHARACTERISTIC_FREQUENCY*0xFFFFFFFF/PWMNUMVAL)-1)
#undef PSCALARSERVO_3RD
#define PSCALARSERVO_3RD (TIMCLOCK/(PWM_CHARACTERISTIC_FREQUENCY*0xFFFF/PWMNUMVAL)-1)
#undef PSCALARMOTOR_MAIN
#define PSCALARMOTOR_MAIN (TIMCLOCK/(4000*0xFFFF/PWMNUMVAL)-1) //(170-1)
#undef PSCALARMOTOR_TAIL
#define PSCALARMOTOR_TAIL (TIMCLOCK/(2000*0xFFFF/PWMNUMVAL)-1)

#define INTERNAL_TEMPSENSOR_AVGSLOPE   ((int32_t) 2500)        /* Internal temperature sensor, parameter Avg_Slope (unit: uV/DegCelsius). Refer to device datasheet for min/typ/max values. */
#define INTERNAL_TEMPSENSOR_V25        ((int32_t)  760)        /* Internal temperature sensor, parameter V25 (unit: mV). Refer to device datasheet for min/typ/max values. */
#define INTERNAL_TEMPSENSOR_V25_TEMP   ((int32_t)   25)
#define INTERNAL_TEMPSENSOR_V25_VREF   ((int32_t) 3300)

#define TIMSERVO_PITCH TIM2
#define TIMSERVO_ROLL  TIM2
#define TIMSERVO_3RD   TIM3
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
