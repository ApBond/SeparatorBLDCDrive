/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#define S_AL TIM1->CCER|=TIM_CCER_CC1NE
#define S_AH TIM1->CCER|=TIM_CCER_CC1E
#define S_BL TIM1->CCER|=TIM_CCER_CC2NE
#define S_BH TIM1->CCER|=TIM_CCER_CC2E
#define S_CL TIM1->CCER|=TIM_CCER_CC3NE
#define S_CH TIM1->CCER|=TIM_CCER_CC3E

#define R_AL TIM1->CCER&=~TIM_CCER_CC1NE
#define R_AH TIM1->CCER&=~TIM_CCER_CC1E
#define R_BL TIM1->CCER&=~TIM_CCER_CC2NE
#define R_BH TIM1->CCER&=~TIM_CCER_CC2E
#define R_CL TIM1->CCER&=~TIM_CCER_CC3NE
#define R_CH TIM1->CCER&=~TIM_CCER_CC3E
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
#define AL_Pin GPIO_PIN_0
#define AL_GPIO_Port GPIOC
#define BL_Pin GPIO_PIN_1
#define BL_GPIO_Port GPIOC
#define CL_Pin GPIO_PIN_2
#define CL_GPIO_Port GPIOC
#define AH_Pin GPIO_PIN_8
#define AH_GPIO_Port GPIOA
#define BH_Pin GPIO_PIN_9
#define BH_GPIO_Port GPIOA
#define CH_Pin GPIO_PIN_10
#define CH_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */
typedef enum
{
	BEGIN_STATE,
	INITIAL_STATE,
	WORK_STATE,
	REVERSE_STATE_1,
	REVERSE_STATE_2
}MotorState_t;

typedef enum
{
	P,
	N,
	NC
}PhaseState_t;

typedef struct
{
	PhaseState_t A;
	PhaseState_t B;
	PhaseState_t C;
}MotorPhaseState_t;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
