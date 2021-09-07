/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "currentLoop.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */
uint8_t direction=1;
MotorState_t motorState=BEGIN_STATE;
uint8_t count=0;
uint8_t startFlag=0;
MotorPhaseState_t currentPhaseState;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
extern PIDHandle_t currentLoopPID;
extern uint16_t controllImpact;
extern uint8_t regulatorStart;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void sixStep(uint8_t ha,uint8_t hb,uint8_t hc)
{
	R_AH;
	R_BH;
	R_CH;
	R_AL;
	R_BL;
	R_CL;
	if(direction==0)
	{
		if(ha && (!hb) && hc)
		{
			S_AH;
			S_CL;
		}
		else if(ha && (!hb) && (!hc))
		{
			S_BH;
			S_CL;
		}
		else if(ha && hb && (!hc))
		{
			S_BH;
			S_AL;
		}
		else if((!ha) && (hb) && (!hc))
		{
			S_CH;
			S_AL;
		}
		else if((!ha) && (hb) && (hc))
		{
			S_CH;
			S_BL;
		}
		else if((!ha) && (!hb) && (hc))
		{
			S_AH;
			S_BL;
		}
	}
	else
	{
		if(ha && (!hb) && hc)
		{
			S_AH;
			S_BL;
		}
		else if(ha && (!hb) && (!hc))
		{
			S_AH;
			S_CL;
		}
		else if(ha && hb && (!hc))
		{
			S_BH;
			S_CL;
		}
		else if((!ha) && (hb) && (!hc))
		{
			S_BH;
			S_AL;
		}
		else if((!ha) && (hb) && (hc))
		{
			S_CH;
			S_AL;
		}
		else if((!ha) && (!hb) && (hc))
		{
			S_CH;
			S_BL;
		}
	}
}

void step()
{
	R_AH;
	R_BH;
	R_CH;
	R_AL;
	R_BL;
	R_CL;
	if(currentPhaseState.A==P && currentPhaseState.B==N)
	{
		S_AH;
		S_CL;
		currentPhaseState.A=P;
		currentPhaseState.B=NC;
		currentPhaseState.C=N;
	}
	else if(currentPhaseState.A==P && currentPhaseState.C==N)
	{
		S_BH;
		S_CL;
		currentPhaseState.A=NC;
		currentPhaseState.B=P;
		currentPhaseState.C=N;
	}
	else if(currentPhaseState.B==P && currentPhaseState.C==N)
	{
		S_BH;
		S_AL;
		currentPhaseState.A=N;
		currentPhaseState.B=P;
		currentPhaseState.C=NC;
	}
	else if(currentPhaseState.B==P && currentPhaseState.A==N)
	{
		S_CH;
		S_AL;
		currentPhaseState.A=N;
		currentPhaseState.B=NC;
		currentPhaseState.C=P;
	}
	else if(currentPhaseState.C==P && currentPhaseState.A==N)
	{
		S_CH;
		S_BL;
		currentPhaseState.A=NC;
		currentPhaseState.B=N;
		currentPhaseState.C=P;
	}
	else if(currentPhaseState.C==P && currentPhaseState.B==N)
	{
		S_AH;
		S_BL;
		currentPhaseState.A=P;
		currentPhaseState.B=N;
		currentPhaseState.C=NC;
	}
}

void reverseStep()
{
	R_AH;
	R_BH;
	R_CH;
	R_AL;
	R_BL;
	R_CL;
	if(currentPhaseState.B==P && currentPhaseState.C==N)
	{
		S_AH;
		S_CL;
		currentPhaseState.A=P;
		currentPhaseState.B=NC;
		currentPhaseState.C=N;
	}
	else if(currentPhaseState.B==P && currentPhaseState.A==N)
	{
		S_BH;
		S_CL;
		currentPhaseState.A=NC;
		currentPhaseState.B=P;
		currentPhaseState.C=N;
	}
	else if(currentPhaseState.C==P && currentPhaseState.A==N)
	{
		S_BH;
		S_AL;
		currentPhaseState.A=N;
		currentPhaseState.B=P;
		currentPhaseState.C=NC;
	}
	else if(currentPhaseState.C==P && currentPhaseState.B==N)
	{
		S_CH;
		S_AL;
		currentPhaseState.A=N;
		currentPhaseState.B=NC;
		currentPhaseState.C=P;
	}
	else if(currentPhaseState.A==P && currentPhaseState.B==N)
	{
		S_CH;
		S_BL;
		currentPhaseState.A=NC;
		currentPhaseState.B=N;
		currentPhaseState.C=P;
	}
	else if(currentPhaseState.A==P && currentPhaseState.C==N)
	{
		S_AH;
		S_BL;
		currentPhaseState.A=P;
		currentPhaseState.B=N;
		currentPhaseState.C=NC;
	}
}


MotorPhaseState_t hallAlignment(uint8_t ha,uint8_t hb,uint8_t hc)
{
	MotorPhaseState_t phaseState;
	R_AH;
	R_BH;
	R_CH;
	R_AL;
	R_BL;
	R_CL;
	if(ha && (!hb) && hc)
	{
		S_CH;
		S_BL;
		phaseState.A=NC;
		phaseState.B=N;
		phaseState.C=P;
	}
	else if(ha && (!hb) && (!hc))
	{
		S_AH;
		S_BL;
		phaseState.A=P;
		phaseState.B=N;
		phaseState.C=NC;
	}
	else if(ha && hb && (!hc))
	{
		S_AH;
		S_CL;
		phaseState.A=P;
		phaseState.B=NC;
		phaseState.C=N;
	}
	else if((!ha) && (hb) && (!hc))
	{
		S_BH;
		S_CL;
		phaseState.A=NC;
		phaseState.B=P;
		phaseState.C=N;
	}
	else if((!ha) && (hb) && (hc))
	{
		S_BH;
		S_AL;
		phaseState.A=N;
		phaseState.B=P;
		phaseState.C=NC;
	}
	else if((!ha) && (!hb) && (hc))
	{
		S_CH;
		S_AL;
		phaseState.A=N;
		phaseState.B=NC;
		phaseState.C=P;
	}
	return phaseState;
}

void EXTI15_10_IRQHandler(void)
{
	static uint8_t statePosition=0;
	uint8_t hallState;
	EXTI->PR|=EXTI_PR_PR13;
	if(motorState==BEGIN_STATE)
	{
		currentLoopPID.integralTerm=0;
		currentLoopPID.prevError=0;
		currentPhaseState=hallAlignment(GPIOB->IDR&(1<<0),((GPIOB->IDR&(1<<1))>>1),((GPIOB->IDR&(1<<2))>>2));
		motorState=INITIAL_STATE;
		regulatorStart=1;
	}
	else if(motorState==INITIAL_STATE)
	{
		motorState=WORK_STATE;
		step();
		//sixStep(GPIOB->IDR&(1<<0),((GPIOB->IDR&(1<<1))>>1),((GPIOB->IDR&(1<<2))>>2));
	}
}

void buttonInit(void)
{
	RCC->AHB1ENR|=RCC_AHB1ENR_GPIOCEN;//???????????? ????? C
	GPIOC->PUPDR|=GPIO_PUPDR_PUPD13_0;//PC13 Pull up
	NVIC_EnableIRQ(EXTI15_10_IRQn);//???????? ??????????
	NVIC_SetPriority(EXTI15_10_IRQn,6);
	SYSCFG->EXTICR[3]|=SYSCFG_EXTICR4_EXTI13_PC;
	EXTI->IMR|=EXTI_IMR_IM13;
	EXTI->FTSR|=EXTI_FTSR_TR13;
	__enable_irq();
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	/*if(motorState==WORK_STATE)
		sixStep(GPIOB->IDR&(1<<0),((GPIOB->IDR&(1<<1))>>1),((GPIOB->IDR&(1<<2))>>2));*/
	if(motorState==WORK_STATE)
	{
		step();
		motorState=REVERSE_STATE_1;
	}
	//count++;
	else if(motorState==REVERSE_STATE_1)
	{
		reverseStep();
		motorState=REVERSE_STATE_2;
	}
	else if(motorState==REVERSE_STATE_2)
	{
		reverseStep();
		motorState=INITIAL_STATE;
	}

}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint8_t i=0;
	uint8_t hallState;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
	buttonInit();
	adcInit(0);
	Tim5InitTrigger();
	setTriggerPetiod(1);
	//GPIOC->ODR&=(~(AL|AH|BL|BH|CL|CH));
	TIM1->CCR1=0;
	TIM1->CCR2=0;
	TIM1->CCR3=0;
	TIM1->BDTR|=TIM_BDTR_MOE;
	TIM1->CR1|=TIM_CR1_CEN;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 10000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 150;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, AL_Pin|BL_Pin|CL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : AL_Pin BL_Pin CL_Pin */
  GPIO_InitStruct.Pin = AL_Pin|BL_Pin|CL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
