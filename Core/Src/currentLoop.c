#include "currentLoop.h"

extern TIM_HandleTypeDef htim1;
uint16_t controllImpact;
float currentAmp;
uint8_t regulatorStart=0;
float currentRef=CURRENT_REFERENCE;
uint8_t overCurrentFlag=0;

PIDHandle_t currentLoopPID=
{
	.kp=1,
  .ki=0.4,
  .kd=0.3,
  .prevError=0,
  .integralTerm=0
};

void ADC_IRQHandler(void)
{
	static int32_t current=0;
	uint16_t adcRez;
	static uint16_t measurmentCount=0;
	if((ADC1->SR & ADC_SR_JEOC_Msk) !=0)
	{
		ADC1->SR&=~ADC_SR_JEOC;//Сбросить флаг
		adcRez=ADC1->JDR1;
		current+=adcRez;
		measurmentCount++;
		if(measurmentCount==MEASURMENT_COUNT)
		{
			current/=MEASURMENT_COUNT;
			if(current-CURRENT_SENSOR_OFFSET>0)
				currentAmp=((float)((float)(current-CURRENT_SENSOR_OFFSET)*0.00081)/CURRENT_SENSOR_SENSETIVITY);
			else
				currentAmp=0;
			if(currentAmp>CURRENT_BREAK_LIMIT_AMP)
			{
				GPIOC->ODR&=(~(AL|BL|CL));
				TIM1->CCR1=0;
				TIM1->CCR2=0;
				TIM1->CCR3=0;
				HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1);
				HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_2);
				HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_3);
				regulatorStart=0;
				overCurrentFlag=1;
			}
			if(regulatorStart)
			{
				controllImpact=(uint16_t)PIDController(&currentLoopPID,currentRef-currentAmp);
				if(controllImpact<0) controllImpact=0;
				if(controllImpact>TIM_PWM_LIMIT) controllImpact=TIM_PWM_LIMIT;
				TIM1->CCR1=controllImpact;
				TIM1->CCR2=controllImpact;
				TIM1->CCR3=controllImpact;
			}
			current=0;
			measurmentCount=0;
		}
	}
}


float PIDController(PIDHandle_t * PID,float error)
{
    float deltaError = error - PID->prevError;
    float controllerOut;
    PID->prevError=error;
    PID->integralTerm+=error;
    controllerOut=error*PID->kp+PID->integralTerm*PID->ki+deltaError*PID->kd;
    return controllerOut;
}


void adcInit(uint8_t channel)
{
	uint16_t timeout;
	RCC->APB2ENR|=RCC_APB2ENR_ADC1EN;//Включить тактирование ADC1
	RCC->AHB1ENR|=RCC_AHB1ENR_GPIOAEN;//Включить тактирование GPIOA
	GPIOA->MODER|=GPIO_MODER_MODE0<<(channel*2);
	ADC->CCR|=ADC_CCR_ADCPRE_0;//Делитель АЦП на 4
	ADC1->CR2|=ADC_CR2_ADON;//Включить АЦП
	ADC1->CR1|=ADC_CR1_SCAN//Режим сканирования
	| ADC_CR1_JEOCIE;//Включить прерывание по окончанию преобразования инжективного канала
	ADC1->JSQR|=(channel<<ADC_JSQR_JSQ4_Pos);//Выбор канала
	ADC1->CR2|=0xB<<ADC_CR2_JEXTSEL_Pos//Запуск преобразования по переполнению таймера 5
	| ADC_CR2_JEXTEN_0;//Разрешить запуск по триггеру
	NVIC_EnableIRQ(ADC_IRQn);
}

void Tim5InitTrigger(void)
{
	RCC->APB1ENR|=RCC_APB1ENR_TIM5EN;//Включить тактирование таймера 5
	TIM5->CR2|=TIM_CR2_MMS_1;//Разрешить генерацию триггера при переполнении
	TIM5->PSC=100;//Делитель на 1МГц
}

void setTriggerPetiod(uint16_t period_us)
{
	TIM5->CR1&=~TIM_CR1_CEN;//Выключить таймер
	TIM5->ARR=period_us;//Период в мкс
	TIM5->CR1|=TIM_CR1_CEN;//Включить таймер
}



