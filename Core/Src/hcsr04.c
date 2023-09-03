/*
 * hcsr04.c
 *
 *  Created on: Sep 3, 2023
 *      Author: Mati
 */


#include "hcsr04.h"

#define CONST_TO_CALC_DIST 58.0

static HCSR04_t hcsr04;

static volatile uint16_t EdgeUpTime;
static volatile uint16_t EdgeDownTime;
static volatile uint8_t ReadyToCalcDist = 0;
static volatile uint8_t StartMeasureDist = 0;


void HCSR04_Init(TIM_HandleTypeDef* htim_trig, TIM_HandleTypeDef* htim_echo, uint16_t GPIO_Pin_Button, uint16_t GPIO_Pin_TrigSignal, GPIO_TypeDef* GPIO_Port_TrigSignal)
{

	hcsr04.htim_trig = htim_trig;
	hcsr04.htim_echo = htim_echo;
	hcsr04.GPIO_Pin_Button = GPIO_Pin_Button;
	hcsr04.GPIO_Pin_TrigSignal = GPIO_Pin_TrigSignal;
	hcsr04.GPIO_Port_TrigSignal = GPIO_Port_TrigSignal;
}



uint8_t HCSR04_Measurement(float* distance)
{
	if(ReadyToCalcDist == 1)
	{
		ReadyToCalcDist = 0;
		*distance = (float)(EdgeDownTime - EdgeUpTime)/CONST_TO_CALC_DIST;
		return 0;
	}

	return 1;
}


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	// Timer 1 for measure time
	if(htim == hcsr04.htim_echo)
	{
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
		{
			EdgeUpTime = __HAL_TIM_GET_COMPARE(htim, TIM_CHANNEL_1);
			// Turn on TIM 1 in input capture mode Channel 1
			HAL_TIM_IC_Start_IT(htim, TIM_CHANNEL_1);

		}
		else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
		{
			EdgeDownTime = __HAL_TIM_GET_COMPARE(htim, TIM_CHANNEL_2);
			// Turn on TIM1 in input capture mode Channel 2
			HAL_TIM_IC_Start_IT(htim, TIM_CHANNEL_2);
			ReadyToCalcDist = 1;
		}
	}

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	// Timer 10 for trigger measurement
	if(htim == hcsr04.htim_trig)
	{
		HAL_GPIO_WritePin(hcsr04.GPIO_Port_TrigSignal, hcsr04.GPIO_Pin_TrigSignal, GPIO_PIN_RESET);
		HAL_TIM_Base_Stop_IT(hcsr04.htim_trig);
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == hcsr04.GPIO_Pin_Button)
	{
		HAL_TIM_Base_Start_IT(hcsr04.htim_trig);
		HAL_GPIO_WritePin(hcsr04.GPIO_Port_TrigSignal, hcsr04.GPIO_Pin_TrigSignal, GPIO_PIN_SET);
	}
}
