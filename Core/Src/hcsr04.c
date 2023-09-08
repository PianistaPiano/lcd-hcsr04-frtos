/*
 * hcsr04.c
 *
 *  Created on: Sep 3, 2023
 *      Author: Mati
 */


#include "hcsr04.h"

#define CONST_TO_CALC_DIST 58.772

extern volatile uint16_t EdgeUpTime;
extern volatile uint16_t EdgeDownTime;
extern volatile uint8_t ReadyToCalcDist;
extern volatile uint8_t StartMeasureDist;

void HCSR04_Init(HCSR04_t* hcsr04,TIM_HandleTypeDef* htim_trig, TIM_HandleTypeDef* htim_echo, uint16_t GPIO_Pin_Button, uint16_t GPIO_Pin_TrigSignal, GPIO_TypeDef* GPIO_Port_TrigSignal)
{

	hcsr04->htim_trig = htim_trig;
	hcsr04->htim_echo = htim_echo;
	hcsr04->GPIO_Pin_Button = GPIO_Pin_Button;
	hcsr04->GPIO_Pin_TrigSignal = GPIO_Pin_TrigSignal;
	hcsr04->GPIO_Port_TrigSignal = GPIO_Port_TrigSignal;
}



uint8_t HCSR04_Measurement(HCSR04_t* hcsr04)
{
	float Distance;
	if(ReadyToCalcDist == 1)
	{
		ReadyToCalcDist = 0;
		Distance = (float)(EdgeDownTime - EdgeUpTime)/CONST_TO_CALC_DIST;
		if(Distance < 0)
			return 1;
		hcsr04->distance = Distance;
		return 0;
	}

	return 1;
}


