/*
 * hcsr04.h
 *
 *  Created on: Sep 3, 2023
 *      Author: Mati
 */

#ifndef INC_HCSR04_H_
#define INC_HCSR04_H_

#include "main.h"

typedef struct HCSR04
{
	TIM_HandleTypeDef* htim_trig; // configured for: after 10us PeriodElapsed
	TIM_HandleTypeDef* htim_echo; // configured to input capture for raising and falling edge on CH1 and CH2, CH1 - direct mode, CH2 indirect mode

	uint16_t GPIO_Pin_Button; // for exti, button
	uint16_t GPIO_Pin_TrigSignal; // for trigger signal
	GPIO_TypeDef* GPIO_Port_TrigSignal; //for trigger signal

} HCSR04_t;


void HCSR04_Init(TIM_HandleTypeDef* htim_trig, TIM_HandleTypeDef* htim_echo, uint16_t GPIO_Pin_Button, uint16_t GPIO_Pin_TrigSignal, GPIO_TypeDef* GPIO_Port_TrigSignal);

uint8_t HCSR04_Measurement(float* distance);



#endif /* INC_HCSR04_H_ */
