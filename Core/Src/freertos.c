/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "printf.h"
#include "hcsr04.h"
#include "stdio.h"
#include "LCD_H44780U.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "semphr.h"
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
/* USER CODE BEGIN Variables */
HCSR04_t hc;

// LCD Init
LCD_H44780_t lcd;
GPIO_TypeDef* Data_Ports[8] = {LCD_DB7_GPIO_Port, LCD_DB6_GPIO_Port, LCD_DB5_GPIO_Port,
								LCD_DB4_GPIO_Port, LCD_DB3_GPIO_Port, LCD_DB2_GPIO_Port,
								LCD_DB1_GPIO_Port, LCD_DB0_GPIO_Port};
uint16_t Data_Pins[8] = {LCD_DB7_Pin, LCD_DB6_Pin, LCD_DB5_Pin, LCD_DB4_Pin,
						LCD_DB3_Pin, LCD_DB2_Pin, LCD_DB1_Pin, LCD_DB0_Pin};
volatile uint16_t EdgeUpTime;
volatile uint16_t EdgeDownTime;
volatile uint8_t ReadyToCalcDist = 0;
volatile uint8_t StartMeasureDist = 0;
/* USER CODE END Variables */
/* Definitions for HeartBeatTask */
osThreadId_t HeartBeatTaskHandle;
const osThreadAttr_t HeartBeatTask_attributes = {
  .name = "HeartBeatTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for CalcDistTask */
osThreadId_t CalcDistTaskHandle;
const osThreadAttr_t CalcDistTask_attributes = {
  .name = "CalcDistTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for PrintfMutex */
osMutexId_t PrintfMutexHandle;
const osMutexAttr_t PrintfMutex_attributes = {
  .name = "PrintfMutex"
};
/* Definitions for CalcDistSem */
osSemaphoreId_t CalcDistSemHandle;
const osSemaphoreAttr_t CalcDistSem_attributes = {
  .name = "CalcDistSem"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartHeartBeatTask(void *argument);
void StartCalcDistTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
	// HCSR04 Init
	HCSR04_Init(&hc, &htim10, &htim1, B1_Pin, HCSR04_Trig_Pin, HCSR04_Trig_GPIO_Port);
    LCD_create(&lcd ,Data_Ports, Data_Pins, LCD_RS_GPIO_Port, LCD_RS_Pin,
			 LCD_RW_GPIO_Port, LCD_RW_Pin, LCD_E_GPIO_Port, LCD_E_Pin);

    LCD_Init(&lcd, LCD_8BIT_MODE, LCD_ONE_LINE, LCD_5x8, LCD_CURSOR_OFF, LCD_BLINKING_OFF);
    // Turn on TIM 1 in input capture mode Channel 1
    HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
    // Turn on TIM1 in input capture mode Channel 2
    HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_2);
  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of PrintfMutex */
  PrintfMutexHandle = osMutexNew(&PrintfMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of CalcDistSem */
  CalcDistSemHandle = osSemaphoreNew(1, 1, &CalcDistSem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of HeartBeatTask */
  HeartBeatTaskHandle = osThreadNew(StartHeartBeatTask, NULL, &HeartBeatTask_attributes);

  /* creation of CalcDistTask */
  CalcDistTaskHandle = osThreadNew(StartCalcDistTask, NULL, &CalcDistTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartHeartBeatTask */
/**
  * @brief  Function implementing the HeartBeatTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartHeartBeatTask */
void StartHeartBeatTask(void *argument)
{
  /* USER CODE BEGIN StartHeartBeatTask */
  /* Infinite loop */
  for(;;)
  {
	HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	printf("LED\n\r");
    osDelay(500);
  }
  /* USER CODE END StartHeartBeatTask */
}

/* USER CODE BEGIN Header_StartCalcDistTask */
/**
* @brief Function implementing the CalcDistTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCalcDistTask */
void StartCalcDistTask(void *argument)
{
  /* USER CODE BEGIN StartCalcDistTask */
  /* Infinite loop */
  for(;;)
  {
	if(xSemaphoreTake(CalcDistSemHandle, portMAX_DELAY) == pdTRUE)
	{
		if(HCSR04_Measurement(&hc) == 0)
		{
			printf("Wynik: %.1f\n\r", hc.distance);
		}
	}
  }
  /* USER CODE END StartCalcDistTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void _putchar(char character)
{
	xSemaphoreTake(PrintfMutexHandle, portMAX_DELAY);
	HAL_UART_Transmit(&huart2, (uint8_t*)&character, 1, 1000);
	xSemaphoreGive(PrintfMutexHandle);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	// Timer 1 for measure time
	if(htim == hc.htim_echo)
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
			xSemaphoreGiveFromISR(CalcDistSemHandle, NULL);
		}
	}

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	// Timer 10 for trigger measurement
	if(htim == hc.htim_trig)
	{
		HAL_GPIO_WritePin(hc.GPIO_Port_TrigSignal, hc.GPIO_Pin_TrigSignal, GPIO_PIN_RESET);
		HAL_TIM_Base_Stop_IT(hc.htim_trig);
	}
	if (htim->Instance == TIM11)
	{
	    HAL_IncTick();
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == hc.GPIO_Pin_Button)
	{
		HAL_TIM_Base_Start_IT(hc.htim_trig);
		HAL_GPIO_WritePin(hc.GPIO_Port_TrigSignal, hc.GPIO_Pin_TrigSignal, GPIO_PIN_SET);
	}
}
/* USER CODE END Application */

