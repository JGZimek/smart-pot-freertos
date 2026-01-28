/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "cmsis_os2.h"
#include "FreeRTOS.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "adc.h"
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
extern volatile uint32_t soil_raw_value;

/* USER CODE END Variables */
/* Definitions for SoilTask */
osThreadId_t SoilTaskHandle;
const osThreadAttr_t SoilTask_attributes = {
  .name = "SoilTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 256 * 4
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartSoilTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

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
  /* creation of SoilTask */
  SoilTaskHandle = osThreadNew(StartSoilTask, NULL, &SoilTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartSoilTask */
/**
  * @brief  Function implementing the SoilTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartSoilTask */
void StartSoilTask(void *argument)
{
  /* USER CODE BEGIN StartSoilTask */
  uint32_t raw_val = 0;

  for(;;)
  {
    // 1. Miganie diodą PB5 - diagnostyka czy task żyje
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);

    // 2. Ręczny odczyt ADC (klasyczny polling)
    HAL_ADC_Start(&hadc); // Startuj pomiar
    if (HAL_ADC_PollForConversion(&hadc, 10) == HAL_OK) // Czekaj max 10ms
    {
      raw_val = HAL_ADC_GetValue(&hadc); // Pobierz wartość
    }
    HAL_ADC_Stop(&hadc); // Zatrzymaj ADC

    // 3. Wypisanie na UART
    printf("Soil Value (Polling): %lu\r\n", raw_val);

    osDelay(500); // Czekaj 0.5s
  }
  /* USER CODE END StartSoilTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
