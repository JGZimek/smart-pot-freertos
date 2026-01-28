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
extern osMutexId_t adcMutexHandle;

/* USER CODE END Variables */
/* Definitions for SoilTask */
osThreadId_t SoilTaskHandle;
const osThreadAttr_t SoilTask_attributes = {
  .name = "SoilTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 256 * 4
};
/* Definitions for LightTask */
osThreadId_t LightTaskHandle;
const osThreadAttr_t LightTask_attributes = {
  .name = "LightTask",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 256 * 4
};
/* Definitions for adcMutex */
osMutexId_t adcMutexHandle;
const osMutexAttr_t adcMutex_attributes = {
  .name = "adcMutex"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartSoilTask(void *argument);
void StartLightTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of adcMutex */
  adcMutexHandle = osMutexNew(&adcMutex_attributes);

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

  /* creation of LightTask */
  LightTaskHandle = osThreadNew(StartLightTask, NULL, &LightTask_attributes);

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
  uint32_t soil_val = 0;

  for(;;)
  {
    // 1. Zdobądź klucz do ADC (czekaj w nieskończoność, aż będzie wolny)
    osMutexAcquire(adcMutexHandle, osWaitForever);

    // 2. Skonfiguruj ADC na Kanał 0
    ADC_Select_Channel(ADC_CHANNEL_0);

    // 3. Wykonaj pomiar
    HAL_ADC_Start(&hadc);
    if (HAL_ADC_PollForConversion(&hadc, 100) == HAL_OK) {
        soil_val = HAL_ADC_GetValue(&hadc);
    }
    HAL_ADC_Stop(&hadc);

    // 4. Oddaj klucz
    osMutexRelease(adcMutexHandle);

    // 5. Logowanie
    printf("[SOIL] Value: %lu\r\n", soil_val);

    // Mruganie dla diagnostyki tego taska
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);

    osDelay(1000); // Co 1 sekunda
  }
}

/* USER CODE BEGIN Header_StartLightTask */
/**
* @brief Function implementing the LightTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLightTask */
void StartLightTask(void *argument)
{
  uint32_t light_val = 0;

  // Przesunięcie startu, żeby nie pchały się na UART w tej samej milisekundzie
  osDelay(500);

  for(;;)
  {
    // 1. Zdobądź klucz do ADC
    osMutexAcquire(adcMutexHandle, osWaitForever);

    // 2. Skonfiguruj ADC na Kanał 1
    ADC_Select_Channel(ADC_CHANNEL_1);

    // 3. Wykonaj pomiar
    HAL_ADC_Start(&hadc);
    if (HAL_ADC_PollForConversion(&hadc, 100) == HAL_OK) {
        light_val = HAL_ADC_GetValue(&hadc);
    }
    HAL_ADC_Stop(&hadc);

    // 4. Oddaj klucz
    osMutexRelease(adcMutexHandle);

    // 5. Logowanie
    printf("[LIGHT] Value: %lu\r\n", light_val);

    osDelay(1000); // Co 1 sekunda
  }
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void ADC_Select_Channel(uint32_t channel)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel = channel;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_160CYCLES_5; // Dłuższy czas próbkowania

  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE END Application */
