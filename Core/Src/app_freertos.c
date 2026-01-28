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
#include "lcd_i2c.h"
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
extern osMessageQueueId_t sensorQueueHandle;

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
/* Definitions for WaterTask */
osThreadId_t WaterTaskHandle;
const osThreadAttr_t WaterTask_attributes = {
  .name = "WaterTask",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 256 * 4
};
/* Definitions for DisplayTask */
osThreadId_t DisplayTaskHandle;
const osThreadAttr_t DisplayTask_attributes = {
  .name = "DisplayTask",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 512 * 4
};
/* Definitions for sensorQueue */
osMessageQueueId_t sensorQueueHandle;
const osMessageQueueAttr_t sensorQueue_attributes = {
  .name = "sensorQueue"
};
/* Definitions for adcMutex */
osMutexId_t adcMutexHandle;
const osMutexAttr_t adcMutex_attributes = {
  .name = "adcMutex"
};
/* Definitions for lcdMutex */
osMutexId_t lcdMutexHandle;
const osMutexAttr_t lcdMutex_attributes = {
  .name = "lcdMutex"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartSoilTask(void *argument);
void StartLightTask(void *argument);
void StartWaterTask(void *argument);
void StartDisplayTask(void *argument);

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

  /* creation of lcdMutex */
  lcdMutexHandle = osMutexNew(&lcdMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of sensorQueue */
  sensorQueueHandle = osMessageQueueNew (16, 8, &sensorQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of SoilTask */
  SoilTaskHandle = osThreadNew(StartSoilTask, NULL, &SoilTask_attributes);

  /* creation of LightTask */
  LightTaskHandle = osThreadNew(StartLightTask, NULL, &LightTask_attributes);

  /* creation of WaterTask */
  WaterTaskHandle = osThreadNew(StartWaterTask, NULL, &WaterTask_attributes);

  /* creation of DisplayTask */
  DisplayTaskHandle = osThreadNew(StartDisplayTask, NULL, &DisplayTask_attributes);

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
  uint32_t raw;
  for(;;)
  {
    osMutexAcquire(adcMutexHandle, osWaitForever);
    ADC_Select_Channel(ADC_CHANNEL_0);
    HAL_ADC_Start(&hadc);
    if (HAL_ADC_PollForConversion(&hadc, 100) == HAL_OK) {
        raw = HAL_ADC_GetValue(&hadc);
        // WYŚLIJ DO KOLEJKI
        SendSensorData(SENSOR_SOIL, raw);
    }
    HAL_ADC_Stop(&hadc);
    osMutexRelease(adcMutexHandle);

    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);
    osDelay(1000);
  }
  /* USER CODE END StartSoilTask */
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
  /* USER CODE BEGIN StartLightTask */
  uint32_t raw;
  osDelay(333);
  for(;;)
  {
    osMutexAcquire(adcMutexHandle, osWaitForever);
    ADC_Select_Channel(ADC_CHANNEL_1);
    HAL_ADC_Start(&hadc);
    if (HAL_ADC_PollForConversion(&hadc, 100) == HAL_OK) {
        raw = HAL_ADC_GetValue(&hadc);
        // WYŚLIJ DO KOLEJKI
        SendSensorData(SENSOR_LIGHT, raw);
    }
    HAL_ADC_Stop(&hadc);
    osMutexRelease(adcMutexHandle);

    osDelay(1000);
  }
  /* USER CODE END StartLightTask */
}

/* USER CODE BEGIN Header_StartWaterTask */
/**
* @brief Function implementing the WaterTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartWaterTask */
void StartWaterTask(void *argument)
{
  /* USER CODE BEGIN StartWaterTask */
  uint32_t raw;
  osDelay(666);
  for(;;)
  {
    osMutexAcquire(adcMutexHandle, osWaitForever);
    ADC_Select_Channel(ADC_CHANNEL_4);
    HAL_ADC_Start(&hadc);
    if (HAL_ADC_PollForConversion(&hadc, 100) == HAL_OK) {
        raw = HAL_ADC_GetValue(&hadc);
        // WYŚLIJ DO KOLEJKI
        SendSensorData(SENSOR_WATER, raw);
    }
    HAL_ADC_Stop(&hadc);
    osMutexRelease(adcMutexHandle);

    osDelay(1000);
  }
  /* USER CODE END StartWaterTask */
}

/* USER CODE BEGIN Header_StartDisplayTask */
/**
* @brief Function implementing the DisplayTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDisplayTask */
void StartDisplayTask(void *argument)
{
  /* USER CODE BEGIN StartDisplayTask */
  SensorMsg_t receivedMsg;
  char lcd_buffer[20];

  // Lokalne zmienne do pamiętania ostatniego stanu
  uint32_t last_soil = 0;
  uint32_t last_light = 0;
  uint32_t last_water = 0;

  // Init LCD
  osMutexAcquire(lcdMutexHandle, osWaitForever);
  lcd_init();
  lcd_clear();
  osMutexRelease(lcdMutexHandle);

  for(;;)
  {
    // Czekaj na wiadomość z kolejki (osWaitForever usypia taska aż coś przyjdzie!)
    if (osMessageQueueGet(sensorQueueHandle, &receivedMsg, NULL, osWaitForever) == osOK)
    {
        // Zaktualizuj odpowiednią zmienną w zależności od ID
        switch(receivedMsg.id) {
            case SENSOR_SOIL:  last_soil = receivedMsg.value; break;
            case SENSOR_LIGHT: last_light = receivedMsg.value; break;
            case SENSOR_WATER: last_water = receivedMsg.value; break;
        }

        // --- Logika Wyświetlania ---
        // Teraz mamy pewność, że mamy najnowsze dane
        printf("QUEUE UPDATE -> S:%lu L:%lu W:%lu\r\n", last_soil, last_light, last_water);

        osMutexAcquire(lcdMutexHandle, osWaitForever);

        lcd_put_cur(0, 0);
        sprintf(lcd_buffer, "GLEBA:%4lu W:%4lu", last_soil, last_water);
        lcd_send_string(lcd_buffer);

        lcd_put_cur(1, 0);
        sprintf(lcd_buffer, "SWIAT:%4lu OK    ", last_light);
        lcd_send_string(lcd_buffer);

        osMutexRelease(lcdMutexHandle);
    }
  }
  /* USER CODE END StartDisplayTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void ADC_Select_Channel(uint32_t channel)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel = channel;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_160CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) { Error_Handler(); }
}

// Funkcja pomocnicza do wysyłania danych do kolejki
void SendSensorData(SensorType_t type, uint32_t value) {
    SensorMsg_t msg;
    msg.id = type;
    msg.value = value;
    // Czekaj 0ms. Jeśli kolejka pełna, trudno, przepadnie jeden odczyt (nie blokujemy sensora)
    osMessageQueuePut(sensorQueueHandle, &msg, 0, 0);
}

/* USER CODE END Application */
