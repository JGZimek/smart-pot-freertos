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

static DisplayView_t current_view = VIEW_ALL;
static ControlMode_t current_mode = MODE_MANUAL;
static uint8_t is_pump_on = 0;

static uint32_t last_soil = 0;
static uint32_t last_light = 0;
static uint32_t last_water = 0;

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
/* Definitions for ButtonTask */
osThreadId_t ButtonTaskHandle;
const osThreadAttr_t ButtonTask_attributes = {
  .name = "ButtonTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
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
void ADC_Select_Channel(uint32_t channel);
void SendAppMessage(MsgSource_t src, uint32_t val);
void SetPumpState(uint8_t state);

/* USER CODE END FunctionPrototypes */

void StartSoilTask(void *argument);
void StartLightTask(void *argument);
void StartWaterTask(void *argument);
void StartDisplayTask(void *argument);
void StartButtonTask(void *argument);

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

  /* creation of ButtonTask */
  ButtonTaskHandle = osThreadNew(StartButtonTask, NULL, &ButtonTask_attributes);

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
        SendAppMessage(MSG_SRC_SENSOR_SOIL, raw);
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
        SendAppMessage(MSG_SRC_SENSOR_LIGHT, raw);
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
        SendAppMessage(MSG_SRC_SENSOR_WATER, raw);
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
  AppMsg_t msg;
  char lcd_line1[17];
  char lcd_line2[17];

  // Init LCD
  osMutexAcquire(lcdMutexHandle, osWaitForever);
  lcd_init();
  lcd_clear();
  osMutexRelease(lcdMutexHandle);

  for(;;)
  {
    // Czekaj na wiadomość
    if (osMessageQueueGet(sensorQueueHandle, &msg, NULL, osWaitForever) == osOK)
    {
        // Aktualizacja zmiennych
        switch(msg.source) {
            case MSG_SRC_SENSOR_SOIL:  last_soil = msg.value; break;
            case MSG_SRC_SENSOR_LIGHT: last_light = msg.value; break;
            case MSG_SRC_SENSOR_WATER: last_water = msg.value; break;
            default: break;
        }

        printf("S:%lu L:%lu W:%lu | Mode:%d View:%d P:%d\r\n",
        			last_soil, last_light, last_water, current_mode, current_view, is_pump_on);

        // Obsługa LCD
        osMutexAcquire(lcdMutexHandle, osWaitForever);

        // --- LINIA 1: Dane ---
        lcd_put_cur(0, 0);
        switch(current_view) {
            case VIEW_ALL:
                // Format: "S:1234 L:123 W:12" (ciasno, ale wejdzie)
                // Używamy %4lu ale jeśli liczba mniejsza to spacje.
                // S:%lu zajmie zmienną ilość. Spróbujmy na sztywno:
                // S1234 L1234 W123
                sprintf(lcd_line1, "S%4lu L%4lu W%3lu", last_soil, last_light, last_water);
                break;
            case VIEW_SOIL:
                sprintf(lcd_line1, "Soil:  %-4lu     ", last_soil);
                break;
            case VIEW_WATER:
                sprintf(lcd_line1, "Water: %-4lu     ", last_water);
                break;
            case VIEW_LIGHT:
                sprintf(lcd_line1, "Light: %-4lu     ", last_light);
                break;
        }
        lcd_send_string(lcd_line1);

        // --- LINIA 2: Status ---
        lcd_put_cur(1, 0);

        char pump_char = is_pump_on ? '*' : ' ';
        char *mode_str = "MAN";

        if (current_mode == MODE_AUTO) mode_str = "AUT";
        else if (current_mode == MODE_REMOTE) mode_str = "RMT ";

        if (current_view == VIEW_ALL) {
             // Wersja skrócona bez zbędnych nawiasów
             // M:AUTO P:*
             sprintf(lcd_line2, "M:%s P:[%c]     ", mode_str, pump_char);
        } else {
             // Wersja pełna
             sprintf(lcd_line2, "Mode:%s P:[%c]   ", mode_str, pump_char);
        }
        lcd_send_string(lcd_line2);

        osMutexRelease(lcdMutexHandle);
    }
  }
  /* USER CODE END StartDisplayTask */
}

/* USER CODE BEGIN Header_StartButtonTask */
/**
* @brief Function implementing the ButtonTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartButtonTask */
void StartButtonTask(void *argument)
{
  /* USER CODE BEGIN StartButtonTask */

  // 1. INICJALIZACJA: Upewnij się, że pompa jest wyłączona na starcie
  // Ponieważ używamy Active LOW, SetPumpState(0) ustawi pin w stan WYSOKI (3.3V)
  SetPumpState(0);

  // Stany poprzednie do wykrywania zboczy (1 = przycisk puszczony)
  uint8_t last_btn_view = 1;
  uint8_t last_btn_mode = 1;

  for(;;)
  {
    // 2. Odczyt stanów (Pull-Up: wciśnięty = 0, puszczony = 1)
    uint8_t curr_btn_view = HAL_GPIO_ReadPin(BTN_VIEW_GPIO_Port, BTN_VIEW_Pin);
    uint8_t curr_btn_mode = HAL_GPIO_ReadPin(BTN_MODE_GPIO_Port, BTN_MODE_Pin);
    uint8_t curr_btn_pump = HAL_GPIO_ReadPin(BTN_PUMP_GPIO_Port, BTN_PUMP_Pin);

    // 3. Obsługa PRZYCISKU 1 (Zmiana Widoku) - Zbocze opadające
    if (last_btn_view == 1 && curr_btn_view == 0) {
        current_view++;
        if (current_view > VIEW_LIGHT) current_view = VIEW_ALL;

        // Odśwież LCD natychmiast
        SendAppMessage(MSG_SRC_BUTTON_VIEW, 0);
    }
    last_btn_view = curr_btn_view;

    // 4. Obsługa PRZYCISKU 2 (Zmiana Trybu) - Zbocze opadające
    if (last_btn_mode == 1 && curr_btn_mode == 0) {
        current_mode++;
        if (current_mode > MODE_REMOTE) current_mode = MODE_AUTO;

        // Wyłącz pompę przy każdej zmianie trybu dla bezpieczeństwa
        SetPumpState(0);

        SendAppMessage(MSG_SRC_BUTTON_MODE, 0);
    }
    last_btn_mode = curr_btn_mode;

    // 5. Obsługa PRZYCISKU 3 (Pompa Manualna) - Działa tylko w trybie MANUAL
    if (current_mode == MODE_MANUAL) {
        if (curr_btn_pump == 0) { // Przycisk trzymany (Stan niski)
            if (is_pump_on == 0) SetPumpState(1); // Włącz (logika w funkcji SetPumpState zrobi reset pinu)
        } else { // Przycisk puszczony
            if (is_pump_on == 1) SetPumpState(0); // Wyłącz (ustaw pin)
        }
    } else {
        // Zabezpieczenie: jeśli w innym trybie pompa została włączona, wyłącz ją
        if (is_pump_on && current_mode != MODE_AUTO) SetPumpState(0);
    }

    osDelay(20); // Debouncing (filtracja drgań styków)
  }
  /* USER CODE END StartButtonTask */
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

void SendAppMessage(MsgSource_t src, uint32_t val) {
    AppMsg_t msg;
    msg.source = src;
    msg.value = val;
    osMessageQueuePut(sensorQueueHandle, &msg, 0, 0);
}

void SetPumpState(uint8_t state) {
    if (state == 1) {
        // WŁĄCZANIE: Stan NISKI (GND) uruchamia przekaźnik
        HAL_GPIO_WritePin(RELAY_PUMP_GPIO_Port, RELAY_PUMP_Pin, GPIO_PIN_RESET);
        is_pump_on = 1;
    } else {
        // WYŁĄCZANIE: Stan WYSOKI (3.3V) wyłącza przekaźnik
        HAL_GPIO_WritePin(RELAY_PUMP_GPIO_Port, RELAY_PUMP_Pin, GPIO_PIN_SET);
        is_pump_on = 0;
    }
}

/* USER CODE END Application */
