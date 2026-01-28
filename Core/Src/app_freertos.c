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
#include "usart.h"
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
extern osMutexId_t adcMutexHandle;
extern osMutexId_t lcdMutexHandle;

// Kolejki zdefiniowane w CubeMX
extern osMessageQueueId_t sensorQueueHandle;
extern osMessageQueueId_t commandQueueHandle;

extern UART_HandleTypeDef huart1;

// Bufor na jeden znak odebrany z klawiatury
uint8_t rx_char;

// --- GLOBALNY STAN SYSTEMU ---
// ControlTask tu pisze, DisplayTask to czyta.
typedef struct {
    uint32_t soil;
    uint32_t light;
    uint32_t water;
    ControlMode_t mode;
    DisplayView_t view;
    uint8_t pump_active;
} SystemState_t;

// Inicjalizacja stanu zerowego
volatile SystemState_t sysState = {
    .soil = 0, .light = 0, .water = 0,
    .mode = MODE_MANUAL,
    .view = VIEW_ALL,
    .pump_active = 0
};

// Progi dla automatyki
#define SOIL_DRY_THRESHOLD 1500  // Poniżej tego włączamy pompę
#define SOIL_WET_THRESHOLD 1800  // Powyżej tego wyłączamy (histereza)
#define WATER_MIN_LEVEL 200      // Minimum wody w zbiorniku

#define SRC_ID_BUTTON 1
#define SRC_ID_REMOTE 2

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
/* Definitions for ControlTask */
osThreadId_t ControlTaskHandle;
const osThreadAttr_t ControlTask_attributes = {
  .name = "ControlTask",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = 256 * 4
};
/* Definitions for CommTask */
osThreadId_t CommTaskHandle;
const osThreadAttr_t CommTask_attributes = {
  .name = "CommTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 256 * 4
};
/* Definitions for sensorQueue */
osMessageQueueId_t sensorQueueHandle;
const osMessageQueueAttr_t sensorQueue_attributes = {
  .name = "sensorQueue"
};
/* Definitions for commandQueue */
osMessageQueueId_t commandQueueHandle;
const osMessageQueueAttr_t commandQueue_attributes = {
  .name = "commandQueue"
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
void SendSensorData(MsgSource_t src, uint32_t val);
void SendCommand(MsgSource_t src, uint32_t val);
void SetPumpHardware(uint8_t state);
/* USER CODE END FunctionPrototypes */

void StartSoilTask(void *argument);
void StartLightTask(void *argument);
void StartWaterTask(void *argument);
void StartDisplayTask(void *argument);
void StartButtonTask(void *argument);
void StartControlTask(void *argument);
void StartCommTask(void *argument);

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

  /* creation of commandQueue */
  commandQueueHandle = osMessageQueueNew (8, 8, &commandQueue_attributes);

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

  /* creation of ControlTask */
  ControlTaskHandle = osThreadNew(StartControlTask, NULL, &ControlTask_attributes);

  /* creation of CommTask */
  CommTaskHandle = osThreadNew(StartCommTask, NULL, &CommTask_attributes);

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
        SendSensorData(MSG_SRC_SENSOR_SOIL, raw);
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
        SendSensorData(MSG_SRC_SENSOR_LIGHT, raw);
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
        SendSensorData(MSG_SRC_SENSOR_WATER, raw);
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
  char lcd_line1[17];
  char lcd_line2[17];

  // Init LCD
  osMutexAcquire(lcdMutexHandle, osWaitForever);
  lcd_init();
  lcd_clear();
  osMutexRelease(lcdMutexHandle);

  for(;;)
  {
    // 1. Logowanie UART (Co odświeżenie ekranu)
    // Przywrócony printf z \r\n na końcu
    printf("S:%lu L:%lu W:%lu | Mode:%d View:%d P:%d\r\n",
           sysState.soil, sysState.light, sysState.water,
           sysState.mode, sysState.view, sysState.pump_active);

    // 2. Obsługa LCD
    osMutexAcquire(lcdMutexHandle, osWaitForever);

    // --- LINIA 1: Dane ---
    lcd_put_cur(0, 0);
    switch(sysState.view) {
        case VIEW_ALL:
            // S1234 L1234 W123
            sprintf(lcd_line1, "S%4lu L%4lu W%3lu", sysState.soil, sysState.light, (sysState.water > 999 ? 999 : sysState.water));
            break;
        case VIEW_SOIL:
            sprintf(lcd_line1, "Soil:  %-4lu     ", sysState.soil);
            break;
        case VIEW_WATER:
            sprintf(lcd_line1, "Water: %-4lu     ", sysState.water);
            break;
        case VIEW_LIGHT:
            sprintf(lcd_line1, "Light: %-4lu     ", sysState.light);
            break;
    }
    lcd_send_string(lcd_line1);

    // --- LINIA 2: Status ---
    lcd_put_cur(1, 0);

    char *mode_str = "MAN";
    if (sysState.mode == MODE_AUTO) mode_str = "AUT";
    else if (sysState.mode == MODE_REMOTE) mode_str = "RMT";

    char pump_char = sysState.pump_active ? '*' : ' ';

    if (sysState.view == VIEW_ALL) {
         // Powrót do Twojego formatu: M:MAN P:[ ]
         sprintf(lcd_line2, "M:%s P:[%c]     ", mode_str, pump_char);
    } else {
         // Pełny format dla widoków szczegółowych
         sprintf(lcd_line2, "Mode:%s P:[%c]   ", mode_str, pump_char);
    }
    lcd_send_string(lcd_line2);

    osMutexRelease(lcdMutexHandle);

    osDelay(500); // Odświeżanie 2Hz (optymalne dla UART i oczu)
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
/* USER CODE BEGIN Header_StartButtonTask */
void StartButtonTask(void *argument)
{
  /* USER CODE BEGIN StartButtonTask */
  SetPumpHardware(0);
  uint8_t last_btn_view = 1;
  uint8_t last_btn_mode = 1;
  uint8_t last_btn_pump = 1;

  for(;;)
  {
    uint8_t curr_btn_view = HAL_GPIO_ReadPin(BTN_VIEW_GPIO_Port, BTN_VIEW_Pin);
    uint8_t curr_btn_mode = HAL_GPIO_ReadPin(BTN_MODE_GPIO_Port, BTN_MODE_Pin);
    uint8_t curr_btn_pump = HAL_GPIO_ReadPin(BTN_PUMP_GPIO_Port, BTN_PUMP_Pin);

    // 1. Zmiana widoku
    if (last_btn_view == 1 && curr_btn_view == 0) {
        SendCommand(MSG_CMD_CHANGE_VIEW, 0);
    }
    last_btn_view = curr_btn_view;

    // 2. Zmiana trybu
    if (last_btn_mode == 1 && curr_btn_mode == 0) {
        SendCommand(MSG_CMD_CHANGE_MODE, 0);
    }
    last_btn_mode = curr_btn_mode;

    // 3. Sterowanie manualne (Podpisujemy jako SRC_ID_BUTTON)
    if (curr_btn_pump != last_btn_pump) {
        if (curr_btn_pump == 0) {
            SendCommand(MSG_CMD_PUMP_REQ_ON, SRC_ID_BUTTON); // <--- ZMIANA
        } else {
            SendCommand(MSG_CMD_PUMP_REQ_OFF, SRC_ID_BUTTON); // <--- ZMIANA
        }
        last_btn_pump = curr_btn_pump;
    }

    osDelay(20);
  }
  /* USER CODE END StartButtonTask */
}

/* USER CODE BEGIN Header_StartControlTask */
/**
* @brief Function implementing the ControlTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartControlTask */
/* USER CODE BEGIN Header_StartControlTask */
void StartControlTask(void *argument)
{
  /* USER CODE BEGIN StartControlTask */
  AppMsg_t msg;
  SetPumpHardware(0);

  for(;;)
  {
    // A. Komendy
    if (osMessageQueueGet(commandQueueHandle, &msg, NULL, 0) == osOK) {
        switch(msg.source) {
            case MSG_CMD_CHANGE_VIEW:
                sysState.view++;
                if (sysState.view > VIEW_LIGHT) sysState.view = VIEW_ALL;
                break;

            case MSG_CMD_CHANGE_MODE:
                sysState.mode++;
                if (sysState.mode > MODE_REMOTE) sysState.mode = MODE_MANUAL;

                // Zawsze resetuj pompę przy zmianie trybu
                SetPumpHardware(0);
                sysState.pump_active = 0;
                break;

            case MSG_CMD_PUMP_REQ_ON:
                // Logika:
                // - Jeśli tryb MANUAL -> akceptuj tylko od PRZYCISKU (SRC_ID_BUTTON)
                // - Jeśli tryb REMOTE -> akceptuj tylko od UART (SRC_ID_REMOTE)
                if ( (sysState.mode == MODE_MANUAL && msg.value == SRC_ID_BUTTON) ||
                     (sysState.mode == MODE_REMOTE && msg.value == SRC_ID_REMOTE) )
                {
                    SetPumpHardware(1);
                    sysState.pump_active = 1;
                }
                break;

            case MSG_CMD_PUMP_REQ_OFF:
                // Ta sama logika dla wyłączania
                if ( (sysState.mode == MODE_MANUAL && msg.value == SRC_ID_BUTTON) ||
                     (sysState.mode == MODE_REMOTE && msg.value == SRC_ID_REMOTE) )
                {
                    SetPumpHardware(0);
                    sysState.pump_active = 0;
                }
                break;

            default: break;
        }
    }

    // B. Sensory (bez zmian)
    if (osMessageQueueGet(sensorQueueHandle, &msg, NULL, 5) == osOK) {
        switch(msg.source) {
            case MSG_SRC_SENSOR_SOIL:  sysState.soil = msg.value; break;
            case MSG_SRC_SENSOR_LIGHT: sysState.light = msg.value; break;
            case MSG_SRC_SENSOR_WATER: sysState.water = msg.value; break;
            default: break;
        }
    }

    // C. Logika AUTO (bez zmian)
    if (sysState.mode == MODE_AUTO) {
        if (sysState.soil < SOIL_DRY_THRESHOLD && sysState.water > WATER_MIN_LEVEL) {
            if (sysState.pump_active == 0) {
                SetPumpHardware(1);
                sysState.pump_active = 1;
            }
        }
        else if (sysState.soil > SOIL_WET_THRESHOLD || sysState.water < WATER_MIN_LEVEL) {
            if (sysState.pump_active == 1) {
                SetPumpHardware(0);
                sysState.pump_active = 0;
            }
        }
    }

    osDelay(10);
  }
  /* USER CODE END StartControlTask */
}

/* USER CODE BEGIN Header_StartCommTask */
/**
* @brief Function implementing the CommTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCommTask */
void StartCommTask(void *argument)
{
  /* USER CODE BEGIN StartCommTask */

  // 1. Start nasłuchu na przerwanie (używamy poprawnej nazwy zmiennej 'rx_char')
  HAL_UART_Receive_IT(&huart1, &rx_char, 1);

  for(;;)
  {
    // 2. Czekaj na flagę od przerwania
    osThreadFlagsWait(0x01, osFlagsWaitAny, osWaitForever);

    // 3. Logika komend
    // Wysyłamy komendę z podpisem SRC_ID_REMOTE (2)
    if (rx_char == '1') {
        printf("\r\nREMOTE: Sent ON request\r\n");
        SendCommand(MSG_CMD_PUMP_REQ_ON, SRC_ID_REMOTE);
    }
    else if (rx_char == '0') {
        printf("\r\nREMOTE: Sent OFF request\r\n");
        SendCommand(MSG_CMD_PUMP_REQ_OFF, SRC_ID_REMOTE);
    }
    // Opcjonalnie: ignoruj inne znaki (np. Enter)

    // 4. Wznów nasłuch (ponownie używamy poprawnej nazwy 'rx_char')
    HAL_UART_Receive_IT(&huart1, &rx_char, 1);
  }
  /* USER CODE END StartCommTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void ADC_Select_Channel(uint32_t channel) {
  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel = channel;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_160CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) { Error_Handler(); }
}

// Wysyła dane do kolejki czujników
void SendSensorData(MsgSource_t src, uint32_t val) {
    AppMsg_t msg;
    msg.source = src;
    msg.value = val;
    osMessageQueuePut(sensorQueueHandle, &msg, 0, 0);
}

// Wysyła dane do kolejki komend (przyciski)
void SendCommand(MsgSource_t src, uint32_t val) {
    AppMsg_t msg;
    msg.source = src;
    msg.value = val;
    osMessageQueuePut(commandQueueHandle, &msg, 0, 0);
}

// Sterownik sprzętowy pompy (tylko włącz/wyłącz pin)
void SetPumpHardware(uint8_t state) {
    if (state == 1) {
        // Active LOW (Włącz)
        HAL_GPIO_WritePin(RELAY_PUMP_GPIO_Port, RELAY_PUMP_Pin, GPIO_PIN_RESET);
    } else {
        // Active LOW (Wyłącz)
        HAL_GPIO_WritePin(RELAY_PUMP_GPIO_Port, RELAY_PUMP_Pin, GPIO_PIN_SET);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    // Sprawdzamy, czy przerwanie przyszło z UART1
    if (huart->Instance == USART1) {
        // Wyślij sygnał (flaga nr 1) do CommTask, żeby go obudzić
        osThreadFlagsSet(CommTaskHandle, 0x01);
    }
}
/* USER CODE END Application */
