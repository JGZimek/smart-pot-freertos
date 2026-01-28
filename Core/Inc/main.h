/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32wlxx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
// Typy źródeł danych dla kolejki
typedef enum {
    MSG_SRC_SENSOR_SOIL,
    MSG_SRC_SENSOR_LIGHT,
    MSG_SRC_SENSOR_WATER,
    MSG_SRC_BUTTON_VIEW,  // Nowe: Odśwież bo zmieniono widok
    MSG_SRC_BUTTON_MODE   // Nowe: Odśwież bo zmieniono tryb
} MsgSource_t;

// Struktura wiadomości w kolejce
typedef struct {
    MsgSource_t source;
    uint32_t value; // Dla sensorów to pomiar, dla przycisków może być puste
} AppMsg_t;

// --- DEFINICJE STANÓW SYSTEMU ---
typedef enum {
    VIEW_ALL,   // Pokazuj wszystko (jak teraz)
    VIEW_SOIL,  // Tylko gleba
    VIEW_WATER, // Tylko woda
    VIEW_LIGHT  // Tylko światło
} DisplayView_t;

typedef enum {
    MODE_AUTO,   // Automat decyduje o podlewaniu
    MODE_MANUAL, // Tylko przycisk
    MODE_REMOTE  // Czekamy na komendy z LoRa/Zigbee (na przyszłość)
} ControlMode_t;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BTN_VIEW_Pin GPIO_PIN_0
#define BTN_VIEW_GPIO_Port GPIOA
#define BTN_MODE_Pin GPIO_PIN_3
#define BTN_MODE_GPIO_Port GPIOA
#define BTN_PUMP_Pin GPIO_PIN_2
#define BTN_PUMP_GPIO_Port GPIOA
#define RELAY_PUMP_Pin GPIO_PIN_10
#define RELAY_PUMP_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
