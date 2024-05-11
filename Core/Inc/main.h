/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

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
#define BIN_BAND1_Pin GPIO_PIN_13
#define BIN_BAND1_GPIO_Port GPIOC
#define BIN_BAND2_Pin GPIO_PIN_14
#define BIN_BAND2_GPIO_Port GPIOC
#define BIN_BAND3_Pin GPIO_PIN_15
#define BIN_BAND3_GPIO_Port GPIOC
#define DS18B20_Pin GPIO_PIN_0
#define DS18B20_GPIO_Port GPIOB
#define Front_Mode_Pin GPIO_PIN_1
#define Front_Mode_GPIO_Port GPIOB
#define Front_Mode_EXTI_IRQn EXTI1_IRQn
#define MCU_LED_Pin GPIO_PIN_2
#define MCU_LED_GPIO_Port GPIOB
#define bar_cs_Pin GPIO_PIN_12
#define bar_cs_GPIO_Port GPIOB
#define bar_clock_Pin GPIO_PIN_13
#define bar_clock_GPIO_Port GPIOB
#define bar_data_Pin GPIO_PIN_14
#define bar_data_GPIO_Port GPIOB
#define Front_stdby_Pin GPIO_PIN_15
#define Front_stdby_GPIO_Port GPIOB
#define BIAS_EN_Pin GPIO_PIN_11
#define BIAS_EN_GPIO_Port GPIOA
#define buzzer_Pin GPIO_PIN_12
#define buzzer_GPIO_Port GPIOA
#define vdd48V_EN_Pin GPIO_PIN_15
#define vdd48V_EN_GPIO_Port GPIOA
#define TX_REL2_Pin GPIO_PIN_4
#define TX_REL2_GPIO_Port GPIOB
#define DATA_Pin GPIO_PIN_6
#define DATA_GPIO_Port GPIOB
#define PTT_Pin GPIO_PIN_7
#define PTT_GPIO_Port GPIOB
#define PTT_EXTI_IRQn EXTI9_5_IRQn
#define CLK_Pin GPIO_PIN_8
#define CLK_GPIO_Port GPIOB
#define SRCLK_Pin GPIO_PIN_9
#define SRCLK_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define MAX_FWD_PWR 400 //Output power limit, here smaller for test
#define MAX_REF_PWR 200  //TODO check later
#define MAX_IN_PWR 8
#define MAX_LPF_LOSS 100 //LPF S21 or reflection from PA_Deck coupler can indicate wrong band selected
#define MAX_Deck_ref 200

#define MAX_TEMP 70 //Maximal transistor package temperature
#define MAX_U 50
#define MAX_I 17 //TODO change for test only

#define MIN_U 42
#define Min_FAN_DT 20

#define ADC_BUFFER_SIZE 8
#define AVG_TIMES 10


void Display_digits(uint8_t x, uint8_t y, uint32_t dispnum); //TODO add other prototypes of functions to reduce warnings
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
