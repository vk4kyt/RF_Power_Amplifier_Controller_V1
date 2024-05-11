/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include <stdbool.h>
/* USER CODE BEGIN Includes */
#include "stm32f1xx_hal.h"
#include "74hc595.h"
#include "onewire.h"
#include "ds18b20.h"
#include "lcd_hd44780_i2c.h"
#include "fault_handler_func.h"
#include "globals.h"
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

/* USER CODE BEGIN PV */
//ADC DMA Buffer
uint16_t ADC_val[ADC_BUFFER_SIZE];
uint16_t ADC_avg_val[ADC_BUFFER_SIZE];

//Input coupler
float in_fwd_u = 0;
float in_ref_u = 0;
float in_fwd_pwr = 0;
float in_ref_pwr = 0;

//Deck coupler
uint16_t deck_fwd_pwr = 0;
uint16_t deck_ref_pwr = 0;
float deck_swr = 0;

//LPF coupler
uint16_t lpf_fwd_pwr = 0;
uint16_t lpf_ref_pwr = 0;
float lpf_swr = 0;

//Coupler voltages
float deck_fwd_u = 0;
float deck_ref_u = 0;
float lpf_fwd_u = 0;
float lpf_ref_u = 0;

//Power bus
float u_48v = 0;
float i_48v = 0;    //drain current
float i_48v_sensor_u = 0; //ACS current sensor output voltage is half the VCC, reading this is better for accuracy in comparison to 5V/2 because error of linear reg and ADC resolution stacks up to offset real result.
float sensor_vcc = 0; //Take at least one reading when PTT is off

//Sensors
float temperature_arr[_DS18B20_MAX_SENSORS];

//float voltage_test;
//float voltageArray[200];
//float temperaturearray[200];
//uint16_t test_counter = 0;

uint16_t fan_tacho1 = 0;
uint16_t fan_tacho2 = 0;

//Band selection
uint8_t  selected_band = 0;
uint8_t  band_mode = 0;	//TODO simulate EEPROM to save last used mode and band
uint8_t	 mode_read = 0;
float band_analogU = 0;
uint16_t band_analogU_mV = 0;
uint16_t band_man_sel = 0;
bool band_binary_sel[3] = {0};

//Flags
uint16_t fault = 0; //Different number can be set by conditional statement and then handled by separate function
uint8_t bias_flag = 0;
uint8_t ptt_status = 0;
uint8_t stdby_flag = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

void Enable_bias(){
	HAL_GPIO_WritePin(BIAS_EN_GPIO_Port, BIAS_EN_Pin, GPIO_PIN_SET);
	//bias_flag = 1;
}

void Disable_bias(){
	HAL_GPIO_WritePin(BIAS_EN_GPIO_Port, BIAS_EN_Pin, GPIO_PIN_RESET);
	//bias_flag = 0;
}


int mapRange(int input, int inMin, int inMax, int outMin, int outMax){
	return ((input - inMin) * (outMax - outMin) / (inMax - inMin) + outMin);
}

uint32_t linspace(double start, double end, int numPoints){

   uint32_t* array = (double*)malloc(numPoints * sizeof(double));
    uint8_t step = (end - start) / (numPoints - 1);

    for (int i = 0; i < numPoints; i++) {
        array[i] = start + i * step;
    }

    return array;
}

void Switch_band(uint8_t band){
	ShiftRegister74HC595_setAll(LOW);
	ShiftRegister74HC595_update();

    switch (band) {
        case 1:
            ShiftRegister74HC595_setPin(6, HIGH);
            ShiftRegister74HC595_update();
            break;
        case 2:
            ShiftRegister74HC595_setPin(5, HIGH);
            ShiftRegister74HC595_update();
            break;
        case 3:
            ShiftRegister74HC595_setPin(4, HIGH);
            ShiftRegister74HC595_update();
            break;
        case 4:
            ShiftRegister74HC595_setPin(3, HIGH);
            ShiftRegister74HC595_update();
            break;
        case 5:
            ShiftRegister74HC595_setPin(2, HIGH);
            ShiftRegister74HC595_update();
            break;
        case 6:
            ShiftRegister74HC595_setPin(1, HIGH);
            ShiftRegister74HC595_update();
        default:
            //Fault_handler()
        	break;
    }

}

uint8_t decode_binary_band(bool bin_array[]){ //TODO test this function
    int decimalValue = 0;


    decimalValue = band_binary_sel[0] * 4 + band_binary_sel[1] * 2 + band_binary_sel[2];

        switch (decimalValue) {
            case 0:
                //000
                break;
            case 1:
                //001
                break;
            case 2:
                //010
                break;
            case 3:
                //011
                break;
            case 4:
                //100
                break;
            case 5:
                //101
                break;
            case 6:
                //110
                break;
            case 7:
                //111
                break;
            default:

                break;
        }

        return decimalValue;

    /*
    for (int i = 0; i < 3; ++i) {
        if (bin_array[i]) {
            decimalValue += (1 << (2 - i)); // Convert binary to decimal
        }
    }

    return decimalValue + 1; //get value in range
    */
}


//Important conditions are set by interrupts
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == 2){ // External interrupt from mode button on the front panel
		mode_read ++; //Works as counter then it can be determined how long the interrupt pulse was?
	    if (band_mode <= 3) {
	        band_mode++; // Change mode on every press of a button
	    }
	    if (band_mode > 3) {
	        band_mode = 1; // Roll over to the first mode after three presses
	    }
	}

    if(GPIO_Pin == 128) {
        ptt_status = 1;
    }

    if(GPIO_Pin = 1){
    	//TODO replace with HAL read in task, bistable to monostable circuitry can be added to tact switch.
    	//stdby_flag = 1; //this will require something to set it back to zero after second press, use similar logic to mode but 2 states?
    }
    else {
    	//Undefined interrupt number

    }

}

uint32_t roundToDecimalPlaces(double number, int decimalPlaces) {
	uint32_t multiplier = pow(10.0, decimalPlaces);
	uint32_t roundedNumber = round(number * multiplier) / multiplier;
    return roundedNumber;
}




void Display_digits(uint8_t x, uint8_t y, uint32_t dispnum){
	//Displays number as a string of digits, x is column y is row of a display
	char str[4];

	lcdSetCursorPosition(x, y);

	snprintf(str, sizeof(str), "%d", dispnum);

	if(dispnum >= 100){
		lcdPrintStr((uint8_t*) str, 3);
	}
	if((dispnum < 100) && (dispnum >= 10)){
		lcdPrintStr((uint8_t*) str, 2);
	}
	if((dispnum < 10)){
		lcdPrintStr((uint8_t*) str, 1);
	}
}




/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET); //Set PSU fan to low

  HAL_Delay(100);



  DS18B20_Init(DS18B20_Resolution_12bits);
  HAL_GPIO_WritePin(buzzer_GPIO_Port, buzzer_Pin, SET);
  HAL_Delay(1000);
  HAL_GPIO_WritePin(buzzer_GPIO_Port, buzzer_Pin, RESET);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */


  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
