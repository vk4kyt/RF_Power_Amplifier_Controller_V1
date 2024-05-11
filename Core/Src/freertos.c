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
#include "stm32f1xx_hal.h"
#include "74hc595.h"
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "gpio.h"
#include "semphr.h"
#include "onewire.h"
#include "ds18b20.h"
#include "globals.h"
#include "i2c.h"
#include "math.h"

#include "lcd_hd44780_i2c.h"
#include "fault_handler_func.h"
//#include "liquidcrystal_i2c.h"

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

float temperature;

float Calculate_SWR(uint16_t fwd_pwr, uint16_t ref_pwr){
	if((fwd_pwr == 0) || (ref_pwr == 0)){
		return 1;
	}

	float swr = 0;
	swr = (1 + sqrt(ref_pwr / fwd_pwr)) / (1  - sqrt(ref_pwr / fwd_pwr));
	return swr;
}

void Display_float_digit(uint8_t x, uint8_t y, float dispnum){
	if(dispnum < 100){
	    int integerPart;
	    int decimalPart;

	    //splitFloat(dispnum, &integerPart, &decimalPart);

	    integerPart = (int) dispnum;
	    decimalPart = (int)((dispnum - integerPart) * 100);

		char str_int[3];
		char str_dec[3];
		lcdCursorHome();
		lcdSetCursorPosition(x, y);
		//snprintf(str_int, sizeof(str_int), "%d", integerPart);

		if(dispnum == 100){
			lcdPrintStr((uint8_t*) str_int, 3);
		}
		if((dispnum < 100) && (dispnum >= 10)){
			//lcdPrintStr((uint8_t*) str_int, 2);
			lcdSetCursorPosition(x+2, y);
			lcdPrintStr((uint8_t*) ".", 1);

			decimalPart = decimalPart/10;
			//decimalPart = roundToDecimalPlaces(decimalPart, 1);
			snprintf(str_dec, sizeof(str_dec), "%d", decimalPart);

			lcdSetCursorPosition(x+3, y);
			lcdPrintStr((uint8_t*) str_dec, 1);
		}
		if((dispnum < 10) && (dispnum >= 1)){
			//lcdPrintStr((uint8_t*) str_int[0], 1);
			lcdSetCursorPosition(x+1, y);
			lcdPrintStr((uint8_t*) ".", 1);

			decimalPart = (int)((dispnum - integerPart) * 100);
			//decimalPart = roundToDecimalPlaces(decimalPart, 2);
			snprintf(str_dec, sizeof(str_dec), "%d", decimalPart);

			lcdSetCursorPosition(x+2, y);
			lcdPrintStr((uint8_t*) str_dec, 2);
		}
		if(dispnum < 1){
			lcdPrintStr((uint8_t*) "0.", 2);

			decimalPart = (int)((dispnum - integerPart) * 100);
			//decimalPart = roundToDecimalPlaces(decimalPart, 3);
			snprintf(str_dec, sizeof(str_dec), "%d", decimalPart);

			lcdSetCursorPosition(x+2, y);
			lcdPrintStr((uint8_t*) str_dec, 2);
		}




	}
	Display_digits(x, y, (int) dispnum);
}


void PSU_Set_Fan_Speed(float temperature, uint16_t minPWM, uint16_t maxPWM) {
    // Calculate the desired PWM value based on the temperature
    // You would need to implement your own logic to map temperature to PWM
    // For the sake of example, let's assume a linear mapping
	uint16_t max_temp = 60;
	uint16_t min_temp = 0;

    uint16_t temperatureRange = max_temp - min_temp;
    uint16_t pwmRange = maxPWM - minPWM;
    uint16_t pwmPercentage = ((temperature - min_temp) / temperatureRange) * 100.0;
    uint16_t pwmValue = (uint16_t)((pwmPercentage / 100.0) * pwmRange) + minPWM;

    // Ensure the PWM value is within bounds
    if (pwmValue < minPWM) {
        pwmValue = minPWM;
    } else if (pwmValue > maxPWM) {
        pwmValue = maxPWM;
    }

    // Update the PWM duty cycle
    TIM1->CCR2 = pwmValue;  // Assuming you're controlling PWM on channel 2 of TIM1
}

void HS_Set_Fan_Speed(float temperature, uint16_t minPWM, uint16_t maxPWM) {
	    // Calculate the desired PWM value based on the temperature
	    // You would need to implement your own logic to map temperature to PWM
	    // For the sake of example, let's assume a linear mapping
		uint16_t max_temp = 60;
		uint16_t min_temp = 0;
		uint16_t temperatureRange = max_temp - min_temp;
		uint16_t pwmRange = maxPWM - minPWM;
		uint16_t pwmPercentage = 100.0 - ((temperature - min_temp) / temperatureRange) * 100.0;
	    uint16_t pwmValue = (uint16_t)((pwmPercentage / 100.0) * pwmRange) + minPWM;

	    // Ensure the PWM value is within bounds
	    if (pwmValue < minPWM) {
	        pwmValue = minPWM;
	    } else if (pwmValue > maxPWM) {
	        pwmValue = maxPWM;
	    }

	    // Update the specified PWM duty cycle (CCR)
	    TIM1->CCR1 = pwmValue;  // Assuming you're controlling PWM on channel 2 of TIM1
	}

void write_byte(uint8_t byte)
{
  for(int i=0;i<8;i++)
  {
     HAL_GPIO_WritePin(bar_clock_GPIO_Port, bar_clock_Pin, 0);  // Pull the CLK LOW
     HAL_GPIO_WritePin(bar_data_GPIO_Port, bar_data_Pin, byte&0x80);// Write one BIT data MSB first
     byte = byte<<1;  // shift the data to the left
     HAL_GPIO_WritePin(bar_clock_GPIO_Port, bar_clock_Pin, 1);  // Pull the CLK HIGH
  }
}

void write_max (uint8_t address, uint8_t data)
{
    HAL_GPIO_WritePin(bar_cs_GPIO_Port, bar_cs_Pin, 0);   // Pull the CS pin LOW
    write_byte(address);   //  write address
    write_byte(data);  //  write data
    HAL_GPIO_WritePin(bar_cs_GPIO_Port, bar_cs_Pin, 1);  // pull the CS HIGH
}

void max_init(void)
{
 write_max(0x09, 0x00);       //  no decoding
 write_max(0x0a, 0x03);       //  brightness intensity
 write_max(0x0b, 0x07);       //  scan limit = 8 LEDs
 write_max(0x0c, 0x01);       //  power down =0，normal mode = 1
 write_max(0x0f, 0x00);       //  no test display
}

void display_pixels_consecutively() {
    for (int row = 0; row < 8; row++) {
        for (int col = 0; col < 8; col++) {
            uint8_t columnData = 0;
            columnData |= (1 << col); // Set the bit for the current column

            // Clear the previous column
            for (int clearCol = 0; clearCol < 8; clearCol++) {
                write_max(clearCol + 1, 0); // Clear column data in the row registers
            }

            // Write the new column data
            write_max(row + 1, columnData);
            HAL_Delay(100); // Delay for visualization (adjust as needed)
        }
    }
}


void bar_graph_power(uint8_t value) {
    if (value > 64) {
        value = 64; // Limit the value to the number of LEDs available (8x8 matrix)
    }

    uint8_t matrixBuffer[8] = {0}; // Initialize the matrix buffer

    // Calculate the number of full rows and the remaining pixel
    uint8_t fullRows = value / 8;
    uint8_t remainingPixels = value % 8;

    // Set the pixels in the full rows
    for (int row = 0; row < fullRows; row++) {
        matrixBuffer[row] = 0xFF; // Set all pixels in the row
    }

    // Set the remaining pixels in the next row
    matrixBuffer[fullRows] = (0xFF >> (8 - remainingPixels));

    // Update the LED matrix with the buffer
    for (int row = 0; row < 8; row++) {
        write_max(row + 1, matrixBuffer[row]); // Display the row data
    }
}



void split_bar_graph(uint8_t section1Value, uint8_t section2Value, uint8_t section3Value) {
    uint8_t matrixBuffer[8] = {0}; // Initialize the matrix buffer

    // Set the pixels in the first section
    for (int i = 0; i < section1Value; i++) {
        int row = i / 8;
        int col = i % 8;
        matrixBuffer[row] |= (1 << col);
    }

    // Set the pixels in the second section
    for (int i = 0; i < section2Value; i++) {
        int row = (i + 32) / 8;
        int col = (i + 32) % 8;
        matrixBuffer[row] |= (1 << col);
    }

    // Set the pixels in the third section
    for (int i = 0; i < section3Value; i++) {
        int row = (i + 48) / 8;
        int col = (i + 48) % 8;
        matrixBuffer[row] |= (1 << col);
    }

    // Update the LED matrix with the buffer
    for (int row = 0; row < 8; row++) {
        write_max(row + 1, matrixBuffer[row]); // Display the row data
    }
}

float Map_float(float val, float I_Min, float I_Max, float O_Min, float O_Max){
		return(((val-I_Min)*((O_Max-O_Min)/(I_Max-I_Min)))+O_Min);
   }


float convertADCtoCurrent(uint16_t ADC_avg_val) {
    // Calculate the voltage from the ADC reading
    float voltage = ((float)ADC_avg_val / 4095) * 3.3;

    // Calculate the voltage offset (Vcc/2) due to zero current
    float zeroCurrentVoltage = 2.5;

    // Calculate the voltage difference from zero current
    float voltageDifference = voltage - zeroCurrentVoltage;

    // Convert the voltage difference to current (in Amperes)
    float current = voltageDifference / (66 / 1000.0) + 0.75; // Convert mV to V

    return current;
}

float adcValueToVoltage(uint16_t adcValue) {
    // Assuming a 3.3V reference voltage and a 12-bit ADC
    float referenceVoltage = 3.3;
    uint16_t maxADCValue = 4095;

    // Calculate the voltage using the formula: voltage = (adcValue / maxADCValue) * referenceVoltage
    float voltage = (float)adcValue / (float)maxADCValue * referenceVoltage;

    return voltage;
}

uint8_t Map_voltage_band(uint32_t analog_U){
	if(analog_U <= 250){
		return 1;	//160 m
	}
	if((analog_U > 250) && (analog_U <= 500)){
		return 2;	//80 m
	}
	if((analog_U > 750) && (analog_U <= 1250)){
		return 3;	//40-30 m
	}
	if((analog_U > 1250) && (analog_U <= 1650)){
		return 4;	//20-17 m
	}
	if((analog_U > 1650) && (analog_U <= 2500)){
		return 5;	//15-10 m
	}
	if(analog_U > 2500){
		return 6;	//6 m
	}


}




/* USER CODE END Variables */
/* Definitions for Measure_ADC_Tas */
osThreadId_t Measure_ADC_TasHandle;
const osThreadAttr_t Measure_ADC_Tas_attributes = {
  .name = "Measure_ADC_Tas",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for TXRX_Task */
osThreadId_t TXRX_TaskHandle;
const osThreadAttr_t TXRX_Task_attributes = {
  .name = "TXRX_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for LPF_Task */
osThreadId_t LPF_TaskHandle;
const osThreadAttr_t LPF_Task_attributes = {
  .name = "LPF_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for Display_Task */
osThreadId_t Display_TaskHandle;
const osThreadAttr_t Display_Task_attributes = {
  .name = "Display_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Measure_ADC_Task(void *argument);
void Start_Task_TXRX(void *argument);
void Start_LPF_Task(void *argument);
void Start_Display_Task(void *argument);

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
  /* creation of Measure_ADC_Tas */
  Measure_ADC_TasHandle = osThreadNew(Measure_ADC_Task, NULL, &Measure_ADC_Tas_attributes);

  /* creation of TXRX_Task */
  TXRX_TaskHandle = osThreadNew(Start_Task_TXRX, NULL, &TXRX_Task_attributes);

  /* creation of LPF_Task */
  LPF_TaskHandle = osThreadNew(Start_LPF_Task, NULL, &LPF_Task_attributes);

  /* creation of Display_Task */
  Display_TaskHandle = osThreadNew(Start_Display_Task, NULL, &Display_Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_Measure_ADC_Task */
/**
  * @brief  Function implementing the Measure_ADC_Tas thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Measure_ADC_Task */
void Measure_ADC_Task(void *argument)
{
  /* USER CODE BEGIN Measure_ADC_Task */
	  uint32_t temp[ADC_BUFFER_SIZE] = {0};
	  uint16_t counter = 0; 		//Keep track for average for ADC
	  uint16_t counter_temp = 0;	//Read temperature only when counter is reached

		SemaphoreHandle_t xSemaphore;
		xSemaphore = xSemaphoreCreateMutex();

	  //Start DMA reading to a buffer
	  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_val, ADC_BUFFER_SIZE);

	  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

  /* Infinite loop */
  for(;;)
  {


	//Calculate average of readings
	  //Note, can remove noise but M3 does not have an FPU so small amount of averaging is used.
	for(int i = 0; i < ADC_BUFFER_SIZE; i++){
		temp[i] += ADC_val[i];
	}
	counter++;

	if(counter == AVG_TIMES){

		for(int i = 0; i < ADC_BUFFER_SIZE; i++){
			ADC_avg_val[i] = temp[i]/AVG_TIMES;
			temp[i] = 0;
		}
		counter = 0;

		//Map the measured values to global variables
		band_analogU = adcValueToVoltage(ADC_avg_val[0]);
		band_analogU_mV =  1000 * band_analogU;

		deck_fwd_u = adcValueToVoltage(ADC_avg_val[2]);
		deck_fwd_pwr = (22.8 * pow(deck_fwd_u, 2)) + (10.13 * deck_fwd_u) - 8.54; //Apply trend line measured from coupler


		//deck_fwd_pwr = mapRange(ADC_avg_val[2], 0, 4096, 0, 600);
		deck_ref_pwr = mapRange(ADC_avg_val[1], 0, 4096, 0, 600); //TODO calibrate with known mismatch later


		//lpf_fwd_pwr = mapRange(ADC_avg_val[3], 0, 4096, 0, 600);
		lpf_fwd_u = adcValueToVoltage(ADC_avg_val[3]);
		lpf_fwd_pwr = (113 * pow(lpf_fwd_u, 2)) - (11.5 * lpf_fwd_u) + 11.6;

		lpf_ref_pwr = mapRange(ADC_avg_val[4], 0, 4096, 0, 600);

		u_48v = 18.837 * adcValueToVoltage(ADC_avg_val[5]); //Map_float(ADC_avg_val[5], 0, 4096, 0, 60);
		//i_48v = Map_float(ADC_avg_val[6], 1948, 4096, 0, 30); //seems that small change in 5v rail can offset this result. 2048 because output is Vcc/2 + 66 mV/A
		//i_48v = convertADCtoCurrent(ADC_avg_val[6]);

		if(ptt_status == 0){
			sensor_vcc = adcValueToVoltage(ADC_avg_val[6]);
		}

		i_48v_sensor_u = adcValueToVoltage(ADC_avg_val[6]);
		i_48v = (i_48v_sensor_u - sensor_vcc) / 0.066;

		in_fwd_u = adcValueToVoltage(ADC_avg_val[7]);
		in_fwd_pwr = (6.23 * pow(in_fwd_u, 2)) + (1.24 * in_fwd_u);

	}


	lpf_swr = Calculate_SWR(lpf_fwd_pwr, lpf_ref_pwr);
	deck_swr = Calculate_SWR(deck_ref_pwr, deck_ref_pwr);

	//Reading temperature
	if(counter_temp > 100){
		//This delays reading time for one wire to say in spec, proper way would be to use timer for interrupt and callback, alternatively another rtos task on better mcu
		  DS18B20_ReadAll();
		  DS18B20_StartAll();
		  uint8_t ROM_tmp[8];
		  uint8_t i;

		    for (uint8_t i = 0; i < DS18B20_Quantity(); i++)
		    {
		        if (DS18B20_GetTemperature(i, &temperature_arr[i]))
		        {
		            DS18B20_GetROM(i, ROM_tmp);
		        }
		    }
		  counter_temp = 0;
		  HS_Set_Fan_Speed(temperature_arr[1], 600, 1000);
		  PSU_Set_Fan_Speed(temperature_arr[1], 220, 650);

	}
	counter_temp++;

	//voltage_test = adcValueToVoltage(ADC_avg_val[1]);


	stdby_flag = HAL_GPIO_ReadPin(Front_stdby_GPIO_Port, Front_stdby_Pin);

	if(i_48v > MAX_I){
		fault_handler(1);
	}
	if(u_48v > MAX_U){
		fault_handler(2);
	}
	if ((temperature_arr[1] > MAX_TEMP) || (temperature_arr[0] > MAX_TEMP)) {
	    fault_handler(3);
	}
	if(in_fwd_pwr > MAX_IN_PWR){
		fault_handler(5);
	}
	if(deck_fwd_pwr > MAX_FWD_PWR){
		fault_handler(6);
	}
	if(deck_ref_pwr > MAX_Deck_ref){
		fault_handler(7);
	}
	if(deck_ref_pwr > MAX_REF_PWR){
		fault_handler(8);
	}
	if((deck_fwd_pwr - lpf_fwd_pwr) > MAX_LPF_LOSS){
		fault_handler(9);
	}


    osDelay(1);
  }
  /* USER CODE END Measure_ADC_Task */
}

/* USER CODE BEGIN Header_Start_Task_TXRX */
/**
* @brief Function implementing the TXRX_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Task_TXRX */
void Start_Task_TXRX(void *argument)
{
  /* USER CODE BEGIN Start_Task_TXRX */
	uint8_t ptt_read = 0;

	HAL_GPIO_WritePin(vdd48V_EN_GPIO_Port, vdd48V_EN_Pin, GPIO_PIN_RESET);
	bias_flag = 0;
	osDelay(1000); //This gives time for measurement to kick in etc...



  /* Infinite loop */
  for(;;)
  {
	if(ptt_status == 1){ //Switch to TX
		HAL_GPIO_WritePin(MCU_LED_GPIO_Port, MCU_LED_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(TX_REL2_GPIO_Port, TX_REL2_Pin, GPIO_PIN_SET);
		vTaskSuspend(LPF_TaskHandle);
		osDelay(100);
		HAL_GPIO_WritePin(BIAS_EN_GPIO_Port, BIAS_EN_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(vdd48V_EN_GPIO_Port, vdd48V_EN_Pin, GPIO_PIN_SET);

	}

	ptt_read = HAL_GPIO_ReadPin(PTT_GPIO_Port, PTT_Pin);

	if((ptt_status == 1) && (ptt_read == 0)){ //Switch back to RX
		HAL_GPIO_WritePin(BIAS_EN_GPIO_Port, BIAS_EN_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(vdd48V_EN_GPIO_Port, vdd48V_EN_Pin, GPIO_PIN_RESET);
		osDelay(100);
		HAL_GPIO_WritePin(TX_REL2_GPIO_Port, TX_REL2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(MCU_LED_GPIO_Port, MCU_LED_Pin, GPIO_PIN_RESET);
		vTaskResume(LPF_TaskHandle);
		ptt_status = 0;
	}

    osDelay(1);
  }
  /* USER CODE END Start_Task_TXRX */
}

/* USER CODE BEGIN Header_Start_LPF_Task */
/**
* @brief Function implementing the LPF_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_LPF_Task */
void Start_LPF_Task(void *argument)
{
  /* USER CODE BEGIN Start_LPF_Task */
  /* Infinite loop */
	  uint8_t current_band = 0;
	  //Read last band from EEPROM?
	  ShiftRegister74HC595_setAll(LOW);
	  ShiftRegister74HC595_update();

  for(;;)
  {

	    switch(band_mode) {
	        case 1:
	            //Analog voltage band selection
	        	//detect if band has changed
	        	selected_band = Map_voltage_band(band_analogU_mV);
	        	if(selected_band != current_band){
	        		Switch_band(selected_band);
	        		current_band = selected_band;
	        	}

	            break;
	        case 2:
	            //3-bit band selection
	        	band_binary_sel[0] = HAL_GPIO_ReadPin(BIN_BAND1_GPIO_Port, BIN_BAND1_Pin);
	        	band_binary_sel[1] = HAL_GPIO_ReadPin(BIN_BAND2_GPIO_Port, BIN_BAND2_Pin);
	        	band_binary_sel[2] = HAL_GPIO_ReadPin(BIN_BAND3_GPIO_Port, BIN_BAND3_Pin);
	        	selected_band = decode_binary_band(band_binary_sel);

	        	if(current_band != selected_band){ 		//Switch band only if settings has changed
	        		Switch_band(selected_band);
	        		current_band = selected_band;
	        	}

	            break;
	        case 3:
	            //Manual band selection, binary from front panel takes the same format so just multiplex between the two as inputs to case 2?

	            break;
	        //case 4:
	            //Automatic band selection - option for later with frequency counter
	            break;
	        default:
	            //Undefined behavior
	        	//fault_handler();  //TODO write fault handler and pass error code
	        	break;
	    }


	  //Test all the bands
	  /*
		ShiftRegister74HC595_setAll(LOW);
		ShiftRegister74HC595_update();

	  ShiftRegister74HC595_setPin(0, HIGH);
	  ShiftRegister74HC595_update();
	  ShiftRegister74HC595_setPin(0, LOW);
	  ShiftRegister74HC595_update();

	  ShiftRegister74HC595_setPin(5, HIGH); //6 m
	  ShiftRegister74HC595_update();
	  ShiftRegister74HC595_setPin(5, LOW);
	  ShiftRegister74HC595_update();

	  ShiftRegister74HC595_setPin(2, HIGH); //15-10 m
	  ShiftRegister74HC595_update();
	  ShiftRegister74HC595_setPin(2, LOW);
	  ShiftRegister74HC595_update();

	  ShiftRegister74HC595_setPin(3, HIGH); //20-17 m
	  ShiftRegister74HC595_update();
	  ShiftRegister74HC595_setPin(3, LOW);
	  ShiftRegister74HC595_update();

	  ShiftRegister74HC595_setPin(4, HIGH); //40 m
	  ShiftRegister74HC595_update();
	  ShiftRegister74HC595_setPin(4, LOW); //Stops working here?
	  ShiftRegister74HC595_update();

	  ShiftRegister74HC595_setPin(5, HIGH);
	  ShiftRegister74HC595_update();
	  ShiftRegister74HC595_setPin(5, LOW); //80 m
	  ShiftRegister74HC595_update();

	  ShiftRegister74HC595_setPin(6, HIGH); //160 m
	  ShiftRegister74HC595_update();
	  ShiftRegister74HC595_setPin(6, LOW);
	  ShiftRegister74HC595_update();
		ShiftRegister74HC595_setAll(LOW);
		ShiftRegister74HC595_update();
		*/


    osDelay(10);
  }
  /* USER CODE END Start_LPF_Task */
}

/* USER CODE BEGIN Header_Start_Display_Task */
/**
* @brief Function implementing the Display_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Display_Task */
void Start_Display_Task(void *argument)
{
  /* USER CODE BEGIN Start_Display_Task */

	uint8_t last_band, last_mode;
	uint16_t tx_disp_counter = 0; //TODO use later to detect whether to update the display

	lcdInit(&hi2c2, (uint8_t)0x27, (uint8_t)2, (uint8_t)20);
	lcdBacklightOn();
	lcdDisplayOn();

	lcdPrintStr((uint8_t*)"   FORTE 600W", 13);
	lcdSetCursorPosition(0, 2);
	lcdPrintStr((uint8_t*)"    LDMOS HF", 12);
	osDelay(2000);
	lcdDisplayClear();
	lcdPrintStr((uint8_t*)"   FORTE 600W", 13);
	lcdSetCursorPosition(0, 2);
	lcdPrintStr((uint8_t*)" AMPLIFIER v1.0", 15);
	osDelay(2000);
	lcdDisplayClear();
	lcdPrintStr((uint8_t*)"   FORTE 600W", 13);
	lcdSetCursorPosition(0, 2);
	lcdPrintStr((uint8_t*)"     SP6GK", 10);
	osDelay(2000);

	max_init();

	uint8_t bar_pwr = 0;
	uint8_t bar_swr = 0;
	uint8_t bar_amp = 0;


  /* Infinite loop */
  for(;;)
  {

	//split_bar_graph(pwr, swr, amp)
	split_bar_graph(0, 0, 0); //Clear bargraph display

	lcdCursorHome();

	lcdDisplayClear();

	if((ptt_status == 0) && (stdby_flag == 0)){
		split_bar_graph(0, 0, 0); //Clear bargraph display
		lcdPrintStr((uint8_t*)" BAND  M", 8);
		//print selected band
	    switch (selected_band) {
	        case 6:
	        	lcdSetCursorPosition(0, 2);
	        	lcdPrintStr((uint8_t*)"  6m", 4);

	            break;
	        case 5:
	        	lcdSetCursorPosition(0, 2);
	        	lcdPrintStr((uint8_t*)"15-10m", 6);

	            break;
	        case 4:
	        	lcdSetCursorPosition(0, 2);
	        	lcdPrintStr((uint8_t*)"20-17m", 6);

	            break;
	        case 3:
	        	lcdSetCursorPosition(0, 2);
	        	lcdPrintStr((uint8_t*)"40-30m", 6);

	            break;
	        case 2:
	        	lcdSetCursorPosition(0, 2);
	        	lcdPrintStr((uint8_t*)" 80m", 4);
	        	break;

	        case 1:
	        	lcdSetCursorPosition(0, 2);
	        	lcdPrintStr((uint8_t*)"160m", 4);
	        	break;

	        default:
	            //Fault_handler()
	        	break;
	    }

		//print mode
	    switch(band_mode){
	    	case 1:
	    		lcdSetCursorPosition(7, 2);
	    		lcdPrintStr((uint8_t*)"V", 1);	//Analog Voltage
	    		break;
	    	case 2:
	    		lcdSetCursorPosition(7, 2);
	    		lcdPrintStr((uint8_t*)"M", 1);	//Manual
	    		break;
	    	case 3:
	    		lcdSetCursorPosition(7, 2);
	    		lcdPrintStr((uint8_t*)"B", 1); //Binary
	    		break;
	    	case 4:
	    		lcdSetCursorPosition(7, 2);
	    		lcdPrintStr((uint8_t*)"A", 1); //Automatic
	    		break;

	    	default:
	    		//
	    		break;
	    }

	    //print temperature
		lcdSetCursorPosition(10, 0);
		lcdPrintStr((uint8_t*)"T1", 2);
		Display_digits(13, 0, (int) temperature_arr[0]);


		lcdSetCursorPosition(10, 1);
		lcdPrintStr((uint8_t*)"T2", 2);
		Display_digits(13, 1, (int) temperature_arr[1]);

		lcdSetCursorPosition(15, 0);
		lcdPrintStr((uint8_t*)"C", 1);
		lcdSetCursorPosition(15, 1);
		lcdPrintStr((uint8_t*)"C", 1);

	}



	if(stdby_flag == 1){
		lcdDisplayClear();
		//Print standby mode (or maybe beside analog / binary/ manual add standby?)
		lcdSetCursorPosition(0, 0);
		lcdPrintStr((uint8_t*)"   FORTE 600", 12);
		lcdSetCursorPosition(0, 2);
		lcdPrintStr((uint8_t*)"Stand by", 8);
	}
	if(ptt_status == 1){
		//TX display
		lcdDisplayClear();
		//Display_float_digit(0, 0, temperature);

		  bar_pwr = mapRange(lpf_fwd_pwr, 0, 600, 0, 30);
		  bar_swr = Map_float(lpf_swr, 0, 3, 0, 10);
		  if(bar_swr > 10){
			  bar_swr = 10;
		  }
		  bar_amp = mapRange((int) i_48v, 0, 30, 0, 10);
		  split_bar_graph(bar_pwr, bar_swr, bar_amp);

		tx_disp_counter ++; // change display on left from drain monitoring to temperature
		if(tx_disp_counter < 10){
			lcdSetCursorPosition(0, 0);
			lcdPrintStr((uint8_t*)"U", 1);
			Display_float_digit(2, 0, u_48v);


			lcdSetCursorPosition(0, 1);
			lcdPrintStr((uint8_t*)"I", 1);
			Display_float_digit(2, 1, i_48v);

			lcdSetCursorPosition(6, 0);
			lcdPrintStr((uint8_t*)"V", 1);
			lcdSetCursorPosition(6, 1);
			lcdPrintStr((uint8_t*)"A", 1);
		}
		if(tx_disp_counter >= 10){
			lcdSetCursorPosition(0, 0);
			lcdPrintStr((uint8_t*)"T1", 2);
			Display_digits(3, 0, (int) temperature_arr[0]);


			lcdSetCursorPosition(0, 1);
			lcdPrintStr((uint8_t*)"T2", 2);
			Display_digits(3, 1, (int) temperature_arr[1]);

			lcdSetCursorPosition(6, 0);
			lcdPrintStr((uint8_t*)"C", 1);
			lcdSetCursorPosition(6, 1);
			lcdPrintStr((uint8_t*)"C", 1);
		}
		if(tx_disp_counter == 20){
			tx_disp_counter = 0;
		}

		lcdSetCursorPosition(8, 0);
		lcdPrintStr((uint8_t*)"PWR", 3);
		lcdSetCursorPosition(8, 1);
		lcdPrintStr((uint8_t*)"SWR", 3);

		Display_digits(12, 0, lpf_fwd_pwr);
		Display_float_digit(12, 1, lpf_swr);

		lcdSetCursorPosition(15, 0);
		lcdPrintStr((uint8_t*)"W", 1);




	}

	//voltageArray[test_counter] = voltage_test;
	//temperaturearray[test_counter] = temperature_arr[0];
	//test_counter ++;




	osDelay(100);
  }

  /* USER CODE END Start_Display_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

