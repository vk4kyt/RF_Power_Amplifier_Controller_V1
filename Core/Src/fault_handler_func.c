/*
 * fault_handler_func.c
 *
 *  Created on: Sep 4, 2023
 *      Author: sp6gk
 */

#include "stm32f1xx_hal.h"
#include "main.h"
#include "gpio.h"
#include "globals.h"
#include "lcd_hd44780_i2c.h"

void fault_handler(uint8_t fault_nr){
	vTaskSuspend(TXRX_TaskHandle);
	ptt_status = 0;
	HAL_GPIO_WritePin(vdd48V_EN_GPIO_Port, vdd48V_EN_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(BIAS_EN_GPIO_Port, BIAS_EN_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(TX_REL2_GPIO_Port, TX_REL2_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(buzzer_GPIO_Port, buzzer_Pin, SET);
	//HAL_Delay(1000);
	//HAL_GPIO_WritePin(buzzer_GPIO_Port, buzzer_Pin, RESET);


	lcdDisplayClear();
	lcdSetCursorPosition(0, 0);
	lcdPrintStr((uint8_t*)"Fault: ", 7);
	Display_digits(8, 0, fault_nr);

	lcdSetCursorPosition(0, 1);

	switch(fault_nr){
		case 1:
			//Drain over current protection tripped
			lcdPrintStr((uint8_t*)"MAX Id CURRENT!", 15);
			HAL_Delay(10000);


			break;
		case 2:
			//Over voltage drain protection
			lcdPrintStr((uint8_t*)"MAX Ud VOLTAGE!", 15);
			HAL_Delay(10000);
			break;
		case 3:
			//Thermal protection

			lcdPrintStr((uint8_t*)"MAX TEMPERATURE!", 16);

			TIM1->CCR2 = 700;
			TIM1->CCR1 = 1000;
			HAL_Delay(5000);
			HAL_GPIO_WritePin(buzzer_GPIO_Port, buzzer_Pin, RESET);
			HAL_Delay(10000);

			break;
		case 4:
			//Too high output power
			lcdPrintStr((uint8_t*)"Out Power limit!", 16);
			HAL_Delay(10000);
			break;
		case 5:
			//Too high input power
			lcdPrintStr((uint8_t*)"In Power limit!", 15);
			HAL_Delay(10000);
			break;
		case 6:
			//High output SWR
			lcdPrintStr((uint8_t*)"High out SWR!", 12);
			HAL_Delay(10000);
			break;
		case 7:
			//High output SWR
			lcdPrintStr((uint8_t*)"High in SWR!", 11);
			HAL_Delay(10000);
			break;
		case 8:
			//High reflection from LPF
			lcdPrintStr((uint8_t*)"Mismatch LPF!", 12);
			HAL_Delay(10000);
			break;
		case 9:
			//High S21 loss in LPF
			lcdPrintStr((uint8_t*)"LOSS PWR in LPF!", 16);
			HAL_Delay(10000);
			break;
		case 10:
			//Temperature sensor not detected
			lcdPrintStr((uint8_t*)"TEMP SENS ERROR", 15);
			HAL_Delay(10000);
			break;
		default:
			lcdPrintStr((uint8_t*)"Undefined ERROR", 15);
			HAL_Delay(1000);
			break;



	}
	HAL_NVIC_SystemReset();

}
