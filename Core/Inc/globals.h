/*
 * globals.h
 *
 *  Created on: Aug 18, 2023
 *      Author: sp6gk
 */

#ifndef GLOBALS_H
#define GLOBALS_H

#include "main.h"
#include "cmsis_os.h"
#include "stdbool.h"
#include "ds18b20.h"

// ADC DMA Buffer
extern uint16_t ADC_val[ADC_BUFFER_SIZE];
extern uint16_t ADC_avg_val[ADC_BUFFER_SIZE];

// Input coupler
extern float in_fwd_u;
extern float in_ref_u;
extern float in_fwd_pwr;
extern float in_ref_pwr;

// Deck coupler
extern uint16_t deck_fwd_pwr;
extern uint16_t deck_ref_pwr;
extern float deck_swr;

// LPF coupler
extern uint16_t lpf_fwd_pwr;
extern uint16_t lpf_ref_pwr;
extern float lpf_swr;

//Coupler voltages
extern float deck_fwd_u;
extern float deck_ref_u;
extern float lpf_fwd_u;
extern float lpf_ref_u;

// Power bus
extern float u_48v;
extern float i_48v;
extern float i_48v_sensor_u;
extern float sensor_vcc;

// Sensors

extern float temperature_arr[_DS18B20_MAX_SENSORS];
//extern float voltage_test;
//extern float voltageArray[200];
//extern float temperaturearray[200];
//extern uint16_t test_counter;

extern uint16_t fan_tacho1;
extern uint16_t fan_tacho2;

// Band selection
extern uint8_t selected_band;
extern uint8_t band_mode;
extern uint8_t	 mode_read;
extern float band_analogU;
extern uint16_t band_analogU_mV;
extern uint16_t band_man_sel;
extern bool band_binary_sel[3];

// Flags
extern uint16_t fault;
extern uint8_t bias_flag;
extern uint8_t ptt_status;
extern uint8_t stdby_flag;

extern uint16_t gu16_TIM2_OVC;

extern osThreadId_t TXRX_TaskHandle;

#endif // GLOBALS_H
