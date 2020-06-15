/*
 * buzzer.c
 *
 *  Created on: Jun 1, 2020
 *      Author: luigi
 */
#include "main.h"
#include "tim.h"
#include "gpio.h"
#include "buzzer.h"
#include "configuration.h"

extern TIM_HandleTypeDef htim9;
extern buzzer_status buzzer;

HAL_StatusTypeDef buzzer_low_hr (uint8_t hr)
{
	if (hr<=conf.hr_low_thresh)
		return HAL_ERROR;
	return HAL_OK;
}

HAL_StatusTypeDef buzzer_high_hr (uint8_t hr)
{
	if (hr>=conf.hr_high_thresh)
		return HAL_ERROR;
	return HAL_OK;
}

HAL_StatusTypeDef buzzer_low_ox (uint8_t ox)
{
	if (ox<=conf.ox_low_thresh)
		return HAL_ERROR;
	return HAL_OK;
}

HAL_StatusTypeDef buzzer_high_ox (uint8_t ox)
{
	if (ox>=conf.ox_high_thresh)
		return HAL_ERROR;
	return HAL_OK;
}

void buzzer_beep()
{
	htim3.Init.Prescaler=320;
	htim2.Instance->CCR1 = OUTPUT_FREQUENCY;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  __HAL_TIM_CLEAR_FLAG(&htim3, TIM_SR_UIF);
	HAL_TIM_Base_Start_IT(&htim3);
}

void buzzer_check(uint8_t hr, uint8_t ox) {

	if (buzzer_low_hr(hr)) {
		buzzer.sound_number_hr = 1;
	} else if (buzzer_high_hr(hr)) {
		buzzer.sound_number_hr = 2;
	}
	if (buzzer_low_ox(ox)) {
		buzzer.sound_number_ox = 3;
	} else if (buzzer_high_ox(ox)) {
		buzzer.sound_number_ox = 4;
	}

	if (buzzer.sound_number_hr > 0 || buzzer.sound_number_ox > 0) {
		buzzer_beep();
	}
}


