#include "main.h"
#include "tim.h"
#include "gpio.h"
#include "buzzer.h"
#include "configuration.h"
#include "stm32f4xx_it.h"

extern pwm_status pwm;
extern TIM_HandleTypeDef htim3;
extern buzzer_status buzzer;
extern TIM_HandleTypeDef htim2;

HAL_StatusTypeDef buzzer_low_hr(uint8_t hr) {
	if (hr <= conf.hr_low_thresh)
		return HAL_ERROR;
	return HAL_OK;
}

HAL_StatusTypeDef buzzer_high_hr(uint8_t hr) {
	if (hr >= conf.hr_high_thresh)
		return HAL_ERROR;
	return HAL_OK;
}

HAL_StatusTypeDef buzzer_low_ox(uint8_t ox) {
	if (ox <= conf.ox_low_thresh)
		return HAL_ERROR;
	return HAL_OK;
}

HAL_StatusTypeDef buzzer_high_ox(uint8_t ox) {
	if (ox >= conf.ox_high_thresh)
		return HAL_ERROR;
	return HAL_OK;
}

void buzzer_beep() {
	//htim3 gets updated to get 0.2 s period
	htim3.Init.Prescaler = 320;
	htim2.Instance->CCR1 = OUTPUT_FREQUENCY;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	__HAL_TIM_CLEAR_FLAG(&htim3, TIM_SR_UIF);
	HAL_TIM_Base_Start_IT(&htim3);
}

void buzzer_check(uint8_t hr, uint8_t ox) {
	//checks for the thresholds and sets the right sound numbers
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
void buzzer_init() {
	buzzer.sound_number_hr = 0;
	buzzer.sound_number_ox = 0;
}
void buzzer_timer_procedure() {
	if (htim3.Init.Prescaler == 4799 && buzzer.sound_number_ox > 0) {
		HAL_TIM_Base_Stop_IT(&htim3);
		htim3.Init.Prescaler = 320;
		if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
			Error_Handler();
		}
		__HAL_TIM_CLEAR_FLAG(&htim3, TIM_SR_UIF);
		HAL_TIM_Base_Start_IT(&htim3);
	} else if (htim3.Init.Prescaler == 4799 && buzzer.sound_number_ox == 0) {
		HAL_TIM_Base_Stop_IT(&htim3);
	}

	if (htim3.Init.Prescaler == 320) {
		if (buzzer.sound_number_hr > 0) {
			if (pwm == PWM_STOPPED) {
				HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
				pwm = PWM_RUNNING;
			} else if (pwm == PWM_RUNNING) {
				HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
				pwm = PWM_STOPPED;
				buzzer.sound_number_hr--;
				if (buzzer.sound_number_hr == 0) {
					HAL_TIM_Base_Stop_IT(&htim3);
					htim3.Init.Prescaler = 4799;
					if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
						Error_Handler();
					}
					__HAL_TIM_CLEAR_FLAG(&htim3, TIM_SR_UIF);
					HAL_TIM_Base_Start_IT(&htim3);
				}
			}
		} else if (buzzer.sound_number_ox > 0) {
			if (pwm == PWM_STOPPED) {
				htim2.Instance->CCR1 = 200;
				HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
				pwm = PWM_RUNNING;
			} else if (pwm == PWM_RUNNING) {
				HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
				pwm = PWM_STOPPED;
				buzzer.sound_number_ox--;
				if (buzzer.sound_number_ox == 0) {
					HAL_TIM_Base_Stop_IT(&htim3);
				}
			}
		}
	}
}
