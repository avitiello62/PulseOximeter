/*
 * buzze.h
 *
 *  Created on: Jun 1, 2020
 *      Author: luigi
 */

#ifndef INC_BUZZER_H_
#define INC_BUZZER_H_

#define OUTPUT_FREQUENCY (200)



HAL_StatusTypeDef buzzer_low_hr (uint8_t hr);

HAL_StatusTypeDef buzzer_high_hr (uint8_t hr);

HAL_StatusTypeDef buzzer_low_ox (uint8_t ox);

HAL_StatusTypeDef buzzer_high_ox (uint8_t ox);

void buzzer_check (uint8_t hr, uint8_t ox);

void buzzer_beep();

#endif /* INC_BUZZER_H_ */
