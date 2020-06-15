#ifndef INC_BUZZER_H_
#define INC_BUZZER_H_

#define OUTPUT_FREQUENCY (200)

typedef enum pwm_status {
	PWM_RUNNING, PWM_STOPPED
} pwm_status;

typedef struct buzzer_status {
	uint8_t sound_number_hr;
	uint8_t sound_number_ox;
} buzzer_status;

void buzzer_init();

HAL_StatusTypeDef buzzer_low_hr(uint8_t hr);

HAL_StatusTypeDef buzzer_high_hr(uint8_t hr);

HAL_StatusTypeDef buzzer_low_ox(uint8_t ox);

HAL_StatusTypeDef buzzer_high_ox(uint8_t ox);

void buzzer_check(uint8_t hr, uint8_t ox);

void buzzer_beep();
void buzzer_timer_procedure();

#endif /* INC_BUZZER_H_ */
