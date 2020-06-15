#include <MAX30100_spo2_calculator.h>
#include "math.h"
uint32_t spo2_LUT[43] = { 100, 100, 100, 100, 99, 99, 99, 99, 99, 99, 98, 98,
		98, 98, 98, 97, 97, 97, 97, 97, 97, 96, 96, 96, 96, 96, 96, 95, 95, 95,
		95, 95, 95, 94, 94, 94, 94, 94, 93, 93, 93, 93, 93 };

void spo2_update(float ir_ACvalue, float red_ACvalue, int beat_detected,
		SpO2Calculator_t *sp02_calculator) {
	sp02_calculator->ir_ACvalue_sq_sum += ir_ACvalue * ir_ACvalue;
	sp02_calculator->red_ACvalue_sq_sum += red_ACvalue * red_ACvalue;
	++sp02_calculator->samples_recorded;

	if (beat_detected) {
		++sp02_calculator->beats_detected_num;
		if (sp02_calculator->beats_detected_num == CALCULATE_EVERY_N_BEATS) {
			float AC_sq_ratio = 100.0
					* log(
							sp02_calculator->red_ACvalue_sq_sum
									/ sp02_calculator->samples_recorded)
					/ log(
							sp02_calculator->ir_ACvalue_sq_sum
									/ sp02_calculator->samples_recorded);
			uint8_t index = 0;

			if (AC_sq_ratio > 66) {
				index = (uint8_t) AC_sq_ratio - 66;
			} else if (AC_sq_ratio > 50) {
				index = (uint8_t) AC_sq_ratio - 50;
			}
			spo2_reset(sp02_calculator);

			sp02_calculator->spo2 = spo2_LUT[index];
		}
	}
}

void spo2_reset(SpO2Calculator_t *sp02_calculator) {
	sp02_calculator->samples_recorded = 0;
	sp02_calculator->red_ACvalue_sq_sum = 0;
	sp02_calculator->ir_ACvalue_sq_sum = 0;
	sp02_calculator->beats_detected_num = 0;
	sp02_calculator->spo2 = 0;
}

uint8_t get_spO2(SpO2Calculator_t sp02_calculator) {
	return sp02_calculator.spo2;
}
