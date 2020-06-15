#ifndef MAX30100_SPO2CALCULATOR_H
#define MAX30100_SPO2CALCULATOR_H

#include "stdint.h"

#define CALCULATE_EVERY_N_BEATS (3)

struct SpO2Calculator_s
{
	float ir_ACvalue_sq_sum;
	float red_ACvalue_sq_sum;
	uint8_t beats_detected_num;
	uint32_t samples_recorded;
	uint8_t spo2;
};
typedef struct SpO2Calculator_s SpO2Calculator_t;

void spo2_update(float irACValue, float redACValue, int beatDetected, SpO2Calculator_t* SpO2Calculator);
void spo2_reset(SpO2Calculator_t* SpO2Calculator);
uint8_t get_spo2(SpO2Calculator_t SpO2Calculator);

#endif
