#ifndef MAX30100_FILTERS_H
#define MAX30100_FILTERS_H
#include <stdint.h>

#define ALPHA 0.95

#define MEAN_FILTER_SIZE 15

extern float v[2];
struct meanDiffFilter_s {

	uint8_t index;
	float sum;
	uint8_t count;
	float values[MEAN_FILTER_SIZE];
};
typedef struct meanDiffFilter_s meanDiffFilter_t;
float dc_removal(float x, float alpha, float *dcw);
float butterworth_filter(float x);
float mean_diff_filter(float M, meanDiffFilter_t *filterValues);
#endif
