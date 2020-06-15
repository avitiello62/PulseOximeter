#include <MAX30100_filters.h>
float v[2];

float dc_removal(float x, float alpha, float *dcw) {
	//filtering the digital part of the signal
	float olddcw = *dcw;
	*dcw = x + (alpha) * (*dcw);

	return *dcw - olddcw;
}

float butterworth_filter(float x) {
	//applying the BW filter
	v[0] = v[1];
	v[1] = (2.452372752527856026e-1 * x) + (0.50952544949442879485 * v[0]);
	return (v[0] + v[1]);
}
float mean_diff_filter(float M, meanDiffFilter_t *filterValues) {
	//applying the mean diff filter on 45 value
	float avg = 0;

	filterValues->sum -= filterValues->values[filterValues->index];
	filterValues->values[filterValues->index] = M;
	filterValues->sum += filterValues->values[filterValues->index];

	filterValues->index++;
	filterValues->index = filterValues->index % MEAN_FILTER_SIZE;

	if (filterValues->count < MEAN_FILTER_SIZE)
		filterValues->count++;

	avg = filterValues->sum / filterValues->count;
	return avg - M;
}
