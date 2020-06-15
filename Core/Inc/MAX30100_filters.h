#ifndef MAX30100_FILTERS_H
#define MAX30100_FILTERS_H
extern float v[2];
#define ALPHA 0.95

float dc_removal(float x,float alpha,float *dcw);
float butterworth_filter(float x);

#endif
