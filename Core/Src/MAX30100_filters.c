#include <MAX30100_filters.h>
float v[2];   
   
float dc_removal(float x,float alpha,float *dcw)
{   
    float olddcw = *dcw;   
    *dcw = x + (alpha) * (*dcw);
   
    return *dcw - olddcw;   
}   
   
float butterworth_filter(float x)
{   
    v[0] = v[1];   
    v[1] = (2.452372752527856026e-1 * x)   
         + (0.50952544949442879485 * v[0]);   
    return   
         (v[0] + v[1]);   
}   
