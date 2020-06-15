/*
 * MAX30100.h
 *
 *  Created on: May 29, 2020
 *      Author: MANU
 */

#ifndef INC_MAX30100_H_
#define INC_MAX30100_H_

#include <stdint.h>

#define MEAN_FILTER_SIZE 45

struct meanDiffFilter_s
{

  uint8_t index;
  float sum;
  uint8_t count;
  float values[MEAN_FILTER_SIZE];
};
typedef struct meanDiffFilter_s meanDiffFilter_t;

int8_t MAX30100_init(void);
void MAX30100_shutdown(void);
int MAX30100_getID(void);
void MAX30100_setup(void);
void MAX30100_reset(void);
void MAX30100_clearFIFO(void);
void MAX30100_interrupt_enable();

void MAX30100_read_sensor(void);
float MAX30100_mean_diff(float M, meanDiffFilter_t* filterValues);


#endif /* INC_MAX30100_H_ */
