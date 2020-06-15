#ifndef INC_MAX30100_H_
#define INC_MAX30100_H_

#include <stdint.h>

int8_t MAX30100_init(void);
void MAX30100_shutdown(void);
int MAX30100_getID(void);
void MAX30100_setup(void);
void MAX30100_reset(void);
void MAX30100_clearFIFO(void);
void MAX30100_interrupt_enable();

void MAX30100_read_sensor(void);

#endif /* INC_MAX30100_H_ */
