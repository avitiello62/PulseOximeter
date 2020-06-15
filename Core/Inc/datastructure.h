#ifndef INC_DATASTRUCTURE_H_
#define INC_DATASTRUCTURE_H_

#include <stdint.h>

struct date_time_s {
	uint8_t seconds;
	uint8_t minutes;
	uint8_t hours;
	uint8_t day;
	uint8_t month;
	uint16_t year;
};
typedef struct date_time_s date_time_t;

typedef enum sensor_status {
	SENSOR_ACTIVE, SENSOR_INACTIVE
} sensor_status;
struct hr_ox_s {
	uint8_t hr;
	uint8_t ox;
	uint8_t hr_aggregated;
	uint8_t ox_aggregated;
	sensor_status status;

};
typedef struct hr_ox_s hr_ox_t;

hr_ox_t hr_ox_init();
date_time_t date_init();
void date_increment(date_time_t *date, uint8_t increment);

#endif /* INC_DATASTRUCTURE_H_ */
