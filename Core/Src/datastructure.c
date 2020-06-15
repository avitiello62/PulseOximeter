#include "datastructure.h"
#include <stdint.h>

date_time_t date_init() {
	//date init (for convention 01/01/2020 - 08:00:00 AM)
	date_time_t d;
	d.day = 1;
	d.month = 1;
	d.year = 2020;
	d.hours = 8;
	d.minutes = 0;
	d.seconds = 00;
	return d;
}
hr_ox_t hr_ox_init() {
	//hr and ox struct init
	hr_ox_t hr_ox;
	hr_ox.hr = 0;
	hr_ox.ox = 0;
	hr_ox.hr_aggregated = 0;
	hr_ox.ox_aggregated = 0;
	hr_ox.status = SENSOR_INACTIVE;
	return hr_ox;
}

void date_increment(date_time_t *date, uint8_t i) {
	// updating the datetime global status of i seconds
	date->seconds += i;
	if (date->seconds >= 60) {
		date->seconds -= 60;
		date->minutes += 1;
		if (date->minutes >= 60) {
			date->minutes -= 60;
			date->hours += 1;
			if (date->hours >= 24) {
				date->hours -= 24;
				date->day += 1;
			}
		}
	}

}
