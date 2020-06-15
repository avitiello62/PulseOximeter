/*
 * datastructure.c
 *
 *  Created on: 6 giu 2020
 *      Author: albc
 */
#include "datastructure.h"
#include <stdint.h>

date_time_t date_init(){
	date_time_t d;
	d.day=1;
	d.month=1;
	d.year=2000;
	d.hours=8;
	d.minutes=0;
	d.seconds=59;
	return d;
}
hr_ox_t hr_ox_init(){
	hr_ox_t hr_ox;
	hr_ox.hr=0;
	hr_ox.ox=0;
	hr_ox.hr_aggregated=0;
	hr_ox.ox_aggregated=0;
	hr_ox.status=SENSOR_INACTIVE;
	return hr_ox;
}

void hr_ox_increment(hr_ox_t * hr_ox_p,uint8_t increment){
	hr_ox_p->hr+=increment;
	hr_ox_p->ox+=increment;

}
void date_increment(date_time_t * date,uint8_t increment){
	date->seconds+=increment;
	if(date->seconds>=60){
		date->seconds-=60;
		date->minutes+=1;
		if(date->minutes>=60){
			date->minutes-=60;
			date->hours+=1;
			if(date->hours>=24){
				date->hours-=24;
				date->day+=1;
			}
		}
	}

}
