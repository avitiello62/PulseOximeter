/*
 * configuration.c
 *
 *  Created on: 29 May 2020
 *      Author: mario
 */


#include "configuration.h"
#include "eeprom_i2c.h"
#include <stdint.h>
#include <string.h>
#include "i2c.h"
#include "stdlib.h"
#include <stdio.h>

extern configuration conf;
RingBuffer buff;


configuration configuration_init(){
	conf.hr_low_thresh=HR_LOW_THRESH;
	conf.hr_high_thresh=HR_HIGH_THRESH;
	conf.ox_low_thresh=OX_LOW_THRESH;
	conf.ox_high_thresh=OX_HIGH_THRESH;
	conf.mqtt_log_period=MQTT_LOG_PERIOD;
	conf.mqtt_message_period=MQTT_MSG_PERIOD;
	sprintf(conf.mqtt_server,"%s",MQTT_SERVER);
	return conf;
}
int load_conf(){

	uint8_t conf_array[6];
	uint8_t totalSize = 0;
	RingBuffer_Init(&buff);
	uint8_t allData[MAX_SIZE];
	int res;
	res=read_bytes(&hi2c1, DEV_ADDR, MEM_ADDR, &totalSize, 1);

	char mqtt_server[totalSize-6];
	res=read_bytes(&hi2c1, DEV_ADDR, MEM_ADDR+1, allData, totalSize);
	RingBuffer_Write(&buff,allData,totalSize);
	RingBuffer_Read(&buff, conf_array,6);
	RingBuffer_Read(&buff, (uint8_t*) mqtt_server,totalSize-6);

	if(res){
		conf.hr_low_thresh=conf_array[0];
		conf.hr_high_thresh=conf_array[1];
		conf.ox_low_thresh=conf_array[2];
		conf.ox_high_thresh=conf_array[3];
		conf.mqtt_log_period=conf_array[4];
		conf.mqtt_message_period=conf_array[5];
	}
	sprintf(conf.mqtt_server,"%s",mqtt_server);
	//conf.mqtt_server=mqtt_server;
	return 1;

}

int save_conf(){
	//Create an array
	RingBuffer_Init(&buff);
	uint8_t conf_array[6] = {conf.hr_low_thresh, conf.hr_high_thresh, conf.ox_low_thresh, conf.ox_high_thresh, conf.mqtt_log_period, conf.mqtt_message_period};
	uint8_t size = strlen(conf.mqtt_server);
	uint8_t totalSize=size+6;
	RingBuffer_Write(&buff, &totalSize, 1);
	RingBuffer_Write(&buff, conf_array, 6);
	RingBuffer_Write(&buff,(uint8_t*) conf.mqtt_server,size);
	uint8_t buff_size = RingBuffer_GetDataLength(&buff);
	uint8_t buffData[totalSize];
	RingBuffer_Read(&buff, buffData, buff_size);
	write_bytes(&hi2c1, DEV_ADDR, MEM_ADDR, buffData, buff_size);

//write_string(&hi2c1, DEV_ADDR, MEM_ADDR+6,conf.mqtt_server, strlen(conf.mqtt_server)+1);
	return 1;
}
uint8_t read_conf_from_buffer(RingBuffer * ring,char* conf_array){
	RingBuffer_Read(ring,(uint8_t*) conf_array,CONFIGURATION_ATTR_LENGHT);
	 return (uint8_t)(atoi(conf_array));
}
void configuration_check_tr(hr_ox_t hr_ox){
	if(hr_ox.status==SENSOR_ACTIVE){
		if(hr_ox.hr_aggregated<=conf.hr_low_thresh){
			log_low_heart_rate_alarm();
		}
		if(hr_ox.hr_aggregated>=conf.hr_high_thresh){
			log_high_heart_rate_alarm();
		}
	}

}


