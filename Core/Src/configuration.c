#include "configuration.h"
#include "eeprom_i2c.h"
#include <stdint.h>
#include <string.h>
#include "i2c.h"
#include "stdlib.h"
#include <stdio.h>

extern configuration conf;
RingBuffer buff;

configuration configuration_init() {
	//default configuration init

	conf.hr_low_thresh = HR_LOW_THRESH;
	conf.hr_high_thresh = HR_HIGH_THRESH;
	conf.ox_low_thresh = OX_LOW_THRESH;
	conf.ox_high_thresh = OX_HIGH_THRESH;
	conf.mqtt_log_period = MQTT_LOG_PERIOD;
	conf.mqtt_message_period = MQTT_MSG_PERIOD;
	sprintf(conf.mqtt_server, "%s", MQTT_SERVER);
	return conf;
}
int load_conf() {
//loading the configuration stored in the EEPROM
	uint8_t conf_array[CONFIGURATION_ATTR];
	uint8_t totalSize = 0;
	RingBuffer_Init(&buff);
	uint8_t allData[MAX_SIZE_RING_BUFFER];
	int res;
	//reading (first byte) for the configuration size
	res = read_bytes(&hi2c1, DEV_ADDR, MEM_ADDR, &totalSize, 1);
	//reading for the configuration and saving in configuration struct
	char mqtt_server[MAX_SIZE_RING_BUFFER];
	res = read_bytes(&hi2c1, DEV_ADDR, MEM_ADDR + 1, allData, totalSize);
	RingBuffer_Write(&buff, allData, totalSize);
	RingBuffer_Read(&buff, conf_array, CONFIGURATION_ATTR);
	RingBuffer_Read(&buff, (uint8_t*) mqtt_server,
			totalSize - CONFIGURATION_ATTR);
	mqtt_server[totalSize - CONFIGURATION_ATTR]='\0';
	if (res) {
		conf.hr_low_thresh = conf_array[0];
		conf.hr_high_thresh = conf_array[1];
		conf.ox_low_thresh = conf_array[2];
		conf.ox_high_thresh = conf_array[3];
		conf.mqtt_log_period = conf_array[4];
		conf.mqtt_message_period = conf_array[5];
	}
	sprintf(conf.mqtt_server, "%s", mqtt_server);
	return 1;

}

int save_conf() {
	//storing the configuration in the EEPROM
	RingBuffer_Init(&buff);
	uint8_t conf_array[CONFIGURATION_ATTR] = { conf.hr_low_thresh,
			conf.hr_high_thresh, conf.ox_low_thresh, conf.ox_high_thresh,
			conf.mqtt_log_period, conf.mqtt_message_period };
	uint8_t size = strlen(conf.mqtt_server);
	uint8_t totalSize = size + CONFIGURATION_ATTR;
	//storing (1 byte) for the configuration size
	RingBuffer_Write(&buff, &totalSize, 1);
	//storing the configuration in the EEPROM
	RingBuffer_Write(&buff, conf_array, CONFIGURATION_ATTR);
	RingBuffer_Write(&buff, (uint8_t*) conf.mqtt_server, size);
	uint8_t buff_size = RingBuffer_GetDataLength(&buff);
	uint8_t buffData[MAX_SIZE_RING_BUFFER];
	RingBuffer_Read(&buff, buffData, buff_size);
	write_bytes(&hi2c1, DEV_ADDR, MEM_ADDR, buffData, buff_size);
	return 1;
}
uint8_t read_conf_from_buffer(RingBuffer *ring, char *conf_array) {
	//reading 1 attribute of the configuration from ring buffer
	RingBuffer_Read(ring, (uint8_t*) conf_array, CONFIGURATION_ATTR_LENGTH);
	return (uint8_t) (atoi(conf_array));
}

void conf_buffer_transfer(char *conf_buffer, char *conf_array, char *server_array){
	RingBuffer ring;
	RingBuffer_Init(&ring);
	//Store conf_buffer in the ring buffer
	RingBuffer_Write(&ring, (uint8_t*)conf_buffer, CONFIGURATION_LENGTH);
	//read from ring buffer the values in conf structure
	conf.hr_low_thresh=read_conf_from_buffer(&ring,conf_array);
	conf.hr_high_thresh=read_conf_from_buffer(&ring,conf_array);
	conf.ox_low_thresh=read_conf_from_buffer(&ring,conf_array);
	conf.ox_high_thresh=read_conf_from_buffer(&ring,conf_array);
	conf.mqtt_log_period=read_conf_from_buffer(&ring,conf_array);
	conf.mqtt_message_period=read_conf_from_buffer(&ring,conf_array);
	RingBuffer_Read(&ring, (uint8_t*)server_array,CONFIGURATION_SEVER_LENGTH);
	server_array[CONFIGURATION_SEVER_LENGTH]='\n';
	server_array[CONFIGURATION_SEVER_LENGTH+1]='\0';
	sprintf(conf.mqtt_server,"%s",(char*)server_array);
	conf.mqtt_server[CONFIGURATION_SEVER_LENGTH+1]='\0';
}

