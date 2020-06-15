/*
 * configuration.h
 *
 *  Created on: 29 May 2020
 *      Author: mario
 */

#ifndef INC_CONFIGURATION_H_
#define INC_CONFIGURATION_H_
#include <stdint.h>
#include "ring_buffer.h"
#include "log.h"

#define HR_LOW_THRESH (170)
#define HR_HIGH_THRESH (40)
#define OX_LOW_THRESH (60)
#define OX_HIGH_THRESH (96)
#define MQTT_LOG_PERIOD (60)
#define MQTT_MSG_PERIOD (120)
#define MQTT_SERVER ("test.mosquitto.org:60\n\0")

#define CONFIGURATION_LENGTH (39)
#define CONFIGURATION_SEVER_LENGTH (21)
#define CONFIGURATION_ATTR_LENGHT (3)

#define DEV_ADDR (0xA0)
#define MEM_ADDR (0x0)

typedef struct config
{
	uint8_t hr_low_thresh;
	uint8_t hr_high_thresh;
	uint8_t ox_low_thresh;
	uint8_t ox_high_thresh;
	uint8_t mqtt_log_period;
	uint8_t mqtt_message_period;
	char mqtt_server[CONFIGURATION_SEVER_LENGTH+2];
}configuration;

typedef enum boot_status{
	SYSTEM_BOOTED,CONFIGURATION_LOADED_FROM_USER,CONFIGURATION_NOT_LOADED
}boot_status;

extern configuration conf;

configuration configuration_init();

int save_conf(void);
int load_conf(void);
uint8_t read_conf_from_buffer(RingBuffer * ring,char* conf_array);
void configuration_check_tr(hr_ox_t hr_ox);

#endif /* INC_CONFIGURATION_H_ */
