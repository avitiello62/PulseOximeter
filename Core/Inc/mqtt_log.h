/*
 * mqtt_log.h
 *
 *  Created on: Jun 5, 2020
 *      Author: mario
 */

#ifndef INC_MQTT_LOG_H_
#define INC_MQTT_LOG_H_
#define MAX_SIZE (50)
#define ACK_SIZE (3)
#include "stm32f4xx_hal.h"
#include <stdio.h>

enum _States{INACTIVE, HANDSHAKE, STARTED, MQTT_ERROR, CONN_ERR};
typedef enum _States State;

typedef struct mqtt_communication{
	char hearth_rate[MAX_SIZE];
	char oximeter[MAX_SIZE];
	char *server;
	char ack[ACK_SIZE+1];
	State state;

}mqtt_log;

extern mqtt_log mqtt;

void mqtt_init_communication(char *server);
int send_mqtt();
void mqtt_set_hearth_rate(uint8_t hearth_rate);
void mqtt_set_oximeter(uint8_t oximeter);
void mqtt_set_server(char *server);
void mqtt_get_ack();
void mqtt_check_state(void);








#endif /* INC_MQTT_LOG_H_ */
