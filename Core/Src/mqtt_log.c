/*
 * mqtt_log.c
 *
 *  Created on: Jun 5, 2020
 *      Author: mario
 */
#include "mqtt_log.h"
#include "usart.h"
#include "main.h"
#include "datastructure.h"
#include <string.h>
#define LOG_SIZE (40)
mqtt_log mqtt;
extern date_time_t date;

char sep[] = ", ";
char end[] = "\n";
char dest[20];


void mqtt_init_communication(char *server){
	//mqtt.server = server;
	mqtt_set_server(server);
	//NON COPIARE: strcpy(mqtt.server, server);
	//mqtt.started=0;
	//HAL_UART_Transmit_DMA(&huart1,(uint8_t*) mqtt.server, strlen(mqtt.server));
	int size = strlen(server);
	HAL_UART_Transmit_DMA(&huart1,(uint8_t*) server, size );
	mqtt.state=HANDSHAKE;
}

int send_mqtt(){
	dest[0]='\0';
	char timestamp_str[100];
	uint64_t timestamp = (uint64_t)  date.seconds+date.minutes*3600+date.year*86400+(date.year-70)*31536000+((date.year-69)/4)*86400 - ((date.year-1)/100)*86400+((date.year+299)/400)*86400;
	if(mqtt.state==STARTED || mqtt.state==CONN_ERR){
		//char log_buffer[LOG_SIZE+1];
		//sprintf(log_buffer,"%02u-%02u-%04u %02u-%02u-%02u",date.day,date.month,date.year,date.hours,date.minutes,date.seconds);
		sprintf(timestamp_str,"%u",timestamp);
		strcat(dest,timestamp_str);
		strcat(dest, sep);
		strcat(dest,mqtt.hearth_rate);
		strcat(dest, sep);
		strcat(dest,mqtt.oximeter);
		strcat(dest,end);
		int size = strlen(timestamp_str)+strlen(sep)+strlen(mqtt.hearth_rate)+strlen(sep)+strlen(mqtt.oximeter)+strlen(end);
		HAL_UART_Transmit_DMA(&huart1,(uint8_t*) dest, size);
		return 0;
	}
	return -1;
}

void mqtt_set_hearth_rate(uint8_t hearth_rate){
	itoa(hearth_rate, mqtt.hearth_rate, 10);
}
void mqtt_set_oximeter(uint8_t oximeter){
	itoa(oximeter, mqtt.oximeter , 10);
}
void mqtt_set_server(char *server){
	sprintf(mqtt.server,"%s",(char *)server);
}

int mqtt_is_started(){
	return mqtt.state==STARTED;
}
void mqtt_get_ack(){
	if(mqtt.state==HANDSHAKE)
		HAL_UART_Receive_DMA(&huart1,(uint8_t*) mqtt.ack, ACK_SIZE);
}

void mqtt_check_state(){
	if(mqtt.state==HANDSHAKE){
		if(strcmp(mqtt.ack,"ACK")==0){
			mqtt.state=STARTED;
			HAL_UART_Receive_DMA(&huart1,(uint8_t*) mqtt.ack, ACK_SIZE);
		}
		else if(strcmp(mqtt.ack,"MQE")==0)
			mqtt.state=MQTT_ERROR;
		else if(strcmp(mqtt.ack,"ERR")==0){
			mqtt.state=CONN_ERR;
			HAL_UART_Receive_DMA(&huart1,(uint8_t*) mqtt.ack, ACK_SIZE);
		}
	}
	else if(mqtt.state==STARTED){
		if(strcmp(mqtt.ack,"ERR")==0){
			mqtt.state=CONN_ERR;
			HAL_UART_Receive_DMA(&huart1,(uint8_t*) mqtt.ack, ACK_SIZE);
		}

		else if(strcmp(mqtt.ack,"SNT")==0)
			HAL_UART_Receive_DMA(&huart1,(uint8_t*) mqtt.ack, ACK_SIZE);
	}
	else if(mqtt.state==CONN_ERR){
		if(strcmp(mqtt.ack,"SNT")==0){
			HAL_UART_Receive_DMA(&huart1,(uint8_t*) mqtt.ack, ACK_SIZE);
			mqtt.state=STARTED;
		}
	}


}

