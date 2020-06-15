#include <stdint.h>
#include "log.h"
#include "dma.h"
#include "usart.h"
#include <stdio.h>
#include "main.h"
#include "datastructure.h"
#include "configuration.h"
#include <string.h>

extern date_time_t date;
extern RingBuffer dma_buffer;

void log_status(uint8_t hr, uint8_t ox) {
	//storing log status string on dma ring buffer
	char log_buffer[SYSTEM_LOG_SIZE + 1];
	sprintf(log_buffer, "[%02u-%02u-%04u %02u-%02u-%02u] HR:%u bpm-OX:%u%%\t",
			date.day, date.month, date.year, date.hours, date.minutes,
			date.seconds, hr, ox);

	RingBuffer_Write(&dma_buffer, (uint8_t*) log_buffer, strlen(log_buffer));
	date_increment(&date, TIME_INCREMENT);
}

void log_system_boot(void) {
	//storing system boot  string on dma ring buffer
	RingBuffer_Write(&dma_buffer, SYSTEM_BOOT, SYSTEM_BOOT_SIZE);

}

void log_system_configuration_loaded(void) {
	//storing system configuration loaded string on dma ring buffer
	RingBuffer_Write(&dma_buffer, SYSTEM_CONFIGURATION_LOADED,
	SYSTEM_CONFIGURATION_LOADED_SIZE);

}
void log_system_activated(void) {
	//storing system activated string on dma ring buffer
	RingBuffer_Write(&dma_buffer, SYSTEM_ACTIVATED, SYSTEM_ACTIVATED_SIZE);
}
void log_system_deactivated(void) {
	//storing system deactivated string on dma ring buffer
	RingBuffer_Write(&dma_buffer, SYSTEM_DEACTIVATED, SYSTEM_DEACTIVATED_SIZE);
}
void log_low_heart_rate_alarm(void) {
	//storing low heart rate alarm string on dma ring buffer
	RingBuffer_Write(&dma_buffer, LOW_HEART_RATE_ALARM,
	LOW_HEART_RATE_ALARM_SIZE);
}
void log_high_heart_rate_alarm(void) {
	//storing high heart rate alarm string on dma ring buffer
	RingBuffer_Write(&dma_buffer, HIGH_HEART_RATE_ALARM,
	HIGH_HEART_RATE_ALARM_SIZE);
}
void log_mqtt_errors(void) {
	//storing mqtt errors string on dma ring buffer
	RingBuffer_Write(&dma_buffer, MQTT_ERRORS, MQTT_ERRORS_SIZE);
}
void log_mqtt_connection_established(void) {
	//storing mqtt connection established string on dma ring buffer
	RingBuffer_Write(&dma_buffer, MQTT_CONNECTION_ESTABLISHED,
	MQTT_CONNECTION_ESTABLISHED_SIZE);
}
void log_mqtt_connection_closed(void) {
	//storing mqtt connection closed string on dma ring buffer
	RingBuffer_Write(&dma_buffer, MQTT_CONNECTION_CLOSED,
	MQTT_CONNECTION_CLOSED_SIZE);
}
void log_mqtt_message_sent(void) {
	//storing mqtt message sent string on dma ring buffer
	RingBuffer_Write(&dma_buffer, MQTT_MESSAGE_SENT, MQTT_MESSAGE_SENT_SIZE);
}
void log_transmit(void) {
	//trasmit the dma ring buffer over uart2
	HAL_UART_Transmit_DMA(&huart2, (uint8_t*) dma_buffer.buffer,
			dma_buffer.size);
	RingBuffer_Init(&dma_buffer);
}
void configuration_check_tr(hr_ox_t hr_ox) {
	//checks for heart rate alarm
	if (hr_ox.status == SENSOR_ACTIVE) {
		if (hr_ox.hr_aggregated <= conf.hr_low_thresh) {
			log_low_heart_rate_alarm();
		}
		if (hr_ox.hr_aggregated >= conf.hr_high_thresh) {
			log_high_heart_rate_alarm();
		}
	}

}
