/*
 * log.h
 *
 *  Created on: May 29, 2020
 *      Author: albc
 */

#ifndef INC_LOG_H_
#define INC_LOG_H_

#include "datastructure.h"
#include "ring_buffer.h"

#define SYSTEM_BOOT ((uint8_t*)"SYSTEM BOOT\t")
#define SYSTEM_CONFIGURATION_LOADED ((uint8_t*)"SYSTEM CONFIGURATION LOADED\t")
#define SYSTEM_ACTIVATED ((uint8_t*)"SYSTEM ACTIVATED\t")
#define SYSTEM_DEACTIVATED ((uint8_t*)"SYSTEM DEACTIVATED\t")
#define LOW_HEART_RATE_ALARM ((uint8_t*)"LOW HEART RATE ALARM ")
#define HIGH_HEART_RATE_ALARM ((uint8_t*)"HIGH HEART RATE ALARM\t")
#define MQTT_ERRORS ((uint8_t*)"MQTT ERRORS\t")
#define MQTT_CONNECTION_ESTABLISHED ((uint8_t*)"MQTT CONNECTION ESTABLISHED\t")
#define MQTT_CONNECTION_CLOSED ((uint8_t*)"MQTT CONNECTION CLOSED\t")
#define MQTT_MESSAGE_SENT ((uint8_t*)"MQTT MESSAGE SENT\t")

#define SYSTEM_LOG_SIZE (40)
#define SYSTEM_BOOT_SIZE (12)
#define SYSTEM_CONFIGURATION_LOADED_SIZE (28)
#define SYSTEM_ACTIVATED_SIZE (17)
#define SYSTEM_DEACTIVATED_SIZE (19)
#define LOW_HEART_RATE_ALARM_SIZE (21)
#define HIGH_HEART_RATE_ALARM_SIZE (22)
#define MQTT_ERRORS_SIZE (12)
#define MQTT_CONNECTION_ESTABLISHED_SIZE (28)
#define MQTT_CONNECTION_CLOSED_SIZE (23)
#define MQTT_MESSAGE_SENT_SIZE (18)

#define TIME_INCREMENT (10)

void log_status(uint8_t hr, uint8_t ox);
void log_system_boot(void);
void log_system_configuration_loaded(void);
void log_system_activated(void);
void log_system_deactivated(void);
void log_low_heart_rate_alarm(void);
void log_high_heart_rate_alarm(void);
void log_mqtt_errors(void);
void log_mqtt_connection_established(void);
void log_mqtt_connection_closed(void);
void log_mqtt_message_sent(void);
void log_transmit(void);
void configuration_check_tr(hr_ox_t hr_ox);
#endif /* INC_LOG_H_ */
