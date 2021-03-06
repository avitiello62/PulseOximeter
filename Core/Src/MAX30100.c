#include "MAX30100.h"
#include "MAX30100_filters.h"
#include "MAX30100_beat_detector.h"
#include "MAX30100_spo2_calculator.h"
#include "stdint.h"
#include "i2c.h"
#include "stdio.h"
#include "ring_buffer.h"
#include "datastructure.h"

#define ADDRESS_SIZE				(1)
#define DATA_TRANSFER_SIZE			(1)
#define MAX_RETRY					(3)

#define MAX30100_OK 		(0)
#define MAX30100_ERR		(-1)
#define MAX30100_IC2_ERR	(-2)

/*Register status*/
#define MAX30100_INTERRUPT_STATUS			(0x00)
#define MAX30100_INTERRUPT_ENABLE			(0x01)
/* FIFO register*/
#define MAX30100_FIFO_WR_PTR				(0x02)
#define MAX30100_OVF_COUNTER				(0x03)
#define MAX30100_FIFO_RD_PTR				(0x04)
#define MAX30100_FIFO_DATA					(0x05)
/* CONFIGURATION register*/
#define MAX30100_MODE_CONFIG				(0x06)
#define MAX30100_SPO2_CONFIG				(0x07)
#define MAX30100_LED_CONFIG					(0x09)
// MAX30100 I2C addresses
#define MAX30100_WADDRESS        			(0xAE) // 8bit address converted to 7bit + W
#define MAX30100_RADDRESS       			(0xAF)  // 8bit address converted to 7bit + R
// PART ID registers
#define MAX30100_PART_ID            		 (0xFF)
#define EXPECTED_PART_ID            		 (0x11)

extern UART_HandleTypeDef huart2;
extern RingBuffer dma_buffer;
extern hr_ox_t hr_ox;

uint16_t IR = 0;  // Last IR reflectance datapoint
uint16_t RED = 0;     // Last Red reflectance datapoint
SpO2Calculator_t spo2_calculator;
uint8_t temp[4] = { 0 };  // Temporary buffer for read values
float w = 0.0;
float h = 0.0;
float ir_ACvalue, red_ACvalue, filtered_pulse_value;
int beat_detected;

meanDiffFilter_t meanDiffIR;

int8_t MAX30100_init() {
	HAL_StatusTypeDef return_value;
	return_value = HAL_I2C_IsDeviceReady(&hi2c3, MAX30100_WADDRESS, MAX_RETRY,
	HAL_MAX_DELAY);
	if (return_value == HAL_OK) {
		//initialization of  the mean diff filter
		meanDiffIR.index = 0;
		meanDiffIR.sum = 0;
		meanDiffIR.count = 0;
		//setting up the sensor
		MAX30100_reset();
		MAX30100_shutdown();
		MAX30100_getID();
		MAX30100_setup();
		MAX30100_clearFIFO();
		MAX30100_interrupt_enable();
		spo2_reset(&spo2_calculator);

		return MAX30100_OK;
	}

	return MAX30100_ERR;
}

int MAX30100_getID() {
	uint8_t buffer[1];
	HAL_I2C_Mem_Read(&hi2c3, MAX30100_RADDRESS, 0xff, ADDRESS_SIZE, &buffer[1],
			1, HAL_MAX_DELAY);
	if (buffer[1] == EXPECTED_PART_ID)
		return MAX30100_OK;

	return MAX30100_ERR;
}
void MAX30100_setup() {

	uint8_t buffer[1];
	buffer[0] = 0x07; // Sp02[2:0] 100 sample and LED_PW[1:0] 1600μs
	HAL_I2C_Mem_Write(&hi2c3, MAX30100_WADDRESS,
	MAX30100_SPO2_CONFIG, ADDRESS_SIZE, &buffer[0], 1, HAL_MAX_DELAY);
	buffer[0] = 0xFF; //Led current for Red and IR 50mA
	HAL_I2C_Mem_Write(&hi2c3, MAX30100_WADDRESS,
	MAX30100_LED_CONFIG, ADDRESS_SIZE, &buffer[0], 1, HAL_MAX_DELAY);
	buffer[0] = 0x03; // conf Sp02
	HAL_I2C_Mem_Write(&hi2c3, MAX30100_WADDRESS,
	MAX30100_MODE_CONFIG, ADDRESS_SIZE, &buffer[0], 1, HAL_MAX_DELAY);
}

void MAX30100_clearFIFO() {

	uint8_t buffer[] = { 0x00, 0x00, 0x00 };

	//update 3 FIFO_WR_PTR, FIFO_WR_PTR and OVF_COUNTER registers

	HAL_I2C_Mem_Write(&hi2c3, MAX30100_WADDRESS,
	MAX30100_FIFO_WR_PTR, ADDRESS_SIZE, &buffer[1], 1, HAL_MAX_DELAY);
	HAL_I2C_Mem_Write(&hi2c3, MAX30100_WADDRESS,
	MAX30100_FIFO_RD_PTR, ADDRESS_SIZE, &buffer[2], 1, HAL_MAX_DELAY);
	HAL_I2C_Mem_Write(&hi2c3, MAX30100_WADDRESS,
	MAX30100_OVF_COUNTER, ADDRESS_SIZE, &buffer[3], 1, HAL_MAX_DELAY);
}
//remove all setup of max30100
void MAX30100_reset() {

	uint8_t buffer[1];
	buffer[0] = 0x40; //set bit RESET=1
	HAL_I2C_Mem_Write(&hi2c3, MAX30100_WADDRESS,
	MAX30100_MODE_CONFIG, 1, &buffer[0], 1, HAL_MAX_DELAY);
	HAL_I2C_Mem_Read(&hi2c3, MAX30100_RADDRESS,
	MAX30100_MODE_CONFIG, 1, &buffer[1], 1, HAL_MAX_DELAY);

}
//set up the high res
void MAX30100_shutdown() {

	uint8_t buffer[1];
	HAL_I2C_Mem_Read(&hi2c3, MAX30100_RADDRESS,
	MAX30100_MODE_CONFIG, 1, &buffer[1], 1, HAL_MAX_DELAY);
	buffer[0] = (1 << 7);
	HAL_I2C_Mem_Write(&hi2c3, MAX30100_WADDRESS,
	MAX30100_MODE_CONFIG, 1, &buffer[0], 1, HAL_MAX_DELAY);

}

void MAX30100_interrupt_enable() {

	uint8_t buffer[1];
	HAL_I2C_Mem_Read(&hi2c3, MAX30100_RADDRESS,
	MAX30100_INTERRUPT_ENABLE, 1, &buffer[1], 1, HAL_MAX_DELAY);
	buffer[0] = 0x80;
	HAL_I2C_Mem_Write(&hi2c3, MAX30100_WADDRESS,
	MAX30100_INTERRUPT_ENABLE, 1, &buffer[0], 1, HAL_MAX_DELAY);
	HAL_Delay(100);
	//disable SHDN
	buffer[0] = 0x03;
	HAL_I2C_Mem_Write(&hi2c3, MAX30100_WADDRESS,
	MAX30100_MODE_CONFIG, I2C_MEMADD_SIZE_8BIT, &buffer[0], 1,
	HAL_MAX_DELAY);
	HAL_I2C_Mem_Read(&hi2c3, MAX30100_RADDRESS,
	MAX30100_MODE_CONFIG, 1, &buffer[0], 1, HAL_MAX_DELAY);
}

void MAX30100_read_sensor() {

	// Read four bytes from the FIFO for take 1 sample
	HAL_I2C_Mem_Read(&hi2c3, MAX30100_RADDRESS, MAX30100_FIFO_DATA, 1, &temp[0],
			4, HAL_MAX_DELAY);
	IR = (temp[0] << 8) | temp[1];    // Combine values to get the actual number
	RED = (temp[2] << 8) | temp[3];   // Combine values to get the actual number
	//applying the filters
	ir_ACvalue = dc_removal(IR, ALPHA, &w);
	filtered_pulse_value = butterworth_filter(
			mean_diff_filter(ir_ACvalue, &meanDiffIR));
	beat_detected = add_sample(filtered_pulse_value);
	red_ACvalue = dc_removal(RED, ALPHA, &h);
	spo2_update(ir_ACvalue, red_ACvalue, beat_detected, &spo2_calculator);
	//storing the actual values of hr and ox in the struct
	hr_ox.hr = (uint8_t) get_rate();
	hr_ox.ox = spo2_calculator.spo2;

}

