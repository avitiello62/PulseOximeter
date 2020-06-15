#include "eeprom_i2c.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"
#include <string.h>
#include <stdio.h>

int write_bytes(I2C_HandleTypeDef *hi2c, uint16_t DevAddress,
		uint16_t MemAddress, uint8_t *pData, uint8_t TxBufferSize) {
	//storing data on EEPROM with I2C
	while (HAL_I2C_IsDeviceReady(hi2c, DevAddress, 1, HAL_MAX_DELAY) != HAL_OK)
		;
	HAL_StatusTypeDef returnValue;
	returnValue = HAL_I2C_Mem_Write(hi2c, DevAddress, MemAddress,
			I2C_MEMADD_SIZE_16BIT, pData, TxBufferSize, HAL_MAX_DELAY);
	if (returnValue != HAL_OK)
		return EEPROM_ERR;
	return EEPROM_OK;
}
int read_bytes(I2C_HandleTypeDef *hi2c, uint16_t DevAddress,
		uint16_t MemAddress, uint8_t *pData, uint8_t RxBufferSize) {
	//reading data from EEPROM with I2C
	while (HAL_I2C_IsDeviceReady(hi2c, DevAddress, 1, HAL_MAX_DELAY) != HAL_OK)
		;
	HAL_StatusTypeDef returnValue;
	returnValue = HAL_I2C_Mem_Read(hi2c, DevAddress, MemAddress,
			I2C_MEMADD_SIZE_16BIT, pData, RxBufferSize, HAL_MAX_DELAY);
	if (returnValue != HAL_OK)
		return EEPROM_ERR;
	return EEPROM_OK;
}

int write_string(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, char *pString,
		uint16_t MemAddress, uint8_t length) {
	//storing string on EEPROM with I2C
	uint8_t pData[length];
	int i = 0;
	while (*pString)
		(pData[i++]) = (uint8_t) (*pString++);
	write_bytes(hi2c, DevAddress, MemAddress, pData, length);
	return 1;
}
int read_string(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, char *pString,
		uint16_t MemAddress, uint8_t length) {
	//reading string from EEPROM with I2C
	uint8_t pData[length];
	int i = 0;
	read_bytes(hi2c, DevAddress, MemAddress, pData, length);
	while (pData[i])
		(*pString++) = (char) pData[i++];
	return 1;
}
