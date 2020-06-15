/*
 * eeprom_i2c.c
 *
 *  Created on: 29 May 2020
 *      Author: mario
 */

#include "eeprom_i2c.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"
#include <string.h>
#include <stdio.h>




int write_bytes(I2C_HandleTypeDef *hi2c, uint16_t DevAddress,uint16_t MemAddress, uint8_t *pData,uint8_t TxBufferSize){
	while(HAL_I2C_IsDeviceReady(hi2c, DevAddress, 1, HAL_MAX_DELAY) != HAL_OK);
	HAL_StatusTypeDef returnValue;
	returnValue = HAL_I2C_Mem_Write(hi2c, DevAddress, MemAddress, I2C_MEMADD_SIZE_16BIT, pData, TxBufferSize, HAL_MAX_DELAY);
	if(returnValue!=HAL_OK)
		return EEPROM_ERR;
	return EEPROM_OK;
}
int read_bytes(I2C_HandleTypeDef *hi2c,uint16_t DevAddress,uint16_t MemAddress, uint8_t *pData,uint8_t RxBufferSize){
	while(HAL_I2C_IsDeviceReady(hi2c, DevAddress, 1, HAL_MAX_DELAY) != HAL_OK);
	HAL_StatusTypeDef returnValue;
	returnValue = HAL_I2C_Mem_Read(hi2c, DevAddress, MemAddress, I2C_MEMADD_SIZE_16BIT, pData, RxBufferSize, HAL_MAX_DELAY);
	if(returnValue!=HAL_OK)
		return EEPROM_ERR;
	return EEPROM_OK;
}


/**
  * @brief               : This function handles Reading String on the specific Address .
  * 					   This program have this feature that don't force you to use absolute 16 bytes
  * 					   you can use more than 16 bytes buffer.
  * @param  hi2c         : Pointer to a I2C_HandleTypeDef structure that contains
  *                        the configuration information for the specified I2C.
  * @param	MemAddress   : Internal memory address (WHERE YOU WANNA READ)
  * @param	pString	     : Pointer to data buffer(CHAR DATA)
  * @param  length       : Amount of buffer you wanna Read from
  * @retval
  */


int write_string(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, char *pString ,uint16_t MemAddress ,uint8_t length){
	uint8_t pData[length];
	int i =0;
	while(*pString)
		(pData[i++])=(uint8_t)(*pString++);
	write_bytes(hi2c,DevAddress,MemAddress,pData,length);
	return 1;
}
int read_string(I2C_HandleTypeDef *hi2c, uint16_t DevAddress,char *pString,uint16_t MemAddress,uint8_t length){
	uint8_t pData[length];
	int i=0;
	read_bytes(hi2c,DevAddress,MemAddress,pData,length);
	while(pData[i])
		(*pString++)=(char)pData[i++];
	return 1;
}
