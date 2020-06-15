#ifndef INC_EEPROM_I2C_H_
#define INC_EEPROM_I2C_H_

#define EEPROM_ERR (-1)
#define EEPROM_OK (1)
#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"

int write_bytes(I2C_HandleTypeDef *hi2c, uint16_t DevAddress,
		uint16_t MemAddress, uint8_t *pData, uint8_t TxBufferSize);
int read_bytes(I2C_HandleTypeDef *hi2c, uint16_t DevAddress,
		uint16_t MemAddress, uint8_t *pData, uint8_t RxBufferSize);
int write_string(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, char *pString,
		uint16_t MemAddress, uint8_t length);
int read_string(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, char *pString,
		uint16_t MemAddress, uint8_t length);

#endif /* INC_EEPROM_I2C_H_ */
