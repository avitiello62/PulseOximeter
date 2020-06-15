/*
 * lcd.h
 *
 *  Created on: Jun 3, 2020
 *      Author: albc
 */

#ifndef INC_LCD_H_
#define INC_LCD_H_

#include "stdint.h"

#define LCD_ADDRESS 				(0x4E)
#define MAX_RETRY					(3)
#define SLAVE_ADDRESS_LCD 			(0x4E)

#define HR_STATUS_SIZE (15)

#define OX_STATUS_SIZE (10)

void lcd_init(void);

void lcd_send_cmd(char cmd);

void lcd_send_data(char data);

void lcd_send_string(char *str);

void lcd_put_cur(int row, int col);

void lcd_clear(void);

void lcd_send_status(uint8_t bpm, uint8_t ox);

#endif /* INC_LCD_H_ */
