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



int8_t lcd2_is_ready();

void lcd_init (void);   // initialize lcd

void lcd_send_cmd (char cmd);  // send command to the lcd

void lcd_send_data (char data);  // send data to the lcd

void lcd_send_string (char *str);  // send string to the lcd

void lcd_put_cur(int row, int col);  // put cursor at the entered position row (0 or 1), col (0-15);

void lcd_clear (void);
void lcd_send_status(uint8_t bpm, uint8_t ox);


#endif /* INC_LCD_H_ */
