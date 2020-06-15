#include "lcd.h"
#include "i2c.h"
#include <stdint.h>
#include <stdio.h>

void lcd_send_cmd(char cmd) {
	// send command to the lcd
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (cmd & 0xf0);
	data_l = ((cmd << 4) & 0xf0);
	data_t[0] = data_u | 0x0C;  //en=1, rs=0
	data_t[1] = data_u | 0x08;  //en=0, rs=0
	data_t[2] = data_l | 0x0C;  //en=1, rs=0
	data_t[3] = data_l | 0x08;  //en=0, rs=0
	HAL_I2C_Master_Transmit(&hi2c2, SLAVE_ADDRESS_LCD, (uint8_t*) data_t, 4,
			100);
}

void lcd_send_data(char data) {
	// send data to the lcd
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (data & 0xf0);
	data_l = ((data << 4) & 0xf0);
	data_t[0] = data_u | 0x0D;  //en=1, rs=0
	data_t[1] = data_u | 0x09;  //en=0, rs=0
	data_t[2] = data_l | 0x0D;  //en=1, rs=0
	data_t[3] = data_l | 0x09;  //en=0, rs=0
	HAL_I2C_Master_Transmit(&hi2c2, SLAVE_ADDRESS_LCD, (uint8_t*) data_t, 4,
			100);
}

void lcd_clear(void) {
	//clear lcd
	lcd_send_cmd(0x80);
	for (int i = 0; i < 70; i++) {
		lcd_send_data(' ');
	}
}

void lcd_put_cur(int row, int col) {
	// put cursor at the entered position row (0 or 1), col (0-15);
	switch (row) {
	case 0:
		col |= 0x80;
		break;
	case 1:
		col |= 0xC0;
		break;
	}

	lcd_send_cmd(col);
}

void lcd_init(void) {
	// initialize lcd

	// 4 bit initialization
	HAL_Delay(50);  // wait for >40ms
	lcd_send_cmd(0x30);  // display on
	HAL_Delay(5);  // wait for >4.1ms
	lcd_send_cmd(0x30);
	HAL_Delay(1);  // wait for >100us
	lcd_send_cmd(0x30);
	HAL_Delay(10);
	lcd_send_cmd(0x20);  // 4bit mode
	HAL_Delay(10);

	// dislay initialization
	lcd_send_cmd(0x28); // Function set --> DL=0 (4 bit mode), N = 1 (2 line display) F = 0 (5x8 characters) 00100100
	HAL_Delay(1);
	lcd_send_cmd(0x08); //Display on/off control --> D=0,C=0, B=0  ---> display off
	HAL_Delay(1);
	lcd_send_cmd(0x01);  // clear display
	HAL_Delay(1);
	HAL_Delay(1);
	lcd_send_cmd(0x06); //Entry mode set --> I/D = 1 (increment cursor) & S = 0 (no shift)
	HAL_Delay(1);
	lcd_send_cmd(0x0C); //Display on/off control --> D = 1, C and B = 0. (Cursor and blink, last two bits)
}

void lcd_send_string(char *str) {
	// send string to the lcd
	while (*str)
		lcd_send_data(*str++);
}

void lcd_send_status(uint8_t bpm, uint8_t ox) {

	//send hr and ox status on lcd
	lcd_clear();
	lcd_put_cur(0, 0);
	char c[HR_STATUS_SIZE];
	char c2[OX_STATUS_SIZE];
	sprintf(c, "HR:%02u bpm", bpm);
	lcd_send_string(c);

	lcd_put_cur(1, 0);
	sprintf(c2, "OX:%02u%%", ox);
	lcd_send_string(c2);

}

