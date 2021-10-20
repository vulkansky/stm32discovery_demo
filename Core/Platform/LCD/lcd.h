/*
 * lcd.h
 *
 *  Created on: Oct 16, 2021
 *      Author: Kirill
 */

#ifndef PLATFORM_LCD_LCD_H_
#define PLATFORM_LCD_LCD_H_

#include "stdint.h"

void LCD_init();
void LCD_Clear(void);
void LCD_SendChar(char ch);
void LCD_String(char* st);
void LCD_SetPos(uint8_t x, uint8_t y);


#endif /* PLATFORM_LCD_LCD_H_ */
