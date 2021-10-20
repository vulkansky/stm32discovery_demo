/*
 * lcd.c
 *
 *  Created on: Oct 16, 2021
 *      Author: Kirill
 */

#include "main.h"
#include "lcd.h"

#define d4_set()    LL_GPIO_SetOutputPin(GPIOE, LCD_DB4_Pin)
#define d5_set()    LL_GPIO_SetOutputPin(GPIOE, LCD_DB5_Pin)
#define d6_set()    LL_GPIO_SetOutputPin(GPIOE, LCD_DB6_Pin)
#define d7_set()    LL_GPIO_SetOutputPin(GPIOE, LCD_DB7_Pin)

#define d4_reset()  LL_GPIO_ResetOutputPin(GPIOE, LCD_DB4_Pin)
#define d5_reset()  LL_GPIO_ResetOutputPin(GPIOE, LCD_DB5_Pin)
#define d6_reset()  LL_GPIO_ResetOutputPin(GPIOE, LCD_DB6_Pin)
#define d7_reset()  LL_GPIO_ResetOutputPin(GPIOE, LCD_DB7_Pin)

#define LCD_ENABLE  LL_GPIO_SetOutputPin(GPIOE, LCD_Enable_Pin)
#define LCD_DISABLE LL_GPIO_ResetOutputPin(GPIOE, LCD_Enable_Pin)

#define RS_ENABLE   LL_GPIO_SetOutputPin(GPIOE, LCD_RS_Pin)
#define RS_DISABLE  LL_GPIO_ResetOutputPin(GPIOE, LCD_RS_Pin)

static void LCD_Data(uint8_t dt);
static void LCD_WriteData(uint8_t dt);
static void LCD_Command(uint8_t dt);

static void LCD_Data(uint8_t dt)
{
	RS_ENABLE;
    LCD_WriteData(dt>>4);

    LCD_ENABLE;
	LL_mDelay(5);
    LCD_DISABLE;

    LCD_WriteData(dt);

    LCD_ENABLE;
	LL_mDelay(5);
    LCD_DISABLE;
}

static void LCD_WriteData(uint8_t dt)
{
    if(((dt >> 3)&0x01)==1) {d7_set();} else {d7_reset();}
    if(((dt >> 2)&0x01)==1) {d6_set();} else {d6_reset();}
    if(((dt >> 1)&0x01)==1) {d5_set();} else {d5_reset();}
    if((dt&0x01)==1) {d4_set();} else {d4_reset();}
}

static void LCD_Command(uint8_t dt)
{
	RS_DISABLE;
    LCD_WriteData(dt>>4);
    LCD_ENABLE;
	LL_mDelay(5);
    LCD_DISABLE;
    LCD_WriteData(dt);
    LCD_ENABLE;
	LL_mDelay(5);
    LCD_DISABLE;
}

void LCD_String(char* st)
{
	uint8_t i=0;
    while(st[i]!=0)
    {
    	LCD_Data(st[i]);
        LL_mDelay(5);
        i++;
    }
}

void LCD_Clear(void)
{
    LCD_Command(0x01);
    LL_mDelay(2);
}

void LCD_SendChar(char ch)
{
	LCD_Data((uint8_t) ch);
    LL_mDelay(5);
}

void LCD_init(void)
{
	LL_mDelay(50);
    RS_DISABLE;
    LCD_WriteData(0x30);
    LCD_ENABLE;
    LL_mDelay(5);
    LCD_DISABLE;
    LL_mDelay(5);
    LCD_WriteData(0x30);
    LCD_ENABLE;
    LL_mDelay(5);
    LCD_DISABLE;
    LL_mDelay(1);
    LCD_WriteData(0x30);
    LCD_ENABLE;
    LL_mDelay(5);
    LCD_DISABLE;
    LL_mDelay(10);
    LCD_WriteData(0x20);
    LCD_ENABLE;
    LL_mDelay(5);
    LCD_DISABLE;
    LL_mDelay(10);
    LCD_Command(0x28);//режим 4 бит, 2 линии (для нашего большого дисплея это 4 линии), шрифт 5х8
    LL_mDelay(1);
    LCD_Command(0x28);//еще раз для верности
    LL_mDelay(1);
    LCD_Command(0x0C);//дисплей включаем (D=1), также включаем пока все курсоры
    LL_mDelay(1);
    LCD_Command(0x01);//уберем мусор
    LL_mDelay(2);
    LCD_Command(0x06);//пишем влево.
    LL_mDelay(1);
    LCD_Command(0x02);//возврат курсора в нулевое положение
    LL_mDelay(2);
}

void LCD_SetPos(uint8_t x, uint8_t y)
{
	switch(y)
	{
		case 0:
			LCD_Command(x|0x80);
            LL_mDelay(1);
            break;
		case 1:
			LCD_Command((0x40+x)|0x80);
            LL_mDelay(1);
            break;
		case 2:
			LCD_Command((0x14+x)|0x80);
            LL_mDelay(1);
            break;
		case 3:
			LCD_Command((0x54+x)|0x80);
            LL_mDelay(1);
            break;
	}
}
