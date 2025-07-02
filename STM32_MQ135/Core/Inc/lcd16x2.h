#ifndef __LCD16X2_H__
#define __LCD16X2_H__

#include "stm32f1xx_hal.h"

// dinh nghia cac chan
#define LCD_PORT    GPIOB

#define LCD_RS      GPIO_PIN_8
#define LCD_RW      0
#define LCD_E       GPIO_PIN_9

#define LCD_D4      GPIO_PIN_12
#define LCD_D5      GPIO_PIN_13
#define LCD_D6      GPIO_PIN_14
#define LCD_D7      GPIO_PIN_15

#define cmd_reg     0
#define data_reg    1

// cac ham
void LCD_Write(uint8_t data);
void LCD_Send(uint8_t Reg, uint8_t data);
void LCD_Init();
void LCD_Clear();
void LCD_Location(uint8_t x, uint8_t y);
void LCD_Write_String(char* string);
void LCD_Write_Number(int number);


#endif
