/*
 * lcd.c
 *
 *  Created on: Jun 30, 2023
 *      Author: Arlindo Neto Montagnoli
 */

#include "lcd.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

//-------------------------------------------------------------------------------
// static variables
//-------------------------------------------------------------------------------
static GPIO_TypeDef* PORT_CTR;
static uint16_t PIN_RS, PIN_E;
static GPIO_TypeDef* PORT_DATA;
static uint16_t D4_PIN, D5_PIN, D6_PIN, D7_PIN;

//-------------------------------------------------------------------------------
// delay_us
//-------------------------------------------------------------------------------
void delay_us(uint16_t t) {
	for(int i=0;i<t;i++)
		for(int j=0;j<10;j++)
		__NOP();

}
//-------------------------------------------------------------------------------
// lcd_enablePulse
//-------------------------------------------------------------------------------
static void lcd_enablePulse(void)
{
  HAL_GPIO_WritePin(PORT_CTR, PIN_E, 1);
  delay_us(20);
  HAL_GPIO_WritePin(PORT_CTR, PIN_E, 0);
  delay_us(60);
}
//-------------------------------------------------------------------------------
// lcd_write
//-------------------------------------------------------------------------------
static void lcd_write(uint8_t data)
{
	  //send MSB data
    uint8_t nibble = (data >> 4) & 0xF;
    HAL_GPIO_WritePin(PORT_DATA, D4_PIN, nibble & 0x1);
    HAL_GPIO_WritePin(PORT_DATA, D5_PIN, nibble & 0x2);
    HAL_GPIO_WritePin(PORT_DATA, D6_PIN, nibble & 0x4);
    HAL_GPIO_WritePin(PORT_DATA, D7_PIN, nibble & 0x8);
    lcd_enablePulse();
    //send LSB data
    nibble = data & 0xF;
    HAL_GPIO_WritePin(PORT_DATA, D4_PIN, nibble & 0x1);
    HAL_GPIO_WritePin(PORT_DATA, D5_PIN, nibble & 0x2);
    HAL_GPIO_WritePin(PORT_DATA, D6_PIN, nibble & 0x4);
    HAL_GPIO_WritePin(PORT_DATA, D7_PIN, nibble & 0x8);
    lcd_enablePulse();
}
//-------------------------------------------------------------------------------
// lcd_writeCmd
//-------------------------------------------------------------------------------
static void lcd_writeCmd(uint8_t cmd)
{
  HAL_GPIO_WritePin(PORT_CTR, PIN_RS, 0);
  lcd_write(cmd);
}
//-------------------------------------------------------------------------------
// lcd_writeData
//-------------------------------------------------------------------------------
static void lcd_writeData(uint8_t data)
{
  HAL_GPIO_WritePin(PORT_CTR, PIN_RS, 1);
  lcd_write(data);
}
//-------------------------------------------------------------------------------
// lcd_write4
//-------------------------------------------------------------------------------
static void lcd_write4(uint8_t cmd)
{
  cmd &= 0x0F;
  HAL_GPIO_WritePin(PORT_CTR, PIN_RS, 0);
  HAL_GPIO_WritePin(PORT_DATA, D4_PIN, cmd & 0x1);
  HAL_GPIO_WritePin(PORT_DATA, D5_PIN, cmd & 0x2);
  HAL_GPIO_WritePin(PORT_DATA, D6_PIN, cmd & 0x4);
  HAL_GPIO_WritePin(PORT_DATA, D7_PIN, cmd & 0x8);
  lcd_enablePulse();
}
//-------------------------------------------------------------------------------
// lcd_Init
//-------------------------------------------------------------------------------
void lcd_Init(GPIO_TypeDef* port_ctr, uint16_t rs_pin, uint16_t rw_pin, uint16_t e_pin,
              GPIO_TypeDef* port_data, uint16_t d4_pin, uint16_t d5_pin, uint16_t d6_pin, uint16_t d7_pin)
{
  PORT_CTR = port_ctr;
  PIN_RS = rs_pin;
  PIN_E = e_pin;
  PORT_DATA = port_data;
  D4_PIN = d4_pin;
  D5_PIN = d5_pin;
  D6_PIN = d6_pin;
  D7_PIN = d7_pin;

  HAL_Delay(15);
  lcd_write4(0x3);
  HAL_Delay(5);
  lcd_write4(0x3);
  HAL_Delay(1);
  lcd_write4(0x3);
  HAL_Delay(1);
  lcd_write4(0x2);
  HAL_Delay(1);
  lcd_writeCmd(0x28);
  lcd_writeCmd(0x0C);
  lcd_writeCmd(0x01);
  HAL_Delay(30);
}
//-------------------------------------------------------------------------------
// lcd_writeCommand
//-------------------------------------------------------------------------------
void lcd_writeCommand(uint8_t cmd)
{
	lcd_writeCmd(cmd);
	HAL_Delay(3);
}
//-------------------------------------------------------------------------------
// lcd_gotoxy
//-------------------------------------------------------------------------------
void lcd_gotoxy(uint8_t col, uint8_t row)
{
  if(row==0)
    lcd_writeCmd(0x80+col);
  else
    lcd_writeCmd(0xC0+col);
}
//-------------------------------------------------------------------------------
// lcd_clear
// avoid: this routine makes the lcd flicker!
//-------------------------------------------------------------------------------
void lcd_clear()
{
	lcd_writeCommand(0x01);
}
//-------------------------------------------------------------------------------
// lcd_printf
//-------------------------------------------------------------------------------
void lcd_printf(const char* str, ...)
{
  char stringArray[20];
  va_list args;
  va_start(args, str);
  vsprintf(stringArray, str, args);
  va_end(args);
  for(uint8_t i = 0; i < strlen(stringArray); i++)
    lcd_writeData((uint8_t)stringArray[i]);
}
//-------------------------------------------------------------------------------
// lcd_create: Create Specials Characters
//Location 0x01 to 0x08
// obs.: location 0 was avoided because the string is a null-terminated character
/* charmap example: ã
uint8_t a_til[8] = {
					0b01101,
					0b10010,
					0b00000,
					0b01110,
					0b00001,
					0b01111,
					0b10001,
					0b01111
};
*/
//-------------------------------------------------------------------------------
void lcd_create(uint8_t location, uint8_t *charmap)
{
	if (location > 8) return;
	location &= 0x7;
	lcd_writeCmd(0x40 | (location << 3)); // 0x40 CGRAM address
	for (int i = 0; i < 8; i++)
		lcd_writeData(charmap[i]);

}
//-------------------------------------------------------------------------------
// lcd_putc (put char)
//-------------------------------------------------------------------------------
void lcd_putc(uint8_t data)
{
  HAL_GPIO_WritePin(PORT_CTR, PIN_RS, 1);
  lcd_write(data);
}
