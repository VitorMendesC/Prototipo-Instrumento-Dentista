/*
 * LCD.h
 *
 *  Created on: Jun 30, 2023
 *      Author: Arlindo Neto Montagnoli
 */

#ifndef LCD_H_
#define LCD_H_

#include "main.h"

void lcd_Init(GPIO_TypeDef* port_ctr, uint16_t rs_pin, uint16_t rw_pin, uint16_t e_pin,
              GPIO_TypeDef* port_data, uint16_t d4_pin, uint16_t d5_pin, uint16_t d6_pin, uint16_t d7_pin);
void lcd_clear();
void lcd_gotoxy(uint8_t, uint8_t);
void lcd_writeCommand(uint8_t);
void lcd_printf(const char* str, ...);
void lcd_create(uint8_t, uint8_t *);
void lcd_putc(uint8_t data);
#endif /* LCD_H_ */
