/*
 * lcd_i2c.c
 *
 *  Created on: 28 sty 2026
 *      Author: jgzim
 */

#include "lcd_i2c.h"
#include "main.h" // Aby widzieć hi2c2

// Zmień na hi2c1 jeśli używasz I2C1
extern I2C_HandleTypeDef hi2c2;

// Adres I2C (domyślny dla PCF8574 to 0x27 << 1 = 0x4E)
#define SLAVE_ADDRESS_LCD 0x4E

void lcd_send_cmd(char cmd)
{
  char data_u, data_l;
  uint8_t data_t[4];
  data_u = (cmd & 0xf0);
  data_l = ((cmd << 4) & 0xf0);
  data_t[0] = data_u | 0x0C;  // en=1, rs=0
  data_t[1] = data_u | 0x08;  // en=0, rs=0
  data_t[2] = data_l | 0x0C;  // en=1, rs=0
  data_t[3] = data_l | 0x08;  // en=0, rs=0
  HAL_I2C_Master_Transmit(&hi2c2, SLAVE_ADDRESS_LCD, (uint8_t *) data_t, 4, 100);
}

void lcd_send_data(char data)
{
  char data_u, data_l;
  uint8_t data_t[4];
  data_u = (data & 0xf0);
  data_l = ((data << 4) & 0xf0);
  data_t[0] = data_u | 0x0D;  // en=1, rs=1
  data_t[1] = data_u | 0x09;  // en=0, rs=1
  data_t[2] = data_l | 0x0D;  // en=1, rs=1
  data_t[3] = data_l | 0x09;  // en=0, rs=1
  HAL_I2C_Master_Transmit(&hi2c2, SLAVE_ADDRESS_LCD, (uint8_t *) data_t, 4, 100);
}

void lcd_clear(void)
{
    lcd_send_cmd(0x01); // Clear display
    HAL_Delay(2); // Ten Delay jest krytyczny dla komendy Clear
}

void lcd_put_cur(int row, int col)
{
    switch (row)
    {
        case 0:
            col |= 0x80;
            break;
        case 1:
            col |= 0xC0;
            break;
    }
    lcd_send_cmd(col);
}

void lcd_init(void)
{
    // Inicjalizacja 4-bitowa
    HAL_Delay(50);
    lcd_send_cmd(0x30);
    HAL_Delay(5);
    lcd_send_cmd(0x30);
    HAL_Delay(1);
    lcd_send_cmd(0x30);
    HAL_Delay(10);
    lcd_send_cmd(0x20);
    HAL_Delay(10);

    // Konfiguracja
    lcd_send_cmd(0x28); // 4-bit mode, 2 lines, 5x7 font
    HAL_Delay(1);
    lcd_send_cmd(0x08); // Display off
    HAL_Delay(1);
    lcd_send_cmd(0x01); // Clear display
    HAL_Delay(2);
    lcd_send_cmd(0x06); // Entry mode set
    HAL_Delay(1);
    lcd_send_cmd(0x0C); // Display on, cursor off
}

void lcd_send_string(char *str)
{
    while (*str) lcd_send_data(*str++);
}
