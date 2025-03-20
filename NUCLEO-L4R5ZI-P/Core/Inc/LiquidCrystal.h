#ifndef LiquidCrystal_h
#define LiquidCrystal_h

#include "main.h"
#include "stm32l4xx_hal.h"

#include <stdio.h>
#include <string.h>
#include <inttypes.h>

// signal states
#define LOW 0
#define HIGH 1

// commands
#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80

// flags for display entry mode
#define LCD_ENTRYRIGHT 0x00
#define LCD_ENTRYLEFT 0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

// flags for display on/off control
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00

// flags for display/cursor shift
#define LCD_DISPLAYMOVE 0x08
#define LCD_CURSORMOVE 0x00
#define LCD_MOVERIGHT 0x04
#define LCD_MOVELEFT 0x00

// flags for function set
#define LCD_8BITMODE 0x10
#define LCD_4BITMODE 0x00
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00
#define LCD_5x10DOTS 0x04
#define LCD_5x8DOTS 0x00

void LiquidCrystal_init(uint8_t fourbitmode);

void LiquidCrystal_begin(uint8_t cols, uint8_t rows, uint8_t charsize); // Make charsize default value LCD_5x8DOTS

void LiquidCrystal_clear();
void LiquidCrystal_home();

void LiquidCrystal_noDisplay();
void LiquidCrystal_display();
void LiquidCrystal_noBlink();
void LiquidCrystal_blink();
void LiquidCrystal_noCursor();
void LiquidCrystal_cursor();
void LiquidCrystal_scrollDisplayLeft();
void LiquidCrystal_scrollDisplayRight();
void LiquidCrystal_leftToRight();
void LiquidCrystal_rightToLeft();
void LiquidCrystal_autoscroll();
void LiquidCrystal_noAutoscroll();

void LiquidCrystal_setRowOffsets(int row1, int row2, int row3, int row4);
void LiquidCrystal_createChar(uint8_t, uint8_t[]);
void LiquidCrystal_setCursor(uint8_t, uint8_t);
uint8_t LiquidCrystal_write(uint8_t);
void LiquidCrystal_command(uint8_t);

void LiquidCrystal_send(uint8_t, uint8_t);
void LiquidCrystal_write4bits(uint8_t);
void LiquidCrystal_write8bits(uint8_t);
void LiquidCrystal_pulseEnable();

void LiquidCrystal_print(char toPrint[]);

#endif
