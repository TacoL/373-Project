#ifndef LIQUIDCRYSTAL_H
#define LIQUIDCRYSTAL_H

#include "stm32l4xx_hal.h"
#include <stdint.h>

#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80

#define LCD_ENTRYRIGHT 0x00
#define LCD_ENTRYLEFT 0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00

#define LCD_DISPLAYMOVE 0x08
#define LCD_CURSORMOVE 0x00
#define LCD_MOVERIGHT 0x04
#define LCD_MOVELEFT 0x00

#define LCD_8BITMODE 0x10
#define LCD_4BITMODE 0x00
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00
#define LCD_5x10DOTS 0x04
#define LCD_5x8DOTS 0x00

typedef struct {
  uint8_t rs_pin;
  uint8_t rw_pin;
  uint8_t enable_pin;
  uint8_t data_pins[8];

  uint8_t display_function;
  uint8_t display_control;
  uint8_t display_mode;

  uint8_t initialized;
  uint8_t num_lines;
  uint8_t row_offsets[4];
} LiquidCrystal;

// Functions prototypes
void LiquidCrystal_Init(LiquidCrystal *lcd, uint8_t fourbitmode, uint8_t rs, uint8_t rw, uint8_t enable,
                        uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7);
void LiquidCrystal_Begin(LiquidCrystal *lcd, uint8_t cols, uint8_t rows, uint8_t charsize);
void LiquidCrystal_Clear(LiquidCrystal *lcd);
void LiquidCrystal_Home(LiquidCrystal *lcd);
void LiquidCrystal_SetCursor(LiquidCrystal *lcd, uint8_t col, uint8_t row);
void LiquidCrystal_Display(LiquidCrystal *lcd);
void LiquidCrystal_NoDisplay(LiquidCrystal *lcd);
void LiquidCrystal_Cursor(LiquidCrystal *lcd);
void LiquidCrystal_NoCursor(LiquidCrystal *lcd);
void LiquidCrystal_Blink(LiquidCrystal *lcd);
void LiquidCrystal_NoBlink(LiquidCrystal *lcd);
void LiquidCrystal_ScrollDisplayLeft(LiquidCrystal *lcd);
void LiquidCrystal_ScrollDisplayRight(LiquidCrystal *lcd);
void LiquidCrystal_LeftToRight(LiquidCrystal *lcd);
void LiquidCrystal_RightToLeft(LiquidCrystal *lcd);
void LiquidCrystal_AutoScroll(LiquidCrystal *lcd);
void LiquidCrystal_NoAutoScroll(LiquidCrystal *lcd);
void LiquidCrystal_CreateChar(LiquidCrystal *lcd, uint8_t location, uint8_t charmap[]);
void LiquidCrystal_Command(LiquidCrystal *lcd, uint8_t value);
void LiquidCrystal_Write(LiquidCrystal *lcd, uint8_t value);

// Low level functions
void LiquidCrystal_Send(LiquidCrystal *lcd, uint8_t value, uint8_t mode);
void LiquidCrystal_PulseEnable(LiquidCrystal *lcd);
void LiquidCrystal_Write4Bits(LiquidCrystal *lcd, uint8_t value);
void LiquidCrystal_Write8Bits(LiquidCrystal *lcd, uint8_t value);
void LiquidCrystal_PinWrite(uint8_t pin, GPIO_PinState state);
void LiquidCrystal_Delay(uint32_t delay_us);

#endif // LIQUIDCRYSTAL_H
