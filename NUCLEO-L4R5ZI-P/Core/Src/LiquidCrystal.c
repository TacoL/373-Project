#include "LiquidCrystal.h"

extern GPIO_TypeDef* GPIO_PORT[];
extern uint16_t GPIO_PIN[];

void LiquidCrystal_Init(LiquidCrystal *lcd, uint8_t fourbitmode, uint8_t rs, uint8_t rw, uint8_t enable,
                        uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7) {
  lcd->rs_pin = rs;
  lcd->rw_pin = rw;
  lcd->enable_pin = enable;

  lcd->data_pins[0] = d0;
  lcd->data_pins[1] = d1;
  lcd->data_pins[2] = d2;
  lcd->data_pins[3] = d3;
  lcd->data_pins[4] = d4;
  lcd->data_pins[5] = d5;
  lcd->data_pins[6] = d6;
  lcd->data_pins[7] = d7;

  if (fourbitmode)
    lcd->display_function = LCD_4BITMODE | LCD_1LINE | LCD_5x8DOTS;
  else
    lcd->display_function = LCD_8BITMODE | LCD_1LINE | LCD_5x8DOTS;

  LiquidCrystal_Begin(lcd, 16, 1, LCD_5x8DOTS);
}

void LiquidCrystal_Begin(LiquidCrystal *lcd, uint8_t cols, uint8_t rows, uint8_t charsize) {
  if (rows > 1) {
    lcd->display_function |= LCD_2LINE;
  }
  lcd->num_lines = rows;

  LiquidCrystal_SetRowOffsets(lcd, 0x00, 0x40, 0x00 + cols, 0x40 + cols);

  if (charsize != LCD_5x8DOTS && rows == 1) {
    lcd->display_function |= LCD_5x10DOTS;
  }

  LiquidCrystal_PinWrite(lcd->rs_pin, GPIO_PIN_RESET);
  LiquidCrystal_PinWrite(lcd->enable_pin, GPIO_PIN_RESET);
  if (lcd->rw_pin != 255) {
    LiquidCrystal_PinWrite(lcd->rw_pin, GPIO_PIN_RESET);
  }

  for (int i = 0; i < ((lcd->display_function & LCD_8BITMODE) ? 8 : 4); ++i) {
    // Set all data pins as output
    // Use STM32 HAL to configure GPIO pins
    // GPIO_InitTypeDef GPIO_InitStruct;
    // HAL_GPIO_Init(GPIO_PORT[lcd->data_pins[i]], &GPIO_InitStruct);
  }

  LiquidCrystal_Delay(50000);
  if (!(lcd->display_function & LCD_8BITMODE)) {
    LiquidCrystal_Write4Bits(lcd, 0x03);
    LiquidCrystal_Delay(4500);
    LiquidCrystal_Write4Bits(lcd, 0x03);
    LiquidCrystal_Delay(4500);
    LiquidCrystal_Write4Bits(lcd, 0x03);
    LiquidCrystal_Delay(150);
    LiquidCrystal_Write4Bits(lcd, 0x02);
  } else {
    LiquidCrystal_Command(lcd, LCD_FUNCTIONSET | lcd->display_function);
    LiquidCrystal_Delay(4500);
    LiquidCrystal_Command(lcd, LCD_FUNCTIONSET | lcd->display_function);
    LiquidCrystal_Delay(150);
    LiquidCrystal_Command(lcd, LCD_FUNCTIONSET | lcd->display_function);
  }

  LiquidCrystal_Command(lcd, LCD_FUNCTIONSET | lcd->display_function);
  lcd->display_control = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
  LiquidCrystal_Display(lcd);
  LiquidCrystal_Clear(lcd);
  lcd->display_mode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
  LiquidCrystal_Command(lcd, LCD_ENTRYMODESET | lcd->display_mode);
}

void LiquidCrystal_Clear(LiquidCrystal *lcd) {
  LiquidCrystal_Command(lcd, LCD_CLEARDISPLAY);
  LiquidCrystal_Delay(2000);
}

void LiquidCrystal_Home(LiquidCrystal *lcd) {
  LiquidCrystal_Command(lcd, LCD_RETURNHOME);
  LiquidCrystal_Delay(2000);
}

void LiquidCrystal_SetCursor(LiquidCrystal *lcd, uint8_t col, uint8_t row) {
  const size_t max_lines = sizeof(lcd->row_offsets) / sizeof(*lcd->row_offsets);
  if (row >= max_lines) {
    row = max_lines - 1;
  }
  if (row >= lcd->num_lines) {
    row = lcd->num_lines - 1;
  }

  LiquidCrystal_Command(lcd, LCD_SETDDRAMADDR | (col + lcd->row_offsets[row]));
}

// More functions can be implemented as needed, such as display, cursor, blinking, scrolling, etc.

// Low-level functions (Pin writes and delays)
void LiquidCrystal_PinWrite(uint8_t pin, GPIO_PinState state) {
  // Use STM32 HAL to write to a GPIO pin
  HAL_GPIO_WritePin(GPIO_PORT[pin], GPIO_PIN[pin], state);
}

void LiquidCrystal_Delay(uint32_t delay_us) {
  // Use STM32 HAL to introduce a delay (using HAL_Delay or custom function)
  HAL_Delay(delay_us / 1000);
}

// Implement other low-level functions such as sending data, writing 4/8 bits, etc.
