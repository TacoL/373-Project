#include "LiquidCrystal.h"

// When the display powers up, it is configured as follows:
//
// 1. Display clear
// 2. Function set:
//    DL = 1; 8-bit interface data
//    N = 0; 1-line display
//    F = 0; 5x8 dot character font
// 3. Display on/off control:
//    D = 0; Display off
//    C = 0; Cursor off
//    B = 0; Blinking off
// 4. Entry mode set:
//    I/D = 1; Increment by 1
//    S = 0; No shift
//
// Note, however, that resetting doesn't reset the LCD, so we
// can't assume that it's in that state the program starts
void LiquidCrystal_init(uint8_t fourbitmode)
{
  if (fourbitmode)
    _displayfunction = LCD_4BITMODE | LCD_1LINE | LCD_5x8DOTS;
  else
    _displayfunction = LCD_8BITMODE | LCD_1LINE | LCD_5x8DOTS;

  LiquidCrystal_begin(16, 1, LCD_5x8DOTS);
}

void LiquidCrystal_begin(uint8_t cols, uint8_t lines, uint8_t dotsize) {
  if (lines > 1) {
    _displayfunction |= LCD_2LINE;
  }
  _numlines = lines;

  LiquidCrystal_setRowOffsets(0x00, 0x40, 0x00 + cols, 0x40 + cols);

  // for some 1 line displays you can select a 10 pixel high font
  if ((dotsize != LCD_5x8DOTS) && (lines == 1)) {
    _displayfunction |= LCD_5x10DOTS;
  }

  // SEE PAGE 45/46 FOR INITIALIZATION SPECIFICATION!
  // according to datasheet, we need at least 40 ms after power rises above 2.7 V
  // before sending commands. Arduino can turn on way before 4.5 V so we'll wait 50
  HAL_Delay(50);
  // Now we pull both RS and R/W low to begin commands
  HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LCD_EN_GPIO_Port, LCD_EN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LCD_RW_GPIO_Port, LCD_RW_Pin, GPIO_PIN_RESET);

  //put the LCD into 4 bit or 8 bit mode
  if (! (_displayfunction & LCD_8BITMODE)) {
    // this is according to the Hitachi HD44780 datasheet
    // figure 24, pg 46

    // we start in 8bit mode, try to set 4 bit mode
	LiquidCrystal_write4bits(0x03);
	HAL_Delay(5); // wait min 4.1ms

    // second try
    LiquidCrystal_write4bits(0x03);
    HAL_Delay(5); // wait min 4.1ms

    // third go!
    LiquidCrystal_write4bits(0x03);
    HAL_Delay(1);

    // finally, set to 4-bit interface
    LiquidCrystal_write4bits(0x02);
  } else {
    // this is according to the Hitachi HD44780 datasheet
    // page 45 figure 23

    // Send function set command sequence
	LiquidCrystal_command(LCD_FUNCTIONSET | _displayfunction);
	HAL_Delay(5);  // wait more than 4.1 ms

    // second try
    LiquidCrystal_command(LCD_FUNCTIONSET | _displayfunction);
    HAL_Delay(1);

    // third go
    LiquidCrystal_command(LCD_FUNCTIONSET | _displayfunction);
  }

  // finally, set # lines, font size, etc.
  LiquidCrystal_command(LCD_FUNCTIONSET | _displayfunction);

  // turn the display on with no cursor or blinking default
  _displaycontrol = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
  LiquidCrystal_display();

  // clear it off
  LiquidCrystal_clear();

  // Initialize to default text direction (for romance languages)
  _displaymode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
  // set the entry mode
  LiquidCrystal_command(LCD_ENTRYMODESET | _displaymode);

}

void LiquidCrystal_setRowOffsets(int row0, int row1, int row2, int row3)
{
  _row_offsets[0] = row0;
  _row_offsets[1] = row1;
  _row_offsets[2] = row2;
  _row_offsets[3] = row3;
}

/********** high level commands, for the user! */
void LiquidCrystal_clear()
{
  LiquidCrystal_command(LCD_CLEARDISPLAY);  // clear display, set cursor position to zero
  HAL_Delay(2);  // this command takes a long time!
}

void LiquidCrystal_home()
{
  LiquidCrystal_command(LCD_RETURNHOME);  // set cursor position to zero
  HAL_Delay(2);  // this command takes a long time!
}

void LiquidCrystal_setCursor(uint8_t col, uint8_t row)
{
  const size_t max_lines = sizeof(_row_offsets) / sizeof(*_row_offsets);
  if ( row >= max_lines ) {
    row = max_lines - 1;    // we count rows starting w/ 0
  }
  if ( row >= _numlines ) {
    row = _numlines - 1;    // we count rows starting w/ 0
  }

  LiquidCrystal_command(LCD_SETDDRAMADDR | (col + _row_offsets[row]));
}

// Turn the display on/off (quickly)
void LiquidCrystal_noDisplay() {
  _displaycontrol &= ~LCD_DISPLAYON;
  LiquidCrystal_command(LCD_DISPLAYCONTROL | _displaycontrol);
}
void LiquidCrystal_display() {
  _displaycontrol |= LCD_DISPLAYON;
  LiquidCrystal_command(LCD_DISPLAYCONTROL | _displaycontrol);
}

// Turns the underline cursor on/off
void LiquidCrystal_noCursor() {
  _displaycontrol &= ~LCD_CURSORON;
  LiquidCrystal_command(LCD_DISPLAYCONTROL | _displaycontrol);
}
void LiquidCrystal_cursor() {
  _displaycontrol |= LCD_CURSORON;
  LiquidCrystal_command(LCD_DISPLAYCONTROL | _displaycontrol);
}

// Turn on and off the blinking cursor
void LiquidCrystal_noBlink() {
  _displaycontrol &= ~LCD_BLINKON;
  LiquidCrystal_command(LCD_DISPLAYCONTROL | _displaycontrol);
}
void LiquidCrystal_blink() {
  _displaycontrol |= LCD_BLINKON;
  LiquidCrystal_command(LCD_DISPLAYCONTROL | _displaycontrol);
}

// These commands scroll the display without changing the RAM
void LiquidCrystal_scrollDisplayLeft(void) {
  LiquidCrystal_command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT);
}
void LiquidCrystal_scrollDisplayRight(void) {
  LiquidCrystal_command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT);
}

// This is for text that flows Left to Right
void LiquidCrystal_leftToRight(void) {
  _displaymode |= LCD_ENTRYLEFT;
  LiquidCrystal_command(LCD_ENTRYMODESET | _displaymode);
}

// This is for text that flows Right to Left
void LiquidCrystal_rightToLeft(void) {
  _displaymode &= ~LCD_ENTRYLEFT;
  LiquidCrystal_command(LCD_ENTRYMODESET | _displaymode);
}

// This will 'right justify' text from the cursor
void LiquidCrystal_autoscroll(void) {
  _displaymode |= LCD_ENTRYSHIFTINCREMENT;
  LiquidCrystal_command(LCD_ENTRYMODESET | _displaymode);
}

// This will 'left justify' text from the cursor
void LiquidCrystal_noAutoscroll(void) {
  _displaymode &= ~LCD_ENTRYSHIFTINCREMENT;
  LiquidCrystal_command(LCD_ENTRYMODESET | _displaymode);
}

// Allows us to fill the first 8 CGRAM locations
// with custom characters
void LiquidCrystal_createChar(uint8_t location, uint8_t charmap[]) {
  location &= 0x7; // we only have 8 locations 0-7
  LiquidCrystal_command(LCD_SETCGRAMADDR | (location << 3));
  for (int i=0; i<8; i++) {
	LiquidCrystal_write(charmap[i]);
  }
}

/*********** mid level commands, for sending data/cmds */

inline void LiquidCrystal_command(uint8_t value) {
  LiquidCrystal_send(value, LOW);
}

inline uint8_t LiquidCrystal_write(uint8_t value) {
  LiquidCrystal_send(value, HIGH);
  return 1; // assume success
}

/************ low level data pushing commands **********/

// write either command or data, with automatic 4/8-bit selection
void LiquidCrystal_send(uint8_t value, uint8_t mode) {
  HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, mode);

  // if there is a RW pin indicated, set it low to Write
  HAL_GPIO_WritePin(LCD_RW_GPIO_Port, LCD_RW_Pin, GPIO_PIN_RESET);

  if (_displayfunction & LCD_8BITMODE) {
	LiquidCrystal_write8bits(value);
  } else {
	LiquidCrystal_write4bits(value>>4);
	LiquidCrystal_write4bits(value);
  }
}

void LiquidCrystal_pulseEnable(void) {
  HAL_GPIO_WritePin(LCD_EN_GPIO_Port, LCD_EN_Pin, GPIO_PIN_RESET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(LCD_EN_GPIO_Port, LCD_EN_Pin, GPIO_PIN_SET);
  HAL_Delay(1);    // enable pulse must be >450 ns
  HAL_GPIO_WritePin(LCD_EN_GPIO_Port, LCD_EN_Pin, GPIO_PIN_RESET);
  HAL_Delay(1);   // commands need >37 us to settle
}

void LiquidCrystal_write4bits(uint8_t value) {
  HAL_GPIO_WritePin(LCD_D0_GPIO_Port, LCD_D0_Pin, (value >> 0) & 0x01);
  HAL_GPIO_WritePin(LCD_D1_GPIO_Port, LCD_D1_Pin, (value >> 1) & 0x01);
  HAL_GPIO_WritePin(LCD_D2_GPIO_Port, LCD_D2_Pin, (value >> 2) & 0x01);
  HAL_GPIO_WritePin(LCD_D3_GPIO_Port, LCD_D3_Pin, (value >> 3) & 0x01);

  LiquidCrystal_pulseEnable();
}

void LiquidCrystal_write8bits(uint8_t value) {
  HAL_GPIO_WritePin(LCD_D0_GPIO_Port, LCD_D0_Pin, (value >> 0) & 0x01);
  HAL_GPIO_WritePin(LCD_D1_GPIO_Port, LCD_D1_Pin, (value >> 1) & 0x01);
  HAL_GPIO_WritePin(LCD_D2_GPIO_Port, LCD_D2_Pin, (value >> 2) & 0x01);
  HAL_GPIO_WritePin(LCD_D3_GPIO_Port, LCD_D3_Pin, (value >> 3) & 0x01);
  HAL_GPIO_WritePin(LCD_D4_GPIO_Port, LCD_D4_Pin, (value >> 4) & 0x01);
  HAL_GPIO_WritePin(LCD_D5_GPIO_Port, LCD_D5_Pin, (value >> 5) & 0x01);
  HAL_GPIO_WritePin(LCD_D6_GPIO_Port, LCD_D6_Pin, (value >> 6) & 0x01);
  HAL_GPIO_WritePin(LCD_D7_GPIO_Port, LCD_D7_Pin, (value >> 7) & 0x01);

  LiquidCrystal_pulseEnable();
}
