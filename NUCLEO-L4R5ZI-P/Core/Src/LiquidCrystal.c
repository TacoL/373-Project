#include "LiquidCrystal.h"
#include <stddef.h>
#include <stdint.h>
#include "stm32l4xx_hal.h"  // Make sure to include appropriate STM32 HAL header

// Pins and LCD state variables
static uint8_t _rs_pin, _rw_pin, _enable_pin;
static uint8_t _data_pins[8];
static uint8_t _displayfunction, _displaycontrol, _displaymode;
static uint8_t _numlines;
static uint8_t _row_offsets[4];

// Function to set row offsets
void LiquidCrystal_setRowOffsets(int row0, int row1, int row2, int row3)
{
    _row_offsets[0] = row0;
    _row_offsets[1] = row1;
    _row_offsets[2] = row2;
    _row_offsets[3] = row3;
}

// Initialize LCD settings
void LiquidCrystal_init(uint8_t rs, uint8_t rw, uint8_t enable,
                        uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3,
                        uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7)
{
    _rs_pin = rs;
    _rw_pin = rw;
    _enable_pin = enable;

    _data_pins[0] = d0;
    _data_pins[1] = d1;
    _data_pins[2] = d2;
    _data_pins[3] = d3;
    _data_pins[4] = d4;
    _data_pins[5] = d5;
    _data_pins[6] = d6;
    _data_pins[7] = d7;

    _displayfunction = LCD_8BITMODE | LCD_1LINE | LCD_5x8DOTS;  // Default setup
}

// Begin LCD configuration
void LiquidCrystal_begin(uint8_t cols, uint8_t rows, uint8_t charsize)
{
    if (rows > 1)
        _displayfunction |= LCD_2LINE;

    _numlines = rows;
    LiquidCrystal_setRowOffsets(0x00, 0x40, 0x00 + cols, 0x40 + cols);

    // Initialization of GPIO pins
    HAL_GPIO_WritePin(GPIOB, _rs_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, _enable_pin, GPIO_PIN_RESET);
    if (_rw_pin != 255)
        HAL_GPIO_WritePin(GPIOB, _rw_pin, GPIO_PIN_RESET);

    // Setting the data pins as output
    for (int i = 0; i < ((_displayfunction & LCD_8BITMODE) ? 8 : 4); i++)
        HAL_GPIO_WritePin(GPIOB, _data_pins[i], GPIO_PIN_RESET);

    HAL_Delay(50);  // Wait 50ms after power on

    // Sending initialization sequence
    LiquidCrystal_send(0x03, 0);
    HAL_Delay(5);
    LiquidCrystal_send(0x03, 0);
    HAL_Delay(5);
    LiquidCrystal_send(0x03, 0);
    HAL_Delay(1);
    LiquidCrystal_send(0x02, 0);  // Set to 4-bit mode

    // Set LCD function
    LiquidCrystal_command(LCD_FUNCTIONSET | _displayfunction);
    HAL_Delay(1);
    LiquidCrystal_command(LCD_DISPLAYCONTROL | LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF);

    // Clear the display
    LiquidCrystal_clear();
    // Set entry mode
    LiquidCrystal_command(LCD_ENTRYMODESET | LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT);
}

// Clears the display
void LiquidCrystal_clear(void)
{
    LiquidCrystal_command(LCD_CLEARDISPLAY);
    HAL_Delay(2);  // Clear takes longer time
}

// Moves the cursor to the home position
void LiquidCrystal_home(void)
{
    LiquidCrystal_command(LCD_RETURNHOME);
    HAL_Delay(2);
}

// Display on/off control
void LiquidCrystal_noDisplay(void) { _displaycontrol &= ~LCD_DISPLAYON; LiquidCrystal_command(LCD_DISPLAYCONTROL | _displaycontrol); }
void LiquidCrystal_display(void) { _displaycontrol |= LCD_DISPLAYON; LiquidCrystal_command(LCD_DISPLAYCONTROL | _displaycontrol); }

// Blink control
void LiquidCrystal_noBlink(void) { _displaycontrol &= ~LCD_BLINKON; LiquidCrystal_command(LCD_DISPLAYCONTROL | _displaycontrol); }
void LiquidCrystal_blink(void) { _displaycontrol |= LCD_BLINKON; LiquidCrystal_command(LCD_DISPLAYCONTROL | _displaycontrol); }

// Cursor control
void LiquidCrystal_noCursor(void) { _displaycontrol &= ~LCD_CURSORON; LiquidCrystal_command(LCD_DISPLAYCONTROL | _displaycontrol); }
void LiquidCrystal_cursor(void) { _displaycontrol |= LCD_CURSORON; LiquidCrystal_command(LCD_DISPLAYCONTROL | _displaycontrol); }

// Scroll control
void LiquidCrystal_scrollDisplayLeft(void) { LiquidCrystal_command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT); }
void LiquidCrystal_scrollDisplayRight(void) { LiquidCrystal_command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT); }

// Text direction
void LiquidCrystal_leftToRight(void) { LiquidCrystal_command(LCD_ENTRYMODESET | LCD_ENTRYLEFT); }
void LiquidCrystal_rightToLeft(void) { LiquidCrystal_command(LCD_ENTRYMODESET | LCD_ENTRYRIGHT); }

// Autoscroll text
void LiquidCrystal_autoscroll(void) { LiquidCrystal_command(LCD_ENTRYMODESET | LCD_ENTRYSHIFTINCREMENT); }
void LiquidCrystal_noAutoscroll(void) { LiquidCrystal_command(LCD_ENTRYMODESET | LCD_ENTRYSHIFTDECREMENT); }

// Set custom character in CGRAM
void LiquidCrystal_createChar(uint8_t location, uint8_t charmap[])
{
    location &= 0x7;  // We only have 8 locations 0-7
    LiquidCrystal_command(LCD_SETCGRAMADDR | (location << 3));
    for (int i = 0; i < 8; i++)
    {
        LiquidCrystal_write(charmap[i]);
    }
}

// Set cursor position
void LiquidCrystal_setCursor(uint8_t col, uint8_t row)
{
    LiquidCrystal_command(LCD_SETDDRAMADDR | (col + _row_offsets[row]));
}

// Send command to LCD
void LiquidCrystal_command(uint8_t value)
{
    LiquidCrystal_send(value, 0);
}

// Write a value to LCD (data or command)
void LiquidCrystal_write(uint8_t value)
{
    LiquidCrystal_send(value, 1);
}

// Send data/command to the LCD
void LiquidCrystal_send(uint8_t value, uint8_t mode)
{
    HAL_GPIO_WritePin(GPIOB, _rs_pin, mode);  // Set RS pin
    if (_rw_pin != 255)
        HAL_GPIO_WritePin(GPIOB, _rw_pin, GPIO_PIN_RESET);  // Set RW pin low for write

    if (_displayfunction & LCD_8BITMODE)
        LiquidCrystal_write8bits(value);  // 8-bit mode
    else
        LiquidCrystal_write4bits(value >> 4);  // 4-bit mode
        LiquidCrystal_write4bits(value);
}

// Write 4 bits to LCD
void LiquidCrystal_write4bits(uint8_t value)
{
    for (int i = 0; i < 4; i++)
        HAL_GPIO_WritePin(GPIOB, _data_pins[i], (value >> i) & 0x01);

    LiquidCrystal_pulseEnable();
}

// Write 8 bits to LCD
void LiquidCrystal_write8bits(uint8_t value)
{
    for (int i = 0; i < 8; i++)
        HAL_GPIO_WritePin(GPIOB, _data_pins[i], (value >> i) & 0x01);

    LiquidCrystal_pulseEnable();
}

// Pulse the enable pin to latch data
void LiquidCrystal_pulseEnable(void)
{
    HAL_GPIO_WritePin(GPIOB, _enable_pin, GPIO_PIN_RESET);
    HAL_Delay(1);  // Minimum pulse width is 450ns
    HAL_GPIO_WritePin(GPIOB, _enable_pin, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, _enable_pin, GPIO_PIN_RESET);
    HAL_Delay(1);  // Commands need more than 37 us to settle
}
