#ifndef LCD_H
#define LCD_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/debug.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"
#include "driverlib/i2c.h"
#include "inc/hw_i2c.h"

#define GPIO_PB2_I2C0SCL 0x00010803
#define GPIO_PB3_I2C0SDA 0x00010C03
// LCD Commands
#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80

// Flags for display entry mode
#define LCD_ENTRYRIGHT 0x00
#define LCD_ENTRYLEFT 0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

// Flags for display on/off control
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00

// Flags for display/cursor shift
#define LCD_DISPLAYMOVE 0x08
#define LCD_CURSORMOVE 0x00
#define LCD_MOVERIGHT 0x04
#define LCD_MOVELEFT 0x00

// Flags for function set
#define LCD_8BITMODE 0x10
#define LCD_4BITMODE 0x00
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00
#define LCD_5x10DOTS 0x04
#define LCD_5x8DOTS 0x00

// Flags for backlight control
#define LCD_BACKLIGHT 0x08
#define LCD_NOBACKLIGHT 0x00

// Define pins
#define En 4 // Enable bit
#define Rw 2 // Read/Write bit
#define Rs 1 // Register select bit

// LCD I2C Address
#define LCD_ADDR 0x27 // Your LCD's I2C address
#define LCD_COLS 16
#define LCD_ROWS 2

// LCD_I2C structure
typedef struct
{
    uint8_t _Addr;
    uint8_t _displayfunction;
    uint8_t _displaycontrol;
    uint8_t _displaymode;
    uint8_t _numlines;
    uint8_t _cols;
    uint8_t _rows;
    uint8_t _backlightval;
} LCD_I2C;

// Function declarations
void I2C_Init(void);
void I2CSendByte(uint8_t slave_addr, uint8_t value);
void expanderWrite(LCD_I2C *display, uint8_t _data);
void pulseEnable(LCD_I2C *display, uint8_t _data);
void write4bits(LCD_I2C *display, uint8_t value);
void send(LCD_I2C *display, uint8_t value, uint8_t mode);
void command(LCD_I2C *display, uint8_t value);
size_t write(LCD_I2C *display, uint8_t value);
size_t write_string(LCD_I2C *display, const uint8_t *buffer, size_t size);
void display(LCD_I2C *display);
void configDisplay(LCD_I2C *display);
void clear(LCD_I2C *display);
void home(LCD_I2C *display);
void begin(LCD_I2C *display);
void LCDI2CInit(LCD_I2C *display, uint8_t lcd_address, uint8_t lcd_cols, uint8_t lcd_rows);
void setCursor(LCD_I2C *display, uint8_t col, uint8_t row);
void noCursor(LCD_I2C *display);
void backlight(LCD_I2C *display);
void scrollDisplayRight(LCD_I2C *display);
void noDisplay(LCD_I2C *display);
size_t printChar(LCD_I2C *display, const char c);
size_t print(LCD_I2C *display, const char *str);
void clearRow(LCD_I2C *display, uint8_t row);
void displayTextOnLCD(LCD_I2C *display, const char *text, uint8_t row, uint8_t col);

extern LCD_I2C dis;  
#endif // LCD_H
