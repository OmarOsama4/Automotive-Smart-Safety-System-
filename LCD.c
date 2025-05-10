#include "lcd.h"
#include "stdint.h"

LCD_I2C dis;
void Delay(unsigned long counter)
{
    unsigned long i = 0;

    for (i = 0; i < counter * 1000; i++)
        ;
}

// I2C Initialization
void I2C_Init(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
    SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), true);
    HWREG(I2C0_BASE + I2C_O_FIFOCTL) = 80008000;
}

void I2CSendByte(uint8_t slave_addr, uint8_t value)
{
    I2CMasterSlaveAddrSet(I2C0_BASE, slave_addr, false);
    I2CMasterDataPut(I2C0_BASE, value);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);

    while (I2CMasterBusy(I2C0_BASE))
        ;
}

void expanderWrite(LCD_I2C *display, uint8_t _data)
{
    I2CSendByte(display->_Addr, (int)_data | display->_backlightval);
}

void pulseEnable(LCD_I2C *display, uint8_t _data)
{
    expanderWrite(display, _data | En);  // En high
    Delay(0.01);                         // Enable pulse must be >450ns
    expanderWrite(display, _data & ~En); // En low
    Delay(0.05);                         // Commands need > 37us to settle
}

void write4bits(LCD_I2C *display, uint8_t value)
{
    expanderWrite(display, value);
    pulseEnable(display, value);
}

void send(LCD_I2C *display, uint8_t value, uint8_t mode)
{
    uint8_t highnib = value & 0xf0;
    uint8_t lownib = (value << 4) & 0xf0;
    write4bits(display, (highnib) | mode);
    write4bits(display, (lownib) | mode);
}

void command(LCD_I2C *display, uint8_t value)
{
    send(display, value, 0);
}

size_t write(LCD_I2C *display, uint8_t value)
{
    send(display, value, Rs);
    return 1;
}

size_t write_string(LCD_I2C *display, const uint8_t *buffer, size_t size)
{
    size_t n = 0;
    while (size--)
    {
        if (write(display, *buffer++))
            n++;
        else
            break;
    }
    return n;
}

void display(LCD_I2C *display)
{
    display->_displaycontrol |= LCD_DISPLAYON;
    command(display, LCD_DISPLAYCONTROL | display->_displaycontrol);
}

void configDisplay(LCD_I2C *display)
{
    display->_displaycontrol |= LCD_DISPLAYON;
    command(display, LCD_DISPLAYCONTROL | display->_displaycontrol);
}

void clear(LCD_I2C *display)
{
    command(display, LCD_CLEARDISPLAY);
    Delay(10); // Timer0A_delayMs(10);   // This command takes a long time!
}

void home(LCD_I2C *display)
{
    command(display, LCD_RETURNHOME);
    Delay(10); // Timer0A_delayMs(10);   // This command takes a long time!
}

void begin(LCD_I2C *display)
{
    I2C_Init();
    display->_numlines = 2;
    display->_displayfunction = LCD_2LINE;

    Delay(100); // Ensure that LCD is fully initialized
    expanderWrite(display, display->_backlightval);
    Delay(50);

    write4bits(display, 0x03 << 4);
    Delay(10);
    write4bits(display, 0x03 << 4);
    Delay(10);
    write4bits(display, 0x03 << 4);
    Delay(10);
    write4bits(display, 0x02 << 4);

    command(display, LCD_FUNCTIONSET | LCD_4BITMODE | LCD_2LINE | LCD_5x8DOTS);
    display->_displaycontrol = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
    command(display, LCD_DISPLAYCONTROL | display->_displaycontrol);

    clear(display);
    Delay(30); // Allow time for clearing the display
    command(display, LCD_ENTRYMODESET | LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT);
    home(display);
}

void LCDI2CInit(LCD_I2C *display, uint8_t lcd_address, uint8_t lcd_cols, uint8_t lcd_rows)
{
    display->_Addr = lcd_address;
    display->_cols = lcd_cols;
    display->_rows = lcd_rows;
    display->_backlightval = LCD_BACKLIGHT;

    display->_displayfunction = LCD_4BITMODE | LCD_1LINE | LCD_5x8DOTS;
    begin(display);
}

void setCursor(LCD_I2C *display, uint8_t col, uint8_t row)
{
    int row_offsets[] = {0x00, 0x40, 0x14 - (20 - 16), 0x54 - (20 - 16)};
    if (row > 2)
    {
        row = 2 - 1; // we count rows starting w/0
    }
    command(display, LCD_SETDDRAMADDR | (col + row_offsets[row]));
}

void noCursor(LCD_I2C *display)
{
    display->_displaycontrol &= ~LCD_CURSORON;
    command(display, LCD_DISPLAYCONTROL | display->_displaycontrol);
}

void backlight(LCD_I2C *display)
{
    display->_backlightval = LCD_BACKLIGHT;
    expanderWrite(display, 0);
}

void scrollDisplayRight(LCD_I2C *display)
{
    command(display, LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT);
}

void noDisplay(LCD_I2C *display)
{
    display->_displaycontrol &= ~LCD_DISPLAYON;
    command(display, LCD_DISPLAYCONTROL | display->_displaycontrol);
}

size_t printChar(LCD_I2C *display, const char c)
{
    return write(display, c);
}

size_t print(LCD_I2C *display, const char *str)
{
    return write_string(display, (const uint8_t *)str, strlen(str));
}

void clearRow(LCD_I2C *display, uint8_t row)
{
    setCursor(display, 0, row);
    for (int i = 0; i < display->_cols; i++)
    {
        print(display, " ");
    }
    setCursor(display, 0, row);
}

void displayTextOnLCD(LCD_I2C *display, const char *text, uint8_t row, uint8_t col)
{
    setCursor(display, col, row);
    print(display, text);
}
