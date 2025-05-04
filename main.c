#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "tm4c123gh6pm.h"
#include <FreeRTOS.h>
#include <task.h>
#include "queue.h"
#include "semphr.h"
#include "Port_Config.h"
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

/***************************************************************************/
	//The potentiometer     -> PE2
	//The Buzzar 						-> PD3 
	//Echo                  -> PB6
	//Trigger								-> PA4
	//GearShift             -> PB7
	//Red Led               -> PD2
	//Blue Led              -> PD1
	//Green Led             -> PD0	
	//Ignition Switch       -> PB4
	//SDL                   -> PB3
	//SCL                   -> PB2
/***************************************************************************/

/*Define The required Inputs and Outputs*/

// GPIO Pin Definitions (Assuming LEDs are connected to PF1 and PF2)
#define RED_LED     0x02  // PF1 (Red LED) -> Unlocked
#define BLUE_LED    0x04  // PF2 (Yellow)
#define GREEN_LED   0x08  // PF2 (Green LED) -> Locked
#define LOCK_BUTTON 0x01  // PF0 (Lock button)
#define UNLOCK_BUTTON 0x10 // PF4 (Unlock button)

//Potentiometer 
#define SPEED_THRESHOLD 40
#define POTENTIOMETER_CHANNEL 1  // AIN1 corresponds to channel 1 on ADC (PE2)
#define ADC_SEQ0 0x01

//The UltraSonic 
#define BUZZER_PIN     0x01  // PB3 (Buzzer) 
#define MAX_DISTANCE 100  // Maximum distance in cm (for Green LED)

//Ignition Switch
#define IGNITION_SWITCH 0x10

#define GEAR_SWITCH_PIN 0x80

// LCD Control Pins (RS, RW, EN)
#define LCD_RS 0x01  // Register Select pin
#define LCD_RW 0x02  // Read/Write pin
#define LCD_EN 0x04  // Enable pin

// I2C Pins (SDA: PC4, SCL: PC5)
#define SCL_PIN 0x10  // PC4
#define SDA_PIN 0x20  // PC5

#define GPIO_PB2_I2C0SCL 0x00010803
#define GPIO_PB3_I2C0SDA 0x00010C03

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

// flags for backlight control
#define LCD_BACKLIGHT 0x08
#define LCD_NOBACKLIGHT 0x00

// #define LCD_CURSOROFF 0x0c

#define En 4 // Enable bit
#define Rw 2 // Read/Write bit
#define Rs 1 // Register select bit

#define LCD_ADDR 0x27  // Your LCD's I2C address
#define LCD_COLS 16
#define LCD_ROWS 2

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

/***********************************************************************************************************/
float speed;
int doorLocked = 0;  		             // 0: Unlocked, 1: Locked
float time; 					             //stores pulse on time 
float distance; 			             // stores measured distance value */
int gearReverse = 0;  	             // 0: Normal Gear, 1: Reverse Gear
uint32_t ignitionSwitchState ;
uint32_t gearSwitchState ;
SemaphoreHandle_t xBinarySemaphore; 
SemaphoreHandle_t xMutex;
int prevGearSwitchState = 1;  // Start with "gear switch not pressed" (active high)
char diststr[20];
/********************************************************************************************************/

void Delay(unsigned long counter)
{
	unsigned long i = 0;
	
	for(i=0; i< counter*1000; i++);
}


/********************************************************************************************************/
void initBuzzer(void) {
    // Enable clock for Port B
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;  // Enable clock for Port B
    while ((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R1) == 0) {}  // Wait until Port B is ready

    // Configure PB0 as output for the buzzer
    GPIO_PORTB_DIR_R |= 0x01;   // Set PB0 as an output pin
    GPIO_PORTB_DEN_R |= 0x01;   // Enable digital function for PB0
}
/********************************************************************************************************/
void initGearSwitch(void) {
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;  // Enable clock for Port B
    while ((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R1) == 0) {}  // Wait until Port B is ready

    GPIO_PORTB_DIR_R &= ~GEAR_SWITCH_PIN;  // Set PB7 as input
    GPIO_PORTB_DEN_R |= GEAR_SWITCH_PIN;   // Enable digital function for PB7
    GPIO_PORTB_PDR_R |= GEAR_SWITCH_PIN;   // Enable pull-down resistor on PB7
}


void gearreverseon(void) {
    gearReverse = 1;
}

void gearreverseoff(void) {
    gearReverse = 0;
}

// FreeRTOS Task to monitor and handle gear switching
void GearSwitchTask(void *pvParameters) {
    while (1) {
			gearSwitchState = GPIO_PORTB_DATA_R & GEAR_SWITCH_PIN;
        // Check if the Gear Switch button is pressed (active high)
        if ((GPIO_PORTB_DATA_R & GEAR_SWITCH_PIN) == 0) {  // Button pressed (PB7 is high)
            gearreverseoff(); 
        }
				else {
					gearreverseon();
				}
        vTaskDelay(pdMS_TO_TICKS(100));  // Short delay before checking again
    }
}

/********************************************************************************************************/

/*LCD implemetation*/

void I2C_Init(void)
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);

  // reset module
  SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);

  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

  // Configure the pin muxing for I2C0 functions on port B2 and B3.
  GPIOPinConfigure(GPIO_PB2_I2C0SCL);
  GPIOPinConfigure(GPIO_PB3_I2C0SDA);

  // Select the I2C function for I2C0 functions on B2 and B3
  GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
  GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

  /* Enable and initialize the I2C0 master module.  Use the system clock for
   * the I2C0 module.  The last parameter sets the I2C data transfer rate.
   * If false the data rate is set to 100kbps and if true the data rate will
   * be set to 400kbps.  For this example we will use a data rate of 100kbps.
   */
  I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), true);

  // clear I2C FIFOs
  HWREG(I2C0_BASE + I2C_O_FIFOCTL) = 80008000;
}
void I2CSendByte(uint8_t slave_addr, uint8_t value)
{
  I2CMasterSlaveAddrSet(I2C0_BASE, slave_addr, false); // set address of slave for master to communicate with
                                                       // put data to be sent into FIFO
  I2CMasterDataPut(I2C0_BASE, value);
  // Initiate send of data from the MCU
  I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);

  // Wait until MCU is done transferring.
  while (I2CMasterBusy(I2C0_BASE))
    ;
}

void expanderWrite(LCD_I2C *display, uint8_t _data)
{
	I2CSendByte(display->_Addr, (int)_data | display->_backlightval);
}

void pulseEnable(LCD_I2C *display, uint8_t _data)
{
	expanderWrite(display, _data | En); // En high
	Delay(0.01);				// enable pulse must be >450ns

	expanderWrite(display, _data & ~En); // En low
	Delay(0.05);				 // commands need > 37us to settle
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
	command(display, LCD_CLEARDISPLAY); // clear display, set cursor position to zero
	Delay(10);									// timer0A_delayMs(10);   // this command takes a long time!
}

void home(LCD_I2C *display)
{
	command(display, LCD_RETURNHOME); // set cursor position to zero
	Delay(10);								  // timer0A_delayMs(10);   // this command takes a long time!
}

void begin(LCD_I2C *display)
{
    I2C_Init();
    display->_numlines = 2;
    display->_displayfunction = LCD_2LINE;

    // Wait for LCD to stabilize after power on
    Delay(100);  // Ensure that LCD is fully initialized

    // Reset expander and turn backlight off
    expanderWrite(display, display->_backlightval);
    Delay(50);

    // Set the LCD to 4-bit mode
    write4bits(display, 0x03 << 4);
    Delay(10);  // wait min 4.1ms

    write4bits(display, 0x03 << 4);
    Delay(10);  // wait min 4.1ms

    write4bits(display, 0x03 << 4);
    Delay(10);

    // Set to 4-bit interface
    write4bits(display, 0x02 << 4);

    // Set function (2-line, 5x8 dots)
    command(display, LCD_FUNCTIONSET | LCD_4BITMODE | LCD_2LINE | LCD_5x8DOTS);

    // Turn display on with no cursor and no blinking
    display->_displaycontrol = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
    command(display, LCD_DISPLAYCONTROL | display->_displaycontrol);

    // Clear display
    clear(display);
    Delay(30);  // Allow time for clearing the display

    // Set entry mode (left to right)
    command(display, LCD_ENTRYMODESET | LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT);

    // Set cursor to home position
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

// Turns the underline cursor on/off
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

// Turn the display on/off (quickly)
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
	return write_string(display, (const uint8_t *)str, strlen(str)); // beware of casting!!! Type safe in this case :)
}
void clearRow(LCD_I2C *display, uint8_t row) {
    // Set the cursor to the start of the specified row
    setCursor(display, 0, row);

    // Print spaces to clear the row
    for (int i = 0; i < display->_cols; i++) {
        print(display, " ");  // Print a space for each column
    }

    // Optionally, reset the cursor back to the start position (first column of the row)
    setCursor(display, 0, row);
}

void displayTextOnLCD(LCD_I2C *display, const char *text, uint8_t row, uint8_t col) {

    // Set the cursor to the specified row and column
    setCursor(display, col, row);
    
    // Display the text on the LCD
    print(display, text);
}

LCD_I2C dis;
/***********************************************************************************************************/

/*
The manual Lock and Unlock functions
*/


// Function to initialize LEDs (PF1 and PF2) for door status
void initLEDs(void) {
    // Enable Port F (GPIO Port F)
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;  // Enable GPIO Port F clock
    while ((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R5) == 0) {}  // Wait for Port F to be ready

    // Set PF1 and PF2 as output (LEDs)
    GPIO_PORTF_DIR_R |= GREEN_LED | BLUE_LED | RED_LED;   // Set PF1 and PF2 as output
    GPIO_PORTF_DEN_R |= GREEN_LED | BLUE_LED | RED_LED;   // Enable digital functionality for PF1 and PF2
}

// Function to initialize buttons (PF0 for lock and PF4 for unlock)
void initButtons(void) {
    // Enable Port F (GPIO Port F)
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;  // Enable GPIO Port F clock
    while ((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R5) == 0) {}  // Wait for Port F to be ready
			
		GPIO_PORTF_LOCK_R = 0x4C4F434B;  // Unlock the PF0 pin
    GPIO_PORTF_CR_R  |= LOCK_BUTTON;  // Commit PF0 pin

    // Set PF0 and PF4 as input (Buttons)
    GPIO_PORTF_DIR_R &= ~(LOCK_BUTTON | UNLOCK_BUTTON);  // Set PF0 and PF4 as input
    GPIO_PORTF_DEN_R |= LOCK_BUTTON | UNLOCK_BUTTON;     // Enable digital functionality for PF0 and PF4
    GPIO_PORTF_PUR_R |= LOCK_BUTTON | UNLOCK_BUTTON;     // Enable pull-up resistors for buttons
}

// Function to toggle door lock and update LCD
void toggleDoorLock(void) {
    if (doorLocked) {
        // Door is unlocked
        //GPIO_PORTF_DATA_R &= ~GREEN_LED;  // Turn off green LED (locked)
        //GPIO_PORTF_DATA_R |= RED_LED;     // Turn on red LED (unlocked)
        doorLocked = 0;
        clearRow(&dis,0);
				displayTextOnLCD(&dis, "Door UnLocked", 0, 0);  // Display "Door Locked" on the second row
    } else {
        // Door is locked
        //GPIO_PORTF_DATA_R &= ~RED_LED;   // Turn off red LED (unlocked)
        //GPIO_PORTF_DATA_R |= GREEN_LED;  // Turn on green LED (locked)
        doorLocked = 1;
        clearRow(&dis,0);
				displayTextOnLCD(&dis, "Door Locked", 0, 0);  // Display "Door Locked" on the second row
    }
}


// FreeRTOS Task to check button presses for manual door lock/unlock
void ButtonControlTask(void *pvParameters) {
    while (1) {
        // Wait for the mutex to be available
        if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
            
            // Check if the Lock button (PF0) is pressed (active low)
            if ((GPIO_PORTF_DATA_R & LOCK_BUTTON) == 0) {  // Lock button pressed (PF0 is low) 
                // Only lock if the door is not already locked
                if (doorLocked == 0) {
                    toggleDoorLock();  // Lock the door
                }
                vTaskDelay(pdMS_TO_TICKS(200));  // Debounce delay to prevent multiple toggles from one press
            }

            // Check if the Unlock button (PF4) is pressed (active low)
            if ((GPIO_PORTF_DATA_R & UNLOCK_BUTTON) == 0) {  // Unlock button pressed (PF4 is low)
                // Only unlock if the door is locked
                if (doorLocked == 1) {
                    toggleDoorLock();  // Unlock the door
                }
                vTaskDelay(pdMS_TO_TICKS(200));  // Debounce delay to prevent multiple toggles from one press
            }
            
            // Release the mutex after the critical section
            xSemaphoreGive(xMutex);
        }

        vTaskDelay(pdMS_TO_TICKS(100));  // Short delay before checking button again
    }
}

/*********************************************************************************************************/ 

/*Implementation of Automatic Locking Mechanism*/


// Function to initialize the ADC for potentiometer reading
void initPotentiometer(void) {
   // Enable the clock for ADC0 and Port E
    SYSCTL_RCGCADC_R |= SYSCTL_RCGCADC_R0;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R4;

    // Wait for peripherals to be ready
    while ((SYSCTL_PRADC_R & SYSCTL_RCGCADC_R0) == 0);
    while ((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R4) == 0);

    // Configure PE2 (AIN1) for analog input
    GPIO_PORTE_AFSEL_R |= 0x04;  // Enable alternate function on PE2
    GPIO_PORTE_DEN_R &= ~0x04;   // Disable digital function on PE2
    GPIO_PORTE_AMSEL_R |= 0x04;  // Enable analog mode on PE2

    // Configure ADC0
    ADC0_ACTSS_R &= ~ADC_SEQ0;   // Disable sample sequencer 0
    ADC0_EMUX_R = 0x0;           // Set sequencer to software trigger
    ADC0_SSMUX0_R = POTENTIOMETER_CHANNEL; // Select AIN1 (channel 1)
    ADC0_SSCTL0_R = 0x06;        // Configure: single sample, end of sequence
    ADC0_ACTSS_R |= ADC_SEQ0;    // Enable sample sequencer 0
}

// Function to read the potentiometer value (returns value between 0 and 4095)
uint16_t ADC_Read(void) {
    // Trigger the conversion
    ADC0_PSSI_R = 0x01; // Start the conversion on sequencer 0
    while ((ADC0_RIS_R & 0x01) == 0); // Wait until conversion is complete
    uint16_t result = ADC0_SSFIFO0_R & 0xFFF; // Read the result
    ADC0_ISC_R = 0x01; // Clear the interrupt flag
    return result;
}

// Function to read the temperature from LM35
float readPotentiometer(void) {
    uint16_t adcValue = ADC_Read();               // Read ADC value
    if (adcValue > 4095) return -1;              // Error handling for invalid ADC value
    float voltage = (adcValue / 4095.0) * 3.3;    // Convert ADC value to voltage
    speed = voltage * 100.0;         // Convert voltage to temperature in ?C
    return speed;
}


// FreeRTOS Task to check speed and automatically lock doors based on speed
void SpeedCheckTask(void *pvParameters) {
    while (1) {
			if (gearReverse ==0){
					        // Read speed from potentiometer (ADC value mapped to 0 - 100 km/h)
        float potentiometerValue = readPotentiometer();

        // If the potentiometer value is invalid, skip the logic
        if (potentiometerValue == -1) {
            vTaskDelay(pdMS_TO_TICKS(1000));  // 1-second delay before checking again
            continue;
        }

        // Check if the speed exceeds the threshold for auto-locking
        if (potentiometerValue >= SPEED_THRESHOLD && doorLocked == 0) {  // 40 km/h threshold for automatic locking
            toggleDoorLock();  // Lock the door
        }

        // Wait for the mutex before updating the display
        if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
            // Update the LCD with the current speed and door status
            char speedStr[20];
            snprintf(speedStr, sizeof(speedStr), "Speed = %.1f km/h", potentiometerValue);
            
            // Print the speed on the first row (row 1)
						clearRow(&dis,1);
            displayTextOnLCD(&dis, speedStr, 1, 0);  // Display speed on the first row
            // Release the mutex after updating the display
            xSemaphoreGive(xMutex);
        }
			}
			  // Wait for a second before checking again
        vTaskDelay(pdMS_TO_TICKS(1000));  // 1-second delay
    }
}

/********************************************************************************************************/

/*UltraSonic Sensor implemetation*/

/* This function captures consecutive rising and falling edges of a periodic signal */
/* from Timer Block 0 Timer A and returns the time difference (the period of the signal). */

/* Create one microsecond second delay using Timer block 1 and sub timer A */

void initUltrasonicLEDs(void) {
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;  // Enable clock for Port F
    while ((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R5) == 0) {}  // Wait until Port F is ready

    GPIO_PORTF_DIR_R |= (0x02 | 0x04 | 0x08); 
    GPIO_PORTF_DEN_R |= (0x02 | 0x04 | 0x08); 
}


void Delay_MicroSecond(int time)
{
    int i;
    SYSCTL->RCGCTIMER |= 2;     /* enable clock to Timer Block 1 */
    TIMER1->CTL = 0;            /* disable Timer before initialization */
    TIMER1->CFG = 0x04;         /* 16-bit option */ 
    TIMER1->TAMR = 0x02;        /* periodic mode and down-counter */
    TIMER1->TAILR = 16 - 1;  /* TimerA interval load value reg */
    TIMER1->ICR = 0x1;          /* clear the TimerA timeout flag */
    TIMER1->CTL |= 0x01;        /* enable Timer A after initialization */

    for(i = 0; i < time; i++)
    {
        while ((TIMER1->RIS & 0x1) == 0) ;      /* wait for TimerA timeout flag */
        TIMER1->ICR = 0x1;      /* clear the TimerA timeout flag */
    }
}

float Measure_distance(void)
{
    int lastEdge, thisEdge;
	
	  /* Given 10us trigger pulse */
	  GPIOA->DATA &= ~(1<<4); /* make trigger  pin high */
	  Delay_MicroSecond(10); /*10 seconds delay */
	  GPIOA->DATA |= (1<<4); /* make trigger  pin high */
	  Delay_MicroSecond(10); /*10 seconds delay */
	  GPIOA->DATA &= ~(1<<4); /* make trigger  pin low */

 	while(1)
	{
    TIMER0->ICR = 4;            /* clear timer0A capture flag */
    while((TIMER0->RIS & 4) == 0) ;    /* wait till captured */
	  if(GPIOB->DATA & (1<<6)) /*check if rising edge occurs */
		{
    lastEdge = TIMER0->TAR;     /* save the timestamp */
		/* detect falling edge */
    TIMER0->ICR = 4;            /* clear timer0A capture flag */
    while((TIMER0->RIS & 4) == 0) ;    /* wait till captured */
    thisEdge = TIMER0->TAR;     /* save the timestamp */
		return (thisEdge - lastEdge); /* return the time difference */
		}
	}
}

/* Timer0A initialization function */
/* Initialize Timer0A in input-edge time mode with up-count mode */
void Timer0ACapture_init(void)
{
    SYSCTL->RCGCTIMER |= 1;     /* enable clock to Timer Block 0 */
    SYSCTL->RCGCGPIO |= 2;      /* enable clock to PORTB */
    
    GPIOB->DIR &= ~0x40;        /* make PB6 an input pin */
    GPIOB->DEN |= 0x40;         /* make PB6 as digital pin */
    GPIOB->AFSEL |= 0x40;       /* use PB6 alternate function */
    GPIOB->PCTL &= ~0x0F000000;  /* configure PB6 for T0CCP0 */
    GPIOB->PCTL |= 0x07000000;
    
	  /* PB2 as a digital output signal to provide trigger signal */
	  SYSCTL->RCGCGPIO |= 1;      /* enable clock to PORTA */
	  GPIOA->DIR |=(1<<4);         /* set PB2 as a digial output pin */
	  GPIOA->DEN |=(1<<4);         /* make PB2 as digital pin */

    TIMER0->CTL &= ~1;          /* disable timer0A during setup */
    TIMER0->CFG = 4;            /* 16-bit timer mode */
    TIMER0->TAMR = 0x17;        /* up-count, edge-time, capture mode */
    TIMER0->CTL |=0x0C;        /* capture the rising edge */
    TIMER0->CTL |= (1<<0);           /* enable timer0A */
}


// Function to control the RGB LED based on proximity
void controlLEDAndBuzzer(uint32_t distance) {
    // Turn off all LEDs first
    GPIO_PORTF_DATA_R &= ~(0x02 | 0x04 | 0x08);

    // Turn on LED based on distance
    if (distance > 100) {  // Safe zone (Green LED)
        GPIO_PORTF_DATA_R |= 0x08;
    } else if (distance > 30) {  // Caution zone (Yellow LED)
        GPIO_PORTF_DATA_R |= 0x04;
    } else {  // Danger zone (Red LED)
        GPIO_PORTF_DATA_R |= 0x02;
    }

    // Control buzzer: beep faster as the distance decreases
    if (distance > 100) {  // Danger zone
        GPIO_PORTB_DATA_R |= BUZZER_PIN;  // Turn on buzzer
        vTaskDelay(pdMS_TO_TICKS(100));  // Buzzer beep for a short time
        GPIO_PORTB_DATA_R &= ~BUZZER_PIN;  // Turn off buzzer
        vTaskDelay(pdMS_TO_TICKS(100));  // Delay for faster beep
    } else if (distance > 30){
				GPIO_PORTB_DATA_R |= BUZZER_PIN;  // Turn on buzzer
        vTaskDelay(pdMS_TO_TICKS(30));  // Buzzer beep for a short time
        GPIO_PORTB_DATA_R &= ~BUZZER_PIN;  // Turn off buzzer
        vTaskDelay(pdMS_TO_TICKS(30));  // Delay for faster beep
		}
		else {
        GPIO_PORTB_DATA_R |= BUZZER_PIN;  // Turn on buzzer
    }
}


// FreeRTOS Task to handle ultrasonic sensor and display proximity
void UltrasonicTask(void *pvParameters) {
	Timer0ACapture_init();  /*initialize Timer0A in edge edge time */
    while (1) {
			if (gearReverse ==1){
			  time = Measure_distance(); /* take pulse duration measurement */ 
				distance = (time * 10625)/10000000; /* convert pulse duration into distance */
        
				clearRow(&dis,1);
				char diststr[20];
        snprintf(diststr, sizeof(diststr), "Dist = %.1f cm", distance);
				// Take the mutex before updating the display
        if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
             // Display the distance on the LCD (first row)
             displayTextOnLCD(&dis, diststr, 1, 0);  
						// Release the mutex after updating the display
             xSemaphoreGive(xMutex);
         }
				controlLEDAndBuzzer(distance);  // Control LEDs and buzzer based on distance
			}
			else {
				GPIO_PORTF_DATA_R &= ~(0x02 | 0x04 | 0x08 | BUZZER_PIN);
			}
			vTaskDelay(pdMS_TO_TICKS(500));  // Delay for 500ms before next measurement
    }
}
/********************************************************************************************************/
void initIgnitionSwitch(void) {
    SYSCTL->RCGCGPIO |= (1 << 1);                        // Clock to PortB
    while((SYSCTL->PRGPIO & (1 << 1)) == 0);             // Ensure Clock worked for PortB
    GPIOB->DIR &= ~(1 << 4);                             // PB4 input (Ignition switch)
    GPIOB->DEN |= (1 << 4);                              // Digital enable for PB4
    GPIOB->PDR |= (1 << 4);                              // Pull-down resistor on PB4 (reads LOW when not pressed)

    GPIOB->IS &= ~(1 << 4);                              // Edge sensitive for PB4 (clear previous configuration)
    GPIOB->IEV &= ~(1 << 4);                             // Falling edge (detect HIGH to LOW transition)
    GPIOB->ICR |= (1 << 4);                              // Clear any pending interrupts on PB4
    GPIOB->IM |= (1 << 4);                               // Unmask interrupt on PB4
    NVIC_EnableIRQ(GPIOB_IRQn);                          // Enable GPIOB interrupt in NVIC
}

// This is the interrupt handler for GPIO Port B
void GPIOB_Handler(void) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    GPIOB->ICR = (1 << 4);  // Clear PB4 interrupt flag (this clears the flag to avoid re-triggering)

    // Give the semaphore from ISR to unblock the task
    xSemaphoreGiveFromISR(xBinarySemaphore, &xHigherPriorityTaskWoken);

    // If a higher priority task was woken, yield the processor to it
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void IgnitionMonitorTask(void *pvParameters) {
    while (1) {
        ignitionSwitchState = GPIO_PORTB_DATA_R & (1 << 4);  // Check the state of PB4 (ignition switch)

        // Wait for the semaphore to be given from the ISR (this happens when the falling edge occurs)
        if (xSemaphoreTake(xBinarySemaphore, portMAX_DELAY)) {
            // Check if Ignition Switch is OFF (PB4 is LOW)
            if ((GPIO_PORTB_DATA_R & (1 << 4)) == 0) {
                if (doorLocked == 1) {
                    toggleDoorLock();  // Unlock the doors when ignition is OFF
                }
            }
        }
    }
}
/*********************************************************************************************************/ 


int main(void) {
	
  // Initialize LEDs (PF1, PF2 for door lock/unlock status)
	LCDI2CInit(&dis, LCD_ADDR, LCD_COLS, LCD_ROWS);
	initBuzzer();
	initButtons();
  //initIgnitionSwitch();
	initUltrasonicLEDs();
	initGearSwitch();
  initPotentiometer(); 
	//xBinarySemaphore = xSemaphoreCreateBinary();
	xMutex = xSemaphoreCreateMutex();
	
  // Create FreeRTOS tasks
  xTaskCreate(ButtonControlTask, "Button Control Task", 128, NULL, 1, NULL);
	//xTaskCreate(IgnitionMonitorTask, "Ignition Monitor", 128, NULL, 1, NULL);
	xTaskCreate(SpeedCheckTask, "Speed Check Task", 128, NULL, 2, NULL);
	xTaskCreate(UltrasonicTask, "Ultrasonic Task", 128, NULL, 1, NULL);
	xTaskCreate(GearSwitchTask, "Gear Switch Task", 128, NULL, 1, NULL);
	

  // Start the FreeRTOS scheduler
	vTaskStartScheduler();

	return 0; //This part will not reach 
}
