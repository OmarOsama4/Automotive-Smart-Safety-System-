#include "tm4c123gh6pm.h"          // Use the TM4C header for GPIO and system control
#include "FreeRTOS.h"
#include "task.h"
#include <stdint.h> 
#include "semphr.h"
/***************************************************************************/
	//The RGB lights for the ultrasonic sensor on PF1, PF2, PF3
	//The ultrasonic sensor -> PA3 & PA4
	//The potentiometer     -> PE2
	//The Buzzar 						-> PF0 
	//Echo                  -> PB6
	//Trigger								-> PA4
	//GearShift             -> PB7
	//Red Led               -> PB2
	//Blue Led              -> PB1
	//Green Led             -> PB0	
	//Ignition Switch       -> PB4
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
#define BUZZER_PIN     0x08  // PB3 (Buzzer) 
#define MAX_DISTANCE 100  // Maximum distance in cm (for Green LED)

//Ignition Switch
#define IGNITION_SWITCH 0x10

#define GEAR_SWITCH_PIN 0x80

#define LCD_ADDR 0x27  // LCD I2C address (can vary depending on the module)
#define SCL_PIN (1 << 4)  // PC4 for SCL
#define SDA_PIN (1 << 5)  // PC5 for SDA
#define I2C_MCR_MSTR   0x00000010  // I2C Master Enable bit
#define I2C_MCR_IRS    0x00000001  // I2C Interrupt Request Status bit

/***********************************************************************************************************/
float speed;
int doorLocked = 0;  		             // 0: Unlocked, 1: Locked
uint32_t time; 					             //stores pulse on time 
uint32_t distance; 			             // stores measured distance value */
int gearReverse = 0;  	             // 0: Normal Gear, 1: Reverse Gear
uint32_t ignitionSwitchState ;
uint32_t gearSwitchState ;
SemaphoreHandle_t xBinarySemaphore; 
char globalMessage[100];
int prevGearSwitchState = 1;  // Start with "gear switch not pressed" (active high)
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

// Function to toggle door lock state and control LEDs
void toggleDoorLock(void) {
    if (doorLocked) {
        // Door is unlocked
        GPIO_PORTF_DATA_R &= ~GREEN_LED;  // Turn off green LED (locked)
        GPIO_PORTF_DATA_R |= RED_LED;    // Turn on red LED (unlocked)
        doorLocked = 0;
    } else {
        // Door is locked
        GPIO_PORTF_DATA_R &= ~RED_LED;   // Turn off red LED (unlocked)
        GPIO_PORTF_DATA_R |= GREEN_LED;  // Turn on green LED (locked)
        doorLocked = 1;
    }
}

// FreeRTOS Task to check button presses for manual door lock/unlock
void ButtonControlTask(void *pvParameters) {
    while (1) {
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
    speed = voltage * 100.0;         // Convert voltage to temperature in °C
    return speed;
}


// FreeRTOS Task to check speed and automatically lock doors based on speed
void SpeedCheckTask(void *pvParameters) {
    while (1) {
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
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;  // Enable clock for Port B
    while ((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R1) == 0) {}  // Wait until Port B is ready

    // Configure PB0 (Green LED), PB1 (Blue LED), PB2 (Red LED) as output
    GPIO_PORTB_DIR_R |= (0x01 | 0x02 | 0x04| 0x08); // Set PB0, PB1, PB2 as outputs
    GPIO_PORTB_DEN_R |= (0x01 | 0x02 | 0x04| 0x08); // Enable digital function on PB0, PB1, PB2
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

uint32_t Measure_distance(void)
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

void Delay(unsigned long counter)
{
	unsigned long i = 0;
	
	for(i=0; i< counter*1000; i++);
}


// Function to control the RGB LED based on proximity
void controlLEDAndBuzzer(uint32_t distance) {
    // Turn off all LEDs first
    GPIO_PORTB_DATA_R &= ~(0x01 | 0x02 | 0x04);

    // Turn on LED based on distance
    if (distance > 100) {  // Safe zone (Green LED)
        GPIO_PORTB_DATA_R |= 0x01;
    } else if (distance > 30) {  // Caution zone (Yellow LED)
        GPIO_PORTB_DATA_R |= 0x02;
    } else {  // Danger zone (Red LED)
        GPIO_PORTB_DATA_R |= 0x04;
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
        controlLEDAndBuzzer(distance);  // Control LEDs and buzzer based on distance
			}
			else {
				GPIO_PORTB_DATA_R &= ~(0x01 | 0x02 | 0x04 | 0x08);
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

    GPIOB->IS &= ~(1 << 4);                              // Edge sensitive for PB4
    GPIOB->IEV &= ~(1 << 4);                             // Falling edge (detect press)
    GPIOB->ICR |= (1 << 4);                              // Clear any pending interrupts on PB4
    GPIOB->IM |= (1 << 4);                               // Unmask interrupt on PB4
    NVIC_EnableIRQ(GPIOB_IRQn);                          // Enable GPIOB interrupt in NVIC
}

// This is the interrupt handler for GPIO Port B
void GPIOB_Handler(void) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    GPIOB->ICR = (1 << 4);  // Clear PB4 interrupt flag

    // Give the semaphore from ISR to unblock the task
    xSemaphoreGiveFromISR(xBinarySemaphore, &xHigherPriorityTaskWoken);

    // If a higher priority task was woken, yield the processor to it
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void IgnitionMonitorTask(void *pvParameters) {
    while (1) {
			ignitionSwitchState = GPIO_PORTB_DATA_R & IGNITION_SWITCH;
        // Wait for the semaphore to be given from the ISR
        if (xSemaphoreTake(xBinarySemaphore, portMAX_DELAY)) {
            // Check if Ignition Switch is OFF (PB4 is LOW)
            if ((GPIO_PORTB_DATA_R & IGNITION_SWITCH) == 0) {
                if (doorLocked == 1) {
                    toggleDoorLock();  // Unlock the doors
                }
            }
        }
    }
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


// Initialize I2C1
void I2C1_Init(void) {
    SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R1;    // Enable clock for I2C1
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R2;  // Enable clock for Port C (SCL, SDA on PC4, PC5)

    while ((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R2) == 0); // Wait for Port C to be ready

    // Configure SCL (PC4) and SDA (PC5) for I2C
    GPIO_PORTC_AFSEL_R |= SCL_PIN | SDA_PIN;   // Enable alternate function for SCL and SDA
    GPIO_PORTC_DEN_R |= SCL_PIN | SDA_PIN;     // Enable digital function for SCL and SDA
    GPIO_PORTC_ODR_R |= SDA_PIN;               // Enable open-drain for SDA

    // Configure I2C1 as master
    I2C1_MCR_R = I2C_MCR_MSTR | I2C_MCR_IRS;   // Set I2C1 as master and use system clock
    I2C1_MTPR_R = 0x07;                        // Set I2C1 SCL clock speed (Standard mode ~100kHz)
}

// Write byte to the LCD via I2C
void I2C1_Write(uint8_t address, uint8_t data) {
    I2C1_MSA_R = (address << 1); // Set the address for write (shifted 1 bit)
    I2C1_MDR_R = data;           // Set the data to be sent
    I2C1_MCS_R = I2C_MCS_RUN | I2C_MCS_START | I2C_MCS_STOP;  // Start transmission

    // Wait until the transmission is completed
    while (I2C1_MCS_R & I2C_MCS_BUSY);
}

// LCD Initialization (I2C version)
void LCD_Init(void) {
    // Initialize the I2C interface
    I2C1_Init();

    // Send initialization commands to LCD (using I2C)
    I2C1_Write(LCD_ADDR, 0x38);  // Function set: 8-bit mode, 2 lines
    I2C1_Write(LCD_ADDR, 0x0C);  // Display ON, Cursor OFF, Blink OFF
    I2C1_Write(LCD_ADDR, 0x06);  // Entry Mode Set: Increment cursor, no shift
    I2C1_Write(LCD_ADDR, 0x01);  // Clear display
    I2C1_Write(LCD_ADDR, 0x02);  // Return home
}

// Display a character on the LCD
void LCD_DisplayChar(char character) {
    I2C1_Write(LCD_ADDR, character);  // Write character to LCD
}

// Display a string on the LCD
void LCD_DisplayString(char* str) {
    while (*str) {
        LCD_DisplayChar(*str++); // Display each character from the string
    }
}

/*********************************************************************************************************/ 


int main(void) {
	
  // Initialize LEDs (PF1, PF2 for door lock/unlock status)
  initLEDs();
	initButtons();
  initIgnitionSwitch();
	initUltrasonicLEDs();
	initGearSwitch();
  initPotentiometer(); 
	xBinarySemaphore = xSemaphoreCreateBinary();

	GPIO_PORTF_DATA_R |= RED_LED;
  // Create FreeRTOS tasks
  xTaskCreate(ButtonControlTask, "Button Control Task", 128, NULL, 2, NULL);
	xTaskCreate(IgnitionMonitorTask, "Ignition Monitor", 128, NULL, 1, NULL);
	xTaskCreate(SpeedCheckTask, "Speed Check Task", 128, NULL, 3, NULL);
	xTaskCreate(UltrasonicTask, "Ultrasonic Task", 128, NULL, 1, NULL);
	xTaskCreate(GearSwitchTask, "Gear Switch Task", 128, NULL, 1, NULL);
	

    // Start the FreeRTOS scheduler
   vTaskStartScheduler();
		
		return 0; //This part will not reach 
}
