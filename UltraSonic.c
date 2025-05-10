#include "UltraSonic.h"

// Function to initialize the LEDs (Red, Blue, Green) on Port F
void initUltrasonicLEDs(void)
{
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5; // Enable clock for Port F
    while ((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R5) == 0)
    {
    } // Wait until Port F is ready

    GPIO_PORTF_DIR_R |= (RED_LED | BLUE_LED | GREEN_LED); // Set as output for LEDs
    GPIO_PORTF_DEN_R |= (RED_LED | BLUE_LED | GREEN_LED); // Enable digital function for LEDs
}

// Delay function that creates a microsecond delay using Timer 1
void Delay_MicroSecond(int time)
{
    int i;
    SYSCTL->RCGCTIMER |= 2; // Enable clock to Timer Block 1
    TIMER1->CTL = 0;        // Disable Timer before initialization
    TIMER1->CFG = 0x04;     // 16-bit option
    TIMER1->TAMR = 0x02;    // Periodic mode and down-counter
    TIMER1->TAILR = 16 - 1; // TimerA interval load value register
    TIMER1->ICR = 0x1;      // Clear the TimerA timeout flag
    TIMER1->CTL |= 0x01;    // Enable Timer A after initialization

    for (i = 0; i < time; i++)
    {
        while ((TIMER1->RIS & 0x1) == 0)
            ;              // Wait for TimerA timeout flag
        TIMER1->ICR = 0x1; // Clear the TimerA timeout flag
    }
}

// Function to measure the distance using the ultrasonic sensor
float Measure_distance(void)
{
    int lastEdge, thisEdge;

    // Given 10us trigger pulse
    GPIOA->DATA &= ~(1 << 4); // Make trigger pin low
    Delay_MicroSecond(10);    // 10 microseconds delay
    GPIOA->DATA |= (1 << 4);  // Make trigger pin high
    Delay_MicroSecond(10);    // 10 microseconds delay
    GPIOA->DATA &= ~(1 << 4); // Make trigger pin low

    while (1)
    {
        TIMER0->ICR = 4; // Clear timer0A capture flag
        while ((TIMER0->RIS & 4) == 0)
            ;                       // Wait till capture flag is set
        if (GPIOB->DATA & (1 << 6)) // Check if rising edge occurs
        {
            lastEdge = TIMER0->TAR; // Save the timestamp for rising edge
            TIMER0->ICR = 4;        // Clear timer0A capture flag
            while ((TIMER0->RIS & 4) == 0)
                ;                         // Wait till capture flag is set
            thisEdge = TIMER0->TAR;       // Save the timestamp for falling edge
            return (thisEdge - lastEdge); // Return the time difference
        }
    }
}

// Function to initialize Timer0 for capturing the ultrasonic sensor signal
void Timer0ACapture_init(void)
{
    SYSCTL->RCGCTIMER |= 1; // Enable clock to Timer Block 0
    SYSCTL->RCGCGPIO |= 2;  // Enable clock to PORTB

    GPIOB->DIR &= ~0x40;        // Make PB6 an input pin
    GPIOB->DEN |= 0x40;         // Make PB6 as a digital pin
    GPIOB->AFSEL |= 0x40;       // Use PB6 alternate function
    GPIOB->PCTL &= ~0x0F000000; // Configure PB6 for T0CCP0
    GPIOB->PCTL |= 0x07000000;

    // PB2 as a digital output signal to provide trigger signal
    SYSCTL->RCGCGPIO |= 1;  // Enable clock to PORTA
    GPIOA->DIR |= (1 << 4); // Set PB2 as a digital output pin
    GPIOA->DEN |= (1 << 4); // Make PB2 as a digital pin

    TIMER0->CTL &= ~1;       // Disable timer0A during setup
    TIMER0->CFG = 4;         // 16-bit timer mode
    TIMER0->TAMR = 0x17;     // Up-count, edge-time, capture mode
    TIMER0->CTL |= 0x0C;     // Capture the rising edge
    TIMER0->CTL |= (1 << 0); // Enable timer0A
}