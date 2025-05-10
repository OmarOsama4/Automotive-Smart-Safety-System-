#include "buzzer.h"

uint8_t BUZZER_PIN = 0x01;

void initBuzzer(void)
{
    // Enable clock for Port B
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1; // Enable clock for Port B
    while ((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R1) == 0)
    {
    } // Wait until Port B is ready

    // Configure PB0 as output for the buzzer
    GPIO_PORTB_DIR_R |= BUZZER_PIN; // Set PB0 as an output pin
    GPIO_PORTB_DEN_R |= BUZZER_PIN; // Enable digital function for PB0
}
