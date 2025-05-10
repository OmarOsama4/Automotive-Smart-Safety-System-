#include "GearSwitch.h"

// Variable to store the gear state (1 for gear reverse on, 0 for off)
uint8_t gearReverse = 0;

void initGearSwitch(void)
{
    // Enable clock for Port B
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1; // Enable clock for Port B
    while ((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R1) == 0)
    {
    } // Wait until Port B is ready

    // Configure PB7 as input for the gear switch
    GPIO_PORTB_DIR_R &= ~GEAR_SWITCH_PIN; // Set PB7 as input
    GPIO_PORTB_DEN_R |= GEAR_SWITCH_PIN;  // Enable digital function for PB7
    GPIO_PORTB_PDR_R |= GEAR_SWITCH_PIN;  // Enable pull-down resistor on PB7
}

void gearreverseon(void)
{
    gearReverse = 1; // Set gearReverse to 1 to indicate the gear is in reverse
}

void gearreverseoff(void)
{
    gearReverse = 0; // Set gearReverse to 0 to indicate the gear is not in reverse
}
