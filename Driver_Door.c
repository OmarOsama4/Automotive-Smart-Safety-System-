#include "Driver_Door.h"

// Function to initialize the driver's door input (PD0)
void initDriverDoor(void)
{
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R3; // Enable clock for Port D
    while ((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R3) == 0)
    {
    } // Wait until Port D is ready

    // Configure PD0 as input (assuming the door state is read from this pin)
    GPIO_PORTD_DIR_R &= ~0x01; // Set PD0 as input (0 = input, 1 = output)
    GPIO_PORTD_DEN_R |= 0x01;  // Enable digital function on PD0

    // Enable pull-down resistor on PD0 (if the door state uses an active-high switch)
    GPIO_PORTD_PDR_R |= 0x01; // Enable pull-down resistor on PD0
}

// Function to read the driver's door state (open or closed)
int isDoorOpen(void)
{
    // Check if PD0 is high (active-high switch for door open with pull-down resistor)
    return (GPIO_PORTD_DATA_R & 0x01) == 0x01; // Returns 1 if door is open, 0 if closed
}

// Buzzer control function
void controlBuzzer(int state)
{
    if (state)
    {
        GPIO_PORTB_DATA_R |= BUZZER_PIN; // Turn on buzzer (PB0 high)
    }
    else
    {
        GPIO_PORTB_DATA_R &= ~BUZZER_PIN; // Turn off buzzer (PB0 low)
    }
}
