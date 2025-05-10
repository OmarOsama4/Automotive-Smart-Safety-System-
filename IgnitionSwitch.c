#include "IgnitionSwitch.h"

// Function to initialize the ignition switch (PB4)
void initIgnitionSwitch(void)
{
    SYSCTL->RCGCGPIO |= (1 << 1); // Enable clock to Port B
    while ((SYSCTL->PRGPIO & (1 << 1)) == 0)
        ;                    // Wait until Port B is ready
    GPIOB->DIR &= ~(1 << 4); // Set PB4 as input (Ignition switch)
    GPIOB->DEN |= (1 << 4);  // Enable digital function for PB4
    GPIOB->PDR |= (1 << 4);  // Enable pull-down resistor on PB4 (reads LOW when not pressed)
}
