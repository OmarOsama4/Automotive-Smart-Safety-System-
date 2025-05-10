#ifndef GEAR_SWITCH_H
#define GEAR_SWITCH_H

#include <stdint.h>
#include "tm4c123gh6pm.h" // Include the header file for your TM4C123 microcontroller

#define GEAR_SWITCH_PIN 0x80

// Function prototypes
void initGearSwitch(void);
void gearreverseon(void);
void gearreverseoff(void);

extern uint8_t gearReverse; // Variable to hold the gear state

#endif // GEAR_SWITCH_H
