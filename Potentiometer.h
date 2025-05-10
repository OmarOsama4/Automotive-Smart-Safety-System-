#ifndef POTENTIOMETER_H
#define POTENTIOMETER_H

#include <stdint.h>
#include "tm4c123gh6pm.h" // Include the header file for your TM4C123 microcontroller

// Define the potentiometer ADC channel (AIN1 corresponds to PE2)
#define POTENTIOMETER_CHANNEL 1 // AIN1 corresponds to channel 1 on ADC (PE2)
#define ADC_SEQ0 0x01

// Function prototypes
void initPotentiometer(void);
uint16_t ADC_Read(void);
float readPotentiometer(void);

extern float speed; // Variable to hold the speed value

#endif // POTENTIOMETER_H
