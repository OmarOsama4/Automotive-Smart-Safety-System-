#ifndef BUZZER_H
#define BUZZER_H

#include <stdint.h>
#include "tm4c123gh6pm.h"  // Include the header file for your TM4C123 microcontroller

// Define the buzzer pin (e.g., PB0)
extern uint8_t BUZZER_PIN; 

// Function prototype for initializing the buzzer
void initBuzzer(void);

#endif // BUZZER_H
