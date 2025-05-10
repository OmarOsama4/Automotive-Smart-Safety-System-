#ifndef BUTTONS_H
#define BUTTONS_H

#include <stdint.h>
#include "tm4c123gh6pm.h"  // Include the header file for your TM4C123 microcontroller
#include "LCD.h"           // Include the LCD header file
#define LOCK_BUTTON 0x01   // PF0 (Lock button)
#define UNLOCK_BUTTON 0x10 // PF4 (Unlock button)

// Define the door lock state
extern uint8_t doorLocked; // 1 for locked, 0 for unlocked

// Function prototypes
void initButtons(void);
void toggleDoorLock(void);
void showdoorstatus(void);

#endif // BUTTONS_H