#ifndef DRIVER_DOOR_H
#define DRIVER_DOOR_H

#include <stdint.h>
#include "tm4c123gh6pm.h" // Include the header file for your TM4C123 microcontroller
#include "Buzzer.h"
#include "LCD.h"

// Function prototypes
void initDriverDoor(void);
int isDoorOpen(void);
void controlBuzzer(int state);
void showDoorOpen(void);
void showDoorClose(void);

#endif // DRIVER_DOOR_H
