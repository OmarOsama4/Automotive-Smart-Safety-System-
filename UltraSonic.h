#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <stdint.h>
#include "tm4c123gh6pm.h" // Include the header file for your TM4C123 microcontroller
#include <FreeRTOS.h>

#define RED_LED 0x02
#define BLUE_LED 0x04
#define GREEN_LED 0x08

// Function prototypes
void initUltrasonicLEDs(void);
void Delay_MicroSecond(int time);
float Measure_distance(void);
void Timer0ACapture_init(void);

#endif // ULTRASONIC_H
