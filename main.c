#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "tm4c123gh6pm.h"
#include <FreeRTOS.h>
#include <task.h>
#include "queue.h"
#include "semphr.h"
#include "LCD.h"
#include "Buzzer.h"
#include "GearSwitch.h"
#include "ManualButtons.h"
#include "Potentiometer.h"
#include "UltraSonic.h"
#include "IgnitionSwitch.h"
#include "Driver_Door.h"

/***********************************************************************************************************/
// The potentiometer     -> PE2
// The Buzzar 			     -> PB0
// Echo                  -> PB6
// Trigger				 			 -> PA4
// GearShift             -> PB7
// Red Led               -> PD2
// Blue Led              -> PD1
// Green Led             -> PD0
// Ignition Switch       -> PB4
// SDL                   -> PB3
// SCL                   -> PB2
/***********************************************************************************************************/
#define IGNITION_SWITCH 0x10
#define SPEED_THRESHOLD 40
#define MAX_DISTANCE 100 
/***********************************************************************************************************/
float time;
float distance;
uint32_t gearSwitchState;
SemaphoreHandle_t xMutex;
char diststr[20];
char speedStr[20];
bool globalSpeed = 0;
bool globalLock = 0;
int doorOpen; 
float speedAR;
bool driverdoor =0;
/***********************************************************************************************************/
void GearSwitchTask(void *pvParameters);
void ButtonControlTask(void *pvParameters);
void SpeedCheckTask(void *pvParameters);
void controlBuzzerSN(uint32_t distance);
void controlLED(uint32_t distance);
void UltrasonicTask(void *pvParameters);
void IgnitionMonitorTask(void *pvParameters);
void VehicleAlertTask(void *pvParameters);
/***********************************************************************************************************/

int main(void)
{
    LCDI2CInit(&dis, LCD_ADDR, LCD_COLS, LCD_ROWS);
    initBuzzer();
    initButtons();
    initIgnitionSwitch();
    initUltrasonicLEDs();
    initGearSwitch();
    initPotentiometer();
    initDriverDoor();
    xMutex = xSemaphoreCreateMutex();

    // Create FreeRTOS tasks
    xTaskCreate(ButtonControlTask, "Button Control Task", 128, NULL, 1, NULL);
    xTaskCreate(IgnitionMonitorTask, "Ignition Monitor", 128, NULL, 1, NULL);
    xTaskCreate(SpeedCheckTask, "Speed Check Task", 128, NULL, 2, NULL);
    xTaskCreate(UltrasonicTask, "Ultrasonic Task", 128, NULL, 1, NULL);
    xTaskCreate(GearSwitchTask, "Gear Switch Task", 128, NULL, 1, NULL);
    xTaskCreate(VehicleAlertTask, "Driver Door Task", 128, NULL, 1, NULL);

    // Start the FreeRTOS scheduler
    vTaskStartScheduler();

    return 0; // This part will not reach
}

/***********************************************************************************************************/
// FreeRTOS Task to monitor and handle gear switching
void GearSwitchTask(void *pvParameters)
{
    while (1)
    {
        gearSwitchState = GPIO_PORTB_DATA_R & GEAR_SWITCH_PIN;
        // Check if the Gear Switch button is pressed (active high)
        if ((GPIO_PORTB_DATA_R & GEAR_SWITCH_PIN) == 0)
        { // Button pressed (PB7 is high)
            gearreverseoff();
        }
        else
        {
            gearreverseon();
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // Short delay before checking again
    }
}
/********************************************************************************************************/
// FreeRTOS Task to check button presses for manual door lock/unlock
void ButtonControlTask(void *pvParameters)
{
    while (1)
    {
        if (xSemaphoreTake(xMutex, portMAX_DELAY))
        {
            if ((GPIO_PORTF_DATA_R & LOCK_BUTTON) == 0)
            {
                if (doorLocked == 0)
                {
                    toggleDoorLock(); // Lock the door
                }
                vTaskDelay(pdMS_TO_TICKS(200));
            }

            if ((GPIO_PORTF_DATA_R & UNLOCK_BUTTON) == 0)
            {
                if (doorLocked == 1)
                {
                    toggleDoorLock(); // Unlock the door
                }
                vTaskDelay(pdMS_TO_TICKS(200));
            }
            xSemaphoreGive(xMutex);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
/*********************************************************************************************************/
// FreeRTOS Task to check speed and automatically lock doors based on speed
void SpeedCheckTask(void *pvParameters)
{
    while (1)
    {
        if (gearReverse == 0)
        {
            // Read speed from potentiometer (ADC value mapped to 0 - 100 km/h)
            float potentiometerValue = readPotentiometer();
            if (potentiometerValue == -1)
            {
                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }
            if (potentiometerValue >= SPEED_THRESHOLD && doorLocked == 0 && globalSpeed == 0)
            {
                toggleDoorLock();
                globalSpeed = 1;
            }

            if (potentiometerValue < SPEED_THRESHOLD)
            {
                globalSpeed = 0;
            }

            if (xSemaphoreTake(xMutex, portMAX_DELAY))
            {
                snprintf(speedStr, sizeof(speedStr), "Speed = %.1f km/h", potentiometerValue);
                clearRow(&dis, 1);
                displayTextOnLCD(&dis, speedStr, 1, 0);
                xSemaphoreGive(xMutex);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
/********************************************************************************************************/
// Function to control the RGB LED based on proximity
void controlLED(uint32_t distance)
{
    // Turn off all LEDs first
    GPIO_PORTF_DATA_R &= ~(RED_LED | BLUE_LED | GREEN_LED);

    // Turn on LED based on distance
    if (distance > 100)
    { // Safe zone (Green LED)
        GPIO_PORTF_DATA_R |= GREEN_LED;
    }
    else if (distance > 30)
    { // Caution zone (Yellow LED)
        GPIO_PORTF_DATA_R |= BLUE_LED;
    }
    else
    { // Danger zone (Red LED)
        GPIO_PORTF_DATA_R |= RED_LED;
    }
}

// Function to control the buzzer based on proximity
void controlBuzzerSN(uint32_t distance)
{
    // Control buzzer: beep faster as the distance decreases
    if (distance > 100)
    {                                     // Safe zone
        GPIO_PORTB_DATA_R |= BUZZER_PIN;  // Turn on buzzer
        vTaskDelay(pdMS_TO_TICKS(100));   // Buzzer beep for a short time
        GPIO_PORTB_DATA_R &= ~BUZZER_PIN; // Turn off buzzer
        vTaskDelay(pdMS_TO_TICKS(100));   // Delay for faster beep
    }
    else if (distance > 30)
    {                                     // Caution zone
        GPIO_PORTB_DATA_R |= BUZZER_PIN;  // Turn on buzzer
        vTaskDelay(pdMS_TO_TICKS(30));    // Buzzer beep for a short time
        GPIO_PORTB_DATA_R &= ~BUZZER_PIN; // Turn off buzzer
        vTaskDelay(pdMS_TO_TICKS(30));    // Delay for faster beep
    }
    else
    {                                    // Danger zone (continuous buzzer)
        GPIO_PORTB_DATA_R |= BUZZER_PIN; // Keep buzzer on
    }
}

// FreeRTOS Task to handle ultrasonic sensor and display proximity
void UltrasonicTask(void *pvParameters)
{
    Timer0ACapture_init(); /*initialize Timer0A in edge edge time */
    while (1)
    {
        if (gearReverse == 1)
        {
            time = Measure_distance();            /* take pulse duration measurement */
            distance = (time * 10625) / 10000000; /* convert pulse duration into distance */
            controlLED(distance);
            controlBuzzerSN(distance);
            clearRow(&dis, 1);
            snprintf(diststr, sizeof(diststr), "Dist = %.1f cm", distance);
            if (xSemaphoreTake(xMutex, portMAX_DELAY))
            {
                displayTextOnLCD(&dis, diststr, 1, 0);
                xSemaphoreGive(xMutex);
            }
        }
        else
        {
            GPIO_PORTF_DATA_R &= ~(RED_LED | BLUE_LED | GREEN_LED | BUZZER_PIN);
        }
        vTaskDelay(pdMS_TO_TICKS(500)); // Delay for 500ms before next measurement
    }
}
/***********************************************************************************************************/
void IgnitionMonitorTask(void *pvParameters)
{
    while (1)
    {
        if ((GPIO_PORTB_DATA_R & IGNITION_SWITCH) == 0 && doorLocked == 1 && globalLock == 0)
        {
            if (xSemaphoreTake(xMutex, portMAX_DELAY))
            {
                toggleDoorLock(); // Unlock the doors when ignition is OFF
                globalLock = 1;   // Set the global lock to prevent further toggles
                xSemaphoreGive(xMutex);
            }
        }

        // When the ignition switch turns on, reset the global lock (but don't toggle)
        if ((GPIO_PORTB_DATA_R & IGNITION_SWITCH) == 0x10)
        {
            globalLock = 0;
        }

        vTaskDelay(500); // Add a small delay to avoid unnecessary fast polling
    }
}

/*********************************************************************************************************/
// Function to handle the alert when the door is open and vehicle is moving
void VehicleAlertTask(void *pvParameters)
{
    while (1)
    {
				if (gearReverse == 0){
						speedAR = readPotentiometer();
						doorOpen = isDoorOpen(); 
						// If vehicle is moving and the driver's door is open, trigger buzzer and display message
						if (speedAR > 10 && doorOpen && driverdoor ==0)
						{
								driverdoor =1;
								controlBuzzer(1); // Turn on buzzer
								if (xSemaphoreTake(xMutex, portMAX_DELAY))
								{
										clearRow(&dis, 0);
										displayTextOnLCD(&dis, "Driver Door Open!", 0, 0);
										xSemaphoreGive(xMutex);
								}
						}
						else if (!doorOpen && driverdoor ==1)
						{		
								driverdoor =0;
								showdoorstatus();
								controlBuzzer(0); // Turn off buzzer
						}								
				}
        vTaskDelay(pdMS_TO_TICKS(500)); // Delay for 500ms before checking again
    }
}