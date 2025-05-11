#include "ManualButtons.h"

// Variable to store the door lock state (1 for locked, 0 for unlocked)
uint8_t doorLocked = 0;

void initButtons(void)
{
    // Enable Port F (GPIO Port F)
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5; // Enable GPIO Port F clock
    while ((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R5) == 0)
    {
    } // Wait for Port F to be ready

    GPIO_PORTF_LOCK_R = 0x4C4F434B; // Unlock the PF0 pin
    GPIO_PORTF_CR_R |= LOCK_BUTTON; // Commit PF0 pin

    // Set PF0 and PF4 as input (Buttons)
    GPIO_PORTF_DIR_R &= ~(LOCK_BUTTON | UNLOCK_BUTTON); // Set PF0 and PF4 as input
    GPIO_PORTF_DEN_R |= LOCK_BUTTON | UNLOCK_BUTTON;    // Enable digital functionality for PF0 and PF4
    GPIO_PORTF_PUR_R |= LOCK_BUTTON | UNLOCK_BUTTON;    // Enable pull-up resistors for buttons
}

void toggleDoorLock(void)
{
    if (doorLocked)
    {
        // Door is unlocked
        doorLocked = 0;
        clearRow(&dis, 0);
        displayTextOnLCD(&dis, "Door UnLocked", 0, 0); 
    }
    else
    {
        // Door is locked
        doorLocked = 1;
        clearRow(&dis, 0);
        displayTextOnLCD(&dis, "Door Locked", 0, 0); 
    }
}

void showdoorstatus(void)
{
    if (doorLocked)
    {
        // Door is locked
        clearRow(&dis, 0);
        displayTextOnLCD(&dis, "Door Locked", 0, 0); 
    }
    else
    {
        // Door is unlocked
        clearRow(&dis, 0);
        displayTextOnLCD(&dis, "Door UnLocked", 0, 0); 
    }
}
