#include "potentiometer.h"

float speed; // Initialize the speed variable

// Function to initialize the ADC for potentiometer reading
void initPotentiometer(void)
{
    // Enable the clock for ADC0 and Port E
    SYSCTL_RCGCADC_R |= SYSCTL_RCGCADC_R0;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R4;

    // Wait for peripherals to be ready
    while ((SYSCTL_PRADC_R & SYSCTL_RCGCADC_R0) == 0)
        ;
    while ((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R4) == 0)
        ;

    // Configure PE2 (AIN1) for analog input
    GPIO_PORTE_AFSEL_R |= 0x04; // Enable alternate function on PE2
    GPIO_PORTE_DEN_R &= ~0x04;  // Disable digital function on PE2
    GPIO_PORTE_AMSEL_R |= 0x04; // Enable analog mode on PE2

    // Configure ADC0
    ADC0_ACTSS_R &= ~ADC_SEQ0;             // Disable sample sequencer 0
    ADC0_EMUX_R = 0x0;                     // Set sequencer to software trigger
    ADC0_SSMUX0_R = POTENTIOMETER_CHANNEL; // Select AIN1 (channel 1)
    ADC0_SSCTL0_R = 0x06;                  // Configure: single sample, end of sequence
    ADC0_ACTSS_R |= ADC_SEQ0;              // Enable sample sequencer 0
}

// Function to read the potentiometer value (returns value between 0 and 4095)
uint16_t ADC_Read(void)
{
    // Trigger the conversion
    ADC0_PSSI_R = 0x01; // Start the conversion on sequencer 0
    while ((ADC0_RIS_R & 0x01) == 0)
        ;                                     // Wait until conversion is complete
    uint16_t result = ADC0_SSFIFO0_R & 0xFFF; // Read the result
    ADC0_ISC_R = 0x01;                        // Clear the interrupt flag
    return result;
}

// Function to read the temperature from LM35 (using potentiometer value)
float readPotentiometer(void)
{
    uint16_t adcValue = ADC_Read(); // Read ADC value
    if (adcValue > 4095)
        return -1;                             // Error handling for invalid ADC value
    float voltage = (adcValue / 4095.0) * 3.3; // Convert ADC value to voltage
    speed = voltage * 100.0;                   // Convert voltage to temperature in °C
    return speed;
}
