# Automotive Smart Safety System

## Project Overview
This project implements an **Automotive Smart Safety System** using the **TM4C123GH6PM** microcontroller and **FreeRTOS**. It integrates two key safety features typically found in modern vehicles: **Intelligent Car Door Locking** and **Rear Parking Assistance**.

### Key Features:
1. **Intelligent Car Door Locking**
   - Automatically locks the doors when the vehicle speed exceeds a predefined threshold (e.g., 10 km/h).
   - Automatically unlocks the doors when the ignition is turned off, simulating the typical behavior in a vehicle.

2. **Rear Parking Assistance**
   - Uses an **HC-SR04 Ultrasonic Sensor** to detect obstacles behind the vehicle when in reverse.
   - Provides real-time distance feedback via an **LCD display**.
   - **RGB LED** provides visual alerts based on proximity (Green for safe, Yellow for caution, Red for danger).
   - **Buzzer** gives audible feedback that changes in frequency based on obstacle proximity.

3. **FreeRTOS Task Management**
   - Multiple tasks for real-time system control, including speed detection, door locking, obstacle detection, and system status updates.
   - Semaphore-based synchronization to manage access to shared resources like the **LCD** and **buzzer**.
   - Message queues for task communication and data passing.

## Hardware Components:
- **Microcontroller**: TM4C123GH6PM (Tiva C Series)
- **Sensors**:
  - Potentiometer for speed detection
  - HC-SR04 Ultrasonic Sensor for rear parking assistance
- **User Interface**:
  - Push buttons for manual door lock/unlock
  - Switches to simulate gears (Park, Drive, Reverse)
  - Ignition switch
  - LCD display for system status
  - RGB LED for proximity alerts
  - Buzzer for audible alerts

## Software:
- **FreeRTOS**: Used to manage concurrent tasks, including speed detection, door control, obstacle detection, and system status updates.
- **Keil IDE**: The project is developed using Keil for programming and debugging the TM4C123GH6PM microcontroller.
- **TivaWare**: The software libraries for TM4C123GH6PM, provided by Texas Instruments, are used for hardware abstraction and peripheral configuration.

## Setup Instructions:
1. **Install Keil IDE**: Download and install Keil from [Keil website](https://www.keil.com/download/).
2. **Install TivaWare**: Download TivaWare from [TI's website](https://www.ti.com/tool/SW-TM4C) for the required libraries and drivers.
3. **Create a New Project**:
   - Open Keil and create a new project for **TM4C123GH6PM**.
   - Add the source files from this repository.
   - Include necessary TivaWare libraries.
4. **Build and Program**:
   - Build the project using Keil’s build tools.
   - Program the microcontroller via the appropriate debugging tool (e.g., **JTAG** or **SWD**).

## How to Contribute:
- **Fork the repository** to make changes to your own copy.
- **Clone** the repository locally to begin working on the project.
- **Create a new branch** for any changes or new features.
- Submit a **pull request** for any changes you wish to contribute.

