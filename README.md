# stm32f407xx_peripheral_drivers

Project implementing bare metal drivers for peripheral control for STM32407xx family. Written entirely in C with direct register manipulation and without the use of a HAL. Offers several APIs to allow user defined programs to utilize the drivers inside the application layer for their own programs.

**Drivers Implemented:**

LCD1602A - Drivers for initializing and controlling a LCD1602A display. Allows the user to print characters to the screen, control where the cursor is, blinking cursors, auto increment the cursor pointer, etc.

DS1307 - Drivers for initializing and controlling a DS1307 RTC module. Allows the user to set and retrieve time and date information using I2C communication protocol. (Note: This uses my custom I2C drivers so the typical STM32 HAL drivers won't work for this and would require changing a few calls)

MPU6500 - Drivers for initializing and retrieving data from a MPU6500 accelerometer/gyroscope module. Allows the user to use the MPU to retrieve the rotation / acceleration details of the MPU.

# API Description & Usage
Each driver typically provides a configuration/handle struct typedef that the user will configure and then pass to the approrpriate init() function. Beyond this each driver provides unique APIs that should be described in the header file for each module.

# Installation

Standard compilation routine using ARM compiler for STM32F407xx family of devices is sufficient for compiling the drivers.

# Implementation
Implemented in C. Utilized the datasheets for both the STM32F4xx family of MCUs as well the datasheets for each module to know how to interface with the modules and what the code should do.


# Background
This repo is mainly for keeping track of the drivers that I write for modules in my other projects.

# Known Issues
The MPU6500 in interrupt mode can send so many interrupts that the MCU will get stuck in the IRQ handler indefinitely.


# ***TODO***
