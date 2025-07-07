# Smart Card Access Control System

This repository contains the firmware for a Smart Card Access Control System, developed as "Projeto 2" for the UFSC - EEL7030 Microprocessadores course.

## 🚀 Check out the Simulator!

You can explore and interact with a live simulation of this project directly in your browser: [Smart Card Access Control System Simulator](https://wokwi.com/projects/435774319763008513)

## Project Overview

The core of this system is an **ESP32-C3 microcontroller** which acts as the central processing unit for the door lock. It interacts with a **SmartCard EEPROM** via an I2C protocol and communicates with a central server over Wi-Fi using HTTP POST requests.

When a guest performs a check-in, a **16-character random ID** is generated and associated with the room number. This ID is then registered in the hotel's database and written to the first two pages (first 16 bytes) of a SmartCard EEPROM.

### Key Features:

* **Smart Card Interaction:** Reads the 16-character ID from the first two pages of the SmartCard EEPROM (base addresses `0x00` and `0x08`) upon card insertion, which is detected by a limit switch button ($\mathbf{B_{FC}}$).
* **Wi-Fi Connectivity:** Connects to the hotel's Wi-Fi network.
* **Server Communication:** Sends the SmartCard ID and room number to a central server via HTTP POST requests for validation.
* **Access Decision:** Based on the server's positive or negative response, the ESP32-C3 decides whether to grant or deny access.
* **Door Lock Simulation:** A **red LED**, connected to GPIO 9, simulates the door lock release system, remaining lit for 2 seconds to open the lock.
* **Timestamping:** Records a 64-bit (8 bytes) timestamp of each access attempt (regardless of outcome) onto page 4 (base address `0x18`) of the SmartCard.
* **User Feedback (LCD Display):** An **LCD display** provides real-time status updates via I2C, such as "Conectando ao Wi-Fi" (Connecting to Wi-Fi), "Aguardando Cartão" (Waiting for Card), "Verificando Cartão" (Verifying Card), "Acesso Liberado" (Access Granted), or "Acesso Negado" (Access Denied).
* **System Status (Green LED):** A **green LED**, connected to GPIO 0, provides visual feedback on the system's state through different blinking patterns: slow blinking when disconnected, solid when connected, single flash on read start, double flash on access granted, and six flashes on access denied.

## Hardware Setup

The project utilizes the **ESP32-C3 microcontroller**, an EEPROM SmartCard, a SmartCard reader, an LCD display, a red LED, a green LED, and a limit switch button ($\mathbf{B_{FC}}$) to detect card insertion. A circuit diagram illustrating the proposed hardware for testing is included in the project documentation.


### Connections Diagram

Here's the Fritzing diagram showing the connections for the SmartCard and LCD with the ESP32-C3:

![SmartCard and LCD with ESP32-C3 Connections](https://github.com/rebeccaquintino/acesss-control/blob/main/doc/connections.jpeg)


## Development

The firmware is written in **C** and built upon the **Espressif IoT Development Framework (ESP-IDF)**. It leverages the **FreeRTOS** operating system for task management.