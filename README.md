# OpenGauge
OpenGauge is an ESP32-based OBD-II CAN bus interface for reading real-time vehicle telemetry such as speed, engine RPM, and coolant temperature using an MCP2551 CAN transceiver.

## Components and Materials
- ESP32 Microcontroller
- 12V to 5V Buck Converter
- MCP2551 CAN Transceiver
- 5V 3.3V Bidirectional Logic Shifter
- 16x2 I2C LCD Display
- OBD-II Harness
- Push Button
- Breadboard
- Jumperwires
- Multimeter

## Circuit
### OBD-II Harness
The OBD-II harness will plug into the vehicle, and provide a means for interfacing with the vehicle ECU. The pins of interest are CAN High, CAN Low, Chasis ground, Signal ground, and +12V battery power. Double check the harness connetions with the corresponding pin numbers using a miltimeter.

### ESP32 and Components
This project uses an ESP32 development board as the primary microcontroller, connected to three 3 components:

CAN Bus Interface: The ESP32's built-in TWAI (Two-Wire Automotive Interface) is used for OBD-II communication. GPIO 4 (RX) and GPIO 5 (TX) connect to a MCP2551 CAN transceiver module, which interfaces with the vehicle's OBD-II diagnostic port. A 3.3V-5V logic shifter is placed between the transceiver and EPS32, to step up/down the voltage logic accordinly.

Display: A 16×2 LCD display (I2C address 0x27) connects via the ESP32's I2C bus to display the currently selected sensor reading and its value.

User Input: A momentary push button connected to GPIO 23, and ground (with internal pull-up enabled) allows the user to cycle through available sensor modes.

## Code
This project is implemented in Arduino C++ with PlatformIO for an ESP32-based OBD-II gauge. At a high level, main.cpp initializes the CAN interface and LCD display, then runs a simple state polling loop (IDLE and WAITING states) to request and display live vehicle data.

Using the [ESP32-TWAI-CAN library](https://github.com/handmade0octopus/ESP32-TWAI-CAN), the firmware periodically sends OBD-II Mode 01 requests for the currently selected PID. A push button cycles through these display modes, and the LCD is updated with the latest valid reading. Currently, the requested PIDs allow display modes of engine load, engine RPM, vehicle speed, coolant temperature, intake temperature, oil temperature, and fuel pressure.

The runtime logic is designed to be bus-friendly and resilient. Requests are limited to one every 1.2 seconds to avoid ECU-lockout, responses are validated by CAN ID/mode/PID before use, and timeout handling returns the system to request mode without blocking. If repeated transmit failures occur, CAN is marked offline and the code periodically attempts automatic bus recovery.

## PCB
A custom PCB has been designed to provide a more integrated solution. The board features soldered pads for the ESP32 microcontroller, MCP2551 CAN transceiver, logic level shifter, and user button.

Screw terminals provide connectors for the OBD-II diagnostic harness and I2C LCD display, and integrated mounting holes allow the buck converter and LCD to be secured directly to the board for a clean, compact layout. This design enables quick assembly for a more permanent design of this project.
