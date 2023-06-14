# MQTT Implementation via simples AT Commands 

This repository provides an example implementation of MQTT (Message Queuing Telemetry Transport) protocol for STM32 microcontrollers using the SIM900 module through AT commands. MQTT is a lightweight messaging protocol commonly used in IoT (Internet of Things) applications to enable communication between devices.

> **Note**
>  In this example, a Stm32 microcontroller is used, but it can be replaced with any other microcontroller (Arduino,Pic, NXP)  As is the case with the GSLM/GPRS model SIM900, You can change the model with any other supports AT commands and TCP 


## Features

- **MQTT Client**: This implementation demonstrates how to establish an MQTT client on an STM32 microcontroller using the SIM900 module and AT commands.

## Hardware Requirements

- STM32 microcontroller board (e.g., STM32F103, STM32F407, etc.)
- SIM900 module (or compatible GSM/GPRS module)
- Serial communication interface (UART) between STM32 and SIM900

## Dependencies

STM32Cube HAL (Hardware Abstraction Layer): This project is built using the STM32Cube HAL library, which provides a high-level abstraction layer for STM32 peripherals and middleware components.

## Getting Started

1. Clone this repository to your local machine or download the source code.

2. Open the project in your STM32CubeIDE.

3. Configure the necessary UART parameters (baud rate, parity, stop bits, etc.) to establish communication between the STM32 microcontroller and the SIM900 module.

4. Adjust the MQTT broker settings, such as the broker address, port number (default : 1883) , authentication credentials, and topic subscriptions, as per your requirements.

5. Build the project and flash it onto your STM32 microcontroller.

6. Connect the SIM900 module to the microcontroller and power it up.

7. Monitor the debug output or LED indicators on your microcontroller board to track the MQTT connection status and message exchange.

## Contributing

Contributions to this project are welcome. If you find any issues or have suggestions for improvements, please feel free to submit a pull request or open an issue on the GitHub repository.

## License

This project is licensed under the [Creative Commons Zero v1.0 Universal License](LICENSE). Feel free to use and modify the code according to your needs.

