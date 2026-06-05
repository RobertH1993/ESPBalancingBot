# ESP32 BalancingBot
A prototype project of an balancing robot created with the Waveshare generic robot controller.


https://github.com/user-attachments/assets/8b719d6f-a049-4152-96c9-c822f076874d


## Features
- Cascading PID loops to control balance and position
- UDP Telemetry and PID tuning
- Motor control through MCPWM and encoder counting through PCNTL

## Getting Started

Compile the project and flash it to the hardware, after connecting to Wi-Fi hold the robot upright and it should start balancing on its own.

## Fixed Problems
- The standard waveshare driver for the QMI8658 has an bug thas has been fixed in this repository. The qmi8658_gyro_range_t was wrongly defined in their driver (https://components.espressif.com/components/waveshare/qmi8658/versions/1.0.1/readme)

## TODO
- The desktop app and the protocol for telemetry, control and tuning should be extended. 
- Control through a BLE remote control should be added.
- Distance sensors should be added to prevent driving against walls or other objects.

