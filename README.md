# ESP32 Self-Balancing Robot

A two-wheeled robot with a 3D-printed chassis that balances itself, based on ESP-IDF.

https://github.com/user-attachments/assets/8b719d6f-a049-4152-96c9-c822f076874d

Status: Balancing works like it should, remote control is 90% implemented. The dashboard app is still very crude and needs a bit more work. 


## PID Control

The robot is controlled by 3 PID controllers, of which 2 are in a cascading setup. The angle-PID controls the angle of the robot chassis towards gravity. The speed-PID controls the angle-PID such that the robot keeps a certain horizontal velocity. A third, non-cascading PID, the wheel-trim PID, is used to prevent the robot from yawing to the left or the right.

The angle-PID runs at 100 Hz and gets the [Kalman-filtered and fused x-angle](docs/kalman_vs_complementary.md) of the robot as input. It produces a PWM value as output to the motors. The setpoint of this PID is only controlled by the speed-PID.

The speed-PID runs at 10 Hz and gets the velocity, calculated from the wheel encoders, as input. The output is an angle the robot should lean towards to reach a certain velocity. The output of this PID is fed into the setpoint of the angle-PID, making it a cascading setup. The setpoint of this PID (will be) controlled by a remote controller that sets the desired speed of the robot.

The wheel-trim PID controls a small part of the PWM signal that goes to the motors. Its input is the difference in encoder ticks between the left and right motor encoders. If, for example, the left motor has turned further than the right motor, the left motor gets a lower PWM value and the right motor gets a higher PWM value. This way the robot can be steered to a very fine degree, or keep driving straight when the setpoint is zero. The setpoint of this PID (will be) controlled by a remote controller to steer the robot left and right.

At first the speed-PID was implemented as a position-PID based on the encoder ticks of the wheels. This made the robot hold position almost perfectly, but it was not a logical fit for remote control. In the future I might extend the code to switch to precise position holding when the speed of the robot is set to 0. For now a PID based on speed is used.

## Remote Control

The robot can be remotely controlled by a cheap nunchuck-style controller over classic Bluetooth 4.1. The controller is sold as a device to control phones and seems to be used mostly in combination with VR headsets. It supports multiple modes of operation and can output binary left/right/up/down positions, or a value between 0 and 16 for the y-axis and 0 and 16 for the x-axis, depending on how far the user moves the stick.

Telemetry is sent over a UDP socket to my desktop computer, which runs a custom-built Python dashboard or Teleplot. This is of course done through a Wi-Fi connection to my home network. The communication between the Python dashboard and the robot uses protobufs. Using protobufs lets me define the protocol and automatically generate the Python and C files needed to implement it in the firmware and the dashboard.

## Hardware

The chassis of the robot has seen [multiple revisions](docs/chassis_revisions.md) but is now most likely in its final form. The PCB is a Waveshare general robotics controller. This controller has an ESP32 with plenty of ROM and RAM to control the robot. The ESP32 was chosen because that is what the general robotics driver came with, however the ability to stream metrics over WiFi and the build in Bluetooth are great for development purposes. However the ESP32 is probably not optimal for this task and an STM32 variant may be more suited.

The Hall encoders of the motors are read through the PCNT interface on the ESP32, offloading that work from the main CPU cores while not missing any inputs. The PWM to the motors is also offloaded by using MCPWM. A high PWM frequency was chosen such that it falls outside the human hearing range of 20 Hz to 20 kHz. This prevents annoying, human-hearable coil whine from the motors.

A bulky battery with 6S2P Li-ion cells was chosen as the power supply. This gives the robot plenty of power to run for several hours. The robot is charged outside my house, for safety, through a benchtop supply. The battery delivers 12.6 V and is stated to have 20,000 mAh of capacity. That last claim is certainly not true :).

While connecting the battery I got some I2C hardware errors that I was [able to fix](docs/i2c_debug.md) using an oscilloscope.

## Firmware

The robot is programmed using the ESP-IDF framework, which uses FreeRTOS. ESP-IDF spawns a lot of background FreeRTOS tasks under the hood to keep the Wi-Fi connection up and running. Luckily the ESP32 is a dual-core processor, so this has been used to keep the control loop running undisturbed by those background tasks. The control task of the robot runs on application core 1, while all the telemetry and background tasks run on core 0. Practically this is enough to prevent the robot from falling over. I have plotted the delta time between runs of the control task, and this seems to be fully stable after over an hour of running. The ESP-IDF runtime does not guarantee hard real-time, so this might be an issue.

Telemetry is pushed through a queue from core 1 to core 0, using a producer-consumer pattern. A UDP output task runs on core 0, takes the data from the queue, and pushes it out over UDP to my desktop computer.

The robot runs a [state machine](docs/statemachine.md) that defines its behaviour in every state. After calibrating, it switches to the settling state, where the robot waits for the user to keep it upright. If that condition is met, the robot switches to the balancing state, where it keeps itself upright. If the robot tips over too far, it switches back to the settling state.

## Known issue

Automated unit testing should definitely be added. The robustness of the code should be improved, it's a bit of a crude PoC implementation for now. The dashboard app is far from fully functioning and needs more work. The compass and power sensor should be implemented to track heading and battery status. Remote control is not fully functional yet and is especially crude, it should implement reconnects after connection-loss.

TODO:

- Add automated unit tests.
- Add integration on HIL-test.
- Finish remote control over Bluetooth.
- Make the code way more robust and handle more errors.
- Recheck I2C bus-speed.
- Finish the dashboard app.
- Implement the use of the compass sensor.
- Implement the use of the power sensor to track battery usage and maybe even motor stalls.
- Add precision holding when speed-setpoint is zero.
- Create pull request for waveshare IMU driver.

## License

[MIT](LICENSE). The vendored nanopb sources under `code/balancingbot/components/proto/` keep their own zlib license.
