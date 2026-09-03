# Poloticus

Poloticus is embedded C++ firmware for my differential-drive robot built on a Raspberry Pi Pico W but can be customized for many applications.

## Interfaces
The reusable interfaces currently include:

* `Motor`: motor direction and speed control through a PWM-based motor driver.
* `Encoder`: pulse-count and timer-window angular-speed measurement.
* `PIDController`: velocity feedback control with configurable gains and output limits.
* `Wheel`: higher-level abstraction, composed of wheel, and/or encoder and pid

* `MPU6050`: interface for the MPU6050 IMU.(very limited at this point,) 
* `I2cDevice`: reusable I2C device interface.
* `Buzzer`: funny addition to a project cos I have a buzzer, I have not used in a while

## Requirements

* `Board`: Raspberry Pi Pico W or another Raspberry Pi Pico-series board supported by the Pico SDK.
* `VS Code`: The Raspberry Pi Pico extension is recommended (makes life so easy, haha).or you can set up your environment the old Pico way.

My current Hardware is the Raspberry Pi `pico_w`. If you are using a different Pico-series board, change the configured board in `CMakeLists.txt`.


## Build

### Using VS Code
Import the project in VS Code as a Raspberry Pi Pico project using the Raspberry Pi Pico extension. donfigure and compile available in VS Code.

### `Alternative`: Using CMake and Ninja
From the project root:

```powershell
cmake -S . -B build -G Ninja
cmake --build build
```
