# Digital Weight: Latest Version 4 (V4) - Overview

## Release Date

June 2024, as demonstrated at open sauce SF

## Introduction

The Digital Weight V4 represents the latest iteration of our open source fitness device. Working next towards V5 which will maintain the innovative and open spirit seen at Open Sauce. Version 4 introduces several enhancements and new features aimed at improving usability, safety, and performance.

## V4 Features

- **Dual Motor System**: Incorporates dual motors and controllers for enhanced performance.
- **Supports rowing**: The smaller motor and one-way clutch system allows this unit to rapidly respond with high speed while maintaining low inertia for safe use
- **Main Controller**: Utilizes an esp32s3 main controller board equipped with a CAN transceiver.
- **Design**: Features a flat output arm that can fold out flat onto the ground for versatile "free-floating" usage and easy mounting.
- **User Interface**: Equipped with a touch screen display and a physical rotary input knob for intuitive operation.
- **Strap**: Includes 6 meters (20 ft) of seatbelt webbing strap for secure handling.
- **Safety**: Integrated accelerometer/gyro for detecting device movement, enhancing safety.

## V4 Known Issues and Limitations

- **Thermal Management**: Both motors tend to heat up too quickly, indicating a potential thermal issue.
- **Maintenance Challenges**:
  - Difficulty in removing motors with controllers for calibration.
  - Painful removal process for the handle when maintaining the odrive s1 controller.
- **Startup Stability**: Power startup sequence with the touch screen is not stable, especially when the screen powers up before the device.
- **Power Distribution**: Issues with hijacking power for the LCD and main controller from the odrive S1.
- **Wiring**: Wire congestion around the device leads to pinched wires, reducing load carrying capacity.
- **Battery**: The current battery setup lacks an on-board BMS, balancer, power management, or thermal monitoring.
- **Mounting**: The existing frame mounting requires straps placed around the unit, necessitating a better solution.
- **Firmware**: The firmware has been rewritten from scratch for V4, resulting in the absence of some base features critical for safety and convenience.
- **General safety**: The device has not yet fully enabled all watchdog, movement monitoring, and control limitations to ensure the safety of the user. Test and use with caution.

## V4 Image Preview

![V4 Render Angle](cad/v4/images/render_angle.png)

## V5 Next Steps

Squash bugs and known limitations
