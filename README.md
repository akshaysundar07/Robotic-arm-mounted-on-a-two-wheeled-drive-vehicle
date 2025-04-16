# Robotic Rover Control System

This repository contains the source code for a robotic rover equipped with a 4-DOF robotic arm and a claw end effector, controlled via a web interface. The system uses a Raspberry Pi, L298N motor drivers for the 4-wheeled base, PCA9685 for servo control, and ADXL345 for vibration analysis. It includes predictive maintenance features based on motor current signal analysis.

## Features
- **Mobility**: 4-wheeled differential drive with forward, backward, left, and right movement.
- **Arm Control**: 4-DOF arm with smooth servo movements for base, elbow, and gripper.
- **Web Interface**: Flask-based control panel accessible via browser.
- **Vibration Analysis**: Real-time anomaly detection using ADXL345 and FFT processing.
- **Predictive Maintenance**: Motor current signal analysis for fault prediction.

## Hardware Requirements
- Raspberry Pi (with GPIO and I2C enabled)
- L298N Motor Driver
- PCA9685 Servo Driver
- 4 Servos (for 4-DOF arm)
- ADXL345 Accelerometer (via SPI)
- 4 DC Motors (for wheeled base)
- Power Supply (e.g., 7.4V LiPo battery)

## Software Requirements
- Python 3.x
- Libraries:
  - `RPi.GPIO`
  - `adafruit-pca9685`
  - `adafruit-motor`
  - `flask`
  - `spidev`
  - `numpy`
- Operating System: Raspberry Pi OS

## Installation
1. Clone the repository:
   ```bash
   git clone https://github.com/yourusername/rover.git
   cd rover
