# Autonomous Hovercraft

## Overview

This project contains the embedded C code used to control our autonomous hovercraft. The hovercraft uses an AVR microcontroller, an MPU6050 gyroscope, an ultrasonic sensor, fans, and a servo to move around and avoid obstacles.

## Main Features

- Uses the **MPU6050 gyroscope** to track the hovercraft's rotation and heading.
- Uses an **ultrasonic sensor** to detect walls and obstacles in front of the hovercraft.
- Uses **PI control** to help the hovercraft stay straight while moving forward.
- Uses **PD control** while turning to reduce overshoot and improve the turn.
- Scans both the **left and right sides** when an obstacle is detected and turns toward the side with more space.
- Controls the **lift fan and propulsion fan using PWM**.
- Controls the steering servo using the AVR timer.
- Includes a **stuck detection** function so the hovercraft can try another direction if it stops making progress.
- Uses different states for cruising, approaching an obstacle, scanning, turning, and straightening after a turn.

## Hardware Used

- AVR microcontroller
- MPU6050 gyroscope
- Ultrasonic distance sensor
- Servo motor
- Lift fans
- Propulsion fan

## Code

The main control code is written in embedded C and handles the sensors, fan control, steering, and autonomous navigation.
