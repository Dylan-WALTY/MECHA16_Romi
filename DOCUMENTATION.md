# Project Documentation for Mecha 16 Romi

## Table of Contents
1. [System Architecture](#system-architecture)
2. [Hardware Components](#hardware-components)
3. [Software Modules](#software-modules)
4. [Control Systems](#control-systems)
5. [State Machine Design](#state-machine-design)
6. [Performance Results](#performance-results)

---

## System Architecture
The Mecha 16 Romi project is designed with a modular architecture that separates hardware and software components. The architecture includes:
- Microcontroller as the central processing unit
- Sensor modules for environmental detection
- Actuator modules for movement and control

## Hardware Components
- **Microcontroller:** Atmel ATmega328P
- **Motor Drivers:** L298N motor driver
- **Sensors:** Infrared obstacle detection sensors, line following sensors
- **Power Supply:** 6V rechargeable battery pack

## Software Modules
The software is composed of several modules that manage different aspects of the robot's functionality:
- **Sensor Module:** Interfaces with sensors to gather data.
- **Actuator Module:** Controls motors based on sensor input.
- **Communication Module:** Handles communication between modules.

## Control Systems
The control system employs a PID controller to improve stability and responsiveness of the movement.

## State Machine Design
The robot’s behavior is modeled using a state machine that includes the following states:
- **Idle:** Waiting for input.
- **Moving:** Active state where the robot navigates.
- **Obstacle Detected:** Changes behavior based on sensor input.

## Performance Results
The performance of the Mecha 16 Romi has been evaluated under various conditions with the following results:
- **Speed:** Maximum speed of 10 cm/s.
- **Response Time:** Less than 200ms for sensor input.

---

This document will be updated as the project progresses.