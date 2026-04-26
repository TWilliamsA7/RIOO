# RIOO (Robotic Intelligent Optical Operator)

RIOO is an assistive robotic arm controlled via real-time human gaze tracking. Developed as a 24-hour hardware hackathon project for HackaBull 2026, RIOO bridges the gap between computer vision and physical actuation to provide a hands-free interface especially for those with paralysis or spinal cord injuries.

Unlike standard robotic controllers, RIOO utilizes **Inverse Kinematics (IK)** and **Exponential Moving Average (EMA)** filtering to translate normalized gaze coordinates into fluid, organic movement.

## Features

- **Eye-Tracked Control:** Direct translation of gaze coordinates from a Raspberry Pi 3 vision system to physical (X, Y, Z) coordinates.
- **Dual-Node Architecture:** Distributed processing between a Raspberry Pi (Vision/Logic) and an ESP32 (Motion/Safety).
- **Intelligent Safety System:** Real-time collision avoidance using a non-blocking ultrasonic array (RCWL-1601) with automatic evasive rotation logic.
- **High-Torque Actuation:** 2:1 gear ratios on primary joints for increased resolution and lifting capacity.
- **4-Point Workspace Calibration:** Dynamic coordinate mapping to align the user’s field of vision with the arm's physical workspace.
- **Sequential Movement Logic:** Optimized power management and smooth motion profiles through single-joint-per-loop updates.

---

## System Architecture

RIOO is split into two primary processing nodes communicating over a dedicated UART bridge.

### 1. Vision Node (Raspberry Pi 3)

- **Role:** Processes eye-tracking data, performs 4-point calibration, and generates command packets.
- **Stack:** Python, OpenCV, Mediapipe, PySerial.

### 2. Motion Node (ESP32)

- **Role:** Executes Inverse Kinematics, manages servo PWM, processes safety sensors, and handles EMA filtering.
- **Stack:** C++, Arduino Framework, NewPing (Non-blocking), HardwareSerial.

---

## Hardware Specifications

| Component            | Specification                                              |
| :------------------- | :--------------------------------------------------------- |
| **Microcontroller**  | ESP32-WROOM-32                                             |
| **Vision Processor** | Raspberry Pi 3 Model B                                     |
| **Sensors**          | RCWL-1601 Ultrasonic (3.3V Native),VL1680 TOF Range Finder |
| **Primary Joints**   | High-Torque Servos with 2:1 Gear Ratios                    |
| **Logic Level**      | 3.3V (Direct ESP32/Pi Compatibility)                       |
| **Power Strategy**   | Star-grounded separate rails for Logic and Actuation       |

---

## Communication Protocol

Communication follows a header-based, comma/tag-separated UART protocol at **115200 Baud**.

### Gaze Packet (Live Tracking)

Sent from Pi to ESP32 at ~12 FPS.  
`TX[posx]Y[posy]G[grab]\n`  
_Example:_ `TX0.453Y-0.122G0`

### Setup Packet (Calibration)

Sent once after the 4-point calibration procedure.  
`C[xMin],[xMax],[yMin],[yMax]\n`  
_Example:_ `C-120.0,120.0,50.0,200.0`

---

## Control Algorithms

### Inverse Kinematics & Filtering

To ensure the arm doesn't "jitter" with natural eye micro-saccades, RIOO employs a stateful EMA filter:  
`filtered = (alpha * raw) + ((1.0 - alpha) * previous)`  
The filtered (X, Y) is then passed to a geometric IK solver to calculate joint angles for the base, shoulder, and elbow.

### Non-Blocking Collision Avoidance

Safety is handled via the `NewPing` library using a polling-based approach to prevent `pulseIn()` from blocking the main motion loop. If an obstacle is detected, the arm calculates an **Evasive Bias** to rotate away from the object without losing the user's target.

---

## Installation & Setup

### ESP32 (Firmware)

1. Install the dependencies listed within the `platformio.ini` file
2. Connect the ESP32 to your PC via Micro-USB.
3. Upload the RIOO firmware to the ESP32.
4. Ensure `Serial.setTimeout(5)` is configured for low-latency parsing.

### Raspberry Pi (Vision)

1. Connect the Pi to the ESP32 (via USB-Serial or GPIO Pins 14/15).
2. Enable UART in `raspi-config` and disable the Linux serial console.
3. Launch the main controller: `python3 mp_eye.py`.

### Arm (Mechanical)

1. Ensure that servos are set to their mechanical zero before assembly
2. Connect all grounds together in a star configuration to ensure valid signals and power transfer

---

## Contributors

- **Tai Williams** - System Architecture & Firmware
- **Aiden O'Connor** - Mechanical System & Design
- **Natalia Cano** - Computer Vision & Calibration
