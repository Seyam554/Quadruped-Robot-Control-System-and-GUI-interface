# Quadruped Robot Control System & GUI Interface 🐾💻

[![Demo Video](https://img.youtube.com/vi/Nx-Y6TwPJRM/0.jpg)](https://www.youtube.com/watch?v=Nx-Y6TwPJRM)

[![Python](https://img.shields.io/badge/Python-3.8+-blue.svg)](https://www.python.org/downloads/)
[![PyQt5](https://img.shields.io/badge/PyQt-5-green.svg)](https://pypi.org/project/PyQt5/)
[![YOLOv8](https://img.shields.io/badge/YOLO-v8-yellow.svg)](https://ultralytics.com/)
[![Arduino](https://img.shields.io/badge/Platform-Arduino/ESP32-teal.svg)](https://www.arduino.cc/)

A comprehensive, industry-grade robotic ecosystem integrating a physical quadruped robot (with a manipulator arm), a tactical teleoperation GUI, real-time computer vision for structural analysis, and an environmental monitoring module. This project is designed for advanced remote inspection, hazardous environment navigation, and structural health monitoring.

---

## 📖 Table of Contents
- [System Overview](#-system-overview)
- [Functional Requirements & Features](#-functional-requirements--features)
- [System Architecture & Methodology](#-system-architecture--methodology)
- [Module Descriptions](#-module-descriptions)
- [Hardware Stack](#-hardware-stack)
- [Software Setup & Installation](#-software-setup--installation)
- [Usage Guide](#-usage-guide)
- [Future Enhancements & Analysis](#-future-enhancements--analysis)

---

## 🚀 System Overview

This repository contains the complete software stack—from micro-controller firmware to deep learning inference scripts—for a highly capable quadruped robot. 

The core of the system is the **Tactical Edition PyQt5 GUI**, an intuitive, zero-browser desktop application that allows a human operator to control the robot via a gamepad, view real-time camera feeds, monitor telemetry, and trigger autonomous routines. Sub-systems include a deep-learning-based crack segmentation pipeline and a dedicated Air Quality Monitoring (AQM) sensor array.

---

## ✨ Functional Requirements & Features

### 1. Tactical Teleoperation GUI (`robot_gui.py`)
*   **Low-Latency Control:** UDP-based communication protocol (50 Hz transmission rate) ensuring responsive gamepad teleoperation.
*   **Dual-Camera Vision System:** Seamless toggling between an onboard ESP32-S3-CAM and a local auxiliary camera (e.g., laptop webcam).
*   **Adaptive UI/UX:** Features a dynamic interface with Dark/Light mode toggles, scalable typography, and real-time telemetry dashboards.
*   **Manipulator Arm Control:** Dedicated packet structures (12 Hz) for precise inverse kinematics or direct joint control of the integrated robotic arm.

### 2. Structural Health Analysis (Computer Vision)
*   **Crack Segmentation:** Utilizes a custom-trained Ultralytics YOLOv8 segmentation model (`yolo26n-seg.pt`) to detect and isolate structural anomalies (cracks) in real-time.
*   **Automated Inspection:** Designed to process frames from the robot's camera feed, mapping defects during locomotion.

### 3. Air Quality Monitoring (AQM)
*   **Environmental Telemetry:** A standalone module (`AQM.py` & `AQM_With_LED.ino`) that streams real-time environmental data over UDP from a dedicated ESP32 node.
*   **Visual Feedback:** Terminal-based rich UI with ANSI styling for monitoring metrics, coupled with physical LED indicators on the hardware node.

---

## 🏗️ System Architecture & Methodology

The system follows a **distributed computing architecture**, separating the high-level planning/interface from low-level real-time control:

1.  **Network Toplogy:** The robot hosts a localized Wi-Fi Hotspot (`wifi hotspot code/`). All subsystems (GUI PC, AQM node, Camera node) authenticate to this isolated network, ensuring secure, high-bandwidth communication independent of external infrastructure.
2.  **Communication Protocol:** High-frequency control packets are sent via **UDP datagrams** to minimize latency and avoid TCP overhead. The GUI transmits generalized motion vectors (X, Y, Yaw, Pitch, Roll) which the robot's firmware translates into joint angles.
3.  **Firmware Execution:** The master ESP32 runs an embedded C++ state machine (`V26_Full_Body_With_Arm_Control.ino`), parsing UDP payloads, interpolating trajectories, and driving PWM signals to the servo controllers.
4.  **Inference Pipeline:** The CV pipeline is decoupled. Video is streamed via HTTP/MJPEG from the ESP32-CAM to the host PC, where PyTorch/YOLO performs inference on dedicated hardware (GPU) to prevent frame drops in the control loop.

---

## 🧩 Module Descriptions

*   `robot_gui.py`: The central command station. Handles gamepad inputs (`inputs` library), renders the GUI (`PyQt5`), and manages asynchronous UDP socket threads.
*   `yolo_crack_seg.py` / `train_crack_seg.py`: Training and inference scripts for the YOLOv8-based crack segmentation model.
*   `AQM.py`: The Air Quality Monitoring terminal client. Connects to the AQM hardware node to display environmental data.
*   `V26_Full_Body_With_Arm_Control/`: Arduino IDE project containing the core kinematics and UDP server firmware for the robot.
*   `wifi hotspot code/`: Firmware to configure an ESP32 as a soft-AP.

---

## 🛠️ Hardware Stack

*   **Compute Nodes:** ESP32 (Main Controller), ESP32-S3-CAM (Vision Node), ESP32 (AQM Node).
*   **Actuation:** High-torque serial or PWM servos for the quadruped legs and manipulator arm.
*   **Input Device:** Standard USB/Bluetooth Gamepad (XInput compatible).
*   **Sensors:** Environmental sensors (AQM), RGB Camera module.
*   **Power:** High-discharge LiPo battery packs with buck converters for logic and servo power segregation.

---

## ⚙️ Software Setup & Installation

### Prerequisites
*   Python 3.8+
*   Arduino IDE (with ESP32 board manager installed)

### 1. Host Environment Setup (PC)
Clone the repository and install the required Python dependencies:

```bash
git clone https://github.com/Seyam554/Quadruped-Robot-Control-System-and-GUI-interface.git
cd Quadruped-Robot-Control-System-and-GUI-interface

# It is highly recommended to use a virtual environment
python -m venv .venv
source .venv/bin/activate  # On Windows: .venv\Scripts\activate

# Install dependencies (ensure PyTorch matches your CUDA version if using GPU)
pip install PyQt5 inputs opencv-python numpy requests ultralytics torch
```

### 2. Firmware Flashing
1.  Open `wifi hotspot code/quadruped_hotspot/quadruped_hotspot.ino` in Arduino IDE and flash it to the designated Hotspot ESP32.
2.  Open `V26_Full_Body_With_Arm_Control/V26_Full_Body_With_Arm_Control.ino` and flash it to the Main Controller ESP32.
3.  (Optional) Flash the AQM firmware to the sensor node.

---

## 🕹️ Usage Guide

1.  **Network Connect:** Connect your host PC's Wi-Fi to the robot's newly created Hotspot.
2.  **Launch GUI:**
    ```bash
    python robot_gui.py
    ```
3.  **Control:** Connect your gamepad. Use the GUI to adjust settings, toggle camera feeds, and verify the connection. The robot will respond to stick inputs based on the defined deadzones.
4.  **Crack Detection:** Run the inference script independently to analyze images or network streams:
    ```bash
    python yolo_crack_seg.py
    ```
5.  **Air Quality:** Launch the terminal dashboard:
    ```bash
    python AQM.py
    ```

---

## 🔬 Future Enhancements & Analysis

*   **Sensor Fusion:** Integrate IMU data from the ESP32 back into the GUI for real-time 3D pose estimation and auto-balancing algorithms.
*   **Edge AI Integration:** Migrate the YOLO crack segmentation inference directly onto an edge TPU (like Google Coral) or utilize the ESP32-S3's vector instructions to reduce PC dependency.

## Contributers
* Tazwar Ahmed
* Shahariar Hossain
* Shadid Al Akib
* Rafin Abrar

## **Autonomous Navigation:** Implement ROS 2 (Robot Operating System) nodes for SLAM (Simultaneous Localization and Mapping) using depth cameras/LiDAR.

---
*Developed for advanced robotic control and environmental interaction.*
