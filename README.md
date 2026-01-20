# 🦆 Duckie Autonomous Robot (ROS 2)

A complete **ROS 2–based autonomous mobile robot** project built for **beginners**, running on **Ubuntu for Raspberry Pi**, featuring **lane following**, **ArUco marker navigation**, **motor control**, and a **safety watchdog**.

This repository is designed to teach **proper ROS 2 architecture**, **safe robotics practices**, and **clean modular design**.

---

## 📌 Features

- ✅ USB camera support (Logitech C110 tested)
- ✅ Lane following using OpenCV (HSV color detection)
- ✅ ArUco marker detection for junction control
- ✅ Finite State Machine (FSM) for robot behavior
- ✅ Motor control using `gpiozero` (Ubuntu compatible)
- ✅ Safety watchdog (automatic motor stop on failure)
- ✅ One-command startup using ROS 2 launch files
- ✅ Clean, modular ROS 2 package structure

---

## 🧠 System Architecture

```text
USB Camera
    ↓
usb_cam
    ↓  /image_raw
duckie_perception
    ↓  /cmd_motor_raw
duckie_safety (watchdog)
    ↓  /cmd_motor
duckie_motor
    ↓
Motors
