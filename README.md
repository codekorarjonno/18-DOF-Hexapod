# 18-DOF Hexapod

An undergraduate project to build a 3D-printed, 18-servo hexapod robot that can map its surroundings, navigate autonomously to a chosen point on the map, and pick up objects with a front-mounted gripper.

> **Status:** work in progress. The mechanical assembly and a first walking firmware (ESP32) are done. ROS 2, mapping and navigation are next.

---

## Features (planned and in progress)

- **Six legs, three joints each** (coxa, femur, tibia) for 18 degrees of freedom
- **Walking gaits:** tripod (fast) and wave (slow, stable)
- **Gripper** at the front of the base to grab objects
- **2D mapping** with an RPLIDAR C1 and `slam_toolbox`
- **Autonomous navigation** to a goal on the map with Nav2
- **Camera** for vision tasks
- **IMU** for body orientation and better odometry
- **Wireless control** from a Raspberry Pi 5 running ROS 2

---

## Hardware

| Component | Model | Qty | Purpose |
|---|---|---|---|
| Leg servos | MG995 | 12 | Leg joints |
| Leg servos | MG996R | 6 | Leg joints |
| Gripper servo | SG90 | 1 | Gripper open/close |
| Servo driver | PCA9685 (I2C, 16-ch) | 2 | Drives the 18 leg servos (addresses 0x40 and 0x41) |
| Microcontroller | STM32 (final) / ESP32 (testing) | 1 | Low-level servo and gait control |
| Main computer | Raspberry Pi 5 | 1 | ROS 2, SLAM, navigation, high-level commands |
| LiDAR | RPLIDAR C1 | 1 | Mapping and obstacle detection, mounted on top |
| IMU | MPU6050 (for now) | 1 | Orientation |
| Camera | Raspberry Pi Camera | 1 | Vision |
| Power | XL4005 buck converters | — | 6 V servo supply |

---

## System architecture

```
                 ┌──────────────────────────────────────────┐
                 │            Raspberry Pi 5 (ROS 2)        │
                 │  slam_toolbox · Nav2 · camera · IMU      │
                 └───────┬───────────────┬──────────────────┘
                         │               │
              commands   │               │ USB
           (WiFi / UART) │               │
                         ▼               ▼
              ┌───────────────────┐   RPLIDAR C1
              │ ESP32 / STM32     │
              │ gaits + gripper   │
              └───┬───────────┬───┘
                  │ I2C       │ PWM
                  ▼           ▼
          ┌──────────────┐  SG90 gripper
          │ PCA9685 0x40 │──► right legs (0, 1, 2)
          │ PCA9685 0x41 │──► left legs  (3, 4, 5)
          └──────────────┘
```

The Raspberry Pi does the thinking (mapping, path planning) and sends simple motion commands. The microcontroller turns those commands into coordinated servo movements.

---

## Leg layout

```
          front
      0 ───────── 3
      │           │
      1           4
      │           │
      2 ───────── 5
          back
```

- Legs 0–2 (right side) on PCA9685 **0x40**, legs 3–5 (left side) on **0x41**
- Tripod A = legs 0, 4, 2 · Tripod B = legs 3, 1, 5

---

## Current firmware (ESP32)

The test firmware `hexapod_gripper.ino` runs on the ESP32 and accepts text commands over USB serial or WiFi (TCP port 8888):

| Command | Action |
|---|---|
| `w` / `s` | Walk forward / backward |
| `a` / `d` | Turn left / right in place |
| `x` | Stop and settle into stance |
| `tripod` / `wave` | Choose the gait |
| `g` / `r` | Grab / release with the gripper |
| `t` | Test each leg in turn |
| `?` | Show all commands |

Before flashing, set your own WiFi name and password in the sketch.

---

## Roadmap

- [x] Mechanical assembly
- [x] Servo calibration and leg mapping
- [x] Tripod and wave gaits on ESP32
- [x] Gripper control
- [x] WiFi command link
- [ ] Port firmware to STM32
- [ ] ROS 2 setup on Raspberry Pi 5
- [ ] ROS 2 node to send commands to the microcontroller
- [ ] IMU integration (MPU6050)
- [ ] RPLIDAR C1 + `slam_toolbox` mapping
- [ ] Nav2 autonomous navigation
- [ ] Raspberry Pi camera integration
- [ ] Object pick-up with the gripper

---

## Software

- Arduino IDE (ESP32 firmware) — libraries: `Adafruit_PWMServoDriver`, `ESP32Servo`
- ROS 2 on Raspberry Pi 5 (planned)
- `slam_toolbox`, Nav2, `rplidar_ros` (planned)

---

## Project

Undergraduate project, Department of Mechatronics Engineering.
