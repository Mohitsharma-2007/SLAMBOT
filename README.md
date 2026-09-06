<div align="center">

# 🤖 SLAM Bot
### Autonomous 2D SLAM Robot · ROS 2 Humble · Nav2 · Web Dashboard · AI Assistant

[![ROS 2](https://img.shields.io/badge/ROS_2-Humble_Hawksbill-22314E.svg?logo=ros&logoColor=white)](https://docs.ros.org/en/humble/)
[![FastAPI](https://img.shields.io/badge/FastAPI-0.100+-009688.svg?logo=fastapi&logoColor=white)](https://fastapi.tiangolo.com/)
[![React](https://img.shields.io/badge/React-18-61DAFB.svg?logo=react&logoColor=black)](https://react.dev/)
[![Arduino](https://img.shields.io/badge/Arduino_Uno_R4_WiFi-00979D.svg?logo=arduino&logoColor=white)](https://store.arduino.cc/products/uno-r4-wifi)
[![ESP8266](https://img.shields.io/badge/ESP8266-NodeMCU-E7352C.svg?logo=espressif&logoColor=white)](https://www.espressif.com/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

<br/>

<img src="SLAMBOT.png" alt="SLAM Bot Logo" width="120" style="border-radius: 16px; box-shadow: 0 4px 20px rgba(0,229,255,0.4);" />

<p align="center">
  A production-ready, differential-drive autonomous mapping robot featuring real-time 2D LiDAR SLAM, frontier-based autonomous exploration, sub-millimeter wheel odometry, and an interactive glassmorphic web dashboard.
</p>

</div>

---

## 📑 Table of Contents
1. [System Architecture](#-system-architecture)
2. [Hardware Components & BOM](#-hardware-components--bom)
3. [Wiring & Power Tree](#-wiring--power-tree)
4. [Repository Layout](#-repository-layout)
5. [Quickstart & Setup](#-quickstart--setup)
6. [Web Dashboard & Features](#-web-dashboard--features)
7. [License](#-license)

---

## 🏗 System Architecture

```
                                      ┌────────────────────────────────────────────────────────┐
                                      │                      PC / HOST                         │
                                      │                                                        │
┌───────────────────────────┐         │  ┌───────────────────────┐   ┌──────────────────────┐  │
│    Slamtec RPLIDAR A1     │         │  │   FastAPI Backend     │   │   ROS 2 Navigation   │  │
│  (360° 2D Laser Scanner)  │         │  │   (WebSocket Hub)     │   │   (Humble / Nav2)    │  │
└─────────────┬─────────────┘         │  │                       │   │                      │  │
              │ UART (115200)         │  │  • /ws/lidar ─────────┼──▶│  • /scan             │  │
              ▼                       │  │  • /ws/motion ────────┼──▶│  • /odom             │  │
┌───────────────────────────┐  WiFi   │  │  • /ws/app ───────────┼──▶│  • slam_toolbox      │  │
│      NodeMCU ESP8266      ├─────────┼─▶│  • /ws/relay ◀────────┼───┤  • Nav2 Planner/DWB  │  │
│   (LiDAR Packet Parser)   │ (5.5Hz) │  │                       │   │  • explore_node      │  │
└───────────────────────────┘         │  └───────────┬───────────┘   └──────────────────────┘  │
                                      │              │ WebSocket                               │
                                      │              ▼                                         │
┌───────────────────────────┐         │  ┌───────────────────────┐                             │
│    Arduino Uno R4 WiFi    │  WiFi   │  │    React Dashboard    │                             │
│   (50Hz PID Motor Ctrl    ├─────────┼─▶│   (Live Map / Tuning) │                             │
│    & Quadrature Odom)     │ (20Hz)  │  └───────────────────────┘                             │
└─────────────┬─────────────┘         └────────────────────────────────────────────────────────┘
              │ PWM (DRV8833) + Interrupts
              ▼
┌───────────────────────────┐
│  2x N20 Gearmotors with   │
│ Optical/Magnetic Encoders │
└───────────────────────────┘
```

---

## 🔩 Hardware Components & BOM

For complete part numbers, links, and detailed notes, see [`docs/HARDWARE_AND_WIRING.md`](docs/HARDWARE_AND_WIRING.md).

| Component | Quantity | Key Specifications | Role |
|---|:---:|---|---|
| **Arduino Uno R4 WiFi** | 1 | Renesas RA4M1 32-bit (48 MHz), ESP32-S3 | 50 Hz PID wheel velocity control & 20 Hz odometry |
| **NodeMCU ESP8266** | 1 | Tensilica L106 80 MHz, 802.11 b/g/n | High-speed RPLIDAR packet parsing & WiFi relay |
| **Slamtec RPLIDAR A1M8** | 1 | 360° 2D LiDAR, 12 m range, 5.5–10 Hz | 2D Laser range-finding for SLAM |
| **N20 Micro Gearmotors** | 2 | 6V DC, 100:1 Gear Ratio, 700 CPR Encoders | Differential drive propulsion with encoder feedback |
| **DRV8833 Dual H-Bridge** | 1 | 2.7–10.8V, 1.5A RMS per channel | Low-loss dual DC motor driver |
| **LM2596 Buck Converter** | 1 | Step-down tuned to **5.00V ± 0.05V** | Regulates battery down to clean 5V logic rail |
| **2S LiPo / Li-Ion Battery**| 1 | 7.4V nominal (8.4V max), 1500–2200 mAh | Primary power supply |
| **Electrolytic Capacitor** | 1 | 470 µF – 1000 µF (16V/25V) | Absorbs RPLIDAR spinup voltage drops |
| **2WD Mini Chassis** | 1 | 43mm rubber wheels, 150mm track width | Base platform |

---

## ⚡ Wiring & Power Tree

> [!CAUTION]
> **Power Isolation**: Motors pull up to 1.5A during startup. The battery positive rail splits directly: one branch to `DRV8833 VM` (motors), and the other through the `LM2596` (5.00V) to logic boards. Never power motors from the 5V logic rail!

```
2S LiPo (7.4V - 8.4V) ──┬── Raw Battery ────────────▶ DRV8833 VM (Motor Power)
                        │
                        └── LM2596 (5.00V Regulated) ──▶ 5V Bus (Capacitor Filtered)
                                                          ├── Arduino Uno R4 (5V pin)
                                                          ├── NodeMCU (VIN)
                                                          ├── RPLIDAR A1M8 (5V)
                                                          └── Encoders & DRV8833 VCC
```

### Arduino Uno R4 Pin Map
* `D2` — Left Encoder CH-A (Hardware Interrupt)
* `D4` — Left Encoder CH-B (Digital Read)
* `D3` — Right Encoder CH-A (Hardware Interrupt)
* `D5` — Right Encoder CH-B (Digital Read)
* `D6` — Left Motor PWM `AIN1`
* `D11` — Left Motor PWM `AIN2`
* `D10` — Right Motor PWM `BIN1`
* `D9` — Right Motor PWM `BIN2`
* `D8` — DRV8833 `nSLEEP` (Driver Enable)

---

## 📂 Repository Layout

```
SLAM-Bot/
├── backend/                 # FastAPI WebSocket hub, state machine & REST API
│   ├── main.py              # Application entrypoint
│   ├── ws_robot.py          # Microcontroller WebSocket endpoints (/ws/motion, /ws/lidar)
│   ├── ws_frontend.py       # Browser & ROS2 relay WebSocket endpoints
│   ├── state.py             # Atomic thread-safe state container
│   └── tuning.py            # Runtime calibration engine & EEPROM syncing
├── firmware/
│   ├── arduino_uno_r4/      # Arduino C++ motion controller sketch
│   ├── nodemcu_lidar/       # ESP8266 C++ RPLIDAR stream sketch
│   └── movement_test/       # Standalone hardware validation sketch
├── ros2_ws/                 # ROS 2 Humble Workspace
│   └── src/
│       ├── slam_bot_bridge/ # Bridge nodes, TF broadcast, frontier exploration
│       ├── slam_bot_bringup/# Launch files (slam_toolbox + nav2 + bridge)
│       └── slam_bot_nav/    # SmacPlanner2D (A*) & DWB local controller configs
├── webapp/                  # Modern glassmorphic React 18 + Vite dashboard
│   ├── src/pages/           # Live Map, Dashboard, Tuning, Flash Center, Logs, AI
│   └── src/components/      # Canvas occupancy grid renderer, polar radar scan
└── docs/                    # Architectural specs, hardware guides, and media
```

---

## 🚀 Quickstart & Setup

### 1. Configure WiFi Credentials
Copy the example secret files and insert your local WiFi SSID and Password:
```bash
cp firmware/arduino_uno_r4/secrets.h.example firmware/arduino_uno_r4/secrets.h
cp firmware/nodemcu_lidar/secrets.h.example firmware/nodemcu_lidar/secrets.h
```
*Flash `arduino_uno_r4.ino` to the Uno R4 and `nodemcu_lidar.ino` to the ESP8266 via Arduino IDE.*

### 2. Launch FastAPI Backend
```bash
cd backend
python -m venv .venv
source .venv/bin/activate    # On Windows: .venv\Scripts\activate
pip install -r requirements.txt
python -m uvicorn main:app --host 0.0.0.0 --port 8000 --ws wsproto
```

### 3. Launch ROS 2 Nav2 & SLAM (WSL2 or Linux)
```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch slam_bot_bringup bringup.launch.py standalone_bridge:=true
```

### 4. Open Web Dashboard
Navigate to `http://localhost:8000` or `http://<HOST_IP>:8000` in any web browser.

---

## 🖥 Web Dashboard & Features

* **Real-time Occupancy Grid**: Canvas-accelerated `/map` rendering with dynamic zoom, pan, and click-to-navigate goal dispatching.
* **Polar LiDAR Radar**: Live 360° point cloud visualization with distance filters and angle masking.
* **Live PID & Motion Tuning**: Adjust motor speeds, PID gains, deadband thresholds, and motor polarity in real time with 1-click EEPROM persistence.
* **Autonomous Frontier Exploration**: 1-click auto-explore to map entire rooms autonomously.
* **Integrated Safety Watchdog**: 2000 ms link-loss automatic e-stop and collision avoidance.

---

## 📜 License
Distributed under the MIT License. See `LICENSE` for details.
