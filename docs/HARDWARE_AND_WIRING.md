# SLAM Bot Hardware & Wiring Guide

This document outlines the complete Bill of Materials (components list), pin mappings, and power distribution tree for the SLAM Bot differential-drive autonomous mapping platform.

---

## 1. Bill of Materials (Components List)

| Category | Component | Qty | Specifications & Details | Role / Notes |
|---|---|---|---|---|
| **Microcontrollers** | Arduino Uno R4 WiFi | 1 | Renesas RA4M1 32-bit ARM Cortex-M4 (48 MHz), ESP32-S3 Wi-Fi/BLE module, 12x8 LED matrix | Main motion controller: executes 50 Hz PID control loop, counts quadrature encoder ticks, integrates 20 Hz odometry |
| **Microcontrollers** | NodeMCU ESP8266 | 1 | Tensilica 32-bit L106 @ 80/160 MHz, 802.11 b/g/n Wi-Fi | Dedicated RPLIDAR parser & WebSocket telemetry relay |
| **LIDAR Sensor** | Slamtec RPLIDAR A1M8 | 1 | 360° 2D Laser Scanner, 12m range, 5.5–10 Hz scan rate, ~400–8000 samples/s, 115200 baud UART | Real-time environment mapping & obstacle avoidance |
| **Motors & Drive** | N20 Micro Metal Gearmotors with Magnetic Quadrature Encoders | 2 | 6V DC, 100:1 Gear Ratio, ~100–300 RPM, 700 CPR (counts/rev output shaft), D-shaft | Differential drive propulsion + closed-loop speed feedback |
| **Motor Driver** | Texas Instruments DRV8833 Dual H-Bridge Module | 1 | 2.7V–10.8V operating range, 1.5A RMS per channel (2A peak), low RDS(ON), ultra-low sleep current | Controls Left & Right N20 motors via PWM |
| **Power Management** | 2S LiPo / Li-Ion Battery Pack | 1 | 7.4V nominal (8.4V full charge), 1500–2200 mAh, 20C+ discharge | Powers motors directly (VM) and step-down converter |
| **Voltage Regulation** | LM2596 DC-DC Buck Converter Module | 1 | Input 4.5V–40V, Output adjusted to **5.00V ± 0.05V**, 3A max output | Steps down 7.4V battery to clean 5V rail for all logic/sensors |
| **Capacitor & Filtering** | Electrolytic Buffer Capacitor | 1 | 470 µF – 1000 µF, 16V / 25V | Placed across 5V rail to absorb RPLIDAR spin-up surge |
| **Power Switch** | SPST Rocker / Toggle Switch | 1 | 5A rating | Main battery power disconnect switch |
| **Chassis & Wheels** | 2WD Acrylic / 3D-Printed Mini Chassis | 1 | Two 43mm silicone rubber drive wheels + 1 front ball caster | Robot base frame (150mm track width) |
| **Cables & Harness** | Jumper Wires & Power Distribution Rail | 1 kit | 22–24 AWG silicone stranded wires, mini breadboard or power distribution board | Star-ground and 5V power bus |

---

## 2. Power Architecture: Single Battery, Dual-Rail Isolation

```
                   ┌── [SPST Main Switch] ── 2S LiPo (7.4V - 8.4V) ──┐
                   │                                                  │
                   ▼ (Raw Battery Power)                              ▼ (Input)
            DRV8833 VM (Pin 1)                              LM2596 Buck Converter
          (Motors stall up to 1.5A)                         (Trimmed to exactly 5.00V)
                   │                                                  │
                   │                                                  ▼ (Clean 5.0V Logic Rail)
                   │                                           [470µF Filter Cap]
                   │                                                  │
                   │                         ┌────────────────────────┼────────────────────────┐
                   │                         ▼                        ▼                        ▼
                   │                  Arduino Uno R4             NodeMCU ESP8266         RPLIDAR A1M8
                   │                     (5V Pin)                    (VIN)                   (5V)
                   │                         │                        │                        │
                   │                         └────────┬───────────────┴────────────────────────┘
                   │                                  ▼
                   └───────────────────────▶ [COMMON STAR GROUND]
```

> [!IMPORTANT]
> **Motor Isolation Rule**: Motors pull ~1.5A surge when starting. Motors are powered **directly from raw battery (VM)** to prevent brownout resets on the microcontrollers. The 5.00V buck converter powers the logic only.

---

## 3. Pin Mapping & Interconnect Table

### A. Arduino Uno R4 WiFi $\rightarrow$ DRV8833 & Encoders

| Signal | Arduino Pin | DRV8833 / Encoder Pin | Description / Type |
|---|---|---|---|
| **Encoder Left A** | **D2** | Left Motor Encoder CH-A | Hardware Interrupt (INT0) |
| **Encoder Left B** | **D4** | Left Motor Encoder CH-B | Digital Input (Sampled in ISR) |
| **Encoder Right A** | **D3** | Right Motor Encoder CH-A | Hardware Interrupt (INT1) |
| **Encoder Right B** | **D5** | Right Motor Encoder CH-B | Digital Input (Sampled in ISR) |
| **Motor Left AIN1** | **D6** | DRV8833 AIN1 | Hardware PWM (Forward speed) |
| **Motor Left AIN2** | **D11** | DRV8833 AIN2 | Hardware PWM (Reverse speed) |
| **Motor Right BIN1** | **D10** | DRV8833 BIN1 | Hardware PWM (Forward speed) |
| **Motor Right BIN2** | **D9** | DRV8833 BIN2 | Hardware PWM (Reverse speed) |
| **Motor Sleep** | **D8** | DRV8833 nSLEEP | Digital Output (HIGH = Active, LOW = Low Power Sleep) |
| **Logic VCC** | **5V** | DRV8833 VCC & Encoders VCC | 5V Logic Supply |
| **Ground** | **GND** | DRV8833 GND & Encoders GND | Common Ground |

### B. NodeMCU ESP8266 $\rightarrow$ RPLIDAR A1M8

| NodeMCU Pin | RPLIDAR A1 Pin | Wire Function | Note |
|---|---|---|---|
| **GPIO3 (RXD0 / RX)** | **TX** | LIDAR Telemetry Data Out | 115200 baud serial packets |
| **GPIO1 (TXD0 / TX)** | **RX** | LIDAR Command In | Start / Stop / Scan commands |
| **VIN (or 5V)** | **5V / VCC** | Laser Core Power | Connected to 5.00V regulated rail |
| **GND** | **GND** | Sensor Ground | Common ground |
| **5V Rail** | **MOTO_PWM / MOTO_EN** | Motor Power | Continuous rotation at ~5.5 Hz |

---

## 4. Visual Diagrams & Media

*(Place your images in `docs/images/` using the file paths below)*

### Circuit Schematic
![Circuit Diagram](images/circuit_diagram.png)

### Assembled Robot Chassis
![Robot Chassis](images/robot_chassis.jpg)
