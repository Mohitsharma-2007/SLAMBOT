# Production-Ready FLUX.ai PCB Design Prompt & Hardware Architecture Specification

## 📋 How to Use This Prompt in FLUX.ai
Copy and paste the prompt in Section 1 directly into the **Flux Copilot** chat window. Flux will automatically generate the schematic hierarchy, select approved library components, route net connections, configure design rules, and prepare the 2-layer or 4-layer PCB layout.

---

## 1. Complete FLUX Copilot Master Prompt

```text
Design a production-grade, 2-layer (or 4-layer) integrated robotics carrier PCB named "SLAM_BOT_MAINBOARD_V2" for an autonomous 2D LiDAR differential-drive mobile robot. The design must feature a dual-microcontroller architecture (Arduino Uno R4 WiFi socket + NodeMCU ESP8266 / ESP32-S3 module), a dual-rail power distribution system, low-noise motor drive circuitry, and modular plug-and-play expansion headers for future sensory upgrades (I2C/SPI Display, DVP/ESP32 Camera, I2S Digital Microphone, and I2S Audio Amplifier with Speaker).

Follow these exact architectural requirements, net connections, and design rules:

==============================================================================
1. POWER DISTRIBUTION TREE & DUAL-RAIL ELECTRICAL TOPOLOGY
==============================================================================
- Input Power Connector:
  * 1x XT30 or 2-pin 3.81mm screw terminal for 2S LiPo battery (7.4V nominal, 8.4V peak, 2200mAh).
  * 1x SPDT high-current slide switch (SS-12D00G3 or similar, rated >= 3A) in series with positive battery lead.
- Input Reverse Polarity & Surge Protection:
  * P-Channel Power MOSFET (AO3401A or SI2301CDS, SOT-23) configured as low-loss reverse polarity protection gate on the high side.
  * 1x PPTC resettable fuse (MF-MSMF260, 2.6A hold, 5.0A trip, 1812 footprint).
  * 1x SMAJ9.0A or SMBJ9.0A TVS diode across battery input for inductive load dump suppression.
  * 1x Green "BATT_OK" LED with 2.2k resistor.
- Dual-Rail Power Branching:
  * Branch A: Raw Motor Rail (VM, 7.4V - 8.4V):
    - Connects directly after reverse polarity MOSFET and PPTC fuse to the DRV8833 motor driver VM pin.
    - Decoupled with 1x 470uF 25V low-ESR radial electrolytic capacitor and 2x 10uF 1206 ceramic capacitors placed directly adjacent to motor driver pins.
  * Branch B: Regulated 5.00V Logic Rail (5V_SYS, 3.0A):
    - Texas Instruments TPS54302 or Monolithic Power Systems MP1584 high-frequency synchronous step-down buck converter (or LM2596-5.0 module footprint).
    - Input: 7.4V battery rail.
    - Output: 5.00V +/- 0.05V, rated for 3A continuous output current.
    - Inductor: 10uH shielded power inductor (rated >= 4A saturation current).
    - Filter capacitors: 2x 22uF 16V 1206 ceramic capacitors at output, buffered by 1x 470uF 10V low-ESR electrolytic capacitor.
    - 1x Blue "5V_LOGIC" power LED with 1k resistor.
  * Branch C: Regulated 3.30V Peripheral Rail (3V3_SYS, 1.0A):
    - AMS1117-3.3 (SOT-223) or AP2112K-3.3 (SOT-23-5) linear regulator fed from the 5V_SYS rail.
    - Decoupled with 1x 10uF 0805 ceramic at input, 1x 22uF 0805 ceramic + 100nF 0402 ceramic at output.
    - Powers 3.3V expansion sensors, camera logic, and microphone.

==============================================================================
2. MICROCONTROLLER SOCKETS & CORE PROCESSING
==============================================================================
- Microcontroller 1 (Motion & Odometry Controller):
  * Female pin header sockets mating with standard Arduino Uno R4 WiFi form factor (0.1" pitch female headers: 1x 10-pin, 2x 8-pin, 1x 6-pin).
  * Connects 5V_SYS to Arduino 5V pin; Connects System GND to Arduino GND pins.
  * Pin Routing:
    - Arduino D2: Connected to LEFT_ENC_A with 10k pull-up to 5V.
    - Arduino D4: Connected to LEFT_ENC_B with 10k pull-up to 5V.
    - Arduino D3: Connected to RIGHT_ENC_A with 10k pull-up to 5V.
    - Arduino D5: Connected to RIGHT_ENC_B with 10k pull-up to 5V.
    - Arduino D6 (PWM): Left Motor Forward (DRV8833 AIN1).
    - Arduino D7: Left Motor Backward (DRV8833 AIN2).
    - Arduino D9 (PWM): Right Motor Forward (DRV8833 BIN1).
    - Arduino D8: Right Motor Backward (DRV8833 BIN2).
    - Arduino SDA (A4) / SCL (A5): Connected to I2C bus with 4.7k pull-up resistors to 5V/3V3.
- Microcontroller 2 (Optical LiDAR Telemetry Bridge):
  * Dual row female header footprint mating standard NodeMCU ESP8266 V3 / CP2102 (15 pins x 2 rows, 0.9" row spacing) OR dual footprints allowing an ESP32-S3-WROOM-1 module.
  * Connects 5V_SYS to ESP8266 VIN; GND to GND.
  * Pin Routing:
    - ESP8266 RX0 (GPIO3): Connected to RPLIDAR_TX.
    - ESP8266 TX0 (GPIO1): Connected to RPLIDAR_RX.
    - ESP8266 GPIO4 (SDA) and GPIO5 (SCL): Auxiliary inter-MCU I2C / telemetry link.

==============================================================================
3. MOTOR DRIVE & ACTUATION SUBSYSTEM
==============================================================================
- Motor Driver IC:
  * Texas Instruments DRV8833PWP (HTSSOP-16 with exposed thermal pad) or 16-pin socket for standard DRV8833 breakout board.
  * VM pin connected to Raw Battery Rail with low-ESR bulk capacitor (470uF) and 100nF ceramic capacitor.
  * VINT connected to 10uF bypass.
  * Current sense pins (AISEN, BISEN) tied to GND via 0.20 Ohm 1W 1206 current sense resistors (or jumpered to GND if internal limit not used).
  * Fault line (nFAULT) pulled up to 5V with 10k resistor and routed to Arduino A0.
  * Sleep pin (nSLEEP) pulled up to 5V through 10k resistor.
- Motor Output Connectors:
  * Left Motor: JST-XH 6-pin male connector (P1: VM+, P2: VM-, P3: 5V, P4: GND, P5: ENC_A, P6: ENC_B).
  * Right Motor: JST-XH 6-pin male connector (P1: VM+, P2: VM-, P3: 5V, P4: GND, P5: ENC_A, P6: ENC_B).
  * Provide 100nF 50V ceramic filtering capacitors across motor terminal leads to dampen high-frequency brush arcing noise.

==============================================================================
4. LIDAR SENSOR INTERFACE
==============================================================================
- 1x JST-XH 5-pin right-angle male connector for Slamtec RPLIDAR A1:
  * Pin 1: 5V_SYS (Operating power, 5V @ 500mA peak).
  * Pin 2: RPLIDAR_RX (Connects to ESP8266 TX0 / GPIO1).
  * Pin 3: RPLIDAR_TX (Connects to ESP8266 RX0 / GPIO3).
  * Pin 4: GND (Star ground).
  * Pin 5: MOTOCTRL (LiDAR motor PWM speed control).
- Motor PWM Drive Circuitry:
  * 2N7002 or BSS138 N-channel MOSFET driving MOTOCTRL to ground via PWM.
  * Gate driven from ESP8266 GPIO14 (or Arduino D10) with 10k pull-down resistor and 100 Ohm series gate resistor.
  * Flyback diode (1N4148 or BAT54) across 5V and MOTOCTRL drain line.

==============================================================================
5. FUTURE-PROOF EXPANSION HEADERS & PERIPHERALS
==============================================================================
Please break out dedicated, clearly silkscreened expansion connectors for future hardware upgrades:

A. DISPLAY INTERFACE (Dual Support):
   1. 1x 4-pin 0.1" female/male header for I2C OLED (0.96" or 1.3" SSD1306/SH1106):
      - Pin 1: GND
      - Pin 2: VCC (Selectable 5V / 3V3 via 3-pin solder jumper, default 3V3)
      - Pin 3: SCL (with 4.7k pull-up)
      - Pin 4: SDA (with 4.7k pull-up)
   2. 1x 8-pin 0.1" header for SPI Color TFT Display (ST7789 or ILI9341):
      - Pins: GND, 3V3, SCLK (SCK), MOSI (SDA), RES (Reset), DC (Data/Command), BLK (Backlight PWM), CS (Chip Select).

B. CAMERA & VISION INTERFACE:
   1. 1x 16-pin 0.1" dual-row header configured to accept an ESP32-CAM AI-Thinker daughterboard or USB-C Host camera module:
      - Pinout: 5V, 3V3, GND, U0T, U0R, GPIO0, GPIO16, GPIO2, GPIO14, GPIO15, GPIO13, GPIO12, GPIO4, GND.
   2. Optional footprint for 24-pin 0.5mm pitch bottom-contact FPC connector for direct OV2640 DVP camera module with 3.3V and 1.3V LDO rails.

C. AUDIO & VOICE INTERFACE:
   1. Digital I2S Microphone Header (INMP441 or SPH0645):
      - 1x 6-pin 0.1" header:
        * Pin 1: 3V3_SYS
        * Pin 2: GND
        * Pin 3: I2S_SD (Serial Data)
        * Pin 4: I2S_WS (Word Select / Left-Right Clock)
        * Pin 5: I2S_SCK (Serial Continuous Clock)
        * Pin 6: L/R (Channel select, pull-down to GND for Left channel)
   2. I2S Audio Amplifier & Speaker Output:
      - Integrate onboard footprint for Maxim Integrated MAX98357A 3.2W Class-D Audio DAC/Amp (QFN-16) or 1x 7-pin header for breakout module:
        * Inputs: I2S_DIN, I2S_BCLK, I2S_LRC, 5V_SYS, GND, GAIN, SD_MODE.
      - Output: 1x 2-pin 3.5mm screw terminal (P1: SPK+, P2: SPK-) to drive an external 4-Ohm or 8-Ohm 3W mini speaker.
      - Filter: 2x 220pF EMI filter capacitors to GND on speaker lines.

D. AUXILIARY GPIO & I2C EXPANSION:
   - 1x Qwiic / Stemma QT compatible 4-pin JST-SH (1.0mm) connector (3V3, GND, SDA, SCL) for daisy-chaining IMUs (BNO085/MPU6050) and ToF distance sensors (VL53L0X).

==============================================================================
6. PCB LAYOUT, GROUNDING & SIGNAL INTEGRITY RULES
==============================================================================
- Board Dimensions: 100 mm x 100 mm (compatible with standard $2 budget PCB manufacturing).
- Layer Stackup: 2-Layer FR4, 1.6mm thickness, 1 oz copper (or 4-layer: Signal - GND Plane - Power Plane - Signal for superior EMI).
- Grounding Strategy:
  * Implement strict split star grounding: Power Ground (PGND for motors, buck converter switch node, LiPo ground) and Signal Ground (SGND for microcontrollers, encoders, audio, and I2C).
  * Join PGND and SGND at a single physical point directly at the battery negative terminal using a 0-Ohm resistor or Net-Tie (NET_TIE).
- Trace Widths:
  * Battery Input & Motor Traces (VM, Motor Outs): Minimum 60 mil (1.5 mm), routed on both top and bottom copper with stitching vias.
  * 5V_SYS Logic Power: Minimum 30 mil (0.75 mm).
  * 3V3_SYS Power: Minimum 20 mil (0.5 mm).
  * Low-voltage Signals & I2C/SPI/UART: 10–12 mil.
- Decoupling & Placement:
  * Place 100nF decoupling capacitors within 3 mm of every IC power pin (DRV8833, AMS1117, MAX98357A, buck converter).
  * High-frequency buck converter loop (inductor, input cap, catch diode/MOSFET) must be kept ultra-compact.
  * Keep motor driver and inductor switching nodes physically distant (> 15 mm) from the I2S microphone lines and I2C lines to prevent inductive cross-talk.
- Mechanical:
  * 4x M3 mounting holes located at the corners (90 mm x 90 mm center-to-center spacing) with 6mm circular copper ground keep-outs.
  * High-visibility silkscreen with all pin names, power rails, component polarities, and a high-contrast SLAM Bot logo.
```

---

## 2. Hardware Bill of Materials (BOM) for FLUX Design

| RefDes | Component Description | Footprint / Package | Nominal Specs | Purpose |
| :--- | :--- | :--- | :--- | :--- |
| **J1** | Battery Input Connector | XT30-M / 3.81mm Screw | 7.4V–8.4V, 15A | LiPo Battery Input |
| **Q1** | Reverse Polarity P-MOSFET | SOT-23 (AO3401A) | -30V, -4.2A, $44\text{m}\Omega$ | Low-loss reverse protection |
| **F1** | Resettable Fuse | 1812 SMD (MF-MSMF260) | 2.6A Hold, 5.0A Trip | Overcurrent protection |
| **D1** | TVS Surge Diode | SMB / DO-214AA (SMBJ9.0A) | 9.0V Breakdown, 600W | Back-EMF suppression |
| **U1** | Synchronous Buck Converter | SOP-8 / Module (MP1584) | 4.5V–28V In, 5.0V 3A Out | 5.00V Logic Rail |
| **U2** | Linear LDO Regulator | SOT-223 (AMS1117-3.3) | 5V In, 3.3V 1A Out | 3.3V Sensor Rail |
| **U3** | Dual H-Bridge Motor Driver | HTSSOP-16 (DRV8833PWP) | 2.7V–10.8V, 1.5A/ch | N20 Motor Control |
| **U4** | I2S Class-D Amplifier | TQFN-16 (MAX98357A) | 3.2W, 4–8 Ohm, 5V | Future Audio/Voice Output |
| **C1, C4** | Bulk Filter Capacitors | Radial 8x12mm (Low ESR) | $470\mu\text{F}$, 25V / 10V | Motor & 5V rail buffer |
| **C2, C3** | Ceramic Filter Caps | 1206 SMD | $10\mu\text{F}$ / $22\mu\text{F}$, 25V | Buck & regulator stability |
| **C5-C12**| Decoupling Capacitors | 0603 SMD | $100\text{nF}$, 50V X7R | High-frequency noise shunts |
| **J2, J3** | Motor & Encoder Headers | 6-pin JST-XH 2.54mm | Right-Angle Through-Hole | N20 Motor + Encoder A/B |
| **J4** | RPLIDAR A1 Connector | 5-pin JST-XH 2.54mm | Right-Angle Through-Hole | LiDAR 5V, UART, MOTOCTRL |
| **J5** | I2C OLED Display Header | 4-pin 2.54mm Female | Through-Hole Header | 0.96"/1.3" OLED Display |
| **J6** | SPI Color TFT Header | 8-pin 2.54mm Female | Through-Hole Header | ST7789/ILI9341 Color LCD |
| **J7** | I2S Microphone Header | 6-pin 2.54mm Female | Through-Hole Header | INMP441 Digital Mic |
| **J8** | Speaker Terminal | 2-pin 3.5mm Screw Block | Through-Hole Terminal | 3W 4-Ohm Speaker Output |
| **J9** | ESP32-CAM Header | 16-pin Dual Row 2.54mm | Dual Through-Hole | Camera Daughterboard |
| **J10**| Qwiic / Stemma QT Port | 4-pin JST-SH 1.00mm | Surface Mount | External I2C IMU / ToF |

---

## 3. Recommended Multi-Layer PCB Stackup in FLUX

```
Layer 1 (Top Signal):  High-speed signals (I2S, SPI, UART, PWM) + Local ground flood
Layer 2 (Inner GND):    Solid unbroken Signal Ground plane (SGND)
Layer 3 (Inner Power):  Split power pours: VM (Motor Rail) and 5V_SYS / 3V3_SYS
Layer 4 (Bottom Power): High-current motor drive traces + Star ground tie point (PGND -> SGND)
```
