# SLAM Bot

Firmware + ROS2 + web app + AI-assisted SLAM for a 2-wheel differential-drive
robot. Built to [`SLAM_Bot_Full_Spec.md`](SLAM_Bot_Full_Spec.md) — that file is
the authority on every design decision; this one covers how to actually run it.

```
RPLIDAR A1M8 ──UART──▶ NodeMCU ──WiFi──▶ FastAPI ──▶ ROS2 /scan ──▶ slam_toolbox ──▶ /map
                                            │                             │
                                            │                             ▼
                                            │                       Nav2 (A* + DWB)
                                            │                             │
                                            ▼                             ▼
                                     React web app ◀── WebSocket ──  FastAPI ──WiFi──▶ Arduino Uno R4 ──▶ DRV8833 ──▶ motors
                                                                                              │
                                                                        ◀───── encoder odometry ┘
```

---

## What's here

| Path | What it is |
|---|---|
| `firmware/arduino_uno_r4/` | Motion controller: PID wheel velocity, odometry, runtime config, 2 s watchdog, EEPROM persistence |
| `firmware/nodemcu_lidar/` | RPLIDAR A1M8 packet parser + WiFi relay, LittleFS persistence |
| `backend/` | FastAPI: MCU WebSocket server, browser WebSocket server, ROS2 node, arduino-cli wrapper, OpenRouter client |
| `ros2_ws/src/slam_bot_bridge/` | Standalone bridge node + web relay (optional — the backend embeds the same logic) |
| `ros2_ws/src/slam_bot_bringup/` | Launch files + slam_toolbox params |
| `ros2_ws/src/slam_bot_nav/` | Nav2 params: SmacPlanner2D (A*) + DWB, scaled for this robot |
| `webapp/` | React + Vite frontend, six pages |

---

## Deviation from the spec's pin table — read before wiring

**§3's Arduino pin assignment cannot work as written, and this build corrects it.**

On the UNO R4 WiFi, hardware PWM exists only on **D3, D5, D6, D9, D10, D11**.
The spec assigns DRV8833 `AIN2`→D7 and `BIN1`→D8, which are digital-only pins.
A DRV8833 has no separate enable input, so in IN/IN mode the *reverse* direction
is speed-controlled by PWM-ing the second input. With the spec's mapping each
motor would have had proportional speed forward and full-speed-only in reverse.

The default map moves the two affected motor lines and relocates `nSLEEP` (which
needs no PWM) onto D8:

| Signal | **Used here** | Spec §3 |
|---|---|---|
| Encoder L CH-A (interrupt) | D2 | D2 |
| Encoder L CH-B (read in ISR) | D4 | D3 |
| Encoder R CH-A (interrupt) | D3 | D4 |
| Encoder R CH-B (read in ISR) | D5 | D5 |
| DRV8833 AIN1 (PWM) | D6 | D6 |
| DRV8833 AIN2 (PWM) | **D11** | D7 — not PWM |
| DRV8833 BIN1 (PWM) | **D10** | D8 — not PWM |
| DRV8833 BIN2 (PWM) | D9 | D9 |
| DRV8833 nSLEEP | **D8** | D10 |

Encoder CH-B is sampled inside the ISR rather than interrupting, so only D2/D3
carry interrupts. If your harness is already crimped to the literal §3 table,
uncomment `#define USE_SPEC_PINMAP` in the sketch — motion still works, but
reverse becomes bang-bang.

Everything else in §2–3 (power tree, DRV8833 VM on raw battery, common star
ground, the 5.00 V rail check) is implemented as specified.

---

## Power: one battery, one buck converter

```
2S LiPo 7.4 V ──┬── raw ──────────────────▶ DRV8833 VM  (motors, ~1.5 A stall)
                │
                └── LM2596 (trim to 5.00 V) ──▶ breadboard 5 V rail
                                                  ├─▶ Arduino Uno R4  (5V pin)
                                                  ├─▶ NodeMCU         (VIN)
                                                  ├─▶ RPLIDAR A1M8    (5V)
                                                  ├─▶ DRV8833 VCC     (logic only)
                                                  └─▶ both encoders
```

**The battery + wire splits in two.** One branch feeds the buck converter for
all logic; the other goes straight to `DRV8833 VM` for motor power.

**Never put the motors on the 5 V rail.** Two N20s pull ~1.5 A at stall and a
starting motor is briefly a near short. On a shared rail that collapses the
supply every time you accelerate — both MCUs reboot mid-drive and the LIDAR
loses UART sync. `VM` (motor power) and `VCC` (logic power) are two different
DRV8833 pins doing two different jobs.

Rail budget: RPLIDAR ~450 mA + NodeMCU ~80 mA + Arduino ~50 mA + encoders and
driver logic ~40 mA ≈ **620 mA**. An LM2596 is rated 2–3 A, so that is
comfortable — but **fit a heatsink**, since ~1.5 W in an enclosed chassis is
enough to make it thermally throttle. A 470–1000 µF electrolytic across the 5 V
rail absorbs the LIDAR's spin-up surge.

**The LIDAR connects to the NodeMCU only, never to both boards.** A UART is
point-to-point; two transmitters on one line fight each other. The Uno R4 has no
spare hardware UART either — D0/D1 are the USB serial port. The NodeMCU is in
this build specifically to be the LIDAR's dedicated UART host, forwarding scans
over WiFi so the Arduino still gets obstacle data via the backend.

---

## Setup

### 1. Wire it, and verify the rail first

Follow §2–3 of the spec. **Trim the LM2596 and confirm 5.00 V ±0.1 V with a
multimeter before connecting any board.** Common ground at one star point.

### 2. Firmware credentials

Both sketches need a `secrets.h` beside them. Templates are already in place —
edit them:

```c
// firmware/arduino_uno_r4/secrets.h  and  firmware/nodemcu_lidar/secrets.h
#define WIFI_SSID     "your-wifi"
#define WIFI_PASS     "your-password"
#define BACKEND_HOST  "192.168.1.50"   // LAN IP of the machine running the backend
#define BACKEND_PORT  8000
```

`BACKEND_HOST` must be the backend machine's **LAN IP**, not `localhost` —
from the MCU's perspective localhost is the MCU itself.

**Install these before compiling** (Arduino IDE → Tools → Manage Libraries).
A missing one fails as `fatal error: <header>: No such file or directory`:

| Board | Library | Author | Provides |
|---|---|---|---|
| Uno R4 | `ArduinoHttpClient` | Arduino | `WebSocketClient.h` |
| Uno R4 | `ArduinoJson` v7.x | Benoit Blanchon | `ArduinoJson.h` |
| NodeMCU | `WebSockets` | **Markus Sattler** | `WebSocketsClient.h` |
| NodeMCU | `ArduinoJson` v7.x | Benoit Blanchon | `ArduinoJson.h` |

Note the two WebSocket libraries are different and not interchangeable — the
Uno R4 header is `WebSocketClient.h` (singular), the ESP8266 one is
`WebSocketsClient.h` (plural). Searching "WebSockets" returns several
similarly named libraries; the ESP8266 needs Markus Sattler's.

`ESP8266WiFi` and `LittleFS` ship with the ESP8266 board package (Boards
Manager → `esp8266` by ESP8266 Community); `WiFiS3` and `EEPROM` ship with the
UNO R4 package.

Per §12 step 2, **flash both boards over USB the traditional way first** to
confirm the firmware works, before relying on the Flash Center.

### 3. Backend

```bash
cd backend
python -m venv ../.venv
../.venv/bin/pip install -r requirements.txt      # Windows: ..\.venv\Scripts\pip
cp ../.env.example ../.env                        # then edit
uvicorn main:app --host 0.0.0.0 --port 8000
```

`--host 0.0.0.0` matters: the MCUs connect *to* this server over WiFi, so it
must be reachable on the LAN.

`rclpy` is deliberately not in `requirements.txt` — it isn't pip-installable and
comes from your ROS2 distribution. **The backend runs fine without ROS2**: the
Dashboard, polar LIDAR plot, Logs, Flash Center and Tuning Panel all work. Only
the occupancy grid and Nav2 goals need it.

### 4. Web app

```bash
cd webapp
npm install
npm run dev      # dev server on :5173, proxies /api and /ws to :8000
```

Or build once and let the backend serve it from a single origin:

```bash
npm run build    # backend then serves webapp/dist at http://<host>:8000/
```

### 5. ROS2 (optional but needed for mapping and autonomy)

Requires ROS2 Humble or newer with `slam_toolbox`, `nav2_bringup` and
`nav2_smac_planner`. **Linux only** — ROS2 on Windows won't give you the Nav2
stack this config expects; run the backend and ROS2 on a Linux machine (or WSL2
with the robot on the same network).

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch slam_bot_bringup bringup.launch.py
```

Useful arguments:

```bash
# mapping only, no autonomous navigation
ros2 launch slam_bot_bringup bringup.launch.py use_nav2:=false

# measured lidar mount offset (§7 — default is 0,0,0)
ros2 launch slam_bot_bringup bringup.launch.py laser_offset_x:=0.04 laser_offset_z:=0.06

# run the bridge as its own process instead of inside the backend
ros2 launch slam_bot_bringup bringup.launch.py standalone_bridge:=true
```

> Run the embedded bridge **or** the standalone one, never both — they would each
> publish `/scan`. The default (`standalone_bridge:=false`) uses the backend's
> embedded node, which is what §4 describes.

---

## Bring-up order (§12)

1. Wire per §2–3; **verify the 5.0 V rail with a multimeter** before connecting boards.
2. Flash both MCUs over USB via the Arduino IDE to confirm the firmware works.
3. Start the backend; confirm both dots go green on the Dashboard.
4. Start the ROS2 bringup launch.
5. Start the web app; confirm the Live Map shows real scan data.
6. **Verify the safety gates.** Press Start, confirm the wheels can move. Press
   Stop, confirm they halt immediately and stay halted. Then pull WiFi power to
   the robot mid-drive and confirm the 2-second watchdog stops it on its own.
7. Change a tuning value (e.g. `max_linear_speed_mm_s`) and confirm it takes
   effect immediately with no reflash. Then "Save as default" and power-cycle to
   confirm it persisted.
8. Drive manually from the Dashboard; confirm motion and collision-stop work
   *before* trusting Nav2 autonomous goals.
9. Only then use the Flash Center for iterative firmware updates — which should
   now be rare, since nearly everything is tunable live.

---

## Measure these three values

§10 marks three parameters "measure and set — no safe default", and the Tuning
Panel flags them with a **measure** badge. The shipped numbers are placeholders;
odometry will drift until you replace them:

| Parameter | How to measure |
|---|---|
| `encoder_cpr` | Mark a wheel, roll it exactly 10 turns by hand, read the tick delta from the Dashboard, divide by 10. Includes gearbox ratio and this firmware's 2× decode. |
| `wheel_diameter_mm` | Calipers, under the robot's own weight — tyres compress. |
| `wheel_base_mm` | Centre-to-centre between the wheels' contact patches. Errors here show up as rotational drift. |

---

## How live tuning works (§11)

Every §10 parameter lives in a **runtime struct** on the MCU, never a compiled
`#define`. Moving a slider sends a partial `tuning_update` over WebSocket; the
firmware patches the matching fields and the next control-loop iteration uses
them. No reflash, no reboot.

- Unsaved changes show an **unsaved** badge and revert on power-cycle.
- **Save as default** writes to EEPROM (Arduino) / LittleFS (NodeMCU).
- If a device is offline when you change a value, the backend holds it and pushes
  it automatically when that device reconnects.
- Parameters are declared once in `backend/tuning.py` and the UI renders itself
  from that schema, so the two can't drift apart.

**Still needs a reflash** (§11.5): pin assignments, wiring-dependent constants,
and changes to the message schema itself.

### Safety model

Three independent layers, deliberately not relying on each other:

1. **Firmware run-state gate.** `running` boots `false`; motors stay dead until
   the web app explicitly starts them.
2. **Firmware watchdog.** No backend message for >2 s → firmware forces
   `running = false` and zeroes both PWM outputs on its own. Pull the WiFi and
   the robot stops without being told to.
3. **Firmware collision stop.** The backend forwards each revolution's nearest
   LIDAR return; the Arduino blocks forward motion below
   `collision_stop_distance_mm`. Reverse stays available so you can back out of a
   wall. Nav2's collision monitor and costmap inflation are additional layers,
   not the primary one.

The browser is never a safety layer. Manual drive commands expire after 500 ms,
so a frozen tab cannot leave the robot driving.

---

## AI features (§9)

Needs `OPENROUTER_API_KEY`. Everything else works without it.

- **Analyze last session** — sends a *summary* (collision count, stalls, map
  coverage, distance, current tuning), never raw sensor streams. Returns
  structured suggestions you apply with one click.
- **Explain this map** — room counting and area ratios are computed locally by
  flood-fill; the model only puts words to numbers the backend calculated.
- **Chat** — questions about telemetry, SLAM behaviour, wiring, tuning.

Two guardrails worth knowing: the model id is **never hardcoded** (§9 — the
backend queries `/models` at startup, keeps only zero-priced entries, and falls
through a candidate list, because free-tier availability rotates); and every
suggestion is validated against the §10 registry and clamped to range before the
UI can apply it. Rejected suggestions are shown rather than hidden.

Natural-language goal setting is deliberately **out of scope** (§9.3, §13): an
LLM's output must never reach the motors without a bounds-checked intermediate
step.

---

## Verification

```bash
python backend/smoke_test.py
```

74 checks, no hardware required. Covers the tuning registry's clamping and
routing, RLE round-tripping, LaserScan binning, flood-fill room detection, and
AI-suggestion validation — then drives the real WebSocket endpoints with a
simulated Arduino and NodeMCU to confirm the §11 protocol end to end: boot-stopped
default, Start/Stop gating, partial `tuning_update` patches, limits applying with
no reflash, e-stop, and LIDAR→Arduino proximity forwarding.

---

## Troubleshooting

**MCU won't connect.** `BACKEND_HOST` must be the backend's LAN IP. Start uvicorn
with `--host 0.0.0.0`. Check the host firewall allows inbound :8000. Both the
robot and the backend must be on the same network — most consumer routers isolate
a 2.4 GHz guest SSID from the main one, and the ESP8266 is 2.4 GHz only.

**No LIDAR data.** The NodeMCU's serial console is unusable at runtime by design
— the RPLIDAR occupies the hardware UART (§3). Its diagnostics appear on the
**Logs** page instead. If scans stop entirely the firmware re-issues the SCAN
command after 3 s, which usually means a brown-out on the shared 5 V rail.

**Motors buzz but don't turn.** `max_pwm_duty` too low to overcome stiction, or
the DRV8833's `nSLEEP` isn't high. Check it's on D8 (or D10 with the spec pinmap).

**Robot drives in an arc when commanded straight.** Encoder direction inverted on
one side — swap that motor's CH-A/CH-B, or flip the sign in that ISR.

**Map drifts badly when turning.** `wheel_base_mm` is wrong. Measure it.

**Bot won't start.** It refuses if the Arduino isn't connected. If an e-stop is
latched, the Dashboard says so — press Start to clear it.

**`arduino-cli not found`.** Install it, then
`arduino-cli core install arduino:renesas_uno` (and `esp8266:esp8266` for the
NodeMCU). Set `SLAM_ARDUINO_CLI` to the full path if it isn't on PATH.

**NodeMCU flashing button does nothing.** WebSerial only exists in Chrome, Edge
and Opera on desktop. Firefox, Safari and all iOS browsers don't implement it.

**No `/map`.** Needs ROS2 + slam_toolbox. Check `ros2 topic hz /scan` — if that's
empty the backend isn't publishing, so the bridge isn't up or the NodeMCU is
offline.

---

## Out of scope for v1 (§13)

RRT*/D* Lite custom planner plugin · natural-language goal setting ·
multi-session map merging · battery-aware auto-return-to-charge · semantic object
labelling (needs a camera, not in the BOM).
