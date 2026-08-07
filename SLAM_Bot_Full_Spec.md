# SLAM Bot — Full Application Specification
### Firmware + ROS2 + Web App + AI-assisted SLAM

This is the single master spec. Feed this whole file to Claude Code as the build brief — every architectural decision below is final, not a suggestion to re-derive.

---

## 0. Assumptions made where your brief left room (stated up front so nothing is silently guessed)

- Web app stack: **React (Vite) frontend + FastAPI (Python) backend**, because the backend needs to run a ROS2 (`rclpy`) node, a WebSocket server for the robot, `arduino-cli` subprocess calls, and OpenRouter API calls — Python covers all of that natively alongside ROS2.
- Arduino Uno R4 WiFi connects to WiFi **directly** — no serial bridge to NodeMCU. NodeMCU is now LIDAR-only.
- "Train bot based on map and surroundings" is interpreted as: log traversed maps + collision events, let an OpenRouter model analyze logs and suggest tuning-parameter adjustments (not on-device reinforcement learning — a Cortex-M4-class MCU and a 7.4V bot budget can't support real RL training; an LLM-assisted parameter advisor is the realistic version of this feature).

---

## 1. Finalized Bill of Materials

| Component | Spec |
|---|---|
| Battery | 2S LiPo, 7.4V nominal / 8.4V full charge |
| Buck converter | LM2596 (adjustable) |
| Motor driver | DRV8833 (dual H-bridge) |
| Motors | 2x N20, 6V, with quadrature encoders |
| LIDAR | RPLIDAR A1M8 |
| Main controller | Arduino Uno R4 WiFi |
| Secondary controller | NodeMCU (ESP8266) — LIDAR relay only |

---

## 2. Power Architecture

```
                 ┌─────────────────────┐
   2S LiPo 7.4V──┤ Battery + / −        │
                 └────────┬────────────┘
                          │
             ┌────────────┼─────────────┐
             │                          │
        (raw 7.4–8.4V)            (into LM2596)
             │                          │
             ▼                          ▼
      DRV8833 VM pin            LM2596 trimmed to 5.0V
      (within 10.8V max,               │
       no regulation needed)   ┌───────┼────────┬─────────┬──────────┐
                                ▼       ▼        ▼         ▼          ▼
                           Arduino   NodeMCU   RPLIDAR   DRV8833   (spare)
                            5V pin    VIN/5V     5V        VCC
```

**Rules:**
- DRV8833 **VM** ← raw battery (7.4–8.4V). This is now within spec — no separate trim rail needed, unlike a 12V pack.
- DRV8833 **VCC** (logic) ← 5V regulated rail, same as everything else.
- Common ground across battery−, buck GND, both MCU GNDs, DRV8833 GND, RPLIDAR GND, both motor/encoder GNDs. One star point, not daisy-chained loosely.
- Trim the LM2596 pot and **verify 5.00V ± 0.1V with a multimeter before connecting any board.**
- N20 motors are 6V-rated but see up to 8.4V raw through the driver — this is handled in firmware via a PWM duty-cycle cap (§10), not extra hardware. Don't skip setting that cap. This cap is now a **live-tunable runtime value**, not a compiled constant — see §11.

---

## 3. Pin Connection Table

### Arduino Uno R4 WiFi

| Pin | Connects to | Purpose |
|---|---|---|
| D2 | Motor L encoder CH-A | interrupt |
| D3 | Motor L encoder CH-B | interrupt |
| D4 | Motor R encoder CH-A | interrupt |
| D5 | Motor R encoder CH-B | interrupt |
| D6 | DRV8833 AIN1 | PWM |
| D7 | DRV8833 AIN2 | PWM |
| D8 | DRV8833 BIN1 | PWM |
| D9 | DRV8833 BIN2 | PWM |
| D10 | DRV8833 nSLEEP | drive HIGH at boot |
| 5V | 5V regulated rail | power |
| GND | common ground | — |
| *(onboard WiFi)* | laptop WebSocket server | motion commands + odometry |

### NodeMCU (ESP8266) — LIDAR relay only

| Pin | Connects to | Purpose |
|---|---|---|
| GPIO3 (RX0) | RPLIDAR TX | hardware UART, 115200 baud |
| GPIO1 (TX0) | RPLIDAR RX | hardware UART |
| VIN / 5V | 5V regulated rail | power |
| GND | common ground | — |
| *(onboard WiFi)* | laptop WebSocket server | scan data out |

Note: NodeMCU's USB-serial debug output is unusable at runtime once RPLIDAR occupies the hardware UART — same caveat as before, debug over WiFi/logs page instead.

### DRV8833 → Motors

| DRV8833 pin | Connects to |
|---|---|
| OUT1 / OUT2 | Motor L terminals |
| OUT3 / OUT4 | Motor R terminals |
| VM | raw battery (7.4–8.4V) |
| VCC | 5V regulated rail |
| GND | common ground |

---

## 4. System Data Flow

```
RPLIDAR A1M8 ──UART──▶ NodeMCU ──WiFi/WebSocket──▶ FastAPI Backend ──▶ ROS2 (/scan)
                                                          │
                                                          ▼
                                                    slam_toolbox ──▶ /map, TF
                                                          │
                                                          ▼
                                                    Nav2 (planner + controller)
                                                          │
                                                          ▼
                                            FastAPI Backend ──WiFi/WebSocket──▶ Arduino Uno R4 WiFi ──▶ DRV8833 ──▶ Motors
                                                          ▲                            │
                                                          └────── encoder ticks ────────┘

                                     FastAPI Backend ──WebSocket──▶ React Web App
                                     (live scan, map, logs, odom, AI panel, tuning, flash)
```

Both MCUs connect **directly** to the FastAPI backend as WebSocket clients (`/ws/lidar` and `/ws/motion` endpoints). The backend is simultaneously a ROS2 node (`rclpy`), a WebSocket server for the robot, a WebSocket server for the browser frontend, and the process that shells out to `arduino-cli` and calls OpenRouter.

---

## 5. Repository Layout

```
slam_bot/
├── firmware/
│   ├── arduino_uno_r4/
│   │   └── arduino_uno_r4.ino
│   └── nodemcu_lidar/
│       └── nodemcu_lidar.ino
├── ros2_ws/src/
│   ├── slam_bot_bridge/         # bridge node, also embeds FastAPI (or run alongside)
│   ├── slam_bot_bringup/        # launch files, slam_toolbox + nav2 params
│   └── slam_bot_nav/            # nav2 config, costmaps, planner/controller params
├── backend/                     # FastAPI app
│   ├── main.py
│   ├── ws_robot.py              # WebSocket endpoints for NodeMCU + Arduino
│   ├── ws_frontend.py           # WebSocket endpoint for the browser
│   ├── flasher.py               # arduino-cli subprocess wrapper
│   ├── ai_advisor.py            # OpenRouter integration
│   └── ros_bridge.py            # rclpy node run in a background thread
└── webapp/                      # React + Vite frontend
    └── src/
        ├── pages/
        │   ├── Dashboard.jsx
        │   ├── LiveMap.jsx
        │   ├── Logs.jsx
        │   ├── FlashCenter.jsx
        │   ├── Tuning.jsx
        │   └── AIAssistant.jsx
        └── components/
```

---

## 6. Web App — Feature Spec

### 6.1 Dashboard
- Live connection status for NodeMCU and Arduino (green/red)
- Battery voltage readout (add a simple resistor-divider on an Arduino analog pin if you want this — otherwise omit, don't fake a reading)
- Current linear/angular velocity, live encoder tick rate
- **Start / Stop toggle** — this is the bot's run-state control, separate from e-stop (see §11.2): Stop means motors are disabled and ignore all `cmd_vel`/Nav2 output until Start is pressed again. Persists across a page refresh (state lives on the backend, not the browser tab).
- Emergency stop button (sends a hard-stop `cmd_vel` **and** flips the bot into Stopped state — same effect as Stop, but framed as the "something's wrong, halt now" action)

### 6.2 Live Map / Visualization
- Real-time polar plot of the current LIDAR scan (distance vs. angle) — this satisfies your "shows distance and angle" requirement directly
- Live occupancy grid map render from `/map` (canvas or WebGL grid)
- Robot pose marker + trail
- Rendered fully in-browser from WebSocket data — do **not** try to embed rviz2 itself in a browser; render the same data with a lightweight canvas renderer instead

### 6.3 Logs & Debug
- Live-tailing log stream (this is also where NodeMCU's "no USB serial" debug problem gets solved — everything logs here over WiFi instead)
- Filterable by source: `lidar`, `motion`, `slam`, `nav`, `ai`, `system`
- Log severity levels (info/warn/error), most recent at top

### 6.4 Flash Center
- **NodeMCU:** true browser-based flashing via WebSerial (`esptool-js`). User plugs in via USB, clicks Connect, selects the compiled `.bin`, clicks Flash. No backend involved for this one — runs entirely in-browser.
- **Arduino Uno R4:** "Flash" button sends a request to the FastAPI backend, which runs `arduino-cli compile` + `arduino-cli upload` against the connected serial port and streams the console output back to the page live. This requires the backend to run on the same machine the Arduino is physically plugged into — call this out clearly in the UI ("Arduino must be connected to this server, not to your browsing device, if they differ").
- Show compile/upload progress and errors in both cases.

### 6.5 Tuning Panel
See §10 for the full parameter list, §11 for how a change actually reaches the hardware. Each parameter: slider or numeric input, live-applies via a `/tuning/update` WebSocket message to the backend, which pushes it down to the relevant MCU or ROS2 param server **over WiFi, immediately, with no reflash and no reboot**. Include a "Save as default" (persists to the MCU's flash so the value survives a power cycle) and "Reset to default" per section.

### 6.6 AI Assistant
- Chat-style panel backed by OpenRouter (see §10)
- Button: "Analyze last session" — sends recent map + log + collision-event summary to the model, gets back plain-language tuning suggestions the user can apply with one click (writes to the Tuning Panel's values, doesn't auto-apply without confirmation)
- Button: "Explain this map" — describes the generated occupancy grid in plain language (room count guess, open areas, choke points)

---

## 7. ROS2 Nodes & Topics

| Node | Publishes | Subscribes |
|---|---|---|
| `slam_bot_bridge` | `/scan` (LaserScan), `/odom` (Odometry), `/tf` | `/cmd_vel` |
| `slam_toolbox` (async) | `/map`, `/tf` (map→odom) | `/scan` |
| `nav2` stack (planner_server, controller_server, bt_navigator, costmap nodes) | `/plan`, `/local_costmap`, `/global_costmap` | `/map`, `/scan`, `/odom`, goal poses |
| `slam_bot_web_relay` | — (forwards ROS topics to frontend WebSocket) | `/scan`, `/map`, `/odom`, `/plan` |

Static TF: `base_link → laser` (set lidar mount offset as a launch param, default 0,0,0 unless you measure the actual offset).

---

## 8. Algorithms Reference

| Purpose | Algorithm | Where it runs |
|---|---|---|
| SLAM (map building + localization) | `slam_toolbox` (pose-graph based, online async mode) | ROS2, laptop |
| Global path planning | A* (Nav2's `NavFn` or `SmacPlanner2D`, both A*-family) | Nav2 planner_server |
| Local obstacle avoidance / trajectory following | DWB (Dynamic Window Approach) controller | Nav2 controller_server |
| Alternative local planner (optional, more aggressive) | TEB (Timed Elastic Band) | Nav2 controller_server (swap-in) |
| Alternative global planner (optional, for tight spaces) | RRT* or D* Lite | would require a custom Nav2 plugin — flag as a stretch goal, not v1 |
| Localization within known map | AMCL (particle filter) | Nav2, optional once you have a saved map and want relocalization instead of continuous SLAM |
| Collision protection | Costmap inflation layer + Nav2's built-in velocity smoother / collision monitor | Nav2 |

Start with `slam_toolbox` + Nav2 default `SmacPlanner2D` (A*) + DWB — this combination is the standard, well-documented ROS2 path and is what Claude Code should implement first. TEB/RRT*/D* Lite are documented here as the upgrade path, not v1 scope.

---

## 9. AI Integration (OpenRouter)

**Free-tier model recommendation:** at build time, query `https://openrouter.ai/api/v1/models` and filter for `pricing.prompt == "0"` — free-tier model availability rotates, so don't hardcode a model ID into firmware/backend logic; make it a config value the backend reads at startup, with a fallback list of 2–3 candidates in case the first is unavailable.

**Concrete AI features (not vague "processing"):**
1. **Session analysis** — after a mapping run, send the backend's log summary (collision count, stuck events, map coverage %) as text to the model, ask for tuning suggestions in structured JSON (parameter name → suggested value → reasoning). Never send raw sensor data streams to the API — summarize first, both for cost and because raw scan data isn't meaningfully interpretable by a text LLM anyway.
2. **Map description** — send a text-encoded summary of the occupancy grid (dimensions, open/closed area ratios, detected room-like enclosed regions from simple flood-fill) and ask for a plain-language description.
3. **Chat control** (optional, flag as stretch goal) — natural language like "explore the room on the left" parsed into a Nav2 goal pose. This needs a deterministic parser with the LLM only used for intent extraction, not for generating raw motor commands — never let an LLM output go straight to motors without a bounds-checked intermediate step.

```python
# backend/ai_advisor.py — sample call pattern
import httpx

async def get_tuning_advice(summary: dict, api_key: str, model: str):
    resp = await httpx.AsyncClient().post(
        "https://openrouter.ai/api/v1/chat/completions",
        headers={"Authorization": f"Bearer {api_key}"},
        json={
            "model": model,  # from resolved free-tier config, not hardcoded
            "messages": [
                {"role": "system", "content": "You are a robotics tuning advisor. Respond only with JSON: {parameter, suggested_value, reasoning}[]."},
                {"role": "user", "content": str(summary)}
            ]
        }
    )
    return resp.json()
```

---

## 10. Tuning Parameters Exposed in Web UI

| Parameter | Affects | Default |
|---|---|---|
| `max_pwm_duty` | motor speed cap (protects 6V motors from 8.4V pack) | 200/255 (~78%) |
| `max_linear_speed_mm_s` | Nav2 velocity limit | 300 |
| `max_angular_speed_mdeg_s` | Nav2 velocity limit | 60000 (60°/s) |
| `lidar_min_range_mm` / `lidar_max_range_mm` | scan filtering, ignore noise below/above these | 150 / 6000 |
| `lidar_angle_filter` | mask out angles blocked by robot's own chassis, if any | none by default |
| `obstacle_inflation_radius_mm` | Nav2 costmap safety margin | 150 |
| `encoder_cpr` | odometry accuracy | measure and set — no safe default |
| `wheel_diameter_mm` | odometry accuracy | measure and set |
| `wheel_base_mm` | odometry accuracy | measure and set |
| `collision_stop_distance_mm` | emergency stop trigger distance | 100 |

---

## 11. Runtime Control & Live Tuning Protocol (no reflash required)

This is the piece that makes tuning and Start/Stop work over WiFi instead of needing a firmware rebuild for every adjustment.

### 11.1 Firmware-side rule
Every value in the §10 table must exist on the MCU as a **runtime variable in a config struct**, never as a compiled `#define`/`const` baked into the binary. Firmware initializes that struct from sane defaults at boot, then overwrites individual fields whenever a `tuning_update` message arrives over WebSocket — takes effect on the very next control loop iteration, no reboot.

```cpp
// Arduino — example runtime config struct (NodeMCU has its own equivalent for lidar_min/max_range etc.)
struct RuntimeConfig {
  int   max_pwm_duty       = 200;
  int   max_linear_speed   = 300;   // mm/s
  int   max_angular_speed  = 60000; // millideg/s
  int   collision_stop_mm  = 100;
  bool  running            = false; // Start/Stop state — see 11.2
} config;
```

### 11.2 Start/Stop as a firmware-level gate, not just a UI state
`config.running` must be checked in firmware itself, not only enforced by the backend withholding `cmd_vel`. Reasoning: if the WiFi link drops mid-session, you want the robot's own firmware to already be in a safe default (motors off) rather than depending on a command that may never arrive. Concretely:
- Boot default: `running = false` — the bot does not move on power-up until the web app explicitly starts it.
- `{"type":"control","action":"start"}` → sets `running = true`.
- `{"type":"control","action":"stop"}` → sets `running = false` **and** immediately zeroes both motor PWM outputs, regardless of what `cmd_vel` says.
- If no message (of any type) is received from the backend for >2 seconds, firmware treats the link as lost and forces `running = false` — a watchdog, not just a UI convenience.

### 11.3 WebSocket message schemas (add to the ones already defined for `/ws/motion` and `/ws/lidar`)

**Backend → Arduino, tuning update:**
```json
{"type":"tuning_update","max_pwm_duty":180,"max_linear_speed":250}
```
Only include the fields being changed — firmware patches the matching struct fields and leaves the rest untouched.

**Backend → Arduino, control:**
```json
{"type":"control","action":"start"}
```
or `"action":"stop"`.

**Backend → NodeMCU, tuning update** (separate message, same pattern, for `lidar_min_range_mm`/`lidar_max_range_mm`/`lidar_angle_filter`):
```json
{"type":"tuning_update","lidar_min_range_mm":150,"lidar_max_range_mm":6000}
```

### 11.4 Persistence across power cycles
"Save as default" in the Tuning Panel triggers a `{"type":"tuning_save"}` message — firmware writes the current `RuntimeConfig` struct to onboard flash (`EEPROM.put()` on the Arduino Uno R4, `LittleFS` on the ESP8266) and loads from there at boot instead of the hardcoded defaults if a saved config exists. Without this step, tuning changes are live but reset to firmware defaults on next power-up — make sure that distinction is visible in the UI (e.g. an "unsaved changes" indicator).

### 11.5 What still requires an actual reflash
Pin assignments, wiring-dependent constants (which physical pin is which encoder channel), and communication protocol/message schema changes themselves. Everything in the §10 tuning table does not.

---

## 12. Build & Run Order

1. Wire per §2–3, verify 5.0V rail with multimeter before connecting boards.
2. Flash NodeMCU and Arduino once via USB + Arduino IDE the traditional way first, to confirm firmware works, **before** relying on the web app's Flash Center.
3. Bring up `backend/` (`uvicorn main:app`), confirm both MCUs connect (Dashboard shows green).
4. Bring up `ros2_ws` bringup launch (bridge + slam_toolbox + nav2).
5. Bring up `webapp/` (`npm run dev`), confirm Live Map shows real scan data.
6. Confirm Start/Stop actually gates motion: press Start, verify wheels can move; press Stop, verify wheels immediately halt and stay halted even if a `cmd_vel` is still being sent. Then pull WiFi power to the robot mid-drive and confirm the 2-second watchdog (§11.2) forces a stop on its own.
7. Try a couple of tuning-panel changes (e.g. `max_linear_speed`) and confirm they take effect immediately with no reflash, then confirm "Save as default" survives a power cycle.
8. Drive manually via Dashboard controls — confirm motion + collision-stop work — before trusting Nav2 autonomous goals.
9. Only then start relying on Flash Center for iterative firmware updates (this should now be rare — most adjustments go through the Tuning Panel instead).

---

## 13. Roadmap / Stretch Features (explicitly out of v1 scope)

- RRT*/D* Lite custom Nav2 planner plugin
- Chat-based natural-language goal setting
- Multi-session map merging
- Battery-voltage-aware auto-return-to-charge behavior
- Semantic object labeling on the map via a vision model (would need a camera you haven't listed in the BOM)
