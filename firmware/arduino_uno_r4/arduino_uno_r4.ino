// ===========================================================================
// SLAM Bot — Arduino Uno R4 WiFi main controller firmware
//
// Responsibilities
//   * Connect to WiFi and hold a WebSocket session to the FastAPI backend
//     (BACKEND_PATH = /ws/motion).
//   * Drive two N20 motors through a DRV8833 in IN/IN (sign-magnitude) mode.
//   * Read both quadrature encoders, integrate differential-drive odometry,
//     and stream it back to the backend at ODOM_HZ.
//   * Hold every §10 tuning parameter in a RUNTIME struct (never a #define)
//     so the web app can retune over WiFi with no reflash — see §11.1.
//   * Enforce Start/Stop in firmware (§11.2), including a 2 s link watchdog
//     that forces Stop on its own if the backend goes quiet.
//   * Persist / restore the config to onboard EEPROM on "Save as default"
//     (§11.4).
//
// Libraries required (Arduino IDE → Library Manager):
//   * "ArduinoHttpClient"  (provides WebSocketClient)
//   * "ArduinoJson"        v7.x
//   WiFiS3 + EEPROM ship with the Arduino UNO R4 board package.
//
// ---------------------------------------------------------------------------
// PIN MAP NOTE — READ THIS BEFORE WIRING
// ---------------------------------------------------------------------------
// The master spec's §3 table assigns DRV8833 AIN2→D7 and BIN1→D8. On the UNO
// R4 WiFi, hardware PWM exists only on D3, D5, D6, D9, D10, D11 — D7 and D8
// are digital-only. A DRV8833 has no separate ENABLE input, so in IN/IN mode
// the *reverse* direction is speed-controlled by PWM-ing the second input. Had
// AIN2/BIN1 stayed on D7/D8, each motor would have had proportional speed in
// one direction and full-speed-only in the other.
//
// The default map below therefore moves two motor lines and relocates nSLEEP
// (which needs no PWM) onto D8. Encoder CH-B lines are read inside the ISR
// rather than interrupting, so only D2/D3 — the two pins guaranteed
// interrupt-capable on this board — carry interrupts.
//
//   Signal            Default (used)   Spec §3 value
//   ----------------  ---------------   -------------
//   Enc L CH-A  IRQ   D2               D2
//   Enc L CH-B  read  D4               D3
//   Enc R CH-A  IRQ   D3               D4
//   Enc R CH-B  read  D5               D5
//   DRV8833 AIN1 PWM  D6               D6
//   DRV8833 AIN2 PWM  D11              D7   <-- not a PWM pin
//   DRV8833 BIN1 PWM  D10              D8   <-- not a PWM pin
//   DRV8833 BIN2 PWM  D9               D9
//   DRV8833 nSLEEP    D8               D10
//
// If your harness is already crimped to the literal §3 table, uncomment the
// line below — motion still works, but reverse becomes bang-bang.
// #define USE_SPEC_PINMAP
// ===========================================================================

#include <WiFiS3.h>
#include <ArduinoHttpClient.h>
#include <ArduinoJson.h>
#include <EEPROM.h>
#include "secrets.h"

// ---------------------------------------------------------------------------
// Pin assignments  (§11.5: these are the things that DO require a reflash)
// ---------------------------------------------------------------------------
#ifdef USE_SPEC_PINMAP
  #define PIN_ENC_L_A   2
  #define PIN_ENC_L_B   3
  #define PIN_ENC_R_A   4
  #define PIN_ENC_R_B   5
  #define PIN_AIN1      6
  #define PIN_AIN2      7
  #define PIN_BIN1      8
  #define PIN_BIN2      9
  #define PIN_NSLEEP   10
#else
  #define PIN_ENC_L_A   2
  #define PIN_ENC_L_B   4
  #define PIN_ENC_R_A   3
  #define PIN_ENC_R_B   5
  #define PIN_AIN1      6
  #define PIN_AIN2     11
  #define PIN_BIN1     10
  #define PIN_BIN2      9
  #define PIN_NSLEEP    8
#endif

// Optional battery sense: resistor divider from raw pack into A0.
// Leave BATTERY_SENSE_ENABLED at 0 unless you actually fit the divider —
// §6.1 is explicit that a missing reading must be omitted, never faked.
#define BATTERY_SENSE_ENABLED  0
#define PIN_VBAT              A0
#define VBAT_DIVIDER_RATIO    3.0f    // (R1+R2)/R2 for your fitted divider
#define ADC_REF_VOLTS         5.0f
#define ADC_MAX_COUNTS        1023.0f

// ---------------------------------------------------------------------------
// Loop rates
// ---------------------------------------------------------------------------
static const uint32_t CONTROL_PERIOD_MS = 20;    //  50 Hz motor control loop
static const uint32_t ODOM_PERIOD_MS    = 50;    //  20 Hz odometry telemetry
static const uint32_t WATCHDOG_MS       = 2000;  // §11.2 link-loss timeout
static const uint32_t RECONNECT_MS      = 2000;

// ---------------------------------------------------------------------------
// §11.1 Runtime config — every §10 parameter lives here, patchable over WiFi
// ---------------------------------------------------------------------------
static const uint32_t CONFIG_MAGIC   = 0x53424D31UL;  // "SBM1"
// Bumped 1 -> 2 when the motion-calibration block below was added. A stored
// v1 struct has a different layout, so loadConfigFromEeprom() will reject it
// and fall back to these defaults rather than reading garbage into the new
// fields. Re-save from the Tuning page after a firmware update.
static const uint16_t CONFIG_VERSION = 2;
static const int      EEPROM_ADDR    = 0;

// Per-wheel state for checkWheelSignSanity(). Declared up here because the
// IDE's auto-generated prototypes are emitted above the first function, and a
// struct used in a signature must already be a complete type by then.
struct WheelSignCheck {
  uint32_t badSinceMs = 0;
  bool     reported   = false;
};

void logToBackend(const char* level, const String& msg);   // defined below

struct RuntimeConfig {
  uint32_t magic   = CONFIG_MAGIC;
  uint16_t version = CONFIG_VERSION;

  // --- motion limits -------------------------------------------------------
  int   max_pwm_duty          = 200;    // 0..255  protects 6 V motors off 8.4 V
  int   max_linear_speed      = 300;    // mm/s
  int   max_angular_speed     = 60000;  // millideg/s  (60 deg/s)
  int   collision_stop_mm     = 100;    // hard-stop trigger distance

  // --- odometry geometry (§10: measure these, no safe default) ------------
  float encoder_cpr           = 700.0f; // counts per output-shaft revolution
  float wheel_diameter_mm     = 43.0f;
  float wheel_base_mm         = 150.0f;

  // --- closed-loop wheel velocity gains -----------------------------------
  float pid_kp                = 0.60f;
  float pid_ki                = 2.50f;
  float pid_kd                = 0.00f;

  // --- motion calibration (validated on the bench with movement_test) ------
  // These exist because the PID loop cannot fix everything on its own:
  //
  // invert_*  A motor whose leads are handed spins the wrong way. Correcting
  //           it in firmware beats re-soldering, and it MUST be applied after
  //           the PID output so the encoder sign still matches the wheel.
  // trim_*    Two nominally identical N20s differ by 5-15% for the same duty.
  //           The PID closes that gap once it is moving, but trim removes the
  //           bias up front so the integrator does not have to fight it.
  // turn_boost / kick_duty / kick_ms
  //           Static friction is much higher than rolling friction. From rest
  //           the wheels can sit below breakaway torque and never start, which
  //           looks exactly like a dead control. A brief shove fixes it. Turns
  //           in place need the most, because both tyres scrub sideways.
  bool  invert_left           = false;
  bool  invert_right          = true;   // bench-confirmed on this chassis
  float trim_left             = 1.00f;  // 0.5..1.0 multiplier, only ever down
  float trim_right            = 1.00f;
  float turn_boost            = 1.60f;  // extra command for turn-in-place
  int   kick_duty             = 110;    // duty floor while breaking stiction
  int   kick_ms               = 90;     // how long that floor is held

  // --- §11.2 run state ----------------------------------------------------
  bool  running               = false;  // boot default: DO NOT move
};

RuntimeConfig config;

// `running` is deliberately never restored from EEPROM even when a saved
// config exists — the bot must always power up stopped (§11.2).
void loadConfigFromEeprom() {
  RuntimeConfig stored;
  EEPROM.get(EEPROM_ADDR, stored);
  if (stored.magic == CONFIG_MAGIC && stored.version == CONFIG_VERSION) {
    stored.running = false;
    config = stored;
  }
}

void saveConfigToEeprom() {
  RuntimeConfig out = config;
  out.magic   = CONFIG_MAGIC;
  out.version = CONFIG_VERSION;
  out.running = false;
  EEPROM.put(EEPROM_ADDR, out);
}

void clearSavedConfig() {
  RuntimeConfig blank;
  blank.magic = 0;
  EEPROM.put(EEPROM_ADDR, blank);
}

// ---------------------------------------------------------------------------
// Encoder state — 2x decode: interrupt on CH-A, CH-B sampled inside the ISR
// ---------------------------------------------------------------------------
volatile long encLeftTicks  = 0;
volatile long encRightTicks = 0;

void isrLeft() {
  // Rising or falling edge on A; B's level tells us the direction.
  bool a = digitalRead(PIN_ENC_L_A);
  bool b = digitalRead(PIN_ENC_L_B);
  encLeftTicks += (a == b) ? 1 : -1;
}

void isrRight() {
  // Same sign convention as the left wheel: ticks count UP when the wheel
  // rolls the robot forward.
  //
  // This used to be negated "because the right motor faces the opposite way".
  // That was wrong, and it made the robot spin instead of drive: the reversed
  // motor leads were already being corrected by config.invert_right on the
  // duty, so negating here corrected the same physical fact a second time.
  // The result was inverted feedback on the right wheel's velocity PID —
  // driving harder made the measured error grow, so it saturated at full
  // reverse while the left wheel drove forward. Reversed leads are an OUTPUT
  // concern (invert_right); encoder sign is an INPUT concern and must stay
  // positive-when-forward or odometry integrates backwards.
  bool a = digitalRead(PIN_ENC_R_A);
  bool b = digitalRead(PIN_ENC_R_B);
  encRightTicks += (a == b) ? 1 : -1;
}

// ---------------------------------------------------------------------------
// Motor output — DRV8833 IN/IN sign-magnitude, fast decay
// ---------------------------------------------------------------------------
void writeMotor(int pinIn1, int pinIn2, int duty) {
  duty = constrain(duty, -config.max_pwm_duty, config.max_pwm_duty);
  if (duty >= 0) {
    analogWrite(pinIn1, duty);
    analogWrite(pinIn2, 0);
  } else {
    analogWrite(pinIn1, 0);
    analogWrite(pinIn2, -duty);
  }
}

void motorsCoast() {
  analogWrite(PIN_AIN1, 0);
  analogWrite(PIN_AIN2, 0);
  analogWrite(PIN_BIN1, 0);
  analogWrite(PIN_BIN2, 0);
}

// Turn a PID output into a duty for one wheel.
//
// Inversion is applied HERE, at the very last step, and deliberately not to
// the setpoint or the measurement. A motor whose winding is wired backwards
// still has to have its encoder counting positive when the robot moves
// forward, otherwise the velocity loop sees the error growing as it corrects
// and runs away. Flipping only the final duty keeps the loop's sign
// convention intact.
//
// This is the ONLY place a reversed motor is compensated. Do not also negate
// the encoder in the ISR — doing both cancels out into inverted feedback and
// the robot spins in place instead of driving. checkWheelSignSanity() below
// exists to catch exactly that mistake at runtime.
//
// floorDuty is the kickstart: while it is non-zero, a wheel that is being
// asked to move at all gets at least that much duty. Zero commands stay zero
// — a stopped wheel must never be kicked.
int applyWheelCal(float pidOut, bool invert, float floorDuty) {
  float d = pidOut;
  if (floorDuty > 0.0f && d != 0.0f && fabsf(d) < floorDuty) {
    d = (d > 0.0f) ? floorDuty : -floorDuty;
  }
  if (invert) d = -d;
  return (int)d;
}

// ---------------------------------------------------------------------------
// Wheel sign sanity check
//
// A wheel being driven one way while its encoder reports motion the other way
// means the duty sign and the feedback sign disagree — either invert_* is set
// wrong for this chassis, or the encoder A/B leads are swapped. Left
// unreported this is nasty to diagnose, because the closed loop turns it into
// "the robot spins when I press forward" rather than anything that names a
// wheel.
//
// Reported, not corrected: auto-flipping a sign would hide a wiring fault and
// could invert steering on a robot that was driving fine a moment ago. A
// stalled wheel reads zero velocity and is not a disagreement, so the check
// needs real measured motion before it will complain.
WheelSignCheck signCheckLeft, signCheckRight;

void checkWheelSignSanity(WheelSignCheck& st, int duty, float measMmS,
                          const char* wheel, const char* configKey) {
  const int   MIN_DUTY = 40;     // below this the wheel may not be moving yet
  const float MIN_VEL  = 15.0f;  // mm/s; under this is noise or a stall
  const uint32_t GRACE_MS = 1000;

  bool disagrees = (abs(duty) >= MIN_DUTY) && (fabsf(measMmS) >= MIN_VEL) &&
                   ((duty > 0) != (measMmS > 0.0f));

  if (!disagrees) {
    st.badSinceMs = 0;
    st.reported   = false;
    return;
  }
  if (st.badSinceMs == 0) { st.badSinceMs = millis(); return; }
  if (st.reported || (millis() - st.badSinceMs) < GRACE_MS) return;

  st.reported = true;
  logToBackend("error", String("wheel ") + wheel +
               ": duty and encoder disagree in sign for 1 s (duty " + duty +
               ", measured " + (int)measMmS + " mm/s). Check " + configKey +
               " or the encoder A/B leads. Odometry and steering will be wrong.");
}

// ---------------------------------------------------------------------------
// Command + control state
// ---------------------------------------------------------------------------
float cmdLinear  = 0.0f;   // mm/s     requested body velocity
float cmdAngular = 0.0f;   // mdeg/s

float targetLeftMmS  = 0.0f;   // per-wheel setpoints
float targetRightMmS = 0.0f;

float measLeftMmS  = 0.0f;     // per-wheel measurement
float measRightMmS = 0.0f;

float integLeft = 0.0f, integRight = 0.0f;
float prevErrLeft = 0.0f, prevErrRight = 0.0f;
float outLeft = 0.0f, outRight = 0.0f;

// What actually reached the pins, after inversion and the kick floor. Kept
// separate from outLeft/outRight so telemetry reports the real duty rather
// than the PID's pre-calibration intent.
int dutyLeft = 0, dutyRight = 0;

// Kickstart state. These N20 gearmotors need noticeably more duty to break
// away from standstill than to keep turning. The velocity PID does get there
// on its own via integrator wind-up, but that takes a few hundred ms during
// which the wheel is silently stalled — on the bench that read as "the turn
// button does nothing until you also press forward". A short duty floor when
// leaving zero removes the dead press.
uint32_t kickUntilMs = 0;
bool     wheelsIdle  = true;

// Odometry pose, integrated on-board
float poseX = 0.0f, poseY = 0.0f, poseTheta = 0.0f;   // mm, mm, rad
long  lastEncLeft = 0, lastEncRight = 0;

// Nearest-obstacle distance, pushed down by the backend from the LIDAR feed
// (the Arduino has no sensor of its own). INT32_MAX == "no data / clear".
long  proximityMm         = 2147483647L;
bool  collisionLatched    = false;
uint32_t lastProximityMs  = 0;
static const uint32_t PROXIMITY_STALE_MS = 1000;

uint32_t lastRxMs       = 0;
uint32_t lastControlMs  = 0;
uint32_t lastOdomMs     = 0;
uint32_t lastConnectMs  = 0;
bool     watchdogTripped = false;

// ---------------------------------------------------------------------------
// Networking
// ---------------------------------------------------------------------------
WiFiClient      wifiClient;
WebSocketClient ws(wifiClient, BACKEND_HOST, BACKEND_PORT);
bool wsConnected = false;

// ---------------------------------------------------------------------------
// Frame sending — DO NOT use ws.beginMessage()/print()/endMessage() here
//
// ArduinoHttpClient 0.6.1 buffers an entire outgoing frame in a 128-byte array
// (WS_TX_BUFFER_SIZE) and its overflow guard is buggy:
//
//     if ((iTxSize + aSize) > sizeof(iTxBuffer))
//         aSize = sizeof(iTxSize) - iTxSize;      // sizeof(iTxSize), not iTxBuffer
//
// iTxSize is a uint64_t, so that clamps the write to 8 bytes. Every odom frame
// (~200 bytes) therefore reached the backend as the literal string '{"type":'
// and was logged as "non-JSON from Arduino" — which starved the ROS bridge of
// odometry, so no odom->base_link TF was published, so slam_toolbox never
// produced a map frame, and every Nav2 costmap failed on a missing transform.
//
// WS_TX_BUFFER_SIZE cannot be raised from the sketch: build_opt.h does not
// reach library translation units (verified — RAM usage was unchanged), and a
// sketch-local #define would give the sketch and the library different ideas
// of the struct layout, which is worse than the original bug.
//
// So we write the frame to the TCP socket directly. A client-to-server frame
// must be masked (RFC 6455 §5.3); the mask key is obfuscation, not security,
// and does not need to be cryptographically random.
void wsSendText(const String& payload) {
  if (!wsConnected) return;

  size_t len = payload.length();
  uint8_t header[8];
  size_t  h = 0;
  header[h++] = 0x81;                    // FIN + opcode 0x1 (text)

  if (len < 126) {
    header[h++] = 0x80 | (uint8_t)len;   // MASK bit + 7-bit length
  } else if (len < 65536) {
    header[h++] = 0x80 | 126;            // MASK bit + 16-bit extended length
    header[h++] = (len >> 8) & 0xFF;
    header[h++] = len & 0xFF;
  } else {
    return;                              // never happens; a frame this big is a bug
  }

  uint8_t mask[4];
  for (uint8_t i = 0; i < 4; i++) mask[i] = (uint8_t)random(0, 256);

  wifiClient.write(header, h);
  wifiClient.write(mask, 4);

  // Mask and send in blocks so we never need a full-size copy of the payload.
  uint8_t buf[64];
  size_t  sent = 0;
  while (sent < len) {
    size_t n = min((size_t)sizeof(buf), len - sent);
    for (size_t i = 0; i < n; i++) {
      buf[i] = (uint8_t)payload[sent + i] ^ mask[(sent + i) & 3];
    }
    if (wifiClient.write(buf, n) != n) { wsConnected = false; return; }
    sent += n;
  }
}

void logToBackend(const char* level, const String& msg) {
  if (!wsConnected) return;
  JsonDocument doc;
  doc["type"]   = "log";
  doc["source"] = "motion";
  doc["level"]  = level;
  doc["msg"]    = msg;
  String out;
  serializeJson(doc, out);
  wsSendText(out);
}

void ensureWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 10000) {
    delay(250);
  }
}

void ensureWebSocket() {
  if (wsConnected) return;
  if (millis() - lastConnectMs < RECONNECT_MS) return;
  lastConnectMs = millis();

  ensureWiFi();
  if (WiFi.status() != WL_CONNECTED) return;

  if (ws.begin(BACKEND_PATH) == 0) {
    wsConnected = true;
    lastRxMs    = millis();
    // Announce ourselves so the Dashboard can flip the Arduino dot green and
    // the Tuning Panel can sync its sliders to what the firmware actually has.
    JsonDocument doc;
    doc["type"]     = "hello";
    doc["device"]   = "arduino_uno_r4";
    doc["fw"]       = "1.0.0";
    doc["ip"]       = WiFi.localIP().toString();
    JsonObject cfg  = doc["config"].to<JsonObject>();
    cfg["max_pwm_duty"]       = config.max_pwm_duty;
    cfg["max_linear_speed"]   = config.max_linear_speed;
    cfg["max_angular_speed"]  = config.max_angular_speed;
    cfg["collision_stop_mm"]  = config.collision_stop_mm;
    cfg["encoder_cpr"]        = config.encoder_cpr;
    cfg["wheel_diameter_mm"]  = config.wheel_diameter_mm;
    cfg["wheel_base_mm"]      = config.wheel_base_mm;
    cfg["pid_kp"]             = config.pid_kp;
    cfg["pid_ki"]             = config.pid_ki;
    cfg["pid_kd"]             = config.pid_kd;
    cfg["running"]            = config.running;
    String out;
    serializeJson(doc, out);
    wsSendText(out);
  }
}

// ---------------------------------------------------------------------------
// Inbound message handling
// ---------------------------------------------------------------------------
void applyStop(const char* reason) {
  config.running = false;
  cmdLinear = cmdAngular = 0.0f;
  targetLeftMmS = targetRightMmS = 0.0f;
  integLeft = integRight = 0.0f;
  outLeft = outRight = 0.0f;
  dutyLeft = dutyRight = 0;
  kickUntilMs = 0;
  wheelsIdle  = true;
  motorsCoast();
  if (reason && wsConnected) {
    logToBackend("warn", String("stopped: ") + reason);
  }
}

void handleTuningUpdate(JsonObject doc) {
  // §11.3: only the fields present are patched; everything else untouched.
  if (!doc["max_pwm_duty"].isNull())
    config.max_pwm_duty = constrain((int)doc["max_pwm_duty"], 0, 255);
  if (!doc["max_linear_speed"].isNull())
    config.max_linear_speed = max(0, (int)doc["max_linear_speed"]);
  if (!doc["max_angular_speed"].isNull())
    config.max_angular_speed = max(0, (int)doc["max_angular_speed"]);
  if (!doc["collision_stop_mm"].isNull())
    config.collision_stop_mm = max(0, (int)doc["collision_stop_mm"]);
  if (!doc["encoder_cpr"].isNull())
    config.encoder_cpr = max(1.0f, (float)doc["encoder_cpr"]);
  if (!doc["wheel_diameter_mm"].isNull())
    config.wheel_diameter_mm = max(1.0f, (float)doc["wheel_diameter_mm"]);
  if (!doc["wheel_base_mm"].isNull())
    config.wheel_base_mm = max(1.0f, (float)doc["wheel_base_mm"]);
  if (!doc["pid_kp"].isNull()) config.pid_kp = (float)doc["pid_kp"];
  if (!doc["pid_ki"].isNull()) config.pid_ki = (float)doc["pid_ki"];
  if (!doc["pid_kd"].isNull()) config.pid_kd = (float)doc["pid_kd"];

  // Motion calibration. Trim is clamped to 1.0 at the top: allowing >1 would
  // scale duty past max_pwm_duty and remove the headroom that keeps the 6 V
  // motors safe on an 8.4 V pack.
  if (!doc["invert_left"].isNull())
    config.invert_left = (bool)doc["invert_left"];
  if (!doc["invert_right"].isNull())
    config.invert_right = (bool)doc["invert_right"];
  if (!doc["trim_left"].isNull())
    config.trim_left = constrain((float)doc["trim_left"], 0.5f, 1.0f);
  if (!doc["trim_right"].isNull())
    config.trim_right = constrain((float)doc["trim_right"], 0.5f, 1.0f);
  if (!doc["turn_boost"].isNull())
    config.turn_boost = constrain((float)doc["turn_boost"], 1.0f, 3.0f);
  if (!doc["kick_duty"].isNull())
    config.kick_duty = constrain((int)doc["kick_duty"], 0, 255);
  if (!doc["kick_ms"].isNull())
    config.kick_ms = constrain((int)doc["kick_ms"], 0, 500);
}

void sendAck(const char* what, bool ok) {
  if (!wsConnected) return;
  JsonDocument doc;
  doc["type"] = "ack";
  doc["for"]  = what;
  doc["ok"]   = ok;
  String out;
  serializeJson(doc, out);
  wsSendText(out);
}

void handleMessage(const String& payload) {
  lastRxMs = millis();          // any message counts as link liveness (§11.2)
  watchdogTripped = false;

  JsonDocument doc;
  if (deserializeJson(doc, payload)) return;

  const char* type = doc["type"] | "";

  if (!strcmp(type, "cmd_vel")) {
    // Nav2 / manual drive. Clamped to the *runtime* limits, not compiled ones.
    float lin = doc["linear_mm_s"]   | 0.0f;
    float ang = doc["angular_mdeg_s"] | 0.0f;
    cmdLinear  = constrain(lin, -(float)config.max_linear_speed,
                                 (float)config.max_linear_speed);
    cmdAngular = constrain(ang, -(float)config.max_angular_speed,
                                 (float)config.max_angular_speed);

  } else if (!strcmp(type, "control")) {
    const char* action = doc["action"] | "";
    if (!strcmp(action, "start")) {
      config.running   = true;
      collisionLatched = false;
      integLeft = integRight = 0.0f;
      logToBackend("info", "started");
    } else if (!strcmp(action, "stop") || !strcmp(action, "estop")) {
      applyStop(!strcmp(action, "estop") ? "e-stop from web app"
                                         : "stop from web app");
    }

  } else if (!strcmp(type, "tuning_update")) {
    handleTuningUpdate(doc.as<JsonObject>());
    sendAck("tuning_update", true);

  } else if (!strcmp(type, "tuning_save")) {
    saveConfigToEeprom();                      // §11.4
    sendAck("tuning_save", true);
    logToBackend("info", "config saved to EEPROM");

  } else if (!strcmp(type, "tuning_reset")) {
    bool wasRunning = config.running;
    clearSavedConfig();
    config = RuntimeConfig();                  // back to firmware defaults
    config.running = wasRunning;
    sendAck("tuning_reset", true);
    logToBackend("info", "config reset to firmware defaults");

  } else if (!strcmp(type, "proximity")) {
    // Backend forwards the LIDAR's nearest return so collision-stop can be
    // enforced in firmware rather than only in the planner.
    proximityMm      = doc["min_distance_mm"] | 2147483647L;
    lastProximityMs  = millis();

  } else if (!strcmp(type, "ping")) {
    sendAck("ping", true);
  }
}

// ---------------------------------------------------------------------------
// Control loop
// ---------------------------------------------------------------------------
void updateWheelSetpoints() {
  // Differential drive inverse kinematics.
  // angular is mdeg/s -> rad/s ; half track width in mm.
  float ang = cmdAngular;

  // Turn-in-place costs more than an arc: both tyres scrub sideways across
  // the floor instead of rolling. Asking for a bigger rotation when there is
  // no linear component compensates, and is bounded by the same runtime cap
  // so the boost can never exceed the configured angular limit.
  if (cmdLinear == 0.0f && ang != 0.0f) {
    ang *= config.turn_boost;
    ang = constrain(ang, -(float)config.max_angular_speed,
                          (float)config.max_angular_speed);
  }

  float wRadS  = (ang / 1000.0f) * (float)DEG_TO_RAD;
  float halfWb = config.wheel_base_mm * 0.5f;
  targetLeftMmS  = cmdLinear - wRadS * halfWb;
  targetRightMmS = cmdLinear + wRadS * halfWb;

  // Trim scales the *setpoint*, not the duty. Trimming duty would do nothing
  // here: the velocity PID would simply wind up until it hit the original
  // speed again, and the slider would feel dead. Scaling the target is what
  // actually corrects a wheel whose encoder CPR or gearing is slightly off.
  targetLeftMmS  *= config.trim_left;
  targetRightMmS *= config.trim_right;
}

float pidStep(float target, float meas, float dt,
              float& integ, float& prevErr) {
  float err = target - meas;
  integ += err * dt;
  // Anti-windup: bound the integral to what the duty cap can express.
  float iLimit = (config.max_pwm_duty / max(0.001f, config.pid_ki));
  integ = constrain(integ, -iLimit, iLimit);
  float deriv = (dt > 0.0f) ? (err - prevErr) / dt : 0.0f;
  prevErr = err;
  float out = config.pid_kp * err + config.pid_ki * integ + config.pid_kd * deriv;
  return constrain(out, -(float)config.max_pwm_duty, (float)config.max_pwm_duty);
}

void controlStep(float dt) {
  // --- measure wheel velocities from encoder deltas -----------------------
  noInterrupts();
  long l = encLeftTicks;
  long r = encRightTicks;
  interrupts();

  long dL = l - lastEncLeft;
  long dR = r - lastEncRight;
  lastEncLeft  = l;
  lastEncRight = r;

  float mmPerTick = (PI * config.wheel_diameter_mm) / config.encoder_cpr;
  float dLeftMm   = dL * mmPerTick;
  float dRightMm  = dR * mmPerTick;

  measLeftMmS  = dLeftMm  / dt;
  measRightMmS = dRightMm / dt;

  // --- integrate odometry (exact-arc when turning) ------------------------
  float dCenter = 0.5f * (dLeftMm + dRightMm);
  float dTheta  = (dRightMm - dLeftMm) / config.wheel_base_mm;
  if (fabsf(dTheta) < 1e-6f) {
    poseX += dCenter * cosf(poseTheta);
    poseY += dCenter * sinf(poseTheta);
  } else {
    float radius = dCenter / dTheta;
    poseX += radius * (sinf(poseTheta + dTheta) - sinf(poseTheta));
    poseY -= radius * (cosf(poseTheta + dTheta) - cosf(poseTheta));
  }
  poseTheta += dTheta;
  while (poseTheta >  PI) poseTheta -= 2.0f * PI;
  while (poseTheta < -PI) poseTheta += 2.0f * PI;

  // --- §11.2 watchdog: silence from the backend forces Stop ---------------
  if (wsConnected && (millis() - lastRxMs > WATCHDOG_MS)) {
    if (config.running || !watchdogTripped) {
      watchdogTripped = true;
      applyStop("link watchdog: no backend message for 2 s");
    }
  }
  if (!wsConnected && config.running) {
    watchdogTripped = true;
    applyStop("websocket disconnected");
  }

  // --- collision guard ----------------------------------------------------
  bool proximityFresh = (millis() - lastProximityMs) < PROXIMITY_STALE_MS;
  if (proximityFresh && proximityMm <= config.collision_stop_mm) {
    if (!collisionLatched) {
      collisionLatched = true;
      logToBackend("error", String("collision stop at ") + proximityMm + " mm");
    }
  } else if (collisionLatched && proximityFresh &&
             proximityMm > config.collision_stop_mm * 1.5f) {
    collisionLatched = false;   // hysteresis: needs 1.5x clearance to release
  }

  // --- gate + drive -------------------------------------------------------
  // Forward motion is blocked while latched; reverse is still allowed so the
  // operator can back out of a wall instead of being stranded.
  bool blockedForward = collisionLatched && cmdLinear > 0.0f;

  if (!config.running || blockedForward) {
    if (blockedForward) { cmdLinear = 0.0f; }
    targetLeftMmS = targetRightMmS = 0.0f;
    integLeft = integRight = 0.0f;
    outLeft = outRight = 0.0f;
    dutyLeft = dutyRight = 0;
    kickUntilMs = 0;
    wheelsIdle  = true;
    signCheckLeft = signCheckRight = WheelSignCheck();
    motorsCoast();
    return;
  }

  updateWheelSetpoints();

  // Zero setpoint means "hold still" — cut drive rather than let the
  // integrator hunt around a stalled wheel.
  if (fabsf(targetLeftMmS) < 1.0f && fabsf(targetRightMmS) < 1.0f) {
    integLeft = integRight = 0.0f;
    outLeft = outRight = 0.0f;
    dutyLeft = dutyRight = 0;
    kickUntilMs = 0;
    wheelsIdle  = true;
    signCheckLeft = signCheckRight = WheelSignCheck();
    motorsCoast();
    return;
  }

  // Open the kickstart window on the transition out of standstill only, so a
  // held command does not keep re-triggering it.
  if (wheelsIdle) {
    kickUntilMs = millis() + (uint32_t)config.kick_ms;
    wheelsIdle  = false;
  }
  float floorDuty = 0.0f;
  if (config.kick_duty > 0 && (int32_t)(kickUntilMs - millis()) > 0) {
    floorDuty = (float)min(config.kick_duty, config.max_pwm_duty);
  }

  outLeft  = pidStep(targetLeftMmS,  measLeftMmS,  dt, integLeft,  prevErrLeft);
  outRight = pidStep(targetRightMmS, measRightMmS, dt, integRight, prevErrRight);

  dutyLeft  = applyWheelCal(outLeft,  config.invert_left,  floorDuty);
  dutyRight = applyWheelCal(outRight, config.invert_right, floorDuty);
  writeMotor(PIN_AIN1, PIN_AIN2, dutyLeft);
  writeMotor(PIN_BIN1, PIN_BIN2, dutyRight);

  // Compare what we commanded against what the wheels actually did.
  checkWheelSignSanity(signCheckLeft,  dutyLeft,  measLeftMmS,
                       "L", "invert_left");
  checkWheelSignSanity(signCheckRight, dutyRight, measRightMmS,
                       "R", "invert_right");
}

// ---------------------------------------------------------------------------
// Telemetry
// ---------------------------------------------------------------------------
void sendOdom() {
  if (!wsConnected) return;

  noInterrupts();
  long l = encLeftTicks;
  long r = encRightTicks;
  interrupts();

  float wRadS = (measRightMmS - measLeftMmS) / config.wheel_base_mm;

  JsonDocument doc;
  doc["type"]            = "odom";
  doc["t_ms"]            = millis();
  doc["x_mm"]            = poseX;
  doc["y_mm"]            = poseY;
  doc["theta_rad"]       = poseTheta;
  doc["linear_mm_s"]     = 0.5f * (measLeftMmS + measRightMmS);
  doc["angular_mdeg_s"]  = wRadS * (float)RAD_TO_DEG * 1000.0f;
  doc["ticks_l"]         = l;
  doc["ticks_r"]         = r;
  doc["duty_l"]          = dutyLeft;
  doc["duty_r"]          = dutyRight;
  doc["running"]         = config.running;
  doc["collision"]       = collisionLatched;
  doc["rssi"]            = WiFi.RSSI();

#if BATTERY_SENSE_ENABLED
  float counts = (float)analogRead(PIN_VBAT);
  doc["battery_v"] = (counts / ADC_MAX_COUNTS) * ADC_REF_VOLTS * VBAT_DIVIDER_RATIO;
#endif
  // No battery_v key at all when the divider isn't fitted — §6.1 says omit,
  // never fabricate.

  String out;
  serializeJson(doc, out);
  wsSendText(out);
}

// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);

  pinMode(PIN_NSLEEP, OUTPUT);
  digitalWrite(PIN_NSLEEP, HIGH);        // wake the DRV8833

  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT);
  pinMode(PIN_BIN2, OUTPUT);
  motorsCoast();

  pinMode(PIN_ENC_L_A, INPUT_PULLUP);
  pinMode(PIN_ENC_L_B, INPUT_PULLUP);
  pinMode(PIN_ENC_R_A, INPUT_PULLUP);
  pinMode(PIN_ENC_R_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_A), isrLeft,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_A), isrRight, CHANGE);

  loadConfigFromEeprom();
  config.running = false;                // §11.2 boot default, always

  uint32_t now = millis();
  lastControlMs = lastOdomMs = lastRxMs = now;
}

void loop() {
  ensureWebSocket();

  if (wsConnected) {
    int len = ws.parseMessage();
    while (len > 0) {
      handleMessage(ws.readString());
      len = ws.parseMessage();
    }
    if (!ws.connected()) {
      wsConnected = false;
      applyStop("websocket closed");
    }
  }

  uint32_t now = millis();
  if (now - lastControlMs >= CONTROL_PERIOD_MS) {
    float dt = (now - lastControlMs) / 1000.0f;
    lastControlMs = now;
    controlStep(dt);
  }
  if (now - lastOdomMs >= ODOM_PERIOD_MS) {
    lastOdomMs = now;
    sendOdom();
  }
}
