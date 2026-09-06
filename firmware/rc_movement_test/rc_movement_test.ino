// ---------------------------------------------------------------------------
// SLAM Bot — STANDALONE RC MOVEMENT & ENCODER CALIBRATION TEST
// ---------------------------------------------------------------------------
// Board: Arduino Uno R4 WiFi
// No ROS2, no LIDAR, no NodeMCU, no backend required.
//
// Controls:
//   Option 1: Open Serial Monitor at 115200 baud and type commands (1-8 or w/a/s/d/x).
//   Option 2: Join WiFi (phone hotspot) and open the printed Web URL to drive from a phone browser.
//
// PHYSICAL MOTOR IDENTIFICATION (Standing behind the robot looking forward):
//   LEFT MOTOR  -> DRV8833 OUT1 & OUT2 | Arduino D6 (IN1) & D11 (IN2) | Encoder D2 (A) & D4 (B)
//   RIGHT MOTOR -> DRV8833 OUT3 & OUT4 | Arduino D10 (IN3) & D9 (IN4)  | Encoder D3 (A) & D5 (B)
// ---------------------------------------------------------------------------

#include <WiFiS3.h>
#include "secrets.h"      // WIFI_SSID & WIFI_PASS

// --- Pin Assignments -------------------------------------------------------
#define PIN_AIN1      6     // Left  Motor IN1
#define PIN_AIN2     11     // Left  Motor IN2
#define PIN_BIN1     10     // Right Motor IN3
#define PIN_BIN2      9     // Right Motor IN4
#define PIN_NSLEEP    8     // DRV8833 nSLEEP / enable (MUST be HIGH)

#define PIN_ENC_L_A   2     // Left  Encoder CH-A (Interrupt)
#define PIN_ENC_L_B   4     // Left  Encoder CH-B
#define PIN_ENC_R_A   3     // Right Encoder CH-A (Interrupt)
#define PIN_ENC_R_B   5     // Right Encoder CH-B

// --- Calibration Constants (Bench Measured) --------------------------------
static const bool INVERT_LEFT  = false; // Left motor forward: D6 PWM, D11 LOW
static const bool INVERT_RIGHT = false; // Right motor forward: D10 PWM, D9 LOW (re-wired OUT3/OUT4)

static const float WHEEL_DIAMETER_MM = 43.0f;
static const float WHEEL_BASE_MM     = 150.0f;
static const float ENCODER_CPR       = 700.0f;

// --- Derived Motion Math ---------------------------------------------------
// 1 rev = PI * 43.0 mm = 135.088 mm
// Ticks per mm = 700 / 135.088 = 5.1818 ticks/mm
static const float TICKS_PER_MM = ENCODER_CPR / (PI * WHEEL_DIAMETER_MM);

// Ticks per turn degree = (PI * WHEEL_BASE_MM / 360.0) * TICKS_PER_MM
// = (471.239 mm / 360 deg) * 5.1818 ticks/mm = 6.783 ticks/deg
static const float TICKS_PER_DEG = (PI * WHEEL_BASE_MM / 360.0f) * TICKS_PER_MM;

static const int SPEED_DUTY = 140;     // Strong breakaway test duty (0..255)

// --- Encoder State ---------------------------------------------------------
volatile long encLeftTicks  = 0;
volatile long encRightTicks = 0;

void isrLeft() {
  bool a = digitalRead(PIN_ENC_L_A);
  bool b = digitalRead(PIN_ENC_L_B);
  encLeftTicks += (a == b) ? 1 : -1;
}

void isrRight() {
  bool a = digitalRead(PIN_ENC_R_A);
  bool b = digitalRead(PIN_ENC_R_B);
  encRightTicks += (a == b) ? -1 : 1;
}

// --- Motor Control ---------------------------------------------------------
void writeMotor(int pinIn1, int pinIn2, int duty) {
  duty = constrain(duty, -200, 200);
  if (duty >= 0) {
    analogWrite(pinIn1, duty);
    analogWrite(pinIn2, 0);
  } else {
    analogWrite(pinIn1, 0);
    analogWrite(pinIn2, -duty);
  }
}

void setMotorDuty(int leftDuty, int rightDuty) {
  int l = INVERT_LEFT  ? -leftDuty  : leftDuty;
  int r = INVERT_RIGHT ? -rightDuty : rightDuty;
  digitalWrite(PIN_NSLEEP, HIGH); // Driver ALWAYS awake
  writeMotor(PIN_AIN1, PIN_AIN2, l);
  writeMotor(PIN_BIN1, PIN_BIN2, r);
}

void stopMotors() {
  analogWrite(PIN_AIN1, 0);
  analogWrite(PIN_AIN2, 0);
  analogWrite(PIN_BIN1, 0);
  analogWrite(PIN_BIN2, 0);
}

void resetEncoders() {
  noInterrupts();
  encLeftTicks  = 0;
  encRightTicks = 0;
  interrupts();
}

void printEncoders() {
  noInterrupts();
  long l = encLeftTicks;
  long r = encRightTicks;
  interrupts();
  Serial.print(F(" [Encoder Ticks] Left: "));
  Serial.print(l);
  Serial.print(F(" | Right: "));
  Serial.print(r);
  Serial.print(F(" | Left Dist: "));
  Serial.print(l / TICKS_PER_MM);
  Serial.print(F(" mm | Right Dist: "));
  Serial.print(r / TICKS_PER_MM);
  Serial.println(F(" mm"));
}

// --- Precision Movement Routines -------------------------------------------
void driveDistance(float distanceMm, int speed = SPEED_DUTY) {
  resetEncoders();
  long targetTicks = labs((long)(distanceMm * TICKS_PER_MM));
  int dir = (distanceMm >= 0) ? 1 : -1;

  Serial.print(F(">> Driving "));
  Serial.print(distanceMm);
  Serial.print(F(" mm (Target Ticks: "));
  Serial.print(targetTicks);
  Serial.println(F(")..."));

  setMotorDuty(dir * speed, dir * speed);

  uint32_t t0 = millis();
  while (millis() - t0 < 8000) {
    noInterrupts();
    long l = labs(encLeftTicks);
    long r = labs(encRightTicks);
    interrupts();

    if (max(l, r) >= targetTicks) break;
    delay(5);
  }

  stopMotors();
  delay(200);
  printEncoders();
}

void rotateDegrees(float degrees, int speed = SPEED_DUTY) {
  resetEncoders();
  long targetTicks = labs((long)(degrees * TICKS_PER_DEG));
  int dir = (degrees >= 0) ? 1 : -1; // Positive = Left (CCW), Negative = Right (CW)

  Serial.print(F(">> Rotating "));
  Serial.print(degrees);
  Serial.print(F(" deg (Target Ticks: "));
  Serial.print(targetTicks);
  Serial.println(F(")..."));

  // Left turn: Left wheel BACKWARD (-), Right wheel FORWARD (+)
  // Right turn: Left wheel FORWARD (+), Right wheel BACKWARD (-)
  setMotorDuty(-dir * speed, dir * speed);

  uint32_t t0 = millis();
  while (millis() - t0 < 6000) {
    noInterrupts();
    long l = labs(encLeftTicks);
    long r = labs(encRightTicks);
    interrupts();

    if (max(l, r) >= targetTicks) break;
    delay(5);
  }

  stopMotors();
  delay(200);
  printEncoders();
}

void runSequenceSquareReturn() {
  Serial.println(F("\n=== ROUTINE: Square Drive & Return to Start ==="));
  for (int i = 1; i <= 4; i++) {
    Serial.print(F("\n--- Leg ")); Serial.print(i); Serial.println(F(" of 4 ---"));
    driveDistance(500);          // Drive Forward 50 cm (0.5 m)
    delay(500);
    rotateDegrees(90);           // Turn 90° Left
    delay(500);
  }
  Serial.println(F("=== ROUTINE COMPLETE: Returned to Start Position! ===\n"));
}

void printMenu() {
  Serial.println(F("\n============================================="));
  Serial.println(F("   SLAM BOT — RC MOVEMENT & ENCODER TEST    "));
  Serial.println(F("============================================="));
  Serial.println(F("  [1] Drive Forward  (500 mm / 0.5 m)"));
  Serial.println(F("  [2] Drive Backward (500 mm / 0.5 m)"));
  Serial.println(F("  [3] Turn Left 90 deg"));
  Serial.println(F("  [4] Turn Right 90 deg"));
  Serial.println(F("  [5] Rotate 360 deg Spin"));
  Serial.println(F("  [6] Sharp 180 deg U-Turn"));
  Serial.println(F("  [7] Square Routine (Drive & Return to Start)"));
  Serial.println(F("  [8] Reset & Read Live Encoders"));
  Serial.println(F("  -------------------------------------------"));
  Serial.println(F("  Manual RC Keys: w = Fwd | s = Back | a = Left | d = Right | x = Stop"));
  Serial.println(F("=============================================\n"));
}

void processCommand(char cmd) {
  switch (cmd) {
    case '1': driveDistance(500); break;
    case '2': driveDistance(-500); break;
    case '3': rotateDegrees(90); break;
    case '4': rotateDegrees(-90); break;
    case '5': rotateDegrees(360); break;
    case '6': rotateDegrees(180); break;
    case '7': runSequenceSquareReturn(); break;
    case '8': resetEncoders(); printEncoders(); break;

    // Manual RC commands:
    case 'w': case 'W': Serial.println(F(">> RC: Forward"));  setMotorDuty(SPEED_DUTY, SPEED_DUTY); break;
    case 's': case 'S': Serial.println(F(">> RC: Backward")); setMotorDuty(-SPEED_DUTY, -SPEED_DUTY); break;
    case 'a': case 'A': Serial.println(F(">> RC: Left"));     setMotorDuty(-SPEED_DUTY, SPEED_DUTY); break;
    case 'd': case 'D': Serial.println(F(">> RC: Right"));    setMotorDuty(SPEED_DUTY, -SPEED_DUTY); break;
    case 'x': case 'X': Serial.println(F(">> RC: Stop"));     stopMotors(); break;
    default: break;
  }
}

// --- Web Server Setup for Phone/Browser RC Control -----------------------
WiFiServer server(80);

const char PAGE_HTML[] PROGMEM = R"HTML(<!DOCTYPE html><html><head>
<meta name=viewport content="width=device-width,initial-scale=1">
<title>SLAM Bot - Movement Test</title><style>
body{background:#0d1117;color:#e6edf3;font-family:sans-serif;text-align:center;padding:20px}
.pad{display:grid;grid-template-columns:repeat(3,90px);gap:10px;justify-content:center;margin:20px auto}
button{padding:20px;border-radius:10px;border:none;background:#1f6feb;color:#fff;font-size:20px;font-weight:700;cursor:pointer}
button:active{background:#388bfd}
.stop{background:#da3633}
.seq{margin-top:15px;padding:12px 20px;background:#238636;font-size:15px;width:290px}
</style></head><body>
<h2>SLAM Bot Movement Test</h2>
<div class=pad>
<div></div><button onclick="fetch('/cmd?c=w')">&#9650;</button><div></div>
<button onclick="fetch('/cmd?c=a')">&#9664;</button><button class=stop onclick="fetch('/cmd?c=x')">STOP</button><button onclick="fetch('/cmd?c=d')">&#9654;</button>
<div></div><button onclick="fetch('/cmd?c=s')">&#9660;</button><div></div>
</div>
<button class=seq onclick="fetch('/cmd?c=1')">Drive 0.5m Forward</button><br>
<button class=seq onclick="fetch('/cmd?c=3')">Turn 90° Left</button><br>
<button class=seq onclick="fetch('/cmd?c=4')">Turn 90° Right</button><br>
<button class=seq onclick="fetch('/cmd?c=5')">Rotate 360° Spin</button><br>
<button class=seq onclick="fetch('/cmd?c=6')">180° Sharp U-Turn</button><br>
<button class=seq onclick="fetch('/cmd?c=7')">Return to Start (Square)</button>
</body></html>)HTML";

void handleWebClient(WiFiClient& client) {
  String req = client.readStringUntil('\n');
  if (req.indexOf("/cmd?c=") >= 0) {
    int idx = req.indexOf("/cmd?c=");
    char c = req[idx + 7];
    processCommand(c);
  }
  client.print(F("HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nConnection: close\r\n\r\n"));
  client.print(PAGE_HTML);
  client.stop();
}

void setup() {
  Serial.begin(115200);

  pinMode(PIN_NSLEEP, OUTPUT);
  digitalWrite(PIN_NSLEEP, HIGH); // Keep DRV8833 active

  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT);
  pinMode(PIN_BIN2, OUTPUT);
  stopMotors();

  pinMode(PIN_ENC_L_A, INPUT_PULLUP);
  pinMode(PIN_ENC_L_B, INPUT_PULLUP);
  pinMode(PIN_ENC_R_A, INPUT_PULLUP);
  pinMode(PIN_ENC_R_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_A), isrLeft,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_A), isrRight, CHANGE);

  delay(1000);
  printMenu();

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 5000) { delay(200); }

  if (WiFi.status() == WL_CONNECTED) {
    server.begin();
    Serial.print(F(">> Web Controller URL: http://"));
    Serial.println(WiFi.localIP());
  } else {
    Serial.println(F(">> WiFi optional: use Serial Monitor for controls!"));
  }
}

void loop() {
  if (Serial.available() > 0) {
    char c = Serial.read();
    if (c != '\r' && c != '\n') {
      processCommand(c);
      delay(200);
      printMenu();
    }
  }

  WiFiClient client = server.available();
  if (client) {
    handleWebClient(client);
  }
}
