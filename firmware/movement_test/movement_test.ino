// ---------------------------------------------------------------------------
// SLAM Bot — MOVEMENT TEST ONLY
// ---------------------------------------------------------------------------
// Standalone drive test for the Arduino Uno R4 WiFi. Nothing else is needed:
// no laptop backend, no ROS, no NodeMCU, no LIDAR. The Arduino joins your
// phone hotspot, serves its own control page, and you drive it from a browser.
//
// This is deliberately NOT the production firmware. It exists to answer one
// question — "is the motor half of the robot wired and working?" — with the
// fewest moving parts possible. Once this passes, flash arduino_uno_r4.ino
// for the real WebSocket + backend + ROS path.
//
// WIRING ASSUMED (matches your circuit diagram exactly):
//   DRV8833 IN1  -> D6      DRV8833 IN2  -> D11
//   DRV8833 IN3  -> D10     DRV8833 IN4  -> D9
//   DRV8833 EEP  -> D8      (nSLEEP / enable — must be HIGH to drive)
//   DRV8833 VCC  -> RAW BATTERY +  (this board's VCC *is* the motor supply)
//   DRV8833 GND  -> battery -      (shared with the 5 V rail ground)
//   Left  encoder C1 -> D2   C2 -> D4
//   Right encoder C1 -> D3   C2 -> D5
//   Arduino 5V   -> buck converter 5 V output
//
// !! READ BEFORE POWERING UP !!
//   1. Put the robot ON A STAND with the wheels off the ground for the first
//      run. Direction errors are normal on a first bring-up.
//   2. Do NOT have USB and the battery connected at the same time once the
//      buck feeds the Arduino 5V pin. See the note by USB_AND_BATTERY below.
// ---------------------------------------------------------------------------

#include <WiFiS3.h>
#include "secrets.h"      // WIFI_SSID / WIFI_PASS — your phone hotspot

// ---------------------------------------------------------------------------
// Static IP — pins the URL so it never changes
// ---------------------------------------------------------------------------
// By default the hotspot's DHCP server picks our address. That is fine while
// USB is plugged in and you can read the URL off the Serial Monitor, but once
// the robot runs on battery there is no serial port and no way to discover a
// changed address. Setting a static IP makes the URL permanent, so you can
// bookmark it.
//
// HOW TO FILL THIS IN — do the DHCP run first:
//   1. Run once on USB and read the banner, e.g.
//        OPEN:    http://192.168.43.112
//        Gateway: 192.168.43.1
//   2. Keep the first three numbers of the gateway, choose a high last number
//      that DHCP is unlikely to hand out (200 is a safe bet):
//        STATIC_IP  ->  192.168.43.200
//        GATEWAY_IP ->  192.168.43.1      (exactly as printed)
//   3. Uncomment USE_STATIC_IP below and re-upload.
//
// The first three numbers MUST match your gateway. Android hotspots are
// usually 192.168.43.x, iPhone 172.20.10.x — so do not guess, read them.

// NOTE: currently DISABLED. The values below had mismatched networks —
// STATIC_IP was 10.223.11.200 but GATEWAY_IP was 10.23.11.29 (223 vs 23), which
// puts them on two different networks and makes the board unreachable even
// though it reports "connected". Run on DHCP first, read the real numbers off
// the banner, then re-enable.
// #define USE_STATIC_IP

#ifdef USE_STATIC_IP
  // The first THREE numbers must be identical in both lines. Only the last
  // number differs. Read the gateway off the serial banner — do not guess.
  IPAddress STATIC_IP (10, 223, 11, 200);   // <-- last number: pick 200
  IPAddress GATEWAY_IP(10, 223, 11,   1);   // <-- copy EXACTLY from banner
  IPAddress SUBNET_IP (255, 255, 255, 0);
#endif

// ---------------------------------------------------------------------------
// Motor voltage safety — the important bit for 6 V motors on a 7.4 V pack
// ---------------------------------------------------------------------------
// The DRV8833 passes the battery voltage straight through to the motors, so
// the ONLY thing standing between your 6 V N20s and a 7.4-8.4 V pack is the
// PWM duty cap below. Average motor volts = VBAT * (duty / 255).
//
//   duty 255 @ 8.4 V (full charge) = 8.4 V  <-- would cook the motors
//   duty 200 @ 8.4 V               = 6.6 V  <-- production cap, still spicy
//   duty 120 @ 8.4 V               = 4.0 V  <-- this test, comfortable
//
// 120 is intentionally gentle: you asked for "not that speedy", and a slow
// bot is a bot you can catch before it drives off a table.
static const int MAX_DUTY   = 120;   // absolute ceiling, never exceeded
static const int START_DUTY = 70;    // minimum that actually breaks stiction

// Ramp rate — duty counts per control tick (50 Hz), so 6 meant ~0.4 s to reach
// full duty. That was too slow from a standstill: the motor sat below its
// breakaway torque for a couple of hundred milliseconds and never started,
// which is why turns felt dead until a forward press got the wheels rolling.
// 20 reaches full duty in ~0.12 s — still smooth, but it punches through
// static friction instead of creeping up to it.
static const int RAMP_STEP = 20;

// Kickstart: when starting from a standstill, jump straight to this duty for
// one tick before the ramp takes over. Static friction is much higher than
// rolling friction, so a brief shove costs nothing and removes the dead zone
// entirely. Set to 0 to disable.
static const int KICK_DUTY = 110;
static const uint32_t KICK_MS = 90;

// ---------------------------------------------------------------------------
// Pins — identical to the production firmware so the test proves the real map
// ---------------------------------------------------------------------------
#define PIN_AIN1      6     // left  motor, DRV8833 IN1
#define PIN_AIN2     11     // left  motor, DRV8833 IN2
#define PIN_BIN1     10     // right motor, DRV8833 IN3
#define PIN_BIN2      9     // right motor, DRV8833 IN4
#define PIN_NSLEEP    8     // DRV8833 EEP — LOW = driver asleep = coast

#define PIN_ENC_L_A   2     // interrupt-capable
#define PIN_ENC_L_B   4
#define PIN_ENC_R_A   3     // interrupt-capable
#define PIN_ENC_R_B   5

// Flip these if a wheel spins backwards. Far easier than re-soldering OUT1/OUT2.
//
// INVERT_RIGHT was turned on after the first bring-up. The symptom was:
// forward/back did nothing, "left" drove backwards, "right" drove forwards.
// That is the exact signature of ONE motor being reversed — the turn commands
// come out as straight-line motion and the straight commands cancel into a
// spin. It was the right motor because "left" drove backwards rather than
// forwards. Nothing is wrong with the wiring; the motor leads are simply
// handed, and flipping the flag is equivalent to swapping OUT3/OUT4.
static const bool INVERT_LEFT  = false;
static const bool INVERT_RIGHT = true;

// ---------------------------------------------------------------------------
// Wheel speed matching (trim)
// ---------------------------------------------------------------------------
// Two nominally identical N20s never run at the same speed for the same duty —
// gearbox friction, brush wear and motor tolerance easily differ by 5-15%. The
// result is that "forward" curves gently to one side. There is no encoder
// feedback in this test sketch (the production firmware closes that loop with
// PID), so we correct it open-loop: scale the FASTER wheel down until both
// match. Values are multipliers, 1.00 = untouched.
//
// HOW TO TUNE — wheels ON THE GROUND, on a hard flat floor:
//   1. Both at 1.00, drive forward ~2 m at 45% speed and watch which way it
//      curves.
//   2. Curves LEFT  => the RIGHT wheel is faster => lower TRIM_RIGHT.
//      Curves RIGHT => the LEFT  wheel is faster => lower TRIM_LEFT.
//   3. Change by 0.05 at a time, re-upload, repeat until it tracks straight.
//
// Only ever trim DOWNWARD (below 1.00). Trimming up would push past MAX_DUTY
// and lose you the motor-voltage headroom that protects the 6 V motors.
// These are the power-on defaults. The browser sliders override them live, so
// tune with the sliders first, then copy the numbers you settled on down here
// so they survive a reboot.
float TRIM_LEFT  = 1.00f;
float TRIM_RIGHT = 1.00f;

// Extra push for on-the-spot turns. Scrubbing both tyres sideways takes far
// more torque than rolling forward, so without this the turn commands stall at
// low speed settings. 1.6 = turns get 60% more command than you asked for.
static const float TURN_BOOST = 1.60f;

// ---------------------------------------------------------------------------
// Safety timing
// ---------------------------------------------------------------------------
// If the browser stops talking to us — page closed, phone locked, WiFi dropped,
// you walked out of range — the bot must stop by itself. The page sends a
// command every 150 ms; if 500 ms passes with nothing, we cut the motors.
static const uint32_t COMMAND_TIMEOUT_MS = 500;
static const uint32_t CONTROL_PERIOD_MS  = 20;    // 50 Hz

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------
WiFiServer server(80);

volatile long encLeft  = 0;
volatile long encRight = 0;

int targetLeft  = 0, targetRight = 0;   // requested duty, signed
int actualLeft  = 0, actualRight = 0;   // ramped duty, signed

bool     armed          = false;        // boot stopped, always
uint32_t lastCommandMs  = 0;
uint32_t lastControlMs  = 0;
uint32_t lastBannerMs   = 0;            // status reprint — see printBanner()
uint32_t kickUntilMs    = 0;            // kickstart window — see controlTick()

// ---------------------------------------------------------------------------
// Encoders — quadrature, so we can prove both wheels actually turned
// ---------------------------------------------------------------------------
void isrLeft() {
  bool a = digitalRead(PIN_ENC_L_A);
  bool b = digitalRead(PIN_ENC_L_B);
  encLeft += (a == b) ? 1 : -1;
}

void isrRight() {
  bool a = digitalRead(PIN_ENC_R_A);
  bool b = digitalRead(PIN_ENC_R_B);
  encRight += (a == b) ? 1 : -1;
}

// ---------------------------------------------------------------------------
// Motor output — DRV8833 IN/IN sign-magnitude
// ---------------------------------------------------------------------------
void writeMotor(int pinIn1, int pinIn2, int duty) {
  duty = constrain(duty, -MAX_DUTY, MAX_DUTY);
  if (duty >= 0) {
    analogWrite(pinIn1, duty);
    analogWrite(pinIn2, 0);
  } else {
    analogWrite(pinIn1, 0);
    analogWrite(pinIn2, -duty);
  }
}

void motorsOff() {
  analogWrite(PIN_AIN1, 0);
  analogWrite(PIN_AIN2, 0);
  analogWrite(PIN_BIN1, 0);
  analogWrite(PIN_BIN2, 0);
  targetLeft = targetRight = 0;
  actualLeft = actualRight = 0;
}

// Move `actual` toward `target` by at most RAMP_STEP.
int rampToward(int actual, int target) {
  if (actual < target) return min(actual + RAMP_STEP, target);
  if (actual > target) return max(actual - RAMP_STEP, target);
  return actual;
}

// Map a -100..100 percentage onto the usable duty band. Anything below the
// deadband becomes zero — a duty of 20 just makes the motor buzz and heat up.
// `trim` scales the result so a fast motor can be slowed to match a slow one.
int pctToDuty(int pct, float trim) {
  pct = constrain(pct, -100, 100);
  if (pct == 0) return 0;
  int mag = START_DUTY + (abs(pct) * (MAX_DUTY - START_DUTY)) / 100;
  mag = (int)(mag * trim + 0.5f);
  mag = constrain(mag, 0, MAX_DUTY);
  return (pct > 0) ? mag : -mag;
}

// ---------------------------------------------------------------------------
// Drive command — linear/turn in percent, mixed to differential
// ---------------------------------------------------------------------------
void drive(int linearPct, int turnPct) {
  if (!armed) { targetLeft = targetRight = 0; return; }

  // Spinning on the spot is much harder work than driving straight: both tyres
  // have to scrub sideways across the floor instead of rolling. At the gentle
  // duty this test uses, that is often enough to stall the motors completely —
  // which looks exactly like "the turn buttons do nothing". So when there is no
  // forward component, push the turn command harder.
  if (linearPct == 0 && turnPct != 0) {
    int boosted = (int)(abs(turnPct) * TURN_BOOST);
    boosted = constrain(boosted, 0, 100);
    turnPct = (turnPct > 0) ? boosted : -boosted;
  }

  int l = constrain(linearPct + turnPct, -100, 100);
  int r = constrain(linearPct - turnPct, -100, 100);

  int newLeft  = pctToDuty(INVERT_LEFT  ? -l : l, TRIM_LEFT);
  int newRight = pctToDuty(INVERT_RIGHT ? -r : r, TRIM_RIGHT);

  // Opening a kick window when we go from stopped to moving.
  bool wasStopped = (targetLeft == 0 && targetRight == 0);
  bool nowMoving  = (newLeft  != 0 || newRight  != 0);
  if (wasStopped && nowMoving) kickUntilMs = millis() + KICK_MS;

  targetLeft  = newLeft;
  targetRight = newRight;
  lastCommandMs = millis();
}

// ---------------------------------------------------------------------------
// Control loop
// ---------------------------------------------------------------------------
void controlTick() {
  // Watchdog — silence means stop. This is the single most important line
  // in the file; without it a dropped WiFi link means a runaway robot.
  if (armed && (millis() - lastCommandMs > COMMAND_TIMEOUT_MS)) {
    targetLeft = targetRight = 0;
  }
  if (!armed) {
    targetLeft = targetRight = 0;
  }

  actualLeft  = rampToward(actualLeft,  targetLeft);
  actualRight = rampToward(actualRight, targetRight);

  // Kickstart. Starting from rest the motor has to overcome static friction,
  // which is well above the duty needed to keep it turning. Without this the
  // first ~100 ms of any command is wasted energising a stalled motor, and on a
  // turn-in-place that stall can be permanent. Applied only when leaving zero.
  int outLeft = actualLeft, outRight = actualRight;
  if (KICK_DUTY > 0 && armed) {
    bool starting = (kickUntilMs != 0) && (millis() < kickUntilMs);
    if (starting) {
      if (outLeft  != 0) outLeft  = (outLeft  > 0) ?  max(outLeft,  KICK_DUTY)
                                                  :  min(outLeft,  -KICK_DUTY);
      if (outRight != 0) outRight = (outRight > 0) ?  max(outRight, KICK_DUTY)
                                                  :  min(outRight, -KICK_DUTY);
    } else if (millis() >= kickUntilMs) {
      kickUntilMs = 0;
    }
  }

  digitalWrite(PIN_NSLEEP, armed ? HIGH : LOW);
  writeMotor(PIN_AIN1, PIN_AIN2, outLeft);
  writeMotor(PIN_BIN1, PIN_BIN2, outRight);
}

// ---------------------------------------------------------------------------
// The control page — served from the Arduino itself, no laptop needed
// ---------------------------------------------------------------------------
const char PAGE[] PROGMEM = R"HTML(<!DOCTYPE html><html><head>
<meta name=viewport content="width=device-width,initial-scale=1,user-scalable=no">
<title>SLAM Bot - Movement Test</title><style>
*{box-sizing:border-box;-webkit-user-select:none;user-select:none;-webkit-tap-highlight-color:transparent}
body{margin:0;background:#0d1117;color:#e6edf3;font:15px/1.5 system-ui,sans-serif;
padding:16px;display:flex;flex-direction:column;align-items:center;gap:14px}
h1{font-size:17px;margin:0;font-weight:600}
.bar{display:flex;gap:8px;align-items:center;font-size:13px;color:#9aa7b4}
.dot{width:9px;height:9px;border-radius:50%;background:#f85149}
.dot.ok{background:#3fb950}
#arm{padding:13px 30px;border-radius:9px;border:0;font-size:15px;font-weight:700;
background:#238636;color:#fff;cursor:pointer}
#arm.on{background:#8b1a1a}
.pad{display:grid;grid-template-columns:repeat(3,86px);grid-template-rows:repeat(3,86px);gap:9px}
.pad button{border:1px solid #30363d;background:#1c232c;color:#e6edf3;border-radius:11px;
font-size:26px;cursor:pointer;touch-action:none}
.pad button:active,.pad button.hot{background:#1f6feb;border-color:#4f9cf9}
.pad .stop{background:#8b1a1a;border-color:#c93c3c;font-size:15px;font-weight:700}
.sp{visibility:hidden}
.spd{display:flex;gap:9px;align-items:center;font-size:13px;color:#9aa7b4}
input[type=range]{width:190px}
.enc{font:12px ui-monospace,monospace;color:#6b7684;text-align:center}
.hint{font-size:12px;color:#6b7684;text-align:center;max-width:290px}
</style></head><body>
<h1>SLAM Bot &mdash; Movement Test</h1>
<div class=bar><span class=dot id=d></span><span id=s>connecting</span></div>
<button id=arm>ARM</button>
<div class=spd><span>speed</span><input type=range id=sp min=20 max=100 value=45><span id=spv>45%</span></div>
<div class=pad>
<div class=sp></div><button data-l=1 data-t=0>&#9650;</button><div class=sp></div>
<button data-l=0 data-t=-1>&#9664;</button><button class=stop data-l=0 data-t=0>STOP</button><button data-l=0 data-t=1>&#9654;</button>
<div class=sp></div><button data-l=-1 data-t=0>&#9660;</button><div class=sp></div>
</div>
<div class=spd><span>trim&nbsp;L</span><input type=range id=tl min=60 max=100 value=100><span id=tlv>1.00</span></div>
<div class=spd><span>trim&nbsp;R</span><input type=range id=tr min=60 max=100 value=100><span id=trv>1.00</span></div>
<div class=enc id=e>L 0 &nbsp; R 0</div>
<div class=hint>Hold a button to move. Release to stop. Arrow keys or WASD also work; space is an emergency stop.</div>
<script>
let lin=0,turn=0,armed=false,spd=45,fails=0;
const d=document.getElementById('d'),s=document.getElementById('s'),
      e=document.getElementById('e'),arm=document.getElementById('arm'),
      sp=document.getElementById('sp'),spv=document.getElementById('spv');

sp.oninput=()=>{spd=+sp.value;spv.textContent=spd+'%'};

// Live trim. Sliders are 60-100 and get sent as-is; the firmware divides by
// 100. Tuning this in the browser beats re-uploading for every 0.05 change.
const tl=document.getElementById('tl'),tr=document.getElementById('tr'),
      tlv=document.getElementById('tlv'),trv=document.getElementById('trv');
let trimL=100,trimR=100;
tl.oninput=()=>{trimL=+tl.value;tlv.textContent=(trimL/100).toFixed(2)};
tr.oninput=()=>{trimR=+tr.value;trv.textContent=(trimR/100).toFixed(2)};

// One request every 150 ms. The firmware stops the motors if 500 ms passes
// with no request, so a closed tab or a dead link stops the robot.
async function tick(){
  try{
    const r=await fetch(`/c?l=${Math.round(lin*spd)}&t=${Math.round(turn*spd)}`+
                        `&tl=${trimL}&tr=${trimR}&a=${armed?1:0}`,
                        {cache:'no-store'});
    const j=await r.json();
    fails=0; d.classList.add('ok');
    s.textContent=j.armed?'armed':'disarmed';
    e.textContent=`L ${j.el}  R ${j.er}`;
    armed=j.armed; arm.textContent=armed?'DISARM':'ARM';
    arm.classList.toggle('on',armed);
  }catch(err){
    if(++fails>2){d.classList.remove('ok');s.textContent='link lost';}
  }
}
setInterval(tick,150);

arm.onclick=async()=>{ await fetch('/arm?v='+(armed?0:1),{cache:'no-store'}); tick(); };

// Send the new command IMMEDIATELY on press instead of waiting for the next
// 150 ms poll. That wait was why a turn felt dead until you also pressed
// forward: the press registered, but nothing reached the board for up to 150 ms,
// and by then the wheels had settled back into static friction.
function set(l,t){lin=l;turn=t;tick()}
for(const b of document.querySelectorAll('.pad button')){
  const l=+b.dataset.l,t=+b.dataset.t;
  const on=ev=>{ev.preventDefault();set(l,t)};
  const off=ev=>{ev.preventDefault();set(0,0)};
  b.addEventListener('pointerdown',on);
  b.addEventListener('pointerup',off);
  b.addEventListener('pointerleave',off);
  b.addEventListener('pointercancel',off);
}
const keys={};
const KM={ArrowUp:[1,0],ArrowDown:[-1,0],ArrowLeft:[0,-1],ArrowRight:[0,1],
          w:[1,0],s:[-1,0],a:[0,-1],d:[0,1]};
function applyKeys(){
  let l=0,t=0;
  for(const k in keys) if(keys[k]&&KM[k]){l+=KM[k][0];t+=KM[k][1]}
  set(Math.max(-1,Math.min(1,l)),Math.max(-1,Math.min(1,t)));
}
addEventListener('keydown',ev=>{
  if(ev.key===' '){ev.preventDefault();set(0,0);fetch('/arm?v=0',{cache:'no-store'});return}
  const k=ev.key.length===1?ev.key.toLowerCase():ev.key;
  if(KM[k]){ev.preventDefault();keys[k]=1;applyKeys()}
});
addEventListener('keyup',ev=>{
  const k=ev.key.length===1?ev.key.toLowerCase():ev.key;
  if(KM[k]){keys[k]=0;applyKeys()}
});
addEventListener('blur',()=>{for(const k in keys)keys[k]=0;set(0,0)});
</script></body></html>)HTML";

// ---------------------------------------------------------------------------
// Tiny HTTP handling
// ---------------------------------------------------------------------------
// Read one query parameter out of the request line.
//
// The naive indexOf(key) is WRONG here: searching for "l=" would happily match
// the "l=" inside "tl=", so ?l=50&tl=85 would parse the left drive command as
// 85. A parameter only really starts after a '?' or '&', so check the byte
// before the match and keep looking if it is anything else.
long queryInt(const String& req, const char* key, long fallback) {
  int from = 0;
  while (true) {
    int i = req.indexOf(key, from);
    if (i < 0) return fallback;

    char before = (i == 0) ? '\0' : req[i - 1];
    if (before != '?' && before != '&') { from = i + 1; continue; }

    int v = i + strlen(key);
    int end = v;
    if (end < (int)req.length() && req[end] == '-') end++;   // sign
    while (end < (int)req.length() && isdigit(req[end])) end++;
    if (end == v) return fallback;
    return req.substring(v, end).toInt();
  }
}

void sendJson(WiFiClient& c) {
  String body = String("{\"armed\":") + (armed ? "true" : "false") +
                ",\"el\":" + String(encLeft) +
                ",\"er\":" + String(encRight) + "}";
  c.print(F("HTTP/1.1 200 OK\r\nContent-Type: application/json\r\n"
            "Cache-Control: no-store\r\nConnection: close\r\nContent-Length: "));
  c.print(body.length());
  c.print(F("\r\n\r\n"));
  c.print(Hey When I am Compling the Code of nodemcu in Ardiuno IDE , I am receiving this error fix this immediately or tell me the fix : 
D:\SLAM Bot\firmware\nodemcu_lidar\nodemcu_lidar.ino:35:10: fatal error: WebSocketsClient.h: No such file or directory
   35 | #include <WebSocketsClient.h>
      |          ^~~~~~~~~~~~~~~~~~~~
compilation terminated.
exit status 1

Compilation error: WebSocketsClient.h: No such file or directorybody);
}

// Send the control page in small chunks.
//
// This matters more than it looks. The page is ~4 kB but the WiFi
// co-processor's transmit buffer is far smaller, so pushing the whole string
// in one print() overruns it and the browser gets a truncated response or a
// dropped connection — which shows up as "this site can't be reached" even
// though the board is perfectly healthy. Feeding it in 256-byte pieces and
// letting the stack drain between them is reliable.
void sendPage(WiFiClient& client) {
  client.print(F("HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n"
                 "Cache-Control: no-store\r\nConnection: close\r\n\r\n"));

  const char* p = PAGE;
  size_t remaining = strlen_P(PAGE);
  char buf[257];

  while (remaining > 0 && client.connected()) {
    size_t n = remaining < 256 ? remaining : 256;
    memcpy_P(buf, p, n);
    buf[n] = '\0';
    client.write((const uint8_t*)buf, n);
    p += n;
    remaining -= n;
    delay(1);              // let the co-processor flush
  }
}

void handleClient(WiFiClient& client) {
  // server.available() can hand back a client before its request bytes have
  // actually arrived. Reading immediately would return an empty line and every
  // request would fall through to the 404 branch — so wait for real data first.
  uint32_t t0 = millis();
  while (client.connected() && !client.available()) {
    if (millis() - t0 > 1000) { client.stop(); return; }
    delay(1);
  }
  if (!client.available()) { client.stop(); return; }

  String line = client.readStringUntil('\n');
  line.trim();

  // Drain the remaining request headers so the browser sees a clean exchange.
  uint32_t drainStart = millis();
  while (client.connected() && millis() - drainStart < 200) {
    if (!client.available()) { delay(1); continue; }
    String h = client.readStringUntil('\n');
    h.trim();
    if (h.length() == 0) break;      // blank line ends the headers
  }

  if (line.startsWith("GET /c?")) {
    int l = queryInt(line, "l=", 0);
    int t = queryInt(line, "t=", 0);
    // Trim arrives as 60..100 and means 0.60..1.00. Clamped so a malformed
    // request can never drive the motors harder than the sliders allow.
    long tl = constrain(queryInt(line, "tl=", 100), 40, 100);
    long tr = constrain(queryInt(line, "tr=", 100), 40, 100);
    TRIM_LEFT  = tl / 100.0f;
    TRIM_RIGHT = tr / 100.0f;
    drive(l, t);
    sendJson(client);

  } else if (line.startsWith("GET /arm?")) {
    armed = queryInt(line, "v=", 0) != 0;
    if (!armed) motorsOff();
    lastCommandMs = millis();
    sendJson(client);

  } else if (line.startsWith("GET / ") || line.startsWith("GET /index") ||
             line.startsWith("GET /?")) {
    sendPage(client);

  } else {
    // Anything else — /favicon.ico, probe requests — gets a real 404 rather
    // than a silent hang, so the browser does not sit there waiting.
    client.print(F("HTTP/1.1 404 Not Found\r\nContent-Length: 0\r\n"
                   "Connection: close\r\n\r\n"));
  }

  client.flush();
}

// ---------------------------------------------------------------------------
// Status LED — the only feedback you get when running on battery
// ---------------------------------------------------------------------------
//   fast blink (5 Hz) ... searching for WiFi. Hotspot off, 5 GHz, or bad password.
//   slow blink (1 Hz) ... on WiFi, disarmed, waiting for a browser.
//   solid on .......... armed. Motors will move on command.
void updateStatusLed() {
  uint32_t now = millis();
  bool on;
  if (WiFi.status() != WL_CONNECTED) on = (now % 200) < 100;   // fast
  else if (!armed)                   on = (now % 1000) < 100;  // slow
  else                               on = true;                // solid
  digitalWrite(LED_BUILTIN, on ? HIGH : LOW);
}

// ---------------------------------------------------------------------------
// Status banner
// ---------------------------------------------------------------------------
// Printed at boot AND repeated every 5 s. The repeat matters: the Arduino IDE
// resets the board when you open Serial Monitor, and if the monitor was shut
// during boot the one-shot message is simply lost. Reprinting means you can
// open the monitor whenever you like and still see the URL.
void printBanner() {
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.println(F("---------------------------------------------"));
    Serial.print  (F("  WiFi:  ")); Serial.println(WIFI_SSID);
    Serial.print  (F("  RSSI:  ")); Serial.print(WiFi.RSSI()); Serial.println(F(" dBm"));
    Serial.print  (F("  OPEN:    http://"));
    Serial.println(WiFi.localIP());
    Serial.print  (F("  Gateway: "));
    Serial.println(WiFi.gatewayIP());     // needed if you set a static IP
    Serial.print  (F("  State:   "));
    Serial.println(armed ? F("ARMED") : F("disarmed"));
    Serial.println(F("---------------------------------------------"));
  } else {
    Serial.print(F("WiFi not connected (status "));
    Serial.print(WiFi.status());
    Serial.println(F("). Retrying..."));
  }
}

// ---------------------------------------------------------------------------
void setup() {
  // Motors first, and OFF, before anything slow like WiFi can run.
  pinMode(PIN_NSLEEP, OUTPUT);
  digitalWrite(PIN_NSLEEP, LOW);        // driver asleep until armed
  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT);
  pinMode(PIN_BIN2, OUTPUT);
  motorsOff();

  pinMode(PIN_ENC_L_A, INPUT_PULLUP);
  pinMode(PIN_ENC_L_B, INPUT_PULLUP);
  pinMode(PIN_ENC_R_A, INPUT_PULLUP);
  pinMode(PIN_ENC_R_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_A), isrLeft,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_A), isrRight, CHANGE);

  // On battery there is no Serial Monitor, so the built-in LED becomes the
  // only status channel. See updateStatusLed() for what the patterns mean.
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 2000) { }

  Serial.println(F("\n\n=== SLAM Bot movement test ==="));

  // The R4's WiFi lives on a separate ESP32-S3 co-processor talking over SPI.
  // If that link is dead nothing else can work, so check it explicitly rather
  // than letting it look like a wrong-password failure.
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println(F("ERROR: WiFi module not responding."));
    Serial.println(F("Update the board's WiFi firmware in the IDE, or reseat the board."));
    return;
  }

  Serial.print(F("Joining \"")); Serial.print(WIFI_SSID); Serial.println(F("\"..."));

#ifdef USE_STATIC_IP
  // Must be called BEFORE begin() or it is ignored.
  WiFi.config(STATIC_IP, GATEWAY_IP, GATEWAY_IP, SUBNET_IP);
  Serial.println(F("Using a static IP."));
#endif

  WiFi.begin(WIFI_SSID, WIFI_PASS);

  t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) {
    delay(400);
    Serial.print('.');
  }
  Serial.println();

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println(F("Not connected yet. Common causes:"));
    Serial.println(F("  - Hotspot is 5 GHz. The R4 is 2.4 GHz ONLY."));
    Serial.println(F("  - SSID or password wrong in secrets.h (case-sensitive)."));
    Serial.println(F("  - Hotspot turned off, or no device attached to keep it awake."));
    Serial.println(F("Will keep retrying in the background. Motors stay disabled."));
  }

  server.begin();
  printBanner();
  Serial.println(F("Wheels off the ground for the first run."));
}

void loop() {
  WiFiClient client = server.available();
  if (client) {
    Serial.println(F("[http] client connected"));
    handleClient(client);
    client.stop();
  }

  uint32_t now = millis();
  if (now - lastControlMs >= CONTROL_PERIOD_MS) {
    lastControlMs = now;
    controlTick();
  }

  updateStatusLed();

  // Losing WiFi entirely must also stop the robot.
  if (WiFi.status() != WL_CONNECTED && armed) {
    armed = false;
    motorsOff();
    Serial.println(F("WiFi lost - disarmed."));
  }

  // Reprint the banner every 5 s so the URL is visible whenever you open the
  // Serial Monitor, and retry the join if we are still not on the network.
  if (now - lastBannerMs >= 5000) {
    lastBannerMs = now;
    printBanner();
    if (WiFi.status() != WL_CONNECTED) {
      WiFi.begin(WIFI_SSID, WIFI_PASS);
    }
  }
}
