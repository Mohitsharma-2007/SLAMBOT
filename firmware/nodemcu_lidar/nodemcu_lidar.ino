// ===========================================================================
// SLAM Bot — NodeMCU (ESP8266) RPLIDAR A1M8 relay
//
// Reads the RPLIDAR A1M8's standard 5-byte scan packets off the hardware UART
// and relays filtered (angle, distance, quality) samples to the FastAPI backend
// over a WebSocket at /ws/lidar. One WebSocket text frame per LIDAR revolution.
//
// Filtering (lidar_min_range_mm / lidar_max_range_mm / lidar_angle_filter) is
// applied here, on-board, from a RUNTIME config struct that the web app can
// patch live over WiFi — §11.1/§11.3. No reflash to retune.
// "Save as default" persists to LittleFS — §11.4.
//
// Libraries required (Arduino IDE → Library Manager):
//   * "WebSockets" by Markus Sattler   (WebSocketsClient)
//   * "ArduinoJson" v7.x
//   ESP8266WiFi + LittleFS ship with the ESP8266 board package.
//
// ---------------------------------------------------------------------------
// IMPORTANT — serial debug is unavailable at runtime (spec §3 note)
// ---------------------------------------------------------------------------
// The RPLIDAR occupies GPIO3(RX0)/GPIO1(TX0), i.e. the same hardware UART as
// the USB-serial bridge. Anything printed to Serial goes down the LIDAR's RX
// line and corrupts the command stream. So: this sketch NEVER writes to Serial
// once running. All diagnostics go out as {"type":"log"} WebSocket frames and
// surface on the web app's Logs page.
//
// Wiring, per §3:  RPLIDAR TX -> GPIO3 (RX0),  RPLIDAR RX -> GPIO1 (TX0),
//                  5 V rail -> VIN,  common ground.
// The A1M8's MOTOCTL pin is left tied to its default (motor always on) — the
// bare A1M8 board spins whenever powered; only the packaged A1 with the
// control board exposes PWM motor control.
// ===========================================================================

#include <ESP8266WiFi.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>
#include <LittleFS.h>
#include "secrets.h"

// ---------------------------------------------------------------------------
// RPLIDAR A1 protocol
// ---------------------------------------------------------------------------
static const uint32_t LIDAR_BAUD = 115200;

static const uint8_t RP_SYNC       = 0xA5;
static const uint8_t RP_CMD_STOP   = 0x25;
static const uint8_t RP_CMD_RESET  = 0x40;
static const uint8_t RP_CMD_SCAN   = 0x20;

// A 7-byte response descriptor precedes the sample stream.
static const uint8_t RESP_DESCRIPTOR_LEN = 7;

// Max samples we buffer per revolution. The A1M8 produces ~360-500 samples per
// turn at 5.5 Hz; anything past this in one revolution is dropped and counted.
static const uint16_t MAX_SAMPLES = 400;

// ---------------------------------------------------------------------------
// §11.1 Runtime config
// ---------------------------------------------------------------------------
static const char*    CONFIG_PATH = "/lidar_cfg.json";
static const uint8_t  MAX_ANGLE_MASKS = 4;

struct AngleMask {
  float start_deg = 0.0f;
  float end_deg   = 0.0f;
  bool  active    = false;
};

struct RuntimeConfig {
  int   lidar_min_range_mm = 150;
  int   lidar_max_range_mm = 6000;
  int   min_quality        = 10;      // A1 reports 0..63; 0 = no valid return
  AngleMask masks[MAX_ANGLE_MASKS];   // lidar_angle_filter — none by default
};

RuntimeConfig config;

bool angleMasked(float deg) {
  for (uint8_t i = 0; i < MAX_ANGLE_MASKS; i++) {
    const AngleMask& m = config.masks[i];
    if (!m.active) continue;
    if (m.start_deg <= m.end_deg) {
      if (deg >= m.start_deg && deg <= m.end_deg) return true;
    } else {
      // Wrapped range, e.g. 350 -> 10 degrees across the zero crossing.
      if (deg >= m.start_deg || deg <= m.end_deg) return true;
    }
  }
  return false;
}

// ---------------------------------------------------------------------------
// Networking
// ---------------------------------------------------------------------------
WebSocketsClient ws;
bool wsConnected = false;

void wsSend(const JsonDocument& doc) {
  if (!wsConnected) return;
  String out;
  serializeJson(doc, out);
  ws.sendTXT(out);
}

void logToBackend(const char* level, const String& msg) {
  JsonDocument doc;
  doc["type"]   = "log";
  doc["source"] = "lidar";
  doc["level"]  = level;
  doc["msg"]    = msg;
  wsSend(doc);
}

// ---------------------------------------------------------------------------
// §11.4 Persistence (LittleFS)
// ---------------------------------------------------------------------------
void saveConfig() {
  JsonDocument doc;
  doc["lidar_min_range_mm"] = config.lidar_min_range_mm;
  doc["lidar_max_range_mm"] = config.lidar_max_range_mm;
  doc["min_quality"]        = config.min_quality;
  JsonArray arr = doc["masks"].to<JsonArray>();
  for (uint8_t i = 0; i < MAX_ANGLE_MASKS; i++) {
    if (!config.masks[i].active) continue;
    JsonObject m = arr.add<JsonObject>();
    m["start_deg"] = config.masks[i].start_deg;
    m["end_deg"]   = config.masks[i].end_deg;
  }
  File f = LittleFS.open(CONFIG_PATH, "w");
  if (!f) { logToBackend("error", "LittleFS open failed on save"); return; }
  serializeJson(doc, f);
  f.close();
}

void loadConfig() {
  if (!LittleFS.exists(CONFIG_PATH)) return;
  File f = LittleFS.open(CONFIG_PATH, "r");
  if (!f) return;
  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, f);
  f.close();
  if (err) return;

  config.lidar_min_range_mm = doc["lidar_min_range_mm"] | config.lidar_min_range_mm;
  config.lidar_max_range_mm = doc["lidar_max_range_mm"] | config.lidar_max_range_mm;
  config.min_quality        = doc["min_quality"]        | config.min_quality;
  for (uint8_t i = 0; i < MAX_ANGLE_MASKS; i++) config.masks[i].active = false;
  uint8_t i = 0;
  for (JsonObject m : doc["masks"].as<JsonArray>()) {
    if (i >= MAX_ANGLE_MASKS) break;
    config.masks[i].start_deg = m["start_deg"] | 0.0f;
    config.masks[i].end_deg   = m["end_deg"]   | 0.0f;
    config.masks[i].active    = true;
    i++;
  }
}

void resetConfig() {
  config = RuntimeConfig();
  if (LittleFS.exists(CONFIG_PATH)) LittleFS.remove(CONFIG_PATH);
}

// ---------------------------------------------------------------------------
// Inbound tuning messages (§11.3)
// ---------------------------------------------------------------------------
void handleMessage(const char* payload, size_t len) {
  JsonDocument doc;
  if (deserializeJson(doc, payload, len)) return;
  const char* type = doc["type"] | "";

  if (!strcmp(type, "tuning_update")) {
    if (!doc["lidar_min_range_mm"].isNull())
      config.lidar_min_range_mm = max(0, (int)doc["lidar_min_range_mm"]);
    if (!doc["lidar_max_range_mm"].isNull())
      config.lidar_max_range_mm = max(1, (int)doc["lidar_max_range_mm"]);
    if (!doc["min_quality"].isNull())
      config.min_quality = constrain((int)doc["min_quality"], 0, 63);

    // lidar_angle_filter arrives as a list of {start_deg, end_deg} sectors to
    // mask out. An empty list clears all masks.
    if (!doc["lidar_angle_filter"].isNull()) {
      for (uint8_t i = 0; i < MAX_ANGLE_MASKS; i++) config.masks[i].active = false;
      uint8_t i = 0;
      for (JsonObject m : doc["lidar_angle_filter"].as<JsonArray>()) {
        if (i >= MAX_ANGLE_MASKS) break;
        config.masks[i].start_deg = m["start_deg"] | 0.0f;
        config.masks[i].end_deg   = m["end_deg"]   | 0.0f;
        config.masks[i].active    = true;
        i++;
      }
    }
    JsonDocument ack;
    ack["type"] = "ack"; ack["for"] = "tuning_update"; ack["ok"] = true;
    wsSend(ack);

  } else if (!strcmp(type, "tuning_save")) {
    saveConfig();
    JsonDocument ack;
    ack["type"] = "ack"; ack["for"] = "tuning_save"; ack["ok"] = true;
    wsSend(ack);
    logToBackend("info", "lidar config saved to LittleFS");

  } else if (!strcmp(type, "tuning_reset")) {
    resetConfig();
    JsonDocument ack;
    ack["type"] = "ack"; ack["for"] = "tuning_reset"; ack["ok"] = true;
    wsSend(ack);
    logToBackend("info", "lidar config reset to defaults");
  }
}

void sendHello() {
  JsonDocument doc;
  doc["type"]   = "hello";
  doc["device"] = "nodemcu_lidar";
  doc["fw"]     = "1.0.0";
  doc["ip"]     = WiFi.localIP().toString();
  JsonObject cfg = doc["config"].to<JsonObject>();
  cfg["lidar_min_range_mm"] = config.lidar_min_range_mm;
  cfg["lidar_max_range_mm"] = config.lidar_max_range_mm;
  cfg["min_quality"]        = config.min_quality;
  JsonArray arr = cfg["lidar_angle_filter"].to<JsonArray>();
  for (uint8_t i = 0; i < MAX_ANGLE_MASKS; i++) {
    if (!config.masks[i].active) continue;
    JsonObject m = arr.add<JsonObject>();
    m["start_deg"] = config.masks[i].start_deg;
    m["end_deg"]   = config.masks[i].end_deg;
  }
  wsSend(doc);
}

void onWsEvent(WStype_t type, uint8_t* payload, size_t len) {
  switch (type) {
    case WStype_CONNECTED:
      wsConnected = true;
      sendHello();
      break;
    case WStype_DISCONNECTED:
      wsConnected = false;
      break;
    case WStype_TEXT:
      handleMessage((const char*)payload, len);
      break;
    default:
      break;
  }
}

// ---------------------------------------------------------------------------
// LIDAR driver
// ---------------------------------------------------------------------------
void lidarSendCommand(uint8_t cmd) {
  uint8_t buf[2] = { RP_SYNC, cmd };
  Serial.write(buf, 2);
  Serial.flush();
}

void lidarStop() {
  lidarSendCommand(RP_CMD_STOP);
  delay(50);
  while (Serial.available()) Serial.read();
}

bool lidarStartScan() {
  lidarStop();
  lidarSendCommand(RP_CMD_SCAN);

  // Consume the 7-byte response descriptor: A5 5A 05 00 00 40 81
  uint32_t deadline = millis() + 1000;
  uint8_t got = 0;
  while (got < RESP_DESCRIPTOR_LEN && millis() < deadline) {
    if (Serial.available()) { Serial.read(); got++; }
    else yield();
  }
  return got == RESP_DESCRIPTOR_LEN;
}

// Per-revolution accumulation buffers.
uint16_t sampleCount = 0;
float    angles[MAX_SAMPLES];
uint16_t dists[MAX_SAMPLES];
uint8_t  quals[MAX_SAMPLES];

uint16_t droppedThisRev = 0;
uint32_t revStartMs     = 0;
uint32_t revCounter     = 0;
uint32_t rawBytesRead   = 0;

// Per-revolution diagnostics.
//
// "n = 0 samples every revolution" is indistinguishable at the backend from a
// dead LIDAR, a mis-synced parser, or a filter that happens to reject
// everything — all three look like silence. Counting each rejection reason
// separately turns that one ambiguous symptom into a specific answer, and
// costs a handful of increments per packet.
struct ScanDiag {
  uint16_t packets    = 0;  // 5-byte packets that passed the checksum bits
  uint16_t badFrame   = 0;  // failed check-bit / start-flag validation
  uint16_t rejQuality = 0;  // quality < min_quality
  uint16_t rejZero    = 0;  // distance 0 == no return
  uint16_t rejNear    = 0;  // closer than lidar_min_range_mm
  uint16_t rejFar     = 0;  // further than lidar_max_range_mm
  uint16_t rejAngle   = 0;  // outside 0..360 or inside an angle mask
  uint8_t  maxQuality = 0;  // best quality seen — 0 means nothing is returning
};
ScanDiag diag;

// Smallest in-range return of the revolution — the backend forwards this to
// the Arduino so collision-stop can act on it (§10 collision_stop_distance_mm).
uint16_t minDistThisRev = 0xFFFF;

void publishRevolution() {
  if (!wsConnected) {
    sampleCount = 0; droppedThisRev = 0; diag = ScanDiag();
    return;
  }

  uint32_t now  = millis();
  uint32_t dtMs = (revStartMs == 0) ? 0 : (now - revStartMs);
  revStartMs = now;

  String out;
  out.reserve(sampleCount * 18 + 180);
  out += "{\"type\":\"scan\",\"seq\":";
  out += String(revCounter++);
  out += ",\"t_ms\":";
  out += String(now);
  out += ",\"rev_ms\":";
  out += String(dtMs);
  out += ",\"n\":";
  out += String(sampleCount);
  out += ",\"dropped\":";
  out += String(droppedThisRev);
  out += ",\"min_range_mm\":";
  out += String(config.lidar_min_range_mm);
  out += ",\"max_range_mm\":";
  out += String(config.lidar_max_range_mm);
  if (minDistThisRev != 0xFFFF) {
    out += ",\"min_distance_mm\":";
    out += String(minDistThisRev);
  }

  out += ",\"angles_deg\":[";
  for (uint16_t i = 0; i < sampleCount; i++) {
    if (i > 0) out += ",";
    out += String(angles[i], 1);
  }
  out += "],\"dists_mm\":[";
  for (uint16_t i = 0; i < sampleCount; i++) {
    if (i > 0) out += ",";
    out += String(dists[i]);
  }
  out += "],\"quality\":[";
  for (uint16_t i = 0; i < sampleCount; i++) {
    if (i > 0) out += ",";
    out += String(quals[i]);
  }
  out += "]}";

  ws.sendTXT(out);
  ESP.wdtFeed();

  sampleCount    = 0;
  droppedThisRev = 0;
  minDistThisRev = 0xFFFF;
  diag           = ScanDiag();
}

// 5-byte packet: [S !S Q(6)] [Aq[6:0] C=1] [Aq[14:7]] [Dist_L] [Dist_H]
uint8_t  pktBuf[5];
uint8_t  pktIdx = 0;
bool     synced = false;

void processPacket() {
  uint8_t b0 = pktBuf[0];
  uint8_t startFlag    = b0 & 0x01;
  uint8_t invStartFlag = (b0 >> 1) & 0x01;
  uint8_t quality      = b0 >> 2;

  // The check bit of byte 1 must be 1 on a valid packet.
  if ((pktBuf[1] & 0x01) != 0x01) { diag.badFrame++; synced = false; return; }
  if (startFlag == invStartFlag)  { diag.badFrame++; synced = false; return; }
  diag.packets++;
  if (quality > diag.maxQuality) diag.maxQuality = quality;

  float    angleDeg = (float)(((uint16_t)(pktBuf[2]) << 7) | (pktBuf[1] >> 1)) / 64.0f;
  uint16_t distMm   = (uint16_t)((((uint16_t)pktBuf[4]) << 8) | pktBuf[3]) / 4;

  if (startFlag) publishRevolution();   // new revolution begins at this sample

  // --- filtering, from the live runtime config ---------------------------
  if (quality < config.min_quality)       { diag.rejQuality++; return; }
  if (distMm == 0)                        { diag.rejZero++;    return; }
  if (distMm < config.lidar_min_range_mm) { diag.rejNear++;    return; }
  if (distMm > config.lidar_max_range_mm) { diag.rejFar++;     return; }
  if (angleDeg < 0.0f || angleDeg >= 360.0f) { diag.rejAngle++; return; }
  if (angleMasked(angleDeg))                 { diag.rejAngle++; return; }

  if (distMm < minDistThisRev) minDistThisRev = distMm;

  if (sampleCount < MAX_SAMPLES) {
    angles[sampleCount] = angleDeg;
    dists[sampleCount]  = distMm;
    quals[sampleCount]  = quality;
    sampleCount++;
  } else {
    droppedThisRev++;
  }
}

void pumpLidar() {
  // Bounded per-call read so the WebSocket and WiFi stacks still get serviced.
  int budget = 256;
  while (Serial.available() && budget-- > 0) {
    uint8_t b = Serial.read();
    rawBytesRead++;

    if (!synced) {
      // Resync on a start-of-revolution byte: S=1, !S=0 -> low two bits 0b01.
      // Waiting for a revolution boundary rather than any plausible byte means
      // we resync on a known-good frame instead of guessing at alignment.
      if ((b & 0x03) == 0x01) {
        pktBuf[0] = b;
        pktIdx    = 1;
        synced    = true;
      }
      continue;
    }

    pktBuf[pktIdx++] = b;
    if (pktIdx >= 5) {
      pktIdx = 0;
      // Validates the packet and, on a bad one, clears `synced` so the next
      // iteration falls into the resync branch above.
      processPacket();
    }
  }
}

// ---------------------------------------------------------------------------
uint32_t lastLidarRestartMs = 0;
uint32_t lastSampleSeenMs   = 0;

void setup() {
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  uint32_t deadline = millis() + 20000;
  while (WiFi.status() != WL_CONNECTED && millis() < deadline) {
    delay(100);
    yield();
  }

  // UART is the LIDAR link, initialized AFTER WiFi connects.
  Serial.begin(LIDAR_BAUD);
  Serial.setRxBufferSize(1024);

  LittleFS.begin();
  loadConfig();

  ws.begin(BACKEND_HOST, BACKEND_PORT, BACKEND_PATH);
  ws.onEvent(onWsEvent);
  ws.setReconnectInterval(2000);
  ws.enableHeartbeat(15000, 3000, 2);

  lidarStartScan();
  lastLidarRestartMs = millis();
  lastSampleSeenMs   = millis();
  revStartMs         = millis();
}

void loop() {
  ws.loop();

  uint32_t rawBefore = rawBytesRead;
  pumpLidar();
  if (rawBytesRead != rawBefore) {
    lastSampleSeenMs = millis();
  }

  // If the LIDAR stops producing bytes for 5 s, re-issue SCAN.
  if (millis() - lastSampleSeenMs > 5000 &&
      millis() - lastLidarRestartMs > 6000) {
    logToBackend("warn", "no lidar bytes for 5 s — re-issuing scan command");
    rawBytesRead = 0;
    synced = false;
    pktIdx = 0;
    lidarStartScan();
    lastLidarRestartMs = millis();
    lastSampleSeenMs   = millis();
  }
}
