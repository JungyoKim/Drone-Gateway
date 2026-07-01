// =============================================================================
// tellovoice — ESP32-S3 Tello relay firmware
//
// Pipeline:  phone browser --(wss)--> cloud backend --(wss)--> THIS DEVICE
//            --(UDP:8889)--> DJI Tello (RoboMaster TT)
//
// Network topology (STA+AP dual-mode):
//   - STA: connects to phone hotspot for internet (backend WebSocket)
//   - AP ("TelloBridge"): local WiFi that the Tello joins as a station
//   This avoids phone-hotspot client-isolation issues and gives us a
//   predictable subnet (192.168.4.x) for Tello discovery.
//
// This device DIALS OUT to the backend (it is behind the phone-hotspot NAT and
// can never accept an inbound connection). It:
//   1. starts a soft AP ("TelloBridge") for the Tello to join,
//   2. joins the phone hotspot as a WiFi station (STA) for internet,
//   3. opens an outbound WebSocket to the backend `/ws/device` and authenticates
//      with a `hello` frame,
//   4. talks to the Tello over UDP:8889 on the AP subnet (enters SDK mode,
//      discovers the drone IP via subnet broadcast),
//   5. relays backend `command` frames to the Tello and returns `result`s,
//   6. runs an autonomous 5 s keepalive (`battery?`) so the Tello never hits its
//      15 s no-command auto-land — this does NOT depend on the internet link,
//   7. drives a hardware emergency button that sends `land` straight to the
//      Tello over UDP, even when the backend link is down.
//
// The wire protocol (message `type` strings, field names/types) and the numeric
// LIMITS below are mirrored EXACTLY from ../../src/protocol.ts — that file is the
// single source of truth. Do not diverge.
// =============================================================================

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WiFiClientSecure.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>

// ----------------------------------------------------------------------------
// Configuration — from src/config.h (copy config.h.example) or -D build flags.
// Any symbol not defined by either falls back to the placeholder default here.
// ----------------------------------------------------------------------------
#if __has_include("config.h")
#include "config.h"
#endif

#ifndef WIFI_SSID
#define WIFI_SSID "YOUR_HOTSPOT_SSID"
#endif
#ifndef WIFI_PASS
#define WIFI_PASS "YOUR_HOTSPOT_PASSWORD"
#endif
#ifndef WS_HOST
#define WS_HOST "drone.xenv.cc"
#endif
#ifndef WS_PORT
#define WS_PORT 443
#endif
#ifndef WS_PATH
#define WS_PATH "/ws/device"
#endif
#ifndef WS_USE_TLS
#define WS_USE_TLS 1
#endif
#ifndef DEVICE_ID
#define DEVICE_ID "tello-esp32-01"
#endif
#ifndef DEVICE_TOKEN
#define DEVICE_TOKEN "dev-device-token"
#endif
#ifndef FW_VERSION
#define FW_VERSION "esp32s3-tello-relay/1.0.0"
#endif
// Soft-AP for Tello — the drone joins this network instead of the phone hotspot.
#ifndef AP_SSID
#define AP_SSID "TelloBridge"
#endif
#ifndef AP_PASS
#define AP_PASS "tellovoice"
#endif
#ifndef AP_CHANNEL
#define AP_CHANNEL 6
#endif

// ----------------------------------------------------------------------------
// LIMITS — mirrored from src/protocol.ts (LIMITS). Keep in lockstep.
// ----------------------------------------------------------------------------
static const int  DISTANCE_MIN_CM = 20;   // LIMITS.distanceCm.min
static const int  DISTANCE_MAX_CM = 500;  // LIMITS.distanceCm.max
static const int  DEGREE_MIN      = 1;    // LIMITS.degree.min
static const int  DEGREE_MAX      = 360;  // LIMITS.degree.max
static const unsigned long KEEPALIVE_MS = 5000UL; // LIMITS.keepaliveMs

// ----------------------------------------------------------------------------
// Tello UDP / timing constants.
// ----------------------------------------------------------------------------
static const uint16_t TELLO_PORT      = 8889;  // Tello SDK command/response port
static const uint16_t UDP_LOCAL_PORT  = 8889;  // local bind for replies
static const IPAddress TELLO_BROADCAST(192, 168, 4, 255); // AP subnet broadcast
static const unsigned long REPLY_TIMEOUT_MS   = 2000UL; // await a Tello reply
static const unsigned long DISCOVERY_EVERY_MS = 3000UL; // rebroadcast when lost
static const int LOST_MISS_THRESHOLD = 3;      // consecutive misses => tello_lost

// ----------------------------------------------------------------------------
// Emergency button — BOOT/GPIO0, active-LOW with internal pull-up, debounced.
// ----------------------------------------------------------------------------
static const uint8_t EMERGENCY_PIN   = 0;    // GPIO0 (BOOT button on most S3 minis)
static const unsigned long DEBOUNCE_MS = 40; // stable-state debounce window

// ----------------------------------------------------------------------------
// Globals.
// ----------------------------------------------------------------------------
static WebSocketsClient webSocket;
static WiFiUDP udp;

static IPAddress telloIp;               // discovered drone IP
static bool  telloKnown       = false;  // have we heard from the Tello?
static int   telloMissStreak  = 0;      // consecutive send-without-reply
static bool  wsAuthed         = false;  // welcome ok received

static unsigned long lastTelloSendMs   = 0; // for keepalive gating
static unsigned long lastDiscoveryMs   = 0;

// Emergency-button debounce state.
static int  ebLastReading   = HIGH; // raw pin (HIGH = released, pull-up)
static int  ebStableState   = HIGH; // debounced state
static unsigned long ebLastChangeMs = 0;

// Serialize command handling: replies are read synchronously, so refuse a
// nested dispatch that arrives while one is already in flight.
static bool commandInFlight = false;

// ----------------------------------------------------------------------------
// Forward declarations.
// ----------------------------------------------------------------------------
static void connectWiFi();
static void wsEvent(WStype_t type, uint8_t *payload, size_t length);
static void sendJson(JsonDocument &doc);
static void sendHello();
static void sendResult(long id, const String &response, bool ok);
static void sendTelemetryBattery(int battery);
static void sendEvent(const char *event, const String &detail);
static void sendPong();
static void handleCommandFrame(JsonDocument &doc);
static bool telloReplyOk(const String &reply);

static void telloSendTo(const IPAddress &ip, const char *cmd);
static String telloSendAwait(const char *cmd, bool *gotReply);
static bool telloRecv(String &out, unsigned long timeoutMs);

static void discoverTello();
static void markTelloLost();
static void keepaliveTick();
static void pollEmergencyButton();
static void emergencyLand();

// ============================================================================
// Setup / loop
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println(F("[boot] tellovoice ESP32-S3 relay " FW_VERSION));

  pinMode(EMERGENCY_PIN, INPUT_PULLUP);

  connectWiFi();

  // Bind local UDP so Tello replies land back on us; we send to TELLO_PORT.
  udp.begin(UDP_LOCAL_PORT);

  // Outbound WebSocket to the backend. beginSSL() uses WiFiClientSecure; with no
  // CA/fingerprint supplied the library defaults to setInsecure() (demo only).
#if WS_USE_TLS
  webSocket.beginSSL(WS_HOST, WS_PORT, WS_PATH);
#else
  webSocket.begin(WS_HOST, WS_PORT, WS_PATH);
#endif
  webSocket.onEvent(wsEvent);
  webSocket.setReconnectInterval(3000); // auto-reconnect on drop

  // Enter SDK mode + find the drone before we take commands.
  discoverTello();
}

void loop() {
  webSocket.loop();     // pump WS: connect/reconnect, dispatch frames
  pollEmergencyButton(); // safety first — must run every iteration
  keepaliveTick();       // dodge the 15 s auto-land
  yield();
}

// ============================================================================
// WiFi
// ============================================================================
static void connectWiFi() {
  // ---- Soft-AP for Tello (starts immediately, no internet needed) ----------
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(AP_SSID, AP_PASS, AP_CHANNEL, /*hidden=*/0, /*max_conn=*/2);
  Serial.printf("[wifi] AP started: SSID=\"%s\" ip=%s\n",
                AP_SSID, WiFi.softAPIP().toString().c_str());

  // ---- STA to phone hotspot (internet / backend WebSocket) ----------------
  WiFi.setSleep(false); // keep the link responsive for UDP relay + WS
  Serial.printf("[wifi] joining STA SSID \"%s\" ...\n", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    pollEmergencyButton();
    delay(100);
    if (millis() - start > 20000UL) {
      Serial.println(F("[wifi] STA retrying..."));
      WiFi.disconnect();
      WiFi.begin(WIFI_SSID, WIFI_PASS);
      start = millis();
    }
  }
  Serial.print(F("[wifi] STA connected, ip="));
  Serial.println(WiFi.localIP());
}

// ============================================================================
// WebSocket (backend link)
// ============================================================================
static void wsEvent(WStype_t type, uint8_t *payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED:
      Serial.printf("[ws] connected to %s\n", WS_PATH);
      wsAuthed = false;
      sendHello();
      break;

    case WStype_DISCONNECTED:
      Serial.println(F("[ws] disconnected"));
      wsAuthed = false;
      break;

    case WStype_TEXT: {
      // arduinoWebSockets NUL-terminates TEXT payloads.
      JsonDocument doc;
      DeserializationError err = deserializeJson(doc, payload, length);
      if (err) {
        Serial.printf("[ws] bad json: %s\n", err.c_str());
        return;
      }
      const char *t = doc["type"] | "";
      if (strcmp(t, "welcome") == 0) {
        bool ok = doc["ok"] | false;
        wsAuthed = ok;
        Serial.printf("[ws] welcome ok=%d\n", ok ? 1 : 0);
        // If we already know the drone, tell the backend right away.
        if (ok && telloKnown) sendEvent("tello_found", telloIp.toString());
      } else if (strcmp(t, "command") == 0) {
        handleCommandFrame(doc);
      } else if (strcmp(t, "ping") == 0) {
        sendPong();
      }
      break;
    }

    default:
      break; // BIN/PING/PONG/ERROR/fragments — nothing to do here
  }
}

static void sendJson(JsonDocument &doc) {
  String out;
  serializeJson(doc, out);
  webSocket.sendTXT(out);
}

// { type:'hello', deviceId, token, fw }  (DeviceToServer)
static void sendHello() {
  JsonDocument doc;
  doc["type"]     = "hello";
  doc["deviceId"] = DEVICE_ID;
  doc["token"]    = DEVICE_TOKEN;
  doc["fw"]       = FW_VERSION;
  sendJson(doc);
}

// { type:'result', id, response, ok }  (DeviceToServer)
static void sendResult(long id, const String &response, bool ok) {
  JsonDocument doc;
  doc["type"]     = "result";
  doc["id"]       = id;
  doc["response"] = response; // copied into the doc
  doc["ok"]       = ok;
  sendJson(doc);
}

// { type:'telemetry', battery }  (DeviceToServer; battery is an int)
static void sendTelemetryBattery(int battery) {
  JsonDocument doc;
  doc["type"]    = "telemetry";
  doc["battery"] = battery;
  sendJson(doc);
}

// { type:'event', event, detail? }  (DeviceToServer)
static void sendEvent(const char *event, const String &detail) {
  JsonDocument doc;
  doc["type"]  = "event";
  doc["event"] = event;
  if (detail.length() > 0) doc["detail"] = detail;
  sendJson(doc);
}

// { type:'pong' }  (DeviceToServer)
static void sendPong() {
  JsonDocument doc;
  doc["type"] = "pong";
  sendJson(doc);
}

// ============================================================================
// Command relay:  backend { id, tello, meta } -> UDP -> Tello -> result
// ============================================================================
static void handleCommandFrame(JsonDocument &doc) {
  long id = doc["id"] | -1L;
  const char *tello = doc["tello"] | "";
  if (id < 0 || tello[0] == '\0') {
    Serial.println(F("[cmd] malformed command frame"));
    return;
  }

  // Reject re-entrant dispatch (reply reads are synchronous/blocking).
  if (commandInFlight) {
    sendResult(id, "busy", false);
    return;
  }
  commandInFlight = true;

  Serial.printf("[cmd] #%ld -> tello \"%s\"\n", id, tello);

  bool gotReply = false;
  String reply = telloSendAwait(tello, &gotReply);

  if (gotReply) {
    bool ok = telloReplyOk(reply);
    sendResult(id, reply, ok);
    // A `battery?` command carries the level: surface it as telemetry too.
    if (strcmp(tello, "battery?") == 0 && ok) {
      sendTelemetryBattery(reply.toInt());
    }
  } else {
    sendResult(id, "timeout", false);
  }

  commandInFlight = false;
}

// "ok" -> success; a bare integer (e.g. battery? -> "87") -> success; else error.
// Mirrors parseTelloReply() in src/tello.ts.
static bool telloReplyOk(const String &reply) {
  String v = reply;
  v.trim();
  if (v.equalsIgnoreCase("ok")) return true;
  if (v.length() == 0) return false;
  size_t i = (v[0] == '-') ? 1 : 0;
  if (i >= v.length()) return false;
  for (; i < v.length(); i++) {
    if (!isDigit(v[i])) return false;
  }
  return true; // all digits
}

// ============================================================================
// Tello UDP
// ============================================================================
static void telloSendTo(const IPAddress &ip, const char *cmd) {
  udp.beginPacket(ip, TELLO_PORT);
  udp.write(reinterpret_cast<const uint8_t *>(cmd), strlen(cmd));
  udp.endPacket();
  lastTelloSendMs = millis(); // any send resets the keepalive clock
}

// Send `cmd` to the known Tello and wait up to REPLY_TIMEOUT_MS for a reply.
// Updates discovery/liveness bookkeeping. Returns the reply text; *gotReply
// tells whether anything came back.
static String telloSendAwait(const char *cmd, bool *gotReply) {
  *gotReply = false;
  if (!telloKnown) {
    // No drone yet — kick discovery and report the miss.
    discoverTello();
    return String();
  }

  telloSendTo(telloIp, cmd);

  String reply;
  if (telloRecv(reply, REPLY_TIMEOUT_MS)) {
    *gotReply = true;
    telloMissStreak = 0;
    return reply;
  }

  // No reply — count it; drop to "lost" after a few consecutive misses.
  if (++telloMissStreak >= LOST_MISS_THRESHOLD) {
    markTelloLost();
  }
  return String();
}

// Block up to timeoutMs for a UDP datagram; ignore our own broadcast echoes.
// The emergency button is still polled while we wait so the safety path stays
// responsive during the ~2 s command window.
static bool telloRecv(String &out, unsigned long timeoutMs) {
  unsigned long start = millis();
  char buf[256];
  while (millis() - start < timeoutMs) {
    int sz = udp.parsePacket();
    if (sz > 0) {
      IPAddress from = udp.remoteIP();
      int n = udp.read(buf, sizeof(buf) - 1);
      if (n < 0) n = 0;
      buf[n] = '\0';
      if (from == WiFi.localIP() || from == WiFi.softAPIP()) continue; // ignore self
      // First responder becomes / confirms the Tello.
      if (!telloKnown) {
        telloIp = from;
        telloKnown = true;
      }
      out = String(buf);
      return true;
    }
    pollEmergencyButton(); // keep the panic button live during the wait
    delay(2);
  }
  return false;
}

// Enter SDK mode + locate the drone by broadcasting `command` and capturing the
// responder's IP. Safe to call repeatedly (it rate-limits internally).
static void discoverTello() {
  unsigned long now = millis();
  if (telloKnown) return;
  if (now - lastDiscoveryMs < DISCOVERY_EVERY_MS && lastDiscoveryMs != 0) return;
  lastDiscoveryMs = now;

  Serial.println(F("[tello] broadcasting 'command' to discover drone..."));
  telloSendTo(TELLO_BROADCAST, "command");

  String reply;
  if (telloRecv(reply, REPLY_TIMEOUT_MS)) {
    // telloRecv set telloIp/telloKnown from the responder.
    telloMissStreak = 0;
    Serial.print(F("[tello] found at "));
    Serial.print(telloIp);
    Serial.printf(" reply=\"%s\"\n", reply.c_str());
    if (wsAuthed) sendEvent("tello_found", telloIp.toString());
  } else {
    Serial.println(F("[tello] no responder yet"));
  }
}

static void markTelloLost() {
  if (!telloKnown) return;
  Serial.println(F("[tello] lost (no replies)"));
  telloKnown = false;
  telloMissStreak = 0;
  lastDiscoveryMs = 0; // allow immediate re-discovery
  if (wsAuthed) sendEvent("tello_lost", String());
}

// ============================================================================
// Keepalive — poll `battery?` when idle so the Tello never auto-lands (15 s).
// Runs regardless of backend link state; reports battery up when connected.
// ============================================================================
static void keepaliveTick() {
  if (commandInFlight) return; // a real command already reset the timer
  if (millis() - lastTelloSendMs < KEEPALIVE_MS) return;

  if (!telloKnown) {
    discoverTello();
    return;
  }

  bool gotReply = false;
  String reply = telloSendAwait("battery?", &gotReply);
  if (gotReply && telloReplyOk(reply)) {
    int battery = reply.toInt();
    Serial.printf("[keepalive] battery=%d%%\n", battery);
    if (wsAuthed) sendTelemetryBattery(battery);
  }
}

// ============================================================================
// Emergency button — debounced, active-LOW. On a fresh press: send `land`
// straight to the Tello over UDP (never via the backend) and, if the WS is up,
// also emit an emergency_button event. Works with the backend link fully down.
// ============================================================================
static void pollEmergencyButton() {
  int reading = digitalRead(EMERGENCY_PIN);
  unsigned long now = millis();

  if (reading != ebLastReading) {
    ebLastReading = reading;
    ebLastChangeMs = now;
  }

  if (now - ebLastChangeMs >= DEBOUNCE_MS && reading != ebStableState) {
    ebStableState = reading;
    if (ebStableState == LOW) { // pressed (pull-up -> LOW)
      emergencyLand();
    }
  }
}

static void emergencyLand() {
  Serial.println(F("[EMERGENCY] button pressed -> UDP land"));
  // Direct to the drone. Send twice (UDP is lossy) — either to the known IP or,
  // if we haven't discovered it, to the broadcast address as a best effort.
  const IPAddress dst = telloKnown ? telloIp : TELLO_BROADCAST;
  telloSendTo(dst, "land");
  delay(20);
  telloSendTo(dst, "land");

  // Best-effort notify the backend; land already went out regardless.
  if (webSocket.isConnected()) {
    sendEvent("emergency_button", String());
  }
}
