// =============================================================================
// tellovoice — ESP32-S3 Tello relay firmware
//
// Pipeline:  phone browser --(wss)--> cloud backend --(wss)--> THIS DEVICE
//            --(UDP:8889)--> DJI Tello (RoboMaster TT)
//
// Network topology — selectable via NET_MODE (see config.h):
//   2 = AP+STA (default, verified): ESP32 runs a soft-AP ("TelloBridge") that the
//       Tello joins, AND joins the phone hotspot (STA) for the backend link.
//   1 = AP only: standalone AP for the Tello, no internet/backend.
//   0 = STA only: ESP32 + Tello both join the phone hotspot.
//
// KEY LESSON: neither the ESP32 soft-AP nor an iOS Personal Hotspot forwards a
// UDP *broadcast* to associated clients — only unicast reaches the Tello. So we
// never broadcast "command": in AP modes we learn the Tello's IP from the DHCP
// lease event and unicast to it; in STA-only we unicast-sweep the /28 subnet.
//
// This device DIALS OUT to the backend (it is behind the phone-hotspot NAT and
// can never accept an inbound connection). It:
//   1. brings up WiFi per NET_MODE (soft-AP for the Tello and/or STA to hotspot),
//   2. opens an outbound WebSocket to the backend `/ws/device` and authenticates
//      with a `hello` frame (STA modes),
//   3. talks to the Tello over UDP:8889 (enters SDK mode via unicast "command",
//      discovers the drone IP),
//   4. relays backend `command` frames to the Tello and returns `result`s,
//   5. runs an autonomous 5 s keepalive (`battery?`) so the Tello never hits its
//      15 s no-command auto-land — this does NOT depend on the internet link,
//   6. drives a hardware emergency button that sends `land` straight to the
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

// ---- Network mode -----------------------------------------------------------
// 0 = STA only  : ESP32 joins the phone hotspot; Tello ALSO joins that hotspot
//                 (ap <phoneSSID> ...). Reach Tello by unicast sweep of the LAN.
// 1 = AP only   : ESP32 is a standalone AP; Tello joins it. No internet/voice.
// 2 = AP + STA  : ESP32 runs its own AP (Tello joins it) AND joins the phone
//                 hotspot as a station for the backend. Voice + local drone AP.
//                 (recommended: predictable Tello IP, no hotspot client-isolation)
#define NET_STA    0
#define NET_AP     1
#define NET_AP_STA 2
#ifndef NET_MODE
#define NET_MODE NET_AP_STA
#endif
// Derived flags used throughout.
#define USE_AP  (NET_MODE == NET_AP || NET_MODE == NET_AP_STA)
#define USE_STA (NET_MODE == NET_STA || NET_MODE == NET_AP_STA)
#define HAS_BACKEND USE_STA

// RGB status LED (WS2812/NeoPixel). Generic ESP32-S3 devkits usually wire it to
// GPIO48 (some use 38). Set LED_ENABLE 0 for boards with no addressable LED.
#ifndef LED_ENABLE
#define LED_ENABLE 1
#endif
#ifndef LED_PIN
#define LED_PIN 48
#endif
#ifndef LED_BRIGHTNESS
#define LED_BRIGHTNESS 40   // 0-255 flash ceiling; idle is a fraction of this
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
static const IPAddress TELLO_BROADCAST(255, 255, 255, 255); // limited broadcast (phone LAN)
static const unsigned long REPLY_TIMEOUT_MS   = 2000UL; // await a Tello reply
static const unsigned long DISCOVERY_EVERY_MS = 800UL;  // sweep cadence (probes 3 hosts/call)
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
#if USE_AP
static IPAddress apClientIp;             // DHCP lease handed to our AP client (Tello)
static bool  apClientPresent  = false;   // a client joined our soft-AP
#endif
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

// RGB status LED — non-blocking flash overlay on top of an idle color.
static unsigned long ledFlashUntilMs = 0; // 0 = no active flash
static uint8_t ledFlashR = 0, ledFlashG = 0, ledFlashB = 0;

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

static void ledInit();
static void ledTick();
static void ledFlash(uint8_t r, uint8_t g, uint8_t b, unsigned long ms);
static void ledApplyIdle();

// ============================================================================
// Setup / loop
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println(F("[boot] tellovoice ESP32-S3 relay " FW_VERSION));

  pinMode(EMERGENCY_PIN, INPUT_PULLUP);
  ledInit();

  connectWiFi();

  // Bind local UDP so Tello replies land back on us; we send to TELLO_PORT.
  udp.begin(UDP_LOCAL_PORT);

#if HAS_BACKEND
  // Outbound WebSocket to the backend. beginSSL() uses WiFiClientSecure; with no
  // CA/fingerprint supplied the library defaults to setInsecure() (demo only).
#if WS_USE_TLS
  webSocket.beginSSL(WS_HOST, WS_PORT, WS_PATH);
#else
  webSocket.begin(WS_HOST, WS_PORT, WS_PATH);
#endif
  webSocket.onEvent(wsEvent);
  webSocket.setReconnectInterval(3000); // auto-reconnect on drop
#else
  Serial.println(F("[boot] no backend (AP-only mode) — local control only"));
#endif
}

void loop() {
#if HAS_BACKEND
  webSocket.loop();      // pump WS: connect/reconnect, dispatch frames
#endif
  pollEmergencyButton(); // safety first — must run every iteration
  if (!telloKnown) discoverTello(); // unicast sweep (internally rate-limited)
  keepaliveTick();       // dodge the 15 s auto-land
  ledTick();             // expire the flash overlay back to the idle color
  yield();
}

// ============================================================================
// WiFi
// ============================================================================
static void connectWiFi() {
#if NET_MODE == NET_AP_STA
  WiFi.mode(WIFI_AP_STA);
#elif NET_MODE == NET_AP
  WiFi.mode(WIFI_AP);
#else
  WiFi.mode(WIFI_STA);
#endif
  WiFi.setSleep(false); // keep the link responsive for UDP relay + WS

#if USE_AP
  // Log AP station association + DHCP lease so we learn the Tello's IP directly
  // (the soft-AP does NOT forward our UDP broadcast to clients — must unicast).
  WiFi.onEvent([](arduino_event_id_t ev, arduino_event_info_t info) {
    switch (ev) {
      case ARDUINO_EVENT_WIFI_AP_STACONNECTED: {
        const uint8_t *m = info.wifi_ap_staconnected.mac;
        Serial.printf("[AP] STA CONNECTED  %02x:%02x:%02x:%02x:%02x:%02x\n",
                      m[0], m[1], m[2], m[3], m[4], m[5]);
        break;
      }
      case ARDUINO_EVENT_WIFI_AP_STADISCONNECTED: {
        const uint8_t *m = info.wifi_ap_stadisconnected.mac;
        Serial.printf("[AP] STA DISCONNECTED %02x:%02x:%02x:%02x:%02x:%02x aid=%d\n",
                      m[0], m[1], m[2], m[3], m[4], m[5],
                      info.wifi_ap_stadisconnected.aid);
        break;
      }
      case ARDUINO_EVENT_WIFI_AP_STAIPASSIGNED: {
        apClientIp = IPAddress(info.wifi_ap_staipassigned.ip.addr);
        apClientPresent = true;
        Serial.print(F("[AP] STA GOT IP     "));
        Serial.println(apClientIp);
        break;
      }
      default: break;
    }
  });
  WiFi.softAP(AP_SSID, AP_PASS, AP_CHANNEL, /*hidden=*/0, /*max_conn=*/4);
  Serial.printf("[wifi] AP up: SSID=\"%s\" ip=%s ch=%d\n",
                AP_SSID, WiFi.softAPIP().toString().c_str(), AP_CHANNEL);
#endif

#if USE_STA
  // Join the phone hotspot for the backend link (and, in NET_STA, to share the
  // LAN with the Tello). Keep the emergency button live while it comes up.
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
#endif
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
      ledApplyIdle(); // link down -> red
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
        ledApplyIdle(); // authed -> blue (or green if drone already known)
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
  // Blue while the command is dispatched (telloSendAwait blocks ~up to 2s).
  ledFlash(0, 0, LED_BRIGHTNESS, REPLY_TIMEOUT_MS);

  bool gotReply = false;
  String reply = telloSendAwait(tello, &gotReply);

  if (gotReply) {
    bool ok = telloReplyOk(reply);
    sendResult(id, reply, ok);
    // Cyan = accepted, magenta = Tello rejected the command.
    if (ok) ledFlash(0, LED_BRIGHTNESS, LED_BRIGHTNESS, 300);
    else    ledFlash(LED_BRIGHTNESS, 0, LED_BRIGHTNESS, 400);
    // A `battery?` command carries the level: surface it as telemetry too.
    if (strcmp(tello, "battery?") == 0 && ok) {
      sendTelemetryBattery(reply.toInt());
    }
  } else {
    sendResult(id, "timeout", false);
    ledFlash(LED_BRIGHTNESS, 0, LED_BRIGHTNESS, 400); // magenta = no reply
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
      if (from == WiFi.localIP()) continue; // ignore self (broadcast echo)
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

// Locate the drone by UNICAST-sweeping the phone-hotspot subnet: iOS Personal
// Hotspot forwards client-to-client unicast but blocks broadcast, so we can't
// rely on a 255.255.255.255 "command". Instead we send "command" to every host
// in our /nn subnet and capture whoever replies. iOS hotspot is a /28 (14 hosts)
// so the sweep is cheap. Safe to call repeatedly (rate-limited internally).
static void discoverTello() {
  unsigned long now = millis();
  if (telloKnown) return;
  if (now - lastDiscoveryMs < DISCOVERY_EVERY_MS && lastDiscoveryMs != 0) return;
  lastDiscoveryMs = now;

#if USE_AP
  // The soft-AP does NOT forward our broadcast to associated clients, so we must
  // UNICAST. We learn the Tello's IP from the DHCP-lease event (apClientIp).
  // Send "command" there to enter SDK mode; telloRecv captures it as the Tello.
  if (!apClientPresent) return; // no client joined yet
  Serial.print(F("[tello] AP client at "));
  Serial.print(apClientIp);
  Serial.println(F(" -> sending 'command'"));
  telloSendTo(apClientIp, "command");
  String reply;
  if (telloRecv(reply, REPLY_TIMEOUT_MS)) {
    telloMissStreak = 0;
    Serial.print(F("[tello] found at "));
    Serial.print(telloIp);
    Serial.printf(" reply=\"%s\"\n", reply.c_str());
#if HAS_BACKEND
    if (wsAuthed) sendEvent("tello_found", telloIp.toString());
#endif
    ledApplyIdle();
  } else {
    Serial.println(F("[tello] client present but no reply to 'command' yet"));
  }
  return;
#endif

  IPAddress ip   = WiFi.localIP();
  IPAddress mask = WiFi.subnetMask();
  if (ip == IPAddress(0, 0, 0, 0)) return; // not connected yet

  uint32_t ipv   = (uint32_t)ip;
  uint32_t maskv = (uint32_t)mask;
  uint32_t base  = ipv & maskv;
  uint32_t bcast = base | ~maskv;
  uint32_t first = base + 1;
  uint32_t last  = bcast - 1;
  uint32_t span  = (last >= first) ? (last - first + 1) : 0;
  if (span == 0) return;

  // Probe only a FEW hosts per call. Blasting all 14 at once overruns the LwIP
  // send queue (ENOMEM) and starves the TLS/WS heap. A persistent cursor walks
  // the subnet across calls; each probe pumps the WebSocket + a short recv so
  // discovery never blocks the backend link.
  static uint32_t cursor = 0;
  const int BATCH = 3;
  for (int i = 0; i < BATCH; i++) {
    uint32_t a = first + (cursor % span);
    cursor++;
    IPAddress host(a);
    if (host != ip) {
      telloSendTo(host, "command");
      webSocket.loop();          // keep the backend link serviced
      String reply;
      if (telloRecv(reply, 120)) { // short per-probe wait; telloRecv sets telloIp
        telloMissStreak = 0;
        Serial.print(F("[tello] found at "));
        Serial.print(telloIp);
        Serial.printf(" reply=\"%s\"\n", reply.c_str());
        if (wsAuthed) sendEvent("tello_found", telloIp.toString());
        ledApplyIdle();
        return;
      }
    }
  }
}

static void markTelloLost() {
  if (!telloKnown) return;
  Serial.println(F("[tello] lost (no replies)"));
  telloKnown = false;
  telloMissStreak = 0;
  lastDiscoveryMs = 0; // allow immediate re-discovery
  if (wsAuthed) sendEvent("tello_lost", String());
  ledApplyIdle(); // drone gone -> back to blue
}

// ============================================================================
// Keepalive — poll `battery?` when idle so the Tello never auto-lands (15 s).
// Runs regardless of backend link state; reports battery up when connected.
// ============================================================================
static void keepaliveTick() {
  if (!telloKnown) return;      // discovery handled in loop()
  if (commandInFlight) return;  // a real command already reset the timer
  if (millis() - lastTelloSendMs < KEEPALIVE_MS) return;

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
  ledFlash(LED_BRIGHTNESS, 0, 0, 1000); // bright red, 1s
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

// ============================================================================
// RGB status LED (WS2812). Non-blocking: a flash sets a colour + expiry, and
// ledTick() restores the idle colour when the flash elapses. Idle colour tracks
// link state so the board is a glanceable status indicator at rest.
// ============================================================================
static void ledSet(uint8_t r, uint8_t g, uint8_t b) {
#if LED_ENABLE
  neopixelWrite(LED_PIN, r, g, b);
#else
  (void)r; (void)g; (void)b;
#endif
}

static void ledInit() {
#if LED_ENABLE
  ledApplyIdle(); // boot -> disconnected red until links come up
#endif
}

// Idle colour tracks link state, all at full LED_BRIGHTNESS:
// red = offline, yellow = backend up (no Tello yet), green = ready.
static void ledApplyIdle() {
#if LED_ENABLE
  const uint8_t b = LED_BRIGHTNESS;
#if HAS_BACKEND
  if (wsAuthed && telloKnown)      ledSet(0, b, 0);   // ready: green
  else if (wsAuthed)               ledSet(b, b, 0);   // backend up: yellow
  else                             ledSet(b, 0, 0);   // offline: red
#else
  // No backend (AP-only): green = Tello joined & found, red = waiting.
  if (telloKnown) ledSet(0, b, 0);
  else            ledSet(b, 0, 0);
#endif
#endif
}

static void ledFlash(uint8_t r, uint8_t g, uint8_t b, unsigned long ms) {
#if LED_ENABLE
  ledFlashR = r; ledFlashG = g; ledFlashB = b;
  ledFlashUntilMs = millis() + ms;
  ledSet(r, g, b);
#else
  (void)r; (void)g; (void)b; (void)ms;
#endif
}

static void ledTick() {
#if LED_ENABLE
  if (ledFlashUntilMs != 0 && millis() >= ledFlashUntilMs) {
    ledFlashUntilMs = 0;
    ledApplyIdle();
  }
#endif
}
