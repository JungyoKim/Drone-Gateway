# tellovoice — ESP32-S3 Tello relay firmware

Firmware for the ESP32-S3 that bridges the cloud backend to a DJI Tello / RoboMaster TT.

```
phone browser --(wss)--> cloud backend --(wss, OUTBOUND)--> ESP32-S3 --(UDP:8889)--> Tello
```

The ESP32 **dials out** to the backend (it lives behind the phone-hotspot NAT and can
never accept an inbound connection). Once connected it relays Tello SDK command strings,
polls battery to keep the drone alive, and — critically — drives a **hardware emergency
button that lands the drone over UDP even when the internet link is down**.

**Network (default `NET_MODE 2`, AP+STA):** the ESP32 runs its **own soft-AP
`TelloBridge`** that the Tello joins, **and** connects to the phone hotspot as a station
for the backend. The Tello sits on the ESP32's own subnet (`192.168.4.x`) with a
predictable DHCP IP — no reliance on phone-hotspot client-to-client forwarding. See
**Network mode** below for STA-only / AP-only alternatives.

The wire protocol and numeric limits are mirrored **exactly** from
[`../src/protocol.ts`](../src/protocol.ts) (the single source of truth). `main.cpp` calls
out each mirrored constant/field.

---

## Board & toolchain

- Target: **LOLIN/WEMOS S3 Mini** (`board = lolin_s3_mini`). An alternate
  `esp32-s3-devkitc-1` env is commented at the bottom of `platformio.ini` — either
  ESP32-S3 board works, just pick the one matching your hardware.
- Framework: **Arduino**.
- Build with [PlatformIO](https://platformio.org/):

  ```sh
  cd firmware
  pio run                 # compile
  pio run -t upload       # flash over USB
  pio device monitor      # 115200 baud serial log
  ```

### Libraries (`lib_deps`, auto-installed by PlatformIO)

| Library | Purpose |
|---|---|
| `links2004/WebSockets` | outbound WS/WSS client (`WiFiClientSecure` under the hood) |
| `bblanchon/ArduinoJson` | protocol frame (de)serialization (v7 API) |

---

## Configuration

Two ways to set config — pick one:

**A. `config.h` (recommended).** Copy the template and edit it:

```sh
cp src/config.h.example src/config.h   # then edit src/config.h
```

`config.h` is git-ignored-friendly (keep secrets out of the repo). `main.cpp` includes
it automatically via `#if __has_include("config.h")`.

**B. Build flags.** Uncomment/edit the `-D...` lines in `platformio.ini` under
`build_flags`. These override the `config.h` / compiled defaults.

### Fields

| Field | Meaning |
|---|---|
| `NET_MODE` | `2` = AP+STA (default), `1` = AP-only, `0` = STA-only. See **Network mode**. |
| `WIFI_SSID` | Phone-hotspot SSID (STA modes). **Must be 2.4 GHz** (see below). |
| `WIFI_PASS` | Hotspot password. |
| `AP_SSID` | Soft-AP SSID the Tello joins (AP modes). Alphanumeric. Default `TelloBridge`. |
| `AP_PASS` | Soft-AP password (≥ 8 chars for WPA2). Default `tellovoice`. |
| `AP_CHANNEL` | Soft-AP channel `1–13`. Default `6`. |
| `WS_HOST` | Backend host only — no scheme, no path (e.g. `api.example.com`). |
| `WS_PORT` | `443` for `wss`, or `8080` (or your dev port) for plain `ws`. |
| `WS_PATH` | Fixed: `/ws/device` (the backend's device endpoint). |
| `WS_USE_TLS` | `1` = `wss` (`beginSSL`), `0` = plain `ws` (`begin`) for local dev. |
| `VIDEO_HOST` | Where the raw Tello video relay (UDP) is sent. Defaults to `WS_HOST` — same backend. |
| `VIDEO_PORT` | Backend's video-ingest UDP port. Must match its `VIDEO_PORT` env var. Default `8890`. |
| `DEVICE_ID` | Identifier sent in the `hello` frame. |
| `DEVICE_TOKEN` | Shared secret; **must equal the backend's `DEVICE_TOKEN`** env var. |
| `FW_VERSION` | Reported in the `hello` frame's `fw` field. |
| `LED_ENABLE` / `LED_PIN` / `LED_BRIGHTNESS` | WS2812 status LED (default on, GPIO48, brightness 30). |

The device authenticates by sending, on connect:

```json
{ "type": "hello", "deviceId": "...", "token": "...", "fw": "..." }
```

and expects `{ "type": "welcome", "ok": true }` back.

---

## 2.4 GHz + alphanumeric-SSID requirements

- **The Tello only supports 2.4 GHz WiFi.** The ESP32-S3 (single-band 2.4 GHz) and the
  Tello must join the **same 2.4 GHz** hotspot. On phones that broadcast a dual-band
  hotspot, force it to 2.4 GHz (iPhone: enable "Maximize Compatibility"; Android: set the
  hotspot band to 2.4 GHz).
- **The hotspot SSID must be alphanumeric** (no spaces or punctuation). The Tello's
  `ap {ssid} {pass}` STA-mode command does not tolerate special characters in the SSID —
  keep it to `A–Z a–z 0–9`.

---

## Network mode (`NET_MODE`)

| Mode | ESP32 radio | Tello joins | Backend / voice | Notes |
|---|---|---|---|---|
| `2` **AP+STA** (default) | own AP **+** hotspot STA | ESP32's `TelloBridge` AP | ✅ | Tello on `192.168.4.x`, predictable IP. **Verified working.** |
| `1` AP-only | own AP | ESP32's `TelloBridge` AP | ❌ | Local control only (no internet). Good for bench tests. |
| `0` STA-only | hotspot STA | the **phone hotspot** | ✅ | Both peers on the hotspot LAN; relies on client-to-client unicast. |

`2` is recommended: the ESP32 owns the Tello's network (stable association, known DHCP
IP) while still reaching the backend over the phone hotspot for cloud voice. All three
modes talk to the Tello by **unicast** (see discovery, below).

## Config portal (change WiFi without reflashing)

On boot the ESP32 serves a small web form (port 80) to change the **phone hotspot
SSID/password** and the **device token** at runtime — saved to NVS, no reflash needed.
Join the `TelloBridge` AP and open any of:

- **auto pop-up** — a captive-portal prompt appears on join (like a WiFi login page)
- **`http://tello.local`** — mDNS (`CONFIG_HOST`)
- **`http://192.168.4.1`** — the AP IP (always works)

Enter a password only to change it (blank = keep current); **Save → reboots** to apply.
The AP itself (`AP_SSID` / `AP_PASS`) is **fixed at compile time and not editable** from
the portal. The AP + portal come up even if the hotspot is down, so bad hotspot creds are
always recoverable. Disable with `-DCONFIG_PORTAL=0`.

## Status LED (WS2812)

If `LED_ENABLE=1`, the onboard addressable LED shows state (idle colors; brief flashes on
events):

| Color | Meaning |
|---|---|
| 🔴 red | backend not connected (STA modes) / Tello not found yet (AP-only) |
| 🟡 yellow | backend up, Tello not found yet (STA modes) |
| 🟢 green | ready — backend up **and** Tello found |
| 🔵 blue (flash) | relaying a command |
| 🩵 cyan (flash) | command accepted |
| 🟣 magenta (flash) | command rejected / timed out |
| 🔴 red (1 s) | emergency-button `land` sent |

Generic S3 devkits usually wire the LED to **GPIO48** (some use 38 — set `LED_PIN`).

## One-time Tello station-mode setup (`ap`)

Out of the box the Tello is its own WiFi **access point** (`TELLO-XXXXXX` /
`RMTT-XXXXXX`); you connect *to it*. For this project the Tello must instead **join the
ESP32's `TelloBridge` AP** (default `NET_MODE 2`) so it gets a DHCP IP on the ESP32's
`192.168.4.x` subnet. This is a **one-time** `ap` command sent from a laptop while
connected to the Tello's own AP:

1. Power on the Tello. On a laptop, join its WiFi AP `TELLO-XXXXXX`.
2. Send the SDK handshake, then the `ap` command, to `192.168.10.1:8889` over UDP. Using
   netcat (or any UDP tool):

   ```sh
   # enter SDK mode first
   echo -n "command" | nc -u -w1 192.168.10.1 8889
   # then push it onto the ESP32's AP (must match AP_SSID / AP_PASS in config.h)
   echo -n "ap TelloBridge tellovoice" | nc -u -w1 192.168.10.1 8889
   ```

   > On Windows, bind the UDP socket to the Tello-facing adapter IP if a wired NIC shares
   > the same `192.168.10.x` subnet, or the packet may egress the wrong interface.
   >
   > For `NET_MODE 0` (STA-only) push it onto your phone hotspot instead:
   > `ap MyHotspot24 MyPassword` (alphanumeric SSID).

3. The Tello replies `OK,drone will reboot in 3s` and reboots into **station mode**,
   joining `TelloBridge`. The setting **persists** across reboots (to undo it,
   factory-reset the Tello by holding its power button ~5 s).

Power-up order: **ESP32 on (AP up) → Tello on (auto-joins TelloBridge)**. In `NET_MODE 2`
the ESP32 also needs the phone hotspot up for the backend. The ESP32 discovers the
Tello's IP automatically (below) — you never hardcode it.

### How discovery works (no hardcoded Tello IP)

Neither the ESP32 soft-AP nor an iOS Personal Hotspot forwards a UDP **broadcast** to
associated clients, so discovery is **unicast**:

- **AP modes (`1`/`2`):** the ESP32 is the DHCP server, so it learns the Tello's IP from
  the DHCP-lease event, then unicasts `command` to it to enter SDK mode.
- **STA-only (`0`):** the ESP32 unicast-sweeps its `/28` hotspot subnet (a few hosts per
  pass) until a device replies.

On success it reports `{"type":"event","event":"tello_found","detail":"<ip>"}` to the
backend, and `{"type":"event","event":"tello_lost"}` if the drone stops replying
(3 consecutive missed replies).

---

## Runtime behavior

1. **WiFi** — brings up the interface per `NET_MODE`: soft-AP `TelloBridge` for the Tello
   (AP modes) and/or STA to the hotspot (`WIFI_SSID`/`WIFI_PASS`, retries on failure).
2. **Backend link** (STA modes) — outbound WS/WSS to `WS_HOST:WS_PORT` `WS_PATH`,
   auto-reconnects every 3 s, sends `hello` on connect.
3. **Tello UDP** — binds local UDP `8889`, discovers the drone by **unicast** (DHCP-lease
   IP in AP modes, subnet sweep in STA-only), then enters SDK mode with `command`.
4. **Command relay** — on `{type:"command", id, tello, meta}` from the backend, sends the
   raw `tello` string over UDP, waits ~2 s for the reply, then returns
   `{type:"result", id, response, ok}`. `ok` follows the Tello convention: `"ok"` or a
   bare integer (e.g. `battery?` → `"87"`) is success; anything else (`"error ..."`) is
   failure — mirrors `parseTelloReply()` in `../src/tello.ts`.
5. **Keepalive** — if nothing has been sent to the Tello for `keepaliveMs` (**5000 ms**,
   mirrored from `LIMITS.keepaliveMs` in `../src/protocol.ts`), it sends `battery?` and
   reports `{type:"telemetry", battery:<int>}`. This dodges the Tello's 15 s no-command
   auto-land. **It runs on the ESP32 itself**, so drone safety never depends on the
   internet link.
6. **Emergency button** — see below.

Mirrored `LIMITS` (from `../src/protocol.ts`): `distanceCm` 20–500, `degree` 1–360,
`keepaliveMs` 5000. (Range validation itself happens on the backend; the firmware relays
the already-validated `tello` string and mirrors the constants for reference.)

---

## Emergency button (safety backstop)

A physical button wired to **GPIO0 (the BOOT button on most S3 Mini boards)**, configured
`INPUT_PULLUP` and software-debounced (40 ms). On a debounced press the firmware:

1. Immediately sends **`land`** over UDP **straight to the Tello** (to its discovered IP,
   or to the broadcast address if not yet discovered), sent **twice** since UDP is lossy.
   **This does NOT route through the backend** and works with the WS link fully down.
2. *If* the backend link happens to be up, it also emits
   `{"type":"event","event":"emergency_button"}` — but this is best-effort and secondary;
   the land command has already gone out.

Because `pollEmergencyButton()` runs every `loop()` iteration **and** inside the reply-wait
loop, the button stays responsive even while a command is in flight.

### Wiring

The onboard **BOOT** button already sits on GPIO0 — no wiring needed if you use it.
For a dedicated external button:

```
   GPIO0 ──────┬────────┐
               │       [ push button ]
             (internal  │
              pull-up)  │
   GND ─────────────────┘
```

- One button terminal → **GPIO0**, the other → **GND**.
- No external resistor required — the internal pull-up holds GPIO0 HIGH; pressing pulls it
  LOW (active-LOW).
- To use a different GPIO, change `EMERGENCY_PIN` in `main.cpp`. Avoid strapping pins that
  interfere with boot; GPIO0 is safe *after* boot (only sampled at reset for download
  mode), which is why it doubles as a runtime button here.

---

## Security note — `setInsecure()` TLS caveat

When `WS_USE_TLS=1`, the firmware calls `WebSocketsClient::beginSSL(host, port, path)`
with **no CA certificate, no CA bundle, and no fingerprint**. On ESP32 the library then
calls `WiFiClientSecure::setInsecure()` internally, which establishes an **encrypted but
unauthenticated** TLS connection: traffic is encrypted, but the server certificate is
**not verified**, so the link is vulnerable to man-in-the-middle attacks.

**This is acceptable for a demo only.** For production, pin the backend's certificate:
provide the CA with `beginSslWithCA(host, port, path, CA_cert)` (or a CA bundle via
`beginSslWithBundle(...)`) instead of the plain `beginSSL(...)`, and drop `setInsecure()`.

# Drone-Gateway
