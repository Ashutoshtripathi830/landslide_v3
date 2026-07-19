/*
  ============================================================
  LORA RECEIVER - WIFI PROVISIONING + OLED + WEB DASHBOARD
  ============================================================
  Standard LoRa wiring (E32-433T30D, transparent/normal mode):
    M0  -> GPIO4   (LOW)
    M1  -> GPIO5   (LOW)
    RX2 -> GPIO16  (ESP32 RX2 <- E32 TX)
    TX2 -> GPIO17  (ESP32 TX2 -> E32 RX)
    LED -> GPIO2   (onboard, RX blink)

  OLED (0.96" SSD1306, I2C, 128x64):
    SDA -> GPIO21
    SCL -> GPIO22
    Address: 0x3C (default on most 0.96" modules)

  Status LED strip (WS2812B / NeoPixel, 7 pixels):
    DIN -> GPIO27
    5V  -> 5V (external supply recommended for 7 LEDs at full brightness)
    GND -> GND (common ground with ESP32)

  Relay module (drives a buzzer as a siren):
    IN  -> GPIO25
    VCC -> 5V (or 3.3V - check your relay module's spec)
    GND -> GND (common ground with ESP32)
    Wire the buzzer's own power feed through the relay's COM/NO screw
    terminals so the relay switches the buzzer on and off.
    Most cheap 1-channel relay boards trigger when IN is driven HIGH.
    If yours is the opposite (trigger on LOW), flip RELAY_ACTIVE_HIGH
    to false below - that's the only change needed.

  Libraries needed (Library Manager):
    - Adafruit SSD1306
    - Adafruit GFX Library
    - Adafruit NeoPixel
    (ESPmDNS and Preferences are built into the ESP32 core)

  ------------------------------------------------------------
  BOOT / WIFI FLOW
  ------------------------------------------------------------
  1. On power-up, checks NVS for a saved WiFi SSID/password.
  2. If found, tries to join that network (STA mode, ~20s timeout).
     - Success -> hosts dashboard on the DHCP IP it gets, shown
       on the OLED and also reachable at http://landslide.local
     - Failure -> falls back to step 3.
  3. If nothing saved (or join failed), opens its OWN open
     (no password) WiFi network:
         SSID: Landslide-Setup
     Connect any phone/laptop to it, open http://192.168.4.1,
     pick your real WiFi from the scanned list, enter its
     password, submit. The board saves it and reboots, then
     goes back to step 1.
  4. From the dashboard's Admin panel, "Reset WiFi" clears the
     saved network and reboots back into setup mode.

  Must exactly match the transmitter's SensorPacket struct
  (1 vibration sensor, 1 soil sensor, no water level sensor).
  Open Serial Monitor at 115200 baud for debug logs.

  ------------------------------------------------------------
  ALERTS, SIREN, SD LOGGING, GEOFENCE (added on top of the base build)
  ------------------------------------------------------------
  - Vibration and tilt/roll alerts now LATCH for ALERT_HOLD_MS (default
    1 minute) after the last hit, instead of clearing the instant the
    sensor reading goes back to normal. To change how long alerts last,
    edit the single ALERT_HOLD_MS number near the top of this file.
  - A relay (GPIO25) drives a buzzer as a siren. Tilt/roll = fast wail,
    vibration = double-beep, MPU-confirmed landslide = solid continuous tone.
  - LEDs: RED strobe = tilt/roll ("box moved"), ORANGE beacon = vibration,
    very-fast RED strobe = MPU-confirmed landslide warning (highest priority).
  - GPS drift alone can be a false alarm (multipath/jitter), so it only
    escalates to a "landslide warning" when the MPU also shows tilt/roll or
    vibration in the same window - see landslideWarning in computeRisk().
  - The GPS geofence radius (how far it can drift from its boot-time "home"
    point before counting as drift) is no longer a fixed #define - it's
    adjustable live from the dashboard's Admin View ("Set Geofence") and
    is saved to flash so it survives a reboot.
  - An SD card (SPI: CS=13, SCK=18, MISO=19, MOSI=23) logs one CSV row per
    received packet - timestamp plus every sensor/alert field - to
    /yaksha_log.csv. Downloadable from Admin View. If no card is present
    the board just continues without logging (never blocks anything else).
  - Transmitter is unchanged - none of this required touching its firmware
    or its SensorPacket struct.

  ------------------------------------------------------------
  ADMIN VIEW: TRENDS, PREDICTION & SYSTEM HEALTH (added on top of the CORS
  build)
  ------------------------------------------------------------
  - New GET /history endpoint reads the tail of the SD card's CSV log and
    returns it as compact JSON (timestamp, risk score, tilt, vibration, soil,
    temp/humidity, displacement). Bounded to a fixed byte chunk so it can
    never exhaust RAM even on a large log file. This is what lets the
    dashboard's Admin View trend charts show real history spanning beyond the
    current browser session, not just what's been polled live.
  - /data now also reports freeHeap, wifiRSSI, and uptimeSec so the Admin
    View can show basic device/system health alongside the sensor data.
  - Everything else (relay/siren, LED priority, geofence, tilt threshold,
    SD logging, CORS, tiles) is unchanged from the previous build.
  ============================================================
*/

#include <Adafruit_GFX.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_SSD1306.h>
#include <ESPmDNS.h>
#include <HardwareSerial.h>
#include <Preferences.h>
#include <SD.h>
#include <SPI.h>
#include <WebServer.h>
#include <WiFi.h>
#include <Wire.h>

// ---------------- LoRa pins ----------------
#define M0 4
#define M1 5
#define LED 2
HardwareSerial E32Serial(2);

// ---------------- OLED ----------------
#define OLED_SDA 21
#define OLED_SCL 22
#define OLED_W 128
#define OLED_H 64
#define OLED_ADDR 0x3C
Adafruit_SSD1306 display(OLED_W, OLED_H, &Wire, -1);
bool oledOk = false;

// ---------------- Status LED strip ----------------
#define STRIP_PIN 27
#define NUM_LEDS 7
Adafruit_NeoPixel strip(NUM_LEDS, STRIP_PIN, NEO_GRB + NEO_KHZ800);
String ledStatusText = "Booting";

uint32_t COL_OFF, COL_BLUE, COL_YELLOW, COL_GREEN, COL_ORANGE, COL_RED,
    COL_WHITE, COL_PURPLE, COL_CYAN;

// ---------------- Relay / Buzzer (siren) ----------------
#define RELAY_PIN 25
#define RELAY_ACTIVE_HIGH                                                      \
  true // flip to false if your relay triggers on LOW instead

// ---------------- SD card (data logger) ----------------
// Explicit SPI pins (not the ESP32's default VSPI mapping) - GPIO5 is already
// used by the LoRa module's M1 pin, so the card's CS moves to GPIO13 instead.
#define SD_SCK 18
#define SD_MISO 19
#define SD_MOSI 23
#define SD_CS 13
#define LOG_FILENAME "/yaksha_log.csv"
bool sdOk = false;
unsigned long logRowsWritten = 0;

// ---------------- Alert hold time ----------------
// Once vibration or a tilt/roll ("box moved") event is detected, the alert
// (red/orange LEDs + siren) stays ON for this long afterwards even if the
// sensor reading goes back to normal a moment later - so a single bump or
// shake doesn't disappear before anyone in the area notices it.
//
// TO CHANGE THE ALERT DURATION LATER: just edit the number below (it's in
// milliseconds - 1000 = 1 second). Examples:
//   #define ALERT_HOLD_MS 60000UL     // 1 minute (current)
//   #define ALERT_HOLD_MS 120000UL    // 2 minutes
//   #define ALERT_HOLD_MS 30000UL     // 30 seconds
// Re-upload the sketch after changing it. Both the vibration alert and the
// tilt/roll alert use this same value.
#define ALERT_HOLD_MS 60000UL

// ---------------- WiFi config portal (open AP) ----------------
const char *CONFIG_AP_SSID = "Landslide-Setup"; // open network, no password
IPAddress apIP(192, 168, 4, 1);
IPAddress apGateway(192, 168, 4, 1);
IPAddress apSubnet(255, 255, 255, 0);

WebServer server(80);
Preferences prefs;

// ---------------- System state ----------------
enum SystemState { STATE_AP_CONFIG, STATE_OPERATIONAL };
SystemState sysState = STATE_AP_CONFIG;

// ---- Forward declarations for ALL functions defined later in this file ----
// The large raw-string HTML/JS literals (R"rawliteral(...)") below confuse
// the Arduino IDE's automatic prototype generator, so nothing after them
// gets an auto-prototype. Declaring everything here manually avoids
// "not declared in this scope" errors regardless of definition order.
void oledInit();
void oledShowConfigMode();
void oledShowConnecting(const String &ssid, int attempt);
void oledShowOperational();
void ledStripInit();
void ledFill(uint32_t c);
void ledChaseStep(uint32_t c);
void updateLedStrip();
void relayInit();
void relayWrite(bool on);
void updateRelay();
int soilPercentFromRaw(uint16_t raw);
void sdInit();
void logToSD();
void handleSetGeofence();
void handleSetTilt();
void handleTile();
void handleNotFound();
void handleDownloadLog();
void handleScan();
void handleConfigRoot();
void handleSaveWifi();
void handleResetWifi();
void startConfigPortal();
bool tryConnectSavedWifi();
void processByte(uint8_t b);
void onPacketReceived();
float haversineMeters(float lat1, float lon1, float lat2, float lon2);
String epochToTimeStr(unsigned long epoch);
void addEvent(String level, unsigned long epoch);
void computeRisk();
String buildEventLogJson();
void handleData();
void handleRoot();
void handleRecalibrate();
void handleHistory();

// ---------------- Tilt calibration (box orientation baseline) ----------------
// The box sits on sloped/uneven ground, so raw pitch/roll is never really "0".
// We record the orientation on first packet after boot as the baseline, and
// everything else is reported/scored relative to that baseline - i.e. did the
// box move from however it was originally sitting, not "is it perfectly level".
bool calibrated = false;
float baselinePitch = 0;
float baselineRoll = 0;
float relPitch = 0;
float relRoll = 0;
bool boxMovedAlert = false;
unsigned long boxMovedAlertUntil = 0; // millis() timestamp - alert stays true
                                      // until this time (see ALERT_HOLD_MS)
float tiltThresholdDeg =
    8.0; // combined |relPitch|+|relRoll| beyond this = flagged as moved/knocked
         // - now runtime-settable, see /settilt

String staIP = "";
String staSSID = "";

#pragma pack(1)
struct SensorPacket {
  uint32_t timestamp;
  uint32_t rtcEpoch;
  float accelX, accelY, accelZ;
  float gyroX, gyroY, gyroZ;
  float pitch, roll;
  float temperature;
  float humidity;
  uint8_t vib1;
  uint16_t soil1;
  float latitude;
  float longitude;
  uint8_t gpsFix;
  uint8_t satellites;
};
#pragma pack()

SensorPacket packet;
const uint8_t PACKET_LEN = sizeof(SensorPacket);
bool havePacket = false;

// Simple state machine for frame sync
enum RxState { WAIT_HDR1, WAIT_HDR2, WAIT_LEN, WAIT_PAYLOAD, WAIT_CHK };
RxState state = WAIT_HDR1;
uint8_t payloadBuf[96]; // must be >= sizeof(SensorPacket)
uint8_t payloadIdx = 0;
uint8_t expectedLen = 0;

unsigned long lastPacketTime = 0;
unsigned long packetCount = 0;
unsigned long checksumErrors = 0;

// ---------------- Risk engine state ----------------
#define SOIL_HISTORY_SIZE 5
uint16_t soilHistory1[SOIL_HISTORY_SIZE];
uint8_t soilHistIdx = 0;
bool soilHistFilled = false;

#define VIB_WINDOW 10
uint8_t vibHistory[VIB_WINDOW];
uint8_t vibHistIdx = 0;
float vibRateScore = 0;
bool vibrationAlert = false; // stays true for ALERT_HOLD_MS after the last hit
                             // (sensor is buried, so any hit matters)
unsigned long vibrationAlertUntil =
    0; // millis() timestamp - alert stays true until this time

// ---------------- Soil moisture calibration ----------------
// Capacitive soil sensors read HIGH (near SOIL_RAW_DRY) in open air / dry soil
// and LOW (near SOIL_RAW_WET) when fully saturated/submerged. These are
// reasonable starting defaults - for an accurate %, dip the probe in water and
// note the raw reading shown in the Admin View for SOIL_RAW_WET, then let it
// sit in dry air/soil and note that raw reading for SOIL_RAW_DRY, and update
// the two numbers below.
#define SOIL_RAW_DRY 3000
#define SOIL_RAW_WET 1200
int soilPercent1 = 0;

bool homeSet = false;
float homeLat = 0, homeLng = 0;
// If true, homeLat/homeLng were typed in manually via /sethome (e.g. after
// moving this compact unit to a new deployment site) and should NOT be
// overwritten by the next GPS fix. If false (the normal/default case), home
// is auto-captured from the first GPS fix after boot, same as always.
bool homeIsManual = false;
float distanceFromHome = 0;
// Geofence radius - how far the GPS can drift from its "home" point (first
// fix after boot) before it counts as ground displacement. This used to be a
// fixed #define; it's now a runtime variable so it can be changed live from
// the dashboard's Admin View (Set Geofence) without re-flashing, and it's
// saved to flash (Preferences) so it survives a reboot.
#define GPS_DRIFT_THRESHOLD_M_DEFAULT                                          \
  8.0 // used only the very first time the board boots (nothing saved yet)
float geofenceRadiusM = GPS_DRIFT_THRESHOLD_M_DEFAULT;
#define GPS_DRIFT_STREAK_NEEDED                                                \
  3 // must see drift on 3 readings in a row before alerting
uint8_t gpsDriftStreak = 0;
bool gpsDriftAlert = false;

// A GPS drift alert on its own can be a fluke (multipath reflections, cheap
// module jitter, momentarily poor fix). landslideWarning only fires when the
// GPS drift is CONFIRMED by the MPU - i.e. the ground position moved AND the
// box itself also felt tilt/roll or vibration around the same time. This is
// the strongest, least false-positive-prone alert the system raises.
bool landslideWarning = false;
bool prevLandslideWarning = false;

bool rainfallLikely = false;

int riskScore = 0;
String riskLevel = "SAFE";
String riskColor = "#2ecc71";
String riskMessage = "Everything looks normal.";
String prevRiskLevel = "";

#define EVENT_LOG_SIZE 8
String eventLogTimes[EVENT_LOG_SIZE];
String eventLogLevels[EVENT_LOG_SIZE];
uint8_t eventLogCount = 0;

// ============================================================
// WIFI CONFIG PORTAL PAGE (served only in STATE_AP_CONFIG)
// ============================================================
const char CONFIG_PAGE_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Landslide Monitor Setup</title>
<style>
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body { font-family: -apple-system, Segoe UI, Roboto, Arial, sans-serif; background:#f4f6f8; color:#222; padding:20px; max-width:420px; margin:0 auto; }
  h1 { font-size:20px; text-align:center; margin-bottom:4px; color:#333; }
  .subtitle { text-align:center; color:#888; font-size:13px; margin-bottom:20px; }
  .card { background:white; border-radius:14px; padding:16px; box-shadow:0 1px 4px rgba(0,0,0,0.08); margin-bottom:16px; }
  .net-row { display:flex; justify-content:space-between; align-items:center; padding:10px 8px; border-bottom:1px solid #eee; cursor:pointer; }
  .net-row:last-child { border-bottom:none; }
  .net-row.selected { background:#eaf6ff; border-radius:8px; }
  .net-name { font-size:14px; font-weight:600; }
  .net-bars { font-size:12px; color:#888; }
  label { font-size:12px; color:#666; display:block; margin:12px 0 4px; }
  input[type=text], input[type=password] { width:100%; padding:12px; border:1px solid #ddd; border-radius:8px; font-size:14px; }
  button { display:block; width:100%; padding:14px; border:none; border-radius:10px; background:#34495e; color:white; font-size:15px; font-weight:600; cursor:pointer; margin-top:16px; }
  button:active { background:#2c3e50; }
  #status { text-align:center; font-size:13px; margin-top:10px; color:#888; }
  .rescan { text-align:center; font-size:12px; color:#3498db; margin-top:8px; cursor:pointer; }
</style>
</head>
<body>
  <h1>Landslide Monitor Setup</h1>
  <div class="subtitle">Connect the receiver to your WiFi</div>

  <div class="card">
    <div id="netList">Scanning nearby WiFi...</div>
    <div class="rescan" onclick="scanNetworks()">Rescan</div>
  </div>

  <div class="card">
    <label>WiFi Name (SSID)</label>
    <input type="text" id="ssid" placeholder="Selected network or type manually">
    <label>Password</label>
    <input type="password" id="password" placeholder="Leave blank if open network">
    <button onclick="saveWifi()">Connect</button>
    <div id="status"></div>
  </div>

<script>
function scanNetworks() {
  document.getElementById('netList').textContent = 'Scanning nearby WiFi...';
  fetch('/scan').then(r => r.json()).then(list => {
    if (!list.length) { document.getElementById('netList').textContent = 'No networks found. Try rescan.'; return; }
    document.getElementById('netList').innerHTML = list.map(n =>
      '<div class="net-row" onclick="pick(this,\'' + n.ssid.replace(/'/g,"\\'") + '\')">' +
        '<span class="net-name">' + n.ssid + (n.secure ? ' 🔒' : '') + '</span>' +
        '<span class="net-bars">' + n.rssi + ' dBm</span>' +
      '</div>'
    ).join('');
  }).catch(() => { document.getElementById('netList').textContent = 'Scan failed. Try rescan.'; });
}
function pick(el, ssid) {
  document.querySelectorAll('.net-row').forEach(r => r.classList.remove('selected'));
  el.classList.add('selected');
  document.getElementById('ssid').value = ssid;
}
function saveWifi() {
  const ssid = document.getElementById('ssid').value.trim();
  const password = document.getElementById('password').value;
  if (!ssid) { document.getElementById('status').textContent = 'Enter a WiFi name first.'; return; }
  document.getElementById('status').textContent = 'Saving and connecting... the board will restart.';
  fetch('/savewifi', {
    method: 'POST',
    headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
    body: 'ssid=' + encodeURIComponent(ssid) + '&password=' + encodeURIComponent(password)
  }).then(() => {
    document.getElementById('status').textContent = 'Saved. Reconnect your phone to your normal WiFi, then check the OLED screen on the device for the new address.';
  }).catch(() => { document.getElementById('status').textContent = 'Could not reach the board.'; });
}
scanNetworks();
</script>
</body>
</html>
)rawliteral";

// ============================================================
// DASHBOARD PAGE (served only in STATE_OPERATIONAL)
// ============================================================
const char PAGE_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Landslide Monitor</title>
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"/>
<style>
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body {
    font-family: -apple-system, Segoe UI, Roboto, Arial, sans-serif;
    background: #f4f6f8;
    color: #222;
    padding: 20px;
    max-width: 600px;
    margin: 0 auto;
  }
  h1 { font-size: 22px; text-align: center; margin-bottom: 4px; color: #333; }
  .subtitle { text-align: center; color: #888; font-size: 13px; margin-bottom: 20px; }
  .status-card {
    border-radius: 16px;
    padding: 30px 20px;
    text-align: center;
    color: white;
    margin-bottom: 16px;
    transition: background 0.5s ease;
  }
  .status-level { font-size: 40px; font-weight: 800; letter-spacing: 1px; }
  .status-message { font-size: 16px; margin-top: 10px; opacity: 0.95; }
  .link-banner {
    text-align: center;
    padding: 8px;
    border-radius: 8px;
    font-size: 13px;
    margin-bottom: 16px;
    font-weight: 600;
  }
  .link-up { background: #e8f8f0; color: #1e8449; }
  .link-down { background: #fdecea; color: #c0392b; }
  .alert-strip { display:flex; flex-direction:column; gap:8px; margin-bottom:16px; }
  .alert-chip { display:none; padding:12px 14px; border-radius:10px; font-size:14px; font-weight:600; align-items:center; gap:8px; }
  .alert-chip.show { display:flex; }
  @keyframes pulseAlert { 0%{opacity:1;} 50%{opacity:0.55;} 100%{opacity:1;} }
  .chip-vib { background:#fff2e0; color:#a5610a; animation: pulseAlert 1.3s infinite; }
  .chip-full { background:#fdecea; color:#c0392b; border:2px solid #c0392b; animation: pulseAlert 1s infinite; }
  .chip-landslide { background:#c0392b; color:white; font-weight:800; border:3px solid #7b241c; animation: pulseAlert 0.6s infinite; }
  .chip-gps { background:#fdf2e3; color:#a5610a; }
  .chip-rain { background:#e8f2fd; color:#1a5fa8; }
  .chip-siren { background:#2c3e50; color:white; animation: pulseAlert 0.8s infinite; }
  .mini-grid {
    display: grid;
    grid-template-columns: repeat(3, minmax(0, 1fr));
    gap: 12px;
    margin-bottom: 20px;
  }
  .mini-card {
    background: white;
    border-radius: 12px;
    padding: 16px;
    text-align: center;
    box-shadow: 0 1px 4px rgba(0,0,0,0.06);
  }
  .mini-value { font-size: 26px; font-weight: 700; color: #333; }
  .mini-label { font-size: 12px; color: #888; margin-top: 4px; }
  #map { width:100%; height:220px; border-radius:12px; margin-bottom:20px; box-shadow: 0 1px 4px rgba(0,0,0,0.06); }
  .map-note { text-align:center; font-size:12px; color:#888; margin:-12px 0 20px; }
  button {
    display: block;
    width: 100%;
    padding: 14px;
    border: none;
    border-radius: 10px;
    background: #34495e;
    color: white;
    font-size: 15px;
    font-weight: 600;
    cursor: pointer;
    margin-bottom: 20px;
  }
  button:active { background: #2c3e50; }
  #adminPanel { display: none; }
  .admin-table {
    width: 100%;
    background: white;
    border-radius: 12px;
    overflow: hidden;
    box-shadow: 0 1px 4px rgba(0,0,0,0.06);
    margin-bottom: 20px;
  }
  .admin-table td {
    padding: 10px 14px;
    border-bottom: 1px solid #eee;
    font-size: 13px;
  }
  .admin-table td:first-child { color: #888; }
  .admin-table td:last-child { text-align: right; font-weight: 600; color: #222; }
  .admin-table tr:last-child td { border-bottom: none; }
  .section-title { font-size: 13px; font-weight: 700; color: #555; margin: 16px 0 8px; text-transform: uppercase; letter-spacing: 0.5px; }
  .footer { text-align: center; font-size: 11px; color: #aaa; margin-top: 10px; }
  #eventLogList { background: white; border-radius: 12px; box-shadow: 0 1px 4px rgba(0,0,0,0.06); margin-bottom: 20px; overflow: hidden; }
  .event-row { display: flex; justify-content: space-between; padding: 10px 14px; border-bottom: 1px solid #eee; font-size: 13px; }
  .event-row:last-child { border-bottom: none; }
  #resetWifiBtn { background:#c0392b; }
  #resetWifiBtn:active { background:#96281b; }
  .horizon-wrap { display:flex; align-items:center; gap:16px; background:white; border-radius:12px; padding:16px; box-shadow: 0 1px 4px rgba(0,0,0,0.06); margin-bottom:12px; }
  #horizonSvg { border-radius:50%; flex-shrink:0; }
  .horizon-labels { display:flex; flex-direction:column; gap:12px; text-align:center; flex:1; }
</style>
</head>
<body>

  <h1>Landslide Monitor</h1>

  <div id="linkBanner" class="link-banner link-down">Connecting...</div>

  <div id="statusCard" class="status-card" style="background:#95a5a6;">
    <div id="statusLevel" class="status-level">--</div>
    <div id="statusMessage" class="status-message">Waiting for data...</div>
  </div>

  <div class="alert-strip">
    <div id="chipLandslide" class="alert-chip chip-landslide">🚨 LANDSLIDE WARNING — GPS shift confirmed by tilt/vibration. Evacuate the area.</div>
    <div id="chipMoved" class="alert-chip chip-full">📐 FULL ALERT — the box has tilted/rolled from its mounted position.</div>
    <div id="chipVib" class="alert-chip chip-vib">⚠️ Vibration detected underground — alerting for the next minute.</div>
    <div id="chipGps" class="alert-chip chip-gps">📡 GPS position drifting beyond the geofence — watching closely.</div>
    <div id="chipRain" class="alert-chip chip-rain">🌧️ Soil is getting wetter — rain may be softening the ground.</div>
    <div id="chipSiren" class="alert-chip chip-siren">🔊 Siren sounding</div>
  </div>

  <div class="mini-grid">
    <div class="mini-card">
      <div id="tempValue" class="mini-value">--°</div>
      <div class="mini-label">Temperature</div>
    </div>
    <div class="mini-card">
      <div id="humValue" class="mini-value">--%</div>
      <div class="mini-label">Humidity</div>
    </div>
    <div class="mini-card">
      <div id="soilValue" class="mini-value">--%</div>
      <div class="mini-label">Soil Moisture</div>
    </div>
  </div>


  <div id="map"></div>
  <div class="map-note" id="mapNote">Waiting for GPS fix...</div>

  <button onclick="toggleAdmin()">Admin View</button>

  <div id="adminPanel">

    <div class="section-title">Tilt / Orientation (relative to calibrated baseline)</div>
    <div class="horizon-wrap">
      <svg id="horizonSvg" viewBox="0 0 200 200" width="200" height="200">
        <defs>
          <clipPath id="horizonClip"><circle cx="100" cy="100" r="90"/></clipPath>
        </defs>
        <circle cx="100" cy="100" r="92" fill="#222" />
        <g clip-path="url(#horizonClip)">
          <g id="horizonGroup">
            <rect x="-100" y="-300" width="400" height="300" fill="#3a86ff" />
            <rect x="-100" y="0" width="400" height="300" fill="#8a5a2b" />
            <line x1="-100" y1="0" x2="300" y2="0" stroke="white" stroke-width="2"/>
          </g>
        </g>
        <circle cx="100" cy="100" r="90" fill="none" stroke="#555" stroke-width="4" />
        <line x1="60" y1="100" x2="85" y2="100" stroke="#ffd400" stroke-width="4" />
        <line x1="115" y1="100" x2="140" y2="100" stroke="#ffd400" stroke-width="4" />
        <polygon points="100,92 108,104 92,104" fill="#ffd400" />
      </svg>
      <div class="horizon-labels">
        <div><span class="mini-label">Pitch</span><div id="a_relpitch" class="mini-value" style="font-size:20px;">--</div></div>
        <div><span class="mini-label">Roll</span><div id="a_relroll" class="mini-value" style="font-size:20px;">--</div></div>
      </div>
    </div>
    <button id="recalBtn" onclick="recalibrate()" style="background:#3a86ff;">Recalibrate (set current position as level)</button>
    <table class="admin-table">
      <tr><td>Raw Pitch / Roll</td><td id="a_pitch">--</td></tr>
      <tr><td>Baseline Pitch / Roll</td><td id="a_baseline">--</td></tr>
      <tr><td>Box Moved Alert</td><td id="a_movedalert">--</td></tr>
      <tr><td>Accel X / Y / Z</td><td id="a_accel">--</td></tr>
      <tr><td>Gyro X / Y / Z</td><td id="a_gyro">--</td></tr>
    </table>

    <div class="section-title">Vibration</div>
    <table class="admin-table">
      <tr><td>Sensor (now)</td><td id="a_vib">--</td></tr>
      <tr><td>Vibration Rate Score</td><td id="a_vibrate">--</td></tr>
      <tr><td>Vibration Alert</td><td id="a_vibalert">--</td></tr>
    </table>

    <div class="section-title">Ground</div>
    <table class="admin-table">
      <tr><td>Soil Moisture (raw ADC)</td><td id="a_soil">--</td></tr>
      <tr><td>Soil Moisture (%)</td><td id="a_soilpct">--</td></tr>
      <tr><td>Rainfall Likely</td><td id="a_rain">--</td></tr>
    </table>

    <div class="section-title">GPS / Ground Movement (Geofence)</div>
    <table class="admin-table">
      <tr><td>Fix</td><td id="a_fix">--</td></tr>
      <tr><td>Lat / Lng</td><td id="a_latlng">--</td></tr>
      <tr><td>Satellites</td><td id="a_sats">--</td></tr>
      <tr><td>Home Locked</td><td id="a_home">--</td></tr>
      <tr><td>Distance From Home</td><td id="a_dist">--</td></tr>
      <tr><td>Geofence Radius</td><td id="a_geofence">--</td></tr>
      <tr><td>GPS Drift Alert</td><td id="a_gpsalert">--</td></tr>
      <tr><td>MPU Verified (Landslide)</td><td id="a_landslide">--</td></tr>
    </table>
    <div style="display:flex; gap:8px; margin-bottom:20px;">
      <input id="geofenceInput" type="number" min="1" max="500" step="0.5" placeholder="radius in meters"
        style="flex:1; padding:12px; border-radius:10px; border:1px solid #ccc; font-size:14px;">
      <button style="width:auto; margin-bottom:0; flex:1;" onclick="setGeofence()">Set Geofence</button>
    </div>

    <div class="section-title">Risk / Prediction Engine</div>
    <table class="admin-table">
      <tr><td>Risk Score</td><td id="a_score">--</td></tr>
      <tr><td>Packets Received</td><td id="a_count">--</td></tr>
      <tr><td>Checksum Errors</td><td id="a_errs">--</td></tr>
      <tr><td>Last Packet</td><td id="a_lastpkt">--</td></tr>
      <tr><td>Network</td><td id="a_wifi">--</td></tr>
      <tr><td>LED Strip</td><td id="a_led">--</td></tr>
    </table>

    <div class="section-title">SD Card Logger</div>
    <table class="admin-table">
      <tr><td>Card Status</td><td id="a_sdstatus">--</td></tr>
      <tr><td>Rows Logged</td><td id="a_sdrows">--</td></tr>
    </table>
    <button style="background:#3a86ff;" onclick="window.location.href='/downloadlog'">Download Log (CSV)</button>

    <div class="section-title">Event Log</div>
    <div id="eventLogList"><div class="event-row"><span>No events yet</span></div></div>

    <button id="resetWifiBtn" onclick="resetWifi()">Reset WiFi (back to setup mode)</button>


  </div>

  <div class="footer">Landslide Early-Warning System</div>

<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<script>
let adminOpen = false;
function toggleAdmin() {
  adminOpen = !adminOpen;
  document.getElementById('adminPanel').style.display = adminOpen ? 'block' : 'none';
  document.querySelector('button').textContent = adminOpen ? 'Hide Admin View' : 'Admin View';
}
function resetWifi() {
  if (!confirm('This will forget the saved WiFi and reboot into setup mode. Continue?')) return;
  fetch('/resetwifi', { method: 'POST' }).then(() => {
    alert('WiFi reset. The board is rebooting into setup mode (open network: Landslide-Setup).');
  });
}

function recalibrate() {
  if (!confirm('This sets the CURRENT orientation as the new "level" baseline. Only do this if the box is sitting the way you want it. Continue?')) return;
  fetch('/recalibrate', { method: 'POST' }).then(r => r.text()).then(msg => alert(msg));
}

function setGeofence() {
  const val = document.getElementById('geofenceInput').value;
  if (!val || isNaN(val)) { alert('Enter a radius in meters (1-500)'); return; }
  fetch('/setgeofence', {
    method: 'POST',
    headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
    body: 'radius=' + encodeURIComponent(val)
  }).then(r => r.text()).then(msg => alert(msg));
}

let map = null, marker = null, mapReady = false;
function ensureMap(lat, lng) {
  if (mapReady) return;
  map = L.map('map').setView([lat, lng], 17);
  L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
    attribution: '&copy; OpenStreetMap contributors',
    maxZoom: 19
  }).addTo(map);
  marker = L.marker([lat, lng]).addTo(map);
  mapReady = true;
}

function refresh() {
  fetch('/data').then(r => r.json()).then(d => {
    const banner = document.getElementById('linkBanner');
    if (d.linkUp) {
      banner.className = 'link-banner link-up';
      banner.textContent = 'Connected \u2014 live data';
    } else {
      banner.className = 'link-banner link-down';
      banner.textContent = 'Signal lost \u2014 last seen ' + d.secsSinceLast + 's ago';
    }

    document.getElementById('statusCard').style.background = d.riskColor;
    document.getElementById('statusLevel').textContent = d.riskLevel;
    document.getElementById('statusMessage').textContent = d.riskMessage;

    document.getElementById('chipVib').classList.toggle('show', !!d.vibrationAlert);
    document.getElementById('chipMoved').classList.toggle('show', false);
    document.getElementById('chipGps').classList.toggle('show', !!d.gpsDriftAlert);
    document.getElementById('chipRain').classList.toggle('show', !!d.rainfallLikely);
    document.getElementById('chipLandslide').classList.toggle('show', !!d.landslideWarning);
    document.getElementById('chipSiren').classList.toggle('show', !!(d.landslideWarning));

    document.getElementById('tempValue').textContent = d.temperature.toFixed(1) + '\u00b0C';
    document.getElementById('humValue').textContent = d.humidity.toFixed(0) + '%';
    document.getElementById('soilValue').textContent = d.soilPercent + '%';

    // Drone-style artificial horizon: roll rotates it, pitch shifts it vertically
    const pxPerDeg = 3.2;
    const horizonGroup = document.getElementById('horizonGroup');
    horizonGroup.setAttribute('transform',
      'rotate(' + (-d.relRoll).toFixed(1) + ' 100 100) translate(0 ' + (d.relPitch * pxPerDeg).toFixed(1) + ')');
    document.getElementById('a_relpitch').textContent = d.relPitch.toFixed(1) + '\u00b0';
    document.getElementById('a_relroll').textContent = d.relRoll.toFixed(1) + '\u00b0';

    document.getElementById('a_pitch').textContent = d.pitch.toFixed(1) + '\u00b0 / ' + d.roll.toFixed(1) + '\u00b0';
    document.getElementById('a_baseline').textContent = d.baselinePitch.toFixed(1) + '\u00b0 / ' + d.baselineRoll.toFixed(1) + '\u00b0';
    document.getElementById('a_movedalert').textContent = d.boxMovedAlert ? 'YES' : 'no';
    document.getElementById('a_accel').textContent = d.accelX.toFixed(2)+' / '+d.accelY.toFixed(2)+' / '+d.accelZ.toFixed(2)+' g';
    document.getElementById('a_gyro').textContent = d.gyroX.toFixed(1)+' / '+d.gyroY.toFixed(1)+' / '+d.gyroZ.toFixed(1)+' dps';

    document.getElementById('a_vib').textContent = d.vib1;
    document.getElementById('a_vibrate').textContent = d.vibRateScore.toFixed(0) + ' / 100';
    document.getElementById('a_vibalert').textContent = d.vibrationAlert ? 'YES' : 'no';

    document.getElementById('a_soil').textContent = d.soil1;
    document.getElementById('a_soilpct').textContent = d.soilPercent + '%';
    document.getElementById('a_rain').textContent = d.rainfallLikely ? 'YES' : 'no';

    document.getElementById('a_fix').textContent = d.gpsFix ? 'YES' : 'NO';
    document.getElementById('a_latlng').textContent = d.lat.toFixed(6) + ' / ' + d.lng.toFixed(6);
    document.getElementById('a_sats').textContent = d.sats;
    document.getElementById('a_home').textContent = d.homeSet ? 'YES' : 'Not yet (needs a GPS fix)';
    document.getElementById('a_dist').textContent = d.homeSet ? d.distanceFromHome.toFixed(1) + ' m' : '--';
    document.getElementById('a_geofence').textContent = d.geofenceRadius.toFixed(1) + ' m';
    document.getElementById('a_gpsalert').textContent = d.gpsDriftAlert ? 'YES' : 'no';
    document.getElementById('a_landslide').textContent = d.landslideWarning ? 'YES - GPS + MPU confirmed' : 'no';

    document.getElementById('a_score').textContent = d.riskScore + ' / 100';
    document.getElementById('a_count').textContent = d.packetCount;
    document.getElementById('a_errs').textContent = d.checksumErrors;
    document.getElementById('a_lastpkt').textContent = d.secsSinceLast + 's ago';
    document.getElementById('a_wifi').textContent = d.wifiSSID + ' (' + d.wifiIP + ')';
    document.getElementById('a_led').textContent = d.ledStatus;
    document.getElementById('a_sdstatus').textContent = d.sdOk ? 'OK - logging' : 'Not detected';
    document.getElementById('a_sdrows').textContent = d.logRows;

    if (d.gpsFix && d.lat !== 0 && d.lng !== 0) {
      document.getElementById('mapNote').style.display = 'none';
      ensureMap(d.lat, d.lng);
      if (mapReady) { marker.setLatLng([d.lat, d.lng]); map.panTo([d.lat, d.lng]); }
    }

    const logDiv = document.getElementById('eventLogList');
    if (d.eventLog && d.eventLog.length > 0) {
      logDiv.innerHTML = d.eventLog.slice().reverse().map(e =>
        '<div class="event-row"><span>' + e.time + '</span><span>' + e.level + '</span></div>'
      ).join('');
    }
  }).catch(() => {
    document.getElementById('linkBanner').className = 'link-banner link-down';
    document.getElementById('linkBanner').textContent = 'Connection error';
  });
}

setInterval(refresh, 1000);
refresh();
</script>
</body>
</html>
)rawliteral";

// ============================================================
// OLED HELPERS
// ============================================================
void oledInit() {
  Wire.begin(OLED_SDA, OLED_SCL);
  oledOk = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  if (!oledOk) {
    Serial.println(
        "[WARN] OLED not found at 0x3C - continuing without display");
    return;
  }
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Landslide Monitor");
  display.println("Booting...");
  display.display();
}

void oledShowConfigMode() {
  if (!oledOk)
    return;
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.println("SETUP MODE");
  display.println("--------------------");
  display.println("Connect WiFi to:");
  display.setTextSize(1);
  display.println(CONFIG_AP_SSID);
  display.println("(open, no password)");
  display.println("");
  display.println("Then open:");
  display.println("192.168.4.1");
  display.display();
}

void oledShowConnecting(const String &ssid, int attempt) {
  if (!oledOk)
    return;
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Connecting to:");
  display.println(ssid);
  display.println("");
  String dots = "";
  for (int i = 0; i < (attempt % 4); i++)
    dots += ".";
  display.println("Please wait" + dots);
  display.display();
}

void oledShowOperational() {
  if (!oledOk)
    return;
  display.clearDisplay();
  display.setTextSize(1);

  // WiFi network name (truncated so it can't run off the 128px-wide screen)
  String ssidShort = staSSID;
  if (ssidShort.length() > 20)
    ssidShort = ssidShort.substring(0, 17) + "...";
  display.setCursor(0, 0);
  display.print("WiFi: ");
  display.println(ssidShort);

  display.setCursor(0, 12);
  display.println("IP address:");

  // IP highlighted with an inverted bar instead of a bigger font size -
  // size 2 text physically can't fit a full IPv4 address on a 128px screen
  // (e.g. "192.168.100.105" at size 2 is ~192px wide), which is what was
  // causing the wrap/overlap. Size 1 always fits (max 15 chars = 90px).
  display.fillRect(0, 22, OLED_W, 10, SSD1306_WHITE);
  display.setTextColor(SSD1306_BLACK);
  display.setCursor(2, 24);
  display.print(staIP);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 36);
  display.println("(or landslide.local)");

  display.drawLine(0, 48, OLED_W, 48, SSD1306_WHITE);
  display.setCursor(0, 52);
  display.print("Risk:");
  display.print(riskLevel);
  display.print(" ");
  bool linkUp = havePacket && (millis() - lastPacketTime) < 5000;
  display.print(linkUp ? "OK" : "NO-LINK");

  display.display();
}

// ============================================================
// STATUS LED STRIP (7x WS2812B)
// ============================================================
void ledStripInit() {
  strip.begin();
  strip.setBrightness(90);
  COL_OFF = strip.Color(0, 0, 0);
  COL_BLUE = strip.Color(0, 80, 255);
  COL_YELLOW = strip.Color(255, 190, 0);
  COL_GREEN = strip.Color(0, 200, 0);
  COL_ORANGE = strip.Color(255, 80, 0);
  COL_RED = strip.Color(255, 0, 0);
  COL_WHITE = strip.Color(180, 180, 180);
  COL_PURPLE = strip.Color(150, 0, 220);
  COL_CYAN = strip.Color(0, 200, 200);
  strip.fill(COL_OFF);
  strip.show();
}

void ledFill(uint32_t c) {
  strip.fill(c);
  strip.show();
}

// Single moving pixel "chase" - used during setup mode and while connecting.
// Safe to call from inside a blocking wait loop since it does its own show().
void ledChaseStep(uint32_t c) {
  static uint8_t pos = 0;
  strip.clear();
  strip.setPixelColor(pos % NUM_LEDS, c);
  strip.show();
  pos++;
}

// Non-blocking status pattern, called every loop() iteration.
// Priority: no LoRa link > active vibration (buried sensor, any hit matters)
// > risk level color > GPS drift tint > calm green breathing.
void updateLedStrip() {
  unsigned long now = millis();
  static unsigned long lastStep = 0;
  static bool toggle = false;
  static uint8_t chasePos = 0;

  if (sysState == STATE_AP_CONFIG) {
    ledStatusText = "Setup mode (blue chase)";
    if (now - lastStep > 120) {
      lastStep = now;
      strip.clear();
      strip.setPixelColor(chasePos % NUM_LEDS, COL_BLUE);
      strip.show();
      chasePos++;
    }
    return;
  }

  bool linkUp = havePacket && (now - lastPacketTime) < 5000;

  if (!linkUp) {
    ledStatusText = "No signal (white blink)";
    if (now - lastStep > 500) {
      lastStep = now;
      toggle = !toggle;
      ledFill(toggle ? COL_WHITE : COL_OFF);
    }
    return;
  }

  // MPU-confirmed landslide warning: highest priority of all, all-pixel
  // very-fast red strobe so it's unmistakably different from a plain tilt
  // alert.
  if (landslideWarning) {
    ledStatusText =
        "LANDSLIDE WARNING - GPS + MPU confirmed (rapid red strobe)";
    if (now - lastStep > 60) {
      lastStep = now;
      toggle = !toggle;
      ledFill(toggle ? COL_RED : COL_OFF);
    }
    return;
  }

  // Tilt/roll ("box moved") takes next priority - it means the ground itself
  // shifted, not just a vibration hit. FULL ALERT: fast red strobe.
  if (boxMovedAlert) {
    ledStatusText = "FULL ALERT - tilt/roll detected (red strobe)";
    if (now - lastStep > 100) {
      lastStep = now;
      toggle = !toggle;
      ledFill(toggle ? COL_RED : COL_OFF);
    }
    return;
  }

  // Vibration alone: orange beacon (two pixels sweeping opposite sides,
  // like a rotating warning beacon) rather than a plain blink.
  if (vibrationAlert) {
    ledStatusText = "Vibration detected (orange beacon)";
    if (now - lastStep > 150) {
      lastStep = now;
      strip.clear();
      strip.setPixelColor(chasePos % NUM_LEDS, COL_ORANGE);
      strip.setPixelColor((chasePos + NUM_LEDS / 2) % NUM_LEDS, COL_ORANGE);
      strip.show();
      chasePos++;
    }
    return;
  }

  if (riskLevel == "DANGER") {
    ledStatusText = "DANGER (red strobe)";
    if (now - lastStep > 150) {
      lastStep = now;
      toggle = !toggle;
      ledFill(toggle ? COL_RED : COL_OFF);
    }
  } else if (riskLevel == "WARNING") {
    ledStatusText = "WARNING (orange blink)";
    if (now - lastStep > 400) {
      lastStep = now;
      toggle = !toggle;
      ledFill(toggle ? COL_ORANGE : COL_OFF);
    }
  } else if (riskLevel == "WATCH") {
    ledStatusText = gpsDriftAlert ? "WATCH + GPS drift (yellow/purple chase)"
                                  : "WATCH (yellow chase)";
    if (now - lastStep > 220) {
      lastStep = now;
      strip.clear();
      strip.setPixelColor(chasePos % NUM_LEDS,
                          gpsDriftAlert ? COL_PURPLE : COL_YELLOW);
      strip.show();
      chasePos++;
    }
  } else {
    // SAFE - gentle green breathing. gpsDriftAlert alone (without high score)
    // still gets a purple tint.
    if (gpsDriftAlert) {
      ledStatusText = "SAFE + GPS drift (purple blink)";
      if (now - lastStep > 400) {
        lastStep = now;
        toggle = !toggle;
        ledFill(toggle ? COL_PURPLE : COL_OFF);
      }
    } else if (rainfallLikely) {
      ledStatusText = "SAFE, rain likely (blue breathing)";
      float phase = (now % 3000) / 3000.0;
      uint8_t b = (uint8_t)(60 + 60 * sin(phase * 2 * PI));
      ledFill(strip.Color(0, b / 2, b));
    } else {
      ledStatusText = "SAFE (green breathing)";
      float phase = (now % 3000) / 3000.0;
      uint8_t b = (uint8_t)(60 + 60 * sin(phase * 2 * PI));
      ledFill(strip.Color(0, b, 0));
    }
  }
}

// ============================================================
// SD CARD DATA LOGGER
// ============================================================
// Logs one CSV row per received packet: a wall-clock timestamp (from the
// transmitter's RTC) plus every field the dashboard shows. If the card is
// missing or fails to mount, the board just carries on without logging -
// it never blocks the dashboard or LoRa link.
void sdInit() {
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  if (!SD.begin(SD_CS)) {
    Serial.println("[WARN] SD card not found - continuing without logging");
    sdOk = false;
    return;
  }
  sdOk = true;

  // Write a header row only if this is a brand new file (first boot with a
  // fresh/empty card). Existing logs are appended to, never overwritten.
  if (!SD.exists(LOG_FILENAME)) {
    File f = SD.open(LOG_FILENAME, FILE_WRITE);
    if (f) {
      f.println(
          "rtcEpoch,timeOfDay,pitch,roll,relPitch,relRoll,accelX,accelY,accelZ,"
          "gyroX,gyroY,gyroZ,temperature,humidity,vib1,soil1,soilPercent,"
          "gpsFix,latitude,longitude,satellites,distanceFromHome,"
          "riskScore,riskLevel,vibrationAlert,boxMovedAlert,gpsDriftAlert,"
          "landslideWarning");
      f.close();
    }
  }
  Serial.println("[SD] Card ready, logging to " LOG_FILENAME);
}

void logToSD() {
  if (!sdOk)
    return;
  File f = SD.open(LOG_FILENAME, FILE_APPEND);
  if (!f) {
    Serial.println("[WARN] SD write failed - marking card unavailable");
    sdOk = false;
    return;
  }
  f.print(packet.rtcEpoch);
  f.print(",");
  f.print(epochToTimeStr(packet.rtcEpoch));
  f.print(",");
  f.print(packet.pitch, 2);
  f.print(",");
  f.print(packet.roll, 2);
  f.print(",");
  f.print(relPitch, 2);
  f.print(",");
  f.print(relRoll, 2);
  f.print(",");
  f.print(packet.accelX, 3);
  f.print(",");
  f.print(packet.accelY, 3);
  f.print(",");
  f.print(packet.accelZ, 3);
  f.print(",");
  f.print(packet.gyroX, 2);
  f.print(",");
  f.print(packet.gyroY, 2);
  f.print(",");
  f.print(packet.gyroZ, 2);
  f.print(",");
  f.print(packet.temperature, 1);
  f.print(",");
  f.print(packet.humidity, 1);
  f.print(",");
  f.print(packet.vib1);
  f.print(",");
  f.print(packet.soil1);
  f.print(",");
  f.print(soilPercent1);
  f.print(",");
  f.print(packet.gpsFix);
  f.print(",");
  f.print(packet.latitude, 6);
  f.print(",");
  f.print(packet.longitude, 6);
  f.print(",");
  f.print(packet.satellites);
  f.print(",");
  f.print(distanceFromHome, 1);
  f.print(",");
  f.print(riskScore);
  f.print(",");
  f.print(riskLevel);
  f.print(",");
  f.print(vibrationAlert ? 1 : 0);
  f.print(",");
  f.print(boxMovedAlert ? 1 : 0);
  f.print(",");
  f.print(gpsDriftAlert ? 1 : 0);
  f.print(",");
  f.println(landslideWarning ? 1 : 0);
  f.close();
  logRowsWritten++;
}

// ============================================================
// RELAY / BUZZER (SIREN)
// ============================================================
void relayInit() {
  pinMode(RELAY_PIN, OUTPUT);
  relayWrite(false);
}

// Respects RELAY_ACTIVE_HIGH so wiring either kind of relay module just
// means flipping that one #define, nothing else changes.
void relayWrite(bool on) {
  bool level = RELAY_ACTIVE_HIGH ? on : !on;
  digitalWrite(RELAY_PIN, level ? HIGH : LOW);
}

// Non-blocking siren, mirrors updateLedStrip()'s priority so the buzzer
// always matches what the LEDs are showing. Both patterns keep going for
// as long as boxMovedAlert / vibrationAlert stay latched (ALERT_HOLD_MS).
//   FULL ALERT (tilt/roll)  -> fast continuous wail
//   Vibration only          -> short double-beep, repeating
void updateRelay() {
  unsigned long now = millis();
  bool linkUp = havePacket && (now - lastPacketTime) < 5000;

  if (sysState != STATE_OPERATIONAL || !linkUp) {
    relayWrite(false);
    return;
  }

  if (landslideWarning) {
    relayWrite(true); // continuous solid tone - the most urgent pattern
    return;
  }

  if (boxMovedAlert) {
    relayWrite((now % 240) < 120); // rapid on/off wail, ~4 times/sec
    return;
  }

  if (vibrationAlert) {
    // vibration = alert only, no buzzer/siren
    relayWrite(false);
    return;
  }

  relayWrite(false);
}

// ============================================================
// WIFI PROVISIONING
// ============================================================
void handleScan() {
  int n = WiFi.scanNetworks();
  String json = "[";
  for (int i = 0; i < n; i++) {
    if (i > 0)
      json += ",";
    String ssid = WiFi.SSID(i);
    ssid.replace("\"", "");
    json += "{\"ssid\":\"" + ssid + "\",\"rssi\":" + String(WiFi.RSSI(i)) +
            ",\"secure\":" +
            (WiFi.encryptionType(i) == WIFI_AUTH_OPEN ? "false" : "true") + "}";
  }
  json += "]";
  server.send(200, "application/json", json);
}

void handleConfigRoot() { server.send(200, "text/html", CONFIG_PAGE_HTML); }

void handleSaveWifi() {
  String ssid = server.arg("ssid");
  String password = server.arg("password");
  if (ssid.length() == 0) {
    server.send(400, "text/plain", "SSID required");
    return;
  }
  prefs.putString("ssid", ssid);
  prefs.putString("pass", password);
  server.send(200, "text/plain", "Saved. Rebooting.");
  Serial.println("[WIFI] Saved new credentials, restarting in 1s...");
  delay(1000);
  ESP.restart();
}

void handleResetWifi() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  prefs.remove("ssid");
  prefs.remove("pass");
  server.send(200, "text/plain", "WiFi reset. Rebooting.");
  Serial.println("[WIFI] Credentials cleared, restarting in 1s...");
  delay(1000);
  ESP.restart();
}

void handleRecalibrate() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  if (havePacket) {
    baselinePitch = packet.pitch;
    baselineRoll = packet.roll;
    calibrated = true;
    Serial.print("[CAL] Manual recalibration: pitch=");
    Serial.print(baselinePitch, 1);
    Serial.print(" roll=");
    Serial.println(baselineRoll, 1);
    server.send(200, "text/plain", "Recalibrated");
  } else {
    server.send(409, "text/plain", "No packet received yet - can't calibrate");
  }
}

// Lets you set the geofence radius (meters) live from the Admin View instead
// of re-flashing firmware. Saved to flash so it survives a reboot.
void handleSetGeofence() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  if (!server.hasArg("radius")) {
    server.send(400, "text/plain", "radius required");
    return;
  }
  float r = server.arg("radius").toFloat();
  if (r < 1.0 || r > 500.0) {
    server.send(400, "text/plain", "Radius must be between 1 and 500 meters");
    return;
  }
  geofenceRadiusM = r;
  gpsDriftStreak = 0; // don't carry an old streak across a threshold change
  prefs.putFloat("geofence", r);
  Serial.print("[CFG] Geofence radius set to ");
  Serial.print(r, 1);
  Serial.println(" m");
  server.send(200, "text/plain", "Geofence set to " + String(r, 1) + " m");
}

// Lets you type in the home/site coordinates directly from the dashboard -
// e.g. right after moving this compact unit to a new deployment spot,
// instead of waiting for a GPS fix or editing any files. Saved to flash so
// it survives a reboot, and takes priority over auto-capture from GPS until
// you explicitly clear it with /resethome.
void handleSetHome() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  if (!server.hasArg("lat") || !server.hasArg("lng")) {
    server.send(400, "text/plain", "lat and lng required");
    return;
  }
  float lat = server.arg("lat").toFloat();
  float lng = server.arg("lng").toFloat();
  if (lat < -90.0 || lat > 90.0 || lng < -180.0 || lng > 180.0) {
    server.send(400, "text/plain",
                "lat must be -90..90 and lng must be -180..180");
    return;
  }
  homeLat = lat;
  homeLng = lng;
  homeSet = true;
  homeIsManual = true;
  distanceFromHome = 0;
  gpsDriftStreak = 0;
  prefs.putFloat("homeLat", lat);
  prefs.putFloat("homeLng", lng);
  prefs.putBool("homeManual", true);
  Serial.print("[CFG] Home location set manually to ");
  Serial.print(lat, 6);
  Serial.print(", ");
  Serial.println(lng, 6);
  server.send(200, "text/plain",
              "Home set to " + String(lat, 6) + ", " + String(lng, 6));
}

// Clears a manually-typed home location and goes back to normal behavior:
// the NEXT GPS fix becomes the new home point. Use this after physically
// moving the unit somewhere new if you'd rather just let GPS re-lock than
// type in exact coordinates.
void handleResetHome() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  homeSet = false;
  homeIsManual = false;
  distanceFromHome = 0;
  gpsDriftStreak = 0;
  prefs.putBool("homeManual", false);
  Serial.println(
      "[CFG] Home location cleared - next GPS fix will set a new home");
  server.send(200, "text/plain",
              "Home cleared - next GPS fix becomes the new home point");
}

// Lets you set the tilt alert threshold (degrees, combined
// |relPitch|+|relRoll|) live from the dashboard instead of re-flashing
// firmware. Saved to flash so it survives a reboot. This is what flips
// boxMovedAlert to "excessive".
void handleSetTilt() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  if (!server.hasArg("threshold")) {
    server.send(400, "text/plain", "threshold required");
    return;
  }
  float t = server.arg("threshold").toFloat();
  if (t < 1.0 || t > 90.0) {
    server.send(400, "text/plain",
                "Threshold must be between 1 and 90 degrees");
    return;
  }
  tiltThresholdDeg = t;
  prefs.putFloat("tiltdeg", t);
  Serial.print("[CFG] Tilt alert threshold set to ");
  Serial.print(t, 1);
  Serial.println(" deg");
  server.send(200, "text/plain",
              "Tilt threshold set to " + String(t, 1) + " deg");
}

// Serves the raw CSV log file for download from the Admin View.
void handleDownloadLog() {
  if (!sdOk || !SD.exists(LOG_FILENAME)) {
    server.send(
        404, "text/plain",
        "No log file available (SD card not present or not yet written)");
    return;
  }
  File f = SD.open(LOG_FILENAME, FILE_READ);
  if (!f) {
    server.send(500, "text/plain", "Could not open log file");
    return;
  }
  server.streamFile(f, "text/csv");
  f.close();
}

// ---- Offline map tiles ----
// Serves standard XYZ map tiles (the same tile scheme Google/OSM/Leaflet use)
// straight off the SD card, so the dashboard's map works with zero internet
// access at the deployment site. Put tiles on the card as:
//   /tiles/<z>/<x>/<y>.png
// e.g. /tiles/16/54823/32987.png
// Use the included download_tiles.py (run once, at home, with internet) to
// pre-fetch the tiles for your site's coordinates in that exact folder layout,
// then copy the whole "tiles" folder onto the SD card root.
// Caught via onNotFound since ESP32 WebServer has no wildcard route matching.
void handleTile() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  String uri = server.uri(); // e.g. /tiles/16/54823/32987.png
  if (!uri.startsWith("/tiles/")) {
    server.send(404, "text/plain", "Not found");
    return;
  }
  if (!sdOk) {
    server.send(404, "image/png",
                ""); // Leaflet just treats a missing tile as blank
    return;
  }
  if (!SD.exists(uri)) {
    server.send(404, "image/png", "");
    return;
  }
  File f = SD.open(uri, FILE_READ);
  if (!f) {
    server.send(404, "image/png", "");
    return;
  }
  server.streamFile(f, "image/png");
  f.close();
}

// Anything not matched by an explicit server.on() route falls through here -
// used only to catch /tiles/<z>/<x>/<y>.png requests, since WebServer can't
// register a wildcard route directly.
void handleNotFound() {
  if (server.uri().startsWith("/tiles/")) {
    handleTile();
    return;
  }
  server.send(404, "text/plain", "Not found");
}

void startConfigPortal() {
  sysState = STATE_AP_CONFIG;
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(apIP, apGateway, apSubnet);
  WiFi.softAP(CONFIG_AP_SSID); // open network, no password arg

  server.on("/", handleConfigRoot);
  server.on("/scan", handleScan);
  server.on("/savewifi", HTTP_POST, handleSaveWifi);
  server.begin();

  Serial.println("=== SETUP MODE ===");
  Serial.print("Connect WiFi to: ");
  Serial.println(CONFIG_AP_SSID);
  Serial.print("Then open: http://");
  Serial.println(WiFi.softAPIP());

  oledShowConfigMode();
}

// Tries the saved WiFi credentials. Returns true and starts the
// operational web server on success. Returns false if nothing is
// saved, or the connection attempt times out.
bool tryConnectSavedWifi() {
  String ssid = prefs.getString("ssid", "");
  String password = prefs.getString("pass", "");
  if (ssid.length() == 0)
    return false;

  Serial.print("[WIFI] Trying saved network: ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  if (password.length() > 0)
    WiFi.begin(ssid.c_str(), password.c_str());
  else
    WiFi.begin(ssid.c_str());

  unsigned long start = millis();
  int attempt = 0;
  while (WiFi.status() != WL_CONNECTED && millis() - start < 20000) {
    delay(500);
    Serial.print(".");
    oledShowConnecting(ssid, attempt++);
    ledChaseStep(COL_YELLOW);
  }
  Serial.println();

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[WIFI] Failed to connect within timeout.");
    WiFi.disconnect(true);
    return false;
  }

  staSSID = ssid;
  staIP = WiFi.localIP().toString();
  sysState = STATE_OPERATIONAL;

  if (MDNS.begin("landslide")) {
    Serial.println("[WIFI] mDNS started: http://landslide.local");
  }

  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/resetwifi", HTTP_POST, handleResetWifi);
  server.on("/recalibrate", HTTP_POST, handleRecalibrate);
  server.on("/setgeofence", HTTP_POST, handleSetGeofence);
  server.on("/sethome", HTTP_POST, handleSetHome);
  server.on("/resethome", HTTP_POST, handleResetHome);
  server.on("/settilt", HTTP_POST, handleSetTilt);
  server.on("/downloadlog", HTTP_GET, handleDownloadLog);
  server.on("/history", HTTP_GET, handleHistory);
  server.onNotFound(handleNotFound);
  server.begin();

  Serial.println("=== OPERATIONAL MODE ===");
  Serial.print("Joined: ");
  Serial.println(staSSID);
  Serial.print("Dashboard: http://");
  Serial.println(staIP);

  oledShowOperational();
  return true;
}

// ============================================================
// SETUP / LOOP
// ============================================================
void setup() {
  Serial.begin(115200);

  pinMode(M0, OUTPUT);
  pinMode(M1, OUTPUT);
  pinMode(LED, OUTPUT);
  digitalWrite(M0, LOW);
  digitalWrite(M1, LOW);
  E32Serial.begin(9600, SERIAL_8N1, 16, 17);

  for (int i = 0; i < SOIL_HISTORY_SIZE; i++)
    soilHistory1[i] = 0;
  for (int i = 0; i < VIB_WINDOW; i++)
    vibHistory[i] = 0;

  oledInit();
  ledStripInit();
  relayInit();
  sdInit();

  prefs.begin("wifi", false);
  geofenceRadiusM = prefs.getFloat("geofence", GPS_DRIFT_THRESHOLD_M_DEFAULT);
  tiltThresholdDeg = prefs.getFloat("tiltdeg", 8.0);
  homeIsManual = prefs.getBool("homeManual", false);
  if (homeIsManual) {
    homeLat = prefs.getFloat("homeLat", 0);
    homeLng = prefs.getFloat("homeLng", 0);
    homeSet = true;
    Serial.print("[CFG] Loaded manual home location from flash: ");
    Serial.print(homeLat, 6);
    Serial.print(", ");
    Serial.println(homeLng, 6);
  }
  if (!tryConnectSavedWifi()) {
    startConfigPortal();
  }
}

void loop() {
  while (E32Serial.available() > 0) {
    uint8_t b = E32Serial.read();
    processByte(b);
  }
  server.handleClient();
  updateLedStrip();
  updateRelay();

  if (sysState == STATE_OPERATIONAL) {
    // Watch for dropped WiFi and try a quiet reconnect
    static unsigned long lastWifiCheck = 0;
    if (millis() - lastWifiCheck > 10000) {
      lastWifiCheck = millis();
      if (WiFi.status() != WL_CONNECTED) {
        Serial.println("[WARN] WiFi dropped, attempting reconnect...");
        WiFi.reconnect();
      }
    }

    // Refresh OLED periodically (not every loop - avoid I2C spam)
    static unsigned long lastOled = 0;
    if (millis() - lastOled > 2000) {
      lastOled = millis();
      oledShowOperational();
    }

    // Warn in serial if nothing has arrived in a while
    static unsigned long lastWarn = 0;
    if (packetCount > 0 && millis() - lastPacketTime > 5000 &&
        millis() - lastWarn > 5000) {
      lastWarn = millis();
      Serial.println(
          "[WARN] No packet received in 5+ seconds - link may be down");
    }
  }
}

void processByte(uint8_t b) {
  switch (state) {
  case WAIT_HDR1:
    if (b == 0xAA)
      state = WAIT_HDR2;
    break;

  case WAIT_HDR2:
    if (b == 0x55)
      state = WAIT_LEN;
    else
      state = WAIT_HDR1;
    break;

  case WAIT_LEN:
    expectedLen = b;
    payloadIdx = 0;
    if (expectedLen == PACKET_LEN && expectedLen <= sizeof(payloadBuf)) {
      state = WAIT_PAYLOAD;
    } else {
      Serial.print("[ERR] Length mismatch, got ");
      Serial.print(expectedLen);
      Serial.print(" expected ");
      Serial.println(PACKET_LEN);
      state = WAIT_HDR1;
    }
    break;

  case WAIT_PAYLOAD:
    payloadBuf[payloadIdx++] = b;
    if (payloadIdx >= expectedLen) {
      state = WAIT_CHK;
    }
    break;

  case WAIT_CHK: {
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < expectedLen; i++)
      checksum ^= payloadBuf[i];
    if (checksum == b) {
      memcpy(&packet, payloadBuf, expectedLen);
      onPacketReceived();
    } else {
      checksumErrors++;
      Serial.println("[ERR] Checksum mismatch, packet dropped");
    }
    state = WAIT_HDR1;
    break;
  }
  }
}

void onPacketReceived() {
  havePacket = true;
  packetCount++;
  lastPacketTime = millis();

  digitalWrite(LED, HIGH);
  delay(20);
  digitalWrite(LED, LOW);

  Serial.println("------------------------------------------");
  Serial.print("PACKET #");
  Serial.print(packetCount);
  Serial.print("  t=");
  Serial.print(packet.timestamp);
  Serial.print("ms  rtcEpoch=");
  Serial.println(packet.rtcEpoch);

  Serial.print("MPU  Accel(g): X=");
  Serial.print(packet.accelX, 3);
  Serial.print(" Y=");
  Serial.print(packet.accelY, 3);
  Serial.print(" Z=");
  Serial.print(packet.accelZ, 3);
  Serial.print("   Gyro(dps): X=");
  Serial.print(packet.gyroX, 2);
  Serial.print(" Y=");
  Serial.print(packet.gyroY, 2);
  Serial.print(" Z=");
  Serial.println(packet.gyroZ, 2);

  Serial.print("TILT  Pitch=");
  Serial.print(packet.pitch, 1);
  Serial.print("deg  Roll=");
  Serial.print(packet.roll, 1);
  Serial.println("deg");

  Serial.print("DHT11  Temp=");
  Serial.print(packet.temperature, 1);
  Serial.print("C  Humidity=");
  Serial.print(packet.humidity, 1);
  Serial.println("%");

  Serial.print("VIB  1=");
  Serial.println(packet.vib1);

  Serial.print("SOIL  1=");
  Serial.println(packet.soil1);

  Serial.print("GPS  Fix=");
  Serial.print(packet.gpsFix ? "YES" : "NO");
  Serial.print("  Lat=");
  Serial.print(packet.latitude, 6);
  Serial.print(" Lng=");
  Serial.print(packet.longitude, 6);
  Serial.print("  Sats=");
  Serial.println(packet.satellites);

  computeRisk();
  logToSD();
}

// ============================================================
// RISK / PREDICTION ENGINE
// ============================================================
float haversineMeters(float lat1, float lon1, float lat2, float lon2) {
  const float R = 6371000.0;
  float dLat = radians(lat2 - lat1);
  float dLon = radians(lon2 - lon1);
  float a = sin(dLat / 2) * sin(dLat / 2) + cos(radians(lat1)) *
                                                cos(radians(lat2)) *
                                                sin(dLon / 2) * sin(dLon / 2);
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return R * c;
}

String epochToTimeStr(unsigned long epoch) {
  unsigned long secsOfDay = epoch % 86400UL;
  int hh = secsOfDay / 3600;
  int mm = (secsOfDay % 3600) / 60;
  int ss = secsOfDay % 60;
  char buf[9];
  snprintf(buf, sizeof(buf), "%02d:%02d:%02d", hh, mm, ss);
  return String(buf);
}

void addEvent(String level, unsigned long epoch) {
  if (eventLogCount >= EVENT_LOG_SIZE) {
    for (int i = 1; i < EVENT_LOG_SIZE; i++) {
      eventLogTimes[i - 1] = eventLogTimes[i];
      eventLogLevels[i - 1] = eventLogLevels[i];
    }
    eventLogCount = EVENT_LOG_SIZE - 1;
  }
  eventLogTimes[eventLogCount] = epochToTimeStr(epoch);
  eventLogLevels[eventLogCount] = level;
  eventLogCount++;
}

int soilPercentFromRaw(uint16_t raw) {
  long pct = map((long)raw, SOIL_RAW_DRY, SOIL_RAW_WET, 0, 100);
  return (int)constrain(pct, 0, 100);
}

void computeRisk() {
  // ---- Calibrate baseline orientation on the first packet after boot ----
  if (!calibrated) {
    baselinePitch = packet.pitch;
    baselineRoll = packet.roll;
    calibrated = true;
    Serial.print("[CAL] Baseline orientation set: pitch=");
    Serial.print(baselinePitch, 1);
    Serial.print(" roll=");
    Serial.println(baselineRoll, 1);
  }
  relPitch = packet.pitch - baselinePitch;
  relRoll = packet.roll - baselineRoll;

  // ---- Tilt, measured relative to the calibrated baseline (not raw/absolute)
  // ----
  float tilt = abs(relPitch) + abs(relRoll);
  float tiltScore = constrain((tilt / 45.0) * 100.0, 0, 100);
  if (tilt > tiltThresholdDeg) {
    boxMovedAlertUntil =
        millis() + ALERT_HOLD_MS; // (re)start the 1-minute hold
  }
  boxMovedAlert = (millis() < boxMovedAlertUntil);

  // ---- Vibration: sensor is buried, so ANY hit right now is a flag,
  // separate from the smoothed rate score used for the gradient risk score ----
  uint8_t vibNow = packet.vib1;
  if (vibNow > 0) {
    vibrationAlertUntil =
        millis() + ALERT_HOLD_MS; // (re)start the 1-minute hold
  }
  vibrationAlert = (millis() < vibrationAlertUntil);
  vibHistory[vibHistIdx] = vibNow;
  vibHistIdx = (vibHistIdx + 1) % VIB_WINDOW;
  uint16_t vibSum = 0;
  for (int i = 0; i < VIB_WINDOW; i++)
    vibSum += vibHistory[i];
  vibRateScore = constrain(((float)vibSum / VIB_WINDOW) * 100.0, 0, 100);

  // ---- Soil moisture trend (also doubles as a rainfall signal) ----
  float soilTrendScore = 0;
  if (soilHistFilled) {
    float delta1 = (float)packet.soil1 -
                   (float)soilHistory1[soilHistIdx % SOIL_HISTORY_SIZE];
    soilTrendScore = constrain((delta1 / 500.0) * 100.0, 0, 100);
  }
  soilHistory1[soilHistIdx % SOIL_HISTORY_SIZE] = packet.soil1;
  if (soilHistIdx == 0)
    soilHistFilled = true;
  soilPercent1 = soilPercentFromRaw(packet.soil1);

  // Rising soil moisture with no shaking or tilt change reads as rain, not
  // landslide
  rainfallLikely = (soilTrendScore > 15) && !vibrationAlert && tiltScore < 20;

  // ---- GPS displacement: debounced so normal GPS jitter (clouds, weather)
  // doesn't falsely trigger an alert. Needs sustained drift over several
  // readings. ----
  float displacementBonus = 0;
  gpsDriftAlert = false;
  if (packet.gpsFix) {
    if (!homeSet && !homeIsManual) {
      homeLat = packet.latitude;
      homeLng = packet.longitude;
      homeSet = true;
      distanceFromHome = 0;
      gpsDriftStreak = 0;
    } else {
      distanceFromHome =
          haversineMeters(homeLat, homeLng, packet.latitude, packet.longitude);
      if (distanceFromHome > geofenceRadiusM) {
        gpsDriftStreak++;
      } else {
        gpsDriftStreak = 0;
      }
      if (gpsDriftStreak >= GPS_DRIFT_STREAK_NEEDED) {
        gpsDriftAlert = true;
        displacementBonus = 20;
      }
    }
  }

  // ---- MPU-verified landslide warning ----
  // GPS drift by itself can be a false alarm (multipath, weak fix, module
  // jitter), so it only escalates to a full landslide warning when the MPU
  // corroborates it - i.e. the box also felt tilt/roll or vibration in the
  // same alert window (both are already latched for ALERT_HOLD_MS, so a hit
  // a little before or after the GPS reading still counts as corroboration).
  landslideWarning = gpsDriftAlert && (boxMovedAlert || vibrationAlert);
  if (landslideWarning && !prevLandslideWarning) {
    addEvent("LANDSLIDE-WARNING", packet.rtcEpoch);
  }
  prevLandslideWarning = landslideWarning;

  // ---- Combine into a weighted score. Vibration gets an immediate jump
  // on top of its smoothed contribution, since any underground shaking matters.
  // (waterScore term removed along with the sensor; soil's weight absorbs it)
  // ----
  float weighted =
      tiltScore * 0.20 + vibRateScore * 0.30 + soilTrendScore * 0.30;
  if (vibrationAlert)
    weighted += 25;
  riskScore = (int)constrain(weighted + displacementBonus, 0, 100);
  if (landslideWarning)
    riskScore =
        100; // MPU-confirmed ground displacement overrides everything else

  if (riskScore < 25) {
    riskLevel = "SAFE";
    riskColor = "#2ecc71";
    riskMessage = "Everything looks normal here. No danger right now.";
  } else if (riskScore < 50) {
    riskLevel = "WATCH";
    riskColor = "#f1c40f";
    riskMessage = "Small changes noticed. Keep an eye on this area.";
  } else if (riskScore < 75) {
    riskLevel = "WARNING";
    riskColor = "#e67e22";
    riskMessage = "Ground conditions are getting worse. Be careful here.";
  } else {
    riskLevel = "DANGER";
    riskColor = "#e74c3c";
    riskMessage = "High danger right now. Stay away from this area.";
  }
  if (landslideWarning) {
    riskMessage = "landslide, evacuate immediately";
  }

  if (riskLevel != prevRiskLevel) {
    addEvent(riskLevel, packet.rtcEpoch);
    prevRiskLevel = riskLevel;
  }
}

// ============================================================
// WEB HANDLERS (operational mode)
// ============================================================
String buildEventLogJson() {
  String s = "[";
  for (int i = 0; i < eventLogCount; i++) {
    if (i > 0)
      s += ",";
    s += "{\"time\":\"" + eventLogTimes[i] + "\",\"level\":\"" +
         eventLogLevels[i] + "\"}";
  }
  s += "]";
  return s;
}

// ---- Historical trend data (for Admin View charts / prediction) ----
// Reads the tail of the SD card CSV log and returns a compact JSON array of
// the fields the trend charts and simple trend-predictor need. This is what
// lets the Admin View show real history spanning beyond the current browser
// session, not just whatever's been polled live since the page was opened.
// Bounded to a max chunk of file bytes (HISTORY_CHUNK_BYTES) so a large log
// file can never risk running the ESP32 out of RAM while serving this.
#define HISTORY_CHUNK_BYTES 32768
#define HISTORY_LINE_CAP 350
#define HISTORY_MAX_ROWS 200

void handleHistory() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  if (!sdOk || !SD.exists(LOG_FILENAME)) {
    server.send(200, "application/json",
                "{\"rows\":[],\"note\":\"No SD log available yet\"}");
    return;
  }

  int wantRows = HISTORY_MAX_ROWS;
  if (server.hasArg("n")) {
    int n = server.arg("n").toInt();
    if (n > 0 && n < HISTORY_MAX_ROWS)
      wantRows = n;
  }

  File f = SD.open(LOG_FILENAME, FILE_READ);
  if (!f) {
    server.send(500, "application/json", "{\"rows\":[]}");
    return;
  }

  size_t fsize = f.size();
  size_t chunk = (fsize > (size_t)HISTORY_CHUNK_BYTES)
                     ? (size_t)HISTORY_CHUNK_BYTES
                     : fsize;
  size_t startPos = (fsize > chunk) ? (fsize - chunk) : 0;
  f.seek(startPos);

  String buf;
  buf.reserve(chunk + 32);
  while (f.available())
    buf += (char)f.read();
  f.close();

  // If we started mid-file (log bigger than the chunk), the very first line
  // read is likely a partial row - drop it rather than risk a garbled entry.
  if (startPos > 0) {
    int firstNL = buf.indexOf('\n');
    if (firstNL >= 0)
      buf = buf.substring(firstNL + 1);
  }

  // Pass 1: record where each usable data line starts (skip header/blank
  // lines).
  int lineStarts[HISTORY_LINE_CAP];
  int totalLines = 0;
  int pos = 0;
  int bufLen = buf.length();
  while (pos < bufLen && totalLines < HISTORY_LINE_CAP) {
    int nl = buf.indexOf('\n', pos);
    int end = (nl == -1) ? bufLen : nl;
    if (end > pos && !buf.startsWith("rtcEpoch", pos)) {
      lineStarts[totalLines++] = pos;
    }
    if (nl == -1)
      break;
    pos = nl + 1;
  }

  int rowsToSend = (totalLines < wantRows) ? totalLines : wantRows;
  int firstIdx = totalLines - rowsToSend;

  // Pass 2: parse only the rows we're actually going to send, straight into
  // JSON.
  String out = "{\"rows\":[";
  bool firstOut = true;
  for (int i = firstIdx; i < totalLines; i++) {
    int p0 = lineStarts[i];
    int nl = buf.indexOf('\n', p0);
    String line = (nl == -1) ? buf.substring(p0) : buf.substring(p0, nl);
    line.trim();
    if (line.length() == 0)
      continue;

    String field[28];
    int fCount = 0, p = 0;
    while (fCount < 28) {
      int c = line.indexOf(',', p);
      if (c == -1) {
        field[fCount++] = line.substring(p);
        break;
      }
      field[fCount++] = line.substring(p, c);
      p = c + 1;
    }
    // CSV column order (see sdInit's header row): 0 rtcEpoch, 4 relPitch,
    // 5 relRoll, 12 temperature, 13 humidity, 14 vib1, 16 soilPercent,
    // 21 distanceFromHome, 22 riskScore.
    if (fCount < 24 || field[0].length() == 0 || field[22].length() == 0)
      continue;

    if (!firstOut)
      out += ",";
    firstOut = false;
    float tiltDeg = fabs(field[4].toFloat()) + fabs(field[5].toFloat());
    out += "{\"t\":" + field[0] + ",\"risk\":" + field[22] +
           ",\"tilt\":" + String(tiltDeg, 1) + ",\"vib\":" + field[14] +
           ",\"soil\":" + field[16] + ",\"temp\":" + field[12] +
           ",\"hum\":" + field[13] + ",\"dist\":" + field[21] + "}";
  }
  out += "]}";

  server.send(200, "application/json", out);
}

void handleData() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  unsigned long secsSinceLast =
      havePacket ? (millis() - lastPacketTime) / 1000 : 9999;
  bool linkUp = havePacket && secsSinceLast < 5;

  char json[2600];
  snprintf(
      json, sizeof(json),
      "{"
      "\"linkUp\":%s,"
      "\"secsSinceLast\":%lu,"
      "\"packetCount\":%lu,"
      "\"checksumErrors\":%lu,"
      "\"freeHeap\":%u,"
      "\"wifiRSSI\":%d,"
      "\"uptimeSec\":%lu,"
      "\"riskScore\":%d,"
      "\"riskLevel\":\"%s\","
      "\"riskColor\":\"%s\","
      "\"riskMessage\":\"%s\","
      "\"vibrationAlert\":%s,"
      "\"gpsDriftAlert\":%s,"
      "\"landslideWarning\":%s,"
      "\"rainfallLikely\":%s,"
      "\"calibrated\":%s,"
      "\"baselinePitch\":%.1f,\"baselineRoll\":%.1f,"
      "\"relPitch\":%.1f,\"relRoll\":%.1f,"
      "\"boxMovedAlert\":%s,"
      "\"tiltThreshold\":%.1f,"
      "\"temperature\":%.1f,"
      "\"humidity\":%.1f,"
      "\"pitch\":%.1f,"
      "\"roll\":%.1f,"
      "\"accelX\":%.3f,\"accelY\":%.3f,\"accelZ\":%.3f,"
      "\"gyroX\":%.2f,\"gyroY\":%.2f,\"gyroZ\":%.2f,"
      "\"vib1\":%d,"
      "\"vibRateScore\":%.1f,"
      "\"soil1\":%d,"
      "\"soilPercent\":%d,"
      "\"gpsFix\":%d,\"lat\":%.6f,\"lng\":%.6f,\"sats\":%d,"
      "\"homeSet\":%s,\"distanceFromHome\":%.1f,"
      "\"homeLat\":%.6f,\"homeLng\":%.6f,\"homeIsManual\":%s,"
      "\"geofenceRadius\":%.1f,"
      "\"rtcEpoch\":%lu,"
      "\"wifiSSID\":\"%s\",\"wifiIP\":\"%s\","
      "\"ledStatus\":\"%s\","
      "\"sdOk\":%s,\"logRows\":%lu,"
      "\"eventLog\":%s"
      "}",
      linkUp ? "true" : "false", secsSinceLast, packetCount, checksumErrors,
      ESP.getFreeHeap(), WiFi.RSSI(), (unsigned long)(millis() / 1000UL),
      riskScore, riskLevel.c_str(), riskColor.c_str(), riskMessage.c_str(),
      vibrationAlert ? "true" : "false", gpsDriftAlert ? "true" : "false",
      landslideWarning ? "true" : "false", rainfallLikely ? "true" : "false",
      calibrated ? "true" : "false", baselinePitch, baselineRoll, relPitch,
      relRoll, boxMovedAlert ? "true" : "false", tiltThresholdDeg,
      packet.temperature, packet.humidity, packet.pitch, packet.roll,
      packet.accelX, packet.accelY, packet.accelZ, packet.gyroX, packet.gyroY,
      packet.gyroZ, packet.vib1, vibRateScore, packet.soil1, soilPercent1,
      packet.gpsFix, packet.latitude, packet.longitude, packet.satellites,
      homeSet ? "true" : "false", distanceFromHome, homeLat, homeLng,
      homeIsManual ? "true" : "false", geofenceRadiusM,
      (unsigned long)packet.rtcEpoch, staSSID.c_str(), staIP.c_str(),
      ledStatusText.c_str(), sdOk ? "true" : "false", logRowsWritten,
      buildEventLogJson().c_str());

  server.send(200, "application/json", json);
}

void handleRoot() { server.send(200, "text/html", PAGE_HTML); }
