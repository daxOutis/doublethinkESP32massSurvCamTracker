#include <Arduino.h>
#include <TinyGPSPlus.h>
#include <math.h>
#include <string.h>
#include <WiFi.h>
#include <WebServer.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include "secrets.h"

// ==================================================================
//                   GPS SERIAL CONFIGURATION
// ==================================================================
HardwareSerial gpsSerial(1);   // ESP32 UART1
#define GPS_RX 16               // GPS module TX -> here
#define GPS_TX 17               // GPS module RX -> here
#define GPS_BAUD 9600           // Standard NMEA baud rate

TinyGPSPlus gps;

// ==================================================================
//                   BUZZER CONFIGURATION
// ==================================================================
#define BUZZER_PIN 25
// A piezo is loudest near its resonant peak (~2-4 kHz), so dropping the tone
// well below that both lowers the pitch and takes the edge off the volume —
// there's no direct volume knob with tone(). Shorter on-time + longer gap
// makes it a slower, less naggy "beep ... beep" instead of a rapid chirp.
const int ALERT_FREQUENCY_HZ = 1200;
const unsigned long BEEP_ON_MS = 80;    // how long each beep sounds
const unsigned long BEEP_OFF_MS = 900;  // pause between beeps
bool inDangerZone = false;      // current zone state, used by the pulse below
bool beepSounding = false;      // whether we're in the "on" half of a beep pulse
unsigned long lastBeepToggle = 0;

// ==================================================================
//                   ACTUATOR RELAY CONFIGURATION
// ==================================================================
// A leftover 2-wire car door-lock actuator: a small DC gear motor with
// NO limit switches that throws a rod one way or the other depending
// on which way current flows through it. It must be PULSED, never held
// — keeping power on it at the end of travel stalls the motor and
// burns it out in under a minute.
//
// Two relays (a cheap dual board) wired as an H-bridge give both
// directions from a single 12 V supply:
//   both relays idle  -> both actuator wires grounded -> no movement
//   EXTEND relay only  -> throws one way   (entered a danger zone)
//   RETRACT relay only -> throws back      (danger has passed)
// The actuator follows the zone *state*, not the buzzer's beep pulse:
// exactly one timed pulse on entry, one on exit, nothing in between.
#define ACTUATOR_EXTEND_PIN 26
#define ACTUATOR_RETRACT_PIN 27
const unsigned long ACTUATOR_PULSE_MS = 100;  // length of each throw

// L298 IN pins are active-HIGH (HIGH = current flows that direction),
// unlike the cheap relay boards this was originally written for.
#define RELAY_ACTIVE_LOW 0
#if RELAY_ACTIVE_LOW
  #define RELAY_ON  LOW
  #define RELAY_OFF HIGH
#else
  #define RELAY_ON  HIGH
  #define RELAY_OFF LOW
#endif

// Edge detection + non-blocking pulse timing, same millis() approach
// as the buzzer so loop() never stalls and GPS bytes keep flowing.
bool actuatorExtended = false;         // last direction we threw it
bool actuatorPulsing = false;          // a relay is energized right now
unsigned long actuatorPulseStart = 0;  // when the current pulse began
int actuatorActivePin = -1;            // which relay pin is energized

// ==================================================================
//                   JOYSTICK (manual actuator override)
// ==================================================================
// A 5-pin analog joystick module (GND, VCC, VRX, VRY, SW) for throwing the
// actuator by hand — bench-verified in bench/actuator_test before being
// carried over here. Only VRX is read; VRY and SW don't need to be wired.
// VCC MUST go to 3.3V, not 5V: the ESP32's ADC tops out at 3.3V.
//
// GPIO34 is an ADC1 pin, not ADC2 — ADC2 shares hardware with the WiFi
// radio and becomes unreliable once WiFi is on, which this firmware's AP
// always is.
#define JOYSTICK_VRX_PIN 34
const int JOYSTICK_SAMPLES = 8;
const int JOYSTICK_DEADZONE_COUNTS = 600;
int joystickCenterX = 2048;
// True once the stick is back in the deadband — a push only fires a pulse
// the moment it crosses out of the deadband while armed, so holding it
// over doesn't repeat-fire; you have to return to center and push again.
bool joystickArmed = true;

int readJoystickX() {
  long sum = 0;
  for (int i = 0; i < JOYSTICK_SAMPLES; i++) sum += analogRead(JOYSTICK_VRX_PIN);
  return sum / JOYSTICK_SAMPLES;
}

// SW is the joystick's click button: a momentary switch to GND with no
// onboard pull-up, so it needs INPUT_PULLUP (idle = HIGH, pressed = LOW).
// GPIO33 is a normal pin, unlike 34-39, which can't do INPUT_PULLUP at
// all. A press toggles actuatorEnabled — a manual kill switch for the
// motor only: the buzzer and zone detection keep running either way, so
// you still get the audible alert with the motor disabled. Debounced in
// software since a mechanical switch can bounce through several
// transitions on one press.
#define JOYSTICK_SW_PIN 33
const unsigned long SW_DEBOUNCE_MS = 50;
bool actuatorEnabled = true;
int swRawState = HIGH;
int swStableState = HIGH;
unsigned long swLastChangeTime = 0;

// ==================================================================
//                   DANGER ZONES
// ==================================================================
// One trigger radius for every zone: how close (in meters) you have to get
// to a zone's center before it counts as "inside" and the buzzer starts.
// Starts at DEFAULT_ALERT_RADIUS_M and can be changed in the field from the
// web page (POST /settings) without a reflash. 50 m is a tight default so it
// only fires when you're genuinely on top of a spot.
#define DEFAULT_ALERT_RADIUS_M 75
int alertRadiusMeters = DEFAULT_ALERT_RADIUS_M;

// Just the coordinates — the alert only ever needs lat/lon to measure
// distance. An earlier design carried a `name` string per zone, but it
// was only used in a debug print, so it was dropped: 16 bytes/zone
// instead of 48, and the JSON loader no longer has to copy strings.
struct DangerZone {
  double lat;
  double lon;
};

// Fixed-capacity array + a count, instead of a compile-time-sized array,
// so the zone list can be replaced at runtime without
// recompiling. At 16 bytes/zone this array is 8 KB of RAM; the real
// ceiling is heap headroom while parsing an upload, which comfortably
// covers this many coordinate-only entries.
#define MAX_DANGER_ZONES 3000
DangerZone dangerZones[MAX_DANGER_ZONES];
int numDangerZones = 0;

// Copies a zone into dangerZones[]. Shared by loadDefaultZones() and the
// JSON loader (loadZonesFromFile()) so both paths fill the array the same
// way. Silently ignores zones past MAX_DANGER_ZONES.
void addDangerZone(double lat, double lon) {
  if (numDangerZones >= MAX_DANGER_ZONES) return;
  DangerZone& zone = dangerZones[numDangerZones];
  zone.lat = lat;
  zone.lon = lon;
  numDangerZones++;
}

// Fallback list, used only when there's no usable zones.json on flash (first
// boot, or a corrupt/empty file). These are placeholders — EDIT them to
// somewhere near you (or just upload a zones.json) so you can walk/drive
// into one and see the alert fire. The example below sits on the actual
// geographic center of the contiguous US, near Lebanon, Kansas — picked
// specifically because it's nowhere real, not a hint about anyone's
// actual location.
void loadDefaultZones() {
  addDangerZone(39.8283, -98.5795);   // example only — center of the US
  addDangerZone(39.8301, -98.5770);   // example only
  addDangerZone(39.8265, -98.5820);   // example only
}

// Great-circle distance between two lat/lon points, in meters (Haversine
// formula). This is what turns two coordinate pairs into "how far apart."
double haversineMeters(double lat1, double lon1, double lat2, double lon2) {
  const double earthRadiusMeters = 6371000.0;
  double dLat = radians(lat2 - lat1);
  double dLon = radians(lon2 - lon1);
  double a = sin(dLat / 2) * sin(dLat / 2) +
             cos(radians(lat1)) * cos(radians(lat2)) *
                 sin(dLon / 2) * sin(dLon / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return earthRadiusMeters * c;
}

// Start a single timed relay pulse in one direction. The pulse is
// ended later in loop() once ACTUATOR_PULSE_MS has passed — this never
// blocks. The opposite relay is left off, so the two are never
// energized together.
void startActuatorPulse(bool extend) {
  actuatorActivePin = extend ? ACTUATOR_EXTEND_PIN : ACTUATOR_RETRACT_PIN;
  digitalWrite(actuatorActivePin, RELAY_ON);
  actuatorPulsing = true;
  actuatorPulseStart = millis();
  Serial.println(extend ? "Actuator: EXTEND pulse" : "Actuator: RETRACT pulse");
}

bool checkDangerZones(double currentLat, double currentLon) {
  bool insideAny = false;
  for (int i = 0; i < numDangerZones; i++) {
    double distance = haversineMeters(currentLat, currentLon,
                                       dangerZones[i].lat, dangerZones[i].lon);
    if (distance <= alertRadiusMeters) {
      insideAny = true;
      Serial.print("!!! INSIDE DANGER ZONE #");
      Serial.print(i);
      Serial.print(" (");
      Serial.print(dangerZones[i].lat, 6);
      Serial.print(", ");
      Serial.print(dangerZones[i].lon, 6);
      Serial.print(") — ");
      Serial.print(distance, 1);
      Serial.println("m from center");
    }
  }
  return insideAny;
}

// ==================================================================
//                   WIFI AP + WEB SERVER
// ==================================================================
// Doublethink hosts its own access point instead of joining home/car WiFi,
// so zone updates work anywhere with no internet required.
WebServer server(80);

// Uploaded zones are written here first, under a temp name, and only
// swapped into place once the whole file has arrived *and* parses as valid
// JSON — so a dropped connection or a bad file never corrupts the last
// known-good zone list sitting on flash.
const char* ZONES_FILE = "/zones.json";
const char* ZONES_TMP_FILE = "/zones.json.tmp";

// Prepended to every page we serve. The viewport tag stops phones from
// rendering at desktop width and then shrinking it to fit (which is what
// made the text tiny); the font-size bump makes it comfortable to read.
const char* PAGE_HEAD =
    "<meta name='viewport' content='width=device-width, initial-scale=1'>"
    "<style>body{font-family:sans-serif;font-size:1.4rem;margin:1rem}"
    "input,button{font-size:1rem}</style>";

File uploadFile;
bool uploadHadError = false;

// ==================================================================
//                   PERSISTENCE
// ==================================================================
// Both things you can change in the field are kept on LittleFS (flash, not
// RAM) so they survive unplugging the device: the uploaded zone list in
// zones.json, and the alert radius in a tiny config.json we write ourselves.
const char* CONFIG_FILE = "/config.json";

// Parse /zones.json into dangerZones[]. Returns false — leaving the current
// in-memory list untouched — if the file is missing, unreadable, not valid
// JSON, or not a JSON array. Only once we know it's a good array do we clear
// the old list and refill it, so a bad file can't wipe working zones.
bool loadZonesFromFile() {
  File f = LittleFS.open(ZONES_FILE, "r");
  if (!f) {
    Serial.println("Zones: no zones.json on flash yet");
    return false;
  }
  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, f);
  f.close();
  if (err) {
    Serial.print("Zones: zones.json won't parse — ");
    Serial.println(err.c_str());
    return false;
  }
  if (!doc.is<JsonArray>()) {
    Serial.println("Zones: zones.json is not a JSON array");
    return false;
  }

  numDangerZones = 0;  // committed to replacing the list now
  int entryIndex = -1;
  for (JsonObject z : doc.as<JsonArray>()) {
    entryIndex++;
    // "| NAN" makes a missing or non-numeric field read back as NaN, so a
    // malformed entry gets skipped instead of dropping a zone at (0, 0).
    double lat = z["lat"] | NAN;
    double lon = z["lon"] | NAN;
    if (isnan(lat) || isnan(lon)) {
      Serial.print("Zones: skipping entry ");
      Serial.print(entryIndex);
      Serial.println(" with bad lat/lon");
      continue;
    }
    addDangerZone(lat, lon);
  }
  Serial.print("Zones: loaded ");
  Serial.print(numDangerZones);
  Serial.println(" from zones.json");
  return true;
}

// Fill dangerZones[] at boot and after each successful upload: use the saved
// file if it's usable, otherwise fall back to the built-in list so the
// device is never left with nothing to alert on.
void loadZones() {
  if (!loadZonesFromFile()) {
    numDangerZones = 0;
    loadDefaultZones();
    Serial.print("Zones: using ");
    Serial.print(numDangerZones);
    Serial.println(" built-in default zones");
  }
}

// Read the alert radius saved from a previous /settings change. Missing or
// bad file just means "keep the compiled-in default."
void loadConfig() {
  File f = LittleFS.open(CONFIG_FILE, "r");
  if (!f) return;
  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, f);
  f.close();
  if (err) {
    Serial.print("Config: config.json won't parse — ");
    Serial.println(err.c_str());
    return;
  }
  int r = doc["alertRadiusMeters"] | alertRadiusMeters;
  if (r >= 10 && r <= 5000) alertRadiusMeters = r;
  Serial.print("Config: alert radius = ");
  Serial.print(alertRadiusMeters);
  Serial.println(" m (from config.json)");
}

void saveConfig() {
  JsonDocument doc;
  doc["alertRadiusMeters"] = alertRadiusMeters;
  File f = LittleFS.open(CONFIG_FILE, "w");
  if (!f) {
    Serial.println("Config: failed to open config.json for writing");
    return;
  }
  serializeJson(doc, f);
  f.close();
}

void handleRoot() {
  server.send(200, "text/html",
              String(PAGE_HEAD) +
              "<h1>Doublethink</h1>"

              "<h2>Alert radius</h2>"
              "<p>How close to a zone's center sets off the buzzer.</p>"
              "<form method='POST' action='/settings'>"
              "<input type='number' name='radius' min='10' max='1000' value='" +
              String(alertRadiusMeters) +
              "'> meters "
              "<input type='submit' value='Save'>"
              "</form>"

              "<h2>Danger zones</h2>"
              "<p>" + String(numDangerZones) + " zones loaded. Upload a JSON "
              "file (array of {lat, lon}) to replace them:</p>"
              "<form method='POST' action='/upload' enctype='multipart/form-data'>"
              "<input type='file' name='zonesFile'>"
              "<input type='submit' value='Upload'>"
              "</form>");
}

// POST /settings — currently just the alert radius. Clamped to a sane range
// so a fat-fingered value can't disable alerts (0) or make everywhere a
// zone. Saved to config.json so it survives a power cycle.
void handleSettings() {
  if (server.hasArg("radius")) {
    long r = server.arg("radius").toInt();
    if (r >= 10 && r <= 5000) {
      alertRadiusMeters = (int)r;
      saveConfig();
      Serial.print("Settings: alert radius set to ");
      Serial.print(alertRadiusMeters);
      Serial.println(" m");
    }
  }
  server.send(200, "text/html",
              String(PAGE_HEAD) +
              "<h1>Doublethink</h1>"
              "<p><b>Alert radius is now " + String(alertRadiusMeters) +
              " m.</b></p>"
              "<p><a href='/'>Back</a></p>");
}

// Called repeatedly by the WebServer library as the uploaded file's bytes
// arrive — once with status START, many times with WRITE (one per chunk),
// then once with END. This is where bytes get onto flash; the file isn't
// touched anywhere else while this is running.
void handleFileUpload() {
  HTTPUpload& upload = server.upload();

  if (upload.status == UPLOAD_FILE_START) {
    uploadHadError = false;
    uploadFile = LittleFS.open(ZONES_TMP_FILE, "w");
    if (!uploadFile) {
      Serial.println("Upload: failed to open temp file for writing");
      uploadHadError = true;
    }
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (uploadFile) {
      uploadFile.write(upload.buf, upload.currentSize);
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    if (uploadFile) {
      uploadFile.close();
    }

    if (!uploadHadError) {
      // Validate before we let it replace the live file: it has to parse as
      // JSON *and* be an array (of zone objects). Per-entry lat/lon problems
      // are tolerated later in loadZonesFromFile() by skipping that entry.
      File check = LittleFS.open(ZONES_TMP_FILE, "r");
      JsonDocument doc;
      DeserializationError err = deserializeJson(doc, check);
      check.close();

      if (err) {
        Serial.print("Upload: invalid JSON — ");
        Serial.println(err.c_str());
        uploadHadError = true;
        LittleFS.remove(ZONES_TMP_FILE);
      } else if (!doc.is<JsonArray>()) {
        Serial.println("Upload: JSON is not an array of zones");
        uploadHadError = true;
        LittleFS.remove(ZONES_TMP_FILE);
      } else {
        LittleFS.remove(ZONES_FILE);  // ok if it doesn't exist yet
        LittleFS.rename(ZONES_TMP_FILE, ZONES_FILE);
        Serial.println("Upload: saved new zones.json");
        loadZones();  // apply it now — no reboot needed
      }
    }
  }
}

// Called once the upload above has fully finished (success or failure) —
// this is what actually sends the HTTP response back to the browser.
// Sent as text/html (not text/plain) so the browser *renders* the result
// on a page instead of offering it as a .txt file to download.
void handleUploadResult() {
  if (uploadHadError) {
    server.send(400, "text/html",
                String(PAGE_HEAD) +
                "<h1>Doublethink</h1>"
                "<p style='color:#b00'><b>Upload failed:</b> the file must be a "
                "JSON array of {lat, lon}. The previous zone list is "
                "unchanged.</p>"
                "<p><a href='/'>Back</a></p>");
  } else {
    server.send(200, "text/html",
                String(PAGE_HEAD) +
                "<h1>Doublethink</h1>"
                "<p style='color:#080'><b>Upload OK.</b> " +
                String(numDangerZones) +
                " zones saved and now active — no reboot needed.</p>"
                "<p><a href='/'>Back</a></p>");
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);

  pinMode(BUZZER_PIN, OUTPUT);

  // Both relays off at boot, so a reset while sitting in a zone leaves
  // the actuator wherever it was rather than driving it blindly.
  pinMode(ACTUATOR_EXTEND_PIN, OUTPUT);
  pinMode(ACTUATOR_RETRACT_PIN, OUTPUT);
  digitalWrite(ACTUATOR_EXTEND_PIN, RELAY_OFF);
  digitalWrite(ACTUATOR_RETRACT_PIN, RELAY_OFF);

  analogReadResolution(12);  // explicit: 0-4095, don't rely on the core default
  pinMode(JOYSTICK_VRX_PIN, INPUT);
  delay(50);                 // let the ADC/pot settle before the first reading
  joystickCenterX = readJoystickX();
  Serial.print("Joystick calibrated — leave it centered at boot. Center = ");
  Serial.println(joystickCenterX);

  pinMode(JOYSTICK_SW_PIN, INPUT_PULLUP);

  // `true` here means "format LittleFS if mounting fails" — the very first
  // boot after flashing, there's no filesystem on the flash chip yet, so
  // this formats it once rather than failing forever. Must happen before
  // loadConfig()/loadZones() below, which read from it.
  if (!LittleFS.begin(true)) {
    Serial.println("LittleFS mount failed");
  }

  loadConfig();  // alert radius from the last /settings change, if any
  loadZones();   // last uploaded zone list, or built-in defaults if none

  WiFi.softAP(AP_SSID, AP_PASSWORD);
  Serial.print("AP started. Connect to WiFi \"");
  Serial.print(AP_SSID);
  Serial.print("\" and browse to ");
  Serial.println(WiFi.softAPIP());

  server.on("/", handleRoot);
  server.on("/settings", HTTP_POST, handleSettings);
  server.on("/upload", HTTP_POST, handleUploadResult, handleFileUpload);
  server.begin();

  Serial.println("GPS test starting. Waiting for GPS coords...");
}

void loop() {
  // Handles any pending web request, then returns immediately — like
  // gpsSerial reads below, this never blocks, so the AP and GPS/buzzer
  // logic all keep running together in the same loop() pass.
  server.handleClient();

  // Feed every byte that arrives from the GPS module into the parser.
  // gps.encode() returns true once it has assembled a complete, valid
  // NMEA sentence — that's when its internal fields (location, etc.) update.
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  if (gps.location.isUpdated()) {
    Serial.print("Lat: ");
    Serial.print(gps.location.lat(), 6);
    Serial.print("  Lon: ");
    Serial.print(gps.location.lng(), 6);
    Serial.print("  Satellites: ");
    Serial.println(gps.satellites.value());

    inDangerZone = checkDangerZones(gps.location.lat(), gps.location.lng());
  }

  // Non-blocking beep pulse: while inDangerZone stays true, alternate the
  // buzzer between BEEP_ON_MS on and BEEP_OFF_MS off using millis(), so
  // loop() never stalls on delay() and GPS bytes keep getting read.
  if (inDangerZone) {
    unsigned long elapsed = millis() - lastBeepToggle;
    if (beepSounding && elapsed >= BEEP_ON_MS) {
      noTone(BUZZER_PIN);
      beepSounding = false;
      lastBeepToggle = millis();
    } else if (!beepSounding && elapsed >= BEEP_OFF_MS) {
      tone(BUZZER_PIN, ALERT_FREQUENCY_HZ);
      beepSounding = true;
      lastBeepToggle = millis();
    }
  } else if (beepSounding) {
    noTone(BUZZER_PIN);
    beepSounding = false;
  }

  // Joystick button: debounced press toggles actuatorEnabled, a manual
  // kill switch for the motor only (buzzer/zone detection above are
  // unaffected). Any press is a HIGH->LOW transition since SW idles HIGH
  // via INPUT_PULLUP. A debounce window means a transition only counts
  // once the reading has held steady for SW_DEBOUNCE_MS.
  int swReading = digitalRead(JOYSTICK_SW_PIN);
  if (swReading != swRawState) {
    swRawState = swReading;
    swLastChangeTime = millis();
  }
  if (swRawState != swStableState && millis() - swLastChangeTime >= SW_DEBOUNCE_MS) {
    swStableState = swRawState;
    if (swStableState == LOW) {
      actuatorEnabled = !actuatorEnabled;
      Serial.println(actuatorEnabled ? "Actuator: ENABLED (button)"
                                      : "Actuator: DISABLED (button)");
    }
  }

  // Actuator relay: fire one timed pulse when the danger-zone state
  // flips, then leave both relays off. Edge-triggered (inDangerZone vs
  // actuatorExtended) so it throws once per entry/exit, not repeatedly
  // while you sit inside a zone. A flip that happens mid-pulse is
  // caught on the next pass once the pulse finishes. Gated on
  // actuatorEnabled so the kill switch above can block new throws
  // without touching the zone/buzzer logic.
  if (actuatorEnabled && !actuatorPulsing && inDangerZone != actuatorExtended) {
    startActuatorPulse(inDangerZone);
    actuatorExtended = inDangerZone;
  }
  // Always let an already-started pulse finish and release, even if the
  // kill switch was pressed mid-throw — cutting power to a moving motor
  // early leaves it in an unknown position, which is worse than letting
  // a 100 ms pulse complete.
  if (actuatorPulsing && millis() - actuatorPulseStart >= ACTUATOR_PULSE_MS) {
    digitalWrite(actuatorActivePin, RELAY_OFF);
    actuatorPulsing = false;
    actuatorActivePin = -1;
  }

  // Manual joystick override: throws the actuator by hand for bench/field
  // testing without waiting for a real zone crossing. Gated on
  // actuatorPulsing exactly like the automatic trigger above so the two
  // never stack, and on actuatorEnabled so the kill switch also blocks
  // manual throws; a manual throw also updates actuatorExtended so the
  // zone logic doesn't fight it on the next GPS update.
  int joystickOffsetX = readJoystickX() - joystickCenterX;
  if (abs(joystickOffsetX) < JOYSTICK_DEADZONE_COUNTS) {
    joystickArmed = true;
  } else if (actuatorEnabled && joystickArmed && !actuatorPulsing) {
    bool extend = joystickOffsetX < 0;
    startActuatorPulse(extend);
    actuatorExtended = extend;
    joystickArmed = false;
  }

  // If we've gone a while with zero recognized characters at all,
  // the wiring/baud rate is probably wrong.
  if (millis() > 8000 && gps.charsProcessed() < 10) {
    Serial.println("No GPS data received — check wiring (RX/TX swapped?) and baud rate.");
    delay(3000);
  }
}
