/*
  MGM beacon code, based on Etherkit JTencode library and PI4Ino.

  Beacon cycle (with valid time):

  even minute: digitalMode (per beacon profile, e.g. Q65-60D ~51 secs,
               JT4 ~47 secs, PI4 ~24 secs) + carrier for the rest
  odd minute:  CW 12 WPM (~36 secs) + carrier for the rest
               (prefixed with "HNY HNY" on 31 December)

  Time comes from NMEA RMC frames (GPS or eCzasPL receiver) and is kept by
  the ESP32 clock for up to 24 hrs after the last valid frame. Without
  valid time (from boot until the first frame, or after those 24 hrs) the
  beacon repeats NOTIME + CW message + ~20 secs carrier instead.

  Based on PI4Ino by Bo OZ2M, thanks!

  Copyright
    Bo, OZ2M, www.rudius.net/oz2m/pi4ino
  
  Q65 code based on work done by Thomas LA3PNA, thanks!
  q65_code "DE CALL LOC"

  Author: Dawid SQ6EMM, February 2024-2025
  Contributor: Tomek SQ6QV

  To be considered for frequency allocation:
  http://www.g4jnt.com/JT4G_Tone_freqs.pdf

  Arduino NANO ESP32 pins used:

  CLK - D13
  DATA - D11
  LE - D10
  GPS/eCzasPL NMEA RX - RX0

  SR6LEG: JO81CE58CD
  SR6LB: JO70SS66UX
*/

#include <ADF4157.h>
#include <ESP32Time.h>
#include <BeaconModes.h>
#include <esp_task_wdt.h>

TaskHandle_t Timing;
TaskHandle_t Transmission;
const TickType_t xDelay = (10 / portTICK_PERIOD_MS);

// Initialize all vars related to RTC Library
// rtc is written on core 0 (TimingCode) and read on core 1 (TransmissionCode).
// ESP-IDF's settimeofday()/gettimeofday() (which setTime()/getTimeStruct() call
// into) already guard cross-core atomicity internally, so no extra locking is
// needed here -- wrapping them in our own portENTER_CRITICAL would (and did,
// when tried) panic the board, since those calls can internally block/take a
// lock, which is illegal inside a critical section. TransmissionCode derives
// minute/day/month from the single epoch value waitForNextMinute() waited
// for, so there are no torn reads across the individual fields.
ESP32Time rtc(0);  // with 0 seconds of offset meaning UTC time is used
unsigned long rtcLastUpdate = 0;  // millis() of the last valid frame; only meaningful once synced
bool synced = false;              // set by the first valid frame since boot
#define rtcLastUpdateTimeoutms 86400000  // How long (ms) we consider the time in local RTC to be valid after the last frame (24 hrs by default)
uint8_t h, m, s, d, mm, y, crc;

// Reboot if TransmissionCode stops checking in. Must exceed the longest gap
// between its esp_task_wdt_reset() calls: one cycle, about a minute (the
// longest digital mode is Q65-60, ~51 s; the NOTIME branch is ~62 s of
// CW + carrier).
#define watchdogTimeoutS 120

// Initialize all vars related to ADF4157
const byte deviceUpdate = D10;  // The Ardunio pin where the device update is controlled, if used
ADF4157 Device(deviceUpdate);

// PER BEACON VARS

// #define nmeaBaudrate 9600 // for GPS
#define nmeaBaudrate 115200  // for eCzasPL receiver

// Basic Frequencies and messages definitions

// Each profile defines CALLSIGN, a 6-character LOCATOR, and which digital
// mode the even-minute slot transmits (digitalMode); every on-air message
// (CW, Q65/wsjtmessage, PI4, JT4) is generated from CALLSIGN+LOCATOR at
// boot by buildMessages() below, respecting each mode's own length/alphabet
// limits, so there's one place to change per beacon. The even-minute slot
// is one minute, so only modes whose transmission fits in it are usable
// (Q65-120/300 are not). digitalMode options:
//   Q65Submode(Q65::Duration::T15,  Q65::Bandwidth::A..E)  -- Q65-15A..15E
//   Q65Submode(Q65::Duration::T30,  Q65::Bandwidth::A..E)  -- Q65-30A..30E
//   Q65Submode(Q65::Duration::T60,  Q65::Bandwidth::A..E)  -- Q65-60A..60E
//   JT4Submode(JT4::Submode::A..G)                         -- JT4A..JT4G
//   PI4Submode()                                           -- PI4
//   CWSubmode(wpm, spaceShiftHz)                            -- CW at a different speed/shift than cwSubmode below

// SR6LEG
//#define carrier 1296805000.0  // SR6LEG 23cm
//#define freqMulti 1  // SR6LEG 23cm
//#define CALLSIGN "SR6LEG"
//#define LOCATOR  "JO81CE"  // full: JO81CE58CD
//const DigitalMode digitalMode = Q65Submode(Q65::Duration::T60, Q65::Bandwidth::D); // Q65-60D
// SR6LEG

// TEST
//#define carrier 1296790000.0  // TEST 23cm
//#define freqMulti 1           // TEST 23cm
// TEST

// SR6LB
//#define carrier   1296830000.0 // SR6LB 23cm
//#define freqMulti 1  // SR6LB 23cm
//#define carrier 10368830000.0 // SR6LB 3cm
//#define freqMulti 4  // SR6LB 3cm
//#define carrier 432830000.0 // SR6LB 1.2cm
//#define freqMulti 0.5  // SR6LB 1.2cm

//#define CALLSIGN "SR6LB"
//#define LOCATOR  "JO70SS"  // full: JO70SS66UX
//const DigitalMode digitalMode = Q65Submode(Q65::Duration::T60, Q65::Bandwidth::D); // Q65-60D
// SR6LB

// SR3LES
#define carrier   1296872000.0 // SR3LES 23cm
#define freqMulti 1  // SR3LES 23cm
//#define carrier   2320872000.0 // SR3LES 13cm
//#define freqMulti 1  // SR3LES 13cm
//#define carrier 10368872000.0 // SR3LES 3cm
//#define freqMulti 4  // SR3LES 3cm

#define CALLSIGN "SR3LES"
#define LOCATOR  "JO81HU"
const DigitalMode digitalMode = Q65Submode(Q65::Duration::T60, Q65::Bandwidth::D); // Q65-60D
// SR3LES

// END OF PER BEACON VARS

// Message buffers, filled once by buildMessages() (called from setup()).
// Sizes are generous relative to CALLSIGN/LOCATOR above; PI4's 8-char cap
// and JT4's 13-char cap are the real constraints (enforced by their
// encoders, not by these buffer sizes).
char cwTextWhenTimeIsValid[40];  // "<CALL> <CALL> LOC <LOC6> <LOC6> "
char wsjtmessage[32];            // "DE <CALL> <LOC4>" -- used for Q65
char pi4Message[PI4::MAX_MESSAGE_LEN + 1];  // "<CALL>" -- PI4 has no room for a locator too
char jt4Message[16];             // "<CALL> <LOC4>"

void buildMessages() {
  char locator4[5];
  memcpy(locator4, LOCATOR, 4);  // LOCATOR is always >=4 chars; not a strncpy since we want exactly 4
  locator4[4] = '\0';

  snprintf(cwTextWhenTimeIsValid, sizeof(cwTextWhenTimeIsValid), "%s %s LOC %s %s ", CALLSIGN, CALLSIGN, LOCATOR, LOCATOR);
  snprintf(wsjtmessage, sizeof(wsjtmessage), "DE %s %s", CALLSIGN, locator4);
  snprintf(pi4Message, sizeof(pi4Message), "%s", CALLSIGN);
  snprintf(jt4Message, sizeof(jt4Message), "%s %s", CALLSIGN, locator4);
}

// Picks the message that matches digitalMode.mode -- each mode's message is
// generated with that mode's own length/alphabet limits in mind (see
// buildMessages() below), so this is what BeaconModes::transmit(digitalMode, ...)
// should always be called with.
const char *messageForDigitalMode() {
  switch (digitalMode.mode) {
    case BeaconMode::Q65: return wsjtmessage;
    case BeaconMode::PI4: return pi4Message;
    case BeaconMode::JT4: return jt4Message;
    case BeaconMode::CW: return cwTextWhenTimeIsValid;
  }
  return wsjtmessage;
}

char cwPrefixWhenNoTime[] = "NOTIME ";
char cwPrefixHNY[] = "HNY HNY ";

// Generic frequency definitions
#define spaceShift 400.0  // CW key-up tone offset below mark; see cwSubmode below
#define mark (carrier / freqMulti)

// Initialize vars related to NMEA sentence analysis
const byte buff_len = 90;
char nmeaLineBuffer[buff_len];

// Initialize vars related to communication with GPS or eCzasPL rx
unsigned long serialLastUpdate = 0;
#define serialLastUpdateTimeoutms 2000  // Time after which we decide that there is no data on Serial port from Timing device
#define frameFreshTimeoutms 3000  // Status LED stays green while valid frames (1/s) keep arriving; tolerates one missed frame

// Generic vars related to communication with user
unsigned long humanLastUpdate = 0;
#define humanLastUpdateTimeoutms 1000  // Time after which we decide that we can send a message to a Human
#define COLOR_OFF 0x00
#define COLOR_RED 0x01
#define COLOR_GREEN 0x02
#define COLOR_BLUE 0x04
#define COLOR_WHITE 0x07

// Definitions related to CW

uint8_t cwSpeedWPM = 12;
const CWSubmode cwSubmode = { cwSpeedWPM, spaceShift };

// Custom Code Functions

// BeaconModes::transmit() takes a plain function pointer, but
// Device.SetFrequency() is a member function -- this free-function wrapper
// is what every mode's transmit() call below uses.
void deviceSetFrequency(double freqHz) {
  Device.SetFrequency(freqHz);
}

// Every USB-serial line goes through usbLog(). The ESP32 core's
// USBCDC::write() busy-waits, with no timeout, for buffer space while a host
// holds the port open (DTR set) but isn't draining it fast enough; a line is
// therefore sent only if the whole line fits into the USB buffer right now,
// and dropped otherwise, so a slow or stuck serial monitor can never hold up
// the timing or transmission tasks.
void usbLog(const char *fmt, ...) {
  char buf[300];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(buf, sizeof buf - 2, fmt, ap);
  va_end(ap);
  if (n < 0) return;
  if (n > (int)sizeof buf - 3) n = sizeof buf - 3;
  buf[n++] = '\r';
  buf[n++] = '\n';
  if (Serial && Serial.availableForWrite() >= n) Serial.write((const uint8_t *)buf, n);
}

// One USB-serial line at the start and end of every transmission slot, so a
// log shows what was sent and when, and a missing "TX start" line (one per
// ~minute) shows the transmission task has stalled. Each line is a single
// printf, so it doesn't interleave with lines printed from core 0.
const char *txWhat;         // what the current slot sends
unsigned long txStartMs;    // millis() at its start

void txTimestamp(char *buf, size_t len) {  // "HH:MM:SS.mmm" from the system clock
  struct timeval tv;
  gettimeofday(&tv, NULL);
  tm t;
  gmtime_r(&tv.tv_sec, &t);
  snprintf(buf, len, "%02d:%02d:%02d.%03ld", t.tm_hour, t.tm_min, t.tm_sec, (long)(tv.tv_usec / 1000));
}

void txBegin(const char *what) {
  char ts[16];
  txWhat = what;
  txStartMs = millis();
  txTimestamp(ts, sizeof ts);
  usbLog("TX start %s at %s", what, ts);
}

void txEnd(bool ok) {
  char ts[16];
  txTimestamp(ts, sizeof ts);
  usbLog("TX end %s at %s, %.1f s%s", txWhat, ts,
         (millis() - txStartMs) / 1000.0, ok ? "" : ", NOT SENT (encode failed)");
}

const char *modeName(BeaconMode mode) {
  switch (mode) {
    case BeaconMode::CW: return "CW";
    case BeaconMode::Q65: return "Q65";
    case BeaconMode::PI4: return "PI4";
    case BeaconMode::JT4: return "JT4";
  }
  return "?";
}

void ledState(uint8_t state) {  // set Color of the state Led
  digitalWrite(LED_BLUE, ((~state & 0x04) >> 2));
  digitalWrite(LED_GREEN, ((~state & 0x02) >> 1));
  digitalWrite(LED_RED, (~state & 0x01));
}  // set Color of the state Led

// NMEA supporting functions

uint8_t str2d(const char *s) {  // String to Decimal Value
  uint8_t x;
  x = (s[0] - '0') * 10;
  x += (s[1] - '0');
  return x;
}  // String to Decimal Value

uint8_t nmea_get_checksum(const char *sentence) {  // NMEA get checksum from frame
  const char *n = sentence + 1;
  while (('*' != *n) && ('\0' != *n)) n++;
  if (*n == '*') return strtol(n + 1, NULL, 16);
  return 0;
}  // NMEA get checksum from frame

uint8_t nmea_checksum(const char *sentence) {  // NMEA calculate checksum based on frame
  const char *n = sentence + 1;
  uint8_t chk = 0;
  while (('*' != *n) && ('\r' != *n) && ('\n' != *n) && ('\0' != *n)) {
    chk ^= (uint8_t)*n;
    n++;
  }
  return chk;
}  // NMEA calculate checksum based on frame

bool nmeaFrameAnalysis(const char *frame) {  // NMEA Frame analysis
  // frame="$GPRMC,092352.00,A,5112.02866,N,01612.54837,E,0.022,,231025,,,A*7C";
  // frame="$GPRMC,092352,V,5112.0286,N,01612.5483,E,0.02,,231025,,,A*7C";
  // 0 - $GPRMC
  // 1 - UTC of position HHMMSS
  // 2 - (A = data valid, V = data invalid)
  // 9 - date ddmmyy

  const char *p = frame;
  uint8_t frameValid = 0;  // reset the validation of the NMEA Frame

  if (nmea_checksum(p) != nmea_get_checksum(p)) { return false; }
  if (strncmp(frame, "$GPRMC,", 7) == 0 || strncmp(frame, "$GNRMC,", 7) == 0) {
    uint8_t i = 0;
    while (*p) {
      if (*p == ',') {
        i++;
        p++;
        if (*p == 0) break;       //frame ends prematurely shouldn't happen with checksum checking
        if (*p == ',') continue;  //field has no data
        if (i == 1) {
          h = str2d(p);
          m = str2d(p + 2);
          s = str2d(p + 4);
          frameValid++;
        }
        if (i == 2) frameValid += (*p == 'A') ? 1 : 0;
        if (i == 9) {
          d = str2d(p);
          mm = str2d(p + 2);
          y = str2d(p + 4);
          frameValid++;
        }
      }
      p++;
    }
    if (frameValid == 3) {
      return true;
    } else {
      return false;
    };
  }
  return false;
}  // NMEA Frame analysis

// The one answer to "can the ESP32 clock be trusted for timed transmissions":
// a valid frame has arrived since boot, and the last one is under 24 hrs old.
// millis() is 32-bit and wraps every ~49.7 days: always compare elapsed time
// (now - then, which wraps correctly), never then + timeout < now.
bool timeValid() {
  return synced && millis() - rtcLastUpdate < rtcLastUpdateTimeoutms;
}

void updateStatusLed() {  // white: never synced, green: frames arriving, blue: holdover, red: expired
  static uint8_t shown = 0xFF;
  uint8_t color;
  if (!synced) color = COLOR_WHITE;
  else if (!timeValid()) color = COLOR_RED;
  else if (millis() - rtcLastUpdate <= frameFreshTimeoutms) color = COLOR_GREEN;
  else color = COLOR_BLUE;
  if (color != shown) {
    ledState(color);
    shown = color;
  }
}

tm waitForNextMinute() {  // Block until the top of the next UTC minute, return that minute
  // Digital modes must start at second 0 (decoders tolerate ~1s of DT), so wait
  // on the system clock at 1ms resolution instead of polling getSecond().
  // The clock is re-read every iteration because core 0 may step it
  // (rtc.setTime()) while we wait.
  struct timeval tv;
  gettimeofday(&tv, NULL);
  time_t nextMinute = (tv.tv_sec / 60 + 1) * 60;
  int64_t targetUs = (int64_t)nextMinute * 1000000;
  while (true) {
    gettimeofday(&tv, NULL);
    int64_t remainingUs = targetUs - ((int64_t)tv.tv_sec * 1000000 + tv.tv_usec);
    if (remainingUs <= 0) break;
    // Sleep most of the remaining time in one go (capped, to notice clock
    // steps), then finish in 1ms steps.
    uint32_t sleepMs = (remainingUs > 2000) ? min<int64_t>(remainingUs / 1000 - 1, 500) : 1;
    vTaskDelay(pdMS_TO_TICKS(sleepMs));
  }
  tm now;
  gmtime_r(&nextMinute, &now);
  return now;
}  // Block until the top of the next UTC minute, return that minute

// END of Custom Code Functions

void setup() {
  // Initialize Serial ports for comminication and time source
  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);  // usbLog() must never wait for the USB lock
  Serial0.begin(nmeaBaudrate);

  // Initialize RGB LED for status updates
  pinMode(LED_BUILTIN, OUTPUT);  // set builtin LED
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  ledState(COLOR_WHITE);

  // Everything TransmissionCode uses (messages, PLL) must be ready before its
  // task starts -- it shares core 1 with setup(), so a CW key-down could
  // otherwise interleave with Device.Initialize()'s register writes.
  buildMessages();  // derive cwTextWhenTimeIsValid/wsjtmessage/pi4Message/jt4Message from CALLSIGN+LOCATOR

  // Validate at boot that every mode this beacon might transmit can encode
  // its message -- BeaconModes::transmit() (used in TransmissionCode) would
  // otherwise fail silently (no transmission) every cycle instead of once,
  // loudly, here.
  uint8_t scratch[BeaconModes::MAX_SYMBOLS];
  if (!BeaconModes::encode(BeaconMode::Q65, wsjtmessage, scratch)) {
    usbLog("Q65 encode of wsjtmessage FAILED -- check wsjtmessage format");
  }
  if (!BeaconModes::encode(BeaconMode::PI4, pi4Message, scratch)) {
    usbLog("PI4 encode of pi4Message FAILED -- check pi4Message format");
  }
  if (!BeaconModes::encode(BeaconMode::JT4, jt4Message, scratch)) {
    usbLog("JT4 encode of jt4Message FAILED -- check jt4Message format");
  }

  Device.Initialize(mark);

  // Reconfigures the core's default 5 s task watchdog (which also keeps
  // watching core 0's idle task, now with this longer timeout).
  esp_task_wdt_init(watchdogTimeoutS, true);

  // creation of the Task that will run our Timing Related Code
  xTaskCreatePinnedToCore(TimingCode, "Timing", 10000, NULL, 1, &Timing, 0);
  delay(500);

  // creation of the Task that will run our Transmission/Beacon related Code
  xTaskCreatePinnedToCore(TransmissionCode, "Transmission", 10000, NULL, 1, &Transmission, 1);
  delay(500);
}

// Core 0 Loop (Timing)
void TimingCode(void *pvParameters) {

  while (1) {
    if (Serial0.available() > 0) {
      serialLastUpdate = millis();  // last incoming data from Serial
      size_t len = Serial0.readBytesUntil('\n', nmeaLineBuffer, buff_len - 1);
      nmeaLineBuffer[len] = '\0';
      // usbLog("%s", nmeaLineBuffer);  // DEBUG: always show input data to Serial
      if (nmeaFrameAnalysis(nmeaLineBuffer)) {
        rtc.setTime(s, m, h, d, mm, 2000 + y);  // set local RTC
        rtcLastUpdate = millis();               // last RTC update
        synced = true;
        usbLog("%s", rtc.getTime("%Y-%m-%d %H:%M:%S").c_str());
      }
    } else {
      if (millis() - serialLastUpdate > serialLastUpdateTimeoutms) {
        if (millis() - humanLastUpdate > humanLastUpdateTimeoutms) {
          usbLog("No serial data");
          if (timeValid()) { usbLog("%s", rtc.getTime("%Y-%m-%d %H:%M:%S").c_str()); };
          humanLastUpdate = millis();
        }
      }
    }
    updateStatusLed();
    vTaskDelay(xDelay);  // END OF EXECUTION THREAD
  }
}

// Core 1 Loop (Transmission)
void TransmissionCode(void *pvParameters) {
  esp_task_wdt_add(NULL);

  while (1) {
    esp_task_wdt_reset();
    // Reload all PLL registers every cycle, not just R0/R1 as SetFrequency()
    // does, so a register corrupted by RF/ESD at the site self-heals within
    // a minute instead of persisting until reboot.
    Device.Initialize(mark);
    if (timeValid()) {
      // Even minute: digitalMode; odd minute: CW
      tm now = waitForNextMinute();
      esp_task_wdt_reset();

      if (now.tm_min % 2 == 0) {  // all even minutes, 0,2,4,6,8,...
        // Which mode this transmits is decided once, per beacon profile,
        // by digitalMode (see the PER BEACON VARS section above).
        txBegin(modeName(digitalMode.mode));
        bool ok = BeaconModes::transmit(digitalMode, messageForDigitalMode(), mark, freqMulti, deviceSetFrequency);
        Device.SetFrequency(mark);
        txEnd(ok);
      } else {  // all odd minutes 1,3,5,7,9,...
        txBegin("CW");
        if (now.tm_mday == 31 && now.tm_mon == 11) {  // tm_mon is 0-11, so December == 11
          BeaconModes::transmit(BeaconMode::CW, cwPrefixHNY, mark, freqMulti, deviceSetFrequency, {}, {}, cwSubmode);
        }
        BeaconModes::transmit(BeaconMode::CW, cwTextWhenTimeIsValid, mark, freqMulti, deviceSetFrequency, {}, {}, cwSubmode);
        Device.SetFrequency(mark);
        txEnd(true);
      }
    } else {
      // PLAY CW only
      txBegin("CW NOTIME");
      BeaconModes::transmit(BeaconMode::CW, cwPrefixWhenNoTime, mark, freqMulti, deviceSetFrequency, {}, {}, cwSubmode);
      BeaconModes::transmit(BeaconMode::CW, cwTextWhenTimeIsValid, mark, freqMulti, deviceSetFrequency, {}, {}, cwSubmode);
      Device.SetFrequency(mark);
      txEnd(true);
      delay(20000);  // give at least 20secs of carrier
    }
    vTaskDelay(xDelay);  // END OF EXECUTION THREAD
  }
}

void loop() {
  delay(10000);
}  // UNUSED
