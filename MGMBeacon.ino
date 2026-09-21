/*
  MGM beacon code, based on Etherkit JTencode library and PI4Ino.

  Beacon cycle:
  
  even minute JT4G (~48 secs ) + carrier (~12 secs)
  odd minute CW WPM12 (15 secs) + carrier (~45 secs)

  if no GPS time is detected in 60sec, beacon will transmit
  NOTIME then standard CW WPM12 message

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
  GPS RX - RX0

  SR6LEG: JO81CE58CD
  SR6LB: JO70SS66UX
*/

#include <ADF4157.h>
#include <ESP32Time.h>
#include <BeaconModes.h>

TaskHandle_t Timing;
TaskHandle_t Transmission;
const TickType_t xDelay = (10 / portTICK_PERIOD_MS);

// Initialize all vars related to RTC Library
// rtc is written on core 0 (TimingCode) and read on core 1 (TransmissionCode).
// ESP-IDF's settimeofday()/gettimeofday() (which setTime()/getTimeStruct() call
// into) already guard cross-core atomicity internally, so no extra locking is
// needed here -- wrapping them in our own portENTER_CRITICAL would (and did,
// when tried) panic the board, since those calls can internally block/take a
// lock, which is illegal inside a critical section. A single getTimeStruct()
// snapshot per read (see TransmissionCode) is enough to avoid torn reads
// across the individual minute/day/month fields.
ESP32Time rtc(0);  // with 0 seconds of offset meaning UTC time is used
unsigned long rtcLastUpdate = 0;
#define rtcLastUpdateTimeoutms 86400000  // 86400000 // How many seconds we consider the time in local RTC to be valid (24 hrs by default)
uint8_t h, m, s, d, mm, y, crc;
bool timeState = false;

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
// limits, so there's one place to change per beacon. digitalMode options:
//   Q65Submode(Q65::Duration::T15,  Q65::Bandwidth::A..E)  -- Q65-15A..15E
//   Q65Submode(Q65::Duration::T30,  Q65::Bandwidth::A..E)  -- Q65-30A..30E
//   Q65Submode(Q65::Duration::T60,  Q65::Bandwidth::A..E)  -- Q65-60A..60E  (this beacon's previous default: 60D)
//   Q65Submode(Q65::Duration::T120, Q65::Bandwidth::A..E)  -- Q65-120A..120E
//   Q65Submode(Q65::Duration::T300, Q65::Bandwidth::A..E)  -- Q65-300A..300E
//   JT4Submode(JT4::Submode::A..G)                         -- JT4A..JT4G (this beacon's previous default: JT4G)
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
bool nmeaFrame = false;

// Initialize vars related to communication with GPS or eCzasPL rx
unsigned long serialLastUpdate = 0;
#define serialLastUpdateTimeoutms 2000  // Time after which we decide that there is no data on Serial port from Timing device

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

void TimeStatus() {  // Time Status validation logic
  if (timeState) {   // we have valid input time from time source
    if ((rtcLastUpdate + rtcLastUpdateTimeoutms) < millis()) {
      ledState(COLOR_RED);
      timeState = false;
    } else {
      if (!nmeaFrame) {
        ledState(COLOR_BLUE);
        // timeState true
      }
    }
  } else {           // last Time status was negative
    if (nmeaFrame) {  // we have valid input time from time source
      ledState(COLOR_GREEN);
      timeState = true;
    }
  }
}  // Time Status validation logic

// END of Custom Code Functions

void setup() {
  // Initialize Serial ports for comminication and time source
  Serial.begin(115200);
  Serial0.begin(nmeaBaudrate);

  // Initialize RGB LED for status updates
  pinMode(LED_BUILTIN, OUTPUT);  // set builtin LED
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  ledState(COLOR_WHITE);

  // creation of the Task that will run our Timing Related Code
  xTaskCreatePinnedToCore(TimingCode, "Timing", 10000, NULL, 1, &Timing, 0);
  delay(500);

  // creation of the Task that will run our Transmission/Beacon related Code
  xTaskCreatePinnedToCore(TransmissionCode, "Transmission", 10000, NULL, 1, &Transmission, 1);
  delay(500);

  buildMessages();  // derive cwTextWhenTimeIsValid/wsjtmessage/pi4Message/jt4Message from CALLSIGN+LOCATOR

  // Validate at boot that every mode this beacon might transmit can encode
  // its message -- BeaconModes::transmit() (used in TransmissionCode) would
  // otherwise fail silently (no transmission) every cycle instead of once,
  // loudly, here.
  uint8_t scratch[BeaconModes::MAX_SYMBOLS];
  if (!BeaconModes::encode(BeaconMode::Q65, wsjtmessage, scratch)) {
    Serial.println("Q65 encode of wsjtmessage FAILED -- check wsjtmessage format");
  }
  if (!BeaconModes::encode(BeaconMode::PI4, pi4Message, scratch)) {
    Serial.println("PI4 encode of pi4Message FAILED -- check pi4Message format");
  }
  if (!BeaconModes::encode(BeaconMode::JT4, jt4Message, scratch)) {
    Serial.println("JT4 encode of jt4Message FAILED -- check jt4Message format");
  }

  Device.Initialize(mark);
}

// Core 0 Loop (Timing)
void TimingCode(void *pvParameters) {

  while (1) {
    if (Serial0.available() > 0) {
      serialLastUpdate = millis();  // last incoming data from Serial
      size_t len = Serial0.readBytesUntil('\n', nmeaLineBuffer, buff_len - 1);
      nmeaLineBuffer[len] = '\0';
      // Serial.println(nmeaLineBuffer);  // DEBUG: always show input data to Serial
      if (nmeaFrameAnalysis(nmeaLineBuffer)) {
        nmeaFrame = true;
        rtc.setTime(s, m, h, d, mm, 2000 + y);  // set local RTC
        rtcLastUpdate = millis();               // last RTC update
        Serial.println(rtc.getTime("%Y-%m-%d %H:%M:%S"));
      }
      TimeStatus();
    } else {
      if ((serialLastUpdate + serialLastUpdateTimeoutms) < millis()) {
        nmeaFrame = false;  // invalidate last frame from GPS
        TimeStatus();
        if ((millis() - humanLastUpdateTimeoutms) > humanLastUpdate) {
          Serial.println("No serial data");
          if (timeState == true) { Serial.println(rtc.getTime("%Y-%m-%d %H:%M:%S")); };
          humanLastUpdate = millis();
        }
      }
    }
    vTaskDelay(xDelay);  // END OF EXECUTION THREAD
  }
}

// Core 1 Loop (Transmission)
void TransmissionCode(void *pvParameters) {

  while (1) {
    Device.SetFrequency(mark);  // (re)assert carrier; full register reload only needed once, in setup()
    if (timeState) {
      // PLAY CW + Q65 and again
      do { delay(500); } while (rtc.getSecond() != 0);

      // Snapshot minute/day/month together (one call) so a concurrent setTime()
      // on core 0 can't hand us a torn mix of old/new fields.
      tm now = rtc.getTimeStruct();

      if (now.tm_min % 2 == 0) {  // all even minutes, 0,2,4,6,8,...
        // Which mode this transmits is decided once, per beacon profile,
        // by digitalMode (see the PER BEACON VARS section above).
        BeaconModes::transmit(digitalMode, messageForDigitalMode(), mark, freqMulti, deviceSetFrequency);
        Device.SetFrequency(mark);
      } else {  // all odd minutes 1,3,5,7,9,...
        if (now.tm_mday == 31 && now.tm_mon == 11) {  // tm_mon is 0-11, so December == 11
          BeaconModes::transmit(BeaconMode::CW, cwPrefixHNY, mark, freqMulti, deviceSetFrequency, {}, {}, cwSubmode);
        }
        BeaconModes::transmit(BeaconMode::CW, cwTextWhenTimeIsValid, mark, freqMulti, deviceSetFrequency, {}, {}, cwSubmode);
        Device.SetFrequency(mark);
      }
    } else {
      // PLAY CW only
      BeaconModes::transmit(BeaconMode::CW, cwPrefixWhenNoTime, mark, freqMulti, deviceSetFrequency, {}, {}, cwSubmode);
      BeaconModes::transmit(BeaconMode::CW, cwTextWhenTimeIsValid, mark, freqMulti, deviceSetFrequency, {}, {}, cwSubmode);
      Device.SetFrequency(mark);
      delay(20000);  // give at least 20secs of carrier
    }
    vTaskDelay(xDelay);  // END OF EXECUTION THREAD
  }
}

void loop() {
  delay(10000);
}  // UNUSED
