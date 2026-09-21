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
#include "CWLibrary.hpp"

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

// SR6LEG
//#define carrier 1296805000.0  // SR6LEG 23cm
//#define freqMulti 1  // SR6LEG 23cm
//char cwTextWhenTimeIsValid[] = "SR6LEG SR6LEG LOC JO81CE JO81CE ";
//const char jtmessage[] = "DE SR6LEG JO81";
//const uint8_t q65_symbols[85] = { 0, 1, 1, 1, 1, 2, 40, 37, 0, 7, 55, 0, 0, 50, 0, 6, 5, 19, 51, 51, 51, 0, 0, 52, 39, 0, 0, 33, 18, 45, 13, 63, 0, 63, 0, 57, 57, 0, 47, 54, 50, 40, 57, 57, 62, 0, 6, 35, 35, 0, 62, 48, 25, 33, 0, 33, 33, 37, 37, 0, 30, 0, 21, 2, 38, 0, 9, 64, 0, 62, 61, 61, 49, 0, 49, 0, 46, 28, 19, 39, 17, 4, 4, 57, 0 }; // SR6LEG
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

//char cwTextWhenTimeIsValid[] = "SR6LB SR6LB LOC JO70SS JO70SS ";
//const char jtmessage[] = "DE SR6LB JO70";
//const uint8_t q65_symbols[85] = { 0, 1, 1, 1, 1, 2, 40, 37, 0, 7, 44, 0, 0, 50, 0, 6, 2, 35, 48, 48, 48, 0, 0, 47, 45, 0, 0, 14, 61, 34, 2, 52, 0, 52, 0, 54, 54, 0, 18, 38, 34, 62, 35, 35, 40, 0, 32, 57, 57, 0, 13, 47, 26, 5, 0, 5, 5, 6, 6, 0, 61, 0, 41, 62, 26, 0, 3, 54, 0, 43, 28, 28, 24, 0, 24, 0, 11, 61, 54, 61, 24, 4, 4, 57, 0 };  // SR6LB
// SR6LB

// SR3LES
#define carrier   1296872000.0 // SR3LES 23cm
#define freqMulti 1  // SR3LES 23cm
//#define carrier   2320872000.0 // SR3LES 13cm
//#define freqMulti 1  // SR3LES 13cm
//#define carrier 10368872000.0 // SR3LES 3cm
//#define freqMulti 4  // SR3LES 3cm

char cwTextWhenTimeIsValid[] = "SR3LES SR3LES LOC JO81HU JO81HU ";
const char jtmessage[] = "SR3LES JO81";
const uint8_t q65_symbols[85] = { 0, 1, 1, 1, 1, 2, 40, 35, 0, 20, 36, 0, 0, 10, 0, 6, 5, 19, 10, 10, 10, 0, 0, 9, 26, 0, 0, 32, 23, 48, 16, 62, 0, 62, 0, 47, 47, 0, 57, 9, 31, 9, 48, 48, 43, 0, 50, 23, 23, 0, 45, 63, 10, 12, 0, 12, 12, 16, 16, 0, 43, 0, 57, 46, 16, 0, 61, 42, 0, 44, 43, 43, 17, 0, 17, 0, 14, 60, 39, 14, 47, 62, 62, 57, 0 };
// SR3LES

// END OF PER BEACON VARS

char cwPrefixWhenNoTime[] = "NOTIME ";
char cwPrefixHNY[] = "HNY HNY ";

// Generic frequency definitions
#define spaceShift 400.0
#define mark (carrier / freqMulti)
#define space ((carrier - spaceShift) / freqMulti)

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

// Definitions related to CWLibrary

uint8_t cwSpeedWPM = 12;
void cwKeyDown() {
  Device.SetFrequency(mark);
}
void cwKeyUp() {
  Device.SetFrequency(space);
}
CWLibrary cw = CWLibrary(cwSpeedWPM, cwKeyDown, cwKeyUp);

// Definitions related to Q65
/* Q65-60D */
const float DF = 13.3333334f / freqMulti;  // Hz (tone spacing)
const uint32_t CENTER = mark + (DF * 32);  // Hz (midt-tone = symbol 32)
const uint16_t SYMBOL_MS = 600;            // 0,600 s per symbol
const uint32_t SLOT_MS = 59900UL;          // 60 s T/R-period

// Custom Code Functions

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

inline uint32_t q65_tone_hz(uint8_t s) {
  // f = (CENTER − 32·Δf) + s·Δf
  return (uint32_t)((double)CENTER - 32.0 * DF + (double)s * DF + 0.5);
}

void q65_sendMessage() {
  // TBC ??? const uint32_t tx_ms = 85UL * SYMBOL_MS;  // ≈ 51 000 ms
  // TBC ??? const uint32_t gap_ms = (SLOT_MS > tx_ms) ? (SLOT_MS - tx_ms) : 0;

  // Send 85 symbols
  for (uint8_t i = 0; i < 85; i++) {
    uint32_t f = q65_tone_hz(q65_symbols[i]);
    Device.SetFrequency((uint64_t)f);
    delay(SYMBOL_MS);
  }
}

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
        q65_sendMessage();  // send Q65 message
        Device.SetFrequency(mark);
      } else {  // all odd minutes 1,3,5,7,9,...
        if (now.tm_mday == 31 && now.tm_mon == 11) {  // tm_mon is 0-11, so December == 11
          cw.sendMessage(cwPrefixHNY);
        }
        cw.sendMessage(cwTextWhenTimeIsValid);
        Device.SetFrequency(mark);
      }
    } else {
      // PLAY CW only
      cw.sendMessage(cwPrefixWhenNoTime);
      cw.sendMessage(cwTextWhenTimeIsValid);
      Device.SetFrequency(mark);
      delay(20000);  // give at least 20secs of carrier
    }
    vTaskDelay(xDelay);  // END OF EXECUTION THREAD
  }
}

void loop() {
  delay(10000);
}  // UNUSED
