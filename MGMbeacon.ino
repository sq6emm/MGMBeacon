/*
  MGM beacon code, based on Etherkit JTencode library and PI4Ino.

  Beacon cycle:
  
  even minute JT4G (~48 secs ) + carrier (~12 secs)
  odd minute CW WPM12 (15 secs) + carrier (~45 secs)

  if no GPS time is detected in 60sec, beacon will transmit

  a lot of V then standard CW WPM12 message and a lot of V

  Based on PI4Ino by Bo OZ2M, thanks!

  Copyright
    Bo, OZ2M, www.rudius.net/oz2m/pi4ino
  
  Q65 code based on work done by Thomas LA3PNA, thanks!

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
#include <JTEncode.h>

TaskHandle_t Timing;
TaskHandle_t Transmission;
const TickType_t xDelay = (10 / portTICK_PERIOD_MS);

// Initialize all vars related to RTC Library
ESP32Time rtc(0);  // with 0 seconds of offset meaning UTC time is used
unsigned long rtcLastUpdate = 0;
#define rtcLastUpdateTimeoutms 86400000 // 86400000 // How many seconds we consider the time in local RTC to be valid (24 hrs by default)
uint8_t h, m, s, d, mm, y, crc;
bool timeState = false;

// Initialize all vars related to ADF4157
const byte deviceUpdate = D10;  // The Ardunio pin where the device update is controlled, if used
ADF4157 Device(deviceUpdate);

// Basic Frequencies and messages definitions
#define carrier 1296805000.0  // SR6LEG 23cm
#define freqMulti 1  // SR6LEG 23cm
char cwTextWhenTimeIsValid[] = "SR6LEG SR6LEG LOC JO81CE JO81CE "; // SR6LEG
const char jtmessage[] = "B SR6LEG JO81"; // SR6LEG
const uint8_t q65_symbols[85] = { 0,10,53,5,51,1,35,61,0,61,56,0,0,57,0,3,1,1,45,47,29,0,0,29,4,0,0,4,60,30,36,51,0,4,0,64,10,0,10,52,34,34,28,24,22,0,48,14,57,0,32,32,32,14,0,54,2,2,6,0,26,0,52,1,61,0,56,41,0,41,41,26,48,0,39,0,22,22,13,1,56,56,10,16,0 }; // SR6LEG

// #define carrier   1296830000.0 // SR6LB 23cm
// #define freqMulti 1  // SR6LB 23cm
// #define carrier 10368830000.0 // SR6LB 3cm
// #define freqMulti 4  // SR6LB 3cm
// #define carrier 432830000.0 // SR6LB 1.2cm
// #define freqMulti 0.5  // SR6LB 1.2cm
// char cwTextWhenTimeIsValid[] = "SR6LB SR6LB LOC JO70SS JO70SS "; // SR6LB
// const char jtmessage[] = "B SR6LB JO70"; // SR6LB
// const uint8_t q65_symbols[85] = { 0,1,15,62,5,16,17,4,0,44,4,0,0,47,0,50,43,1,36,63,59,0,0,54,44,0,0,44,6,19,6,14,0,14,0,39,54,0,37,5,58,58,38,44,27,0,43,59,42,0,19,19,36,31,0,31,17,59,8,0,17,0,31,7,6,0,30,13,0,13,40,10,14,0,14,0,4,62,56,47,46,62,56,41,0 }; // SR6LB

char cwPrefixWhenNoTime[] = "VVVVV ";
char cwPrefixHNY[] = "HNY HNY ";

// Generic frequency definitions
#define spaceShift 400.0
#define mark carrier / freqMulti
#define space (carrier - spaceShift) / freqMulti

// Initialize vars related to NMEA sentence analysis
const byte buff_len = 90;
char CRCbuffer[buff_len];
bool gpsFrame = false;

// Initialize vars realted to communication with GPS or eCzasPL rx
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
void cwKeyDown() { Device.SetFrequency(mark); }
void cwKeyUp() { Device.SetFrequency(space); }
CWLibrary cw = CWLibrary(cwSpeedWPM, cwKeyDown, cwKeyUp);

// Definitions related to Q65
/* Q65-60D */
const float    DF        = 13.3333334f/freqMulti;   // Hz (tone spacing)
const uint32_t CENTER    = carrier+(DF*32);  // Hz (midt-tone = symbol 32)
const uint16_t SYMBOL_MS = 600;          // 0,600 s per symbol
const uint32_t SLOT_MS   = 59900UL;      // 60 s T/R-period

// Definitions related to JT4
#define JT4_TONE_SPACING        315.0          // G 315.000 Hz (center around 1270Hz)
#define JT4_DELAY               229          // Delay value for JT4A
#define DEFAULT_MODE            MODE_JT4
enum mode {MODE_JT4};
uint8_t jt4_tx_buffer[255];
enum mode cur_mode = DEFAULT_MODE;
JTEncode jtencode;
uint8_t jt4_symbol_count = JT4_SYMBOL_COUNT; // From the library defines
uint16_t jt4_tone_spacing = JT4_TONE_SPACING/freqMulti;
uint16_t jt4_tone_delay = JT4_DELAY;

// Custom Code Functions

void ledState(uint8_t state) { // set Color of the state Led
  digitalWrite(LED_BLUE, ((~state&0x04)>>2));
  digitalWrite(LED_GREEN, ((~state&0x02)>>1));
  digitalWrite(LED_RED, (~state&0x01)); 
} // set Color of the state Led

// GPS supporting functions

uint8_t str2d(const char *s) { // String to Decimal Value
  uint8_t x;
  x = (s[0] - '0') * 10;
  x += (s[1] - '0');
  return x;
} // String to Decimal Value

uint8_t nmea_get_checksum(const char *sentence) { // NMEA get checksum from frame
  const char *n = sentence + 1;
  while (('*' != *n) && ('\0' != *n)) n++;
  if (*n == '*') return strtol(n + 1, NULL, 16);
  return 0;
} // NMEA get checksum from frame

uint8_t nmea_checksum(const char *sentence) { // NMEA calculate checksum based on frame
  const char *n = sentence + 1;
  uint8_t chk = 0;
  while (('*' != *n) && ('\n' != *n) && ('\n' != *n) && ('\0' != *n)) {
    chk ^= (uint8_t) *n;
    n++;
  }
  return chk;
} // NMEA calculate checksum based on frame

bool GPSFrameAnalysis(String frame) { // GPS Frame analysis
  // String frame="$GPRMC,092352.00,A,5112.02866,N,01612.54837,E,0.022,,231025,,,A*7C";
  // String frame="$GPRMC,092352,V,5112.0286,N,01612.5483,E,0.02,,231025,,,A*7C";
  // 0 - $GPRMC
  // 1 - UTC of position HHMMSS
  // 2 - (A = data valid, V = data invalid)
  // 9 - date ddmmyy

  if (frame.startsWith("$GPRMC,")) {
    uint8_t frameValid = 0;  // reset the validation of the GPS frame
    const char *p = frame.c_str();

    if (nmea_checksum(p) != nmea_get_checksum(p)) { return false; }

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
    
    if (frameValid == 3) { return true; } else { return false; };
  }
} // GPS Frame analysis

void TimeStatus() { // Time Status validation logic
  if (timeState) {  // we have valid input time from time source
    if ((rtcLastUpdate + rtcLastUpdateTimeoutms) < millis()) {
      ledState(COLOR_RED);
      timeState = false;
    } else {
      if (!gpsFrame) {
        ledState(COLOR_BLUE);
        // timeState true
      }
    }
  } else { // last Time status was negative
    if (gpsFrame) { // we have valid input time from time source
      ledState(COLOR_GREEN);
      timeState = true;
    }
  }
} // Time Status validation logic

inline uint32_t q65_tone_hz(uint8_t s) {
  // f = (CENTER − 32·Δf) + s·Δf
  return (uint32_t)((double)CENTER - 32.0 * DF + (double)s * DF + 0.5);
}

void q65_sendMessage()
{
  const uint32_t tx_ms  = 85UL * SYMBOL_MS;          // ≈ 51 000 ms
  const uint32_t gap_ms = (SLOT_MS > tx_ms) ? (SLOT_MS - tx_ms) : 0;

  // Send 85 symbols
  for (uint8_t i = 0; i < 85; i++) {
    uint32_t f = q65_tone_hz(q65_symbols[i]);
    Device.SetFrequency((uint64_t)f);
    delay(SYMBOL_MS);
  }
}

void jt4_sendMessage()
{
  uint8_t i;
  for(i = 0; i < jt4_symbol_count; i++)
  {
    // transmitting is happening here
    Device.SetFrequency((mark) + (jt4_tx_buffer[i] * jt4_tone_spacing));
    delay(jt4_tone_delay);
  }
  // Turn off the output
  Device.SetFrequency(mark);
}

void jt4_set_tx_buffer()
{
  // Clear out the transmit buffer
  memset(jt4_tx_buffer, 0, 255);
  // Set the proper frequency and timer CTC depending on mode
  jtencode.jt4_encode(jtmessage, jt4_tx_buffer);
}

// END of Custom Code Functions

void setup() {
  // Initialize Serial ports for comminication and time source
  Serial.begin(115200);
  Serial0.begin(115200);

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

  jt4_set_tx_buffer(); // encode JT message

  Device.Initialize(mark);
}

// Core 0 Loop (Timing)
void TimingCode(void *pvParameters) {

  while (1) {
    String r;
    if (Serial0.available() > 0) {
      serialLastUpdate = millis(); // last incoming data from Serial
      r = Serial0.readStringUntil('\n');
      Serial.println(r);  // DEBUG: always show input data to Serial
      if (GPSFrameAnalysis(r)) {
        gpsFrame = true;
        rtc.setTime(s, m, h, d, mm, 2000 + y);  // set local RTC
        rtcLastUpdate = millis();  // last RTC update      
        Serial.println(rtc.getTime("%Y-%m-%d %H:%M:%S"));
      }
      TimeStatus();
    } else {
      if ( (serialLastUpdate + serialLastUpdateTimeoutms) < millis() ) {
        gpsFrame = false; // invalidate last frame from GPS
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
    Device.Initialize(mark);
    if (timeState) {
      // PLAY CW + DIGI
      do { delay(500); } while (rtc.getSecond() != 0);
      if (rtc.getMinute()%2 == 0) {
        if (rtc.getMinute()/2%2 == 0) {
          jt4_sendMessage();
          Device.SetFrequency(mark);
        } else {
          q65_sendMessage(); // send Q65 message
          Device.SetFrequency(mark);
        }
      } else if (rtc.getMinute()%2 == 1 ) {
        if (rtc.getDay() == 31 && rtc.getMonth() == 12 ) {
          cw.sendMessage(cwPrefixHNY);
        }
        cw.sendMessage(cwTextWhenTimeIsValid);
      }
    } else {
      // PLAY CW only
      cw.sendMessage(cwPrefixWhenNoTime);
      cw.sendMessage(cwTextWhenTimeIsValid);
      Device.SetFrequency(mark);
      delay(5000); // give at least 20secs of carrier
    }
    vTaskDelay(xDelay);         // END OF EXECUTION THREAD
  }
}

void loop() {
  delay(10000);
}  // UNUSED