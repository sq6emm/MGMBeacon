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

  Author: Dawid SQ6EMM, February 2024

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
#include <JTEncode.h>

const byte deviceUpdate = D10;          // The Ardunio pin where the device update is controlled, if used
ADF4157 Device(deviceUpdate);

// Frequencies definition
#define carrier   1296805000.0 // SR6LEG freqMulti 1 
// #define carrier   1296830000.0 // SR6LB 23cm freqMulti 1
// #define carrier 10368830000.0 // SR6LB 3cm freqMulti 4
// #define carrier 432830000.0 // SR6LB 1.2 cm freqMulti 0.5

// Multiplier used? For high microwaves like x4...
#define freqMulti 1                              // Multiplier factor default 1 or others

// CW Frequencies definitions
#define cwSpaceShift 400.0                          // CW space shift down, -400Hz for uWaves -250 for VHF
#define cwMark carrier/freqMulti                    // The CW mark or carrier tone 
#define cwSpace (carrier-cwSpaceShift)/freqMulti    // The CW space or "no carrier" tone (-400Hz from carrier)
#define digiMark cwMark                             // If needed adjust to make sure that CW and DIGI can be decoded from USB in wsjt-x

// GPS settings
int gpsDelayComp = 0;
byte gpsSecond = 59;
const int gpsDataDelay = 50;  
unsigned int gpsCheckTimeout = 60;  // GPS check timeout in seconds
unsigned int current_minute = 0; // current minute
unsigned int current_day = 0; // current day
unsigned int current_month = 0; // current month

// Messages for Q65
// SR6LB
// const uint8_t symbols[85] = { 0,1,15,62,5,16,17,4,0,44,4,0,0,47,0,50,43,1,36,63,59,0,0,54,44,0,0,44,6,19,6,14,0,14,0,39,54,0,37,5,58,58,38,44,27,0,43,59,42,0,19,19,36,31,0,31,17,59,8,0,17,0,31,7,6,0,30,13,0,13,40,10,14,0,14,0,4,62,56,47,46,62,56,41,0 };
// SR6LEG
const uint8_t symbols[85] = { 0,10,53,5,51,1,35,61,0,61,56,0,0,57,0,3,1,1,45,47,29,0,0,29,4,0,0,4,60,30,36,51,0,4,0,64,10,0,10,52,34,34,28,24,22,0,48,14,57,0,32,32,32,14,0,54,2,2,6,0,26,0,52,1,61,0,56,41,0,41,41,26,48,0,39,0,22,22,13,1,56,56,10,16,0 };

// Messages for CW or JT modes
// CW single words only!
char call[] = "SR6LEG";                              // The CW callsign // SR6LEG
char locatorPref[] = "LOC";                      // The CW locator Prefix
char locator[] = "JO81CE";                       // The CW locator // JO70SS
char cwnogps[] = "VVVVVVVVVV";                   // When there is no GPS fix add this characters to transmission
char hnymsg[] = "HNY";                           // HNY message

const char jtmessage[] = "B SR6LEG JO81";            // The JT message // "B SR6LEG JO81" "B SR6LB JO70"

byte callLength;
byte locatorLength;
byte locatorPrefLength;
byte cwnogpsLength;
byte hnymsgLength;

// Mode definitions (JTEncode)
#define JT4_TONE_SPACING          315.0          // G 315.000 Hz (center around 1270Hz)
#define JT4_DELAY               229          // Delay value for JT4A

#define DEFAULT_MODE            MODE_JT4

// Mode definitions (Q65)
/* Q65-60D */
const float    DF        = 13.3333334f/freqMulti;   // Hz (tone spacing)
const uint32_t CENTER    = carrier+(DF*32);  // Hz (midt-tone = symbol 32)
const uint16_t SYMBOL_MS = 600;          // 0,600 s per symbol
const uint32_t SLOT_MS   = 59900UL;      // 60 s T/R-period

// Enumerations
enum mode {MODE_JT4, MODE_FT8};

// Class instantiation
JTEncode jtencode;

uint8_t tx_buffer[255];
enum mode cur_mode = DEFAULT_MODE;
uint8_t symbol_count;
uint16_t tone_delay, tone_spacing;

inline uint32_t q65_tone_hz(uint8_t s) {
  // f = (CENTER − 32·Δf) + s·Δf
  return (uint32_t)((double)CENTER - 32.0 * DF + (double)s * DF + 0.5);
}

void q65_encode()
{
  const uint32_t tx_ms  = 85UL * SYMBOL_MS;          // ≈ 51 000 ms
  const uint32_t gap_ms = (SLOT_MS > tx_ms) ? (SLOT_MS - tx_ms) : 0;

  // Send 85 symbols
  for (uint8_t i = 0; i < 85; i++) {
    uint32_t f = q65_tone_hz(symbols[i]);
    Device.SetFrequency((uint64_t)f);
    delay(SYMBOL_MS);
  }
}

// Loop through the string, transmitting one character at a time.
void encode()
{
  uint8_t i;

  for(i = 0; i < symbol_count; i++)
  {
    // transmitting is happening here
    Device.SetFrequency((digiMark) + (tx_buffer[i] * tone_spacing));
    delay(tone_delay);
  }

  // Turn off the output
  Device.SetFrequency(digiMark);
}

void set_tx_buffer()
{
  // Clear out the transmit buffer
  memset(tx_buffer, 0, 255);

  // Set the proper frequency and timer CTC depending on mode
  switch(cur_mode)
  {
  case MODE_JT4:
    jtencode.jt4_encode(jtmessage, tx_buffer);
    break;
  }
}

byte WaitUntil59(const byte theSecond, const int theDelay, const unsigned int theTimeout) {
  Serial0.begin(9600);
  boolean found = false;
  String r;
  unsigned int current_second = 0;
  unsigned long checkStart = millis();
  unsigned int internalTimeout = theTimeout * 1000;

  while (current_second != theSecond) {
    if (millis() - checkStart > internalTimeout) { break; }  // stop if timeout reached
    if (Serial0.available() > 0) {
      r = Serial0.readStringUntil('\n');
      if (r.startsWith("$GPRMC")) {
      Serial.println(r); // GPS DEBUG
        if (r.charAt(17) == 'A') {
          digitalWrite(LED_GREEN, LOW);
          digitalWrite(LED_RED, HIGH);
          digitalWrite(LED_BLUE, HIGH);  // LED color: Green
          checkStart = millis();
        } else {
          digitalWrite(LED_RED, HIGH);
          digitalWrite(LED_GREEN, HIGH);
          digitalWrite(LED_BLUE, LOW);   // LED color: Blue - NO CORRECT TIME INFORMATION FROM GPS
          break;
        }
        current_second = r.substring(11, 13).toInt();
        current_minute = r.substring(9, 11).toInt()+1;
        current_day = r.substring(53, 55).toInt();
        current_month = r.substring(55, 57).toInt();
        if (current_second == theSecond) {
          found = true;
          delay(theDelay+1000);
          break;
        }
      }
    }
  }
  Serial0.end();
  return found;
}

// *** Morse functions ***************************************
byte CharToMorse(char ch)
{   // e.g. 0x42 = 01000010 = 01000 010 = .- 2 bits long = A
    const byte letters[] = {0x42, 0x84, 0xA4, 0x83, 0x01, 0x24, 0xC3, 0x04, 0x02, 0x74, 0xA3, 0x44, 0xC2, 0x82, 0xE3, 0x64, 0xD4, 0x43, 0x03, 0x81, 0x23, 0x14, 0x63, 0x94, 0xB4, 0xC4};
    const byte numbers[] = {0x95, 0xFD, 0x7D, 0x3D, 0x1D, 0x0D, 0x05, 0x85, 0xC5, 0xE5, 0xF5};

    if (ch == 0)
        return (0);                                 // Not a valid morse character
    else if (ch < 'A')                              // Get then Morse pattern
        return (numbers[ch - '/']);
    else
        return (letters[ch - 'A']);
}

void SendMorse(const char *info, const byte msgLen)
{
    byte i, j;
    byte morse;
    byte morseLength;

    for (i = 0; i < msgLen; i++)
    {
        morse = info[i];
        morseLength = morse & 0x07;                 // Bit2 to Bit0 of morse is the length

        for (j = 0; j < morseLength; j++)
        {
            Device.SetFrequency(cwMark);
            // digitalWrite(LED_BUILTIN, HIGH);              // Turn TX LED on

            if ((morse & 0x80) == 0x80)             // If MSB 0 = dot, 1 = dash,
                delay(300);                         // It is a dash, so wait 3 dot durations
            else
                delay(100);                         // It is a dot, so wait 1 dot duration

            Device.SetFrequency(cwSpace);
            // digitalWrite(LED_BUILTIN, LOW);               // Turn TX LED off
            delay(100);
            morse = morse << 1;                     // Point to next bit
        }
        delay(200);                                 // Inter morse character pause, 1 dot duration already "gone" so only 2 left
    }
}
// *** End of Mourse functions ********************************************


void setup() {
  byte i;
  Serial.begin(9600);
  delay(2000);
  pinMode(LED_BUILTIN, OUTPUT); // set builtin LED
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_BLUE, LOW);  // LED color: White (Initialize)

  Device.Initialize(cwMark);

  if (gpsDataDelay > 0)                           // Calc the GPS compensation delay and second
    {   // If any delay at all
      gpsSecond = 59 - (gpsDataDelay / 1000) - 1;
      gpsDelayComp = (int) 1000 - (gpsDataDelay % 1000);
    }

    // Prepare CW call
    callLength = strlen(call);                      // Find the length of the CW call
    for (i = 0; i < callLength; i++)                // Convert call to Morse
        call[i] = CharToMorse(call[i]);

    // Prepare CW locator
    locatorLength = strlen(locator);                // Find the length of the CW locator
    for (i = 0; i < locatorLength; i++)             // Convert locator to Morse
        locator[i] = CharToMorse(locator[i]);

    // Prepare CW nogps
    cwnogpsLength = strlen(cwnogps);                // Find the length of the CW nogps
    for (i = 0; i < cwnogpsLength; i++)             // Convert nogps to Morse
        cwnogps[i] = CharToMorse(cwnogps[i]);

    // Prepare CW HNY
    hnymsgLength = strlen(hnymsg);                // Find the length of the CW hny
    for (i = 0; i < hnymsgLength; i++)            // Convert hny to Morse
        hnymsg[i] = CharToMorse(hnymsg[i]);

  // Set the mode to use
  cur_mode = MODE_JT4;

  // Set the proper frequency, tone spacing, symbol count, and
  // tone delay depending on mode
  switch(cur_mode)
  {
  case MODE_JT4:
    symbol_count = JT4_SYMBOL_COUNT; // From the library defines
    tone_spacing = JT4_TONE_SPACING/freqMulti;
    tone_delay = JT4_DELAY;
    break;
  }

  // Encode the message in the transmit buffer
  // This is RAM intensive and should be done separately from other subroutines
  set_tx_buffer();
}

void loop() {
  Device.Initialize(cwMark);
  byte gpsGot59 = WaitUntil59(gpsSecond, gpsDelayComp, gpsCheckTimeout);
  if (gpsGot59 && current_minute%2 == 0) {
    if (current_minute/2%2 == 0) {
      encode(); // send JT message  
      Device.SetFrequency(cwMark);
      delay(5000);
    } else {
      q65_encode(); // send Q65 message
    }
  } else if (gpsGot59 && current_minute%2 == 1) {
    Device.SetFrequency(cwSpace);
    delay(500);
    if (current_day == 31 && current_month == 12) { // HNY!
      SendMorse(hnymsg, hnymsgLength); // send  HNY  in CW
      delay(500);
      SendMorse(hnymsg, hnymsgLength); // send  HNY  in CW
      delay(500);
      SendMorse(hnymsg, hnymsgLength); // send  HNY  in CW
      delay(500);
    } 
    SendMorse(call, callLength); // send callsing in CW
    delay(500);
    SendMorse(call, callLength); // send callsing in CW
    delay(500);
    SendMorse(locatorPref, locatorPrefLength);
    delay(500);
    SendMorse(locator, locatorLength);
    delay(500);
    SendMorse(locator, locatorLength);
    delay(1000);
    Device.SetFrequency(cwMark); // send locator in CW
    delay(15000);
  } else {
    digitalWrite(LED_GREEN, HIGH);
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_BLUE, HIGH);  // LED color: Red (NO GPS DATA)
    Device.SetFrequency(cwSpace);
    delay(1000);
    SendMorse(cwnogps, cwnogpsLength); // send nogps marker in CW
    delay(500);
    SendMorse(call, callLength); // send callsing in CW
    delay(500);
    SendMorse(call, callLength); // send callsing in CW
    delay(500);
    SendMorse(locatorPref, locatorPrefLength);
    delay(500);
    SendMorse(locator, locatorLength);
    delay(500);
    SendMorse(locator, locatorLength);
    delay(500);
    SendMorse(cwnogps, cwnogpsLength); // send nogps marker in CW
    delay(500);
    Device.SetFrequency(cwMark); // send locator in CW
    delay(2000);
   }
}
