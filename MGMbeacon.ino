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

  Author: Dawid SQ6EMM, February 2024
*/


#include <ADF4157.h>
#include <JTEncode.h>

const byte deviceUpdate = 32;          // The Ardunio pin where the device update is controlled, if used
ADF4157 Device(deviceUpdate);


// Frequencies definition
#define carrier 10368785000.0                       // CW carrier / Mark frequency
#define cwSpaceShift 800.0                         // CW space shift down, -400Hz for uWaves -250 for VHF

// Multiplier used? For high microwaves like x4...
#define freqMulti 4                                 // Multiplier factor default 1 or others

// CW Frequencies definitions
#define cwMark carrier/freqMulti                    // The CW mark or carrier tone 
#define cwSpace (carrier-cwSpaceShift)/freqMulti    // The CW space or "no carrier" tone (-400Hz from carrier)                       

// GPS settings
int gpsDelayComp = 0;
byte gpsSecond = 59;
const int gpsDataDelay = 50;  
unsigned int gpsCheckTimeout = 60;  // GPS check timeout in seconds
unsigned int current_minute = 0; // current minute

// Messages for CW or JT modes
char call[] = "SR6LB SR6LB ";                              // The CW callsign
char locator[] = "LOC JO81CJ JO81CJ";                          // The CW locator
char cwnogps[] = "VVVVVVVVVV";                        // When there is no GPS fix add this characters to transmission
const char jtmessage[] = "SR6LB JO81CJ";            // The JT message
byte callLength;
byte locatorLength;
byte cwnogpsLength;

// Mode defines
// #define JT4_TONE_SPACING      1575          // F 157.500 Hz (center around 1270Hz)
#define JT4_TONE_SPACING          315.0          // G 315.000 Hz (center around 1270Hz)
#define FT8_TONE_SPACING          6.25          // ~6.25 Hz

#define JT4_DELAY               229          // Delay value for JT4A
#define FT8_DELAY               159          // Delay value for FT8

#define DEFAULT_MODE            MODE_JT4

// Enumerations
enum mode {MODE_JT4, MODE_FT8};

// Class instantiation
JTEncode jtencode;

uint8_t tx_buffer[255];
enum mode cur_mode = DEFAULT_MODE;
uint8_t symbol_count;
uint16_t tone_delay, tone_spacing;

// Loop through the string, transmitting one character at a time.
void encode()
{
  uint8_t i;

  for(i = 0; i < symbol_count; i++)
  {
    // transmitting is happening here
    Device.SetFrequency((cwMark) + (tx_buffer[i] * tone_spacing));
    delay(tone_delay);
  }

  // Turn off the output
  Device.SetFrequency(cwMark);
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
  case MODE_FT8:
    jtencode.ft8_encode(jtmessage, tx_buffer);
    break;
  }
}

byte WaitUntil59(const byte theSecond, const int theDelay, const unsigned int theTimeout) {
  Serial1.begin(9600);
  boolean found = false;
  String r;
  unsigned int current_second = 0;
  unsigned int minute = 0;
  unsigned long checkStart = millis();
  unsigned int internalTimeout = theTimeout * 1000;
  digitalWrite(LED_BUILTIN, LOW);  // Turn GPS valid off before checking status

  while (current_second != theSecond) {
    if (millis() - checkStart > internalTimeout) { break; }  // stop if timeout reached
    if (Serial1.available() > 0) {
      r = Serial1.readStringUntil('\n');
      if (r.startsWith("$GPRMC")) {
        if (r.charAt(17) == 'A') {
          digitalWrite(LED_BUILTIN, HIGH);  // Turn GPS status on
          checkStart = millis();
        } else {
          digitalWrite(LED_BUILTIN, LOW);  // Turn GPS status on
        }
        current_second = r.substring(11, 13).toInt();
        current_minute = (r.substring(8, 11).toInt() + 1)%2;
        if (current_second == theSecond) {
          found = true;
          delay(theDelay+1000);
          break;
        }
      }
    }
  }
  Serial1.end();
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
     //       digitalWrite(ledTX, HIGH);              // Turn TX LED on

            if ((morse & 0x80) == 0x80)             // If MSB 0 = dot, 1 = dash,
                delay(300);                         // It is a dash, so wait 3 dot durations
            else
                delay(100);                         // It is a dot, so wait 1 dot duration

            Device.SetFrequency(cwSpace);
       //     digitalWrite(ledTX, LOW);               // Turn TX LED off
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
  Serial.println();
  Serial.println("Started...");
  pinMode(LED_BUILTIN, OUTPUT); // set builtin LED
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

    // Prepare CW locator
    cwnogpsLength = strlen(cwnogps);                // Find the length of the CW locator
    for (i = 0; i < cwnogpsLength; i++)             // Convert locator to Morse
        cwnogps[i] = CharToMorse(cwnogps[i]);

  // Use the Arduino's on-board LED as a keying indicator.


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
  case MODE_FT8:
    symbol_count = FT8_SYMBOL_COUNT; // From the library defines
    tone_spacing = FT8_TONE_SPACING/freqMulti;
    tone_delay = FT8_DELAY;
    break;
  }

  // Set CLK0 output


  // Encode the message in the transmit buffer
  // This is RAM intensive and should be done separately from other subroutines
  set_tx_buffer();
}

void loop() {
  byte gpsGot59 = WaitUntil59(gpsSecond, gpsDelayComp, gpsCheckTimeout);
  if (gpsGot59 && current_minute == 0) {
    encode(); // send JT message
    Device.SetFrequency(cwMark);
    delay(5000);
  } else if (gpsGot59 && current_minute == 1) {
    Device.SetFrequency(cwSpace);
    delay(500);
    SendMorse(call, callLength); // send callsing in CW
    delay(1000);
    SendMorse(locator, locatorLength);
    Device.SetFrequency(cwSpace);
    delay(1000);
    Device.SetFrequency(cwMark); // send locator in CW
    delay(15000);
  } else {
    Device.SetFrequency(cwSpace);
    delay(500);
    SendMorse(cwnogps, cwnogpsLength); // send nogps marker in CW
    delay(1000);
    SendMorse(call, callLength); // send callsing in CW
    delay(1000);
    SendMorse(locator, locatorLength);
    delay(1000);
    SendMorse(cwnogps, cwnogpsLength); // send nogps marker in CW
    delay(1000);
    Device.SetFrequency(cwMark); // send locator in CW
    delay(2000);
  }
}
