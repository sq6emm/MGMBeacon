/*
  ADF4157 step test: steps a single carrier from 1296.780 to 1296.850 MHz
  in 5 kHz steps and back down again, 2 s per step (one sweep up and down
  takes 56 s). No CW, no digital modes, no NMEA/eCzas time. Used to check
  the synthesizer and its 10 MHz reference in isolation: with a correct
  reference every step is received exactly where it is programmed.

  Each step is logged on USB serial ("step <n> <freq> Hz at <ms>").

  Result on SR6LEG, 2026-09-25 (received with the bench RTL-SDR through
  OpenWebRX, tools/openwebrx/track.py): every step landed exactly 5 kHz from
  the previous one, and the whole ladder was a constant 11.8 kHz low
  (-9.1 ppm; 1296.805 MHz received at 1296.793 MHz). A second, external SDR
  agreed (~-13 kHz). The registers are therefore right and the fault is the
  10 MHz reference feeding REFin (about 91 Hz low). The same result was
  obtained with a fixed carrier and with the ADF4157 library as it was on
  GitHub before the September 2026 changes (bfe8760).

  Arduino NANO ESP32 pins (as in MGMBeacon):
  CLK - D13
  DATA - D11
  LE - D10
*/

#include <ADF4157.h>

#define fStart 1296780000.0
#define fStop  1296850000.0
#define fStep  5000.0
#define stepMs 2000

ADF4157 Device(D10);  // LE pin

void setup() {
  Device.Initialize(fStart);  // writes R4..R0: REFin 10 MHz x2 = 20 MHz PFD

  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);
  delay(2000);
  if (Serial) Serial.printf("ADF4157 step test: %.0f..%.0f Hz, %.0f Hz steps, %d ms each\r\n", fStart, fStop, fStep, stepMs);
}

void loop() {
  static double f = fStart;
  static double dir = fStep;
  static unsigned long next = millis();
  static uint32_t n = 0;

  Device.SetFrequency(f);  // R1 then R0
  if (Serial) Serial.printf("step %lu %.0f Hz at %lu ms\r\n", (unsigned long)n++, f, millis());

  if (f + dir > fStop + 1 || f + dir < fStart - 1) dir = -dir;
  f += dir;

  next += stepMs;
  while ((long)(millis() - next) < 0) delay(1);
}
