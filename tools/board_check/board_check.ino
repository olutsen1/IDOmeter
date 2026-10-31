/*
  IDometer board check: the GSR node read by both ADCs side by side.

  For the TTGO T-Display on the IDometer board (ESP32 core 3.x, no libraries).
  Serial Monitor at 115200 baud. Every 0.5 s prints:
      time   ESP32 GPIO36 (mV)   ADS1115 A0 (mV)   difference (%)
  - At start: looks for the pulse sensor (0x57) and the ADS1115 (0x48).
  - Works without the ADS1115 too (prints only the ESP32 column).
  - For the resistor calibration: copy the whole Serial Monitor log.
*/
#include <Wire.h>

const int PIN_GSR = 36;
const uint8_t ADS = 0x48, MAX = 0x57;
bool hasAds = false;
uint32_t t0;

bool present(uint8_t a) {
  Wire.beginTransmission(a);
  return Wire.endTransmission() == 0;
}

bool adsWrite(uint8_t reg, uint16_t v) {
  Wire.beginTransmission(ADS);
  Wire.write(reg);
  Wire.write(v >> 8);
  Wire.write(v & 0xFF);
  return Wire.endTransmission() == 0;
}

bool adsRead(uint8_t reg, uint16_t &v) {
  Wire.beginTransmission(ADS);
  Wire.write(reg);
  if (Wire.endTransmission() != 0) return false;
  if (Wire.requestFrom(ADS, (uint8_t)2) != 2) return false;
  v = (uint16_t)Wire.read() << 8;
  v |= Wire.read();
  return true;
}

// One single-shot conversion of A0 against GND, range ±4.096 V, 128 samples/s. Returns millivolts.
bool adsA0(float &mv) {
  const uint16_t cfg = 0x8000 | (4 << 12) | (1 << 9) | 0x0100 | (4 << 5) | 0x0003;
  if (!adsWrite(1, cfg)) return false;
  uint32_t t = millis();
  uint16_t c = 0;
  do {
    delay(1);
    if (!adsRead(1, c) || millis() - t > 50) return false;
  } while (!(c & 0x8000));
  uint16_t v;
  if (!adsRead(0, v)) return false;
  mv = (int16_t)v * 4096.0f / 32768.0f;
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  Wire.begin(21, 22, 100000);
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_GSR, ADC_11db);

  Serial.println("\n=== IDometer board check ===");
  bool hasMax = present(MAX);
  hasAds = present(ADS);
  Serial.printf("  Pulse sensor MAX30102 (0x57): %s\n", hasMax ? "found" : "NOT FOUND");
  Serial.printf("  ADS1115 (0x48):               %s\n", hasAds ? "found" : "NOT FOUND - only the ESP32 column will be shown");
  Serial.println("\n  time, s | ESP32 GPIO36, mV | ADS1115 A0, mV | difference");
  t0 = millis();
}

void loop() {
  uint32_t sum = 0;
  for (int i = 0; i < 64; i++) sum += analogReadMilliVolts(PIN_GSR);
  float esp = sum / 64.0f;

  float ads = NAN;
  if (hasAds) {
    float acc = 0;
    int n = 0;
    for (int i = 0; i < 4; i++) {
      float mv;
      if (adsA0(mv)) { acc += mv; n++; }
    }
    if (n) ads = acc / n;
  }

  Serial.printf("  %7.1f | %16.0f | ", (millis() - t0) / 1000.0, esp);
  if (isnan(ads))
    Serial.println(hasAds ? "   read error   |" : "       -        |");
  else if (ads > 20)
    Serial.printf("%14.1f | %+6.1f %%\n", ads, (esp - ads) / ads * 100.0);
  else
    Serial.printf("%14.1f |\n", ads);

  delay(400);
}
