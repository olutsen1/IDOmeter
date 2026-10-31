/*
  ADS1115 module check — is it a genuine 16-bit ADS1115?

  Works on Arduino Uno/Nano (SDA = A4, SCL = A5) and on ESP32 (SDA 21, SCL 22).
  No libraries needed. Serial Monitor at 115200 baud.

  Wiring for the test:
    VDD  -> 5V (Uno) or 3V3 (ESP32)      GND -> GND
    SCL  -> SCL                          SDA -> SDA
    ADDR -> GND  (address 0x48)
    A0   -> 3.3V pin  (a known voltage; check it with a multimeter if you have one)
    A1   -> GND
    A2, A3 not connected (their readings are meaningless)

  How a fake is caught. Common fakes carry an ADS1015 (12-bit) die or a
  simpler chip. The tests do not trust the marking:
    1. Conversion time. At the slowest data-rate setting a real ADS1115
       converts at 8 samples/s (~125 ms); an ADS1015 at 128/s (~8 ms).
    2. Resolution. An ADS1015 result is always a multiple of 16 (its lowest
       4 bits are zero). An ADS1115 fills all 16 bits.
    3. Input multiplexer and accuracy: A0 must read ~3.3 V, A1 ~0 V.
       (ADS1113/1114 have no multiplexer: all channels would read the same.)
*/
#include <Wire.h>

const uint8_t REG_CONV = 0, REG_CONFIG = 1, REG_LO = 2, REG_HI = 3;
uint8_t addr = 0;

bool writeReg(uint8_t reg, uint16_t v) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(v >> 8);
  Wire.write(v & 0xFF);
  return Wire.endTransmission() == 0;
}

bool readReg(uint8_t reg, uint16_t &v) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission() != 0) return false;
  if (Wire.requestFrom(addr, (uint8_t)2) != 2) return false;
  v = (uint16_t)Wire.read() << 8;
  v |= Wire.read();
  return true;
}

// Single-shot conversion. mux: 4..7 = AIN0..AIN3 vs GND; pga: 0 = ±6.144 V, 1 = ±4.096 V;
// dr: 0 (slowest) .. 7 (fastest). Returns the raw result; conversion time in us via t_us.
bool convert(uint8_t mux, uint8_t pga, uint8_t dr, int16_t &raw, uint32_t &t_us) {
  uint16_t cfg = 0x8000 | (uint16_t)mux << 12 | (uint16_t)pga << 9 | 0x0100 | (uint16_t)dr << 5 | 0x0003;
  uint32_t t0 = micros();
  if (!writeReg(REG_CONFIG, cfg)) return false;
  delayMicroseconds(50);
  uint16_t c = 0;
  do {                                   // OS bit = 1 when the conversion is finished
    if (!readReg(REG_CONFIG, c)) return false;
    if (micros() - t0 > 500000UL) return false;
  } while (!(c & 0x8000));
  t_us = micros() - t0;
  uint16_t v;
  if (!readReg(REG_CONV, v)) return false;
  raw = (int16_t)v;
  return true;
}

int fails = 0;
void verdict(bool ok, const char *msg) {
  Serial.print(ok ? "  [ OK ] " : "  [FAIL] ");
  Serial.println(msg);
  if (!ok) fails++;
}

void setup() {
  Serial.begin(115200);
  delay(1500);
#if defined(ESP32)
  Wire.begin(21, 22);
#else
  Wire.begin();
#endif
  Serial.println(F("\n=== ADS1115 module check ==="));

  // ---- 1. find the chip on I2C
  for (uint8_t a = 0x48; a <= 0x4B; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) { addr = a; break; }
  }
  if (!addr) {
    Serial.println(F("  [FAIL] Nothing found at 0x48..0x4B. Check VDD, GND, SDA, SCL."));
    Serial.println(F("         (Other I2C devices are not searched for.)"));
    while (true) delay(1000);
  }
  Serial.print(F("  [ OK ] Found a device at 0x"));
  Serial.println(addr, HEX);

  // ---- 2. registers: power-on default and write/read-back
  uint16_t v;
  readReg(REG_CONFIG, v);
  Serial.print(F("  [info] Config register after power-on: 0x"));
  Serial.print(v, HEX);
  Serial.println(v == 0x8583 ? F(" (factory default, as expected)") : F(" (expected 0x8583 right after power-on)"));
  bool rw = writeReg(REG_LO, 0x1234) && writeReg(REG_HI, 0x5678);
  uint16_t lo = 0, hi = 0;
  rw = rw && readReg(REG_LO, lo) && readReg(REG_HI, hi);
  verdict(rw && lo == 0x1234 && hi == 0x5678, "Threshold registers keep what was written");
  writeReg(REG_LO, 0x8000);
  writeReg(REG_HI, 0x7FFF);

  // ---- 3. conversion time at the slowest and fastest settings
  int16_t raw;
  uint32_t tSlow = 0, tFast = 0;
  bool okS = convert(4, 1, 0, raw, tSlow);
  bool okF = convert(4, 1, 7, raw, tFast);
  Serial.print(F("  [info] Conversion time: slowest setting "));
  Serial.print(tSlow / 1000.0, 1);
  Serial.print(F(" ms, fastest "));
  Serial.print(tFast / 1000.0, 2);
  Serial.println(F(" ms"));
  Serial.println(F("         ADS1115: ~125 ms and ~1.2 ms.  ADS1015 (12-bit): ~8 ms and ~0.3 ms."));
  verdict(okS && okF && tSlow > 90000UL && tSlow < 170000UL, "Slowest conversion is ~125 ms (8 samples/s) - ADS1115 timing");

  // ---- 4. resolution: are the lowest 4 bits used?
  int nonzero = 0;
  for (int i = 0; i < 64; i++) {
    uint32_t t;
    if (convert(4, 1, 7, raw, t) && (raw & 0x000F)) nonzero++;
  }
  Serial.print(F("  [info] Readings with non-zero lowest 4 bits: "));
  Serial.print(nonzero);
  Serial.println(F(" of 64 (ADS1015 always gives 0)"));
  verdict(nonzero >= 8, "Full 16-bit results");

  // ---- 5. channels and accuracy
  float volts[4];
  for (uint8_t ch = 0; ch < 4; ch++) {
    uint32_t t;
    long sum = 0;
    for (int i = 0; i < 8; i++) {
      convert(4 + ch, 1, 4, raw, t);     // ±4.096 V range, 128 samples/s
      sum += raw;
    }
    volts[ch] = sum / 8.0 * 4.096 / 32768.0;
  }
  Serial.print(F("  [info] A0 = "));
  Serial.print(volts[0], 4);
  Serial.print(F(" V,  A1 = "));
  Serial.print(volts[1], 4);
  Serial.println(F(" V"));
  verdict(volts[0] > 3.0 && volts[0] < 3.6, "A0 reads the 3.3 V pin (3.0-3.6 V)");
  verdict(fabs(volts[1]) < 0.01, "A1 reads GND (within 10 mV)");

  Serial.println();
  if (fails == 0)
    Serial.println(F("RESULT: genuine 16-bit ADS1115, all checks passed."));
  else {
    Serial.print(F("RESULT: "));
    Serial.print(fails);
    Serial.println(F(" check(s) failed - see [FAIL] lines above. Check wiring first (A0 -> 3.3V, A1 -> GND)."));
  }
  Serial.println(F("\nLive readings every second (A0..A3, volts). Compare A0 with a multimeter:"));
}

void loop() {
  int16_t raw;
  uint32_t t;
  for (uint8_t ch = 0; ch < 4; ch++) {
    long sum = 0;
    for (int i = 0; i < 8; i++) {
      convert(4 + ch, 1, 4, raw, t);
      sum += raw;
    }
    Serial.print('A');
    Serial.print(ch);
    Serial.print(F(" = "));
    Serial.print(sum / 8.0 * 4.096 / 32768.0, 4);
    Serial.print(F(" V   "));
  }
  Serial.println();
  delay(1000);
}
