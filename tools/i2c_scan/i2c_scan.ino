/*
  I2C scanner for ESP32 (TTGO T-Display): scans all addresses twice,
  with SDA=21/SCL=22 and with the two lines swapped, so a swapped pair
  of wires is detected too. Serial Monitor at 115200 baud; repeats every 5 s.
*/
#include <Wire.h>

void scan(int sda, int scl) {
  Wire.end();
  Wire.begin(sda, scl, 100000);
  Serial.printf("SDA=%d, SCL=%d: ", sda, scl);
  int n = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.printf("0x%02X ", a);
      n++;
    }
  }
  Serial.println(n ? "" : "nothing found");
  // Idle level of the lines: both should be HIGH (pull-ups on the module)
  Wire.end();
  pinMode(sda, INPUT);
  pinMode(scl, INPUT);
  delay(5);
  Serial.printf("   idle levels: GPIO%d=%s, GPIO%d=%s\n", sda, digitalRead(sda) ? "HIGH" : "LOW (!)",
                scl, digitalRead(scl) ? "HIGH" : "LOW (!)");
}

void setup() {
  Serial.begin(115200);
  delay(1500);
}

void loop() {
  Serial.println("\n--- I2C scan ---");
  scan(21, 22);
  scan(22, 21);
  delay(5000);
}
