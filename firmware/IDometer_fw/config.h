// =====================================================================
//  IDometer firmware settings. Usually nothing here needs changing.
// =====================================================================
#pragma once

// ---- Wi-Fi (2.4 GHz only; ESP32 does not support 5 GHz)
// Optional: networks are normally added from a phone (setup mode) or with
// the "wifi" command in the serial monitor. If filled in, this network is
// saved on first start, when the device has no networks stored yet.
#define WIFI_SSID      "your_network"
#define WIFI_PASSWORD  "your_password"

// ---- Where to send data.
// Empty: the device finds the computer by itself (broadcasts until the app
// replies, then sends only to it). If that fails, put the computer's IP here,
// e.g. "192.168.88.20".
#define RECEIVER_IP    ""
#define RECEIVER_PORT  5005

// ---- Pins (IDometer V1.0 board + TTGO T-Display)
#define PIN_GSR        36     // Grove GSR output
#define PIN_I2C_SDA    21     // MAX30102
#define PIN_I2C_SCL    22
#define PIN_SD_SCLK    25
#define PIN_SD_MISO    27
#define PIN_SD_MOSI    26
#define PIN_SD_CS      33
#define PIN_BTN_SCREEN 35     // right button: screen on/off; hold at power-on for Wi-Fi setup

// ---- Pulse sensor
#define PPG_LED_POWER  0x1F   // LED current 0x00..0xFF (~6 mA at 0x1F)
#define FINGER_IR_MIN  50000  // below this: no finger on the sensor

// ---- Backup recording to SD card (if a card is inserted)
#define SD_BACKUP      1      // 0 = off
