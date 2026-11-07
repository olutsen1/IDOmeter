// =====================================================================
//  IDometer firmware 2.2 for the IDometer V1.0 board (TTGO T-Display ESP32)
//
//  The device does NOT know real time: it numbers the packets and stamps
//  them with its own tick counter. The receiver app converts this to
//  computer time. Packet format: protocol v1 (protocol.py).
//
//  Tasks:
//    sampler  (core 1, high priority) — on a 100 Hz hardware timer reads
//             PPG (MAX30102) and, at 20 Hz, GSR; builds 100 ms packets
//    sender   (core 0)  — sends over Wi-Fi (UDP) and writes the SD backup
//    display  (core 1, low priority) — screen and button
//    wifi     (core 0, low priority) — connection, setup mode, serial
//             monitor commands (see wifi_setup.h)
//
//  How data flows:
//    hardware timer (100 Hz) --notify--> sampler --queue--> sender --> Wi-Fi / SD
//  The sampler never waits for anything slow (Wi-Fi, SD card, screen), so a
//  Wi-Fi hiccup cannot shift the sampling moments. If the queue is full, a
//  packet is dropped and counted ("drop q" on the screen) instead of delaying
//  the measurement.
//
//  Why FreeRTOS tasks: the ESP32 has two cores. Core 1 does the time-critical
//  sampling; core 0 does networking. Tasks with different priorities let the
//  sampler interrupt everything else on its core the moment the timer fires.
//
//  Requires: board "ESP32 Dev Module", ESP32 core 3.x;
//  libraries "Adafruit ST7735 and ST7789 Library" (with dependencies)
//  and "SparkFun MAX3010x Pulse and Proximity Sensor Library".
// =====================================================================
#include <WiFi.h>              // Wi-Fi connection (ESP32 core)
#include <WiFiUdp.h>           // UDP packets: fast, no connection setup, no retries
#include <Wire.h>              // I2C bus: pulse sensor (and ADS1115 later)
#include <SPI.h>               // SPI bus: screen and SD card
#include <SD.h>                // FAT file system on the microSD card
#include <Adafruit_GFX.h>      // text and graphics on the screen
#include <Adafruit_ST7789.h>   // driver of the T-Display's 1.14" screen
#include "MAX30105.h"          // SparkFun driver; works for the MAX30102 too
#include "config.h"            // user settings: pins, Wi-Fi defaults, SD on/off
#include "wifi_setup.h"        // stored networks, setup mode, serial commands

#define FW_VERSION "2.2"

// ---- protocol v1 (must match software/protocol.py)
constexpr int      BASE_HZ  = 100;               // base sampling rate: PPG every tick
constexpr int      SLOW_DIV = 5;                 // EDA 20 Hz (every 5th tick)
constexpr int      TICKS    = 10;                // ticks per packet (100 ms)
constexpr int      NSLOW    = TICKS / SLOW_DIV;  // EDA samples per packet (2)
constexpr uint32_t TICK_US  = 1000000 / BASE_HZ; // one tick = 10 000 µs
// packet size: 20-byte header + IR and red (4 bytes each per tick)
//              + EDA and breathing (4 bytes each per slow sample)
constexpr size_t   PKT_SIZE = 20 + TICKS * 4 * 2 + NSLOW * 4 * 2;
constexpr uint16_t FLAG_FINGER = 1 << 0;         // bit 0: a finger is on the pulse sensor
constexpr uint16_t FLAG_SD_REC = 1 << 1;         // bit 1: the SD backup is being written
constexpr int      GSR_OVERSAMPLE = 16;          // one EDA sample = sum of 16 ADC reads

// ---- T-Display pins (fixed by the board)
constexpr int TFT_MOSI_PIN = 19, TFT_SCLK_PIN = 18, TFT_CS_PIN = 5,
              TFT_DC_PIN = 16, TFT_RST_PIN = 23, TFT_BL_PIN = 4;   // BL = backlight
constexpr int BAT_ADC_PIN = 34, BAT_EN_PIN = 14;   // battery voltage (through a 1:2 divider)

// One finished packet as raw bytes, ready to send or to write to SD.
struct Packet { uint8_t b[PKT_SIZE]; };

MAX30105 sensor;                                         // pulse sensor (MAX30102)
Adafruit_ST7789 tft(&SPI, TFT_CS_PIN, TFT_DC_PIN, TFT_RST_PIN);   // screen on the default SPI
SPIClass sdSPI(HSPI);                                    // second SPI bus, only for the SD card
File sdFile;                                             // open backup file (if any)
char sdName[24] = "";                                    // its name, e.g. /idm/rec_0007.bin

QueueHandle_t qSend;          // packets from the sampler to the sender
TaskHandle_t hSampler;        // the timer interrupt wakes this task
hw_timer_t* timer = nullptr;  // 100 Hz hardware timer

// ---- state for the screen (written by one task, read by another)
// "volatile" tells the compiler that another task may change these at any
// time, so it must read them from memory every time instead of caching them.
volatile bool     stSensorOk = false, stFinger = false, stSdOk = false, stLinked = false;
volatile uint32_t stIr = 0, stSent = 0, stQueueDrop = 0, stPpgDup = 0, stPpgDrop = 0;
volatile int32_t  stEda = 0;
volatile uint32_t stBatMv = 0;

// =====================================================================
//  Timer and sampling
// =====================================================================

// Timer interrupt, 100 times a second. It does almost nothing: it only wakes
// the sampler task. Interrupts must be short; the real work (I2C, ADC) is done
// in the task. IRAM_ATTR keeps this function in fast internal RAM.
void IRAM_ATTR onTick() {
  BaseType_t hp = pdFALSE;
  vTaskNotifyGiveFromISR(hSampler, &hp);   // +1 to the sampler's "ticks to process" counter
  if (hp) portYIELD_FROM_ISR();            // switch to the sampler right away if it has priority
}

// Copies n bytes into the packet and moves the write pointer on.
static inline void put(uint8_t*& p, const void* src, size_t n) { memcpy(p, src, n); p += n; }

// The sampler: one pass of the inner loop per timer tick (10 ms).
// Every tick: one PPG sample (IR + red). Every 5th tick: one GSR sample.
// Every 10th tick: the packet is complete and goes to the sender's queue.
void samplerTask(void*) {
  uint64_t tick = 0;              // ticks since start; the device's own clock
  uint32_t seq = 0;               // packet number; gaps show lost packets on the PC
  int k = 0;                      // position inside the current packet (0..TICKS-1)
  uint64_t t0 = 0;                // device time of the first sample in the packet, µs
  uint32_t ir[TICKS], red[TICKS]; // PPG samples of the current packet
  int32_t eda[NSLOW], resp[NSLOW];// GSR (and the future breathing) samples

  // The pulse sensor runs on its own oscillator, slightly different from our crystal.
  // A small buffer absorbs the difference: an extra sample is dropped, a missing one repeated.
  // Ring buffer of 8: rh = index of the oldest sample, rn = how many are stored.
  uint32_t rIr[8], rRed[8];
  int rh = 0, rn = 0;
  uint32_t lastIr = 0, lastRed = 0;   // the sample used for this tick

  for (;;) {
    // Sleep until the timer interrupt. Returns how many ticks have passed
    // (normally 1; more if this task was delayed, so no tick is ever skipped).
    uint32_t n = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);   // ticks elapsed
    while (n--) {
      // ---- PPG, 100 Hz
      if (stSensorOk) {
        // Move everything the sensor has collected (its internal FIFO) into our ring buffer.
        sensor.check();
        while (sensor.available()) {
          int w = (rh + rn) % 8;          // first free place
          rIr[w] = sensor.getFIFOIR();
          rRed[w] = sensor.getFIFORed();
          if (rn < 8) rn++; else rh = (rh + 1) % 8;   // buffer full: overwrite the oldest
          sensor.nextSample();
        }
        // The sensor is slightly faster than our timer: too many samples piled up.
        // Keep at most 3 waiting and drop the oldest ones (counted as "-" on the screen).
        while (rn > 3) { rh = (rh + 1) % 8; rn--; stPpgDrop = stPpgDrop + 1; }
        // Take one sample for this tick. If none is ready (the sensor is slightly
        // slower), the previous value is repeated (counted as "+" on the screen).
        if (rn > 0) { lastIr = rIr[rh]; lastRed = rRed[rh]; rh = (rh + 1) % 8; rn--; }
        else stPpgDup = stPpgDup + 1;
      }
      if (k == 0) t0 = tick * TICK_US;    // timestamp of the packet = its first tick
      ir[k] = lastIr;
      red[k] = lastRed;

      // ---- GSR, 20 Hz: no averaging over time, only 16 fast reads against ADC noise
      // The sum of 16 reads keeps the full precision (0..65520) without dividing.
      if (k % SLOW_DIV == 0) {
        int32_t s = 0;
        for (int i = 0; i < GSR_OVERSAMPLE; i++) s += analogRead(PIN_GSR);
        eda[k / SLOW_DIV] = s;
        resp[k / SLOW_DIV] = 0;            // no breathing channel yet
        stEda = s;                         // for the screen
      }
      // ---- battery once a second (here, so that only one task uses the ADC)
      // x2 because the board measures the battery through a 1:2 divider.
      if (tick % 100 == 50) stBatMv = analogReadMilliVolts(BAT_ADC_PIN) * 2;

      k++;
      tick++;
      if (k == TICKS) {
        // ---- the packet is complete: assemble the bytes (protocol v1, little-endian)
        stIr = lastIr;
        stFinger = lastIr > FINGER_IR_MIN;   // a finger reflects a lot of infrared light
        uint16_t flags = (stFinger ? FLAG_FINGER : 0) | (stSdOk ? FLAG_SD_REC : 0);
        uint16_t nt = TICKS;
        Packet pk;
        uint8_t* p = pk.b;
        const uint8_t hdr[4] = {'I', 'D', 1, 1};   // magic "ID", protocol version 1, type 1 (data)
        put(p, hdr, 4);
        put(p, &seq, 4);          // packet number
        put(p, &t0, 8);           // device time of the first sample, µs
        put(p, &nt, 2);           // number of ticks in the packet
        put(p, &flags, 2);        // finger / SD flags
        put(p, ir, sizeof ir);    // 10 x IR
        put(p, red, sizeof red);  // 10 x red
        put(p, eda, sizeof eda);  // 2 x GSR
        put(p, resp, sizeof resp);// 2 x breathing (zeros for now)
        if (xQueueSend(qSend, &pk, 0) != pdTRUE) stQueueDrop = stQueueDrop + 1;   // never wait
        seq++;
        k = 0;
      }
    }
  }
}

// =====================================================================
//  Sending and SD
// =====================================================================

// The sender takes finished packets from the queue and sends each one over
// Wi-Fi by UDP. It also writes them to the SD card, if one is inserted, so a
// session survives even a complete Wi-Fi failure.
//
// Finding the computer: at first the device does not know the computer's
// address, so it sends to everyone on the network (broadcast). The receiver
// app answers "IDHI"; from then on the device sends only to that computer.
// If no answer arrives for 10 s, it goes back to broadcasting.
void senderTask(void*) {
  WiFiUDP udp;
  bool udpOn = false;
  IPAddress peer, fixedIp;
  bool haveFixed = strlen(RECEIVER_IP) > 0 && fixedIp.fromString(RECEIVER_IP);   // fixed IP from config.h
  uint32_t lastHello = 0, lastFlush = 0;
  Packet pk;

  for (;;) {
    // Wait for the next packet (at most 0.5 s, then simply check again).
    if (xQueueReceive(qSend, &pk, pdMS_TO_TICKS(500)) != pdTRUE) continue;

    if (WiFi.status() == WL_CONNECTED) {
      if (!udpOn) udpOn = udp.begin(5007);   // our own port, where the "IDHI" replies arrive
      // the computer's "IDHI" reply: now we know its address
      while (udp.parsePacket() > 0) {
        char buf[8];
        int r = udp.read(buf, sizeof buf);
        if (r >= 4 && memcmp(buf, "IDHI", 4) == 0) {
          peer = udp.remoteIP();
          lastHello = millis();
        }
      }
      bool linked = lastHello && millis() - lastHello < 10000;   // heard from the PC in the last 10 s
      stLinked = linked;
      // destination: fixed IP if set, else the computer that answered, else everyone
      IPAddress dst = haveFixed ? fixedIp : (linked ? peer : WiFi.broadcastIP());
      udp.beginPacket(dst, RECEIVER_PORT);
      udp.write(pk.b, PKT_SIZE);
      if (udp.endPacket()) stSent = stSent + 1;
    } else {
      stLinked = false;   // no Wi-Fi: the packet still goes to SD below
    }

#if SD_BACKUP
    if (sdFile) {
      // The same bytes as sent over the air. On a write error the card is
      // given up (closed) instead of slowing the sender down.
      if (sdFile.write(pk.b, PKT_SIZE) != PKT_SIZE) { sdFile.close(); stSdOk = false; }
      // flush once a second: data reach the card even if power is cut suddenly
      else if (millis() - lastFlush > 1000) { sdFile.flush(); lastFlush = millis(); }
    }
#endif
  }
}

// =====================================================================
//  Screen and button
// =====================================================================

// Prints one line of text at height y, padded with spaces to the full width
// so that old, longer text is overwritten (no flicker from clearing the screen).
// size: 1 = small (6x8 pixels per character), 2 = double size.
void line(int y, uint8_t size, uint16_t color, const char* fmt, ...) {
  char s[48];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(s, sizeof s, fmt, ap);
  va_end(ap);
  int width = 240 / (6 * size);   // characters that fit across the 240-pixel screen
  char pad[48];
  snprintf(pad, sizeof pad, "%-*.*s", width, width, s);
  tft.setTextSize(size);
  tft.setTextColor(color, ST77XX_BLACK);   // black background erases what was there
  tft.setCursor(0, y);
  tft.print(pad);
}

// The "status" command in the serial monitor (called from wifi_setup.h).
void printStatus() {
  Serial.printf("WiFi: %s %s  IP %s  setup mode: %s\n",
                WiFi.status() == WL_CONNECTED ? "connected to" : "not connected", stSsid,
                WiFi.localIP().toString().c_str(), stApOn ? "on" : "off");
  Serial.printf("Computer: %s, packets sent %lu, dropped in queue %lu\n",
                stLinked ? "found" : "not found", (unsigned long)stSent, (unsigned long)stQueueDrop);
  Serial.printf("Pulse: %s, IR %lu;  GSR %ld;  battery %lu mV;  SD: %s\n",
                stSensorOk ? (stFinger ? "finger on" : "no finger") : "sensor not found",
                (unsigned long)stIr, (long)stEda, (unsigned long)stBatMv, stSdOk ? sdName : "none");
}

// Redraws the status screen twice a second and watches the right button.
// Lowest priority: it only runs when sampling and sending have nothing to do.
void displayTask(void*) {
  bool screenOn = true, btnPrev = true;   // the button reads LOW while pressed
  uint32_t lastDraw = 0;
  for (;;) {
    // button: screen on/off (so it does not distract the client)
    bool btn = digitalRead(PIN_BTN_SCREEN);
    if (btnPrev && !btn) {                 // the moment of pressing (HIGH -> LOW)
      screenOn = !screenOn;
      digitalWrite(TFT_BL_PIN, screenOn ? HIGH : LOW);   // backlight on/off
    }
    btnPrev = btn;

    if (screenOn && millis() - lastDraw > 500) {
      lastDraw = millis();
      bool wifi = WiFi.status() == WL_CONNECTED;
      line(0, 2, ST77XX_CYAN, "IDometer " FW_VERSION);
      if (stApOn && !wifi) {
        // setup mode: instructions instead of network status
        line(22, 2, ST77XX_YELLOW, "WiFi SETUP");
        line(40, 1, ST77XX_WHITE, "Phone: join WiFi '" AP_NAME "'");
        line(52, 2, ST77XX_YELLOW, "open 192.168.4.1");
      } else {
        line(22, 2, wifi ? ST77XX_GREEN : ST77XX_RED, wifi ? "WiFi: OK" : "WiFi: ...");
        line(40, 1, ST77XX_WHITE, "%s  %s", wifi ? stSsid : "searching known nets",
             wifi ? WiFi.localIP().toString().c_str() : "");
        line(52, 2, stLinked ? ST77XX_GREEN : ST77XX_YELLOW, stLinked ? "PC: linked" : "PC: searching");
      }
      if (!stSensorOk) line(72, 2, ST77XX_RED, "PPG: no sensor");
      else line(72, 2, stFinger ? ST77XX_GREEN : ST77XX_YELLOW,
                stFinger ? "Finger: OK" : "Finger: none");
      line(92, 2, ST77XX_WHITE, "GSR: %ld", (long)stEda);   // raw sum of 16 reads (0..65520)
      uint32_t mv = stBatMv;
      // above 4.4 V the "battery" is really the USB supply
      line(114, 1, ST77XX_WHITE, "SD: %s  %s  sent %lu",
           stSdOk ? sdName + 5 : "off",                      // +5 skips "/idm/"
           mv > 4400 ? "USB" : (String("Bat ") + String(mv / 1000.0, 2) + "V").c_str(),
           (unsigned long)stSent);
      // diagnostics: packets dropped from the queue; PPG samples repeated (+) / dropped (-)
      line(124, 1, 0x7BEF, "drop q%lu ppg+%lu/-%lu",
           (unsigned long)stQueueDrop, (unsigned long)stPpgDup, (unsigned long)stPpgDrop);
    }
    vTaskDelay(pdMS_TO_TICKS(50));   // check the button 20 times a second
  }
}

// =====================================================================
//  Startup
// =====================================================================

// Opens a new backup file on the SD card: /idm/rec_0001.bin, rec_0002.bin, ...
// (the first number that does not exist yet). No card: recording just continues
// over Wi-Fi only.
void openSdFile() {
#if SD_BACKUP
  sdSPI.begin(PIN_SD_SCLK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS);
  if (!SD.begin(PIN_SD_CS, sdSPI, 10000000)) { Serial.println("SD: no card"); return; }   // 10 MHz SPI
  if (!SD.exists("/idm")) SD.mkdir("/idm");
  for (int i = 1; i < 10000; i++) {
    snprintf(sdName, sizeof sdName, "/idm/rec_%04d.bin", i);
    if (!SD.exists(sdName)) break;
  }
  sdFile = SD.open(sdName, FILE_WRITE);
  stSdOk = (bool)sdFile;
  Serial.printf("SD: %s %s\n", sdName, stSdOk ? "OK" : "open failed");
#endif
}

// Runs once at power-on: hardware set-up, then the tasks and the timer start.
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\nIDometer " FW_VERSION);

  pinMode(PIN_BTN_SCREEN, INPUT);       // GPIO35 has no internal pull-up; the board has one
  pinMode(BAT_EN_PIN, OUTPUT);
  digitalWrite(BAT_EN_PIN, HIGH);       // enables the battery voltage divider
  analogReadResolution(12);             // ADC values 0..4095
  analogSetPinAttenuation(PIN_GSR, ADC_11db);   // input range up to ~3.1 V

  // screen
  pinMode(TFT_BL_PIN, OUTPUT);
  digitalWrite(TFT_BL_PIN, HIGH);       // backlight on
  SPI.begin(TFT_SCLK_PIN, -1, TFT_MOSI_PIN, TFT_CS_PIN);   // -1: the screen has no MISO
  tft.init(135, 240);                   // panel size in pixels
  tft.setRotation(1);                   // landscape
  tft.fillScreen(ST77XX_BLACK);
  line(0, 2, ST77XX_CYAN, "IDometer " FW_VERSION);
  line(30, 2, ST77XX_WHITE, "starting...");

  // pulse sensor: 100 Hz, IR + red, no averaging inside the sensor
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  if (sensor.begin(Wire, I2C_SPEED_FAST)) {      // I2C at 400 kHz
    // setup(LED power, sample averaging 1 = none, LED mode 2 = red + IR,
    //       100 samples/s, pulse width 411 µs (18-bit), ADC range 4096 nA)
    sensor.setup(PPG_LED_POWER, 1, 2, 100, 411, 4096);
    sensor.clearFIFO();                          // start with an empty sensor buffer
    stSensorOk = true;
  } else {
    Serial.println("MAX30102 not found - running without pulse");
  }

  openSdFile();

  // right button held at power-on: go straight to Wi-Fi setup mode
  line(60, 1, ST77XX_YELLOW, "Hold right button: WiFi setup");
  uint32_t t = millis();
  bool held = true;
  // "held" only if the button stays pressed (LOW) for the whole 1.5 s
  while (millis() - t < 1500) { if (digitalRead(PIN_BTN_SCREEN)) held = false; delay(20); }
  forceApAtBoot = held;
  tft.fillScreen(ST77XX_BLACK);

  qSend = xQueueCreate(64, sizeof(Packet));   // 6.4 s of slack for Wi-Fi/SD hiccups
  // xTaskCreatePinnedToCore(function, name, stack bytes, parameter, priority, handle, core)
  xTaskCreatePinnedToCore(samplerTask, "sampler", 4096, nullptr, configMAX_PRIORITIES - 2, &hSampler, 1);
  xTaskCreatePinnedToCore(senderTask, "sender", 6144, nullptr, 3, nullptr, 0);
  xTaskCreatePinnedToCore(displayTask, "display", 4096, nullptr, 1, nullptr, 1);
  xTaskCreatePinnedToCore(wifiTask, "wifi", 8192, nullptr, 1, nullptr, 0);

  // hardware timer 100 Hz (ESP32 core 3.x API)
  timer = timerBegin(1000000);                 // the timer counts at 1 MHz (1 count = 1 µs)
  timerAttachInterrupt(timer, &onTick);
  timerAlarm(timer, TICK_US, true, 0);         // every 10 000 µs, repeating forever
}

// Arduino's loop() is not needed: the tasks above do all the work, so the
// task running loop() deletes itself to free its memory.
void loop() {
  vTaskDelete(nullptr);   // the tasks do all the work
}
