// =====================================================================
//  IDometer — прошивка 2.1 для платы IDometer V1.0 (TTGO T-Display ESP32)
//
//  Устройство НЕ знает реального времени: оно нумерует пакеты и ставит
//  метку по собственному счётчику тактов. Время во время компьютера
//  переводит программа-приёмник. Формат пакета — протокол v1 (protocol.py).
//
//  Задачи:
//    sampler  (ядро 1, высокий приоритет) — по аппаратному таймеру 100 Гц
//             читает PPG (MAX30102) и 20 Гц — GSR, собирает пакеты по 100 мс
//    sender   (ядро 0)  — отправка по Wi-Fi (UDP) и резервная запись на SD
//    display  (ядро 1, низкий приоритет) — экран и кнопка
//    wifi     (ядро 0, низкий приоритет) — подключение, режим настройки, команды
//             монитора порта (см. wifi_setup.h)
//
//  Нужно: плата «ESP32 Dev Module», ядро ESP32 3.x;
//  библиотеки «Adafruit ST7735 and ST7789 Library» (с зависимостями)
//  и «SparkFun MAX3010x Pulse and Proximity Sensor Library».
// =====================================================================
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include "MAX30105.h"
#include "config.h"
#include "wifi_setup.h"

#define FW_VERSION "2.1"

// ---- протокол v1
constexpr int      BASE_HZ  = 100;
constexpr int      SLOW_DIV = 5;                 // EDA 20 Гц
constexpr int      TICKS    = 10;                // тактов в пакете (100 мс)
constexpr int      NSLOW    = TICKS / SLOW_DIV;
constexpr uint32_t TICK_US  = 1000000 / BASE_HZ;
constexpr size_t   PKT_SIZE = 20 + TICKS * 4 * 2 + NSLOW * 4 * 2;
constexpr uint16_t FLAG_FINGER = 1 << 0;
constexpr uint16_t FLAG_SD_REC = 1 << 1;
constexpr int      GSR_OVERSAMPLE = 16;          // отсчёт EDA = сумма 16 чтений АЦП

// ---- выводы T-Display (зашиты в плате)
constexpr int TFT_MOSI_PIN = 19, TFT_SCLK_PIN = 18, TFT_CS_PIN = 5,
              TFT_DC_PIN = 16, TFT_RST_PIN = 23, TFT_BL_PIN = 4;
constexpr int BAT_ADC_PIN = 34, BAT_EN_PIN = 14;

struct Packet { uint8_t b[PKT_SIZE]; };

MAX30105 sensor;
Adafruit_ST7789 tft(&SPI, TFT_CS_PIN, TFT_DC_PIN, TFT_RST_PIN);
SPIClass sdSPI(HSPI);
File sdFile;
char sdName[24] = "";

QueueHandle_t qSend;
TaskHandle_t hSampler;
hw_timer_t* timer = nullptr;

// ---- состояние для экрана (пишет одна задача, читает другая)
volatile bool     stSensorOk = false, stFinger = false, stSdOk = false, stLinked = false;
volatile uint32_t stIr = 0, stSent = 0, stQueueDrop = 0, stPpgDup = 0, stPpgDrop = 0;
volatile int32_t  stEda = 0;
volatile uint32_t stBatMv = 0;

// =====================================================================
//  Таймер и выборка
// =====================================================================
void IRAM_ATTR onTick() {
  BaseType_t hp = pdFALSE;
  vTaskNotifyGiveFromISR(hSampler, &hp);
  if (hp) portYIELD_FROM_ISR();
}

static inline void put(uint8_t*& p, const void* src, size_t n) { memcpy(p, src, n); p += n; }

void samplerTask(void*) {
  uint64_t tick = 0;
  uint32_t seq = 0;
  int k = 0;
  uint64_t t0 = 0;
  uint32_t ir[TICKS], red[TICKS];
  int32_t eda[NSLOW], resp[NSLOW];

  // Датчик пульса живёт на своём генераторе, чуть отличающемся от нашего кварца.
  // Маленький буфер сглаживает разницу: лишний отсчёт отбрасываем, недостающий повторяем.
  uint32_t rIr[8], rRed[8];
  int rh = 0, rn = 0;
  uint32_t lastIr = 0, lastRed = 0;

  for (;;) {
    uint32_t n = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);   // сколько тактов прошло
    while (n--) {
      // ---- PPG, 100 Гц
      if (stSensorOk) {
        sensor.check();
        while (sensor.available()) {
          int w = (rh + rn) % 8;
          rIr[w] = sensor.getFIFOIR();
          rRed[w] = sensor.getFIFORed();
          if (rn < 8) rn++; else rh = (rh + 1) % 8;
          sensor.nextSample();
        }
        while (rn > 3) { rh = (rh + 1) % 8; rn--; stPpgDrop = stPpgDrop + 1; }
        if (rn > 0) { lastIr = rIr[rh]; lastRed = rRed[rh]; rh = (rh + 1) % 8; rn--; }
        else stPpgDup = stPpgDup + 1;
      }
      if (k == 0) t0 = tick * TICK_US;
      ir[k] = lastIr;
      red[k] = lastRed;

      // ---- GSR, 20 Гц: без усреднения во времени, только 16 быстрых чтений против шума АЦП
      if (k % SLOW_DIV == 0) {
        int32_t s = 0;
        for (int i = 0; i < GSR_OVERSAMPLE; i++) s += analogRead(PIN_GSR);
        eda[k / SLOW_DIV] = s;
        resp[k / SLOW_DIV] = 0;            // канала дыхания пока нет
        stEda = s;
      }
      // ---- батарея раз в секунду (здесь же, чтобы АЦП трогала одна задача)
      if (tick % 100 == 50) stBatMv = analogReadMilliVolts(BAT_ADC_PIN) * 2;

      k++;
      tick++;
      if (k == TICKS) {
        stIr = lastIr;
        stFinger = lastIr > FINGER_IR_MIN;
        uint16_t flags = (stFinger ? FLAG_FINGER : 0) | (stSdOk ? FLAG_SD_REC : 0);
        uint16_t nt = TICKS;
        Packet pk;
        uint8_t* p = pk.b;
        const uint8_t hdr[4] = {'I', 'D', 1, 1};
        put(p, hdr, 4);
        put(p, &seq, 4);
        put(p, &t0, 8);
        put(p, &nt, 2);
        put(p, &flags, 2);
        put(p, ir, sizeof ir);
        put(p, red, sizeof red);
        put(p, eda, sizeof eda);
        put(p, resp, sizeof resp);
        if (xQueueSend(qSend, &pk, 0) != pdTRUE) stQueueDrop = stQueueDrop + 1;   // никогда не ждём
        seq++;
        k = 0;
      }
    }
  }
}

// =====================================================================
//  Отправка и SD
// =====================================================================
void senderTask(void*) {
  WiFiUDP udp;
  bool udpOn = false;
  IPAddress peer, fixedIp;
  bool haveFixed = strlen(RECEIVER_IP) > 0 && fixedIp.fromString(RECEIVER_IP);
  uint32_t lastHello = 0, lastFlush = 0;
  Packet pk;

  for (;;) {
    if (xQueueReceive(qSend, &pk, pdMS_TO_TICKS(500)) != pdTRUE) continue;

    if (WiFi.status() == WL_CONNECTED) {
      if (!udpOn) udpOn = udp.begin(5007);
      // ответ компьютера «IDHI»: теперь знаем его адрес
      while (udp.parsePacket() > 0) {
        char buf[8];
        int r = udp.read(buf, sizeof buf);
        if (r >= 4 && memcmp(buf, "IDHI", 4) == 0) {
          peer = udp.remoteIP();
          lastHello = millis();
        }
      }
      bool linked = lastHello && millis() - lastHello < 10000;
      stLinked = linked;
      IPAddress dst = haveFixed ? fixedIp : (linked ? peer : WiFi.broadcastIP());
      udp.beginPacket(dst, RECEIVER_PORT);
      udp.write(pk.b, PKT_SIZE);
      if (udp.endPacket()) stSent = stSent + 1;
    } else {
      stLinked = false;
    }

#if SD_BACKUP
    if (sdFile) {
      if (sdFile.write(pk.b, PKT_SIZE) != PKT_SIZE) { sdFile.close(); stSdOk = false; }
      else if (millis() - lastFlush > 1000) { sdFile.flush(); lastFlush = millis(); }
    }
#endif
  }
}

// =====================================================================
//  Экран и кнопка
// =====================================================================
void line(int y, uint8_t size, uint16_t color, const char* fmt, ...) {
  char s[48];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(s, sizeof s, fmt, ap);
  va_end(ap);
  int width = 240 / (6 * size);
  char pad[48];
  snprintf(pad, sizeof pad, "%-*.*s", width, width, s);
  tft.setTextSize(size);
  tft.setTextColor(color, ST77XX_BLACK);
  tft.setCursor(0, y);
  tft.print(pad);
}

void printStatus() {
  Serial.printf("WiFi: %s %s  IP %s  режим настройки: %s\n",
                WiFi.status() == WL_CONNECTED ? "подключён к" : "нет связи", stSsid,
                WiFi.localIP().toString().c_str(), stApOn ? "вкл" : "выкл");
  Serial.printf("Компьютер: %s, отправлено пакетов %lu, потеряно в очереди %lu\n",
                stLinked ? "найден" : "не найден", (unsigned long)stSent, (unsigned long)stQueueDrop);
  Serial.printf("Пульс: %s, ИК %lu;  GSR %ld;  батарея %lu мВ;  SD: %s\n",
                stSensorOk ? (stFinger ? "палец есть" : "пальца нет") : "датчик не найден",
                (unsigned long)stIr, (long)stEda, (unsigned long)stBatMv, stSdOk ? sdName : "нет");
}

void displayTask(void*) {
  bool screenOn = true, btnPrev = true;
  uint32_t lastDraw = 0;
  for (;;) {
    // кнопка: экран вкл/выкл (чтобы не отвлекал клиента)
    bool btn = digitalRead(PIN_BTN_SCREEN);
    if (btnPrev && !btn) {
      screenOn = !screenOn;
      digitalWrite(TFT_BL_PIN, screenOn ? HIGH : LOW);
    }
    btnPrev = btn;

    if (screenOn && millis() - lastDraw > 500) {
      lastDraw = millis();
      bool wifi = WiFi.status() == WL_CONNECTED;
      line(0, 2, ST77XX_CYAN, "IDometer " FW_VERSION);
      if (stApOn && !wifi) {
        // режим настройки: подсказка вместо статуса сети
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
      line(92, 2, ST77XX_WHITE, "GSR: %ld", (long)stEda);
      uint32_t mv = stBatMv;
      line(114, 1, ST77XX_WHITE, "SD: %s  %s  sent %lu",
           stSdOk ? sdName + 5 : "off",
           mv > 4400 ? "USB" : (String("Bat ") + String(mv / 1000.0, 2) + "V").c_str(),
           (unsigned long)stSent);
      line(124, 1, 0x7BEF, "drop q%lu ppg+%lu/-%lu",
           (unsigned long)stQueueDrop, (unsigned long)stPpgDup, (unsigned long)stPpgDrop);
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// =====================================================================
//  Запуск
// =====================================================================
void openSdFile() {
#if SD_BACKUP
  sdSPI.begin(PIN_SD_SCLK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS);
  if (!SD.begin(PIN_SD_CS, sdSPI, 10000000)) { Serial.println("SD: нет карты"); return; }
  if (!SD.exists("/idm")) SD.mkdir("/idm");
  for (int i = 1; i < 10000; i++) {
    snprintf(sdName, sizeof sdName, "/idm/rec_%04d.bin", i);
    if (!SD.exists(sdName)) break;
  }
  sdFile = SD.open(sdName, FILE_WRITE);
  stSdOk = (bool)sdFile;
  Serial.printf("SD: %s %s\n", sdName, stSdOk ? "OK" : "ошибка открытия");
#endif
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\nIDometer " FW_VERSION);

  pinMode(PIN_BTN_SCREEN, INPUT);       // на GPIO35 нет внутренней подтяжки; на плате она есть
  pinMode(BAT_EN_PIN, OUTPUT);
  digitalWrite(BAT_EN_PIN, HIGH);       // включает делитель измерения батареи
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_GSR, ADC_11db);

  // экран
  pinMode(TFT_BL_PIN, OUTPUT);
  digitalWrite(TFT_BL_PIN, HIGH);
  SPI.begin(TFT_SCLK_PIN, -1, TFT_MOSI_PIN, TFT_CS_PIN);
  tft.init(135, 240);
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);
  line(0, 2, ST77XX_CYAN, "IDometer " FW_VERSION);
  line(30, 2, ST77XX_WHITE, "starting...");

  // датчик пульса: 100 Гц, ИК + красный, без усреднения в датчике
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  if (sensor.begin(Wire, I2C_SPEED_FAST)) {
    sensor.setup(PPG_LED_POWER, 1, 2, 100, 411, 4096);
    sensor.clearFIFO();
    stSensorOk = true;
  } else {
    Serial.println("MAX30102 не найден — работаем без пульса");
  }

  openSdFile();

  // правая кнопка при включении — сразу режим настройки Wi-Fi
  line(60, 1, ST77XX_YELLOW, "Hold right button: WiFi setup");
  uint32_t t = millis();
  bool held = true;
  while (millis() - t < 1500) { if (digitalRead(PIN_BTN_SCREEN)) held = false; delay(20); }
  forceApAtBoot = held;
  tft.fillScreen(ST77XX_BLACK);

  qSend = xQueueCreate(64, sizeof(Packet));   // 6,4 с запаса на случай заминок Wi-Fi/SD
  xTaskCreatePinnedToCore(samplerTask, "sampler", 4096, nullptr, configMAX_PRIORITIES - 2, &hSampler, 1);
  xTaskCreatePinnedToCore(senderTask, "sender", 6144, nullptr, 3, nullptr, 0);
  xTaskCreatePinnedToCore(displayTask, "display", 4096, nullptr, 1, nullptr, 1);
  xTaskCreatePinnedToCore(wifiTask, "wifi", 8192, nullptr, 1, nullptr, 0);

  // аппаратный таймер 100 Гц (API ядра ESP32 3.x)
  timer = timerBegin(1000000);
  timerAttachInterrupt(timer, &onTick);
  timerAlarm(timer, TICK_US, true, 0);
}

void loop() {
  vTaskDelete(nullptr);   // всё делают задачи
}
