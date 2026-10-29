//V 0.9134_beta working fine with 34 server, file name stored correctly. Pulse Sensor Fail if use TTGO-T1 dev (to check if driver/gpio.h may be excluded)
#include <WiFi.h>
#include <PubSubClient.h>
#include "time.h"
#include "driver/gpio.h" //new requirement?
#include <TFT_eSPI.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"

#define MY_CS 33
#define MY_SCLK 25
#define MY_MISO 27
#define MY_MOSI 26
#define StartButtonPin 0
#define StopButtonPin 35
#define DEBOUNCE_DELAY 50

const char* ssid = "yournetwork";
const char* password = "yourpassword";
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 0;
const int daylightOffset_sec = 3600;

const char* mqtt_server = "192.168.88.34"; // RPI server address
const int mqtt_port = 1883; 
const char* mqtt_user = "yourname";
const char* mqtt_password = "yourpass";

const int GSRPin = 36;
const int gsrNumReadings = 10;
int gsrReadings[10];
int gsrIndex = 0;
int gsrTotal = 0;
int gsrAverage = 0;
int averageBPM = 0;

const int RATE_SIZE = 4;

WiFiClient espClient;
PubSubClient mqttClient(espClient);

TFT_eSPI tft = TFT_eSPI();
SPIClass SDSPI(HSPI);
bool recording = false;
File dataFile;
MAX30105 particleSensor;

unsigned long lastButtonPress = 0;

void setup() {
    Serial.begin(115200);
    tft.begin();
    tft.setRotation(0);
    tft.fillScreen(TFT_BLACK);
    updateDisplayAndSerial("IDOmtr_0.9134_beta", 0);
    updateDisplayAndSerial("__________________", 20);
    initializeSDCard();
    connectToWiFi();
    connectToMQTT();
    synchronizeTime();
    initializeSensors();
    pinMode(GSRPin, INPUT);
    pinMode(StartButtonPin, INPUT_PULLUP);
    pinMode(StopButtonPin, INPUT_PULLUP);
    updateDisplayAndSerial("Ready to record", 180);
    updateDisplayAndSerial("L_btn to start,", 200);
    updateDisplayAndSerial("R_btn to stop..", 220);
}

void loop() {
    mqttClient.loop(); // Поддерживает подключение к брокеру активным
    handleSensorReadings();
    handleButtons();
}

void initializeSDCard() {
    SDSPI.begin(MY_SCLK, MY_MISO, MY_MOSI, MY_CS);
    if (!SD.begin(MY_CS, SDSPI)) {
        updateDisplayAndSerial("SD Mount Fail", 40);
        return;
    }
    updateDisplayAndSerial("SD Mount OK", 40);
}
/* //bugfix attempt to resolve setup after reset
void connectToWiFi() {
    updateDisplayAndSerial("Connecting WiFi..", 60);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    updateDisplayAndSerial("WiFi Connected", 80);
}
*/
void connectToWiFi() {
    updateDisplayAndSerial("Connecting WiFi..", 60);
    WiFi.begin(ssid, password);
    
    unsigned long startTime = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startTime < 10000) {
        delay(500);
        Serial.print(".");
    }

    if (WiFi.status() == WL_CONNECTED) {
        updateDisplayAndSerial("WiFi Connected", 80);
    } else {
        updateDisplayAndSerial("WiFi Fail", 80);
    }
}

void connectToMQTT() {
    mqttClient.setServer(mqtt_server, mqtt_port);
    while (!mqttClient.connected()) {
        Serial.print("Attempting MQTT connection...");
        if (mqttClient.connect("ESP32Client", mqtt_user, mqtt_password)) {
            Serial.println("connected");
        } else {
            Serial.print("failed, rc=");
            Serial.print(mqttClient.state());
            Serial.println(" try again in 5 seconds");
            delay(5000);
        }
    }
}

void synchronizeTime() {
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    updateDisplayAndSerial("Syncing Time...", 100);

    unsigned long startAttemptTime = millis();
    bool timeSynced = false;

    while (millis() - startAttemptTime < 8000) { // Даем 8 секунд на синхронизацию
        struct tm timeinfo;
        if (getLocalTime(&timeinfo)) {
            updateDisplayAndSerial("Time Sync OK", 120);
            timeSynced = true;
            break;
        }
    }

    if (!timeSynced) {
        updateDisplayAndSerial("Time Sync Fail", 120);
    }
}


void resetMax30102() {
    pinMode(21, OUTPUT);
    digitalWrite(21, LOW);
    unsigned long startTime = millis();
    while (millis() - startTime < 10);  // Неблокирующая задержка
    pinMode(21, INPUT);
}

void initializeSensors() {
    resetMax30102();  // Принудительный сброс сенсора перед запуском
    Wire.begin(21, 22);  // Указываем SDA и SCL для ESP32

    updateDisplayAndSerial("Sensor init...", 140);
    updateDisplayAndSerial("__________________", 160);
    int attempts = 5;
    unsigned long lastAttemptTime = millis();
    bool sensorInitialized = false;

    while (attempts > 0) {
        if (millis() - lastAttemptTime >= 500) {  // Ожидание между попытками
            lastAttemptTime = millis();

            if (particleSensor.begin(Wire, I2C_SPEED_STANDARD)) {
                particleSensor.setup();
                particleSensor.setPulseAmplitudeRed(0x0A);
                particleSensor.setPulseAmplitudeGreen(0);
                updateDisplayAndSerial("PulseSensor - OK", 140);
                sensorInitialized = true;
                break;
            }
            attempts--;
        }
    }

    if (!sensorInitialized) {
        updateDisplayAndSerial("PulseSensor Fail", 140);
        while (1);  // Остановка, если сенсор не инициализировался
    }
}

void handleSensorReadings() {
    handleGSRReadings();
    handlePulseSensorReadings();
}

void handleGSRReadings() {
    static unsigned long lastGsrUpdateTime = 0;
    const unsigned long gsrUpdateInterval = 200;

    if (millis() - lastGsrUpdateTime >= gsrUpdateInterval) {
        lastGsrUpdateTime = millis();
        int gsrValue = analogRead(GSRPin);
        gsrTotal -= gsrReadings[gsrIndex];
        gsrReadings[gsrIndex] = gsrValue;
        gsrTotal += gsrReadings[gsrIndex];
        gsrIndex = (gsrIndex + 1) % gsrNumReadings;
        gsrAverage = gsrTotal / gsrNumReadings;
        
        struct tm timeinfo;
        if (getLocalTime(&timeinfo)) {
            char timestamp[20];
            strftime(timestamp, sizeof(timestamp), "%Y-%m-%d %H:%M:%S", &timeinfo);
            Serial.printf("%s;%d;%d\n", timestamp, gsrAverage, averageBPM);
        }
        logAndSendData(); // Запись данных в файл и отправка через MQTT
    }
}

void handlePulseSensorReadings() {
    static unsigned long lastBeatTime = 0;
    static float beatsPerMinute;
    static int beatAvg = 0;
    bool fingerDetected = particleSensor.getIR() > 50000;

    if (checkForBeat(particleSensor.getIR()) == true) {
        unsigned long beatDeltaTime = millis() - lastBeatTime;
        lastBeatTime = millis();
        beatsPerMinute = 60 / (beatDeltaTime / 1000.0);

        if (beatsPerMinute >= 40 && beatsPerMinute <= 180) {
            static byte rates[RATE_SIZE];
            static byte rateSpot = 0;
            rates[rateSpot++] = (byte)beatsPerMinute;
            rateSpot %= RATE_SIZE;
            beatAvg = 0;
            for (byte x = 0; x < RATE_SIZE; x++) {
                beatAvg += rates[x];
            }
            averageBPM = beatAvg / RATE_SIZE;
            logAndSendData(); // Запись данных в файл и отправка через MQTT
        }
    }
}

void updateDisplayAndSerial(const char* message, int y) {
    tft.drawString(message, 0, y, 2);
    Serial.println(message);
}

void startRecording() {
    if (millis() - lastButtonPress > DEBOUNCE_DELAY) {
        lastButtonPress = millis();
        recording = true;
        tft.fillScreen(TFT_BLACK);
        updateDisplayAndSerial("Recording started", 20);
        char fileName[64];
        struct tm timeinfo;
        if (!getLocalTime(&timeinfo)) {
            updateDisplayAndSerial("Failed to obtain time", 20);
            return;
        }
        snprintf(fileName, sizeof(fileName), "/DataLogs/dataLog_%04d-%02d-%02d_%02d-%02d-%02d.csv",
         timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
         timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);

        dataFile = SD.open(fileName, FILE_WRITE);
        if (!dataFile) {
            updateDisplayAndSerial("Error opening file", 40);
            return;
        }
        dataFile.println("Time,GSR,HeartRate");
    }
}

void stopRecording() {
    if (millis() - lastButtonPress > DEBOUNCE_DELAY && recording) {
        lastButtonPress = millis();
        recording = false;
        tft.fillScreen(TFT_BLACK);
        updateDisplayAndSerial("Recording stopped", 20);
        if (dataFile) {
            dataFile.close();
        }
    }
}

void handleButtons() {
    if (digitalRead(StartButtonPin) == LOW && !recording) {
        startRecording();
    } else if (digitalRead(StopButtonPin) == LOW && recording) {
        stopRecording();
    }
}

void logAndSendData() {
    if (!recording || !dataFile) return;

    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) return;

    // Корректируем время на один час назад
    //timeinfo.tm_hour -= 1; // оставляем время без изменений
    mktime(&timeinfo); // Пересчитываем время с учетом изменений

    // Формат для записи в файл: "YYYY-MM-DD HH:MM:SS"
    char fileTimestamp[20];
    strftime(fileTimestamp, sizeof(fileTimestamp), "%Y-%m-%d %H:%M:%S", &timeinfo);
    dataFile.printf("%s,%d,%d\n", fileTimestamp, gsrAverage, averageBPM);

    // Формат для бродкаста: "DD-MMM-YYYY HH:MM:SS" без миллисекунд
    char mqttTimestamp[64];
    strftime(mqttTimestamp, sizeof(mqttTimestamp), "%d-%b-%Y %H:%M:%S", &timeinfo);

    // Создаем строку сообщения для бродкаста
    String message = String(mqttTimestamp) + "; " + String(gsrAverage) + "; " + String(averageBPM);

    // Отправляем сообщение на MQTT брокер
    mqttClient.publish("esp32/data", message.c_str(), true);
}



