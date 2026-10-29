//V1.3 - hardware timers + multicore usage

#include <WiFi.h>
#include <PubSubClient.h>
#include <time.h>

// Параметры WiFi
const char* ssid = "yournetwork";
const char* password = "yourpassword";

// Параметры MQTT
const char* mqtt_server = "192.168.88.34";
const int mqtt_port = 1883;
const char* mqtt_user = "yourname";
const char* mqtt_password = "yourpass";

// Настройка NTP сервера
const char* ntpServer = "pool.ntp.org";

WiFiClient espClient;
PubSubClient client(espClient);

// Переменные для генерации данных
unsigned long counter = 0;
unsigned long startTime = 0;
unsigned long lastTimeSync = 0;
const long timeSyncInterval = 3600 * 1000; // Синхронизация времени каждый час

// Очередь для передачи данных между задачами
QueueHandle_t dataQueue;

// Переменные для GSR и BPM
int GSR = 800;
int BPM = 60;

// Таймеры
hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// Флаг для обработки данных в основном цикле
volatile bool timerFlag = false;

void IRAM_ATTR onTimer() {
    portENTER_CRITICAL_ISR(&timerMux);
    timerFlag = true;
    portEXIT_CRITICAL_ISR(&timerMux);
}

void setup() {
    Serial.begin(115200);
    setup_wifi();
    client.setServer(mqtt_server, mqtt_port);
    client.setCallback(callback);
    client.setBufferSize(256); // Увеличение буфера MQTT
    configTime(0, 0, ntpServer); // GMT+0
    lastTimeSync = millis();
    startTime = millis();

    // Создаем очередь для передачи данных
    dataQueue = xQueueCreate(10, sizeof(char) * 128); // Очередь для строковых сообщений

    // Настраиваем аппаратный таймер
    timer = timerBegin(0, 80, true); // Таймер 0, делитель 80 (1 мкс при тактовой частоте 80 МГц)
    timerAttachInterrupt(timer, &onTimer, true);
    timerAlarmWrite(timer, 200000, true); // 200 000 мкс = 200 мс
    timerAlarmEnable(timer); // Включаем таймер

    // Создание задачи для отправки данных
    xTaskCreatePinnedToCore(
        sendDataTask,     // Функция задачи
        "SendData",       // Имя задачи
        8192,             // Размер стека
        NULL,             // Параметры задачи
        1,                // Приоритет
        NULL,             // Дескриптор задачи
        1                 // Ядро ESP32
    );
}

void loop() {
    if (timerFlag) {
        portENTER_CRITICAL(&timerMux);
        timerFlag = false;
        portEXIT_CRITICAL(&timerMux);

        struct tm timeinfo;
        if (!getLocalTime(&timeinfo)) {
            Serial.println("Failed to obtain time");
            return;
        }

        char currentTime[64];
        strftime(currentTime, sizeof(currentTime), "%d-%b-%Y %H:%M:%S", &timeinfo);

        GSR = GSR < 1800 ? GSR + 1 : 800;
        BPM = BPM < 120 ? BPM + 1 : 60;

        // Создаем сообщение вручную, чтобы избежать использования String
        char message[128];
        snprintf(message, sizeof(message), "%s; %d; %d", currentTime, GSR, BPM);

        // Отправка данных в очередь
        if (xQueueSend(dataQueue, &message, portMAX_DELAY) != pdPASS) {
            Serial.println("Failed to send data to queue");
        }
    }
}

void sendDataTask(void *parameter) {
    char message[128];
    while (true) {
        if (xQueueReceive(dataQueue, &message, portMAX_DELAY) == pdTRUE) {
            // Проверка подключения
            if (!client.connected()) {
                reconnect();
            }

            client.publish("esp32/data", message, true);

            // Вывод данных о передаче
            Serial.println(message);

            counter++;

            if (counter >= 10000) {
                unsigned long endTime = millis();
                float totalTime = (endTime - startTime) / 1000.0;
                Serial.print("Completed sending 10000 messages. Time taken: ");
                Serial.print(totalTime, 3);
                Serial.println(" seconds.");
                while (true) {
                    delay(10000);
                }
            }
        } else {
            Serial.println("Failed to receive data from queue");
        }
    }
}

void setup_wifi() {
    Serial.println("Connecting to WiFi...");
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println("\nWiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
    Serial.print("Message arrived on topic: ");
    Serial.print(topic);
    Serial.print(". Message: ");
    for (int i = 0; i < length; i++) {
        Serial.print((char)payload[i]);
    }
    Serial.println();
}

void reconnect() {
    while (!client.connected()) {
        Serial.print("Attempting MQTT connection...");
        if (client.connect("ESP32Client", mqtt_user, mqtt_password)) {
            Serial.println("connected");
        } else {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            Serial.println(" try again in 5 seconds");
            delay(5000);
        }
    }
}
