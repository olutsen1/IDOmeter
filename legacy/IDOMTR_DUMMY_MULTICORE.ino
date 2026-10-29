//V1.3 - hardware timers + multicore usage

#include <WiFi.h>
#include <PubSubClient.h>
#include <time.h>

// Wi-Fi settings
const char* ssid = "yournetwork";
const char* password = "yourpassword";

// MQTT settings
const char* mqtt_server = "192.168.88.34";
const int mqtt_port = 1883;
const char* mqtt_user = "yourname";
const char* mqtt_password = "yourpass";

// NTP server
const char* ntpServer = "pool.ntp.org";

WiFiClient espClient;
PubSubClient client(espClient);

// Data generation variables
unsigned long counter = 0;
unsigned long startTime = 0;
unsigned long lastTimeSync = 0;
const long timeSyncInterval = 3600 * 1000; // Time sync every hour

// Queue for passing data between tasks
QueueHandle_t dataQueue;

// GSR and BPM variables
int GSR = 800;
int BPM = 60;

// Timers
hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// Flag for processing data in the main loop
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
    client.setBufferSize(256); // Larger MQTT buffer
    configTime(0, 0, ntpServer); // GMT+0
    lastTimeSync = millis();
    startTime = millis();

    // Create the data queue
    dataQueue = xQueueCreate(10, sizeof(char) * 128); // Queue of string messages

    // Configure the hardware timer
    timer = timerBegin(0, 80, true); // Timer 0, prescaler 80 (1 µs at 80 MHz clock)
    timerAttachInterrupt(timer, &onTimer, true);
    timerAlarmWrite(timer, 200000, true); // 200 000 µs = 200 ms
    timerAlarmEnable(timer); // Enable the timer

    // Create the data sending task
    xTaskCreatePinnedToCore(
        sendDataTask,     // Task function
        "SendData",       // Task name
        8192,             // Stack size
        NULL,             // Task parameters
        1,                // Priority
        NULL,             // Task handle
        1                 // ESP32 core
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

        // Build the message by hand to avoid String
        char message[128];
        snprintf(message, sizeof(message), "%s; %d; %d", currentTime, GSR, BPM);

        // Push data to the queue
        if (xQueueSend(dataQueue, &message, portMAX_DELAY) != pdPASS) {
            Serial.println("Failed to send data to queue");
        }
    }
}

void sendDataTask(void *parameter) {
    char message[128];
    while (true) {
        if (xQueueReceive(dataQueue, &message, portMAX_DELAY) == pdTRUE) {
            // Check the connection
            if (!client.connected()) {
                reconnect();
            }

            client.publish("esp32/data", message, true);

            // Print what was sent
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
