#include "analogWrite.h"
#include <Arduino.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <SimpleDHT.h>

#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
#define DHTPIN 22     // Digital pin connected to the DHT sensor
#define HUMIDITY_PIN 36


//GPIO pin
const int bluePin = 16;
const int greenPin = 17;
const int redPin = 19;
const int pumpPin = 21;


// WiFi
const char *ssid = "TP-LINK_CC7E";
const char *password = "31460835";

// MQTT Broker
const char *mqtt_broker = "test.mosquitto.org";
const int mqtt_port = 1883;

//temp
unsigned long lastTempCheck = 0;
float temp = 20;
float airHum = 20;

//humidity
const int airValue = 0;
const int waterValue = 3500;
int soilHum = 10;
unsigned long lastSoilHumCheck = 0;
unsigned long lastDoseCheck = 0;
int minSoilHum = 20;

//intervals
static const int TELEMETRY_INTERVAL = 30 * 1000;
static const int SOIL_HUM_INTERVAL = 15 * 1000;
static const int TEMPERATURE_INTERVAL = 10 * 1000;
static const int DOSING_CHECK_INTERVAL = 60 * 1000;

WiFiClient espClient;
PubSubClient client(espClient);

String rootTopic;
String configTopic;
String telemetryTopic;
String pumpTopic;

int lastTelemetrySend = 0;

typedef struct Color {
    int red = 0;
    int blue = 0;
    int green = 0;
} Color;
Color firstColor;
Color secondColor;
int lightStart = 480;
int lightStop = 1200;
bool lightsOn;

boolean soccerMode = true;

//time stuff
const char *ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 7200;
const int daylightOffset_sec = 0;

SimpleDHT22 dht22(DHTPIN);

void connectToWifi();

void connectToMqtt();

void callback(char *topic, byte *payload, unsigned int length) {
    Serial.print("Message arrived in topic: ");
    Serial.println(topic);

    if (String(topic) == rootTopic) {
        Serial.println("a venit mesaj pe root");
    }

    if (String(topic) == configTopic) {
        Serial.println("a venit mesaj pe config");
        StaticJsonDocument<256> doc;
        deserializeJson(doc, payload);
        firstColor.red = doc["red1"];
        firstColor.green = doc["green1"];
        firstColor.blue = doc["blue1"];
        secondColor.red = doc["red2"];
        secondColor.green = doc["green2"];
        secondColor.blue = doc["blue2"];
    }

}

void connectToWifi() {
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.println("Connecting to WiFi..");
    }
    Serial.println("Connected to the WiFi network");
}


void connectToMqtt() {
    client.setServer(mqtt_broker, mqtt_port);
    client.setCallback(callback);
    delay(5000);
    Serial.println("connecting to mqtt...");
    while (!client.connected()) {
        String client_id = "esp32-client-";
        client_id += String(WiFi.macAddress());
        Serial.printf(" %s connects to the public mqtt broker\n", client_id.c_str());
        if (client.connect(client_id.c_str())) {
            Serial.println("connected");
        } else {
            Serial.print("failed with state ");
            Serial.print(client.state());
            delay(2000);
        }
    }
}

void configureMqttTopics() {
    rootTopic = String(WiFi.macAddress());
    configTopic = rootTopic + "-config";
    telemetryTopic = rootTopic + "-telemetry";
    pumpTopic = rootTopic + "-dose";

    Serial.println("telemetry topic: " + telemetryTopic);
    // publish and subscribe
    client.subscribe(rootTopic.c_str());
    client.subscribe(configTopic.c_str());

}

void readTemp() {
    if (millis() - lastTempCheck < TEMPERATURE_INTERVAL) {
        return;
    }
    lastTempCheck = millis();


    float temperature = 0;
    float humidity = 0;
    int err = SimpleDHTErrSuccess;
    if ((err = dht22.read2(&temperature, &humidity, NULL)) != SimpleDHTErrSuccess) {
        Serial.print("Read DHT22 failed, err=");
        Serial.print(SimpleDHTErrCode(err));
        Serial.print(",");
        Serial.println(SimpleDHTErrDuration(err));
        delay(2000);
        return;
    }

    Serial.print("Sample OK: ");
    Serial.print((float) temperature);
    Serial.print("*C temperature, ");
    Serial.print((float) humidity);
    Serial.println("% air humidity");

    temp = temperature;
    airHum = humidity;

    // DHT22 sampling rate is 0.5HZ.
//    delay(2500);

}

void readSoilHum() {
    if (millis() - lastSoilHumCheck < SOIL_HUM_INTERVAL) {
        return;
    }
    Serial.println("reading soil humidity...");
    lastSoilHumCheck = millis();
    int value = analogRead(HUMIDITY_PIN);

    Serial.print("Analog value: ");
    Serial.println(value);

    value = 4096 - value;

    Serial.print("Analog value: ");
    Serial.println(value);

    int soilmoisturepercent = map(value, airValue, waterValue, 0, 100);
    Serial.print("Moisture : ");
    Serial.print(soilmoisturepercent);
    Serial.println("%");
    soilHum = soilmoisturepercent;
}

void sendTelemetry() {
    if (millis() - lastTelemetrySend < TELEMETRY_INTERVAL) {
        return;
    }
    Serial.println("sending telemetry...");
    lastTelemetrySend = millis();
    StaticJsonDocument<300> doc;
    JsonObject jsonObject = doc.to<JsonObject>();
    jsonObject[F("temperature")] = temp;
    jsonObject[F("airHumidity")] = airHum;
    jsonObject[F("soilHumidity")] = soilHum;
    String telemetryMessage;
    serializeJson(doc, telemetryMessage);
    Serial.println("telemetry: " + telemetryMessage);
    client.publish(telemetryTopic.c_str(), telemetryMessage.c_str());
}

void dose() {
    Serial.println("starting motor...");
    analogWrite(pumpPin, 255);
    delay(2000);
    Serial.println("stopping motor...");
    analogWrite(pumpPin, 0);
}

void doseIfNeeded() {
    if (millis() - lastDoseCheck < DOSING_CHECK_INTERVAL) {
        return;
    }
    Serial.println("checking dose...");
    lastDoseCheck = millis();

    if (soilHum < minSoilHum) {
        dose();
    }

}

int getMinuteOfDay() {
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) {
        Serial.println("Failed to obtain time");
        return 0;
    }
    int minuteOfDay = timeinfo.tm_hour * 60 + timeinfo.tm_min;
    return minuteOfDay;
}

void turnLightOff() {
    if (lightsOn) {
        Serial.println("turning lights off...");
        analogWrite(redPin, 0);
        analogWrite(greenPin, 0);
        analogWrite(bluePin, 0);
    }
}

void turnLightOn() {
    if (!lightsOn) {
        lightsOn = true;
        Serial.println("turning lights on...");
        analogWrite(redPin, 200);
        analogWrite(greenPin, 200);
        analogWrite(bluePin, 20);
    }
}

void handleLights() {
    boolean normalMode = !soccerMode;
    if (normalMode) {
        int minuteOfDay = getMinuteOfDay();
        if (minuteOfDay < lightStart || minuteOfDay > lightStop) {
            turnLightOff();
        } else {
            turnLightOn();
        }
    } else {

        for (int i = 0; i < 100; i++) {
            long red = map(i, 0, 99, 0, firstColor.red);
            analogWrite(redPin, red);
            long green = map(i, 0, 99, 0, firstColor.green);
            analogWrite(greenPin, green);
            long blue = map(i, 0, 99, 0, firstColor.blue);
            analogWrite(bluePin, blue);
            delay(10);
        }

        for (int i = 99; i >= 0; i--) {
            long red = map(i, 0, 99, 0, firstColor.red);
            analogWrite(redPin, red);
            long green = map(i, 0, 99, 0, firstColor.green);
            analogWrite(greenPin, green);
            long blue = map(i, 0, 99, 0, firstColor.blue);
            analogWrite(bluePin, blue);
            delay(10);
        }

        for (int i = 0; i < 100; i++) {
            long red = map(i, 0, 99, 0, secondColor.red);
            analogWrite(redPin, red);
            long green = map(i, 0, 99, 0, secondColor.green);
            analogWrite(greenPin, green);
            long blue = map(i, 0, 99, 0, secondColor.blue);
            analogWrite(bluePin, blue);
//            Serial.print("red: ");
//            Serial.println(red);
//            Serial.print("green: ");
//            Serial.println(green);
//            Serial.print("blue: ");
//            Serial.println(blue);
            delay(10);
        }

        for (int i = 99; i >= 0; i--) {
            long red = map(i, 0, 99, 0, secondColor.red);
            analogWrite(redPin, red);
            long green = map(i, 0, 99, 0, secondColor.green);
            analogWrite(greenPin, green);
            long blue = map(i, 0, 99, 0, secondColor.blue);
            analogWrite(bluePin, blue);
//            Serial.print("red: ");
//            Serial.println(red);
//            Serial.print("green: ");
//            Serial.println(green);
//            Serial.print("blue: ");
//            Serial.println(blue);
            delay(10);
        }

        for (int i = 0; i < 3; i++) {
            analogWrite(redPin, firstColor.red);
            analogWrite(greenPin, firstColor.green);
            analogWrite(bluePin, firstColor.blue);
            delay(100);
            analogWrite(redPin, 0);
            analogWrite(greenPin, 0);
            analogWrite(bluePin, 0);
            delay(100);
            analogWrite(redPin, secondColor.red);
            analogWrite(greenPin, secondColor.green);
            analogWrite(bluePin, secondColor.blue);
            delay(100);
            analogWrite(redPin, 0);
            analogWrite(greenPin, 0);
            analogWrite(bluePin, 0);
            delay(400);
        }
    }

}


void configureColors() {
    firstColor.red = 200;
    firstColor.green = 0;
    firstColor.blue = 0;

    secondColor.red = 0;
    secondColor.green = 0;
    secondColor.blue = 250;
}

void setup() {
    Serial.begin(115200);

    connectToWifi();
    connectToMqtt();
    configureMqttTopics();
    configureColors();
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    turnLightOff();
}

void loop() {
    client.loop();
    readTemp();
    readSoilHum();
    sendTelemetry();
    doseIfNeeded();
    handleLights();
    delay(10);
}
