#include <WiFi.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "DHT.h"
#include <LiquidCrystal_I2C.h>

/* ================== PIN ================== */
#define DHTPIN 4
#define DHTTYPE DHT11

#define RELAY_PIN 18        // KY-019 (Active LOW)
#define ACTIVE_LOW 0

#define TRIG_PIN 13
#define ECHO_PIN 12

#define BUTTON_PIN 27
#define LONG_PRESS_TIME 1500   // ms

/* ================== MQTT ================== */
const char* MQTT_HOST    = "app.coreiot.io";
const int   MQTT_PORT    = 1883;
const char* ACCESS_TOKEN = "ACCESS_TOKEN";

WiFiClient wifiClient;
PubSubClient client(wifiClient);

/* ================== DEVICE ================== */
DHT dht(DHTPIN, DHTTYPE);
LiquidCrystal_I2C lcd(0x27, 16, 2);

/* ================== STATE ================== */
bool  relayState        = false;
bool  fanAuto           = true;

float tempThreshold     = 27.5;
int   distanceThreshold = 20;

float temp = 0, hum = 0;
int   distanceCM = 0;

/* ================== BUTTON ================== */
bool lastButtonState = HIGH;
unsigned long buttonPressTime = 0;
bool longPressHandled = false;

/* ================== RELAY ================== */
void setRelay(bool on)
{
    relayState = on;
    digitalWrite(RELAY_PIN, ACTIVE_LOW ? !on : on);
}

/* ================== ULTRASONIC ================== */
int readDistanceCM()
{
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    long duration = pulseIn(ECHO_PIN, HIGH, 30000);
    if (duration == 0) return 999;

    return duration * 0.034 / 2;
}

/* ================== BUTTON ================== */
void handleButton()
{
    bool currentState = digitalRead(BUTTON_PIN);

    // Nhấn xuống
    if (lastButtonState == HIGH && currentState == LOW)
    {
        buttonPressTime = millis();
        longPressHandled = false;
    }

    // Giữ lâu → đổi AUTO / MANUAL
    if (currentState == LOW && !longPressHandled)
    {
        if (millis() - buttonPressTime >= LONG_PRESS_TIME)
        {
            fanAuto = !fanAuto;
            longPressHandled = true;

            Serial.print("Mode -> ");
            Serial.println(fanAuto ? "AUTO" : "MANUAL");

            if (!fanAuto)
                setRelay(false);
        }
    }

    // Nhấn ngắn trong MANUAL
    if (lastButtonState == LOW && currentState == HIGH)
    {
        if (!longPressHandled && !fanAuto)
        {
            setRelay(!relayState);
            Serial.println(relayState ? "Relay ON (MANUAL)" : "Relay OFF (MANUAL)");
        }
    }

    lastButtonState = currentState;
}

/* ================== MQTT CALLBACK ================== */
void callback(char* topic, byte* payload, unsigned int length)
{
    payload[length] = 0;

    StaticJsonDocument<256> doc;
    if (deserializeJson(doc, payload)) return;

    String method = doc["method"];

    if (method == "setStateFan")
        fanAuto = doc["params"];

    if (method == "setTempThreshold")
        tempThreshold = doc["params"];

    if (method == "setDistanceThreshold")
        distanceThreshold = doc["params"];

    if (method == "setRelay" && !fanAuto)
        setRelay(doc["params"]);
}

/* ================== WIFI ================== */
void connectWiFi()
{
    WiFiManager wm;
    wm.setConfigPortalTimeout(180);

    if (!wm.autoConnect("ESP32-CONFIG-PHUC-AN", "password"))
        ESP.restart();
}

/* ================== MQTT ================== */
void connectMQTT()
{
    client.setServer(MQTT_HOST, MQTT_PORT);
    client.setCallback(callback);

    while (!client.connected())
    {
        if (client.connect("ESP32", ACCESS_TOKEN, NULL))
            client.subscribe("v1/devices/me/rpc/request/+");
        else
            delay(1000);
    }
}

/* ================== SETUP ================== */
void setup()
{
    Serial.begin(115200);

    pinMode(RELAY_PIN, OUTPUT);
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    pinMode(BUTTON_PIN, INPUT_PULLUP);

    setRelay(false);

    dht.begin();
    lcd.init();
    lcd.backlight();

    connectWiFi();
    connectMQTT();
}

/* ================== LOOP ================== */
void loop()
{
    handleButton();

    if (!client.connected())
        connectMQTT();
    client.loop();

    static unsigned long lastRead = 0;
    if (millis() - lastRead < 2000) return;
    lastRead = millis();

    temp = dht.readTemperature();
    hum  = dht.readHumidity();
    distanceCM = readDistanceCM();

    if (fanAuto)
    {
        if (temp > tempThreshold && distanceCM < distanceThreshold)
            setRelay(true);
        else
            setRelay(false);
    }

    /* ===== LCD ===== */
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.printf("T:%.1f>%.1f %s", temp, tempThreshold, fanAuto ? "A" : "M");

    lcd.setCursor(0, 1);
    lcd.printf("H:%.0f D:%d<%d %s", hum, distanceCM, distanceThreshold, relayState ? "ON" : "OFF");

    StaticJsonDocument<256> doc;
    doc["temperature"]       = temp;
    doc["humidity"]          = hum;
    doc["distance"]          = distanceCM;
    doc["relay"]             = relayState;
    doc["fanAuto"]           = fanAuto;
    doc["tempThreshold"]     = tempThreshold;
    doc["distanceThreshold"] = distanceThreshold;

    char buffer[256];
    serializeJson(doc, buffer);
    client.publish("v1/devices/me/telemetry", buffer);

    Serial.printf(
        "T=%.1f H=%.1f D=%d Relay=%d Mode=%s\n",
        temp, hum, distanceCM, relayState,
        fanAuto ? "AUTO" : "MANUAL"
    );
}
