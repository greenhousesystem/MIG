//==================================================
// ESP32 SZKLARNIA V2 (wersja lekka)
// MQTT + HiveMQ Cloud, SHT31, czujnik gleby,
// pompka, wiatrak, WS2812B, Preferences
//==================================================

#define FASTLED_INTERNAL   // wycisza komunikaty FastLED przy kompilacji

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <Adafruit_SHT31.h>
#include <FastLED.h>
#include <Preferences.h>

//==================================================
// WIFI
//==================================================

static const char WIFI_SSID[] = "Morfeusz";
static const char WIFI_PASS[] = "123456789";

//==================================================
// HIVEMQ CLOUD
//==================================================

static const char MQTT_SERVER[]    = "548b79c9bba044dc8eb4cd3fc0d2767c.s1.eu.hivemq.cloud";
static const uint16_t MQTT_PORT    = 8883;
static const char MQTT_USER[]      = "esp32";
static const char MQTT_PASS[]      = "123456789";
static const char MQTT_CLIENT_ID[] = "ESP32_GREENHOUSE";

//==================================================
// TOPICI MQTT
//==================================================

static const char TOPIC_TEMP[]     = "greenhouse/sensors/temperature";
static const char TOPIC_HUM[]      = "greenhouse/sensors/humidity";
static const char TOPIC_SOIL[]     = "greenhouse/sensors/moisture_percent";
static const char TOPIC_STATUS[]   = "greenhouse/state/full";
static const char TOPIC_SETTINGS[] = "greenhouse/state/settings";
static const char TOPIC_PUMP[]     = "greenhouse/control/pump";
static const char TOPIC_FAN[]      = "greenhouse/control/fan";
static const char TOPIC_LED[]      = "greenhouse/control/led";
static const char TOPIC_CONFIG[]   = "greenhouse/control/settings";
static const char TOPIC_ONLINE[]   = "if_is_working/good";

//==================================================
// PINY
//==================================================

#define PIN_SOIL   34
#define PIN_FAN    26
#define PIN_PUMP   27
#define PIN_LED    25
#define NUM_LEDS   16

//==================================================
// KALIBRACJA GLEBY
//==================================================

static const int adcDry = 2715;
static const int adcWet = 2017;

//==================================================
// WARUNKI PRACY
//==================================================

struct Settings
{
    float tempMin = 18.0f;
    float tempMax = 28.0f;
    float humMin  = 40.0f;
    float humMax  = 60.0f;
    int   soilMin = 45;
    int   soilMax = 60;
};

static Settings settings;

//==================================================
// CZASY
//==================================================

static const unsigned long SENSOR_INTERVAL = 2000;
static const unsigned long SOIL_INTERVAL   = 5000;
static const unsigned long STATUS_INTERVAL = 5000;
static const unsigned long WATER_TIME      = 3000;
static const unsigned long SOAK_TIME       = 15000;
static const unsigned long MQTT_RETRY      = 5000;
static const unsigned long WIFI_RETRY      = 10000;

//==================================================
// OBIEKTY
//==================================================

static WiFiClientSecure tlsClient;
static PubSubClient mqtt(tlsClient);
static Adafruit_SHT31 sht31;
static Preferences prefs;
static CRGB leds[NUM_LEDS];

//==================================================
// DANE I STANY
//==================================================

static float temperature = 0;
static float humidity    = 0;
static int   soilHumidity = 100;   // start 100 = brak podlewania przed pierwszym odczytem
static bool  validAirData = false;
static bool  validSoil    = false;

static bool pumpState = false;
static bool fanState  = false;
static bool autoPump  = true;
static bool autoFan   = true;
static bool autoLED   = true;

static CRGB currentColor = CRGB::Black;

enum WaterState : uint8_t { IDLE, WATERING, SOAKING };
static WaterState waterState = IDLE;
static unsigned long waterTimer = 0;

static unsigned long lastSensor = 0;
static unsigned long lastSoil   = 0;
static unsigned long lastStatus = 0;
static unsigned long lastMQTT   = 0;
static unsigned long lastWiFi   = 0;

//==================================================
// STEROWANIE WYJSCIAMI
//==================================================

static void setPump(bool state)
{
    digitalWrite(PIN_PUMP, state);
    pumpState = state;
}

static void setFan(bool state)
{
    digitalWrite(PIN_FAN, state);
    fanState = state;
}

//==================================================
// LED
//==================================================

static void setColor(const CRGB &color)
{
    if (color == currentColor)
        return;

    currentColor = color;
    fill_solid(leds, NUM_LEDS, color);
    FastLED.show();
}

static void ledOff()
{
    fill_solid(leds, NUM_LEDS, CRGB::Black);
    FastLED.show();
    currentColor = CRGB::Black;
}

// "#RRGGBB" lub "RRGGBB" -> CRGB
static bool hexToColor(const char *hex, CRGB &out)
{
    if (*hex == '#')
        hex++;

    if (strlen(hex) != 6)
        return false;

    for (int i = 0; i < 6; i++)
        if (!isxdigit((unsigned char)hex[i]))
            return false;

    long v = strtol(hex, NULL, 16);
    out = CRGB((v >> 16) & 0xFF, (v >> 8) & 0xFF, v & 0xFF);
    return true;
}

//==================================================
// USTAWIENIA (Preferences)
//==================================================

static void validateSettings()
{
    settings.tempMin = constrain(settings.tempMin, 0.0f, 45.0f);
    settings.tempMax = constrain(settings.tempMax, settings.tempMin + 1.0f, 50.0f);
    settings.humMin  = constrain(settings.humMin, 0.0f, 95.0f);
    settings.humMax  = constrain(settings.humMax, settings.humMin + 5.0f, 100.0f);
    settings.soilMin = constrain(settings.soilMin, 0, 90);
    settings.soilMax = constrain(settings.soilMax, settings.soilMin + 5, 100);
}

static void saveSettings()
{
    validateSettings();

    prefs.begin("greenhouse", false);
    prefs.putBytes("settings", &settings, sizeof(settings));
    prefs.end();

    Serial.println(F("[CFG] Zapisano"));
}

static void loadSettings()
{
    prefs.begin("greenhouse", true);

    if (prefs.getBytesLength("settings") == sizeof(settings))
        prefs.getBytes("settings", &settings, sizeof(settings));

    prefs.end();

    validateSettings();
    Serial.println(F("[CFG] Wczytano"));
}

static void publishSettings()
{
    char json[160];

    snprintf(json, sizeof(json),
        "{\"temp_min\":%.1f,\"temp_max\":%.1f,"
        "\"hum_min\":%.1f,\"hum_max\":%.1f,"
        "\"soil_min\":%d,\"soil_max\":%d}",
        settings.tempMin, settings.tempMax,
        settings.humMin,  settings.humMax,
        settings.soilMin, settings.soilMax);

    mqtt.publish(TOPIC_SETTINGS, json, true);
}

// payload: "set:temp_min=20;temp_max=30;soil_min=40;..."
static void applySettings(char *payload)
{
    char *p = payload;

    if (strncmp(p, "set:", 4) == 0)
        p += 4;

    char *save = NULL;

    for (char *part = strtok_r(p, ";", &save);
         part;
         part = strtok_r(NULL, ";", &save))
    {
        char *eq = strchr(part, '=');

        if (!eq)
            continue;

        *eq = 0;
        const char *key = part;
        float v = atof(eq + 1);

        if      (!strcmp(key, "temp_min")) settings.tempMin = v;
        else if (!strcmp(key, "temp_max")) settings.tempMax = v;
        else if (!strcmp(key, "hum_min"))  settings.humMin  = v;
        else if (!strcmp(key, "hum_max"))  settings.humMax  = v;
        else if (!strcmp(key, "soil_min")) settings.soilMin = (int)v;
        else if (!strcmp(key, "soil_max")) settings.soilMax = (int)v;
    }

    saveSettings();      // zawiera validateSettings()
    publishSettings();
}

//==================================================
// ODCZYTY
//==================================================

static void readSoil()
{
    long sum = 0;

    for (int i = 0; i < 16; i++)
        sum += analogRead(PIN_SOIL);

    int adc = sum / 16;

    // odlaczony czujnik (ADC ~0) nie moze uruchamiac podlewania
    validSoil = (adc > 100);

    if (!validSoil)
        return;

    soilHumidity = constrain(map(adc, adcDry, adcWet, 0, 100), 0, 100);
}

static void publishSoil()
{
    readSoil();

    char buf[8];
    snprintf(buf, sizeof(buf), "%d", soilHumidity);
    mqtt.publish(TOPIC_SOIL, buf);
}

static void readSHT31()
{
    float t = sht31.readTemperature();
    float h = sht31.readHumidity();

    if (isnan(t) || isnan(h))
    {
        validAirData = false;
        return;
    }

    temperature = t;
    humidity    = h;
    validAirData = true;

    char buf[12];

    snprintf(buf, sizeof(buf), "%.2f", temperature);
    mqtt.publish(TOPIC_TEMP, buf);

    snprintf(buf, sizeof(buf), "%.2f", humidity);
    mqtt.publish(TOPIC_HUM, buf);
}

//==================================================
// STATUS DO STRONY
//==================================================

static void publishStatus()
{
    char json[360];

    snprintf(json, sizeof(json),
        "{\"temperature\":%.2f,\"humidity\":%.2f,\"moisture_percent\":%d,"
        "\"air_ok\":%s,\"soil_ok\":%s,"
        "\"pump\":%s,\"fan\":%s,"
        "\"auto_pump\":%s,\"auto_fan\":%s,\"auto_led\":%s,"
        "\"led\":\"%02X%02X%02X\",\"water_state\":%d}",
        temperature, humidity, soilHumidity,
        validAirData ? "true" : "false",
        validSoil    ? "true" : "false",
        pumpState    ? "true" : "false",
        fanState     ? "true" : "false",
        autoPump     ? "true" : "false",
        autoFan      ? "true" : "false",
        autoLED      ? "true" : "false",
        currentColor.r, currentColor.g, currentColor.b,
        (int)waterState);

    mqtt.publish(TOPIC_STATUS, json);
    mqtt.publish(TOPIC_ONLINE, "online", true);
}

//==================================================
// AUTOMATYKA
//==================================================

static void processWatering()
{
    unsigned long now = millis();

    switch (waterState)
    {
        case IDLE:
            if (autoPump && validSoil && soilHumidity < settings.soilMin)
            {
                setPump(true);
                waterTimer = now;
                waterState = WATERING;
                Serial.println(F("[PUMP] START"));
            }
            break;

        case WATERING:
            if (now - waterTimer >= WATER_TIME)
            {
                setPump(false);
                waterTimer = now;
                waterState = SOAKING;
                Serial.println(F("[PUMP] STOP"));
            }
            break;

        case SOAKING:
            if (now - waterTimer >= SOAK_TIME)
            {
                waterState = IDLE;
                Serial.println(F("[PUMP] READY"));
            }
            break;
    }
}

static void processFan()
{
    if (!autoFan || !validAirData)
        return;

    if (!fanState)
    {
        if (temperature > settings.tempMax || humidity > settings.humMax)
            setFan(true);

        return;
    }

    if (temperature < settings.tempMax - 1.0f &&
        humidity    < settings.humMax - 5.0f)
        setFan(false);
}

static void processLED()
{
    if (!autoLED)
        return;

    bool soilLow  = soilHumidity < settings.soilMin;
    bool soilHigh = soilHumidity > settings.soilMax;

    bool tempBad = validAirData &&
        (temperature < settings.tempMin || temperature > settings.tempMax);

    bool humBad = validAirData &&
        (humidity < settings.humMin || humidity > settings.humMax);

    if (soilLow || tempBad)  setColor(CRGB::Red);
    else if (soilHigh)       setColor(CRGB::Blue);
    else if (humBad)         setColor(CRGB::Yellow);
    else                     setColor(CRGB::Green);
}

//==================================================
// MQTT CALLBACK
//==================================================

static void mqttCallback(char *topic, byte *payload, unsigned int length)
{
    char msg[256];

    if (length >= sizeof(msg))
        length = sizeof(msg) - 1;

    memcpy(msg, payload, length);
    msg[length] = 0;

    // obciecie bialych znakow z konca
    while (length && isspace((unsigned char)msg[length - 1]))
        msg[--length] = 0;

    const char *m = msg;
    while (isspace((unsigned char)*m))
        m++;

    Serial.printf("[MQTT RX] %s : %s\n", topic, m);

    //---------------- POMPKA ----------------
    if (!strcmp(topic, TOPIC_PUMP))
    {
        if (!strcmp(m, "auto_pump"))
        {
            autoPump = true;
        }
        else if (!strcmp(m, "manual_pump"))
        {
            autoPump = false;

            if (waterState == WATERING)   // przerwij automatyczny cykl
            {
                setPump(false);
                waterState = IDLE;
            }
        }
        else if (!strcmp(m, "pompka_on"))
        {
            autoPump = false;
            waterState = IDLE;
            setPump(true);
        }
        else if (!strcmp(m, "pompka_off"))
        {
            autoPump = false;
            waterState = IDLE;
            setPump(false);
        }
        return;
    }

    //---------------- WIATRAK ----------------
    if (!strcmp(topic, TOPIC_FAN))
    {
        if      (!strcmp(m, "auto_fan"))   autoFan = true;
        else if (!strcmp(m, "manual_fan")) autoFan = false;
        else if (!strcmp(m, "wiatrak_on"))  { autoFan = false; setFan(true);  }
        else if (!strcmp(m, "wiatrak_off")) { autoFan = false; setFan(false); }
        return;
    }

    //---------------- LED ----------------
    if (!strcmp(topic, TOPIC_LED))
    {
        if (!strcmp(m, "auto_led"))
        {
            autoLED = true;
        }
        else if (!strcmp(m, "led_off"))
        {
            autoLED = false;
            ledOff();
        }
        else if (!strncmp(m, "led_custom:", 11))
        {
            autoLED = false;

            CRGB color;

            if (hexToColor(m + 11, color))
                setColor(color);
        }
        return;
    }

    //---------------- USTAWIENIA ----------------
    if (!strcmp(topic, TOPIC_CONFIG))
    {
        if (!strcmp(m, "get_settings"))
            publishSettings();
        else if (!strncmp(m, "set:", 4))
        {
            applySettings(msg + (m - msg));
            Serial.println(F("[CFG] UPDATED"));
        }
    }
}

//==================================================
// POLACZENIA
//==================================================

static void maintainWiFi()
{
    if (WiFi.status() == WL_CONNECTED)
        return;

    unsigned long now = millis();

    if (now - lastWiFi < WIFI_RETRY)
        return;

    lastWiFi = now;

    Serial.println(F("[WIFI] RECONNECT"));

    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASS);
}

static void maintainMQTT()
{
    if (WiFi.status() != WL_CONNECTED)
        return;

    if (mqtt.connected())
    {
        mqtt.loop();
        return;
    }

    unsigned long now = millis();

    if (now - lastMQTT < MQTT_RETRY)
        return;

    lastMQTT = now;

    Serial.println(F("[MQTT] CONNECTING..."));

    // LWT: gdy ESP padnie, broker ustawi "offline"
    if (mqtt.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASS,
                     TOPIC_ONLINE, 1, true, "offline"))
    {
        Serial.println(F("[MQTT] OK"));

        mqtt.subscribe(TOPIC_PUMP);
        mqtt.subscribe(TOPIC_FAN);
        mqtt.subscribe(TOPIC_LED);
        mqtt.subscribe(TOPIC_CONFIG);

        mqtt.publish(TOPIC_ONLINE, "online", true);

        publishSettings();
        publishStatus();
    }
    else
    {
        Serial.printf("[MQTT] BLAD rc=%d\n", mqtt.state());
    }
}

//==================================================
// SETUP
//==================================================

void setup()
{
    Serial.begin(115200);
    Serial.println(F("\n\nESP32 GREENHOUSE"));

    pinMode(PIN_PUMP, OUTPUT);
    pinMode(PIN_FAN, OUTPUT);
    pinMode(PIN_SOIL, INPUT);

    setPump(false);
    setFan(false);

    analogReadResolution(12);

    loadSettings();

    FastLED.addLeds<WS2812B, PIN_LED, GRB>(leds, NUM_LEDS);
    FastLED.clear();
    FastLED.show();
    setColor(CRGB::Green);

    if (sht31.begin(0x44))
        Serial.println(F("[SHT31] OK"));
    else
        Serial.println(F("[SHT31] ERROR"));

    readSoil();   // pierwszy odczyt, zanim ruszy automatyka

    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.begin(WIFI_SSID, WIFI_PASS);

    Serial.println(F("[WIFI] CONNECTING..."));

    unsigned long start = millis();

    while (WiFi.status() != WL_CONNECTED && millis() - start < 15000)
    {
        delay(250);
        Serial.print('.');
    }

    Serial.println();

    if (WiFi.status() == WL_CONNECTED)
        Serial.println(WiFi.localIP());

    tlsClient.setInsecure();
    tlsClient.setTimeout(5);   // krotszy timeout, loop nie blokuje sie dlugo

    mqtt.setServer(MQTT_SERVER, MQTT_PORT);
    mqtt.setCallback(mqttCallback);
    mqtt.setBufferSize(1024);

    lastMQTT = millis() - MQTT_RETRY;
    lastWiFi = millis();
}

//==================================================
// LOOP
//==================================================

void loop()
{
    maintainWiFi();
    maintainMQTT();

    unsigned long now = millis();

    if (now - lastSensor >= SENSOR_INTERVAL)
    {
        lastSensor = now;
        readSHT31();
    }

    if (now - lastSoil >= SOIL_INTERVAL)
    {
        lastSoil = now;
        publishSoil();
    }

    processWatering();
    processFan();
    processLED();

    if (now - lastStatus >= STATUS_INTERVAL)
    {
        lastStatus = now;
        publishStatus();
    }
}
