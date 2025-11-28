/* 
  ESP32 — Standalone MQTT Sensor Hub
  - Publishes sensor data (DHT11, BMP280, LDR, MQ135) to MQTT
  - Controls FAN and LIGHT via MQTT commands
  - No Nano communication (removed)
  - Improved sensor readings with error handling
*/

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <Adafruit_BMP280.h>
#include <DHT.h>
#include <ArduinoJson.h>

// ===== CONFIG =====
const char* WIFI_SSID = "Tanmay";
const char* WIFI_PASS = "tnmy9416";

const char* MQTT_SERVER = "afc8a0ac2ccf462c8f92b932403518df.s1.eu.hivemq.cloud";
const uint16_t MQTT_PORT = 8883;
const char* MQTT_USER = "hivemq.webclient.1761751067946";
const char* MQTT_PASS = "wy.b7f8cB*0GTUW&4Ha:";

const char* TOPIC_SENSORS = "project/sensors";
const char* TOPIC_STATUS  = "project/status";
const char* TOPIC_CONTROL = "project/control";

// ===== PINS =====
#define DHTPIN 4
#define DHTTYPE DHT11
#define LDR_PIN 34
#define MQ135_PIN 35

#define FAN_PIN 26
#define LIGHT_PIN 27
#define DOOR_PIN 25

// ===== HARDWARE OBJECTS =====
WiFiClientSecure wifiClient;
PubSubClient mqttClient(wifiClient);
Adafruit_BMP280 bmp;
DHT dht(DHTPIN, DHT11);

// ===== STATE =====
unsigned long lastPublish = 0;
const unsigned long PUBLISH_INTERVAL = 5000; // 5 seconds

bool fanState = false;
bool lightState = false;
bool doorState = false; // false = closed, true = open
bool bmpAvailable = false;

// Sensor reading smoothing
float lastDhtTemp = 0.0;
float lastDhtHum = 0.0;
int dhtFailCount = 0;

// ===== FUNCTION DECLARATIONS =====
void publishStatus(const char* deviceStatus);
void publishSensors();
void mqttCallback(char* topic, byte* payload, unsigned int length);

// ===== WiFi CONNECTION =====
void ensureWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  
  Serial.print("WiFi: connecting to ");
  Serial.println(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi connected! IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("WiFi connection failed - will retry");
  }
}

// ===== MQTT CONNECTION =====
void mqttReconnect() {
  if (mqttClient.connected()) return;
  if (WiFi.status() != WL_CONNECTED) return;

  wifiClient.setInsecure(); // For HiveMQ Cloud with self-signed cert

  String clientId = "esp32-sensor-" + String(random(0xffff), HEX);
  Serial.print("MQTT: connecting as ");
  Serial.println(clientId);

  if (mqttClient.connect(clientId.c_str(), MQTT_USER, MQTT_PASS)) {
    Serial.println("✓ MQTT connected");
    
    // Subscribe to control topic
    if (mqttClient.subscribe(TOPIC_CONTROL)) {
      Serial.print("✓ Subscribed to: ");
      Serial.println(TOPIC_CONTROL);
    } else {
      Serial.println("✗ Subscribe failed");
    }
    
    // Publish online status
    publishStatus("online");
    
  } else {
    Serial.print("✗ MQTT connect failed, rc=");
    Serial.println(mqttClient.state());
  }
}

// ===== PUBLISH STATUS =====
void publishStatus(const char* deviceStatus = nullptr) {
  if (!mqttClient.connected()) return;
  
  StaticJsonDocument<256> doc;
  if (deviceStatus) {
    doc["device"] = "esp32-sensor-hub";
    doc["status"] = deviceStatus;
  }
  doc["fan"] = fanState;
  doc["light"] = lightState;
  doc["door"] = doorState ? "open" : "closed";
  doc["timestamp"] = millis();
  doc["wifi_rssi"] = WiFi.RSSI();
  
  String output;
  serializeJson(doc, output);
  
  bool published = mqttClient.publish(TOPIC_STATUS, output.c_str(), true);
  if (published) {
    Serial.println("✓ Status published");
  }
}

// ===== READ SENSORS WITH IMPROVED ERROR HANDLING =====
void publishSensors() {
  if (!mqttClient.connected()) {
    Serial.println("✗ MQTT not connected, skipping sensor publish");
    return;
  }

  StaticJsonDocument<512> doc;
  
  // === DHT11 Reading (with retry and smoothing) ===
  float temp = dht.readTemperature();
  float hum = dht.readHumidity();
  
  if (isnan(temp) || isnan(hum)) {
    dhtFailCount++;
    if (dhtFailCount < 3) {
      // Use last known good values
      temp = lastDhtTemp;
      hum = lastDhtHum;
      doc["dht_status"] = "using_cached";
    } else {
      temp = 0.0;
      hum = 0.0;
      doc["dht_status"] = "error";
    }
  } else {
    dhtFailCount = 0;
    lastDhtTemp = temp;
    lastDhtHum = hum;
    doc["dht_status"] = "ok";
  }
  
  doc["dht_temp"] = round(temp * 10) / 10.0; // Round to 1 decimal
  doc["dht_humidity"] = round(hum * 10) / 10.0;
  
  // === BMP280 Reading ===
  if (bmpAvailable) {
    float bmpTemp = bmp.readTemperature();
    float pressure = bmp.readPressure() / 100.0F; // Convert Pa to hPa
    
    if (!isnan(bmpTemp) && !isnan(pressure)) {
      doc["bmp_temp"] = round(bmpTemp * 10) / 10.0;
      doc["pressure_hpa"] = round(pressure * 10) / 10.0;
      doc["bmp_status"] = "ok";
    } else {
      doc["bmp_status"] = "error";
    }
  } else {
    doc["bmp_status"] = "not_available";
  }
  
  // === LDR Reading (Light Sensor) ===
  // Take multiple readings and average
  int ldrSum = 0;
  for (int i = 0; i < 5; i++) {
    ldrSum += analogRead(LDR_PIN);
    delay(10);
  }
  int rawLdr = ldrSum / 5;
  
  // Convert to percentage (inverted: darker = lower value on ADC)
  float lightPercent = ((4095.0f - (float)rawLdr) / 4095.0f) * 100.0f;
  lightPercent = constrain(lightPercent, 0.0, 100.0);
  
  doc["light_percent"] = round(lightPercent * 10) / 10.0;
  doc["ldr_raw"] = rawLdr;
  
  // === MQ135 Air Quality Sensor ===
  // Take multiple readings and average
  int mqSum = 0;
  for (int i = 0; i < 5; i++) {
    mqSum += analogRead(MQ135_PIN);
    delay(10);
  }
  int mq = mqSum / 5;
  
  doc["air_quality_raw"] = mq;
  // Simple air quality assessment (adjust thresholds as needed)
  if (mq < 400) {
    doc["air_quality"] = "good";
  } else if (mq < 1000) {
    doc["air_quality"] = "moderate";
  } else {
    doc["air_quality"] = "poor";
  }
  
  // === Device Status ===
  doc["fan"] = fanState;
  doc["light"] = lightState;
  doc["door"] = doorState ? "open" : "closed";
  doc["wifi_rssi"] = WiFi.RSSI();
  doc["uptime_ms"] = millis();
  doc["free_heap"] = ESP.getFreeHeap();
  
  // === Publish ===
  String output;
  serializeJson(doc, output);
  
  bool published = mqttClient.publish(TOPIC_SENSORS, output.c_str());
  
  if (published) {
    Serial.println("✓ Sensors published");
    Serial.print("  Temp: "); Serial.print(temp); Serial.println("°C");
    Serial.print("  Humidity: "); Serial.print(hum); Serial.println("%");
    Serial.print("  Light: "); Serial.print(lightPercent); Serial.println("%");
    Serial.print("  Air Quality: "); Serial.println(mq);
  } else {
    Serial.println("✗ Sensor publish failed");
  }
}

// ===== MQTT CALLBACK (Handle Commands) =====
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String msg = "";
  for (unsigned int i = 0; i < length; i++) {
    msg += (char)payload[i];
  }
  msg.trim();
  msg.toLowerCase(); // Case insensitive
  
  Serial.print("📨 MQTT message on ");
  Serial.print(topic);
  Serial.print(": ");
  Serial.println(msg);

  if (String(topic) == TOPIC_CONTROL) {
    bool stateChanged = false;
    
    // Fan controls
    if (msg == "fan-on" || msg == "fan:on" || msg == "fan:1") {
      fanState = true;
      digitalWrite(FAN_PIN, HIGH);
      stateChanged = true;
      Serial.println("✓ Fan turned ON");
    }
    else if (msg == "fan-off" || msg == "fan:off" || msg == "fan:0") {
      fanState = false;
      digitalWrite(FAN_PIN, LOW);
      stateChanged = true;
      Serial.println("✓ Fan turned OFF");
    }
    else if (msg == "fan-toggle" || msg == "fan:toggle") {
      fanState = !fanState;
      digitalWrite(FAN_PIN, fanState ? HIGH : LOW);
      stateChanged = true;
      Serial.print("✓ Fan toggled to: ");
      Serial.println(fanState ? "ON" : "OFF");
    }
    
    // Light controls
    else if (msg == "light-on" || msg == "light:on" || msg == "light:1") {
      lightState = true;
      digitalWrite(LIGHT_PIN, HIGH);
      stateChanged = true;
      Serial.println("✓ Light turned ON");
    }
    else if (msg == "light-off" || msg == "light:off" || msg == "light:0") {
      lightState = false;
      digitalWrite(LIGHT_PIN, LOW);
      stateChanged = true;
      Serial.println("✓ Light turned OFF");
    }
    else if (msg == "light-toggle" || msg == "light:toggle") {
      lightState = !lightState;
      digitalWrite(LIGHT_PIN, lightState ? HIGH : LOW);
      stateChanged = true;
      Serial.print("✓ Light toggled to: ");
      Serial.println(lightState ? "ON" : "OFF");
    }
    
    // Door controls
    else if (msg == "door-open" || msg == "door:open" || msg == "door:1") {
      doorState = true;
      digitalWrite(DOOR_PIN, HIGH);
      stateChanged = true;
      Serial.println("✓ Door OPENED");
    }
    else if (msg == "door-close" || msg == "door:close" || msg == "door:0") {
      doorState = false;
      digitalWrite(DOOR_PIN, LOW);
      stateChanged = true;
      Serial.println("✓ Door CLOSED");
    }
    else if (msg == "door-toggle" || msg == "door:toggle") {
      doorState = !doorState;
      digitalWrite(DOOR_PIN, doorState ? HIGH : LOW);
      stateChanged = true;
      Serial.print("✓ Door toggled to: ");
      Serial.println(doorState ? "OPEN" : "CLOSED");
    }
    
    else {
      Serial.print("✗ Unknown command: ");
      Serial.println(msg);
    }
    
    if (stateChanged) {
      publishStatus(); // Publish updated status
    }
  }
}

// ===== SETUP =====
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n\n=== ESP32 Sensor Hub Starting ===");

  // Configure pins
  pinMode(FAN_PIN, OUTPUT);
  pinMode(LIGHT_PIN, OUTPUT);
  pinMode(DOOR_PIN, OUTPUT);
  pinMode(LDR_PIN, INPUT);
  pinMode(MQ135_PIN, INPUT);
  
  digitalWrite(FAN_PIN, LOW);
  digitalWrite(LIGHT_PIN, LOW);
  digitalWrite(DOOR_PIN, LOW);
  
  Serial.println("✓ Pins configured");

  // Initialize DHT11
  dht.begin();
  Serial.println("✓ DHT11 initialized");

  // Initialize BMP280 (try both common I2C addresses)
  if (bmp.begin(0x76)) {
    bmpAvailable = true;
    Serial.println("✓ BMP280 found at 0x76");
  } else if (bmp.begin(0x77)) {
    bmpAvailable = true;
    Serial.println("✓ BMP280 found at 0x77");
  } else {
    Serial.println("⚠ BMP280 not found (will continue without it)");
  }

  // Configure BMP280 if available
  if (bmpAvailable) {
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X2,
                    Adafruit_BMP280::SAMPLING_X16,
                    Adafruit_BMP280::FILTER_X16,
                    Adafruit_BMP280::STANDBY_MS_500);
  }

  // WiFi + MQTT setup
  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setBufferSize(512); // Increase buffer for larger JSON

  ensureWiFi();
  mqttReconnect();

  Serial.println("=== ESP32 Sensor Hub Ready ===\n");
}

// ===== LOOP =====
void loop() {
  // Ensure WiFi and MQTT connections
  ensureWiFi();
  mqttReconnect();
  mqttClient.loop();

  // Periodic sensor publishing
  unsigned long now = millis();
  if (now - lastPublish >= PUBLISH_INTERVAL) {
    lastPublish = now;
    publishSensors();
  }
}
