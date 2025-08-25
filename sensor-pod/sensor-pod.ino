/**
 * Wildlife Conservation Monitoring System - The Liana Trust
 * "Actually Works This Time" Edition 🐍
 * 
 * Hardware: PCB Cupid Glyph C3 (ESP32-C3)
 * Sensors: BH1750 Light Sensor, BME280 Environmental Sensor
 * Purpose: Snake habitat monitoring that Lisa will absolutely LOVE
 * 
 * Authors: Amartha (Maximum Sass Division) & Amit (The Snake Whisperer)
 * Version: 2.0.0 - "Complete and Fabulous Edition"
 * Date: When snakes learn to code (approximately now)
 */

// ===== LIBRARY IMPORTS =====
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <BH1750.h>
#include <BME280I2C.h>  // Tyler Green's library - because it actually works
#include "esp_task_wdt.h"
#include "esp_sleep.h"
#include <time.h>
#include <ESPmDNS.h>

// ===== HARDWARE CONFIGURATION =====
// I2C Pin Configuration for PCB Cupid Glyph C3
#define I2C_SDA 4          // GPIO4 - I2C Data Line
#define I2C_SCL 5          // GPIO5 - I2C Clock Line
#define CS_PIN 0           // GPIO0 - Chip Select on D0
#define LED_PIN 2          // Built-in LED for status indication

// Sensor I2C Addresses
#define BH1750_ADDR 0x23   // Light sensor address (ADD pin LOW/floating)
#define BME280_ADDR 0x76   // Environmental sensor default address

// ===== NETWORK CONFIGURATION =====
const char* WIFI_SSID = "@manjusstudio";
const char* WIFI_PASSWORD = "wifi2020!";

// MQTT Configuration with intelligent fallback system
const char* MQTT_BROKER_HOSTNAME = "liana.local";       // Primary target
const char* MQTT_BROKER_IP = "192.168.1.100";          // UPDATE THIS IP!
const char* MQTT_TEST_BROKER = "test.mosquitto.org";   // Public test broker
const int MQTT_PORT = 1883;
const char* MQTT_CLIENT_ID = "sanctuary_glyph_c3_001";
const char* MQTT_USER = "";      // Add if your broker requires auth
const char* MQTT_PASSWORD = "";  // Add if your broker requires auth

// MQTT Topics - Wildlife Conservation Standard
const char* TOPIC_SENSOR_DATA = "liana/habitat/sensors";
const char* TOPIC_ALERTS = "liana/habitat/alerts";
const char* TOPIC_STATUS = "liana/habitat/status";
const char* TOPIC_COMMANDS = "liana/habitat/commands";
const char* TOPIC_DIAGNOSTICS = "liana/habitat/diagnostics";

// ===== TIMING CONFIGURATION (in seconds) =====
const unsigned long SENSOR_READ_INTERVAL = 30;      // Read sensors every 30 seconds
const unsigned long MQTT_PUBLISH_INTERVAL = 60;     // Publish every minute for demo
const unsigned long WIFI_CHECK_INTERVAL = 30;       // Check WiFi every 30 seconds
const unsigned long HEARTBEAT_INTERVAL = 300;       // Heartbeat every 5 minutes
const uint32_t WDT_TIMEOUT = 30;                     // Watchdog timeout

// ===== WILDLIFE MONITORING THRESHOLDS =====
// Temperature thresholds for snake habitats (Celsius)
const float TEMP_CRITICAL_LOW = 15.0;    // Critical low temperature
const float TEMP_CRITICAL_HIGH = 40.0;   // Critical high temperature
const float TEMP_WARNING_LOW = 18.0;     // Warning low temperature
const float TEMP_WARNING_HIGH = 35.0;    // Warning high temperature
const float TEMP_OPTIMAL_MIN = 22.0;     // Optimal range minimum
const float TEMP_OPTIMAL_MAX = 30.0;     // Optimal range maximum

// Humidity thresholds (%)
const float HUMIDITY_CRITICAL_LOW = 20.0;
const float HUMIDITY_CRITICAL_HIGH = 95.0;
const float HUMIDITY_WARNING_LOW = 30.0;
const float HUMIDITY_WARNING_HIGH = 85.0;
const float HUMIDITY_OPTIMAL_MIN = 45.0;
const float HUMIDITY_OPTIMAL_MAX = 75.0;

// Light thresholds (lux) for day/night cycle detection
const float LUX_DAY_THRESHOLD = 100.0;
const float LUX_NIGHT_THRESHOLD = 10.0;
const float LUX_TWILIGHT_THRESHOLD = 50.0;

// ===== GLOBAL OBJECTS =====
BH1750 lightMeter;
BME280I2C bme;    // Tyler Green's BME280 library - the hero we deserved
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

// Connection state tracking
String currentMqttBroker = "";
int currentMqttPort = 0;
bool usingTestBroker = false;

// ===== DATA STRUCTURES =====
struct SensorReading {
  unsigned long timestamp;
  float temperature;
  float humidity;
  float pressure;
  float light_lux;
  int wifi_rssi;
  String light_condition;  // "day", "night", "twilight"
  bool temp_critical;
  bool temp_warning;
  bool humidity_critical;
  bool humidity_warning;
  bool all_optimal;
};

struct AlertLevel {
  String severity;  // "critical", "warning", "info"
  String type;      // "temperature", "humidity", "light", "system"
  String message;
  unsigned long timestamp;
};

// Data buffer for offline storage (because network outages happen)
#define BUFFER_SIZE 100
SensorReading dataBuffer[BUFFER_SIZE];
int bufferIndex = 0;
int bufferCount = 0;

// Alert tracking
AlertLevel lastAlert;
bool alertActive = false;
unsigned long lastAlertTime = 0;

// ===== TIMING VARIABLES =====
unsigned long lastSensorRead = 0;
unsigned long lastMqttPublish = 0;
unsigned long lastWifiCheck = 0;
unsigned long lastHeartbeat = 0;
unsigned long bootTime = 0;
unsigned long totalUptime = 0;
int reconnectAttempts = 0;
int totalMqttPublishes = 0;
int totalAlerts = 0;

// ===== STATUS FLAGS =====
bool sensorsInitialized = false;
bool bmeWorking = false;
bool lightSensorWorking = false;
bool wifiConnected = false;
bool mqttConnected = false;
bool systemHealthy = true;

// ===== FUNCTION DECLARATIONS =====
void initializeSensors();
void connectWiFi();
void connectMQTT();
void reconnectMQTT();
void checkConnections();
SensorReading readSensors();
void publishSensorData(SensorReading& reading);
void publishAlert(AlertLevel alert);
void checkEnvironmentalAlerts(SensorReading& reading);
void storeReading(SensorReading& reading);
void publishBufferedData();
void publishHeartbeat();
void mqttCallback(char* topic, byte* payload, unsigned int length);
void handleCommand(String command, String value);
bool isI2CDevicePresent(uint8_t address);
void printDiagnostics();
void blinkLED(int times, int delayMs = 200);
String getSystemStatus();
String getLightCondition(float lux);
void enterDeepSleep(uint64_t seconds);

// ===== SETUP FUNCTION =====
void setup() {
  // Initialize serial communication for debugging
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n" + String("="));
  Serial.println("WILDLIFE CONSERVATION MONITORING SYSTEM");
  Serial.println("Device: PCB Cupid Glyph C3 (ESP32-C3)");
  Serial.println("Purpose: The Liana Trust Snake Habitat Monitoring");
  Serial.println(String("=") + "\n");
  
  // Initialize hardware pins
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);  // Deselect by default
  pinMode(LED_PIN, OUTPUT);
  
  // Boot indication - because feedback is important
  blinkLED(3, 100);
  
  // Initialize watchdog timer for ESP32-C3 (because things can go wrong)
  esp_task_wdt_config_t wdt_config = {
    .timeout_ms = WDT_TIMEOUT * 1000,
    .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,
    .trigger_panic = true
  };
  esp_task_wdt_init(&wdt_config);
  esp_task_wdt_add(NULL);
  Serial.println("✓ Watchdog timer initialized (30s timeout)");
  
  // Initialize I2C bus with proper error handling
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);  // 100kHz for maximum reliability
  Serial.println("✓ I2C bus initialized on SDA=GPIO4, SCL=GPIO5");
  
  // Comprehensive I2C device scan
  Serial.println("\n🔍 Scanning I2C bus for devices...");
  bool devicesFound = false;
  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) {
      Serial.print("  📍 I2C device found at 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      
      // Identify known devices
      switch(address) {
        case 0x23: Serial.println(" (BH1750 Light Sensor)"); break;
        case 0x76: Serial.println(" (BME280 Environmental Sensor)"); break;
        case 0x77: Serial.println(" (BME280 Alt Address)"); break;
        default: Serial.println(" (Unknown Device)"); break;
      }
      devicesFound = true;
    }
  }
  
  if (!devicesFound) {
    Serial.println("  ⚠️  No I2C devices found! Check your wiring!");
  }
  
  // Initialize sensors with comprehensive error handling
  initializeSensors();
  
  // Connect to WiFi network
  connectWiFi();
  
  // Configure MQTT client
  mqttClient.setCallback(mqttCallback);
  mqttClient.setBufferSize(1024);  // Larger buffer for complex payloads
  mqttClient.setKeepAlive(60);     // 60 second keep-alive
  
  // Connect to MQTT broker with fallback logic
  connectMQTT();
  
  // Record boot time for uptime calculations
  bootTime = millis();
  
  // Take initial sensor reading and publish system status
  if (lightSensorWorking || bmeWorking) {
    SensorReading initial = readSensors();
    
    Serial.println("\n📊 Initial sensor readings:");
    if (bmeWorking) {
      Serial.printf("  🌡️  Temperature: %.2f°C\n", initial.temperature);
      Serial.printf("  💧 Humidity: %.2f%%\n", initial.humidity);
      Serial.printf("  🏔️  Pressure: %.2f hPa\n", initial.pressure);
    }
    if (lightSensorWorking) {
      Serial.printf("  ☀️  Light: %.2f lux (%s)\n", initial.light_lux, initial.light_condition.c_str());
    }
    
    // Store initial reading
    storeReading(initial);
    
    // Publish initial status if MQTT is connected
    if (mqttConnected) {
      publishSensorData(initial);
      publishHeartbeat();
    }
  }
  
  Serial.println("\n✅ System initialization complete!");
  Serial.println("🚀 Starting main monitoring loop...\n");
  
  // Success indication
  blinkLED(5, 100);
}

// ===== MAIN MONITORING LOOP =====
void loop() {
  // Reset watchdog timer to prevent system reset
  esp_task_wdt_reset();
  
  // Handle MQTT communication
  if (mqttConnected) {
    mqttClient.loop();
  }
  
  // Periodic connection health checks
  if (millis() - lastWifiCheck >= WIFI_CHECK_INTERVAL * 1000) {
    checkConnections();
    lastWifiCheck = millis();
  }
  
  // Read sensors at specified interval
  if (millis() - lastSensorRead >= SENSOR_READ_INTERVAL * 1000) {
    if (lightSensorWorking || bmeWorking) {
      SensorReading reading = readSensors();
      
      // Check for environmental alerts
      checkEnvironmentalAlerts(reading);
      
      // Store reading in circular buffer
      storeReading(reading);
      
      // Print diagnostics to serial
      printDiagnostics();
      
      lastSensorRead = millis();
    }
  }
  
  // Publish MQTT data at specified interval
  if (millis() - lastMqttPublish >= MQTT_PUBLISH_INTERVAL * 1000) {
    if (mqttConnected && bufferCount > 0) {
      // Publish latest reading
      int latestIndex = (bufferIndex - 1 + BUFFER_SIZE) % BUFFER_SIZE;
      publishSensorData(dataBuffer[latestIndex]);
      
      // Publish buffered data if we have accumulated readings
      if (bufferCount > 1) {
        publishBufferedData();
      }
      
      totalMqttPublishes++;
      lastMqttPublish = millis();
    }
  }
  
  // Send periodic heartbeat
  if (millis() - lastHeartbeat >= HEARTBEAT_INTERVAL * 1000) {
    if (mqttConnected) {
      publishHeartbeat();
    }
    lastHeartbeat = millis();
  }
  
  // Small delay to prevent CPU hogging
  delay(50);
}

// ===== COMPREHENSIVE SENSOR INITIALIZATION =====
void initializeSensors() {
  Serial.println("\n🔧 Initializing sensors...");
  
  // Initialize BH1750 light sensor
  Serial.print("  📈 BH1750 Light Sensor: ");
  if (isI2CDevicePresent(BH1750_ADDR)) {
    if (lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE)) {
      lightSensorWorking = true;
      Serial.println("✅ WORKING");
      
      // Test read to verify functionality
      float testLux = lightMeter.readLightLevel();
      if (testLux >= 0) {
        Serial.printf("    Initial reading: %.2f lux\n", testLux);
      }
    } else {
      Serial.println("❌ INITIALIZATION FAILED");
    }
  } else {
    Serial.println("❌ NOT DETECTED (Check address 0x23)");
  }
  
  // Initialize BME280 environmental sensor using Tyler Green's library
  Serial.print("  🌡️  BME280 Environmental Sensor: ");
  
  // Try multiple I2C addresses for BME280
  uint8_t bmeAddresses[] = {0x76, 0x77};
  bool bmeInitialized = false;
  
  for (int i = 0; i < 2 && !bmeInitialized; i++) {
    if (isI2CDevicePresent(bmeAddresses[i])) {
      Serial.print("(0x");
      Serial.print(bmeAddresses[i], HEX);
      Serial.print(") ");

      bme.begin();
      
      // Initialize with Tyler Green's library
      // Test sensor by reading values
      float temp(NAN), hum(NAN), pres(NAN);
      bme.read(pres, temp, hum, BME280::TempUnit_Celsius, BME280::PresUnit_hPa);
      
      // Validate readings are reasonable
      if (!isnan(temp) && temp > -50 && temp < 100 && 
          !isnan(hum) && hum >= 0 && hum <= 100 &&
          !isnan(pres) && pres > 800 && pres < 1200) {
        
        bmeWorking = true;
        bmeInitialized = true;
        Serial.println("✅ WORKING");
        Serial.printf("    Initial readings: %.1f°C, %.1f%%, %.1f hPa\n", 
                      temp, hum, pres);
      }
    }
  }
  
  if (!bmeWorking) {
    Serial.println("❌ FAILED");
    Serial.println("    Check: VCC→3.3V, GND→GND, SDA→GPIO4, SCL→GPIO5");
    Serial.println("    Verify: 4.7kΩ pull-up resistors on SDA/SCL lines");
  }
  
  // Set overall sensor status
  sensorsInitialized = (lightSensorWorking || bmeWorking);
  
  if (sensorsInitialized) {
    Serial.println("✅ Sensor initialization completed successfully!");
  } else {
    Serial.println("❌ Sensor initialization failed!");
    Serial.println("⚠️  System will continue but functionality will be limited");
  }
}

// ===== ROBUST WiFi CONNECTION =====
void connectWiFi() {
  Serial.print("\n📶 Connecting to WiFi network: ");
  Serial.println(WIFI_SSID);
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    attempts++;
    esp_task_wdt_reset();  // Reset watchdog during connection
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    Serial.println("\n✅ WiFi connected successfully!");
    Serial.printf("  📍 IP Address: %s\n", WiFi.localIP().toString().c_str());
    Serial.printf("  📶 Signal strength: %d dBm (%s)\n", 
                  WiFi.RSSI(), 
                  WiFi.RSSI() > -50 ? "Excellent" : 
                  WiFi.RSSI() > -60 ? "Good" : 
                  WiFi.RSSI() > -70 ? "Fair" : "Poor");
    
    // Configure NTP for accurate timestamps
    configTime(0, 0, "pool.ntp.org", "time.nist.gov");
    Serial.println("  🕐 NTP time synchronization configured");
    
    // Initialize mDNS for .local domain resolution
    if (MDNS.begin("sanctuary-monitor")) {
      Serial.println("  🌐 mDNS responder started");
    }
    
  } else {
    wifiConnected = false;
    Serial.println("\n❌ WiFi connection failed!");
    Serial.println("    Check: SSID, password, and network availability");
  }
}

// ===== INTELLIGENT MQTT CONNECTION WITH FALLBACK =====
void connectMQTT() {
  if (!wifiConnected) {
    Serial.println("❌ Cannot connect to MQTT - WiFi not connected");
    return;
  }
  
  Serial.println("\n📡 Attempting MQTT broker connections...");
  
  // Define multiple connection targets for maximum reliability
  struct MQTTServer {
    const char* name;
    const char* address;
    int port;
    bool isTest;
    const char* description;
  };
  
  MQTTServer servers[] = {
    {"Liana Local", MQTT_BROKER_HOSTNAME, MQTT_PORT, false, "Primary conservation facility broker"},
    {"Fallback IP", MQTT_BROKER_IP, MQTT_PORT, false, "Direct IP connection"},
    {"Test Broker", MQTT_TEST_BROKER, MQTT_PORT, true, "Public test broker (development only)"}
  };
  
  // Try each server in sequence
  for (int serverIndex = 0; serverIndex < 3; serverIndex++) {
    MQTTServer& server = servers[serverIndex];
    
    Serial.printf("\n📡 Trying %s (%s:%d)\n", server.name, server.address, server.port);
    Serial.printf("    %s\n", server.description);
    
    if (server.isTest) {
      Serial.println("    ⚠️  WARNING: This is a PUBLIC test broker!");
    }
    
    mqttClient.setServer(server.address, server.port);
    
    int attempts = 0;
    while (!mqttClient.connected() && attempts < 3) {
      String clientId = String(MQTT_CLIENT_ID) + "_" + String(random(0xffff), HEX);
      
      Serial.printf("    Attempt %d/3: ", attempts + 1);
      
      if (mqttClient.connect(clientId.c_str(), MQTT_USER, MQTT_PASSWORD)) {
        mqttConnected = true;
        currentMqttBroker = server.address;
        currentMqttPort = server.port;
        usingTestBroker = server.isTest;
        
        Serial.println("✅ SUCCESS!");
        
        if (server.isTest) {
          Serial.println("    🚨 REMOVE TEST BROKER IN PRODUCTION! 🚨");
        }
        
        // Subscribe to command topic for remote management
        if (mqttClient.subscribe(TOPIC_COMMANDS)) {
          Serial.printf("    📬 Subscribed to: %s\n", TOPIC_COMMANDS);
        }
        
        // Publish initial connection status
        StaticJsonDocument<400> doc;
        doc["device_id"] = MQTT_CLIENT_ID;
        doc["status"] = "online";
        doc["ip"] = WiFi.localIP().toString();
        doc["rssi"] = WiFi.RSSI();
        doc["broker"] = server.name;
        doc["broker_address"] = server.address;
        doc["test_mode"] = server.isTest;
        doc["sensors_available"] = sensorsInitialized;
        doc["bme280_working"] = bmeWorking;
        doc["bh1750_working"] = lightSensorWorking;
        doc["firmware_version"] = "2.0.0";
        doc["boot_time"] = bootTime;
        
        char buffer[400];
        serializeJson(doc, buffer);
        mqttClient.publish(TOPIC_STATUS, buffer, true);  // Retained message
        
        return; // Success! Exit function
        
      } else {
        Serial.printf("❌ FAILED (rc=%d) ", mqttClient.state());
        
        // Decode MQTT error codes for better diagnostics
        switch(mqttClient.state()) {
          case -4: Serial.println("[Connection timeout]"); break;
          case -3: Serial.println("[Connection lost]"); break;
          case -2: Serial.println("[Connect failed - broker unreachable]"); break;
          case -1: Serial.println("[Disconnected]"); break;
          case 1: Serial.println("[Bad protocol version]"); break;
          case 2: Serial.println("[Bad client ID]"); break;
          case 3: Serial.println("[Broker unavailable]"); break;
          case 4: Serial.println("[Bad credentials]"); break;
          case 5: Serial.println("[Unauthorized]"); break;
          default: Serial.println("[Unknown error]"); break;
        }
        
        attempts++;
        if (attempts < 3) {
          delay(2000);
        }
      }
      esp_task_wdt_reset();
    }
  }
  
  // If we reach here, all connection attempts failed
  mqttConnected = false;
  Serial.println("\n❌ All MQTT connection attempts failed!");
  Serial.println("\n🔧 Troubleshooting checklist:");
  Serial.println("   1. Verify MQTT broker is running on liana.local");
  Serial.println("   2. Test: ping liana.local");
  Serial.println("   3. Update MQTT_BROKER_IP constant with correct IP");
  Serial.println("   4. Check firewall settings (port 1883)");
  Serial.println("   5. Verify broker accepts anonymous connections");
  Serial.println("   6. Check network connectivity");
}

// ===== CONNECTION HEALTH MONITORING =====
void checkConnections() {
  // Check WiFi connection health
  if (WiFi.status() != WL_CONNECTED) {
    if (wifiConnected) {
      Serial.println("⚠️  WiFi connection lost - attempting reconnection");
      wifiConnected = false;
    }
    
    WiFi.reconnect();
    delay(3000);
    
    if (WiFi.status() == WL_CONNECTED) {
      wifiConnected = true;
      Serial.println("✅ WiFi reconnected successfully");
    }
  } else {
    wifiConnected = true;
  }
  
  // Check MQTT connection health
  if (wifiConnected && !mqttClient.connected()) {
    if (mqttConnected) {
      Serial.println("⚠️  MQTT connection lost - attempting reconnection");
      mqttConnected = false;
    }
    reconnectMQTT();
  }
}

// ===== MQTT RECONNECTION LOGIC =====
void reconnectMQTT() {
  if (!wifiConnected) return;
  
  static unsigned long lastReconnectAttempt = 0;
  unsigned long now = millis();
  
  // Attempt reconnection every 10 seconds
  if (now - lastReconnectAttempt > 10000) {
    lastReconnectAttempt = now;
    
    String clientId = String(MQTT_CLIENT_ID) + "_" + String(random(0xffff), HEX);
    
    if (mqttClient.connect(clientId.c_str(), MQTT_USER, MQTT_PASSWORD)) {
      mqttConnected = true;
      Serial.println("✅ MQTT reconnected successfully");
      
      // Resubscribe to command topic
      mqttClient.subscribe(TOPIC_COMMANDS);
      
      // Publish reconnection status
      publishHeartbeat();
      
      // Publish any buffered data
      if (bufferCount > 0) {
        Serial.println("📤 Publishing buffered data after reconnection");
        publishBufferedData();
      }
      
      reconnectAttempts = 0;
      
    } else {
      reconnectAttempts++;
      Serial.printf("❌ MQTT reconnection failed (attempt %d)\n", reconnectAttempts);
      
      // If too many failures, try a complete WiFi restart
      if (reconnectAttempts > 5) {
        Serial.println("🔄 Too many MQTT failures - restarting WiFi connection");
        WiFi.disconnect();
        delay(2000);
        connectWiFi();
        if (wifiConnected) {
          connectMQTT();
        }
        reconnectAttempts = 0;
      }
    }
  }
}

// ===== COMPREHENSIVE SENSOR READING =====
SensorReading readSensors() {
  SensorReading reading;
  reading.timestamp = millis() / 1000;  // Unix timestamp in seconds
  
  // Read BME280 environmental data if sensor is working
  if (bmeWorking) {
    float temp(NAN), hum(NAN), pres(NAN);
    
    // Use Tyler Green's library read method
    bme.read(pres, temp, hum, BME280::TempUnit_Celsius, BME280::PresUnit_hPa);
    
    // Validate and store readings
    if (!isnan(temp) && !isnan(hum) && !isnan(pres)) {
      reading.temperature = round(temp * 100) / 100.0;  // Round to 2 decimal places
      reading.humidity = round(hum * 100) / 100.0;
      reading.pressure = round(pres * 100) / 100.0;
    } else {
      reading.temperature = -999;
      reading.humidity = -999;
      reading.pressure = -999;
      Serial.println("⚠️  BME280 read failed - invalid values returned");
    }
  } else {
    // Sensor not working, use sentinel values
    reading.temperature = -999;
    reading.humidity = -999;
    reading.pressure = -999;
  }
  
  // Read BH1750 light data if sensor is working
  if (lightSensorWorking) {
    float lux = lightMeter.readLightLevel();
    if (lux >= 0) {
      reading.light_lux = round(lux * 100) / 100.0;
    } else {
      reading.light_lux = -999;
      Serial.println("⚠️  BH1750 read failed");
    }
  } else {
    reading.light_lux = -999;
  }
  
  // Determine light condition based on lux level
  reading.light_condition = getLightCondition(reading.light_lux);
  
  // Get WiFi signal strength for connectivity monitoring
  reading.wifi_rssi = WiFi.RSSI();
  
  // Evaluate environmental conditions
  reading.temp_critical = (reading.temperature != -999) && 
                         (reading.temperature <= TEMP_CRITICAL_LOW || 
                          reading.temperature >= TEMP_CRITICAL_HIGH);
  
  reading.temp_warning = (reading.temperature != -999) && 
                        !reading.temp_critical &&
                        (reading.temperature <= TEMP_WARNING_LOW || 
                         reading.temperature >= TEMP_WARNING_HIGH);
  
  reading.humidity_critical = (reading.humidity != -999) && 
                             (reading.humidity <= HUMIDITY_CRITICAL_LOW || 
                              reading.humidity >= HUMIDITY_CRITICAL_HIGH);
  
  reading.humidity_warning = (reading.humidity != -999) && 
                            !reading.humidity_critical &&
                            (reading.humidity <= HUMIDITY_WARNING_LOW || 
                             reading.humidity >= HUMIDITY_WARNING_HIGH);
  
  reading.all_optimal = (reading.temperature >= TEMP_OPTIMAL_MIN && 
                        reading.temperature <= TEMP_OPTIMAL_MAX) &&
                       (reading.humidity >= HUMIDITY_OPTIMAL_MIN && 
                        reading.humidity <= HUMIDITY_OPTIMAL_MAX) &&
                       !reading.temp_critical && !reading.humidity_critical;
  
  return reading;
}

// ===== COMPREHENSIVE MQTT DATA PUBLISHING =====
void publishSensorData(SensorReading& reading) {
  if (!mqttConnected) {
    Serial.println("📡 Cannot publish - MQTT not connected");
    return;
  }
  
  // Create comprehensive JSON payload
  StaticJsonDocument<800> doc;
  
  // Device identification and metadata
  doc["device_id"] = MQTT_CLIENT_ID;
  doc["timestamp"] = reading.timestamp;
  doc["uptime_seconds"] = (millis() - bootTime) / 1000;
  doc["firmware_version"] = "2.0.0";
  doc["location"] = "The Liana Trust Research Station";
  
  // Environmental sensor data
  JsonObject env = doc.createNestedObject("environmental");
  if (reading.temperature != -999) {
    env["temperature_celsius"] = reading.temperature;
    env["temperature_fahrenheit"] = round((reading.temperature * 9/5 + 32) * 100) / 100.0;
    env["temp_optimal"] = (reading.temperature >= TEMP_OPTIMAL_MIN && 
                           reading.temperature <= TEMP_OPTIMAL_MAX);
  }
  if (reading.humidity != -999) {
    env["humidity_percent"] = reading.humidity;
    env["humidity_optimal"] = (reading.humidity >= HUMIDITY_OPTIMAL_MIN && 
                               reading.humidity <= HUMIDITY_OPTIMAL_MAX);
  }
  if (reading.pressure != -999) {
    env["pressure_hpa"] = reading.pressure;
    env["pressure_inHg"] = round((reading.pressure * 0.02953) * 100) / 100.0;
  }
  if (reading.light_lux != -999) {
    env["light_lux"] = reading.light_lux;
    env["light_condition"] = reading.light_condition;
  }
  
  // Habitat status assessment
  JsonObject habitat = doc.createNestedObject("habitat_status");
  habitat["overall_optimal"] = reading.all_optimal;
  habitat["temperature_status"] = reading.temp_critical ? "critical" : 
                                 reading.temp_warning ? "warning" : "optimal";
  habitat["humidity_status"] = reading.humidity_critical ? "critical" : 
                              reading.humidity_warning ? "warning" : "optimal";
  habitat["alerts_active"] = alertActive;
  habitat["suitable_for_wildlife"] = reading.all_optimal;
  
  // System health and diagnostics
  JsonObject system = doc.createNestedObject("system");
  system["wifi_rssi"] = reading.wifi_rssi;
  system["wifi_quality"] = (reading.wifi_rssi > -50) ? "excellent" : 
                           (reading.wifi_rssi > -60) ? "good" : 
                           (reading.wifi_rssi > -70) ? "fair" : "poor";
  system["mqtt_broker"] = currentMqttBroker;
  system["test_mode"] = usingTestBroker;
  system["sensors_working"] = sensorsInitialized;
  system["bme280_status"] = bmeWorking ? "working" : "failed";
  system["bh1750_status"] = lightSensorWorking ? "working" : "failed";
  system["buffer_count"] = bufferCount;
  system["free_heap"] = ESP.getFreeHeap();
  system["total_publishes"] = totalMqttPublishes;
  
  // Serialize and publish
  char jsonBuffer[800];
  size_t jsonLength = serializeJson(doc, jsonBuffer);
  
  if (mqttClient.publish(TOPIC_SENSOR_DATA, jsonBuffer, false)) {
    Serial.printf("📤 Published sensor data (%d bytes)\n", jsonLength);
    
    // Visual feedback
    digitalWrite(LED_PIN, HIGH);
    delay(50);
    digitalWrite(LED_PIN, LOW);
    
  } else {
    Serial.println("❌ Failed to publish sensor data");
  }
}

// ===== ENVIRONMENTAL ALERT SYSTEM =====
void checkEnvironmentalAlerts(SensorReading& reading) {
  AlertLevel newAlert;
  newAlert.timestamp = reading.timestamp;
  bool shouldAlert = false;
  
  // Check for critical temperature conditions
  if (reading.temp_critical) {
    newAlert.severity = "critical";
    newAlert.type = "temperature";
    if (reading.temperature <= TEMP_CRITICAL_LOW) {
      newAlert.message = "CRITICAL: Temperature dangerously low (" + String(reading.temperature) + "°C)";
    } else {
      newAlert.message = "CRITICAL: Temperature dangerously high (" + String(reading.temperature) + "°C)";
    }
    shouldAlert = true;
  }
  // Check for temperature warnings
  else if (reading.temp_warning) {
    newAlert.severity = "warning";
    newAlert.type = "temperature";
    if (reading.temperature <= TEMP_WARNING_LOW) {
      newAlert.message = "WARNING: Temperature below optimal range (" + String(reading.temperature) + "°C)";
    } else {
      newAlert.message = "WARNING: Temperature above optimal range (" + String(reading.temperature) + "°C)";
    }
    shouldAlert = true;
  }
  
  // Check for critical humidity conditions
  if (reading.humidity_critical) {
    newAlert.severity = "critical";
    newAlert.type = "humidity";
    if (reading.humidity <= HUMIDITY_CRITICAL_LOW) {
      newAlert.message = "CRITICAL: Humidity dangerously low (" + String(reading.humidity) + "%)";
    } else {
      newAlert.message = "CRITICAL: Humidity dangerously high (" + String(reading.humidity) + "%)";
    }
    shouldAlert = true;
  }
  // Check for humidity warnings
  else if (reading.humidity_warning) {
    newAlert.severity = "warning";
    newAlert.type = "humidity";
    if (reading.humidity <= HUMIDITY_WARNING_LOW) {
      newAlert.message = "WARNING: Humidity below optimal range (" + String(reading.humidity) + "%)";
    } else {
      newAlert.message = "WARNING: Humidity above optimal range (" + String(reading.humidity) + "%)";
    }
    shouldAlert = true;
  }
  
  // Publish alert if conditions warrant it
  if (shouldAlert) {
    // Avoid spam - only alert if significantly different from last alert
    if (!alertActive || 
        newAlert.severity != lastAlert.severity || 
        newAlert.type != lastAlert.type ||
        (millis() - lastAlertTime > 300000)) {  // Or 5 minutes have passed
      
      publishAlert(newAlert);
      lastAlert = newAlert;
      lastAlertTime = millis();
      alertActive = true;
      totalAlerts++;
      
      // Console notification
      Serial.printf("🚨 ALERT [%s]: %s\n", newAlert.severity.c_str(), newAlert.message.c_str());
      
      // Visual alert indication
      blinkLED(newAlert.severity == "critical" ? 10 : 5, 100);
    }
  } else if (alertActive && reading.all_optimal) {
    // Conditions have returned to normal
    newAlert.severity = "info";
    newAlert.type = "system";
    newAlert.message = "All environmental conditions have returned to optimal levels";
    publishAlert(newAlert);
    alertActive = false;
    Serial.println("✅ Environmental conditions normalized - alert cleared");
  }
}

// ===== ALERT PUBLISHING =====
void publishAlert(AlertLevel alert) {
  if (!mqttConnected) return;
  
  StaticJsonDocument<400> doc;
  doc["device_id"] = MQTT_CLIENT_ID;
  doc["timestamp"] = alert.timestamp;
  doc["severity"] = alert.severity;
  doc["type"] = alert.type;
  doc["message"] = alert.message;
  doc["location"] = "The Liana Trust Research Station";
  doc["requires_action"] = (alert.severity == "critical");
  
  char buffer[400];
  serializeJson(doc, buffer);
  
  if (mqttClient.publish(TOPIC_ALERTS, buffer, false)) {
    Serial.printf("🚨 Alert published: %s\n", alert.message.c_str());
  }
}

// ===== DATA BUFFERING SYSTEM =====
void storeReading(SensorReading& reading) {
  dataBuffer[bufferIndex] = reading;
  bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;
  
  if (bufferCount < BUFFER_SIZE) {
    bufferCount++;
  }
}

void publishBufferedData() {
  if (!mqttConnected || bufferCount <= 1) return;
  
  Serial.printf("📤 Publishing %d buffered readings\n", bufferCount - 1);
  
  // Publish all buffered readings except the latest (already published)
  for (int i = 0; i < bufferCount - 1; i++) {
    int index = (bufferIndex - bufferCount + i + BUFFER_SIZE) % BUFFER_SIZE;
    publishSensorData(dataBuffer[index]);
    delay(100);  // Small delay between publishes
    esp_task_wdt_reset();
  }
  
  // Reset buffer count to 1 (keeping only latest reading)
  bufferCount = 1;
}

// ===== HEARTBEAT AND STATUS MONITORING =====
void publishHeartbeat() {
  if (!mqttConnected) return;
  
  StaticJsonDocument<600> doc;
  doc["device_id"] = MQTT_CLIENT_ID;
  doc["timestamp"] = millis() / 1000;
  doc["status"] = "alive";
  doc["uptime_seconds"] = (millis() - bootTime) / 1000;
  doc["uptime_minutes"] = ((millis() - bootTime) / 1000) / 60;
  doc["free_heap"] = ESP.getFreeHeap();
  doc["wifi_rssi"] = WiFi.RSSI();
  doc["mqtt_broker"] = currentMqttBroker;
  doc["sensors_healthy"] = sensorsInitialized;
  doc["total_publishes"] = totalMqttPublishes;
  doc["total_alerts"] = totalAlerts;
  doc["buffer_usage"] = bufferCount;
  doc["system_healthy"] = systemHealthy;
  
  JsonObject sensors = doc.createNestedObject("sensor_status");
  sensors["bme280"] = bmeWorking;
  sensors["bh1750"] = lightSensorWorking;
  
  char buffer[600];
  serializeJson(doc, buffer);
  
  mqttClient.publish(TOPIC_STATUS, buffer, true);  // Retained message
  Serial.println("💓 Heartbeat published");
}

// ===== MQTT COMMAND HANDLER =====
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  // Convert payload to string
  String message = "";
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  
  Serial.printf("📬 MQTT command received: %s\n", message.c_str());
  
  // Parse JSON command
  StaticJsonDocument<200> doc;
  DeserializationError error = deserializeJson(doc, message);
  
  if (error) {
    Serial.printf("❌ Invalid JSON command: %s\n", error.c_str());
    return;
  }
  
  String command = doc["command"];
  String value = doc["value"];
  
  handleCommand(command, value);
}

// ===== COMMAND PROCESSING =====
void handleCommand(String command, String value) {
  if (command == "restart") {
    Serial.println("🔄 Restart command received - rebooting in 3 seconds");
    delay(3000);
    ESP.restart();
  }
  else if (command == "sleep") {
    int sleepSeconds = value.toInt();
    if (sleepSeconds > 0 && sleepSeconds <= 3600) {  // Max 1 hour sleep
      Serial.printf("😴 Sleep command received - sleeping for %d seconds\n", sleepSeconds);
      enterDeepSleep(sleepSeconds);
    }
  }
  else if (command == "status") {
    Serial.println("📊 Status command received - publishing comprehensive status");
    publishHeartbeat();
    if (bufferCount > 0) {
      int latestIndex = (bufferIndex - 1 + BUFFER_SIZE) % BUFFER_SIZE;
      publishSensorData(dataBuffer[latestIndex]);
    }
  }
  else if (command == "read_sensors") {
    Serial.println("📊 Manual sensor read requested");
    if (lightSensorWorking || bmeWorking) {
      SensorReading reading = readSensors();
      storeReading(reading);
      if (mqttConnected) {
        publishSensorData(reading);
      }
    }
  }
  else {
    Serial.printf("❓ Unknown command: %s\n", command.c_str());
  }
}

// ===== UTILITY FUNCTIONS =====

bool isI2CDevicePresent(uint8_t address) {
  Wire.beginTransmission(address);
  return (Wire.endTransmission() == 0);
}

String getLightCondition(float lux) {
  if (lux == -999) return "unknown";
  if (lux >= LUX_DAY_THRESHOLD) return "day";
  if (lux <= LUX_NIGHT_THRESHOLD) return "night";
  return "twilight";
}

void blinkLED(int times, int delayMs) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(delayMs);
    digitalWrite(LED_PIN, LOW);
    delay(delayMs);
  }
}

void printDiagnostics() {
  Serial.println("\n" + String("-"));
  Serial.println("📊 SYSTEM DIAGNOSTICS");
  
  // Uptime
  unsigned long uptime = (millis() - bootTime) / 1000;
  Serial.printf("⏱️  Uptime: %lu seconds (%lu minutes)\n", uptime, uptime / 60);
  
  // Connectivity
  Serial.printf("📶 WiFi: %s (RSSI: %d dBm)\n", 
                wifiConnected ? "Connected" : "Disconnected", WiFi.RSSI());
  Serial.printf("📡 MQTT: %s (%s)\n", 
                mqttConnected ? "Connected" : "Disconnected", currentMqttBroker.c_str());
  
  // Sensor Status
  Serial.printf("🌡️  BME280: %s\n", bmeWorking ? "Working" : "Failed");
  Serial.printf("☀️  BH1750: %s\n", lightSensorWorking ? "Working" : "Failed");
  
  // Latest readings
  if (bufferCount > 0) {
    int latestIndex = (bufferIndex - 1 + BUFFER_SIZE) % BUFFER_SIZE;
    SensorReading& latest = dataBuffer[latestIndex];
    
    Serial.println("\n📈 Latest Readings:");
    if (latest.temperature != -999) {
      Serial.printf("   Temperature: %.2f°C (Status: %s)\n", 
                    latest.temperature,
                    latest.temp_critical ? "CRITICAL" : 
                    latest.temp_warning ? "WARNING" : "OPTIMAL");
    }
    if (latest.humidity != -999) {
      Serial.printf("   Humidity: %.2f%% (Status: %s)\n", 
                    latest.humidity,
                    latest.humidity_critical ? "CRITICAL" : 
                    latest.humidity_warning ? "WARNING" : "OPTIMAL");
    }
    if (latest.pressure != -999) {
      Serial.printf("   Pressure: %.2f hPa\n", latest.pressure);
    }
    if (latest.light_lux != -999) {
      Serial.printf("   Light: %.2f lux (%s)\n", latest.light_lux, latest.light_condition.c_str());
    }
  }
  
  // System resources
  Serial.printf("\n💾 Free heap: %d bytes\n", ESP.getFreeHeap());
  Serial.printf("📊 Buffer usage: %d/%d readings\n", bufferCount, BUFFER_SIZE);
  Serial.printf("📤 Total publishes: %d\n", totalMqttPublishes);
  Serial.printf("🚨 Total alerts: %d\n", totalAlerts);
}

void enterDeepSleep(uint64_t seconds) {
  Serial.printf("😴 Entering deep sleep for %llu seconds\n", seconds);
  
  // Publish offline status
  if (mqttConnected) {
    StaticJsonDocument<200> doc;
    doc["device_id"] = MQTT_CLIENT_ID;
    doc["status"] = "sleeping";
    doc["sleep_duration"] = seconds;
    doc["wake_time"] = (millis() / 1000) + seconds;
    
    char buffer[200];
    serializeJson(doc, buffer);
    mqttClient.publish(TOPIC_STATUS, buffer, true);
    delay(100);
  }
  
  // Configure wake-up timer
  esp_sleep_enable_timer_wakeup(seconds * 1000000);  // Convert to microseconds
  
  // Enter deep sleep
  esp_deep_sleep_start();
}

// ===== PLACEHOLDER IMPLEMENTATIONS =====
// These functions are declared but can be expanded based on specific needs

String getSystemStatus() {
  return systemHealthy ? "healthy" : "degraded";
}
