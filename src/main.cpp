#include <Arduino.h>
#include <ArduinoOTA.h>
#include <ESP8266mDNS.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <PubSubClient.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <WiFiManager.h>
#include <algorithm>
#include <ESP8266HTTPClient.h>  // for ntfy notifications
#include <WiFiClientSecureBearSSL.h>

// Forward declaration for printBoth
void printBoth(const String& msg);

// Forward declaration for publishDiscoveryConfig
void publishDiscoveryConfig();

// Forward declaration of publishMQTTData with correct data type
void publishMQTTData(float gasValue);

WiFiServer telnetServer(23);
WiFiClient telnetClient;
WiFiClient espClient;
PubSubClient mqttClient(espClient);
ESP8266WebServer server(80);

const char ALERT_MESSAGE[] PROGMEM = "Gas leak detected! Please take immediate action.";
const char NORMAL_MESSAGE[] PROGMEM = "Gas sensor reading is back to normal.";

struct Config {
  char mqttServer[40];
  char mqttUser[40];
  char mqttPassword[40];
  char deviceName[40];
  int mqttPort;
  bool mqttEnabled;
  int thresholdLimit = 200;       // ppm
  int thresholdDuration = 10;    // seconds
  char topicName[16];       // ntfy topic (6 alphanumeric chars)
  bool ntfyEnabled;         // Enable/disable ntfy notifications
  int baseGasValue = -1;    // Base gas value for calibration, -1 means not set
  int restartCounter = 0;   // Counter for quick restarts
};

struct MQTTConfig {
    char mqtt_server[40] = "";
    int mqtt_port = 1883;
    char mqtt_user[32] = "";
    char mqtt_password[32] = "";
    bool isEmpty() const { return mqtt_server[0] == '\0' || mqtt_port == 0; }
};
MQTTConfig mqttConfig;

Config config;

void setDefaultMQTTConfig() {
    memset(&mqttConfig, 0, sizeof(MQTTConfig));
    mqttConfig.mqtt_port = 1883;
}

void loadMQTTConfig() {
    if (!LittleFS.begin()) {
        printBoth(F("Failed to mount file system"));
        setDefaultMQTTConfig();
        return;
    }
    if (!LittleFS.exists("/mqtt_config.json")) {
        printBoth(F("No MQTT config file found"));
        setDefaultMQTTConfig();
        return;
    }
    File configFile = LittleFS.open("/mqtt_config.json", "r");
    if (!configFile) {
        printBoth(F("Failed to open MQTT config file"));
        setDefaultMQTTConfig();
        return;
    }
    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, configFile);
    configFile.close();
    if (error) {
        printBoth(F("Failed to parse MQTT config file"));
        setDefaultMQTTConfig();
        return;
    }
    if (doc.containsKey(F("server")) && doc.containsKey(F("port"))) {
        strncpy(mqttConfig.mqtt_server, doc[F("server")], sizeof(mqttConfig.mqtt_server) - 1);
        mqttConfig.mqtt_port = doc[F("port")].as<int>();
        if (doc.containsKey(F("user"))) {
            strncpy(mqttConfig.mqtt_user, doc[F("user")], sizeof(mqttConfig.mqtt_user) - 1);
        }
        if (doc.containsKey(F("password"))) {
            strncpy(mqttConfig.mqtt_password, doc[F("password")], sizeof(mqttConfig.mqtt_password) - 1);
        }
    } else {
        setDefaultMQTTConfig();
    }
}

void saveMQTTConfig() {
    if (!LittleFS.begin()) {
        printBoth(F("Failed to mount file system"));
        return;
    }
    StaticJsonDocument<200> doc;
    doc[F("server")] = mqttConfig.mqtt_server;
    doc[F("port")] = mqttConfig.mqtt_port;
    doc[F("user")] = mqttConfig.mqtt_user;
    doc[F("password")] = mqttConfig.mqtt_password;
    File configFile = LittleFS.open("/mqtt_config.json", "w");
    if (!configFile) {
        printBoth(F("Failed to open MQTT config file for writing"));
        return;
    }
    if (serializeJson(doc, configFile) == 0) {
        printBoth(F("Failed to write MQTT config file"));
    }
    configFile.close();
}

// Threshold breach tracking
unsigned long breachStart = 0;
unsigned long underThresholdStart = 0;
unsigned long lastNotificationTime = 0;
unsigned long warmupTime = 60000; // Time to wait before first reading

const int gasSensorPin = A0; // Analog pin connected to MQ9 gas sensor
const int buzzerPin = D8; // Digital pin connected to buzzer

// LED pin definitions
const int redPin = D5;
const int greenPin = D6;
const int bluePin = D7;

// LED status tracking
enum LedState {
  LED_STARTUP,     // Yellow at start
  LED_WIFI_ONLY,   // Green blinking when WiFi connected
  LED_MQTT_ACTIVE, // Blue blinking when MQTT connected
  LED_ALERT,       // Red flashing during alerts
  LED_WIFI_DISCONNECTED  // Red constant when WiFi disconnected
};

LedState currentLedState = LED_STARTUP;
LedState priorLedState = LED_STARTUP;  // Store state before alert
bool ledOn = false;
unsigned long lastLedToggle = 0;
const unsigned long greenBlinkInterval = 5000;  // 5 seconds total cycle
const unsigned long blueBlinkInterval = 5000;   // 5 seconds total cycle
const unsigned long redBlinkInterval = 1000;    // 1 second
const unsigned long onDuration = 100;           // 500ms ON time for blue/green
unsigned long ledBlinkInterval = greenBlinkInterval;
unsigned long currentLedInterval = greenBlinkInterval;

// Variables for non-blocking buzzer operation
unsigned long buzzerStartTime = 0;
unsigned long buzzerDuration = 0;
bool buzzerActive = false;
bool alertState = false;  // Track if we're in alert state
unsigned long lastBuzzerToggle = 0;

// Variables for WiFi disconnection tracking
unsigned long lastWifiBeepTime = 0;

unsigned long lastReconnectAttempt = 0;
unsigned long lastReadingTime = 0;
unsigned long systemStartTime = 0; // Track system start time

#define BUFFER_SIZE 15
float gasDataBuffer[BUFFER_SIZE] = {0}; // Initialize all elements to 0
unsigned long lastPublishTime = 0; // Timestamp of the last publish
const unsigned long publishInterval = 1000; // 15 seconds in milliseconds

// Calibration variables
bool calibrationRunning = false;
unsigned long calibrationStartTime = 0;
unsigned long lastCalibrationLedToggle = 0;
int calibrationLedState = 0; // 0=R, 1=G, 2=B
const unsigned long calibrationDuration = 300000; // 5 minutes in milliseconds
const int numCalibrationReadings = 300; // 300 readings (one per second for 5 minutes)
float calibrationReadings[300];
int calibrationReadingCount = 0;

// AP mode timeout and WiFi retry variables
unsigned long apModeStartTime = 0;
const unsigned long AP_MODE_TIMEOUT = 5 * 60 * 1000; // 5 minutes in milliseconds
const unsigned long WIFI_RETRY_INTERVAL = 5 * 60 * 1000; // 5 minutes in milliseconds
unsigned long lastWifiRetryTime = 0;
bool apModeTimedOut = false;

// Discovery config publish variables
unsigned long lastDiscoveryPublish = 0;
const unsigned long discoveryPublishInterval = 5 * 60 * 1000; // 5 minutes

// Add function prototype at the top of the file, before it's used:
void setLedColor(bool r, bool g, bool b);

// Add these helper functions near the top of the file
void printBoth(const String& msg) {
    Serial.print(msg);
    if (telnetClient && telnetClient.connected()) {
        telnetClient.print(msg);
    }
}
void printlnBoth(const String& msg) {
    Serial.println(msg);
    if (telnetClient && telnetClient.connected()) {
        telnetClient.println(msg);
    }
}
void printfBoth(const char* fmt, ...) {
    char buf[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    Serial.print(buf);
    if (telnetClient && telnetClient.connected()) {
        telnetClient.print(buf);
    }
}

void sendNotification(bool isAlert) {
    if (!(WiFi.status() == WL_CONNECTED) || !config.ntfyEnabled) {
        printlnBoth(F("WiFi not connected or ntfy notifications disabled, skipping notification"));
        return;
    }
   
    // Use the same approach as sendStartupNotification
    WiFiClient client; // Use regular WiFiClient instead of secure client for HTTP
    HTTPClient http;
    String url = String(F("http://ntfy.sh/")) + config.topicName;
    
    if (!http.begin(client, url)) {
        printlnBoth(F("Failed to begin HTTP client"));
        return;
    }
    http.addHeader(F("Title"), F("Gas Detector Alert"));
    http.addHeader(F("Content-Type"), F("text/plain"));
    
    const char* message = isAlert ? ALERT_MESSAGE : NORMAL_MESSAGE;
    int httpResponseCode = http.POST(message);
    
    if (httpResponseCode > 0) {
        printfBoth(PSTR("Notification sent successfully, HTTP code: %d\n"), httpResponseCode);
    } else {
        printfBoth(PSTR("Notification Failed, HTTP error: %s\n"), http.errorToString(httpResponseCode).c_str());
    }
    
    http.end();
}

void sendStartupNotification() {
    if (!(WiFi.status() == WL_CONNECTED) || !config.ntfyEnabled) {
        printlnBoth(F("WiFi not connected or ntfy notifications disabled, skipping startup notification"));
        return;
    }
    // Prepare message
    String ip = WiFi.localIP().toString();
    String hostname = String(config.deviceName);
    hostname.replace(F(" "), F("-"));
    hostname.toLowerCase();
    String mdnsUrl = hostname + F(".local");
    float ppm = analogRead(gasSensorPin) - config.baseGasValue;
    if (ppm < 0) ppm = 0; // Ensure no negative values
    String msg = F("Device started!\nIP: ") + ip + F("\nMDNS: http://") + mdnsUrl + F("/\nCurrent PPM: ") + String(ppm, 1);

    // Send to ntfy using consistent approach
    WiFiClient client;
    HTTPClient http;
    String url = String(F("http://ntfy.sh/")) + config.topicName;
    if (http.begin(client, url)) {
        http.addHeader(F("Title"), F("Gas Detector Online"));
        http.addHeader(F("Content-Type"), F("text/plain"));
        int code = http.POST(msg);
        if (code > 0) {
            printfBoth(PSTR("Startup notification sent, HTTP code: %d\n"), code);
        } else {
            printfBoth(PSTR("Startup notification failed, HTTP error: %s\n"), http.errorToString(code).c_str());
        }
        http.end();
    } else {
        printlnBoth(F("Failed to begin HTTP client for startup notification"));
    }
}

void addGasReading(float gasReading) {
  //print reading to telnet
  if (telnetClient && telnetClient.connected()) {
    telnetClient.printf(PSTR("Gas Sensor Value: %.2f\n"), gasReading);
    printfBoth(PSTR("Gas Sensor Value: %.2f\n"), gasReading);
  }

  // Shift elements to the left
  for (int i = 1; i < BUFFER_SIZE; i++) {
    gasDataBuffer[i - 1] = gasDataBuffer[i];
  }
  // Add new reading to the end
  gasDataBuffer[BUFFER_SIZE - 1] = gasReading;
}

float calculateMedian(float data[], int size) {
  float temp[size];
  memcpy(temp, data, size * sizeof(float)); // Copy data to avoid modifying the original array
  std::sort(temp, temp + size);
  //print sorted values to telnet
  if (telnetClient && telnetClient.connected()) {
    telnetClient.print(F("Sorted values: "));
    for (int i = 0; i < size; i++) {
      telnetClient.printf(PSTR("[%.2f]"), temp[i]);
    }
    telnetClient.println();
  }
  int mid = size / 2;
  return (size % 2 == 0) ? (temp[mid - 1] + temp[mid]) / 2.0 : temp[mid];
}

void saveConfig() {
  File configFile = LittleFS.open("/config.json", "w");
  if (!configFile) {
    printlnBoth(F("Failed to open config file for writing"));
    return;
  }

  JsonDocument json;
  json[F("mqttServer")] = config.mqttServer;
  json[F("mqttUser")] = config.mqttUser;
  json[F("mqttPassword")] = config.mqttPassword;
  json[F("deviceName")] = config.deviceName;
  json[F("mqttPort")] = config.mqttPort;
  json[F("mqttEnabled")] = config.mqttEnabled;
  json[F("thresholdLimit")] = config.thresholdLimit;
  json[F("thresholdDuration")] = config.thresholdDuration;
  json[F("topicName")] = config.topicName;
  json[F("ntfyEnabled")] = config.ntfyEnabled;  // Save ntfy status
  json[F("baseGasValue")] = config.baseGasValue;  // Save base gas value
  json[F("restartCounter")] = config.restartCounter;  // Save restart counter

  if (serializeJson(json, configFile) == 0) {
    printlnBoth(F("Failed to write to config file"));
  }
  else {
    printlnBoth(F("Configuration saved successfully"));
  }

  configFile.close();
}

void loadConfig() {
  File configFile = LittleFS.open("/config.json", "r");
  if (!configFile) {
    printlnBoth(F("Failed to open config file"));
    return;
  }

  JsonDocument json;
  DeserializationError error = deserializeJson(json, configFile);
  if (error) {
    printlnBoth(F("Failed to parse config file"));
    return;
  }

  strlcpy(config.mqttServer, json[F("mqttServer")] | "", sizeof(config.mqttServer));
  strlcpy(config.mqttUser, json[F("mqttUser")] | "", sizeof(config.mqttUser));
  strlcpy(config.mqttPassword, json[F("mqttPassword")] | "", sizeof(config.mqttPassword));
  strlcpy(config.deviceName, json[F("deviceName")] | "", sizeof(config.deviceName));
  config.mqttPort = json[F("mqttPort")] | 1883;
  config.mqttEnabled = json[F("mqttEnabled")] | false;
  config.thresholdLimit = json[F("thresholdLimit")] | 200;
  config.thresholdDuration = json[F("thresholdDuration")] | 5;

  // Always set topicName to GasDetect_Macaddress (no colons)
  String mac = WiFi.macAddress();
  mac.replace(":", ""); // Remove colons from MAC address
  String ntfyChannel = F("GasDetect_") + mac.substring(mac.length() - 6);
  strlcpy(config.topicName, ntfyChannel.c_str(), sizeof(config.topicName));

  config.ntfyEnabled = json[F("ntfyEnabled")] | true;  // Default to enabled for backwards compatibility
  config.baseGasValue = json[F("baseGasValue")] | -1; // Default to -1 if not set
  config.restartCounter = json[F("restartCounter")] | 0; // Default to 0 if not set

  configFile.close();
  //print all config values on serial
  printlnBoth(F("Loaded configuration:"));

  printfBoth(PSTR("MQTT Server: %s\n"), config.mqttServer);
  printfBoth(PSTR("MQTT User: %s\n"), config.mqttUser);
  printfBoth(PSTR("MQTT Password: %s\n"), config.mqttPassword);
  printfBoth(PSTR("Device Name: %s\n"), config.deviceName);
  printfBoth(PSTR("MQTT Port: %d\n"), config.mqttPort);
  printfBoth(PSTR("MQTT Enabled: %s\n"), config.mqttEnabled ? F("true") : F("false"));
  printfBoth(PSTR("Threshold Limit: %d\n"), config.thresholdLimit);
  printfBoth(PSTR("Threshold Duration: %d\n"), config.thresholdDuration);
  printfBoth(PSTR("Topic Name: %s\n"), config.topicName);
  printfBoth(PSTR("NTFY Enabled: %s\n"), config.ntfyEnabled ? F("true") : F("false"));
  printfBoth(PSTR("Base Gas Value: %d\n"), config.baseGasValue);
  printfBoth(PSTR("Restart Counter: %d\n"), config.restartCounter);
}

// Function to handle calibration LED pattern
void updateCalibrationLed() {
  unsigned long currentTime = millis();
  
  // Toggle LED every 800ms (300ms on, 500ms off)
  if (currentTime - lastCalibrationLedToggle >= 800) {
    lastCalibrationLedToggle = currentTime;
    
    // Cycle through Red, Green, Blue
    if (calibrationLedState == 0) {
      // Red
      setLedColor(true, false, false);
      calibrationLedState = 1;
    } else if (calibrationLedState == 1) {
      // Green
      setLedColor(false, true, false);
      calibrationLedState = 2;
    } else {
      // Blue
      setLedColor(false, false, true);
      calibrationLedState = 0;
    }
  } else if (currentTime - lastCalibrationLedToggle >= 300) {
    // Turn LED off after 300ms
    setLedColor(false, false, false);
  }
}

// Calculate average of calibration readings
float calculateCalibrationAverage() {
  if (calibrationReadingCount == 0) return -1;
  
  float sum = 0;
  for (int i = 0; i < calibrationReadingCount; i++) {
    sum += calibrationReadings[i];
  }
  return sum / calibrationReadingCount;
}



// Add a handler function for device reset
void handleReset() {
  server.send(200, F("text/html"), F("<html><body><h1>Resetting Device</h1><p>The device will now reset and all configurations will be wiped.</p></body></html>"));
  delay(1000); // Give time for the response to be sent

  printlnBoth(F("Performing factory reset..."));

  // Clear stored configurations - with error checking
  if (LittleFS.exists("/config.json")) {
    if (LittleFS.remove("/config.json")) {
      printlnBoth(F("Config file deleted successfully"));
    } else {
      printlnBoth(F("Failed to delete config file"));
    }
  } else {
    printlnBoth(F("Config file not found"));
  }
  
  // Clear WiFi settings by removing the wifi config file
  if (LittleFS.exists("/wifi_cred.dat")) {
    if (LittleFS.remove("/wifi_cred.dat")) {
      printlnBoth(F("WiFi credentials file deleted successfully"));
    } else {
      printlnBoth(F("Failed to delete WiFi credentials file"));
    }
  } else {
    printlnBoth(F("WiFi credentials file not found"));
  }
  
  // Ensure the filesystem has time to complete operations
  LittleFS.end();
  delay(500);
  
  // Explicitly clear WiFi settings in memory
  WiFi.disconnect(true);  // disconnect and delete credentials
  
  // Wait for WiFi disconnect to complete
  printlnBoth(F("Disconnecting WiFi..."));
  delay(1000);
  
  // Erase config and reset
  printlnBoth(F("Erasing configuration and restarting..."));
  ESP.eraseConfig();
  delay(1000);
  ESP.restart();
}

void handleResetWiFi() {
  server.send(200, F("text/html"), F("<html><body><h1>Resetting WiFi Settings</h1><p>The device will now reset WiFi settings and reboot.</p></body></html>"));
  delay(1000); // Give time for the response to be sent
  WiFi.disconnect(true); // Disconnect from Wi-Fi
  ESP.eraseConfig(); // Erase all Wi-Fi and network-related settings
  printlnBoth(F("Resetting WiFi settings..."));

  // Clear WiFi settings by removing the WiFi credentials file
  if (LittleFS.exists("/wifi_cred.dat")) {
    if (LittleFS.remove("/wifi_cred.dat")) {
      printlnBoth(F("WiFi credentials file deleted successfully"));
    } else {
      printlnBoth(F("Failed to delete WiFi credentials file"));
    }
  } else {
    printlnBoth(F("WiFi credentials file not found"));
  }

 
  // Wait for WiFi disconnect to complete
  delay(1000);

  // Restart the device
  ESP.restart();
}

void handleResetCalibration() {
  server.send(200, F("text/html"), F("<html><body><h1>Resetting Calibration</h1><p>The device will now reset and calibration will be triggered.</p></body></html>"));
  delay(1000); // Give time for the response to be sent

  // Reset baseGasValue to -1 to trigger calibration
  config.baseGasValue = -1;
  saveConfig();

  // Restart the device
  ESP.restart();
}

void handleRestart() {
  server.send(200, F("text/html"), F("<html><body><h1>Restarting Device</h1><p>The device will now restart.</p></body></html>"));
  delay(1000); // Give time for the response to be sent

  // Restart the device
  ESP.restart();
}

void setupMQTT() {
    loadMQTTConfig();
    if (mqttConfig.isEmpty()) {
        printBoth(F("No MQTT configuration found - MQTT disabled"));
        return;
    }
    // Use device name or fallback to MAC for client ID and topic
    String hostname = String(config.deviceName);
    if (hostname.length() == 0) {
        hostname = WiFi.macAddress();
        hostname.replace(":", "");
    }
    hostname.toLowerCase();
    mqttClient.setServer(mqttConfig.mqtt_server, mqttConfig.mqtt_port);
    // Set callback if you want to handle incoming messages
    // mqttClient.setCallback(mqttCallback);
    printfBoth(PSTR("Attempting to connect to MQTT broker as %s..."), hostname.c_str());
    if (mqttClient.connect(hostname.c_str(), mqttConfig.mqtt_user, mqttConfig.mqtt_password)) {
        printBoth(F("MQTT Connected Successfully"));
        publishDiscoveryConfig(); // Use the clean discovery function only
        // Subscribe to command topic for future remote control
        mqttClient.subscribe((F("homeassistant/") + hostname + F("/command")).c_str());
    } else {
        int state = mqttClient.state();
        String errorMsg = F("Initial MQTT connection failed, state: ");
        switch (state) {
            case -4: errorMsg += F("MQTT_CONNECTION_TIMEOUT"); break;
            case -3: errorMsg += F("MQTT_CONNECTION_LOST"); break;
            case -2: errorMsg += F("MQTT_CONNECT_FAILED"); break;
            case -1: errorMsg += F("MQTT_DISCONNECTED"); break;
            case 1: errorMsg += F("MQTT_CONNECT_BAD_PROTOCOL"); break;
            case 2: errorMsg += F("MQTT_CONNECT_BAD_CLIENT_ID"); break;
            case 3: errorMsg += F("MQTT_CONNECT_UNAVAILABLE"); break;
            case 4: errorMsg += F("MQTT_CONNECT_BAD_CREDENTIALS"); break;
            case 5: errorMsg += F("MQTT_CONNECT_UNAUTHORIZED"); break;
            default: errorMsg += String(state);
        }
        printBoth(errorMsg);
        printBoth(F("Will retry in main loop"));
    }
}

void reconnectMQTT() {
    loadMQTTConfig();
    if (mqttConfig.isEmpty()) return;
    String hostname = String(config.deviceName);
    if (hostname.length() == 0) {
        hostname = WiFi.macAddress();
        hostname.replace(":", "");
    }
    hostname.toLowerCase();
    if (mqttClient.connected()) return;
    printfBoth(PSTR("Attempting MQTT connection as %s..."), hostname.c_str());
    if (mqttClient.connect(hostname.c_str(), mqttConfig.mqtt_user, mqttConfig.mqtt_password)) {
        printBoth(F("Connected to MQTT broker"));
        publishDiscoveryConfig(); // Use the clean discovery function only
        // Subscribe to command topic
        mqttClient.subscribe((F("homeassistant/") + hostname + F("/command")).c_str());
    } else {
        int state = mqttClient.state();
        String errorMsg = F("Connection failed, state: ");
        switch (state) {
            case -4: errorMsg += F("MQTT_CONNECTION_TIMEOUT"); break;
            case -3: errorMsg += F("MQTT_CONNECTION_LOST"); break;
            case -2: errorMsg += F("MQTT_CONNECT_FAILED"); break;
            case -1: errorMsg += F("MQTT_DISCONNECTED"); break;
            case 1: errorMsg += F("MQTT_CONNECT_BAD_PROTOCOL"); break;
            case 2: errorMsg += F("MQTT_CONNECT_BAD_CLIENT_ID"); break;
            case 3: errorMsg += F("MQTT_CONNECT_UNAVAILABLE"); break;
            case 4: errorMsg += F("MQTT_CONNECT_BAD_CREDENTIALS"); break;
            case 5: errorMsg += F("MQTT_CONNECT_UNAUTHORIZED"); break;
            default: errorMsg += String(state);
        }
        printBoth(errorMsg);
        printBoth(F("Will try again later"));
    }
}

void publishMQTTData(float gasValue) {
    if (mqttConfig.isEmpty()) return;
    String hostname = String(config.deviceName);
    if (hostname.length() == 0) {
        hostname = WiFi.macAddress();
        hostname.replace(":", "");
    }
    hostname.toLowerCase();
    if (!mqttClient.connected()) {
        printBoth(F("MQTT disconnected, attempting to reconnect..."));
        if (mqttClient.connect(hostname.c_str(), mqttConfig.mqtt_user, mqttConfig.mqtt_password)) {
            printBoth(F("connected"));
        } else {
            printBoth(F("failed"));
            return;
        }
    }
    mqttClient.loop();
    String topic = F("homeassistant/sensor/") + hostname + F("/gas/state");
    String gasValueStr = String(gasValue, 1); // Format to 1 decimal place
    bool published = mqttClient.publish(topic.c_str(), gasValueStr.c_str(), true);
    printfBoth(PSTR("MQTT publish %s: topic=%s, value=%s\n"), published ? F("SUCCESS") : F("FAILED"), topic.c_str(), gasValueStr.c_str());
}

// Publishes Home Assistant discovery config for the gas sensor
void publishDiscoveryConfig() {
    String hostname = String(config.deviceName);
    hostname.toLowerCase();
    hostname.replace(F("."), F("_"));
    for (size_t i = 0; i < hostname.length(); i++) {
        if (!isalnum(hostname[i]) && hostname[i] != '_') hostname[i] = '_';
    }
    String configTopic = F("homeassistant/sensor/") + hostname + F("/gas/config");
    String stateTopic = F("homeassistant/sensor/") + hostname + F("/gas/state");
    // Do NOT use device_class: gas if using ppm as unit
    String configPayload = F("{");
    configPayload += F("\"name\":\"") + hostname + F(" Gas Sensor\",");
    configPayload += F("\"state_topic\":\"") + stateTopic + F("\",");
    configPayload += F("\"unit_of_measurement\":\"ppm\",");
    configPayload += F("\"unique_id\":\"") + hostname + F("_gas\"}");
    bool pubSuccess = mqttClient.publish(configTopic.c_str(), configPayload.c_str(), true);
    printfBoth(PSTR("MQTT: Discovery config publish %s\n"), pubSuccess ? F("successful") : F("failed"));
    printfBoth(PSTR("Config topic: %s\n"), configTopic.c_str());
    printfBoth(PSTR("Config payload: %s\n"), configPayload.c_str());
}

void handleRoot() {
  String html = F("<html><head><meta name='viewport' content='width=device-width, initial-scale=1.0'>");
  html += F("<style>");
  html += F("body { font-family: Arial, sans-serif; margin: 0; padding: 0; background: #f4f4f9; color: #333; }"
            "h1, h2 { text-align: center; color: #444; }"
            "form, .info-section, .danger-zone { max-width: 90%; margin: 1em auto; padding: 1em; background: #fff; border-radius: 5px; box-shadow: 0 2px 5px rgba(0,0,0,0.1); }"
            "input[type='text'], input[type='password'], input[type='number'] { width: 100%; padding: 0.7em; margin: 0.5em 0; border: 1px solid #ccc; border-radius: 3px; box-sizing: border-box; }"
            "input[type='submit'], button { width: 100%; background: #007BFF; color: white; border: none; padding: 0.7em; border-radius: 3px; cursor: pointer; font-size: 1em; }"
            "input[type='submit']:hover, button:hover { background: #0056b3; }"
            ".danger-button { background: #dc3545; }"
            ".danger-button:hover { background: #c82333; }"
            ".info-section p, .danger-zone p { margin: 0.5em 0; }"
            ".info-section strong, .danger-zone strong { display: inline-block; width: 50%; }"
            ".mqtt-settings { display: none; }"
  );
  html += F("</style>");

  html += F("<script>");
  html += F("function toggleMqttSettings() {"
            "  var mqttDiv = document.getElementById('mqttSettings');"
            "  var enabled = document.getElementById('mqttEnabled').checked;"
            "  mqttDiv.style.display = enabled ? 'block' : 'none';"
            "}"
            "function confirmReset() {"
            "  return confirm('WARNING: This will erase all settings including WiFi credentials and reboot the device. Continue?');"
            "}"
            "</script></head><body>");

  html += F("<h1>Device Configuration</h1>");
  html += F("<form action='/save' method='POST'>");
  html += F("<label for='deviceName'>Device Name:</label>");
  html += "<input type='text' id='deviceName' name='deviceName' value='" + String(config.deviceName) + F("'><br>");

  html += F("<label for='mqttEnabled'>Enable MQTT:</label>");
  html += "<input type='checkbox' id='mqttEnabled' name='mqttEnabled' value='1' onchange='toggleMqttSettings()'";
  if (config.mqttEnabled) {
    html += F(" checked");
  }
  html += F("><br>");

  html += "<div id='mqttSettings' class='mqtt-settings' style='display: " + String(config.mqttEnabled ? F("block") : F("none")) + F(";'>");
  html += F("<label for='mqtt_server'>MQTT Server:</label>");
  html += "<input type='text' id='mqtt_server' name='mqtt_server' value='" + String(config.mqttServer) + F("'><br>");
  html += F("<label for='mqtt_user'>MQTT User:</label>");
  html += "<input type='text' id='mqtt_user' name='mqtt_user' value='" + String(config.mqttUser) + F("'><br>");
  html += F("<label for='mqtt_password'>MQTT Password:</label>");
  html += "<input type='password' id='mqtt_password' name='mqtt_password' value='" + String(config.mqttPassword) + F("'><br>");
  html += F("<label for='mqtt_port'>MQTT Port:</label>");
  html += "<input type='number' id='mqtt_port' name='mqtt_port' value='" + String(config.mqttPort) + F("'><br>");
  html += F("</div>");

  html += F("<label for='thresholdLimit'>Gas Threshold (ppm):</label>");
  html += "<input type='number' id='thresholdLimit' name='thresholdLimit' value='" + String(config.thresholdLimit) + F("'><br>");
  html += F("<label for='thresholdDuration'>Duration (s):</label>");
  html += "<input type='number' id='thresholdDuration' name='thresholdDuration' value='" + String(config.thresholdDuration) + F("'><br>");

  html += F("<label for='ntfyEnabled'>Enable NTFY Notifications:</label>");
  html += "<input type='checkbox' id='ntfyEnabled' name='ntfyEnabled' value='1'";
  if (config.ntfyEnabled) {
    html += F(" checked");
  }
  html += F("><br>");

  html += F("<label for='topicName'>Notification Topic:</label>");
  html += "<input type='text' id='topicName' name='topicName' value='" + String(config.topicName) + F("' readonly><br>");

  html += F("<input type='submit' value='Save'>");
  html += F("</form>");

  html += F("<div class='info-section'>");
  html += F("<h2>Device Information</h2>");
  html += F("<p><strong>IP Address:</strong><span>") + WiFi.localIP().toString() + F("</span></p>");
  html += F("<p><strong>MAC Address:</strong><span>") + WiFi.macAddress() + F("</span></p>");
  
  // Add memory information
  uint32_t freeHeap = ESP.getFreeHeap();
  uint32_t maxFreeBlock = ESP.getMaxFreeBlockSize();
  uint32_t freeSketchSpace = (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;
  uint32_t flashChipSize = ESP.getFlashChipSize();
  
  html += F("<p><strong>Free RAM:</strong><span>") + String(freeHeap) + F(" bytes</span></p>");
  html += F("<p><strong>Largest Free Block:</strong><span>") + String(maxFreeBlock) + F(" bytes</span></p>");
  html += F("<p><strong>Free Sketch Space:</strong><span>") + String(freeSketchSpace) + F(" bytes (") + String((freeSketchSpace * 100) / flashChipSize) + F("%)</span></p>");
  html += F("<p><strong>Flash Chip Size:</strong><span>") + String(flashChipSize) + F(" bytes</span></p>");
  html += F("</div>");

  html += F("<div class='info-section'>");
  html += F("<h2>Sensor Values</h2>");
  html += F("<p><strong>Raw Value:</strong><span>") + String(analogRead(gasSensorPin)) + F("</span></p>");
  html += F("<p><strong>Adjustment Value:</strong><span>") + String(config.baseGasValue) + F("</span></p>");
  html += F("<p><strong>Adjusted Value:</strong><span>") + String(analogRead(gasSensorPin) - config.baseGasValue) + F("</span></p>");
  html += F("</div>");

  html += F("<div class='info-section'>");
  html += F("<h2>Firmware Update</h2>");
  html += F("<p>Download and install latest firmware from GitHub.</p>");
  html += F("<a href='/update'><button style='background-color: #28a745;'>Update Firmware</button></a>");
  html += F("</div>");

  html += F("<div class='danger-zone'>");
  html += F("<h2>Danger Zone</h2>");
  html += F("<p>Erase all configurations including WiFi settings.</p>");
  html += F("<a href='/reset' onclick='return confirmReset()'><button class='danger-button'>Reset Device</button></a>");
  html += F("</div>");

  html += F("<div class='danger-zone'>");
  html += F("<h2>Calibration Reset</h2>");
  html += F("<p>Erase the base gas value and restart the device to trigger calibration.</p>");
  html += F("<a href='/reset-calibration'><button class='danger-button'>Reset Calibration</button></a>");
  html += F("</div>");

  html += F("<div class='danger-zone'>");
  html += F("<h2>Restart Device</h2>");
  html += F("<p>Restart the device without erasing configurations.</p>");
  html += F("<a href='/restart'><button class='danger-button'>Restart Device</button></a>");
  html += F("</div>");

  html += F("<div class='danger-zone'>");
  html += F("<h2>Reset WiFi Settings</h2>");
  html += F("<p>Resetting WiFi settings will erase the saved WiFi credentials and restart the device to display the captive portal.</p>");
  html += F("<a href='/reset-wifi'><button class='danger-button'>Reset WiFi Settings</button></a>");
  html += F("</div>");

  html += F("</body></html>");
  server.send(200, F("text/html"), html);
}

void handleSave() {
  // Always handle device name regardless of MQTT status
  if (server.hasArg("deviceName")) {
    server.arg("deviceName").toCharArray(config.deviceName, 40);
  }

  bool wasMqttEnabled = config.mqttEnabled;
  if (server.hasArg("mqttEnabled")) {
    config.mqttEnabled = server.arg("mqttEnabled") == "1";
  } else {
    config.mqttEnabled = false;
  }

  if (config.mqttEnabled) {
    if (server.hasArg("mqttServer")) {
      server.arg("mqttServer").toCharArray(config.mqttServer, 40);
    }
    if (server.hasArg("mqttUser")) {
      server.arg("mqttUser").toCharArray(config.mqttUser, 40);
    }
    if (server.hasArg("mqttPassword")) {
      server.arg("mqttPassword").toCharArray(config.mqttPassword, 40);
    }
    if (server.hasArg("mqttPort")) {
      config.mqttPort = server.arg("mqttPort").toInt();
    }
  }

  if (server.hasArg("thresholdLimit")) {
    config.thresholdLimit = server.arg("thresholdLimit").toInt();
  }
  if (server.hasArg("thresholdDuration")) {
    config.thresholdDuration = server.arg("thresholdDuration").toInt();
  }

  // Handle NTFY toggle
  if (server.hasArg("ntfyEnabled")) {
    config.ntfyEnabled = server.arg("ntfyEnabled") == "1";
  } else {
    config.ntfyEnabled = false;
  }

  // Handle MQTT settings (use DeskClock-compatible field names and save to MQTTConfig)
  if (server.hasArg("mqtt_server")) {
    strncpy(mqttConfig.mqtt_server, server.arg("mqtt_server").c_str(), sizeof(mqttConfig.mqtt_server) - 1);
  }
  if (server.hasArg("mqtt_port")) {
    mqttConfig.mqtt_port = server.arg("mqtt_port").toInt();
  }
  if (server.hasArg("mqtt_user")) {
    strncpy(mqttConfig.mqtt_user, server.arg("mqtt_user").c_str(), sizeof(mqttConfig.mqtt_user) - 1);
  }
  if (server.hasArg("mqtt_password")) {
    strncpy(mqttConfig.mqtt_password, server.arg("mqtt_password").c_str(), sizeof(mqttConfig.mqtt_password) - 1);
  }
  saveMQTTConfig();
  loadMQTTConfig();
  if (mqttConfig.isEmpty()) {
    config.mqttEnabled = false;
    printlnBoth(F("MQTT config not found or invalid, MQTT disabled"));
  }

  // Handle MQTT client disconnect if being disabled
  if (wasMqttEnabled && !config.mqttEnabled) {
    mqttClient.disconnect();
  }
  
  saveConfig();
  loadMQTTConfig(); // Reload after saving
  if (mqttConfig.isEmpty()) {
    config.mqttEnabled = false;
    printlnBoth(F("MQTT config not found or invalid, MQTT disabled"));
  }
  
  // If MQTT was enabled, initialize it
  if (!wasMqttEnabled && config.mqttEnabled) {
    setupMQTT();
    reconnectMQTT();
  }
  
  server.send(200, F("text/html"), F("<html><body><h1>Configuration Saved</h1><a href='/'>Go Back</a></body></html>"));
}

void handleUpdate() {
  HTTPUpload& upload = server.upload();
  if (upload.status == UPLOAD_FILE_START) {
    printlnBoth(F("Update: ") + String(upload.filename));
    uint32_t maxSketchSpace = (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;
    if (!Update.begin(maxSketchSpace)) {
      Update.printError(Serial);
    }
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
      Update.printError(Serial);
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    if (Update.end(true)) {
      printlnBoth(F("Update Success: ") + String(upload.totalSize));
      server.send(200, F("text/plain"), F("Update successful! Rebooting..."));
      delay(1000);
      ESP.restart();
    } else {
      Update.printError(Serial);
    }
  }
  yield();
}

void handleUpdatePage() {
  String html = F("<html><head><meta name='viewport' content='width=device-width, initial-scale=1.0'>");
  html += F("<style>");
  html += F("body { font-family: Arial, sans-serif; margin: 0; padding: 20px; background: #f4f4f9; color: #333; }");
  html += F("h1 { text-align: center; color: #444; }");
  html += F(".update-container { max-width: 600px; margin: 0 auto; padding: 20px; background: #fff; border-radius: 5px; box-shadow: 0 2px 5px rgba(0,0,0,0.1); }");
  html += F(".progress { width: 100%; height: 20px; background: #eee; border-radius: 10px; margin: 20px 0; display: none; }");
  html += F(".progress-bar { width: 0%; height: 100%; background: #28a745; border-radius: 10px; transition: width 0.3s; }");
  html += F("button { width: 100%; background: #28a745; color: white; border: none; padding: 10px; border-radius: 5px; cursor: pointer; font-size: 16px; }");
  html += F("button:hover { background: #218838; }");
  html += F(".status { text-align: center; margin: 10px 0; }");
  html += F("</style>");

  html += F("<script>");
  html += F("async function startUpdate() {");
  html += F("  const status = document.getElementById('status');");
  html += F("  const progress = document.getElementById('progress');");
  html += F("  const progressBar = document.getElementById('progressBar');");
  html += F("  try {");
  html += F("    status.textContent = 'Downloading firmware...';");
  html += F("    progress.style.display = 'block';");
  html += F("    const response = await fetch('https://arjunus1985.github.io/GasDetect/fwroot/firmware.bin');");
  html += F("    const firmware = await response.arrayBuffer();");
  html += F("    const formData = new FormData();");
  html += F("    formData.append('firmware', new Blob([firmware]), 'firmware.bin');");
  html += F("    status.textContent = 'Uploading firmware to device...';");
  html += F("    progressBar.style.width = '50%';");
  html += F("    const updateResponse = await fetch('/do-update', {");
  html += F("      method: 'POST',");
  html += F("      body: formData");
  html += F("    });");
  html += F("    if (updateResponse.ok) {");
  html += F("      status.textContent = 'Update successful! Device will restart...';");
  html += F("      progressBar.style.width = '100%';");
  html += F("      setTimeout(() => { window.location.href = '/'; }, 5000);");
  html += F("    } else {");
  html += F("      throw new Error('Update failed');");
  html += F("    }");
  html += F("  } catch (error) {");
  html += F("    status.textContent = 'Error: ' + error.message;");
  html += F("    progressBar.style.background = '#dc3545';");
  html += F("  }");
  html += F("}");
  html += F("</script></head><body>");

  html += F("<div class='update-container'>");
  html += F("<h1>Firmware Update</h1>");
  html += F("<p>This will download and install the latest firmware from GitHub.</p>");
  html += F("<button onclick='startUpdate()'>Start Update</button>");
  html += F("<div id='progress' class='progress'>");
  html += F("<div id='progressBar' class='progress-bar'></div>");
  html += F("</div>");
  html += F("<div id='status' class='status'></div>");
  html += F("<p><a href='/'>&larr; Back to main page</a></p>");
  html += F("</div>");
  html += F("</body></html>");

  server.send(200, F("text/html"), html);
}

// Callback for when device enters config mode
void configModeCallback(WiFiManager *myWiFiManager) {
  printlnBoth(F("Failed to connect to WiFi"));
  printlnBoth(F("Entered config mode"));
  printlnBoth(F("AP IP address: ") + WiFi.softAPIP().toString());
  printlnBoth(F("AP SSID: ") + myWiFiManager->getConfigPortalSSID());
}

void printGasDataBuffer() {
    
        for (int i = 0; i < BUFFER_SIZE; i++) {
          if (telnetClient && telnetClient.connected()) {
            telnetClient.println(F("Gas Data Buffer:"));
            telnetClient.printf(PSTR("[%.2f]"), gasDataBuffer[i]);
          }
            printfBoth(PSTR("[%.2f]"), gasDataBuffer[i]);
        }
        if (telnetClient && telnetClient.connected()) {
        telnetClient.printf(PSTR("\n"));
        }
        printfBoth(PSTR("\n"));
    }

// Function to set RGB LED color
void setLedColor(bool r, bool g, bool b) {
  digitalWrite(redPin, r ? HIGH : LOW);
  digitalWrite(greenPin, g ? HIGH : LOW);
  digitalWrite(bluePin, b ? HIGH : LOW);
}

// Function to update LED status
void updateLedStatus() {
  unsigned long currentTime = millis();
  
  // During alert state
  if (buzzerActive) {
    if (currentLedState != LED_ALERT) {
      priorLedState = currentLedState;
      currentLedState = LED_ALERT;
      ledBlinkInterval = redBlinkInterval;
      currentLedInterval = redBlinkInterval;
      lastLedToggle = currentTime;
      ledOn = true;
      setLedColor(true, false, false); // Red
    }
  } else if (currentLedState == LED_ALERT) {
    // Restore previous state
    currentLedState = priorLedState;
    ledOn = true;
    lastLedToggle = currentTime;
    if (currentLedState == LED_WIFI_ONLY) {
      currentLedInterval = greenBlinkInterval;
      ledBlinkInterval = onDuration;
    } else if (currentLedState == LED_MQTT_ACTIVE) {
      currentLedInterval = blueBlinkInterval;
      ledBlinkInterval = onDuration;
    }
  }

  // Update LED state based on connectivity
  if (!buzzerActive) {
    // Check WiFi connection first
    if (WiFi.status() != WL_CONNECTED) {
      // WiFi disconnected - set constant red
      if (currentLedState != LED_WIFI_DISCONNECTED) {
        currentLedState = LED_WIFI_DISCONNECTED;
        ledOn = true;
        setLedColor(true, false, false); // Solid red
      }
      
      // Beep once every minute during WiFi disconnection
      if (currentTime - lastWifiBeepTime >= 60000) { // 60 seconds
        lastWifiBeepTime = currentTime;
        // Schedule a short beep
        buzzerStartTime = currentTime;
        buzzerDuration = 100; // 100ms beep
        buzzerActive = true;
        digitalWrite(buzzerPin, HIGH);
      }
    } else if (WiFi.status() == WL_CONNECTED && mqttClient.connected() && config.mqttEnabled) {
      if (currentLedState != LED_MQTT_ACTIVE) {
        currentLedState = LED_MQTT_ACTIVE;
        currentLedInterval = blueBlinkInterval;
        ledBlinkInterval = onDuration;
        lastLedToggle = currentTime;
        ledOn = true;
        setLedColor(false, false, true); // Blue
      }
    } else if (WiFi.status() == WL_CONNECTED) {
      if (currentLedState != LED_WIFI_ONLY) {
        currentLedState = LED_WIFI_ONLY;
        currentLedInterval = greenBlinkInterval;
        ledBlinkInterval = onDuration;
        lastLedToggle = currentTime;
        ledOn = true;
        setLedColor(false, true, false); // Green
      }
    }
  }

  // Handle blinking based on current state
  if (currentTime - lastLedToggle >= ledBlinkInterval) {
    lastLedToggle = currentTime;
    
    switch (currentLedState) {
      case LED_STARTUP:
        // For startup, just toggle
        ledOn = !ledOn;
        setLedColor(ledOn, false, false); // red 
        break;
        
      case LED_WIFI_DISCONNECTED:
        // Keep constant red, no toggling needed
        break;
        
      case LED_WIFI_ONLY:
        if (ledOn) {
          // Turn off after 500ms ON time
          ledOn = false;
          setLedColor(false, false, false); // OFF
          ledBlinkInterval = greenBlinkInterval - onDuration; // Remaining time until next cycle
        } else {
          // Turn on for 500ms
          ledOn = true;
          setLedColor(false, true, false); // Green
          ledBlinkInterval = onDuration;
        }
        break;
        
      case LED_MQTT_ACTIVE:
        if (ledOn) {
          // Turn off after 500ms ON time
          ledOn = false;
          setLedColor(false, false, false); // OFF
          ledBlinkInterval = blueBlinkInterval - onDuration; // Remaining time until next cycle
        } else {
          // Turn on for 500ms
          ledOn = true;
          setLedColor(false, false, true); // Blue
          ledBlinkInterval = onDuration;
        }
        break;
        
      case LED_ALERT:
        // For alert state, just toggle every interval
        ledOn = !ledOn;
        setLedColor(ledOn, false, false); // Red blink
        break;
    }
  }
}

void setup() {
  Serial.begin(9600);

  // Initialize LittleFS
  if (!LittleFS.begin()) {
    printlnBoth(F("Failed to mount file system"));
    return;
  }

  // Load configuration from LittleFS
  loadConfig();
  loadMQTTConfig(); // Only load once at startup
  if (mqttConfig.isEmpty()) {
    config.mqttEnabled = false;
    printlnBoth(F("MQTT config not found or invalid, MQTT disabled"));
  }

  // Check if restart counter has reached 3 - if so, calibration reset
  if (config.restartCounter >= 3) {
    printlnBoth(F("Restart counter reached 3 - performing calibration reset"));
    
    
    // Reset counter to 0
    config.baseGasValue = -1;
    saveConfig();
    
    // Wait for WiFi disconnect to complete
    delay(1000);
  
  }
  if (config.restartCounter >= 5) {
    printlnBoth(F("Restart counter reached 5 - performing factory reset"));
    
    // Clear stored configurations
    LittleFS.remove("/config.json");
    
    // Clear WiFi settings by removing the wifi config file
    LittleFS.remove("/wifi_cred.dat");  // WiFiManager stored credentials
    
    // Explicitly clear WiFi settings in memory
    WiFi.disconnect(true);  // disconnect and delete credentials
    
    // Reset counter to 0
    //config.restartCounter = 0;
    //saveConfig();
    
    // Wait for WiFi disconnect to complete
    delay(1000);
    
    // Reboot the device with clean config
    ESP.eraseConfig();
    delay(1000);
    ESP.restart();
    return;
  }
  
  // Increment restart counter and save
  config.restartCounter++;
  printfBoth(PSTR("Restart counter: %d\n"), config.restartCounter);
  saveConfig();
  
  
  // Initialize serial communication for debugging
  
  // Initialize LED pins
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  
  // Set initial LED state to red
  setLedColor(true, false, false);
  
  // Initialize buzzer pin
  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, LOW);  // Ensure buzzer is off initially
//buzz the buzzer for 500 MS
  digitalWrite(buzzerPin, HIGH); // Turn on the buzzer
  delay(100);                   // Wait for 500 milliseconds
  digitalWrite(buzzerPin, LOW);  // Turn off the buzzer
  delay(1000);                   // Wait for 500 milliseconds
  // WiFiManager setup
  WiFiManager wifiManager;
  
  // Set config mode callback
  wifiManager.setAPCallback(configModeCallback);
  
  // Set connection timeout to 30 seconds
  wifiManager.setConnectTimeout(30);
  
  // Set timeout for AP mode portal
  wifiManager.setConfigPortalTimeout(300); // 5 minutes (300 seconds) timeout for AP mode
  
  // Set custom AP name
  String apName = F("GasDetector-") + String(ESP.getChipId());
  
  // Try to connect to WiFi or start AP mode if needed
  printlnBoth(F("Attempting to connect to WiFi..."));
  if (!wifiManager.autoConnect(apName.c_str())) {
    printlnBoth(F("Failed to connect to WiFi and AP mode timed out"));
    printlnBoth(F("Continuing in offline mode, will retry WiFi connection later"));
    // Set flag to indicate we're in offline mode after AP timeout
    apModeTimedOut = true;
    lastWifiRetryTime = millis();
  } else {
    printlnBoth(F("Connected to WiFi"));
  }

  // Ensure we are in station mode for mDNS
  WiFi.mode(WIFI_STA);
  delay(100);
  printfBoth(PSTR("WiFi mode: %d (1=STA,2=AP,3=STA+AP)\n"), WiFi.getMode());

  // Format and sanitize hostname
  String hostname = String(config.deviceName);
  hostname.replace(' ', '-');
  for (size_t i = 0; i < hostname.length(); i++) {
    if (!isalnum(hostname[i]) && hostname[i] != '-') hostname[i] = '-';
  }
  hostname.toLowerCase();
  if (hostname.length() == 0) hostname = F("gas-detector");

  // Set WiFi hostname and start mDNS responder
  WiFi.hostname(hostname.c_str());  // set DHCP hostname
  delay(100);
  printBoth(F("DHCP hostname: "));
  printlnBoth(WiFi.hostname());
  
  if (!MDNS.begin(hostname.c_str())) {
    printlnBoth(F("Error setting up mDNS responder"));
    // Debug info
    printBoth(F("Local IP: ")); printlnBoth(WiFi.localIP().toString());
    printBoth(F("MAC: ")); printlnBoth(WiFi.macAddress());
  } else {
    MDNS.addService(F("http"), F("tcp"), 80);
    MDNS.addService(F("telnet"), F("tcp"), 23);
    printfBoth(PSTR("mDNS responder started: %s.local\n"), hostname.c_str());
    // Debug info
    printBoth(F("mDNS hostname: ")); printlnBoth(hostname + F(".local"));
  }

  // Configure OTA with same hostname
  ArduinoOTA.setHostname(hostname.c_str());

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = F("sketch");
    } else { // U_SPIFFS
      type = F("filesystem");
    }
    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    printlnBoth(F("Start updating ") + type);
  });
  ArduinoOTA.onEnd([]() {
    printlnBoth(F("\nEnd"));
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    printfBoth(PSTR("Progress: %u%%\r"), (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    printfBoth(PSTR("Error[%u]: "), error);
    if (error == OTA_AUTH_ERROR) {
      printlnBoth(F("Auth Failed"));
    } else if (error == OTA_BEGIN_ERROR) {
      printlnBoth(F("Begin Failed"));
    } else if (error == OTA_CONNECT_ERROR) {
      printlnBoth(F("Connect Failed"));
    } else if (error == OTA_RECEIVE_ERROR) {
      printlnBoth(F("Receive Failed"));
    } else if (error == OTA_END_ERROR) {
      printlnBoth(F("End Failed"));
    }
  });
  ArduinoOTA.begin();

  printlnBoth(F("OTA Ready"));
  printBoth(F("IP address: "));
  printlnBoth(WiFi.localIP().toString());

  // Send IP address notification after reboot
  //sendIpAddressNotification();

  // Start Telnet server
  telnetServer.begin();
  telnetServer.setNoDelay(true);
  printlnBoth(F("Telnet server started"));

  


  // Start Web Server
  server.on(F("/"), handleRoot);
  server.on(F("/save"), HTTP_POST, handleSave);
 
  server.on(F("/reset"), HTTP_GET, handleReset); // Add handler for reset
  server.on(F("/reset-calibration"), HTTP_GET, handleResetCalibration); // Add handler for reset calibration
  server.on(F("/restart"), HTTP_GET, handleRestart); // Add handler for restart
  server.on(F("/reset-wifi"), HTTP_GET, handleResetWiFi); // Add handler for resetting only WiFi settings
  server.on(F("/update"), HTTP_GET, handleUpdatePage);  // New route for update page
  server.on(F("/do-update"), HTTP_POST, []() {
    server.sendHeader(F("Connection"), F("close"));
    server.send(200, F("text/plain"), (Update.hasError()) ? F("FAIL") : F("OK"));
    ESP.restart();
  }, handleUpdate);
  server.begin();
  printlnBoth(F("Web server started"));

  // Only setup MQTT if enabled in config
  

  if (WiFi.status() == WL_CONNECTED && config.mqttEnabled)
  {
      setupMQTT();
  }
  else
  {
    printlnBoth(F("WiFi not connected. Skipping MQTT setup."));
  }

  systemStartTime = millis(); // Record the system start time
  config.restartCounter = 0;
  saveConfig();

  // At the end of setup, after WiFi/mDNS and config are loaded
  delay(500); // Small delay to ensure network is ready
  sendStartupNotification();
}

void loop() {
  // Handle OTA updates
  ArduinoOTA.handle();
  unsigned long currentTime = millis();
  
  // Check AP mode timeout and WiFi connection
  if (!apModeTimedOut && WiFi.getMode() == WIFI_AP) {
    if (currentTime - apModeStartTime >= AP_MODE_TIMEOUT) {
      printlnBoth(F("AP mode timeout reached. Switching to offline mode."));
      WiFi.softAPdisconnect(true);
      WiFi.mode(WIFI_STA);
      apModeTimedOut = true;
      lastWifiRetryTime = currentTime;
    }
    // While in active AP mode, don't proceed with normal loop execution
    return;
  }

  // If we're in offline mode (AP timed out), periodically try to reconnect to WiFi
  if (apModeTimedOut && WiFi.status() != WL_CONNECTED) {
    if (currentTime - lastWifiRetryTime >= WIFI_RETRY_INTERVAL) {
      printlnBoth(F("Trying to reconnect to WiFi..."));
      // Use WiFi.begin() without parameters to reconnect using stored credentials
      WiFi.begin();
      
      // Wait for connection for a reasonable time (e.g., 10 seconds)
      unsigned long connectStart = millis();
      while (WiFi.status() != WL_CONNECTED && millis() - connectStart < 10000) {
        delay(500);
        printBoth(F("."));
      }
      
      if (WiFi.status() == WL_CONNECTED) {
        printlnBoth(F("\nConnected to WiFi!"));
      } else {
        printlnBoth(F("\nFailed to connect to WiFi, continuing in offline mode"));
      }
      
      lastWifiRetryTime = currentTime;
    }
  }
  
  // Accept new Telnet client using accept() instead of available
  if (telnetServer.hasClient()) {
    if (!telnetClient || !telnetClient.connected()) {
      if (telnetClient) telnetClient.stop();
      telnetClient = telnetServer.accept();
      printlnBoth(F("New Telnet client connected"));
    } else {
      telnetServer.accept().stop(); // Reject new client if already connected
    }
  }

  // Non-blocking buzzer control for ongoing beeping during alert
  if (alertState) {
    // Toggle buzzer every 1 second
    if (currentTime - lastBuzzerToggle >= 1000) {
      lastBuzzerToggle = currentTime;
      buzzerActive = !buzzerActive;
      digitalWrite(buzzerPin, buzzerActive ? HIGH : LOW);
    }
  } else if (buzzerActive) {
    // Turn off buzzer when duration elapsed
    if (currentTime - buzzerStartTime >= buzzerDuration) {
      digitalWrite(buzzerPin, LOW);
      buzzerActive = false;
    }
  }

  // Only perform MQTT operations if enabled
  if (config.mqttEnabled) {
    if (!mqttClient.connected()) {
      reconnectMQTT();
    }
    mqttClient.loop(); // Call loop frequently to maintain connection
  }

  // Publish discovery config every 5 minutes
  if (config.mqttEnabled && mqttClient.connected() && millis() - lastDiscoveryPublish > discoveryPublishInterval) {
    publishDiscoveryConfig();
    lastDiscoveryPublish = millis();
  }

  unsigned long now = millis();

  // Check if we need to start calibration after warmup
  if ((currentTime - systemStartTime > warmupTime) && config.baseGasValue == -1 && !calibrationRunning) {
    // Start calibration process
    calibrationRunning = true;
    calibrationStartTime = currentTime;
    calibrationReadingCount = 0;
    printlnBoth(F("Starting calibration process for 5 minutes..."));
    if (telnetClient && telnetClient.connected()) {
      telnetClient.println(F("Starting calibration process for 5 minutes..."));
    }
  }

  // Handle calibration process
  if (calibrationRunning) {
    // Update calibration LED pattern
    updateCalibrationLed();
    
    // Take readings every second for 5 minutes
    if ((currentTime - calibrationStartTime <= calibrationDuration) && 
        (currentTime - lastReadingTime >= 1000)) {
      lastReadingTime = currentTime;
      
      // Read gas sensor value
      float rawGasReading = analogRead(gasSensorPin);
      
      // Calculate median from buffer
      float medianValue = calculateMedian(gasDataBuffer, BUFFER_SIZE);
      
      // Store median value for calibration if buffer has data
      if (calibrationReadingCount < numCalibrationReadings && medianValue > 0) {
        calibrationReadings[calibrationReadingCount++] = medianValue;
        
        if (telnetClient && telnetClient.connected()) {
          telnetClient.printf(PSTR("Calibration reading %d: %.2f\n"), calibrationReadingCount, medianValue);
        }
        printfBoth(PSTR("Calibration reading %d: %.2f\n"), calibrationReadingCount, medianValue);
      }
      
      // Add gas sensor value to buffer
      addGasReading(rawGasReading);
      
    } else if (currentTime - calibrationStartTime > calibrationDuration) {
      // Calibration complete - calculate average and save
      config.baseGasValue = (int)calculateCalibrationAverage();
      saveConfig();
      calibrationRunning = false;
      //reset buffer to all zeroes
      for (int i = 0; i < BUFFER_SIZE; i++) {
        gasDataBuffer[i] = 0;
      }
      printfBoth(PSTR("Calibration complete. Base gas value: %d\n"), config.baseGasValue);
      if (telnetClient && telnetClient.connected()) {
        telnetClient.printf(PSTR("Calibration complete. Base gas value: %d\n"), config.baseGasValue);
      }
    }
  } else if((currentTime - systemStartTime > warmupTime)) {
    // Normal operation after warmup
    // Update LED status (non-blocking)
    updateLedStatus();
    
    // Read and publish sensor data every second without blocking
    if (now - lastReadingTime >= 1000) {
      lastReadingTime = now;
      
      // Read gas sensor value
      float rawGasReading = analogRead(gasSensorPin);
      
      // Apply baseline offset if calibrated
      float gasReading = rawGasReading;
      if (config.baseGasValue > 0) {
        gasReading = rawGasReading - config.baseGasValue;
        if (gasReading < 0) gasReading = 0; // Ensure no negative values
      }
      
      //print on telnet
      //if (telnetClient && telnetClient.connected()) {
      //  telnetClient.printf("Gas Sensor Value: %.2f\n", gasReading);
      //}
      printfBoth(PSTR("Gas Sensor Value: %.2f (raw: %.2f, base: %d)\n"), gasReading, rawGasReading, config.baseGasValue);
      if (telnetClient && telnetClient.connected()) {
        telnetClient.printf(PSTR("Gas Sensor Value: %.2f (raw: %.2f, base: %d)\n"), gasReading, rawGasReading, config.baseGasValue);
      }
      // Add gas sensor value to buffer
      addGasReading(gasReading);
      printGasDataBuffer();

      // Check threshold breach
      if (gasReading > config.thresholdLimit ) {
        // reset under-threshold tracking
        underThresholdStart = 0;
        // mark start of breach
        if (breachStart == 0) {
          breachStart = now;
        }
        // if breach persists long enough, send notification every 10s
        if (now - breachStart >= (unsigned long)config.thresholdDuration * 1000) {
          alertState = true;  // Enable alert state with beeping
          if (lastNotificationTime == 0 || now - lastNotificationTime >= 30000) {
            sendNotification(true);
            lastNotificationTime = now;
          }
        }
      } else {
        // reading back under threshold: start under-threshold timer
        if (underThresholdStart == 0) {
          underThresholdStart = now;
        }
        // if level stays below threshold long enough, reset breach state and notify
        if (now - underThresholdStart >= (unsigned long)config.thresholdDuration * 1000) {
          // send alert cleared notification
          if (breachStart != 0) {
            sendNotification(false);
            alertState = false;  // Disable alert state, stop beeping
          }
          breachStart = 0;
          underThresholdStart = 0;
          lastNotificationTime = 0;
        }
      }

      // Publish median value every 1 second
      if (now - lastPublishTime >= publishInterval) {
          float medianValue = calculateMedian(gasDataBuffer, BUFFER_SIZE);
          
          // Debug: Always log when we're about to publish
          printfBoth(PSTR("Publishing MQTT data: %.2f (system uptime: %lu ms)\n"), medianValue, currentTime - systemStartTime);
          if (telnetClient && telnetClient.connected()) {
            telnetClient.printf(PSTR("Publishing MQTT data: %.2f (system uptime: %lu ms)\n"), medianValue, currentTime - systemStartTime);
          }
          
          unsigned long currentTime = millis();

          // Reduce startup delay from 60 seconds to 10 seconds
          if (currentTime - systemStartTime > 10000) {
            publishMQTTData(medianValue); // Publish median value
            lastPublishTime = now;
          } else {
            printfBoth(PSTR("Skipping MQTT publish - system still warming up (%lu seconds remaining)\n"), (10000 - (currentTime - systemStartTime)) / 1000);
          }
      }
    }

    // Print the gas data buffer to Telnet
    
  }
  // Handle web server requests
  server.handleClient();

  // Update mDNS once per second
  static unsigned long _mdnsTimer = 0;
  if (millis() - _mdnsTimer >= 1000) {
    MDNS.update();
    _mdnsTimer = millis();
  }
}