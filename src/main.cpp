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

WiFiServer telnetServer(23);
WiFiClient telnetClient;
WiFiClient espClient;
PubSubClient mqttClient(espClient);
ESP8266WebServer server(80);

WiFiClientSecure *secureClient = nullptr;
HTTPClient *httpClient = nullptr;
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

Config config;

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

// Forward declaration of publishMQTTData
void publishMQTTData(int gasValue);

// Add function prototype at the top of the file, before it's used:
void setLedColor(bool r, bool g, bool b);

void setupNotifications() {
    if (secureClient == nullptr) {
        secureClient = new WiFiClientSecure();
        secureClient->setInsecure();
    }
    if (httpClient == nullptr) {
        httpClient = new HTTPClient();
    }
}

void sendNotification(bool isAlert) {
    //if (!(WiFi.status() == WL_CONNECTED) || !config.ntfyEnabled) {
    //    Serial.println("WiFi not connected or ntfy notifications disabled, skipping notification");
    //    return;
    //}
//
    //if (secureClient == nullptr || httpClient == nullptr) {
    //    setupNotifications();
    //}
//
    //String url = String(F("https://ntfy.sh/")) + config.topicName;
    //
    //if (!httpClient->begin(*secureClient, url)) {
    //    Serial.println(F("Failed to begin HTTP client"));
    //    return;
    //}
//
    //httpClient->addHeader(F("Title"), F("Gas Detector Alert"));
    //httpClient->addHeader(F("Content-Type"), F("text/plain"));
    //
    //const char* message = isAlert ? ALERT_MESSAGE : NORMAL_MESSAGE;
    //int httpResponseCode = httpClient->POST(message);
    //
    //if (httpResponseCode > 0) {
    //    Serial.printf_P(PSTR("Notification sent successfully, HTTP code: %d\n"), httpResponseCode);
    //} else {
    //    Serial.printf_P(PSTR("Notification Failed, HTTP error: %s\n"), httpClient->errorToString(httpResponseCode).c_str());
    //}
    //
    //httpClient->end();
}

void cleanup() {
    if (httpClient) {
        delete httpClient;
        httpClient = nullptr;
    }
    if (secureClient) {
        delete secureClient;
        secureClient = nullptr;
    }
}

void addGasReading(float gasReading) {
  //print reading to telnet
  if (telnetClient && telnetClient.connected()) {
    telnetClient.printf("Gas Sensor Value: %.2f\n", gasReading);
    Serial.printf("Gas Sensor Value: %.2f\n", gasReading);
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
    telnetClient.print("Sorted values: ");
    for (int i = 0; i < size; i++) {
      telnetClient.printf("[%.2f]", temp[i]);
    }
    telnetClient.println();
  }
  int mid = size / 2;
  return (size % 2 == 0) ? (temp[mid - 1] + temp[mid]) / 2.0 : temp[mid];
}

void saveConfig() {
  File configFile = LittleFS.open("/config.json", "w");
  if (!configFile) {
    Serial.println("Failed to open config file for writing");
    return;
  }

  JsonDocument json;
  json["mqttServer"] = config.mqttServer;
  json["mqttUser"] = config.mqttUser;
  json["mqttPassword"] = config.mqttPassword;
  json["deviceName"] = config.deviceName;
  json["mqttPort"] = config.mqttPort;
  json["mqttEnabled"] = config.mqttEnabled;
  json["thresholdLimit"] = config.thresholdLimit;
  json["thresholdDuration"] = config.thresholdDuration;
  json["topicName"] = config.topicName;
  json["ntfyEnabled"] = config.ntfyEnabled;  // Save ntfy status
  json["baseGasValue"] = config.baseGasValue;  // Save base gas value
  json["restartCounter"] = config.restartCounter;  // Save restart counter

  if (serializeJson(json, configFile) == 0) {
    Serial.println("Failed to write to config file");
  }
  else {
    Serial.println("Configuration saved successfully");
  }

  configFile.close();
}

void loadConfig() {
  File configFile = LittleFS.open("/config.json", "r");
  if (!configFile) {
    Serial.println("Failed to open config file");
    return;
  }

  JsonDocument json;
  DeserializationError error = deserializeJson(json, configFile);
  if (error) {
    Serial.println("Failed to parse config file");
    return;
  }

  strlcpy(config.mqttServer, json["mqttServer"] | "", sizeof(config.mqttServer));
  strlcpy(config.mqttUser, json["mqttUser"] | "", sizeof(config.mqttUser));
  strlcpy(config.mqttPassword, json["mqttPassword"] | "", sizeof(config.mqttPassword));
  strlcpy(config.deviceName, json["deviceName"] | "", sizeof(config.deviceName));
  config.mqttPort = json["mqttPort"] | 1883;
  config.mqttEnabled = json["mqttEnabled"] | false;
  config.thresholdLimit = json["thresholdLimit"] | 200;
  config.thresholdDuration = json["thresholdDuration"] | 5;

  // Always set topicName to MAC address without colons
  String mac = WiFi.macAddress();
  mac.replace(":", ""); // Remove colons from MAC address
  strlcpy(config.topicName, mac.c_str(), sizeof(config.topicName));

  config.ntfyEnabled = json["ntfyEnabled"] | true;  // Default to enabled for backwards compatibility
  config.baseGasValue = json["baseGasValue"] | -1; // Default to -1 if not set
  config.restartCounter = json["restartCounter"] | 0; // Default to 0 if not set

  configFile.close();
  //print all config values on serial
  Serial.println("Loaded configuration:");

  Serial.printf("MQTT Server: %s\n", config.mqttServer);
  Serial.printf("MQTT User: %s\n", config.mqttUser);
  Serial.printf("MQTT Password: %s\n", config.mqttPassword);
  Serial.printf("Device Name: %s\n", config.deviceName);
  Serial.printf("MQTT Port: %d\n", config.mqttPort);
  Serial.printf("MQTT Enabled: %s\n", config.mqttEnabled ? "true" : "false");
  Serial.printf("Threshold Limit: %d\n", config.thresholdLimit);
  Serial.printf("Threshold Duration: %d\n", config.thresholdDuration);
  Serial.printf("Topic Name: %s\n", config.topicName);
  Serial.printf("NTFY Enabled: %s\n", config.ntfyEnabled ? "true" : "false");
  Serial.printf("Base Gas Value: %d\n", config.baseGasValue);
  Serial.printf("Restart Counter: %d\n", config.restartCounter);
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
  server.send(200, "text/html", "<html><body><h1>Resetting Device</h1><p>The device will now reset and all configurations will be wiped.</p></body></html>");
  delay(1000); // Give time for the response to be sent

  Serial.println("Performing factory reset...");

  // Clear stored configurations - with error checking
  if (LittleFS.exists("/config.json")) {
    if (LittleFS.remove("/config.json")) {
      Serial.println("Config file deleted successfully");
    } else {
      Serial.println("Failed to delete config file");
    }
  } else {
    Serial.println("Config file not found");
  }
  
  // Clear WiFi settings by removing the wifi config file
  if (LittleFS.exists("/wifi_cred.dat")) {
    if (LittleFS.remove("/wifi_cred.dat")) {
      Serial.println("WiFi credentials file deleted successfully");
    } else {
      Serial.println("Failed to delete WiFi credentials file");
    }
  } else {
    Serial.println("WiFi credentials file not found");
  }
  
  // Ensure the filesystem has time to complete operations
  LittleFS.end();
  delay(500);
  
  // Explicitly clear WiFi settings in memory
  WiFi.disconnect(true);  // disconnect and delete credentials
  
  // Wait for WiFi disconnect to complete
  Serial.println("Disconnecting WiFi...");
  delay(1000);
  
  // Erase config and reset
  Serial.println("Erasing configuration and restarting...");
  ESP.eraseConfig();
  delay(1000);
  ESP.restart();
}

void handleResetWiFi() {
  server.send(200, "text/html", "<html><body><h1>Resetting WiFi Settings</h1><p>The device will now reset WiFi settings and reboot.</p></body></html>");
  delay(1000); // Give time for the response to be sent
  WiFi.disconnect(true); // Disconnect from Wi-Fi
  ESP.eraseConfig(); // Erase all Wi-Fi and network-related settings
  Serial.println("Resetting WiFi settings...");

  // Clear WiFi settings by removing the WiFi credentials file
  if (LittleFS.exists("/wifi_cred.dat")) {
    if (LittleFS.remove("/wifi_cred.dat")) {
      Serial.println("WiFi credentials file deleted successfully");
    } else {
      Serial.println("Failed to delete WiFi credentials file");
    }
  } else {
    Serial.println("WiFi credentials file not found");
  }

 
  // Wait for WiFi disconnect to complete
  delay(1000);

  // Restart the device
  ESP.restart();
}

void handleResetCalibration() {
  server.send(200, "text/html", "<html><body><h1>Resetting Calibration</h1><p>The device will now reset and calibration will be triggered.</p></body></html>");
  delay(1000); // Give time for the response to be sent

  // Reset baseGasValue to -1 to trigger calibration
  config.baseGasValue = -1;
  saveConfig();

  // Restart the device
  ESP.restart();
}

void handleRestart() {
  server.send(200, "text/html", "<html><body><h1>Restarting Device</h1><p>The device will now restart.</p></body></html>");
  delay(1000); // Give time for the response to be sent

  // Restart the device
  ESP.restart();
}

void setupMQTT() {
 // loadMQTTConfig();
  
  if (!config.mqttEnabled) {
      Serial.println("MQTT disabled");
      return;
  }

  mqttClient.setServer(config.mqttServer, config.mqttPort);
  //mqttClient.setCallback(mqttCallback);
  
 // printBothf("Attempting to connect to MQTT broker as %s...", deviceConfig.hostname);
  if (mqttClient.connect(config.mqttServer, config.mqttUser, config.mqttPassword)) {
     // printBoth("MQTT Connected Successfully");
      
    
      // Publish discovery configs for gas sensor
     // Define base topic for this device
     String baseTopic = "homeassistant/sensor/" + String(config.deviceName) + "/gas";
     String stateTopic = baseTopic + "/state";
     
     // Create discovery config
     String configTopic = baseTopic + "/config";
     String configPayload = "{\"name\":\"" + String(config.deviceName) + " Gas Sensor\",\"device_class\":\"gas\",\"state_topic\":\"" + stateTopic + "\",\"unit_of_measurement\":\"ppm\",\"unique_id\":\"" + String(config.deviceName) + "_gas\"}";
   // Add small delay between connection and first publish
   delay(100);
        
   bool pubSuccess = mqttClient.publish(configTopic.c_str(), configPayload.c_str(), true);
   
   if (telnetClient && telnetClient.connected()) {
     telnetClient.printf("MQTT: Discovery config publish %s\n", pubSuccess ? "successful" : "failed");
   }
     // mqttClient.subscribe(("homeassistant/" + String(config.deviceName) + "/command").c_str());
  } else {
      int state = mqttClient.state();
      String errorMsg = "Initial MQTT connection failed, state: ";
      switch (state) {
          case -4: errorMsg += "MQTT_CONNECTION_TIMEOUT"; break;
          case -3: errorMsg += "MQTT_CONNECTION_LOST"; break;
          case -2: errorMsg += "MQTT_CONNECT_FAILED"; break;
          case -1: errorMsg += "MQTT_DISCONNECTED"; break;
          case 1: errorMsg += "MQTT_CONNECT_BAD_PROTOCOL"; break;
          case 2: errorMsg += "MQTT_CONNECT_BAD_CLIENT_ID"; break;
          case 3: errorMsg += "MQTT_CONNECT_UNAVAILABLE"; break;
          case 4: errorMsg += "MQTT_CONNECT_BAD_CREDENTIALS"; break;
          case 5: errorMsg += "MQTT_CONNECT_UNAUTHORIZED"; break;
          default: errorMsg += String(state);
      }
    //  printBoth(errorMsg);
     // printBoth("Will retry in main loop");
  }
}
void reconnectMQTT() {
  // Skip if MQTT is not configured or if server name is empty
  if (!mqttClient.connected() && config.mqttEnabled) {
    // First check if we have a valid network connection
    if (WiFi.status() != WL_CONNECTED) {
      if (telnetClient && telnetClient.connected()) {
        telnetClient.println("MQTT: WiFi not connected, skipping MQTT connection attempt");
      }
      return;
    }

  // Check if we're already connected
  if (mqttClient.connected()) {
    telnetClient.println("MQTT Connected");
      return;  // Already connected, nothing to do
  }

  // Try to connect once (non-blocking approach)
  telnetClient.println("Attempting MQTT connection as ");
  if (mqttClient.connect(config.mqttServer, config.mqttUser, config.mqttPassword)) {
      
    //  mqttClient.subscribe(("homeassistant/" + String(deviceConfig.hostname) + "/command").c_str());
  } else {
      int state = mqttClient.state();
      String errorMsg = "Connection failed, state: ";
      switch (state) {
          case -4: errorMsg += "MQTT_CONNECTION_TIMEOUT"; break;
          case -3: errorMsg += "MQTT_CONNECTION_LOST"; break;
          case -2: errorMsg += "MQTT_CONNECT_FAILED"; break;
          case -1: errorMsg += "MQTT_DISCONNECTED"; break;
          case 1: errorMsg += "MQTT_CONNECT_BAD_PROTOCOL"; break;
          case 2: errorMsg += "MQTT_CONNECT_BAD_CLIENT_ID"; break;
          case 3: errorMsg += "MQTT_CONNECT_UNAVAILABLE"; break;
          case 4: errorMsg += "MQTT_CONNECT_BAD_CREDENTIALS"; break;
          case 5: errorMsg += "MQTT_CONNECT_UNAUTHORIZED"; break;
          default: errorMsg += String(state);
      }
      //printBoth(errorMsg);
     // printBoth("Will try again later");
      // No delay here - the function will return and the main loop will continue
  }
}
}

void handleRoot() {
  String html = "<html><head><meta name='viewport' content='width=device-width, initial-scale=1.0'>";
  html += "<style>";
  html += "body { font-family: Arial, sans-serif; margin: 0; padding: 0; background: #f4f4f9; color: #333; }";
  html += "h1, h2 { text-align: center; color: #444; }";
  html += "form, .info-section, .danger-zone { max-width: 90%; margin: 1em auto; padding: 1em; background: #fff; border-radius: 5px; box-shadow: 0 2px 5px rgba(0,0,0,0.1); }";
  html += "input[type='text'], input[type='password'], input[type='number'] { width: 100%; padding: 0.7em; margin: 0.5em 0; border: 1px solid #ccc; border-radius: 3px; box-sizing: border-box; }";
  html += "input[type='submit'], button { width: 100%; background: #007BFF; color: white; border: none; padding: 0.7em; border-radius: 3px; cursor: pointer; font-size: 1em; }";
  html += "input[type='submit']:hover, button:hover { background: #0056b3; }";
  html += ".danger-button { background: #dc3545; }";
  html += ".danger-button:hover { background: #c82333; }";
  html += ".info-section p, .danger-zone p { margin: 0.5em 0; }";
  html += ".info-section strong, .danger-zone strong { display: inline-block; width: 50%; }";
  html += ".mqtt-settings { display: none; }";
  html += "</style>";

  html += "<script>";
  html += "function toggleMqttSettings() {";
  html += "  var mqttDiv = document.getElementById('mqttSettings');";
  html += "  var enabled = document.getElementById('mqttEnabled').checked;";
  html += "  mqttDiv.style.display = enabled ? 'block' : 'none';";
  html += "}";
  html += "function confirmReset() {";
  html += "  return confirm('WARNING: This will erase all settings including WiFi credentials and reboot the device. Continue?');";
  html += "}";
  html += "</script></head><body>";

  html += "<h1>Device Configuration</h1>";
  html += "<form action='/save' method='POST'>";
  html += "<label for='deviceName'>Device Name:</label>";
  html += "<input type='text' id='deviceName' name='deviceName' value='" + String(config.deviceName) + "'><br>";

  html += "<label for='mqttEnabled'>Enable MQTT:</label>";
  html += "<input type='checkbox' id='mqttEnabled' name='mqttEnabled' value='1' onchange='toggleMqttSettings()'";
  if (config.mqttEnabled) {
    html += " checked";
  }
  html += "><br>";

  html += "<div id='mqttSettings' class='mqtt-settings' style='display: " + String(config.mqttEnabled ? "block" : "none") + ";'>";
  html += "<label for='mqttServer'>MQTT Server:</label>";
  html += "<input type='text' id='mqttServer' name='mqttServer' value='" + String(config.mqttServer) + "'><br>";
  html += "<label for='mqttUser'>MQTT User:</label>";
  html += "<input type='text' id='mqttUser' name='mqttUser' value='" + String(config.mqttUser) + "'><br>";
  html += "<label for='mqttPassword'>MQTT Password:</label>";
  html += "<input type='password' id='mqttPassword' name='mqttPassword' value='" + String(config.mqttPassword) + "'><br>";
  html += "<label for='mqttPort'>MQTT Port:</label>";
  html += "<input type='number' id='mqttPort' name='mqttPort' value='" + String(config.mqttPort) + "'><br>";
  html += "</div>";

  html += "<label for='thresholdLimit'>Gas Threshold (ppm):</label>";
  html += "<input type='number' id='thresholdLimit' name='thresholdLimit' value='" + String(config.thresholdLimit) + "'><br>";
  html += "<label for='thresholdDuration'>Duration (s):</label>";
  html += "<input type='number' id='thresholdDuration' name='thresholdDuration' value='" + String(config.thresholdDuration) + "'><br>";

  html += "<label for='ntfyEnabled'>Enable NTFY Notifications:</label>";
  html += "<input type='checkbox' id='ntfyEnabled' name='ntfyEnabled' value='1'";
  if (config.ntfyEnabled) {
    html += " checked";
  }
  html += "><br>";

  html += "<label for='topicName'>Notification Topic:</label>";
  html += "<input type='text' id='topicName' name='topicName' value='" + String(config.topicName) + "' readonly><br>";

  html += "<input type='submit' value='Save'>";
  html += "</form>";

  html += "<div class='info-section'>";
  html += "<h2>Device Information</h2>";
  html += "<p><strong>IP Address:</strong><span>" + WiFi.localIP().toString() + "</span></p>";
  html += "<p><strong>MAC Address:</strong><span>" + WiFi.macAddress() + "</span></p>";
  
  // Add memory information
  uint32_t freeHeap = ESP.getFreeHeap();
  uint32_t maxFreeBlock = ESP.getMaxFreeBlockSize();
  uint32_t freeSketchSpace = (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;
  uint32_t flashChipSize = ESP.getFlashChipSize();
  
  html += "<p><strong>Free RAM:</strong><span>" + String(freeHeap) + " bytes</span></p>";
  html += "<p><strong>Largest Free Block:</strong><span>" + String(maxFreeBlock) + " bytes</span></p>";
  html += "<p><strong>Free Sketch Space:</strong><span>" + String(freeSketchSpace) + " bytes (" + String((freeSketchSpace * 100) / flashChipSize) + "%)</span></p>";
  html += "<p><strong>Flash Chip Size:</strong><span>" + String(flashChipSize) + " bytes</span></p>";
  html += "</div>";

  html += "<div class='info-section'>";
  html += "<h2>Sensor Values</h2>";
  html += "<p><strong>Raw Value:</strong><span>" + String(analogRead(gasSensorPin)) + "</span></p>";
  html += "<p><strong>Adjustment Value:</strong><span>" + String(config.baseGasValue) + "</span></p>";
  html += "<p><strong>Adjusted Value:</strong><span>" + String(analogRead(gasSensorPin) - config.baseGasValue) + "</span></p>";
  html += "</div>";

  html += "<div class='info-section'>";
  html += "<h2>Firmware Update</h2>";
  html += "<p>Download and install latest firmware from GitHub.</p>";
  html += "<a href='/update'><button style='background-color: #28a745;'>Update Firmware</button></a>";
  html += "</div>";

  html += "<div class='danger-zone'>";
  html += "<h2>Danger Zone</h2>";
  html += "<p>Erase all configurations including WiFi settings.</p>";
  html += "<a href='/reset' onclick='return confirmReset()'><button class='danger-button'>Reset Device</button></a>";
  html += "</div>";

  html += "<div class='danger-zone'>";
  html += "<h2>Calibration Reset</h2>";
  html += "<p>Erase the base gas value and restart the device to trigger calibration.</p>";
  html += "<a href='/reset-calibration'><button class='danger-button'>Reset Calibration</button></a>";
  html += "</div>";

  html += "<div class='danger-zone'>";
  html += "<h2>Restart Device</h2>";
  html += "<p>Restart the device without erasing configurations.</p>";
  html += "<a href='/restart'><button class='danger-button'>Restart Device</button></a>";
  html += "</div>";

  html += "<div class='danger-zone'>";
  html += "<h2>Reset WiFi Settings</h2>";
  html += "<p>Resetting WiFi settings will erase the saved WiFi credentials and restart the device to display the captive portal.</p>";
  html += "<a href='/reset-wifi'><button class='danger-button'>Reset WiFi Settings</button></a>";
  html += "</div>";

  html += "</body></html>";
  server.send(200, "text/html", html);
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

  // Handle MQTT client disconnect if being disabled
  if (wasMqttEnabled && !config.mqttEnabled) {
    mqttClient.disconnect();
  }
  
  saveConfig();
  
  // If MQTT was enabled, initialize it
  if (!wasMqttEnabled && config.mqttEnabled) {
    setupMQTT();
    reconnectMQTT();
  }
  
  server.send(200, "text/html", "<html><body><h1>Configuration Saved</h1><a href='/'>Go Back</a></body></html>");
}

void handleUpdate() {
  HTTPUpload& upload = server.upload();
  if (upload.status == UPLOAD_FILE_START) {
    Serial.println("Update: " + String(upload.filename));
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
      Serial.println("Update Success: " + String(upload.totalSize));
      server.send(200, "text/plain", "Update successful! Rebooting...");
      delay(1000);
      ESP.restart();
    } else {
      Update.printError(Serial);
    }
  }
  yield();
}

void handleUpdatePage() {
  String html = "<html><head><meta name='viewport' content='width=device-width, initial-scale=1.0'>";
  html += "<style>";
  html += "body { font-family: Arial, sans-serif; margin: 0; padding: 20px; background: #f4f4f9; color: #333; }";
  html += "h1 { text-align: center; color: #444; }";
  html += ".update-container { max-width: 600px; margin: 0 auto; padding: 20px; background: #fff; border-radius: 5px; box-shadow: 0 2px 5px rgba(0,0,0,0.1); }";
  html += ".progress { width: 100%; height: 20px; background: #eee; border-radius: 10px; margin: 20px 0; display: none; }";
  html += ".progress-bar { width: 0%; height: 100%; background: #28a745; border-radius: 10px; transition: width 0.3s; }";
  html += "button { width: 100%; background: #28a745; color: white; border: none; padding: 10px; border-radius: 5px; cursor: pointer; font-size: 16px; }";
  html += "button:hover { background: #218838; }";
  html += ".status { text-align: center; margin: 10px 0; }";
  html += "</style>";

  html += "<script>";
  html += "async function startUpdate() {";
  html += "  const status = document.getElementById('status');";
  html += "  const progress = document.getElementById('progress');";
  html += "  const progressBar = document.getElementById('progressBar');";
  html += "  try {";
  html += "    status.textContent = 'Downloading firmware...';";
  html += "    progress.style.display = 'block';";
  html += "    const response = await fetch('https://arjunus1985.github.io/GasDetect/fwroot/firmware.bin');";
  html += "    const firmware = await response.arrayBuffer();";
  html += "    const formData = new FormData();";
  html += "    formData.append('firmware', new Blob([firmware]), 'firmware.bin');";
  html += "    status.textContent = 'Uploading firmware to device...';";
  html += "    progressBar.style.width = '50%';";
  html += "    const updateResponse = await fetch('/do-update', {";
  html += "      method: 'POST',";
  html += "      body: formData";
  html += "    });";
  html += "    if (updateResponse.ok) {";
  html += "      status.textContent = 'Update successful! Device will restart...';";
  html += "      progressBar.style.width = '100%';";
  html += "      setTimeout(() => { window.location.href = '/'; }, 5000);";
  html += "    } else {";
  html += "      throw new Error('Update failed');";
  html += "    }";
  html += "  } catch (error) {";
  html += "    status.textContent = 'Error: ' + error.message;";
  html += "    progressBar.style.background = '#dc3545';";
  html += "  }";
  html += "}";
  html += "</script></head><body>";

  html += "<div class='update-container'>";
  html += "<h1>Firmware Update</h1>";
  html += "<p>This will download and install the latest firmware from GitHub.</p>";
  html += "<button onclick='startUpdate()'>Start Update</button>";
  html += "<div id='progress' class='progress'>";
  html += "<div id='progressBar' class='progress-bar'></div>";
  html += "</div>";
  html += "<div id='status' class='status'></div>";
  html += "<p><a href='/'>&larr; Back to main page</a></p>";
  html += "</div>";
  html += "</body></html>";

  server.send(200, "text/html", html);
}

// Callback for when device enters config mode
void configModeCallback(WiFiManager *myWiFiManager) {
  Serial.println("Failed to connect to WiFi");
  Serial.println("Entered config mode");
  Serial.println("AP IP address: " + WiFi.softAPIP().toString());
  Serial.println("AP SSID: " + myWiFiManager->getConfigPortalSSID());
}

void printGasDataBuffer() {
    
        for (int i = 0; i < BUFFER_SIZE; i++) {
          if (telnetClient && telnetClient.connected()) {
            telnetClient.println("Gas Data Buffer:");
            telnetClient.printf("[%.2f]", gasDataBuffer[i]);
          }
            Serial.printf("[%.2f]", gasDataBuffer[i]);
        }
        if (telnetClient && telnetClient.connected()) {
        telnetClient.printf("\n");
        }
        Serial.printf("\n");
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

void sendIpAddressNotification() {
  if (WiFi.status() == WL_CONNECTED ) {
    WiFiClientSecure client;
    client.setInsecure();
    HTTPClient http;

    String url = String("https://ntfy.sh/") + config.topicName;
    String ipAddress = WiFi.localIP().toString();
    String message = "Device IP Address: " + ipAddress;

    Serial.printf("Sending IP address notification to ntfy topic: %s\n", url.c_str());
    http.begin(client, url.c_str());
    http.addHeader("Title", "Device IP Address");

    int httpResponseCode = http.POST(message);
    if (httpResponseCode > 0) {
      Serial.printf("IP address notification sent successfully, HTTP code: %d\n", httpResponseCode);
    } else {
      Serial.printf("Failed to send IP address notification, HTTP error: %s\n", http.errorToString(httpResponseCode).c_str());
    }
    http.end();
  } else {
    Serial.println("WiFi not connected or ntfy notifications disabled, skipping IP address notification");
  }
}

void setup() {
  Serial.begin(9600);

  // Initialize LittleFS
  if (!LittleFS.begin()) {
    Serial.println("Failed to mount file system");
    return;
  }

  // Load configuration from LittleFS
  loadConfig();

  // Check if restart counter has reached 3 - if so, calibration reset
  if (config.restartCounter >= 3) {
    Serial.println("Restart counter reached 3 - performing calibration reset");
    
    
    // Reset counter to 0
    config.baseGasValue = -1;
    saveConfig();
    
    // Wait for WiFi disconnect to complete
    delay(1000);
  
  }
  if (config.restartCounter >= 5) {
    Serial.println("Restart counter reached 5 - performing factory reset");
    
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
  Serial.printf("Restart counter: %d\n", config.restartCounter);
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
  String apName = "GasDetector-" + String(ESP.getChipId());
  
  // Try to connect to WiFi or start AP mode if needed
  Serial.println("Attempting to connect to WiFi...");
  if (!wifiManager.autoConnect(apName.c_str())) {
    Serial.println("Failed to connect to WiFi and AP mode timed out");
    Serial.println("Continuing in offline mode, will retry WiFi connection later");
    // Set flag to indicate we're in offline mode after AP timeout
    apModeTimedOut = true;
    lastWifiRetryTime = millis();
  } else {
    Serial.println("Connected to WiFi");
  }

  // Ensure we are in station mode for mDNS
  WiFi.mode(WIFI_STA);
  delay(100);
  Serial.printf("WiFi mode: %d (1=STA,2=AP,3=STA+AP)\n", WiFi.getMode());

  // Format and sanitize hostname
  String hostname = String(config.deviceName);
  hostname.replace(' ', '-');
  for (size_t i = 0; i < hostname.length(); i++) {
    if (!isalnum(hostname[i]) && hostname[i] != '-') hostname[i] = '-';
  }
  hostname.toLowerCase();
  if (hostname.length() == 0) hostname = "gas-detector";

  // Set WiFi hostname and start mDNS responder
  WiFi.hostname(hostname.c_str());  // set DHCP hostname
  delay(100);
  Serial.print("DHCP hostname: ");
  Serial.println(WiFi.hostname());
  
  if (!MDNS.begin(hostname.c_str())) {
    Serial.println("Error setting up mDNS responder");
    // Debug info
    Serial.print("Local IP: "); Serial.println(WiFi.localIP());
    Serial.print("MAC: "); Serial.println(WiFi.macAddress());
  } else {
    MDNS.addService("http", "tcp", 80);
    MDNS.addService("telnet", "tcp", 23);
    Serial.printf("mDNS responder started: %s.local\n", hostname.c_str());
    // Debug info
    Serial.print("mDNS hostname: "); Serial.println(hostname + ".local");
  }

  // Configure OTA with same hostname
  ArduinoOTA.setHostname(hostname.c_str());

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_SPIFFS
      type = "filesystem";
    }
    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });
  ArduinoOTA.begin();

  Serial.println("OTA Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // Send IP address notification after reboot
  //sendIpAddressNotification();

  // Start Telnet server
  telnetServer.begin();
  telnetServer.setNoDelay(true);
  Serial.println("Telnet server started");

  


  // Start Web Server
  server.on("/", handleRoot);
  server.on("/save", HTTP_POST, handleSave);
 
  server.on("/reset", HTTP_GET, handleReset); // Add handler for reset
  server.on("/reset-calibration", HTTP_GET, handleResetCalibration); // Add handler for reset calibration
  server.on("/restart", HTTP_GET, handleRestart); // Add handler for restart
  server.on("/reset-wifi", HTTP_GET, handleResetWiFi); // Add handler for resetting only WiFi settings
  server.on("/update", HTTP_GET, handleUpdatePage);  // New route for update page
  server.on("/do-update", HTTP_POST, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
    ESP.restart();
  }, handleUpdate);
  server.begin();
  Serial.println("Web server started");

  // Only setup MQTT if enabled in config
  

  if (WiFi.status() == WL_CONNECTED && config.mqttEnabled)
  {
      setupMQTT();
  }
  else
  {
    Serial.println("WiFi not connected. Skipping MQTT setup.");
  }

  systemStartTime = millis(); // Record the system start time
  config.restartCounter = 0;
  saveConfig();
}

void loop() {
  // Handle OTA updates
  ArduinoOTA.handle();
  unsigned long currentTime = millis();
  
  // Check AP mode timeout and WiFi connection
  if (!apModeTimedOut && WiFi.getMode() == WIFI_AP) {
    if (currentTime - apModeStartTime >= AP_MODE_TIMEOUT) {
      Serial.println("AP mode timeout reached. Switching to offline mode.");
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
      Serial.println("Trying to reconnect to WiFi...");
      // Use WiFi.begin() without parameters to reconnect using stored credentials
      WiFi.begin();
      
      // Wait for connection for a reasonable time (e.g., 10 seconds)
      unsigned long connectStart = millis();
      while (WiFi.status() != WL_CONNECTED && millis() - connectStart < 10000) {
        delay(500);
        Serial.print(".");
      }
      
      if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nConnected to WiFi!");
      } else {
        Serial.println("\nFailed to connect to WiFi, continuing in offline mode");
      }
      
      lastWifiRetryTime = currentTime;
    }
  }
  
  // Accept new Telnet client using accept() instead of available
  if (telnetServer.hasClient()) {
    if (!telnetClient || !telnetClient.connected()) {
      if (telnetClient) telnetClient.stop();
      telnetClient = telnetServer.accept();
      Serial.println("New Telnet client connected");
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

  unsigned long now = millis();

  // Check if we need to start calibration after warmup
  if ((currentTime - systemStartTime > warmupTime) && config.baseGasValue == -1 && !calibrationRunning) {
    // Start calibration process
    calibrationRunning = true;
    calibrationStartTime = currentTime;
    calibrationReadingCount = 0;
    Serial.println("Starting calibration process for 5 minutes...");
    if (telnetClient && telnetClient.connected()) {
      telnetClient.println("Starting calibration process for 5 minutes...");
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
          telnetClient.printf("Calibration reading %d: %.2f\n", calibrationReadingCount, medianValue);
        }
        Serial.printf("Calibration reading %d: %.2f\n", calibrationReadingCount, medianValue);
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
      Serial.printf("Calibration complete. Base gas value: %d\n", config.baseGasValue);
      if (telnetClient && telnetClient.connected()) {
        telnetClient.printf("Calibration complete. Base gas value: %d\n", config.baseGasValue);
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
      Serial.printf("Gas Sensor Value: %.2f (raw: %.2f, base: %d)\n", gasReading, rawGasReading, config.baseGasValue);
      if (telnetClient && telnetClient.connected()) {
        telnetClient.printf("Gas Sensor Value: %.2f (raw: %.2f, base: %d)\n", gasReading, rawGasReading, config.baseGasValue);
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
          //telnet print median value
         // if (telnetClient && telnetClient.connected()) {
         //     telnetClient.printf("Median Gas Sensor Value: %.2f\n", medianValue);
         // }
          unsigned long currentTime = millis();

          // Skip publishing data for the first 10 seconds after system start
          if (currentTime - systemStartTime > 60000) {
            publishMQTTData(medianValue); // Publish median value
          lastPublishTime = now;
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

void publishMQTTData(int gasValue) {
  Serial.printf("Publishing gas value: %d\n", gasValue);
  if (telnetClient && telnetClient.connected()) {
    telnetClient.printf("Gas Sensor Value: %d\n", gasValue);
  }

  // Only publish if MQTT is enabled and connected
  if (config.mqttEnabled ) {
   
   
    mqttClient.loop();
    // Publish gas sensor value to MQTT topic
    String gasValueStr = String(gasValue);
    String topic = "homeassistant/sensor/" + String(config.deviceName) + "/gas/state";
    
    // Publish without retain flag for real-time updates
    bool pubSuccess = mqttClient.publish(topic.c_str(), gasValueStr.c_str(), false);
    
    // Log publish result to telnet if connected
    if (telnetClient && telnetClient.connected()) {
      telnetClient.printf("MQTT: State publish to %s %s: %s\n", topic.c_str(),gasValueStr.c_str(), pubSuccess ? "successful" : "failed");
    }
  }
}