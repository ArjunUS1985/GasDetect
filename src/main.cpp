#include <Arduino.h>
#include <ArduinoOTA.h>
#include <ESP8266mDNS.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <PubSubClient.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <WiFiManager.h>

WiFiServer telnetServer(23);
WiFiClient telnetClient;
WiFiClient espClient;
PubSubClient mqttClient(espClient);
ESP8266WebServer server(80);

struct Config {
  char mqttServer[40];
  char mqttUser[40];
  char mqttPassword[40];
  char deviceName[40];
  int mqttPort;
  bool mqttEnabled;
};

Config config;

const int gasSensorPin = A0; // Analog pin connected to MQ9 gas sensor

unsigned long lastReconnectAttempt = 0;
unsigned long lastReadingTime = 0;

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

  if (serializeJson(json, configFile) == 0) {
    Serial.println("Failed to write to config file");
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

  configFile.close();
}

void reconnectMQTT() {
  if (!mqttClient.connected()) {
    unsigned long now = millis();
    if (now - lastReconnectAttempt > 5000) {
      lastReconnectAttempt = now;
      Serial.print("Attempting MQTT connection...");
      if (mqttClient.connect(config.deviceName, config.mqttUser, config.mqttPassword)) {
        Serial.println("connected");
        String configTopic = "homeassistant/sensor/" + String(config.deviceName) + "/config";
        String configPayload = "{\"name\": \"" + String(config.deviceName) + "\", \"state_topic\": \"" + String(config.deviceName) + "/state\", \"unit_of_measurement\": \"ppm\", \"value_template\": \"{{ value }}\"}";
        mqttClient.publish(configTopic.c_str(), configPayload.c_str(), true);

        String gasConfig = "{\"name\":\"" + String(config.deviceName) + " Gas Sensor\",\"device_class\":\"gas\",\"state_topic\":\"homeassistant/sensor/" + String(config.deviceName) + "/gasvalue/state\",\"unit_of_measurement\":\"ppm\",\"unique_id\":\"" + String(config.deviceName) + "_gas\"}";
        mqttClient.publish(("homeassistant/sensor/" + String(config.deviceName) + "/gasvalue/config").c_str(), gasConfig.c_str(), true);
      } else {
        Serial.print("failed, rc=");
        Serial.print(mqttClient.state());
        Serial.println(" try again in 5 seconds");
      }
    }
  }
}

void handleRoot() {
  String html = "<html><head><meta name='viewport' content='width=device-width, initial-scale=1.0'>";
  html += "<style>body { font-family: Arial, sans-serif; margin: 0; padding: 0; }";
  html += "form { max-width: 400px; margin: auto; padding: 1em; background: #f9f9f9; border-radius: 5px; }";
  html += "input[type='text'], input[type='password'] { width: 100%; padding: 0.5em; margin: 0.5em 0; border: 1px solid #ccc; border-radius: 3px; }";
  html += "input[type='submit'] { background: #007BFF; color: white; border: none; padding: 0.7em; border-radius: 3px; cursor: pointer; }";
  html += "input[type='submit']:hover { background: #0056b3; }";
  html += ".mqtt-settings { display: none; }";
  html += "</style>";
  
  // Add JavaScript for dynamic show/hide
  html += "<script>";
  html += "function toggleMqttSettings() {";
  html += "  var mqttDiv = document.getElementById('mqttSettings');";
  html += "  var enabled = document.getElementById('mqttEnabled').checked;";
  html += "  mqttDiv.style.display = enabled ? 'block' : 'none';";
  html += "}";
  html += "</script></head><body>";
  
  html += "<h1 style='text-align: center;'>Device Configuration</h1>";
  html += "<form action='/save' method='POST'>";
  html += "<label for='deviceName'>Device Name:</label>";
  html += "<input type='text' id='deviceName' name='deviceName' value='" + String(config.deviceName) + "'><br>";
  
  html += "<label for='mqttEnabled'>Enable MQTT:</label>";
  html += "<input type='checkbox' id='mqttEnabled' name='mqttEnabled' value='1' onchange='toggleMqttSettings()'";
  if (config.mqttEnabled) {
    html += " checked";
  }
  html += "><br>";

  // MQTT settings in a div that can be shown/hidden
  html += "<div id='mqttSettings' class='mqtt-settings' style='display: " + String(config.mqttEnabled ? "block" : "none") + ";'>";
  html += "<label for='mqttServer'>MQTT Server:</label>";
  html += "<input type='text' id='mqttServer' name='mqttServer' value='" + String(config.mqttServer) + "'><br>";
  html += "<label for='mqttUser'>MQTT User:</label>";
  html += "<input type='text' id='mqttUser' name='mqttUser' value='" + String(config.mqttUser) + "'><br>";
  html += "<label for='mqttPassword'>MQTT Password:</label>";
  html += "<input type='password' id='mqttPassword' name='mqttPassword' value='" + String(config.mqttPassword) + "'><br>";
  html += "<label for='mqttPort'>MQTT Port:</label>";
  html += "<input type='text' id='mqttPort' name='mqttPort' value='" + String(config.mqttPort) + "'><br>";
  html += "</div>";
  
  html += "<input type='submit' value='Save'>";
  html += "</form>";
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

  // Handle MQTT client disconnect if being disabled
  if (wasMqttEnabled && !config.mqttEnabled) {
    mqttClient.disconnect();
  }
  
  saveConfig();
  
  // If MQTT was enabled, initialize it
  if (!wasMqttEnabled && config.mqttEnabled) {
    mqttClient.setServer(config.mqttServer, config.mqttPort);
    reconnectMQTT();
  }
  
  server.send(200, "text/html", "<html><body><h1>Configuration Saved</h1><a href='/'>Go Back</a></body></html>");
}

// Callback for when device enters config mode
void configModeCallback(WiFiManager *myWiFiManager) {
  Serial.println("Failed to connect to WiFi");
  Serial.println("Entered config mode");
  Serial.println("AP IP address: " + WiFi.softAPIP().toString());
  Serial.println("AP SSID: " + myWiFiManager->getConfigPortalSSID());
}

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(115200);

  // WiFiManager setup
  WiFiManager wifiManager;
  
  // Set config mode callback
  wifiManager.setAPCallback(configModeCallback);
  
  // Set connection timeout to 30 seconds
  wifiManager.setConnectTimeout(30);
  
  // Uncomment to reset saved settings
  // wifiManager.resetSettings();
  
  // Set custom AP name
  String apName = "GasDetector-" + String(ESP.getChipId());
  
  // Fetches ssid and password and tries to connect
  // if it does not connect it starts an access point with the specified name
  // and goes into a blocking loop awaiting configuration
  if (!wifiManager.autoConnect(apName.c_str())) {
    Serial.println("Failed to connect and hit timeout");
    delay(3000);
    // Reset and try again
    ESP.reset();
    delay(1000);
  }
  
  Serial.println("Connected to WiFi");

  // Set up mDNS responder for hostname based on device name
  if (!MDNS.begin(config.deviceName)) {
    Serial.println("Error setting up MDNS responder!");
  } else {
    Serial.println("mDNS responder started for " + String(config.deviceName));
  }

  // Configure OTA
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

  // Start Telnet server
  telnetServer.begin();
  telnetServer.setNoDelay(true);
  Serial.println("Telnet server started");

  // Initialize LittleFS
  if (!LittleFS.begin()) {
    Serial.println("Failed to mount file system");
    return;
  }

  // Load configuration from LittleFS
  loadConfig();

  // Set default device name if not already configured
  if (strlen(config.deviceName) == 0) {
    strlcpy(config.deviceName, "GasDetector", sizeof(config.deviceName));
  }

  // Only setup MQTT if enabled in config
  if (config.mqttEnabled) {
    // Set up MQTT client
    mqttClient.setServer(config.mqttServer, config.mqttPort);
    // Ensure MQTT connection
    reconnectMQTT();
  }

  // Start Web Server
  server.on("/", handleRoot);
  server.on("/save", HTTP_POST, handleSave);
  server.begin();
  Serial.println("Web server started");
}

void loop() {
  // Handle OTA updates
  ArduinoOTA.handle();

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

  unsigned long now = millis();
  // Read and publish sensor data every second without blocking
  if (now - lastReadingTime >= 1000) {
    lastReadingTime = now;
    
    // Read gas sensor value
    int gasValue = analogRead(gasSensorPin);

    // Print gas sensor value to Telnet client
    if (telnetClient && telnetClient.connected()) {
      telnetClient.printf("Gas Sensor Value: %d\n", gasValue);
    }

    // Only perform MQTT operations if enabled
    if (config.mqttEnabled) {
      reconnectMQTT();
      if (mqttClient.connected()) {
        mqttClient.loop();
        // Publish gas sensor value to MQTT topic
        String gasValueStr = String(gasValue);
        String topic = String(config.deviceName) + "/state";
        mqttClient.publish(topic.c_str(), gasValueStr.c_str());
      }
    }
  }

  // Handle web server requests
  server.handleClient();
}