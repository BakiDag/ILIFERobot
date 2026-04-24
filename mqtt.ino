#include <ArduinoJson.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

const char* outTopic_status = "ilife-vacuum/status";
const char* inTopic_command = "ilife-vacuum/command";

extern WiFiClient espClient;
extern PubSubClient mqtt;
extern const char* mqtt_server;
extern const char* outTopic_debug;
// Externe Funktionen aus ILIFERobot.ino
extern float calcBattery(boolean returnPercent);
extern bool isCharging();
extern bool isDocked();
// Externe Funktionen aus ILIFERobot.ino
extern float calcBattery(boolean returnPercent);
extern bool isCharging();
extern bool isDocked();
extern Status robotStatus;

// statusPin1/2/3 sind #defines, KEINE extern ints!
// Sie werden direkt als 14, 12, 13 vom Compiler ersetzt

// =======================
// Forward Declarations
// =======================
void handleVacuumCommand(const char* action);
void publishCalibrationMeasurement(const char* eventType);

// ------------------------------------------------
// MQTT Callback — empfängt Befehle vom Home Assistant
// ------------------------------------------------
void callback(char* topic, byte* payload, unsigned int length) {
  payload[length] = '\0';
  String message = String((char*)payload);

  Serial.printf("[MQTT] Topic: %s | Payload: %s\n", topic, message.c_str());

  if (String(topic) == inTopic_command) {
    StaticJsonBuffer<200> jb;
    JsonObject& root = jb.parseObject(message);

    if (!root.success()) {
      Serial.println("[MQTT] JSON parse failed");
      return;
    }

    const char* action = root["action"];
    if (action) {
      handleVacuumCommand(action);
    }
  }
}

// ------------------------------------------------
// MQTT Reconnect — falls Verbindung verloren geht
// ------------------------------------------------
void reconnect() {
  while (!mqtt.connected()) {
    Serial.print("[MQTT] Attempting connection...");
    if (mqtt.connect("ILIFE_Vacuum", "MqttUserName", "password")) {
      Serial.println("connected!");
      mqtt.subscribe(inTopic_command);
      mqtt.publish(outTopic_debug, "ESP8266 reconnected to MQTT");
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqtt.state());
      Serial.println(" — retry in 5s");
      delay(5000);
    }
  }
}

// ------------------------------------------------
// Befehle vom MQTT → ILIFE (über IR-Signal)
// ------------------------------------------------
void handleVacuumCommand(const char* action) {
  Serial.printf("[COMMAND] Received action: %s\n", action);

  if (strcmp(action, "start_cleaning") == 0) {
    SendIRCode(rStart);
    Serial.println("[IR] Sent: START CLEANING");
  }
  else if (strcmp(action, "stop_cleaning") == 0) {
    SendIRCode(rStart); // falls Stop = Start-Toggle; sonst setze den passenden Button
    Serial.println("[IR] Sent: STOP CLEANING");
  }
  else if (strcmp(action, "return_to_dock") == 0) {
    SendIRCode(rHome);
    Serial.println("[IR] Sent: RETURN TO DOCK");
  }
  else if (strcmp(action, "spot_cleaning") == 0) {
    SendIRCode(rSpot);
    Serial.println("[IR] Sent: SPOT CLEANING");
  }
  else if (strcmp(action, "left") == 0) {
    SendIRCode(rLeft);
    Serial.println("[IR] Sent: LEFT");
  }
  else if (strcmp(action, "right") == 0) {
    SendIRCode(rRight);
    Serial.println("[IR] Sent: RIGHT");
  }
  
  else {
    Serial.println("[COMMAND] Unknown command");
  }
}

// ------------------------------------------------
// MQTT Setup — im setup() aufrufen
// ------------------------------------------------
void setupMQTT() {
  mqtt.setServer(mqtt_server, 1883);
  mqtt.setBufferSize(512);
  mqtt.setCallback(callback);
  mqtt.subscribe(inTopic_command);
  Serial.println("[MQTT] Initialized and subscribed to command topic.");
}

// ------------------------------------------------
// MQTT Loop — im loop() aufrufen
// ------------------------------------------------
void mqttLoop() {
  if (!mqtt.connected()) {
    reconnect();
  }
  mqtt.loop();
}
