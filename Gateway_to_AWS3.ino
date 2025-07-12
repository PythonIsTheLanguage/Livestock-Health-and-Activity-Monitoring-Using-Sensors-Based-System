#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <esp_now.h>
#include <ArduinoJson.h>
#include "secrets.h"  // Include secrets for AWS IoT Core credentials

// Create a secure client
WiFiClientSecure net;
PubSubClient client(net);

const char* mqttServer = AWS_IOT_ENDPOINT;  // AWS IoT endpoint
const int mqttPort = 8883;  // MQTT over TLS port

// Define MQTT topics
#define AWS_IOT_PUBLISH_TOPIC "esp32/pub"
#define AWS_IOT_SUBSCRIBE_TOPIC "esp32/sub"

// Structure to hold incoming sensor data
typedef struct {
  uint8_t nodeID;
  float temperature;
  float accelX;
  float accelY;
  float accelZ;
} SensorData;

// Store time for periodic printing
unsigned long lastPrintTime = 0;  // Initialize last print time

// Callback for receiving ESP-NOW data
void onDataReceive(const esp_now_recv_info *info, const uint8_t *data, int len) {
  if (len != sizeof(SensorData)) {
    Serial.println("Received incorrect data length");
    return;
  }

  // Copy data to structure
  SensorData incomingData;
  memcpy(&incomingData, data, sizeof(incomingData));

  // Calculate activity magnitude
  float activityMagnitude = sqrt(pow(incomingData.accelX, 2) + pow(incomingData.accelY, 2) + pow(incomingData.accelZ, 2));

  // Print received data every 30 seconds
  if (millis() - lastPrintTime >= 30000) {
    Serial.println("\n--- Data Received ---");
    Serial.print("Node ID: ");
    Serial.println(incomingData.nodeID);
    Serial.print("Temperature: ");
    Serial.println(incomingData.temperature);
    Serial.print("Acceleration X: ");
    Serial.println(incomingData.accelX);
    Serial.print("Acceleration Y: ");
    Serial.println(incomingData.accelY);
    Serial.print("Acceleration Z: ");
    Serial.println(incomingData.accelZ);
    Serial.print("Activity Magnitude: ");
    Serial.println(activityMagnitude);

    lastPrintTime = millis();  // Reset last print time
  }

  // Send data to AWS
  String healthStatus = (incomingData.temperature >= 27 && incomingData.temperature <= 34) ? "Normal" : "Attention Required";
  sendToAWS(incomingData.nodeID, incomingData.temperature, activityMagnitude, healthStatus);
}

// Send data to AWS IoT Core using MQTT
void sendToAWS(uint8_t nodeID, float temperature, float activityMagnitude, String healthStatus) {
  String payload = "{\"CowID\": " + String(nodeID) +
                   ", \"Temperature\": " + String(temperature) +
                   ", \"ActivityMagnitude\": " + String(activityMagnitude) +
                   ", \"HealthStatus\": \"" + healthStatus + "\"}";

  client.publish(AWS_IOT_PUBLISH_TOPIC, payload.c_str());
}

// Connect to AWS IoT Core using MQTT
void connectToAWS() {
  while (!client.connected()) {
    Serial.print("Connecting to AWS IoT Core...");

    if (client.connect("ESP32_Client")) {
      Serial.println("Connected to AWS IoT Core!");
      client.subscribe(AWS_IOT_SUBSCRIBE_TOPIC);
    } else {
      Serial.print("Failed to connect. Retrying...");
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);

  // Connect to Wi-Fi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to Wi-Fi");

  // Set up secure connection for MQTT
  net.setCACert(AWS_ROOT_CA);
  net.setCertificate(CERTIFICATE);
  net.setPrivateKey(PRIVATE_KEY);

  // Initialize MQTT client
  client.setServer(mqttServer, mqttPort);

  // Connect to AWS IoT Core
  connectToAWS();

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW initialization failed");
    return;
  }

  // Register callback for receiving data
  esp_now_register_recv_cb(onDataReceive);
}

void loop() {
  // Keep the MQTT connection alive
  if (!client.connected()) {
    connectToAWS();
  }
  client.loop();  // Ensure MQTT client stays connected
}
