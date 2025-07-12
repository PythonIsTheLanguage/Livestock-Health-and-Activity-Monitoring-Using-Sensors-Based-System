#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <esp_now.h>
#include <WiFi.h>

// Initialize MPU6050
Adafruit_MPU6050 mpu;

// Define the MAC address of the gateway
uint8_t gatewayAddress[] = {0xEC, 0x64, 0xC9, 0x85, 0x61, 0x98};  // Update with your gateway's MAC address

// Define a unique node ID for this sensor node
const uint8_t NODE_ID = 1;

// Structure to hold sensor data
typedef struct struct_message {
  uint8_t nodeID;         // Unique ID for each node
  float temperature;      // Temperature from MPU6050
  float accelX;
  float accelY;
  float accelZ;
} struct_message;

struct_message sensorData;

void setup() {
  Serial.begin(115200);

  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("MPU6050 initialized");

  // Initialize WiFi in station mode
  WiFi.mode(WIFI_STA);

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register the gateway's MAC address
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, gatewayAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
}

void loop() {
  // Read accelerometer and temperature data from MPU6050
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  sensorData.temperature = temp.temperature;  // Get temperature from MPU6050

  // Read accelerometer data from MPU6050
  sensorData.accelX = a.acceleration.x;
  sensorData.accelY = a.acceleration.y;
  sensorData.accelZ = a.acceleration.z;

  // Add node ID to sensor data
  sensorData.nodeID = NODE_ID;

  // Print the sensor data to the Serial Monitor
  Serial.print("Node ID: ");
  Serial.println(sensorData.nodeID);
  Serial.print("Temperature: ");
  Serial.println(sensorData.temperature);
  Serial.print("AccelX: ");
  Serial.println(sensorData.accelX);
  Serial.print("AccelY: ");
  Serial.println(sensorData.accelY);
  Serial.print("AccelZ: ");
  Serial.println(sensorData.accelZ);
  Serial.println();  // Print a blank line between each data set

  // Send data via ESP-NOW
  esp_err_t result = esp_now_send(gatewayAddress, (uint8_t *)&sensorData, sizeof(sensorData));
  if (result == ESP_OK) {
    Serial.println("Data sent successfully");
  } else {
    Serial.println("Error sending data");
  }

  delay(5000);  // Delay between sends
}
