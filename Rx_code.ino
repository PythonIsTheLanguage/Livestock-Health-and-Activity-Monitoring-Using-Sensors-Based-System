#include <WiFi.h>
#include <WiFiUdp.h>

const char* ssid = "pascal";
const char* password = "pascalorot";
const int udpPort = 12345;

WiFiUDP udp;

void setup() {
  Serial.begin(115200);

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  udp.begin(udpPort);
}

void loop() {
  char incomingPacket[255];
  int packetSize = udp.parsePacket();
  if (packetSize) {
    int len = udp.read(incomingPacket, 255);
    if (len > 0) {
      incomingPacket[len] = 0;
    }
    Serial.println("Received data: ");
    Serial.println(incomingPacket);
  }
}