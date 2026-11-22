#include <ESP8266WiFi.h>

const char* ssid = "SHRESTHSHARMA03";
const char* password = "O/6i2292";

// Python TCP Server Details
const char* server_ip = "192.168.137.1";
const uint16_t server_port = 65432;

WiFiClient client;

void setup() {
  // Must match Mega's Serial3.begin(115200)
  Serial.begin(115200);
  delay(10);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  // Wait silently until Wi-Fi connects
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }

  // Send single confirmation message to Mega
  Serial.println("CONNECTED");
}

void loop() {
  // 1. Silently maintain TCP socket connection
  if (!client.connected()) {
    client.connect(server_ip, server_port);
    delay(500);
    return;
  }

  // 2. Forward everything from Mega to Python socket as-is
  while (Serial.available()) {
    uint8_t buf[128];
    size_t len = Serial.readBytes(buf, min((size_t)Serial.available(), sizeof(buf)));
    client.write(buf, len);
  }
}