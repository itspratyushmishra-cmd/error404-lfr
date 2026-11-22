#include <ESP8266WiFi.h>

const char* ssid = "SHRESTHSHARMA03";
const char* password = "O/6i2292";

// Python TCP Server Details
const char* server_ip = "192.168.137.1"; // Replace with your PC's local IP address
const uint16_t server_port = 65432;      // Port must match the Python script

WiFiClient client;

void setup() {
  Serial.begin(115200);
  delay(10);

  Serial.println("\nConnecting to WiFi...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected. IP: " + WiFi.localIP().toString());
}

void loop() {
  // 1. Reconnect to the Python server if disconnected
  if (!client.connected()) {
    Serial.println("Connecting to Python TCP Server...");
    if (client.connect(server_ip, server_port)) {
      Serial.println("Connected to server!");
      client.println("Hello from ESP8266!"); // Send initial greeting
    } else {
      Serial.println("Connection failed. Retrying in 5 seconds...");
      delay(5000);
      return; 
    }
  }

  // 2. Read incoming data from Python
  if (client.available()) {
    String incoming = client.readStringUntil('\n');
    Serial.print("Python says: ");
    Serial.println(incoming);
  }

  // 3. Send periodic telemetry/data to Python (every 2 seconds)
  static unsigned long lastTransmission = 0;
  if (millis() - lastTransmission > 2000) {
    client.println("Status: Active | Uptime: " + String(millis()));
    lastTransmission = millis();
  }
}