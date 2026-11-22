#include <ESP8266WiFi.h>

// It connects to laptop hotspot over 2.5ghz and connects to python socket connection
// It takes any data from the serial input that is from the mega chip and forwards it to python



const char* ssid = "SHRESTHSHARMA03";
const char* password = "O/6i2292";

// Python TCP Server Details
const char* server_ip = "192.168.137.1"; 
const uint16_t server_port = 65432;      

WiFiClient client;

void setup() {
  // This Serial baud rate must match what you set on the Mega (e.g., Serial3.begin(115200))
  Serial.begin(115200);
  delay(10);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  // Wait silently for WiFi connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
}

void loop() {
  // 1. Maintain connection to Python server
  if (!client.connected()) {
    client.connect(server_ip, server_port);
    delay(500); // Prevent spamming connection attempts if server is down
    return;
  }

  // 2. Listen for Mega chip and forward to Python
  if (Serial.available()) {
    // Read the incoming message from the Mega
    String megaMessage = Serial.readStringUntil('\n');
    
    // Forward the exact message to the Python server
    client.println(megaMessage);
  }
}