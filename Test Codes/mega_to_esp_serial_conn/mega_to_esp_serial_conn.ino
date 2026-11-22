void setup() {
  // Serial3 connects the Mega to the ESP8266 (when DIP switches 1 and 2 are ON)
  // The baud rate MUST match what you set in the ESP8266 code (115200)
  Serial3.begin(115200);
  
  // Optional: Standard Serial for USB debugging to your computer
  Serial.begin(115200);
  delay(100);
  Serial.println("Mega is ready. Sending data...");
}

void loop() {
  String message = "Hello from Mega! Uptime: " + String(millis() / 1000) + "s";
  
  // Send to ESP8266 (which forwards to Python)
  Serial3.print(message);
  
  // Print to your computer's Serial Monitor so you can see it working
  Serial.println("Sent: " + message);
  
  // Wait 2 seconds before sending again
  delay(2000);
}