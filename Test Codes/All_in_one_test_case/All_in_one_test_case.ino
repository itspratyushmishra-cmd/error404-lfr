/*
  Line Follower Comprehensive Test Code - Non-Blocking (Mega + ESP on Serial3)
  Components: 2x N20 Motors + Encoders, QTR-8 Array (A0-A7), 
              2x Digital IR (Pin 30, 32), HC-SR04 Ultrasonic (Pin 40, 41)
*/

// --- Pin Definitions ---
// Motors & Driver
const int ENA = 4;
const int IN1 = 33;
const int IN2 = 31;
const int ENB = 5;
const int IN3 = 35;
const int IN4 = 37;

// Encoders
const int ENC_L_A = 3; // Interrupt pin
const int ENC_L_B = 22;
const int ENC_R_A = 2; // Interrupt pin
const int ENC_R_B = 24;

// QTR-8 Sensor Array (Analog pins)
const int qtrPins[8] = {A0, A1, A2, A3, A4, A5, A6, A7};
int qtrValues[8];

// Digital IR Sensors
const int irLeftPin = 30;
const int irRightPin = 32;

// Ultrasonic Sensor (HC-SR04)
const int trigPin = 40;
const int echoPin = 41;

// --- Encoder Pulse Counters (Volatile for ISR) ---
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;

// --- Timing Variables (Millis / Micros based non-blocking) ---
unsigned long previousTelemetryMillis = 0;
const unsigned long telemetryInterval = 500; // Print data every 200ms

unsigned long previousMotorTestMillis = 0;
int motorTestState = 0; // State machine for simple motor test pattern

// --- Non-Blocking Ultrasonic State Machine Variables ---
enum UltrasonicState { US_IDLE, US_TRIGGERED, US_WAITING_ECHO };
UltrasonicState usState = US_IDLE;
unsigned long usTimer = 0;
unsigned long echoStartTime = 0;
float measuredDistanceCm = 0.0;
const unsigned long usInterval = 60; // Ping every 60ms

void setup() {
  // Initialize serial ports
  Serial.begin(115200);
  Serial3.begin(115200); // Connected to onboard ESP8266
  
  // Motor Pins Setup
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Encoder Pins Setup
  pinMode(ENC_L_A, INPUT_PULLUP);
  pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP);
  pinMode(ENC_R_B, INPUT_PULLUP);

  // Attach Interrupts for Encoders
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), isrLeftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), isrRightEncoder, RISING);

  // QTR Pins Setup
  for (int i = 0; i < 8; i++) {
    pinMode(qtrPins[i], INPUT);
  }

  // Digital IR Pins Setup
  pinMode(irLeftPin, INPUT);
  pinMode(irRightPin, INPUT);

  // Ultrasonic Pins Setup
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  digitalWrite(trigPin, LOW);

  printToBoth("=== ALL-COMPONENT TEST INITIALIZED ===");
}

void loop() {
  // 1. Run Non-Blocking Ultrasonic State Machine
  updateUltrasonicNonBlocking();

  // 2. Run Non-Blocking Motor Test Pattern (Forward -> Stop -> Reverse)
  runMotorTestPatternNonBlocking();

  // 3. Telemetry Streamer (Independent interval)
  unsigned long currentMillis = millis();
  if (currentMillis - previousTelemetryMillis >= telemetryInterval) {
    previousTelemetryMillis = currentMillis;
    
    readQTRSensors();
    bool irLeftState = digitalRead(irLeftPin);
    bool irRightState = digitalRead(irRightPin);

    // Format Telemetry String
    String telemetry = "[TEST]";
    telemetry += " QTR:[";
    for(int i=0; i<8; i++) {
      telemetry += String(qtrValues[i]);
      if(i < 7) telemetry += ",";
    }
    telemetry += "]";
    telemetry += " | IR_L:" + String(irLeftState) + " IR_R:" + String(irRightState);
    telemetry += " | Dist:" + String(measuredDistanceCm, 1) + "cm";
    telemetry += " | Enc_L:" + String(leftEncoderCount) + " Enc_R:" + String(rightEncoderCount);

    printToBoth(telemetry);
  }
}

// --- Encoder Interrupt Service Routines ---
void isrLeftEncoder() {
  if (digitalRead(ENC_L_B) == HIGH) {
    leftEncoderCount++;
  } else {
    leftEncoderCount--;
  }
}

void isrRightEncoder() {
  if (digitalRead(ENC_R_B) == HIGH) {
    rightEncoderCount++;
  } else {
    rightEncoderCount--;
  }
}

// --- Non-Blocking QTR Reader ---
void readQTRSensors() {
  for (int i = 0; i < 8; i++) {
    qtrValues[i] = analogRead(qtrPins[i]);
  }
}

// --- Non-Blocking Ultrasonic State Machine Implementation ---
void updateUltrasonicNonBlocking() {
  unsigned long currentMicros = micros();
  unsigned long currentMillis = millis();

  switch (usState) {
    case US_IDLE:
      if (currentMillis - usTimer >= usInterval) {
        usTimer = currentMillis;
        // Send 10us trigger pulse
        digitalWrite(trigPin, HIGH);
        usState = US_TRIGGERED;
        echoStartTime = currentMicros; // Use as trigger pulse start marker
      }
      break;

    case US_TRIGGERED:
      // Keep trigger high for at least 10 microseconds
      if (currentMicros - echoStartTime >= 10) {
        digitalWrite(trigPin, LOW);
        usState = US_WAITING_ECHO;
        echoStartTime = micros(); // Reset timer to monitor echo timeout
      }
      break;

    case US_WAITING_ECHO:
      int echoVal = digitalRead(echoPin);
      if (echoVal == HIGH) {
        // Echo has started, wait for it to go LOW
        // To keep it strictly non-blocking without pulseIn, we sample pin state
        // If high duration is too long (timeout ~30ms = max ~5 meters), reset
        unsigned long startPulse = micros();
        while (digitalRead(echoPin) == HIGH) {
          if (micros() - startPulse > 30000) { // 30ms timeout safeguard
            break;
          }
        }
        unsigned long duration = micros() - startPulse;
        measuredDistanceCm = (duration * 0.0343) / 2.0;
        usState = US_IDLE;
      } 
      else if (currentMicros - echoStartTime > 30000) {
        // Timeout: no object detected within range
        measuredDistanceCm = 999.0;
        usState = US_IDLE;
      }
      break;
  }
}

// --- Non-Blocking Motor Test Pattern ---
void runMotorTestPatternNonBlocking() {
  unsigned long currentMillis = millis();
  
  // Rotate test states every 3 seconds: 0: Forward, 1: Stop, 2: Reverse, 3: Stop
  if (currentMillis - previousMotorTestMillis >= 3000) {
    previousMotorTestMillis = currentMillis;
    motorTestState = (motorTestState + 1) % 4;

    switch (motorTestState) {
      case 0: // Move Forward
        setMotors(120, 120, true, true);
        break;
      case 1: // Stop
        setMotors(0, 0, true, true);
        break;
      case 2: // Move Reverse
        setMotors(120, 120, false, false);
        break;
      case 3: // Stop
        setMotors(0, 0, true, true);
        break;
    }
  }
}

// Helper to set motor speed and direction
void setMotors(int speedL, int speedR, bool forwardL, bool forwardR) {
  analogWrite(ENA, speedL);
  analogWrite(ENB, speedR);

  digitalWrite(IN1, forwardL ? HIGH : LOW);
  digitalWrite(IN2, forwardL ? LOW : HIGH);
  digitalWrite(IN3, forwardR ? HIGH : LOW);
  digitalWrite(IN4, forwardR ? LOW : HIGH);
}

// Helper to broadcast to USB and Serial3 (ESP)
void printToBoth(String msg) {
  Serial.println(msg);
  Serial3.println(msg);
}