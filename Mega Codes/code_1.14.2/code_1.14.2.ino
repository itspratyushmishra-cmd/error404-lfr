/*
============================================================
       UNIVERSAL HYBRID QTR LINE FOLLOWER (V3.6 - MEGA 2560)
       Supports: Black-on-White & White-on-Black
============================================================
BOARD:   Arduino Mega 2560 (ATmega2560)
SENSOR:  Pololu QTR-8A (Analog array on A0-A7, Emitter D13)
DRIVER:  TB6612FNG (Dual H-Bridge)

TELEMETRY SPECIFICATION (Serial3):
- Strictly silent during calibration and startup countdown.
- Video sync marker: LED_BUILTIN turns ON at exact countdown start:
  "[SYNC] LED_ON,<millis>"
- $POS at 10 Hz (every 100 ms): $POS,<pos>,<visible>
- $DAT at 5 Hz  (every 200 ms): 
  $DAT,<encL>,<encR>,<irL>,<irR>,<usCm_x10>,<state>,<dirL>,<pwmL>,<dirR>,<pwmR>,<pTerm_x100>,<iTerm_x100>,<dTerm_x100>
- State transition logged on action block:
  "[STATE] <prevState> -> <newState>"

PIN ASSIGNMENT:
- QTR Array: A0 (Far Left) -> A7 (Far Right), Emitter D13
- Left Mot:  PWM=D4 (Timer0), IN1=D33, IN2=D31
- Right Mot: PWM=D5 (Timer3), IN1=D35, IN2=D37
- TB6612:    STBY=D7 (HIGH = Enabled)
- Encoders:  L: A=D3 (INT5), B=D22 | R: A=D2 (INT4), B=D24
- BG IR:     Left=D30, Right=D32
- Ultrasonic: TRIG=D40, ECHO=D41
- Button:    D12 (Active LOW / Internal Pullup)
- Wi-Fi UART: Serial3 TX=D14, RX=D15
- Video Sync LED: LED_BUILTIN (Pin 13)
============================================================
*/

#include <Arduino.h>
#include <QTRSensors.h>

/* ============================================================
USER CONFIG & DEFAULTS
============================================================ */

#define TRACK_BLACK_ON_WHITE  0
#define TRACK_WHITE_ON_BLACK  1
#define TRACK_AUTO_DETECT     2

int trackMode = TRACK_BLACK_ON_WHITE;
int autoStart = 1;
int debugSerial = 1;
int autoCalSweep = 0;
int sensorReversed = 0;

#define LEFT_INVERT  0
#define RIGHT_INVERT 0

int stopAtFinish = 0;

#define POLICY_LEFT      0
#define POLICY_RIGHT     1
#define POLICY_STRAIGHT  2

int junctionPolicy = POLICY_RIGHT;
int uturnDir = -1; // -1 = Left, +1 = Right

// Encoder Inversion
#define ENC_LEFT_INVERT   0
#define ENC_RIGHT_INVERT  0

// Background IR Settings
int irLeftBlackIsHigh = 1;
int irRightBlackIsHigh = 1;

/* ============================================================
GLOBAL SPEED MULTIPLIER & SPEED REGISTERS
============================================================ */
// speedScale uniformly multiplies all speeds (e.g., 0.90 scales everything to 90%)
float speedScale = 1.0f;

int baseSpeed        = 180;
int maxSpeed         = 250;
int minSpeed         = 0;
int sweepSpeed       = 130;
int tickSpeed        = 140;
int acrossSpeed      = 150;
int turnOutSpeed     = 170;
int turnInSpeed      = 90;
int finishPullSpeed  = 140;
int creepSpeed       = 100;
int uturnSpeed       = 160;

// Helper to scale speeds uniformly
int scaleSpeed(int spd) {
  return constrain((int)((float)spd * speedScale), 0, 255);
}

/* ============================================================
PID CONTROLLER TUNING
============================================================ */
float Kp = 0.045f;
float Ki = 0.0002f;
float Kd = 0.38f;

float integral = 0.0f;
float maxIntegral = 15000.0f;
int lastError = 0;

// Variables storing latest PID breakdown
float pidPTerm = 0.0f;
float pidITerm = 0.0f;
float pidDTerm = 0.0f;
int lastPIDTurn = 0;

/* ============================================================
HARDWARE PIN ASSIGNMENTS
============================================================ */
const int PIN_ENA   = 4;
const int PIN_IN1   = 33;
const int PIN_IN2   = 31;

const int PIN_ENB   = 5;
const int PIN_IN3   = 35;
const int PIN_IN4   = 37;

const int PIN_STBY  = 7;

const int PIN_ENC_L_A = 3;
const int PIN_ENC_L_B = 22;
const int PIN_ENC_R_A = 2;
const int PIN_ENC_R_B = 24;

const int PIN_IR_LEFT  = 30;
const int PIN_IR_RIGHT = 32;

const int PIN_TRIG = 40;
const int PIN_ECHO = 41;

const int PIN_BUTTON = 12;

/* ============================================================
ROBOT STATE MACHINE
============================================================ */
enum RobotState {
  ST_STOPPED,
  ST_FOLLOW,
  ST_JUNC_PROBE,
  ST_PIVOT,
  ST_UTURN,
  ST_LOST,
  ST_FINISH
};

RobotState state = ST_STOPPED;
unsigned long stateEntryTime = 0;

const char* stateToString(RobotState s) {
  switch (s) {
    case ST_STOPPED:    return "STOPPED";
    case ST_FOLLOW:     return "FOLLOW";
    case ST_JUNC_PROBE: return "JUNC_PROBE";
    case ST_PIVOT:      return "PIVOT";
    case ST_UTURN:      return "UTURN";
    case ST_LOST:       return "LOST";
    case ST_FINISH:     return "FINISH";
    default:            return "UNKNOWN";
  }
}

/* ============================================================
ENCODER INTERRUPTS (ATOMIC 32-BIT COUNTERS)
============================================================ */
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;

void isrLeftEncoder() {
  if (digitalRead(PIN_ENC_L_B) == LOW) {
    ENC_LEFT_INVERT ? leftEncoderCount-- : leftEncoderCount++;
  } else {
    ENC_LEFT_INVERT ? leftEncoderCount++ : leftEncoderCount--;
  }
}

void isrRightEncoder() {
  if (digitalRead(PIN_ENC_R_B) == HIGH) {
    ENC_RIGHT_INVERT ? rightEncoderCount-- : rightEncoderCount++;
  } else {
    ENC_RIGHT_INVERT ? rightEncoderCount++ : rightEncoderCount--;
  }
}

long getLeftEncoder() {
  long val;
  uint8_t oldSREG = SREG;
  cli();
  val = leftEncoderCount;
  SREG = oldSREG;
  return val;
}

long getRightEncoder() {
  long val;
  uint8_t oldSREG = SREG;
  cli();
  val = rightEncoderCount;
  SREG = oldSREG;
  return val;
}

/* ============================================================
EXACT MOTOR DRIVER & PWM TELEMETRY TRACKING
============================================================ */
int actualDirLeft = 0;   // +1 = FWD, -1 = REV, 0 = STOP
int actualPwmLeft = 0;   // 0 to 255
int actualDirRight = 0;  // +1 = FWD, -1 = REV, 0 = STOP
int actualPwmRight = 0;  // 0 to 255

void setMotor(int motor, int speed) {
  speed = constrain(speed, -255, 255);

  if (motor == 0) { // Left Motor
    if (LEFT_INVERT) speed = -speed;
    actualPwmLeft = abs(speed);
    if (speed > 0) {
      actualDirLeft = 1;
      digitalWrite(PIN_IN1, LOW);
      digitalWrite(PIN_IN2, HIGH);
    } else if (speed < 0) {
      actualDirLeft = -1;
      digitalWrite(PIN_IN1, HIGH);
      digitalWrite(PIN_IN2, LOW);
    } else {
      actualDirLeft = 0;
      digitalWrite(PIN_IN1, LOW);
      digitalWrite(PIN_IN2, LOW);
    }
    analogWrite(PIN_ENA, actualPwmLeft);
  } else { // Right Motor
    if (RIGHT_INVERT) speed = -speed;
    actualPwmRight = abs(speed);
    if (speed > 0) {
      actualDirRight = 1;
      digitalWrite(PIN_IN3, LOW);
      digitalWrite(PIN_IN4, HIGH);
    } else if (speed < 0) {
      actualDirRight = -1;
      digitalWrite(PIN_IN3, HIGH);
      digitalWrite(PIN_IN4, LOW);
    } else {
      actualDirRight = 0;
      digitalWrite(PIN_IN3, LOW);
      digitalWrite(PIN_IN4, LOW);
    }
    analogWrite(PIN_ENB, actualPwmRight);
  }
}

void drive(int leftSpeed, int rightSpeed) {
  setMotor(0, leftSpeed);
  setMotor(1, rightSpeed);
}

void brake() {
  actualDirLeft = 0;
  actualPwmLeft = 0;
  actualDirRight = 0;
  actualPwmRight = 0;
  digitalWrite(PIN_IN1, HIGH);
  digitalWrite(PIN_IN2, HIGH);
  analogWrite(PIN_ENA, 0);
  digitalWrite(PIN_IN3, HIGH);
  digitalWrite(PIN_IN4, HIGH);
  analogWrite(PIN_ENB, 0);
}

/* ============================================================
NON-BLOCKING ULTRASONIC STATE MACHINE
============================================================ */
enum UltrasonicState {
  US_IDLE,
  US_TRIGGERING,
  US_WAIT_ECHO_HIGH,
  US_WAIT_ECHO_LOW
};

UltrasonicState usState = US_IDLE;
unsigned long usLastPingMillis = 0;
unsigned long usTrigStartTimeMicros = 0;
unsigned long usEchoStartTimeMicros = 0;
float measuredDistanceCm = 999.0f;
const unsigned long US_PING_INTERVAL_MS = 60;

void updateUltrasonicNonBlocking() {
  unsigned long nowMs = millis();
  unsigned long nowUs = micros();

  switch (usState) {
    case US_IDLE:
      if (nowMs - usLastPingMillis >= US_PING_INTERVAL_MS) {
        usLastPingMillis = nowMs;
        digitalWrite(PIN_TRIG, HIGH);
        usTrigStartTimeMicros = nowUs;
        usState = US_TRIGGERING;
      }
      break;

    case US_TRIGGERING:
      if (nowUs - usTrigStartTimeMicros >= 10) {
        digitalWrite(PIN_TRIG, LOW);
        usTrigStartTimeMicros = nowUs;
        usState = US_WAIT_ECHO_HIGH;
      }
      break;

    case US_WAIT_ECHO_HIGH:
      if (digitalRead(PIN_ECHO) == HIGH) {
        usEchoStartTimeMicros = nowUs;
        usState = US_WAIT_ECHO_LOW;
      } else if (nowUs - usTrigStartTimeMicros > 3000) {
        measuredDistanceCm = 999.0f;
        usState = US_IDLE;
      }
      break;

    case US_WAIT_ECHO_LOW:
      if (digitalRead(PIN_ECHO) == LOW) {
        unsigned long echoDuration = nowUs - usEchoStartTimeMicros;
        measuredDistanceCm = (float)echoDuration / 58.2f;
        usState = US_IDLE;
      } else if (nowUs - usEchoStartTimeMicros > 25000) {
        measuredDistanceCm = 999.0f;
        usState = US_IDLE;
      }
      break;
  }
}

/* ============================================================
QTR SENSORS & LINE POSITION
============================================================ */
QTRSensors qtr;
const uint8_t SENSOR_COUNT = 8;
uint16_t sensorValues[SENSOR_COUNT];
uint16_t lastPosition = 3500;
bool lineVisible = false;

/* ============================================================
TELEMETRY FLAGS & RATE TIMERS
============================================================ */
bool telemetryActive = false; // Strictly false until calibration & countdown end
unsigned long lastPosMillis = 0;
unsigned long lastDatMillis = 0;

void transitionToState(RobotState newState) {
  if (state == newState) return;
  RobotState prevState = state;
  state = newState;
  stateEntryTime = millis();

  // Action block prints state transitions atomically
  char stateMsg[48];
  snprintf(stateMsg, sizeof(stateMsg), "[STATE] %s -> %s", stateToString(prevState), stateToString(newState));
  if (debugSerial) Serial.println(stateMsg);
  if (telemetryActive) Serial3.println(stateMsg);
}

/* ============================================================
CALIBRATION & STARTUP
============================================================ */
void calibrateSensors() {
  if (debugSerial) Serial.println(F("[CAL] Starting QTR calibration..."));

  for (uint16_t i = 0; i < 400; i++) {
    if (autoCalSweep) {
      if (i < 100 || i >= 300) {
        drive(-scaleSpeed(sweepSpeed), scaleSpeed(sweepSpeed));
      } else {
        drive(scaleSpeed(sweepSpeed), -scaleSpeed(sweepSpeed));
      }
    }
    qtr.calibrate();
    delay(5);
  }
  brake();

  if (debugSerial) Serial.println(F("[CAL] QTR Calibration Complete."));
}

void autoDetectTrack() {
  uint32_t sum = 0;
  for (int sample = 0; sample < 10; sample++) {
    qtr.read(sensorValues);
    for (int i = 0; i < SENSOR_COUNT; i++) {
      sum += sensorValues[i];
    }
    delay(10);
  }
  uint32_t avg = sum / (SENSOR_COUNT * 10);
  if (avg > 1500) {
    trackMode = TRACK_WHITE_ON_BLACK;
    if (debugSerial) Serial.println(F("[TRACK] White line on Black surface detected"));
  } else {
    trackMode = TRACK_BLACK_ON_WHITE;
    if (debugSerial) Serial.println(F("[TRACK] Black line on White surface detected"));
  }
}

/* ============================================================
COMMAND PARSER ($CFG)
============================================================ */
char serialBuffer[160];
uint8_t bufferIdx = 0;

void processCommand(char* cmd) {
  if (strncmp(cmd, "$CFG,", 5) == 0) {
    char* token = strtok(cmd + 5, ",");
    if (!token) return;
    float nSpeedScale = atof(token);

    token = strtok(NULL, ","); if (!token) return; float nKp = atof(token);
    token = strtok(NULL, ","); if (!token) return; float nKi = atof(token);
    token = strtok(NULL, ","); if (!token) return; float nKd = atof(token);

    token = strtok(NULL, ","); if (!token) return; int nBase = atoi(token);
    token = strtok(NULL, ","); if (!token) return; int nMax = atoi(token);
    token = strtok(NULL, ","); if (!token) return; int nMin = atoi(token);
    token = strtok(NULL, ","); if (!token) return; int nSweep = atoi(token);
    token = strtok(NULL, ","); if (!token) return; int nTick = atoi(token);
    token = strtok(NULL, ","); if (!token) return; int nAcross = atoi(token);
    token = strtok(NULL, ","); if (!token) return; int nTurnOut = atoi(token);
    token = strtok(NULL, ","); if (!token) return; int nTurnIn = atoi(token);
    token = strtok(NULL, ","); if (!token) return; int nFinishPull = atoi(token);
    token = strtok(NULL, ","); if (!token) return; int nCreep = atoi(token);
    token = strtok(NULL, ","); if (!token) return; int nUturn = atoi(token);

    token = strtok(NULL, ","); if (!token) return; int nPolicy = atoi(token);
    token = strtok(NULL, ","); if (!token) return; int nUturnDir = atoi(token);
    token = strtok(NULL, ","); if (!token) return; int nAutoStart = atoi(token);
    token = strtok(NULL, ","); if (!token) return; int nStopFinish = atoi(token);
    token = strtok(NULL, ","); if (!token) return; int nTrackMode = atoi(token);
    token = strtok(NULL, ","); if (!token) return; int nIrLHigh = atoi(token);
    token = strtok(NULL, ","); if (!token) return; int nIrRHigh = atoi(token);

    speedScale = constrain(nSpeedScale, 0.05f, 2.0f);
    Kp = nKp; Ki = nKi; Kd = nKd;
    baseSpeed = nBase; maxSpeed = nMax; minSpeed = nMin;
    sweepSpeed = nSweep; tickSpeed = nTick; acrossSpeed = nAcross;
    turnOutSpeed = nTurnOut; turnInSpeed = nTurnIn; finishPullSpeed = nFinishPull;
    creepSpeed = nCreep; uturnSpeed = nUturn;
    junctionPolicy = nPolicy; uturnDir = nUturnDir;
    autoStart = nAutoStart; stopAtFinish = nStopFinish;
    trackMode = nTrackMode;
    irLeftBlackIsHigh = nIrLHigh; irRightBlackIsHigh = nIrRHigh;

    Serial3.println(F("$ACK,CFG,OK"));
    if (debugSerial) Serial.println(F("[ACK] CFG applied successfully."));
  }
}

void checkSerialInputs() {
  while (Serial3.available()) {
    char c = Serial3.read();
    if (c == '\n' || c == '\r') {
      if (bufferIdx > 0) {
        serialBuffer[bufferIdx] = '\0';
        processCommand(serialBuffer);
        bufferIdx = 0;
      }
    } else if (bufferIdx < sizeof(serialBuffer) - 1) {
      serialBuffer[bufferIdx++] = c;
    }
  }
}

/* ============================================================
TELEMETRY TRANSMISSION (10 Hz POS, 5 Hz DAT)
============================================================ */
void handleTelemetryStreaming() {
  if (!telemetryActive) return;

  unsigned long now = millis();

  // 1. $POS at 10 Hz (every 100 ms)
  if (now - lastPosMillis >= 100) {
    lastPosMillis = now;
    char posBuf[32];
    snprintf(posBuf, sizeof(posBuf), "$POS,%u,%d", lastPosition, lineVisible ? 1 : 0);
    Serial3.println(posBuf);
    if (debugSerial) Serial.println(posBuf);
  }

  // 2. $DAT at 5 Hz (every 200 ms)
  if (now - lastDatMillis >= 200) {
    lastDatMillis = now;

    long encL = getLeftEncoder();
    long encR = getRightEncoder();
    int irL = (digitalRead(PIN_IR_LEFT) == irLeftBlackIsHigh) ? 1 : 0;
    int irR = (digitalRead(PIN_IR_RIGHT) == irRightBlackIsHigh) ? 1 : 0;

    // Fixed-Point Integer Conversions:
    // Ultrasonic cm multiplied by 10 (e.g. 15.4 cm -> 154)
    long usDistInt = (long)(measuredDistanceCm * 10.0f);
    // PID terms multiplied by 100 (e.g. 1.25 -> 125, -0.40 -> -40)
    long pTermInt = (long)(pidPTerm * 100.0f);
    long iTermInt = (long)(pidITerm * 100.0f);
    long dTermInt = (long)(pidDTerm * 100.0f);

    char datBuf[128];
    snprintf(datBuf, sizeof(datBuf),
             "$DAT,%ld,%ld,%d,%d,%ld,%s,%d,%d,%d,%d,%ld,%ld,%ld",
             encL, encR, irL, irR, usDistInt, stateToString(state),
             actualDirLeft, actualPwmLeft, actualDirRight, actualPwmRight,
             pTermInt, iTermInt, dTermInt);

    Serial3.println(datBuf);
    if (debugSerial) Serial.println(datBuf);
  }
}

/* ============================================================
PID LINE FOLLOWING CALCULATION
============================================================ */
void followPID() {
  int error = (int)lastPosition - 3500;

  pidPTerm = Kp * (float)error;
  integral += (float)error;
  integral = constrain(integral, -maxIntegral, maxIntegral);
  pidITerm = Ki * integral;
  pidDTerm = Kd * (float)(error - lastError);
  lastError = error;

  lastPIDTurn = (int)(pidPTerm + pidITerm + pidDTerm);

  int leftSpd  = scaleSpeed(baseSpeed) + lastPIDTurn;
  int rightSpd = scaleSpeed(baseSpeed) - lastPIDTurn;

  leftSpd  = constrain(leftSpd, -scaleSpeed(maxSpeed), scaleSpeed(maxSpeed));
  rightSpd = constrain(rightSpd, -scaleSpeed(maxSpeed), scaleSpeed(maxSpeed));

  drive(leftSpd, rightSpd);
}

/* ============================================================
SETUP
============================================================ */
void setup() {
  Serial.begin(115200);
  Serial3.begin(115200);

  // Motor Drivers
  pinMode(PIN_ENA, OUTPUT);
  pinMode(PIN_ENB, OUTPUT);
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);
  pinMode(PIN_STBY, OUTPUT);
  digitalWrite(PIN_STBY, HIGH);

  // Video Sync Built-in LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // Encoders
  pinMode(PIN_ENC_L_A, INPUT_PULLUP);
  pinMode(PIN_ENC_L_B, INPUT_PULLUP);
  pinMode(PIN_ENC_R_A, INPUT_PULLUP);
  pinMode(PIN_ENC_R_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_A), isrLeftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_A), isrRightEncoder, RISING);

  // Digital IR Sensors
  pinMode(PIN_IR_LEFT, INPUT);
  pinMode(PIN_IR_RIGHT, INPUT);

  // Ultrasonic Sensor
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  digitalWrite(PIN_TRIG, LOW);

  // Button
  pinMode(PIN_BUTTON, INPUT_PULLUP);

  // QTR Array Setup
  qtr.setTypeAnalog();
  const uint8_t qtrPins[SENSOR_COUNT] = {A0, A1, A2, A3, A4, A5, A6, A7};
  qtr.setSensorPins(qtrPins, SENSOR_COUNT);
  qtr.setEmitterPin(13);

  // 1. Calibrate Sensors
  calibrateSensors();

  // 2. Track Auto-Detection
  if (trackMode == TRACK_AUTO_DETECT) {
    autoDetectTrack();
  }

  // 3. User Start Confirmation
  if (!autoStart) {
    if (debugSerial) Serial.println(F("[WAIT] Press Button D12 to Start..."));
    while (digitalRead(PIN_BUTTON) == HIGH) {
      delay(10);
    }
  }

  // 4. Exact Video Sync Marker and 2-Second Countdown
  digitalWrite(LED_BUILTIN, HIGH);
  unsigned long syncTimestamp = millis();
  char syncBuf[40];
  snprintf(syncBuf, sizeof(syncBuf), "[SYNC] LED_ON,%lu", syncTimestamp);
  Serial.println(syncBuf);
  Serial3.println(syncBuf);

  if (debugSerial) Serial.println(F("[START] Starting in 2 seconds..."));
  delay(2000);

  // 5. Activate Telemetry and Transition to Follow State
  telemetryActive = true;
  transitionToState(ST_FOLLOW);
}

/* ============================================================
MAIN LOOP
============================================================ */
void loop() {
  // 1. Maintain Non-Blocking Ultrasonic Sensor
  updateUltrasonicNonBlocking();

  // 2. Process Incoming Telemetry Commands ($CFG)
  checkSerialInputs();

  // 3. Read QTR Sensor Array
  if (trackMode == TRACK_WHITE_ON_BLACK) {
    lastPosition = qtr.readLineWhite(sensorValues);
  } else {
    lastPosition = qtr.readLineBlack(sensorValues);
  }

  // Determine line visibility
  uint16_t onLineCount = 0;
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    if (sensorValues[i] > 600) onLineCount++;
  }
  lineVisible = (onLineCount > 0);

  // 4. Line Follower State Machine
  switch (state) {
    case ST_FOLLOW: {
      if (lineVisible) {
        // Check for cross/T-junction (all or most sensors on line)
        if (sensorValues[0] > 700 && sensorValues[7] > 700) {
          transitionToState(ST_JUNC_PROBE);
        } else {
          followPID();
        }
      } else {
        // Line lost: reset PID terms and transition to lost search
        pidPTerm = 0.0f;
        pidITerm = 0.0f;
        pidDTerm = 0.0f;
        transitionToState(ST_LOST);
      }
      break;
    }

    case ST_JUNC_PROBE: {
      drive(scaleSpeed(tickSpeed), scaleSpeed(tickSpeed));
      if (millis() - stateEntryTime >= 65) {
        transitionToState(ST_PIVOT);
      }
      break;
    }

    case ST_PIVOT: {
      if (junctionPolicy == POLICY_LEFT) {
        drive(-scaleSpeed(turnOutSpeed), scaleSpeed(turnOutSpeed));
      } else {
        drive(scaleSpeed(turnOutSpeed), -scaleSpeed(turnOutSpeed));
      }

      // Re-acquired line near center
      if (millis() - stateEntryTime > 150 && lastPosition >= 2500 && lastPosition <= 4500) {
        transitionToState(ST_FOLLOW);
      }
      break;
    }

    case ST_LOST: {
      // Rotate towards last known error direction
      if (lastError > 0) {
        drive(scaleSpeed(sweepSpeed), -scaleSpeed(sweepSpeed));
      } else {
        drive(-scaleSpeed(sweepSpeed), scaleSpeed(sweepSpeed));
      }

      if (lineVisible) {
        transitionToState(ST_FOLLOW);
      } else if (millis() - stateEntryTime > 600) {
        transitionToState(ST_UTURN);
      }
      break;
    }

    case ST_UTURN: {
      if (uturnDir < 0) {
        drive(-scaleSpeed(uturnSpeed), scaleSpeed(uturnSpeed));
      } else {
        drive(scaleSpeed(uturnSpeed), -scaleSpeed(uturnSpeed));
      }

      if (millis() - stateEntryTime > 250 && lineVisible && lastPosition >= 2000 && lastPosition <= 5000) {
        transitionToState(ST_FOLLOW);
      }
      break;
    }

    case ST_FINISH: {
      brake();
      break;
    }

    case ST_STOPPED:
    default: {
      brake();
      break;
    }
  }

  // 5. Emit Timed Telemetry Streams (10 Hz POS, 5 Hz DAT)
  handleTelemetryStreaming();
}