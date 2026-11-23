/*
============================================================
       UNIVERSAL HYBRID QTR LINE FOLLOWER (V3.9 - MEGA 2560)
       Supports: Black-on-White & White-on-Black
============================================================
BOARD:   Arduino Mega 2560 (ATmega2560)
SENSOR:  Pololu QTR-8A (Analog array on A0-A7, Emitter D13)
DRIVER:  TB6612FNG (Dual H-Bridge)

CONFIG INPUT PROTOCOL (Serial3):
- Waits before calibration for a single-line $CFG packet:
  $CFG,Kp,Ki,Kd,speedScale,spdStr,spdGnt,spdMed,spdShp,spdMin,spdPrb,spdAln,spdPiv,spdSch,trkMod,jncPol,stpFin,autSt,utrnDir,motLInv,motRInv,encLInv,encRInv\n

TELEMETRY SPECIFICATION (Serial3):
- $POS at 10 Hz (every 100 ms): 
    $POS,<pos>,<visible>
- $DAT at 5 Hz  (every 200 ms): 
    $DAT,<encL>,<encR>,<irL>,<irR>,<usDist>,<dirL>,<pwmL>,<dirR>,<pwmR>,<err>,<pTerm>,<iTerm>,<dTerm>,<corr>,<state>
- Action block state transitions:
    [ACTION] State: <oldState> -> <newState>
- Video Sync optical marker:
    [SYNC] LED_ON,<ms>

PIN ASSIGNMENT:
- QTR Array: A0 (Far Left) -> A7 (Far Right), Emitter D13
- Left Mot:  PWM=D4 (Timer0), IN1=D33, IN2=D31 (IN1 LOW + IN2 HIGH = FWD)
- Right Mot: PWM=D5 (Timer3), IN1=D37, IN2=D35 (IN1 LOW + IN2 HIGH = FWD)
- TB6612:    STBY=D7 (HIGH = Enabled)
- Encoders:  L: A=D3 (INT5), B=D22 | R: A=D2 (INT4), B=D24
- BG IR:     Left=D30, Right=D32
- Ultrasonic: TRIG=D40, ECHO=D41 (100% Non-blocking, no pulseIn)
- Button:    D12 (Active LOW / Internal Pullup)
- Onboard LED: LED_BUILTIN (Pin 13)
- Wi-Fi UART: Serial3 TX=D14, RX=D15
============================================================
*/

#include <Arduino.h>
#include <QTRSensors.h>
#include <avr/dtostrf.h>

/* ============================================================
GLOBAL CONFIGURABLE VARIABLES (LOADABLE VIA SERIAL3)
============================================================ */

#define TRACK_BLACK_ON_WHITE  0  // Black line on White surface
#define TRACK_WHITE_ON_BLACK  1  // White line on Black surface
#define TRACK_AUTO_DETECT     2  // Automatically detect based on contrast at startup

#define POLICY_LEFT      0
#define POLICY_RIGHT     1
#define POLICY_STRAIGHT  2

// 1. General Operational Config
uint8_t trackMode       = TRACK_BLACK_ON_WHITE; 
uint8_t autoStart       = 1;                    
uint8_t autoCalSweep    = 0;                    
uint8_t sensorReversed  = 0;                    
uint8_t leftInvert      = 0;                    
uint8_t rightInvert     = 0;
uint8_t stopAtFinish    = 0;                    
uint8_t junctionPolicy  = POLICY_RIGHT;         
int8_t  uturnDir        = -1;                   
uint8_t encLeftInvert   = 0;                    
uint8_t encRightInvert  = 0;

#define DEBUG_SERIAL 1

// Background IR polarity (1 = HIGH on black)
#define IR_LEFT_BLACK_IS_HIGH   1
#define IR_RIGHT_BLACK_IS_HIGH  1
#define IR_PIN_MODE INPUT
const uint8_t BG_CONFIRM_COUNT = 8;

// 2. PID Tuning Variables
float Kp = 0.068f;
float Ki = 0.0008f;
float Kd = 0.0018f;

float progressiveGain     = 0.000030f;
float errorDeadband       = 30.0f;
float maxIntegral         = 2500.0f;
float integralActiveZone  = 1200.0f;
float dFilterAlpha        = 0.40f;
float maxCorrection       = 240.0f;

// 3. Global Speed Multiplier & Base Speeds
float speedScale = 1.0f; // Global speed multiplier (e.g. 0.9 = 90% across all profiles)

int baseSpeedStraight      = 190;
int baseSpeedGentleCurve   = 90;
int baseSpeedMediumCurve   = 80;
int baseSpeedSharpCurve    = 70;
int baseSpeedMinBase       = 65;
int baseProbeSpeed         = 130;
int baseAlignSpeed         = 120;
int basePivotSpeed         = 160;
int basePivotCaptureSpeed  = 120;
int baseSearchSpeed        = 155;
int baseUturnSpeed         = 160;

// Scaled speeds used during runtime
int speedStraight      = 190;
int speedGentleCurve   = 90;
int speedMediumCurve   = 80;
int speedSharpCurve    = 70;
int speedMinBase       = 65;
int probeSpeed         = 130;
int alignSpeed         = 120;
int pivotSpeed         = 160;
int pivotCaptureSpeed  = 120;
int searchSpeed        = 155;
int uturnSpeed         = 160;

const int MOTOR_MAX_FORWARD = 255;
const int MOTOR_MAX_REVERSE = -100;

float currentBaseSpeed = 65.0f;
float accelRate        = 600.0f;
float decelRate        = 8000.0f;

/* ============================================================
PINS
============================================================ */

const uint8_t QTR_PINS[8] = { A0, A1, A2, A3, A4, A5, A6, A7 };
const uint8_t EMITTER_PIN = 13;

const uint8_t LEFT_PWM = 4;
const uint8_t LEFT_IN1 = 33;
const uint8_t LEFT_IN2 = 31;

const uint8_t RIGHT_PWM = 5;
const uint8_t RIGHT_IN1 = 37;
const uint8_t RIGHT_IN2 = 35;

const uint8_t STBY = 7;
const uint8_t BTN  = 12;

const uint8_t ENC_L_A = 3; // INT5
const uint8_t ENC_L_B = 22;
const uint8_t ENC_R_A = 2; // INT4
const uint8_t ENC_R_B = 24;

const uint8_t IR_LEFT_PIN  = 30;
const uint8_t IR_RIGHT_PIN = 32;

const uint8_t US_TRIG = 40;
const uint8_t US_ECHO = 41;

/* ============================================================
OBJECTS & TELEMETRY VARIABLES
============================================================ */

QTRSensors qtr;
uint16_t sensorValues[8];

bool isWhiteLine = false;

// Fixed 500 Hz control loop: 2000 us
const uint32_t LOOP_INTERVAL_US = 2000;
uint32_t lastLoopMicros = 0;
float dt = 0.0020f;

// Position coordinate system: 0 (Far Left) .. 3500 (Center) .. 7000 (Far Right)
int16_t position = 3500;
int16_t lastPosition = 3500;
float filteredPosition = 3500.0f;
const float POSITION_ALPHA = 0.88f;

bool dataStreamActive = false;

uint32_t lastPosSendMs = 0;     // 10 Hz (every 100 ms)
uint32_t lastAllDataSendMs = 0; // 5 Hz  (every 200 ms)

int16_t ultrasonicDistanceCm = -1;

// Exact Hardware-Applied Motor States
const char* actualDirLeft  = "STP";
const char* actualDirRight = "STP";
int16_t actualPwmLeft  = 0;
int16_t actualPwmRight = 0;

// PID Telemetry Variables
float integral = 0.0f;
float lastError = 0.0f;
float derivativeFiltered = 0.0f;
float pidPTerm = 0.0f;
float pidITerm = 0.0f;
float pidDTerm = 0.0f;
int16_t pidErrorOut = 0;
int16_t pidOutputOut = 0;

/* ============================================================
THRESHOLDS & TIMINGS
============================================================ */

const uint16_t ACTIVE_THR       = 280;
const uint16_t STRONG_THR       = 620;
const uint16_t MIN_LINE_SIGNAL  = 160;
const uint16_t MIN_CAL_SPAN     = 120;

const uint16_t AUTO_DETECT_MIN_GAP         = 250;
const uint16_t AUTO_DETECT_MAX_PAIR_SPREAD = 350;

const uint8_t JUNCTION_CONFIRM  = 8;
const uint8_t FINISH_CONFIRM    = 9;
const uint8_t CHECKER_CONFIRM   = 8;
const uint8_t REACQUIRE_CONFIRM = 4;

const uint16_t PROBE_MS         = 45;
const uint16_t ALIGN_MS         = 30;
const uint16_t PIVOT_TIMEOUT_MS = 1400;
const uint16_t UTURN_TIMEOUT_MS = 2800;
const uint16_t JUNC_COOLDOWN_MS = 220;

const int16_t  REACQUIRE_WINDOW = 1100;

/* ============================================================
SENSOR MASKS
============================================================ */

const uint8_t LEFT_MASK     = (1 << 0) | (1 << 1);
const uint8_t RIGHT_MASK    = (1 << 6) | (1 << 7);
const uint8_t CENTER_MASK   = (1 << 3) | (1 << 4);
const uint8_t STRAIGHT_MASK = (1 << 2) | (1 << 3) | (1 << 4) | (1 << 5);

/* ============================================================
STATE MACHINE
============================================================ */

enum State : uint8_t {
  ST_FOLLOW,
  ST_JUNC_PROBE,
  ST_JUNC_ALIGN,
  ST_PIVOT,
  ST_CHECKER,
  ST_FINISH_BOX,
  ST_LOST,
  ST_STOP,
  ST_IDLE
};

enum Turn : uint8_t {
  T_LEFT = 0,
  T_STRAIGHT = 1,
  T_RIGHT = 2,
  T_BACK = 3
};

enum LostPhase : uint8_t {
  LP_COAST,
  LP_SEARCH1,
  LP_SEARCH2,
  LP_UTURN,
  LP_CAPTURE
};

enum CalStatus : uint8_t {
  CAL_NONE,
  CAL_RUNNING,
  CAL_OK,
  CAL_LOW
};

enum BgColor : uint8_t {
  BG_UNKNOWN,
  BG_WHITE,
  BG_BLACK
};

struct IrFilter {
  bool stableBlack;
  bool valid;
  bool candidate;
  uint8_t count;
};

State state = ST_IDLE;
LostPhase lostPhase = LP_COAST;
CalStatus calStatus = CAL_NONE;

uint8_t mask = 0;
uint8_t activeCount = 0;
uint8_t strongCount = 0;
uint8_t runs = 0;
bool lineVisible = false;

bool jSeenL = false;
bool jSeenR = false;
bool jChecker = false;
Turn pendingTurn = T_STRAIGHT;
uint8_t junctionCounter = 0;
uint8_t checkerCounter = 0;
uint8_t finishCounter = 0;
uint32_t junctionCooldownStart = 0;

uint32_t stateTimer = 0;
int8_t pivotDir = 0;
uint8_t pivotPhase = 0;
uint8_t pivotClearCount = 0;
uint8_t pivotCaptureCount = 0;

int8_t lostDirection = 0;
int8_t captureDirection = -1;
uint8_t lostCaptureCount = 0;

// Encoder counters
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;

// Background IR
IrFilter irLeft  = { false, false, false, 0 };
IrFilter irRight = { false, false, false, 0 };
BgColor bgColor = BG_UNKNOWN;

/* ============================================================
FORWARD DECLARATIONS
============================================================ */

void setMotors(int left, int right);
void stopMotors();
void resumeFollow();
void startLost();
void printCalibrationReport();
void updateUltrasonicNonBlocking();
const char* getStateString(State s);
void transitionToState(State newState);
void handleDataStreaming();
void isrLeftEncoder();
void isrRightEncoder();
void waitForConfigValues();
void setupService();
void applySpeedScale();

/* ============================================================
GLOBAL SPEED MULTIPLIER APPLICATION
============================================================ */

void applySpeedScale() {
  if (speedScale <= 0.0f) speedScale = 1.0f;

  speedStraight      = constrain((int)(baseSpeedStraight * speedScale), 0, 255);
  speedGentleCurve   = constrain((int)(baseSpeedGentleCurve * speedScale), 0, 255);
  speedMediumCurve   = constrain((int)(baseSpeedMediumCurve * speedScale), 0, 255);
  speedSharpCurve    = constrain((int)(baseSpeedSharpCurve * speedScale), 0, 255);
  speedMinBase       = constrain((int)(baseSpeedMinBase * speedScale), 0, 255);
  probeSpeed         = constrain((int)(baseProbeSpeed * speedScale), 0, 255);
  alignSpeed         = constrain((int)(baseAlignSpeed * speedScale), 0, 255);
  pivotSpeed         = constrain((int)(basePivotSpeed * speedScale), 0, 255);
  pivotCaptureSpeed  = constrain((int)(basePivotCaptureSpeed * speedScale), 0, 255);
  searchSpeed        = constrain((int)(baseSearchSpeed * speedScale), 0, 255);
  uturnSpeed         = constrain((int)(baseUturnSpeed * speedScale), 0, 255);
  currentBaseSpeed   = speedMinBase;
}

/* ============================================================
STATE TRANSITION LOGGER (ACTION BLOCK)
============================================================ */

const char* getStateString(State s) {
  switch (s) {
    case ST_FOLLOW:     return "FOLLOW";
    case ST_JUNC_PROBE: return "JPROBE";
    case ST_JUNC_ALIGN: return "JALIGN";
    case ST_PIVOT:      return "PIVOT";
    case ST_CHECKER:    return "CHECKER";
    case ST_FINISH_BOX: return "FINISH";
    case ST_LOST:       return "LOST";
    case ST_STOP:       return "STOP";
    case ST_IDLE:       return "IDLE";
    default:            return "UNKNOWN";
  }
}

void transitionToState(State newState) {
  if (state != newState) {
#if DEBUG_SERIAL
    Serial.print(F("[ACTION] State: "));
    Serial.print(getStateString(state));
    Serial.print(F(" -> "));
    Serial.println(getStateString(newState));

    Serial3.print(F("[ACTION] State: "));
    Serial3.print(getStateString(state));
    Serial3.print(F(" -> "));
    Serial3.println(getStateString(newState));
#endif
    state = newState;
    stateTimer = millis();
  }
}

/* ============================================================
MOTOR DRIVER (HARDWARE DELIVERY TRACKING)
============================================================ */

void setMotor(uint8_t pwmPin, uint8_t in1Pin, uint8_t in2Pin, int speed, bool invert,
              const char* &outDir, int16_t &outPwm) {
  if (invert) speed = -speed;
  speed = constrain(speed, -255, 255);

  if (speed > 0) {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
    analogWrite(pwmPin, speed);
    outDir = "FWD";
    outPwm = speed; // Exact hardware PWM provided
  } else if (speed < 0) {
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
    analogWrite(pwmPin, -speed);
    outDir = "REV";
    outPwm = -speed; // Exact hardware PWM provided
  } else {
    // Active Brake
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, HIGH);
    analogWrite(pwmPin, 255);
    outDir = "STP";
    outPwm = 0;
  }
}

void setMotors(int left, int right) {
  setMotor(LEFT_PWM, LEFT_IN1, LEFT_IN2, left, leftInvert, actualDirLeft, actualPwmLeft);
  setMotor(RIGHT_PWM, RIGHT_IN1, RIGHT_IN2, right, rightInvert, actualDirRight, actualPwmRight);
}

void driveStraight(int speed) {
  setMotors(speed, speed);
}

void spin(int8_t direction, int speed) {
  setMotors(direction * speed, -direction * speed);
}

void stopMotors() {
  setMotors(0, 0);
}

/* ============================================================
ENCODERS (TEST CODE IMPLEMENTATION)
============================================================ */

void isrLeftEncoder() {
  if (encLeftInvert) {
    if (digitalRead(ENC_L_B) == HIGH) leftEncoderCount--;
    else                              leftEncoderCount++;
  } else {
    if (digitalRead(ENC_L_B) == HIGH) leftEncoderCount++;
    else                              leftEncoderCount--;
  }
}

void isrRightEncoder() {
  if (encRightInvert) {
    if (digitalRead(ENC_R_B) == HIGH) rightEncoderCount--;
    else                              rightEncoderCount++;
  } else {
    if (digitalRead(ENC_R_B) == HIGH) rightEncoderCount++;
    else                              rightEncoderCount--;
  }
}

void initEncoders() {
  pinMode(ENC_L_A, INPUT_PULLUP);
  pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP);
  pinMode(ENC_R_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_L_A), isrLeftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), isrRightEncoder, RISING);
}

void readEncoders(long &left, long &right) {
  uint8_t sreg = SREG;
  cli();
  left  = leftEncoderCount;
  right = rightEncoderCount;
  SREG = sreg;
}

void resetEncoders() {
  uint8_t sreg = SREG;
  cli();
  leftEncoderCount = 0;
  rightEncoderCount = 0;
  SREG = sreg;
}

/* ============================================================
100% NON-BLOCKING ULTRASONIC STATE MACHINE (NO pulseIn)
============================================================ */

enum UsMachineState : uint8_t { US_IDLE, US_TRIG_HIGH, US_WAIT_RISE, US_WAIT_FALL };

void updateUltrasonicNonBlocking() {
  static UsMachineState usState = US_IDLE;
  static uint32_t usTimer = 0;
  static uint32_t usTrigStart = 0;
  static uint32_t usEchoStart = 0;

  uint32_t nowMs = millis();
  uint32_t nowUs = micros();

  switch (usState) {
    case US_IDLE:
      if (nowMs - usTimer >= 50) {
        usTimer = nowMs;
        digitalWrite(US_TRIG, HIGH);
        usTrigStart = nowUs;
        usState = US_TRIG_HIGH;
      }
      break;

    case US_TRIG_HIGH:
      if (nowUs - usTrigStart >= 10) {
        digitalWrite(US_TRIG, LOW);
        usTrigStart = nowUs;
        usState = US_WAIT_RISE;
      }
      break;

    case US_WAIT_RISE:
      if (digitalRead(US_ECHO) == HIGH) {
        usEchoStart = nowUs;
        usState = US_WAIT_FALL;
      } else if (nowUs - usTrigStart > 4000) {
        ultrasonicDistanceCm = -1;
        usState = US_IDLE;
        usTimer = nowMs;
      }
      break;

    case US_WAIT_FALL:
      if (digitalRead(US_ECHO) == LOW) {
        uint32_t duration = nowUs - usEchoStart;
        float cm = (float)duration / 58.2f;
        if (cm < 2.0f && cm > 0.0f) cm = 2.0f;
        ultrasonicDistanceCm = (int16_t)(cm + 0.5f);
        usState = US_IDLE;
        usTimer = nowMs;
      } else if (nowUs - usEchoStart > 24000) {
        ultrasonicDistanceCm = -1;
        usState = US_IDLE;
        usTimer = nowMs;
      }
      break;
  }
}

/* ============================================================
BACKGROUND COLOUR (DIGITAL IR SENSORS)
============================================================ */

bool irReadBlack(uint8_t pin, bool blackIsHigh) {
  bool level = (digitalRead(pin) == HIGH);
  return blackIsHigh ? level : !level;
}

void irFilterUpdate(IrFilter &f, bool rawBlack) {
  if (f.valid && rawBlack == f.stableBlack) {
    f.count = 0;
    return;
  }
  if (f.count == 0 || rawBlack != f.candidate) {
    f.candidate = rawBlack;
    f.count = 1;
  } else if (f.count < 255) {
    f.count++;
  }
  if (f.count >= BG_CONFIRM_COUNT) {
    f.stableBlack = f.candidate;
    f.valid = true;
    f.count = 0;
  }
}

void updateBackground() {
  irFilterUpdate(irLeft,  irReadBlack(IR_LEFT_PIN,  IR_LEFT_BLACK_IS_HIGH));
  irFilterUpdate(irRight, irReadBlack(IR_RIGHT_PIN, IR_RIGHT_BLACK_IS_HIGH));

  if (irLeft.valid && irRight.valid && irLeft.stableBlack == irRight.stableBlack) {
    bgColor = irLeft.stableBlack ? BG_BLACK : BG_WHITE;
  }
}

/* ============================================================
STREAMING & TELEMETRY (10 Hz & 5 Hz)
============================================================ */

void handleDataStreaming() {
  if (!dataStreamActive) return;

  uint32_t nowMs = millis();

  // 1. POSITION PACKET: STRICTLY 10 Hz (every 100 ms)
  if (nowMs - lastPosSendMs >= 100) {
    lastPosSendMs = nowMs;

    char posBuf[32];
    snprintf(posBuf, sizeof(posBuf), "$POS,%d,%d", position, lineVisible ? 1 : 0);
    Serial3.println(posBuf);

#if DEBUG_SERIAL
    Serial.print(F("[POS] Pos: "));
    Serial.print(position);
    Serial.print(F(" | Vis: "));
    Serial.println(lineVisible ? 1 : 0);
#endif
  }

  // 2. COMPREHENSIVE TELEMETRY PACKET: STRICTLY 5 Hz (every 200 ms)
  // Format: $DAT,<encL>,<encR>,<irL>,<irR>,<usDist>,<dirL>,<pwmL>,<dirR>,<pwmR>,<err>,<pTerm>,<iTerm>,<dTerm>,<corr>,<state>
  if (nowMs - lastAllDataSendMs >= 200) {
    lastAllDataSendMs = nowMs;

    long encL = 0, encR = 0;
    readEncoders(encL, encR);

    bool isBlackL = irLeft.valid ? irLeft.stableBlack : irReadBlack(IR_LEFT_PIN, IR_LEFT_BLACK_IS_HIGH);
    bool isBlackR = irRight.valid ? irRight.stableBlack : irReadBlack(IR_RIGHT_PIN, IR_RIGHT_BLACK_IS_HIGH);

    const char* stateStr = getStateString(state);

    // Convert floats reliably using dtostrf to bypass AVR-libc %f snprintf omission
    char pStr[10], iStr[10], dStr[10];
    dtostrf(pidPTerm, 1, 2, pStr);
    dtostrf(pidITerm, 1, 2, iStr);
    dtostrf(pidDTerm, 1, 2, dStr);

    char datBuf[140];
    snprintf(datBuf, sizeof(datBuf),
             "$DAT,%ld,%ld,%d,%d,%d,%s,%d,%s,%d,%d,%s,%s,%s,%d,%s",
             encL, encR,
             isBlackL ? 1 : 0, isBlackR ? 1 : 0,
             ultrasonicDistanceCm,
             actualDirLeft, actualPwmLeft,
             actualDirRight, actualPwmRight,
             pidErrorOut,
             pStr, iStr, dStr,
             pidOutputOut,
             stateStr);

    Serial3.println(datBuf);

#if DEBUG_SERIAL
    Serial.print(F("[DAT] ENC: L="));
    Serial.print(encL);
    Serial.print(F(" R="));
    Serial.print(encR);
    Serial.print(F(" | IR: L="));
    Serial.print(isBlackL ? F("B") : F("W"));
    Serial.print(F(" R="));
    Serial.print(isBlackR ? F("B") : F("W"));
    Serial.print(F(" | US: "));
    if (ultrasonicDistanceCm >= 0) {
      Serial.print(ultrasonicDistanceCm);
      Serial.print(F("cm"));
    } else {
      Serial.print(F("CLEAR"));
    }
    Serial.print(F(" | MOT: L=["));
    Serial.print(actualDirLeft);
    Serial.print(':');
    Serial.print(actualPwmLeft);
    Serial.print(F("] R=["));
    Serial.print(actualDirRight);
    Serial.print(':');
    Serial.print(actualPwmRight);
    Serial.print(F("] | PID: [Err: "));
    Serial.print(pidErrorOut);
    Serial.print(F(" P: "));
    Serial.print(pStr);
    Serial.print(F(" I: "));
    Serial.print(iStr);
    Serial.print(F(" D: "));
    Serial.print(dStr);
    Serial.print(F(" Out: "));
    Serial.print(pidOutputOut);
    Serial.print(F("] | ST: "));
    Serial.println(stateStr);
#endif
  }
}

void setupService() {
  updateUltrasonicNonBlocking();
  updateBackground();
}

void setupWait(uint16_t ms) {
  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    setupService();
    delay(1);
  }
}

/* ============================================================
SENSOR PROCESSING
============================================================ */

uint8_t countRuns(uint8_t sensorMask) {
  uint8_t count = 0;
  bool previous = false;
  for (uint8_t i = 0; i < 8; i++) {
    bool current = sensorMask & (1 << i);
    if (current && !previous) count++;
    previous = current;
  }
  return count;
}

void analyzeSensors() {
  mask = 0;
  activeCount = 0;
  strongCount = 0;
  uint16_t maxSignal = 0;
  uint32_t totalSignal = 0;

  for (uint8_t i = 0; i < 8; i++) {
    uint8_t idx = sensorReversed ? (7 - i) : i;
    uint16_t val = sensorValues[idx];
    totalSignal += val;

    if (val > maxSignal) maxSignal = val;
    if (val >= ACTIVE_THR) {
      mask |= (1 << i);
      activeCount++;
    }
    if (val >= STRONG_THR) {
      strongCount++;
    }
  }

  runs = countRuns(mask);
  lineVisible = (maxSignal >= MIN_LINE_SIGNAL) && (activeCount > 0 || totalSignal >= 450);
}

void readQTRLine() {
  uint16_t rawPos;

  if (isWhiteLine) {
    rawPos = qtr.readLineWhite(sensorValues);
  } else {
    rawPos = qtr.readLineBlack(sensorValues);
  }

  if (sensorReversed) {
    rawPos = 7000 - rawPos;
  }

  analyzeSensors();

  if (lineVisible) {
    filteredPosition = (POSITION_ALPHA * rawPos) + ((1.0f - POSITION_ALPHA) * filteredPosition);
    position = (int16_t)(filteredPosition + 0.5f);
    lastPosition = position;
  } else {
    position = lastPosition;
  }
}

/* ============================================================
TRACK AUTO-DETECTION & CALIBRATION
============================================================ */

bool detectTrackType() {
  if (trackMode == TRACK_WHITE_ON_BLACK) {
    isWhiteLine = true;
#if DEBUG_SERIAL
    Serial.println(F("[TRACK] Surface Mode: WHITE Line on BLACK Background"));
    Serial3.println(F("[TRACK] Surface Mode: WHITE Line on BLACK Background"));
#endif
    return true;
  } else if (trackMode == TRACK_BLACK_ON_WHITE) {
    isWhiteLine = false;
#if DEBUG_SERIAL
    Serial.println(F("[TRACK] Surface Mode: BLACK Line on WHITE Background"));
    Serial3.println(F("[TRACK] Surface Mode: BLACK Line on WHITE Background"));
#endif
    return true;
  } else {
    qtr.readCalibrated(sensorValues);

    uint16_t c0 = sensorValues[3];
    uint16_t c1 = sensorValues[4];
    uint16_t e0 = sensorValues[0];
    uint16_t e1 = sensorValues[7];

    uint16_t centerAverage = (c0 + c1) / 2;
    uint16_t edgeAverage   = (e0 + e1) / 2;

    uint16_t centerSpread = (c0 > c1) ? (c0 - c1) : (c1 - c0);
    uint16_t edgeSpread   = (e0 > e1) ? (e0 - e1) : (e1 - e0);

    bool confident = false;

    if (centerSpread <= AUTO_DETECT_MAX_PAIR_SPREAD && edgeSpread <= AUTO_DETECT_MAX_PAIR_SPREAD) {
      if (centerAverage + AUTO_DETECT_MIN_GAP <= edgeAverage) {
        isWhiteLine = true;
        confident = true;
      } else if (edgeAverage + AUTO_DETECT_MIN_GAP <= centerAverage) {
        isWhiteLine = false;
        confident = true;
      }
    }

    if (!confident && edgeSpread <= AUTO_DETECT_MAX_PAIR_SPREAD) {
      uint16_t innerMin1 = 1000, innerMin2 = 1000;
      uint16_t innerMax1 = 0,    innerMax2 = 0;
      for (uint8_t i = 2; i <= 5; i++) {
        uint16_t v = sensorValues[i];
        if (v < innerMin1) { innerMin2 = innerMin1; innerMin1 = v; }
        else if (v < innerMin2) { innerMin2 = v; }
        if (v > innerMax1) { innerMax2 = innerMax1; innerMax1 = v; }
        else if (v > innerMax2) { innerMax2 = v; }
      }
      uint16_t brightAvg   = (innerMin1 + innerMin2) / 2;
      uint16_t brightSprd  = innerMin2 - innerMin1;
      uint16_t darkAvg     = (innerMax1 + innerMax2) / 2;
      uint16_t darkSprd    = innerMax1 - innerMax2;

      if (brightSprd <= AUTO_DETECT_MAX_PAIR_SPREAD &&
          brightAvg + AUTO_DETECT_MIN_GAP <= edgeAverage) {
        isWhiteLine = true;
        confident = true;
      } else if (darkSprd <= AUTO_DETECT_MAX_PAIR_SPREAD &&
                 edgeAverage + AUTO_DETECT_MIN_GAP <= darkAvg) {
        isWhiteLine = false;
        confident = true;
      }
    }

    if (!confident) {
#if DEBUG_SERIAL
      Serial.println(F("[TRACK] AUTO DETECT UNCERTAIN!"));
      Serial3.println(F("[TRACK] AUTO DETECT UNCERTAIN!"));
#endif
      return false;
    }

#if DEBUG_SERIAL
    if (isWhiteLine) {
      Serial.println(F("[TRACK] Surface Mode: WHITE Line on BLACK Background"));
      Serial3.println(F("[TRACK] Surface Mode: WHITE Line on BLACK Background"));
    } else {
      Serial.println(F("[TRACK] Surface Mode: BLACK Line on WHITE Background"));
      Serial3.println(F("[TRACK] Surface Mode: BLACK Line on WHITE Background"));
    }
#endif

    return true;
  }
}

bool checkCalibration() {
  bool valid = true;
  for (uint8_t i = 0; i < 8; i++) {
    uint16_t minVal = qtr.calibrationOn.minimum[i];
    uint16_t maxVal = qtr.calibrationOn.maximum[i];
    uint16_t span = (maxVal > minVal) ? (maxVal - minVal) : 0;
    if (span < MIN_CAL_SPAN) valid = false;
  }
  return valid;
}

void printCalibrationReport() {
#if DEBUG_SERIAL
  Serial.println(F("\n========================================================"));
  Serial.println(F("              QTR SENSOR CALIBRATION REPORT             "));
  Serial.println(F("========================================================"));
  Serial.println(F(" Sensor |  Min Raw  |  Max Raw  |   Span   |   Status   "));
  Serial.println(F("--------------------------------------------------------"));
  Serial3.println(F("\n========================================================"));
  Serial3.println(F("              QTR SENSOR CALIBRATION REPORT             "));
  Serial3.println(F("========================================================"));
  Serial3.println(F(" Sensor |  Min Raw  |  Max Raw  |   Span   |   Status   "));
  Serial3.println(F("--------------------------------------------------------"));

  uint32_t leftSpanSum = 0;
  uint32_t rightSpanSum = 0;
  bool allPassed = true;

  for (uint8_t i = 0; i < 8; i++) {
    uint16_t minVal = qtr.calibrationOn.minimum[i];
    uint16_t maxVal = qtr.calibrationOn.maximum[i];
    uint16_t span = (maxVal > minVal) ? (maxVal - minVal) : 0;

    if (i < 4) leftSpanSum += span;
    else rightSpanSum += span;

    bool passed = (span >= MIN_CAL_SPAN);
    if (!passed) allPassed = false;

    Serial.print(F(" S"));
    Serial3.print(F(" S"));
    Serial.print(i);
    Serial3.print(i);
    if (i == 0) { Serial.print(F(" (Far L)|   ")); Serial3.print(F(" (Far L)|   ")); }
    else if (i == 7) { Serial.print(F(" (Far R)|   ")); Serial3.print(F(" (Far R)|   ")); }
    else { Serial.print(F("        |   ")); Serial3.print(F("        |   ")); }

    if (minVal < 1000) { Serial.print(' '); Serial3.print(' '); }
    if (minVal < 100)  { Serial.print(' '); Serial3.print(' '); }
    Serial.print(minVal);
    Serial3.print(minVal);

    Serial.print(F("   |   "));
    Serial3.print(F("   |   "));
    if (maxVal < 1000) { Serial.print(' '); Serial3.print(' '); }
    if (maxVal < 100)  { Serial.print(' '); Serial3.print(' '); }
    Serial.print(maxVal);
    Serial3.print(maxVal);

    Serial.print(F("   |   "));
    Serial3.print(F("   |   "));
    if (span < 1000) { Serial.print(' '); Serial3.print(' '); }
    if (span < 100)  { Serial.print(' '); Serial3.print(' '); }
    Serial.print(span);
    Serial3.print(span);

    Serial.print(F("   |   "));
    Serial3.print(F("   |   "));
    if (passed) {
      Serial.println(F("OK"));
      Serial3.println(F("OK"));
    } else {
      Serial.println(F("WARN: LOW"));
      Serial3.println(F("WARN: LOW"));
    }
  }

  uint16_t leftAvg = leftSpanSum / 4;
  uint16_t rightAvg = rightSpanSum / 4;

  Serial.println(F("--------------------------------------------------------"));
  Serial.print(F(" Left Side Avg Span (S0-S3):  "));
  Serial.print(leftAvg);
  Serial.println(leftAvg >= MIN_CAL_SPAN ? F(" (OK)") : F(" (LOW)"));

  Serial.print(F(" Right Side Avg Span (S4-S7): "));
  Serial.print(rightAvg);
  Serial.println(rightAvg >= MIN_CAL_SPAN ? F(" (OK)") : F(" (LOW)"));

  Serial3.println(F("--------------------------------------------------------"));
  Serial3.print(F(" Left Side Avg Span (S0-S3):  "));
  Serial3.print(leftAvg);
  Serial3.println(leftAvg >= MIN_CAL_SPAN ? F(" (OK)") : F(" (LOW)"));

  Serial3.print(F(" Right Side Avg Span (S4-S7): "));
  Serial3.print(rightAvg);
  Serial3.println(rightAvg >= MIN_CAL_SPAN ? F(" (OK)") : F(" (LOW)"));

  if (allPassed && leftAvg >= MIN_CAL_SPAN && rightAvg >= MIN_CAL_SPAN) {
    Serial.println(F(" [CAL RESULT] SUCCESS: All sensors calibrated both sides!"));
    Serial3.println(F(" [CAL RESULT] SUCCESS: All sensors calibrated both sides!"));
  } else {
    Serial.println(F(" [CAL RESULT] WARNING: One or more sensors have low span!"));
    Serial3.println(F(" [CAL RESULT] WARNING: One or more sensors have low span!"));
  }
  Serial.println(F("========================================================\n"));
  Serial3.println(F("========================================================\n"));
#endif
}

void calibrateQTR() {
  calStatus = CAL_RUNNING;

#if DEBUG_SERIAL
  Serial.println(F("[CAL] Calibrating QTR Sensors..."));
  Serial3.println(F("[CAL] Calibrating QTR Sensors..."));
#endif

  if (autoCalSweep) {
    const uint16_t HALF_SWEEP_MS = 280;
    const uint16_t FULL_SWEEP_MS = 560;
    const uint8_t CAL_SPEED = 95;

    const uint16_t SWEEP_STAGES[] = {
      HALF_SWEEP_MS,
      (uint16_t)(HALF_SWEEP_MS + FULL_SWEEP_MS),
      (uint16_t)(HALF_SWEEP_MS + 2 * FULL_SWEEP_MS),
      (uint16_t)(HALF_SWEEP_MS + 3 * FULL_SWEEP_MS),
      (uint16_t)(HALF_SWEEP_MS + 4 * FULL_SWEEP_MS),
      (uint16_t)(HALF_SWEEP_MS + 5 * FULL_SWEEP_MS),
      (uint16_t)(2 * HALF_SWEEP_MS + 5 * FULL_SWEEP_MS)
    };
    const uint8_t NUM_STAGES = sizeof(SWEEP_STAGES) / sizeof(SWEEP_STAGES[0]);
    const int8_t STAGE_DIRS[] = { 1, -1, 1, -1, 1, -1, 1 };

    uint32_t calStart = millis();
    uint8_t currentStage = 0;

    while (currentStage < NUM_STAGES) {
      uint32_t elapsed = millis() - calStart;
      if (elapsed >= SWEEP_STAGES[currentStage]) {
        currentStage++;
        if (currentStage >= NUM_STAGES) break;
      }
      spin(STAGE_DIRS[currentStage], CAL_SPEED);
      qtr.calibrate();
      setupService();
    }
  } else {
    uint32_t calStart = millis();
    stopMotors();
    while (millis() - calStart < 4000) {
      qtr.calibrate();
      setupService();
      delay(4);
    }
  }

  stopMotors();
  setupWait(150);

  calStatus = checkCalibration() ? CAL_OK : CAL_LOW;
  printCalibrationReport();
}

/* ============================================================
PID FOLLOW
============================================================ */

void resetPID() {
  integral = 0.0f;
  derivativeFiltered = 0.0f;
  lastError = (float)position - 3500.0f;
  currentBaseSpeed = speedMinBase;
}

void followPID() {
  if (!lineVisible) return;

  float rawError = (float)position - 3500.0f;
  float error = rawError;

  if (fabs(error) < errorDeadband) {
    error = 0.0f;
  }

  if (fabs(error) > errorDeadband && fabs(error) < integralActiveZone) {
    integral += error * dt;
    integral = constrain(integral, -maxIntegral, maxIntegral);
  }

  if ((error > 0.0f && lastError < 0.0f) || (error < 0.0f && lastError > 0.0f)) {
    integral = 0.0f;
  }

  float derivative = (error - lastError) / dt;
  derivativeFiltered = (dFilterAlpha * derivative) + ((1.0f - dFilterAlpha) * derivativeFiltered);
  lastError = error;

  float absErr = fabs(error);
  float pTerm = Kp * error;

  if (absErr > 1500.0f) {
    float excess = absErr - 1500.0f;
    float boost = excess * excess * progressiveGain;
    if (error > 0.0f) pTerm += boost;
    else              pTerm -= boost;
  }

  float iTerm = Ki * integral;
  float dTerm = Kd * derivativeFiltered;

  float correction = pTerm + iTerm + dTerm;
  correction = constrain(correction, -maxCorrection, maxCorrection);

  // Store terms for telemetry
  pidPTerm = pTerm;
  pidITerm = iTerm;
  pidDTerm = dTerm;
  pidErrorOut  = (int16_t)error;
  pidOutputOut = (int16_t)correction;

  int targetSpeed;
  if (absErr > 2200.0f) {
    targetSpeed = speedSharpCurve;
  } else if (absErr > 1400.0f) {
    float factor = (absErr - 1400.0f) / 800.0f;
    targetSpeed = (int)(speedMediumCurve - factor * (speedMediumCurve - speedSharpCurve));
  } else if (absErr > 600.0f) {
    float factor = (absErr - 600.0f) / 800.0f;
    targetSpeed = (int)(speedGentleCurve - factor * (speedGentleCurve - speedMediumCurve));
  } else if (absErr > errorDeadband) {
    float factor = (absErr - errorDeadband) / (600.0f - errorDeadband);
    targetSpeed = (int)(speedStraight - factor * (speedStraight - speedGentleCurve));
  } else {
    targetSpeed = speedStraight;
  }

  if (fabs(derivativeFiltered) > 35000.0f) {
    targetSpeed = min(targetSpeed, (int)(speedSharpCurve + 15));
  }

  if (currentBaseSpeed < targetSpeed) {
    currentBaseSpeed += accelRate * dt;
    if (currentBaseSpeed > targetSpeed) currentBaseSpeed = targetSpeed;
  } else if (currentBaseSpeed > targetSpeed) {
    currentBaseSpeed -= decelRate * dt;
    if (currentBaseSpeed < targetSpeed) currentBaseSpeed = targetSpeed;
  }

  int leftSpd  = (int)(currentBaseSpeed + correction);
  int rightSpd = (int)(currentBaseSpeed - correction);

  leftSpd  = constrain(leftSpd,  MOTOR_MAX_REVERSE, MOTOR_MAX_FORWARD);
  rightSpd = constrain(rightSpd, MOTOR_MAX_REVERSE, MOTOR_MAX_FORWARD);

  setMotors(leftSpd, rightSpd);
}

/* ============================================================
JUNCTION RESOLUTION
============================================================ */

Turn chooseTurn(bool left, bool straight, bool right) {
  Turn pref[3];

  if (junctionPolicy == POLICY_LEFT) {
    pref[0] = T_LEFT;     pref[1] = T_STRAIGHT; pref[2] = T_RIGHT;
  } else if (junctionPolicy == POLICY_RIGHT) {
    pref[0] = T_RIGHT;    pref[1] = T_STRAIGHT; pref[2] = T_LEFT;
  } else {
    pref[0] = T_STRAIGHT; pref[1] = T_LEFT;     pref[2] = T_RIGHT;
  }

  for (uint8_t i = 0; i < 3; i++) {
    if (pref[i] == T_LEFT && left)         return T_LEFT;
    if (pref[i] == T_STRAIGHT && straight) return T_STRAIGHT;
    if (pref[i] == T_RIGHT && right)       return T_RIGHT;
  }

  return T_BACK;
}

void startJunction() {
  transitionToState(ST_JUNC_PROBE);
  jSeenL = (mask & LEFT_MASK);
  jSeenR = (mask & RIGHT_MASK);
  jChecker = false;
}

void startPivot(Turn turn) {
  if (turn == T_LEFT) {
    pivotDir = -1;
  } else if (turn == T_RIGHT) {
    pivotDir = 1;
  } else {
    pivotDir = uturnDir;
  }

  pivotPhase = 0;
  pivotClearCount = 0;
  pivotCaptureCount = 0;
  transitionToState(ST_PIVOT);
}

void evaluateJunction() {
  if (jChecker) {
    transitionToState(ST_CHECKER);
    return;
  }

  if (strongCount >= 7) {
    transitionToState(ST_FINISH_BOX);
    return;
  }

  bool straightStillExists = (mask & STRAIGHT_MASK) && lineVisible;
  Turn selected = chooseTurn(jSeenL, straightStillExists, jSeenR);

  if (selected == T_STRAIGHT) {
    resumeFollow();
  } else {
    pendingTurn = selected;
    transitionToState(ST_JUNC_ALIGN);
  }
}

/* ============================================================
LOST-LINE RECOVERY
============================================================ */

bool isLineCentered() {
  if (!lineVisible) return false;
  bool centerSensors = (mask & (CENTER_MASK | (1 << 2) | (1 << 5)));
  return centerSensors && (abs(position - 3500) <= REACQUIRE_WINDOW);
}

void beginLostCapture(int8_t direction) {
  lostPhase = LP_CAPTURE;
  captureDirection = direction;
  lostCaptureCount = 0;
  stateTimer = millis();
}

void startLost() {
  transitionToState(ST_LOST);
  lostCaptureCount = 0;

  if (lastPosition < 2800) {
    lostDirection = -1;
  } else if (lastPosition > 4200) {
    lostDirection = 1;
  } else {
    lostDirection = uturnDir;
  }

  lostPhase = (abs(lastPosition - 3500) < 800) ? LP_COAST : LP_SEARCH1;
}

void runLost() {
  uint32_t elapsed = millis() - stateTimer;

  switch (lostPhase) {
    case LP_COAST:
      driveStraight(65);
      if (isLineCentered()) {
        resumeFollow();
      } else if (elapsed > 40) {
        lostPhase = LP_SEARCH1;
        stateTimer = millis();
      }
      break;

    case LP_SEARCH1:
      spin(lostDirection, searchSpeed);
      if (isLineCentered()) {
        beginLostCapture(-lostDirection);
      } else if (elapsed > 260) {
        lostPhase = LP_SEARCH2;
        stateTimer = millis();
      }
      break;

    case LP_SEARCH2:
      spin(-lostDirection, searchSpeed);
      if (isLineCentered()) {
        beginLostCapture(lostDirection);
      } else if (elapsed > 520) {
        lostPhase = LP_UTURN;
        stateTimer = millis();
      }
      break;

    case LP_UTURN:
      spin(uturnDir, uturnSpeed);
      if (elapsed > 350 && isLineCentered()) {
        beginLostCapture(-uturnDir);
      } else if (elapsed > UTURN_TIMEOUT_MS) {
        transitionToState(ST_STOP);
      }
      break;

    case LP_CAPTURE:
      if (isLineCentered()) {
        stopMotors();
        if (++lostCaptureCount >= REACQUIRE_CONFIRM) {
          resumeFollow();
          return;
        }
      } else {
        lostCaptureCount = 0;
        spin(captureDirection, 60);
      }

      if (elapsed > 400) {
        lostCaptureCount = 0;
        lostPhase = LP_SEARCH1;
        stateTimer = millis();
      }
      break;
  }
}

/* ============================================================
SUPERVISOR LOOP & ACTION BLOCK
============================================================ */

void processFollowState() {
  static uint32_t lostSince = 0;

  if (strongCount >= 7) {
    if (++finishCounter >= FINISH_CONFIRM) {
      lostSince = 0;
      transitionToState(ST_FINISH_BOX);
      return;
    }
  } else {
    finishCounter = 0;
  }

  bool leftBranch  = (mask & LEFT_MASK);
  bool rightBranch = (mask & RIGHT_MASK);
  bool centerTrack = (mask & CENTER_MASK);

  bool cooldownElapsed = (uint32_t)(millis() - junctionCooldownStart) >= JUNC_COOLDOWN_MS;

  bool branchCandidate = cooldownElapsed &&
    (((leftBranch || rightBranch) && centerTrack && activeCount >= 4) ||
     (runs >= 2 && activeCount >= 4) ||
     (activeCount >= 6));

  if (branchCandidate) {
    if (++junctionCounter >= JUNCTION_CONFIRM) {
      lostSince = 0;
      startJunction();
      return;
    }
  } else {
    junctionCounter = 0;
  }

  if (runs >= 2 && activeCount >= 4 && !leftBranch && !rightBranch) {
    if (++checkerCounter >= CHECKER_CONFIRM) {
      lostSince = 0;
      transitionToState(ST_CHECKER);
      return;
    }
  } else {
    checkerCounter = 0;
  }

  if (!lineVisible) {
    if (lostSince == 0) lostSince = millis();
    stopMotors();
    if ((uint32_t)(millis() - lostSince) > 25) {
      lostSince = 0;
      startLost();
      return;
    }
  } else {
    lostSince = 0;
  }

  followPID();
}

void resumeFollow() {
  transitionToState(ST_FOLLOW);
  resetPID();
  junctionCounter = 0;
  checkerCounter = 0;
  finishCounter = 0;
  lostCaptureCount = 0;
  junctionCooldownStart = millis();
}

/* ============================================================
SETUP & CONFIG RECEIVER
============================================================ */

void waitForConfigValues() {
  Serial.println(F("[CFG] Waiting for Single-Line Configuration from Serial3..."));
  Serial3.println(F("[CFG] Send $CFG,Kp,Ki,Kd,speedScale,spdStr,spdGnt,spdMed,spdShp,spdMin,spdPrb,spdAln,spdPiv,spdSch,trkMod,jncPol,stpFin,autSt,utrnDir,motLInv,motRInv,encLInv,encRInv"));

  bool received = false;
  while (!received) {
    setupService();

    if (digitalRead(BTN) == LOW) {
      applySpeedScale();
      Serial.println(F("[CFG] Bypassed by D12 button. Using defaults."));
      Serial3.println(F("[CFG] Bypassed by D12 button. Using defaults."));
      break;
    }

    if (Serial3.available()) {
      String line = Serial3.readStringUntil('\n');
      line.trim();

      if (line.length() > 0) {
        Serial.print(F("[CFG] Received: "));
        Serial.println(line);

        String data = line;
        if (data.startsWith(F("$CFG,")))         data = data.substring(5);
        else if (data.startsWith(F("CFG,")))     data = data.substring(4);
        else if (data.startsWith(F("$CONFIG,"))) data = data.substring(8);
        else if (data.startsWith(F("CONFIG,")))   data = data.substring(7);
        data.trim();

        int tokenIdx = 0;
        int startIdx = 0;

        while (startIdx < (int)data.length()) {
          int commaIdx = data.indexOf(',', startIdx);
          String token;
          if (commaIdx == -1) {
            token = data.substring(startIdx);
            startIdx = data.length();
          } else {
            token = data.substring(startIdx, commaIdx);
            startIdx = commaIdx + 1;
          }
          token.trim();

          if (token.length() > 0) {
            switch (tokenIdx) {
              case 0: Kp = token.toFloat(); break;
              case 1: Ki = token.toFloat(); break;
              case 2: Kd = token.toFloat(); break;
              case 3: speedScale = token.toFloat(); break;
              case 4: baseSpeedStraight = token.toInt(); break;
              case 5: baseSpeedGentleCurve = token.toInt(); break;
              case 6: baseSpeedMediumCurve = token.toInt(); break;
              case 7: baseSpeedSharpCurve = token.toInt(); break;
              case 8: baseSpeedMinBase = token.toInt(); break;
              case 9: baseProbeSpeed = token.toInt(); break;
              case 10: baseAlignSpeed = token.toInt(); break;
              case 11: basePivotSpeed = token.toInt(); break;
              case 12: baseSearchSpeed = token.toInt(); break;
              case 13: trackMode = token.toInt(); isWhiteLine = (trackMode == TRACK_WHITE_ON_BLACK); break;
              case 14: junctionPolicy = token.toInt(); break;
              case 15: stopAtFinish = token.toInt(); break;
              case 16: autoStart = token.toInt(); break;
              case 17: uturnDir = token.toInt(); break;
              case 18: leftInvert = token.toInt(); break;
              case 19: rightInvert = token.toInt(); break;
              case 20: encLeftInvert = token.toInt(); break;
              case 21: encRightInvert = token.toInt(); break;
            }
            tokenIdx++;
          }
        }

        if (tokenIdx >= 3) {
          applySpeedScale();
          received = true;

          Serial.println(F("[CFG] Configuration successfully loaded:"));
          Serial.print(F(" PID: Kp=")); Serial.print(Kp, 6);
          Serial.print(F(" Ki=")); Serial.print(Ki, 6);
          Serial.print(F(" Kd=")); Serial.println(Kd, 6);
          Serial.print(F(" Speed Scale=")); Serial.print(speedScale, 2);
          Serial.print(F(" | Scaled Straight=")); Serial.print(speedStraight);
          Serial.print(F(" | Scaled Min=")); Serial.println(speedMinBase);
          Serial3.println(F("[CFG] CONFIG_OK"));
          break;
        } else {
          Serial.println(F("[CFG] Warning: Less than 3 parameters received."));
          Serial3.println(F("[CFG] ERROR: Insufficient parameters."));
        }
      }
    }
    delay(5);
  }
}

void waitForButton() {
  while (digitalRead(BTN) == HIGH) {
    setupService();
    delay(10);
  }
  delay(50);
  while (digitalRead(BTN) == LOW) {
    setupService();
    delay(10);
  }
  delay(50);
}

void setup() {
  dataStreamActive = false;

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

#if DEBUG_SERIAL
  Serial.begin(115200);
  Serial3.begin(115200);
  Serial3.setTimeout(50);

  Serial.println(F("[WIFI] Waiting for ESP 'CONNECTED' message (Press D12 to skip)..."));
  Serial3.println(F("[WIFI] Waiting for ESP 'CONNECTED' message..."));

  uint32_t espWaitStart = millis();
  while (true) {
    setupService();

    if (digitalRead(BTN) == LOW) {
      Serial.println(F("[WIFI] Bypassed by D12 button."));
      Serial3.println(F("[WIFI] Bypassed by D12 button."));
      break;
    }

    if (Serial3.available()) {
      String msg = Serial3.readStringUntil('\n');
      msg.trim();
      Serial.println("ESP says: " + msg);
      if (msg.indexOf("CONNECTED") != -1) {
        break;
      }
    }

    if (millis() - espWaitStart > 8000) {
      Serial.println(F("[WIFI] Timeout waiting for ESP. Continuing..."));
      Serial3.println(F("[WIFI] Timeout waiting for ESP. Continuing..."));
      break;
    }
  }

  Serial.println(F("\n=== UNIVERSAL QTR LINE FOLLOWER (V3.9 MEGA 2560) ==="));
  Serial3.println(F("\n=== UNIVERSAL QTR LINE FOLLOWER (V3.9 MEGA 2560) ==="));
#endif

  waitForConfigValues();

  pinMode(LEFT_PWM, OUTPUT);
  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);

  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  pinMode(BTN, INPUT_PULLUP);
  stopMotors();

  initEncoders();

  pinMode(IR_LEFT_PIN, IR_PIN_MODE);
  pinMode(IR_RIGHT_PIN, IR_PIN_MODE);

  pinMode(US_TRIG, OUTPUT);
  digitalWrite(US_TRIG, LOW);
  pinMode(US_ECHO, INPUT);

  ADCSRA = (ADCSRA & 0xF8) | 0x05;

  qtr.setTypeAnalog();
  qtr.setSensorPins(QTR_PINS, 8);
  qtr.setEmitterPin(EMITTER_PIN);
  qtr.setSamplesPerSensor(8);

  setupWait(300);

  if (!autoStart) {
#if DEBUG_SERIAL
    Serial.println(F("[USER] Press D12 button to start calibration..."));
    Serial3.println(F("[USER] Press D12 button to start calibration..."));
#endif
    waitForButton();
  } else {
    setupWait(800);
  }

  calibrateQTR();

  if (calStatus != CAL_OK) {
    stopMotors();
    transitionToState(ST_STOP);
#if DEBUG_SERIAL
    Serial.println(F("[ERROR] Calibration failed!"));
    Serial3.println(F("[ERROR] Calibration failed!"));
    Serial.println(F("[ERROR] Robot will remain stopped."));
    Serial3.println(F("[ERROR] Robot will remain stopped."));
#endif
    while (true) {
      setupService();
      delay(1);
    }
  }

  if (!autoStart) {
#if DEBUG_SERIAL
    Serial.println(F("[USER] Place on line & press D12 to detect track and race..."));
    Serial3.println(F("[USER] Place on line & press D12 to detect track and race..."));
#endif
    waitForButton();

    digitalWrite(LED_BUILTIN, HIGH);
    uint32_t syncTime = millis();
    Serial.print(F("[SYNC] LED_ON at ms: ")); Serial.println(syncTime);
    Serial3.print(F("[SYNC] LED_ON,")); Serial3.println(syncTime);
  } else {
    digitalWrite(LED_BUILTIN, HIGH);
    uint32_t syncTime = millis();

    Serial.print(F("[SYNC] LED_ON at ms: ")); Serial.println(syncTime);
    Serial3.print(F("[SYNC] LED_ON,")); Serial3.println(syncTime);

#if DEBUG_SERIAL
    Serial.println(F("[AUTO] Starting in 2 seconds..."));
    Serial3.println(F("[AUTO] Starting in 2 seconds..."));
#endif
    setupWait(2000);
  }

  if (!detectTrackType()) {
    stopMotors();
    transitionToState(ST_STOP);
#if DEBUG_SERIAL
    Serial.println(F("[ERROR] Automatic track detection failed!"));
    Serial3.println(F("[ERROR] Automatic track detection failed!"));
    Serial.println(F("[ERROR] Robot will remain stopped."));
    Serial3.println(F("[ERROR] Robot will remain stopped."));
#endif
    while (true) {
      setupService();
      delay(1);
    }
  }

  readQTRLine();
  resetPID();
  resetEncoders();
  junctionCooldownStart = millis() - JUNC_COOLDOWN_MS;
  transitionToState(ST_FOLLOW);
  lastLoopMicros = micros();

  lastPosSendMs = millis();
  lastAllDataSendMs = millis();
  dataStreamActive = true;

#if DEBUG_SERIAL
  Serial.println(F("[SYSTEM] Racing started. Telemetry active at 10Hz/5Hz."));
  Serial3.println(F("[SYSTEM] Racing started. Telemetry active at 10Hz/5Hz."));
#endif
}

/* ============================================================
MAIN LOOP & ACTION BLOCK (500 Hz CONTROL RATE)
============================================================ */

void loop() {
  updateUltrasonicNonBlocking();

  uint32_t now = micros();
  if (now - lastLoopMicros < LOOP_INTERVAL_US) {
    updateUltrasonicNonBlocking();
    return;
  }
  dt = (now - lastLoopMicros) * 1e-6f;
  dt = constrain(dt, 0.0010f, 0.0100f);
  lastLoopMicros = now;

  readQTRLine();
  updateBackground();
  updateUltrasonicNonBlocking();

  // ================= ACTION BLOCK =================
  switch (state) {
    case ST_FOLLOW:
      processFollowState();
      break;

    case ST_JUNC_PROBE:
      driveStraight(probeSpeed);

      if (mask & LEFT_MASK) {
        jSeenL = true;
        jChecker = false;
      }

      if (mask & RIGHT_MASK) {
        jSeenR = true;
        jChecker = false;
      }

      if (runs >= 2 && activeCount >= 4 && !jSeenL && !jSeenR) {
        jChecker = true;
      }

      if (millis() - stateTimer >= PROBE_MS) {
        evaluateJunction();
      }
      break;

    case ST_JUNC_ALIGN:
      driveStraight(alignSpeed);
      if (millis() - stateTimer >= ALIGN_MS) {
        startPivot(pendingTurn);
      }
      break;

    case ST_PIVOT: {
      uint32_t elapsed = millis() - stateTimer;
      bool centerCleared = !(mask & CENTER_MASK);

      if (pivotPhase == 0) {
        spin(pivotDir, pivotSpeed);
        if (centerCleared) {
          if (++pivotClearCount >= 2) {
            pivotClearCount = 0;
            pivotPhase = 1;
          }
        } else {
          pivotClearCount = 0;
        }
      } else if (pivotPhase == 1) {
        spin(pivotDir, pivotSpeed);
        if (isLineCentered()) {
          pivotPhase = 2;
        }
      } else {
        spin(pivotDir, pivotCaptureSpeed);
        if (isLineCentered()) {
          if (++pivotCaptureCount >= REACQUIRE_CONFIRM) {
            pivotCaptureCount = 0;
            resumeFollow();
          }
        } else {
          pivotCaptureCount = 0;
        }
      }

      if (elapsed > PIVOT_TIMEOUT_MS && state == ST_PIVOT) {
        startLost();
      }
      break;
    }

    case ST_CHECKER: {
      static uint32_t checkerLostSince = 0;

      if (!lineVisible) {
        if (checkerLostSince == 0) {
          checkerLostSince = millis();
        }

        stopMotors();

        if ((uint32_t)(millis() - checkerLostSince) > 25) {
          checkerLostSince = 0;
          startLost();
        }
      } else {
        checkerLostSince = 0;
        followPID();

        if (runs == 1 && activeCount <= 4) {
          resumeFollow();
        } else if (millis() - stateTimer > 1200) {
          startLost();
        }
      }
      break;
    }

    case ST_FINISH_BOX:
      if (stopAtFinish) {
        stopMotors();
        transitionToState(ST_STOP);
      } else {
        driveStraight(80);
        if (activeCount < 4 && lineVisible) {
          resumeFollow();
        } else if (millis() - stateTimer > 1800) {
          transitionToState(ST_STOP);
        }
      }
      break;

    case ST_LOST:
      runLost();
      break;

    case ST_STOP:
    default:
      stopMotors();
      break;
  }
  // ================= END ACTION BLOCK =================

  handleDataStreaming();
}