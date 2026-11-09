/*
  ============================================================
           UNIVERSAL HYBRID QTR LINE FOLLOWER (V2.3 - DUAL IR BACKGROUND DETECTION)
           Supports: Black-on-White & White-on-Black
  ============================================================
  BOARD:   Arduino Nano (ATmega328P)
  SENSOR:  Pololu QTR-8A (Analog array on A0-A7, Emitter D13)
  DRIVER:  TB6612FNG (Dual H-Bridge)

  ROBOT GEOMETRY & PHYSICAL SETUP:
    - Wheelbase: Caster wheel to rear drive axle midpoint = ~8 cm
    - Sensor height above track: ~1 cm
    - High-frequency non-blocking loop (500 Hz) with fast ADC clock
    - Intelligent curve-based dynamic speed scaling
    - Balanced Left-Right auto-calibration sweep
    - Clean surface inversion layer for Black/White tracks

  PIN ASSIGNMENT:
    - QTR Array: A0 (Far Left) -> A7 (Far Right)
    - Emitter:   D13
    - Left Mot:  PWM=D3,  IN1=D10, IN2=D11 (D10 LOW + D11 HIGH = FWD)
    - Right Mot: PWM=D9,  IN1=D5,  IN2=D6  (D5 LOW + D6 HIGH  = FWD)
    - TB6612:    STBY tied directly to +5V (always enabled)
    - Background IR: Left=D7, Right=D8 (digital; LOW=black, HIGH=white)
    - Button:    D12 (Active LOW / Internal Pullup)
  ============================================================
*/

#include <Arduino.h>
#include <QTRSensors.h>

/* ============================================================
                         USER CONFIG
   ============================================================ */

// ---------------- TRACK SURFACE COLOR ----------------
#define TRACK_BLACK_ON_WHITE  0  // Black line on White background
#define TRACK_WHITE_ON_BLACK  1  // White line on Black background
#define TRACK_AUTO_DETECT     2  // Detect background using the two side digital IR sensors

// Active track mode selection:
// 0 = force black line on white background
// 1 = force white line on black background
// 2 = auto-detect using D7/D8 (recommended)
#define TRACK_MODE TRACK_AUTO_DETECT

// Side IR module polarity confirmed by your test:
// LOW (0) = black surface; HIGH (1) = white surface.
#define BG_SENSOR_BLACK_LEVEL LOW
#define BG_SENSOR_WHITE_LEVEL HIGH

// Robust startup sampling. Both sensors must independently agree.
#define BG_SENSOR_SAMPLE_COUNT       31
#define BG_SENSOR_MIN_STABLE_READS   25
#define BG_SENSOR_DETECT_RETRIES      3
#define BG_SENSOR_SAMPLE_DELAY_MS     2

// 1 = Auto-start after countdown | 0 = Wait for D12 button press
#define AUTO_START 1

// 1 = Enable USB Serial monitor diagnostics | 0 = Maximum performance
#define DEBUG_SERIAL 1

// 1 = Robot sweeps left/right automatically during calibration
#define AUTO_CAL_SWEEP 0

// 1 = Invert sensor array if mounted with A0 on Right and A7 on Left
#define SENSOR_REVERSED 0

// Motor inversion (0 = Normal, 1 = Invert)
#define LEFT_INVERT  0
#define RIGHT_INVERT 0

// 1 = Brake and halt when a solid finish square is detected
#define STOP_AT_FINISH 0

// Junction turn preference policy
#define POLICY_LEFT      0
#define POLICY_RIGHT     1
#define POLICY_STRAIGHT  2

#define JUNCTION_POLICY POLICY_RIGHT

// U-turn / Dead-end direction (-1 = Left, +1 = Right)
#define UTURN_DIR -1

/* ============================================================
                           PINS
   ============================================================ */

const uint8_t QTR_PINS[8] = { A0, A1, A2, A3, A4, A5, A6, A7 };
const uint8_t EMITTER_PIN = 13;

const uint8_t LEFT_PWM = 3;
const uint8_t LEFT_IN1 = 10;
const uint8_t LEFT_IN2 = 11;

const uint8_t RIGHT_PWM = 9;
const uint8_t RIGHT_IN1 = 5;
const uint8_t RIGHT_IN2 = 6;

// TB6612FNG STBY is physically tied to +5V; D7 is therefore free for the sensor.
const uint8_t LEFT_BG_SENSOR_PIN  = 7;
const uint8_t RIGHT_BG_SENSOR_PIN = 8;
const uint8_t BTN = 12;

/* ============================================================
                      OBJECTS & VARIABLES
   ============================================================ */

QTRSensors qtr;
uint16_t sensorValues[8];

// Active track mode (false = Black line on White, true = White line on Black)
bool isWhiteLine = (TRACK_MODE == TRACK_WHITE_ON_BLACK);

// Fixed high-frequency loop interval: 2000 microseconds = 500 Hz
const uint32_t LOOP_INTERVAL_US = 2000;
uint32_t lastLoopMicros = 0;
float dt = 0.0020f;

// Position coordinate system: 0 (Far Left) .. 3500 (Center) .. 7000 (Far Right)
int16_t position = 3500;
int16_t lastPosition = 3500;
float filteredPosition = 3500.0f;
const float POSITION_ALPHA = 0.88f; // Fast tracking with light noise filter

/* ============================================================
                            PID GAINS
   ============================================================ */

// Primary PID gains (optimized for 8cm wheelbase, 1cm sensor height, 500 Hz loop)
float Kp = 0.068f;    // Base proportional gain
float Ki = 0.0008f;   // Small integral gain for long sweeping curves
float Kd = 0.0018f;   // Derivative gain for anticipation & damping

// Progressive steering boost for tight bends (|error| > 1500)
const float PROGRESSIVE_GAIN = 0.000030f;

// Deadband around center to eliminate jitter on straights
const float ERROR_DEADBAND = 30.0f;

// Integral anti-windup bounds
const float MAX_INTEGRAL = 2500.0f;
const float INTEGRAL_ACTIVE_ZONE = 1200.0f; // Only integrate when error is moderate

// Derivative low-pass filter factor (0.0 = old, 1.0 = raw)
const float D_FILTER_ALPHA = 0.40f;

// Maximum steering correction PWM
const float MAX_CORRECTION = 240.0f;

// PID state variables
float integral = 0.0f;
float lastError = 0.0f;
float derivativeFiltered = 0.0f;

/* ============================================================
                       SPEED PROFILES
   ============================================================ */

// Dynamic speed scaling thresholds (PWM limits)
const int SPEED_STRAIGHT      = 100; // High speed on straight sections (controlled competition speed)
// const int SPEED_STRAIGHT      = 255; // High speed on straight sections (controlled competition speed)
const int SPEED_GENTLE_CURVE  = 90; // Moderate speed on gentle curves
const int SPEED_MEDIUM_CURVE  = 80; // Controlled speed on medium curves
const int SPEED_SHARP_CURVE   = 70;  // Low speed for sharp curves / rapid error changes
const int SPEED_MIN_BASE      = 65;  // Minimum forward tracking base speed

// Motor output constraints
const int MOTOR_MAX_FORWARD   = 255; // TB6612 maximum PWM
const int MOTOR_MAX_REVERSE   = -100; // Allow controlled active counter-rotation for sharp pivots

// Maneuver speeds for autonomous routines
const int PROBE_SPEED         = 130;
const int ALIGN_SPEED         = 120;
const int PIVOT_SPEED         = 160;
const int PIVOT_CAPTURE_SPEED = 120;
const int SEARCH_SPEED        = 155;
const int UTURN_SPEED         = 160;

float currentBaseSpeed = SPEED_MIN_BASE;
const float ACCEL_RATE = 600.0f; // PWM units / second (smooth pickup out of corners)
const float DECEL_RATE = 8000.0f; // PWM units / second (rapid braking into sharp curves)

/* ============================================================
                    THRESHOLDS & TIMINGS
   ============================================================ */

const uint16_t ACTIVE_THR       = 280; // Sensor sees meaningful line (calibrated 0-1000)
const uint16_t STRONG_THR       = 620; // Strong solid line presence
const uint16_t MIN_LINE_SIGNAL  = 160; // Minimum threshold to declare line presence
const uint16_t MIN_CAL_SPAN     = 120; // Minimum sensor calibration span

// Confirmation frame counts
const uint8_t JUNCTION_CONFIRM  = 8;
const uint8_t FINISH_CONFIRM    = 9;  // Prevents thin cross-lines from triggering finish
const uint8_t CHECKER_CONFIRM   = 8;
const uint8_t REACQUIRE_CONFIRM = 4;

// State timings (milliseconds)
const uint16_t PROBE_MS         = 45;
const uint16_t ALIGN_MS         = 30;
const uint16_t PIVOT_TIMEOUT_MS = 1400;
const uint16_t UTURN_TIMEOUT_MS = 2800;
const uint16_t JUNC_COOLDOWN_MS = 220; // Prevents junction re-trigger loop

const int16_t  REACQUIRE_WINDOW = 1100;

/* ============================================================
                   SENSOR MASKS (BITWISE)
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
  ST_STOP
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

State state = ST_FOLLOW;
LostPhase lostPhase = LP_COAST;

// Sensor analysis variables
uint8_t mask = 0;
uint8_t activeCount = 0;
uint8_t strongCount = 0;
uint8_t runs = 0;
bool lineVisible = false;

// Junction variables
bool jSeenL = false;
bool jSeenR = false;
bool jChecker = false;
Turn pendingTurn = T_STRAIGHT;
uint8_t junctionCounter = 0;
uint8_t checkerCounter = 0;
uint8_t finishCounter = 0;
uint32_t junctionCooldownTimer = 0;

// Movement & Pivot tracking
uint32_t stateTimer = 0;
int8_t pivotDir = 0;
uint8_t pivotPhase = 0;
uint8_t pivotClearCount = 0;
uint8_t pivotCaptureCount = 0;

// Lost line tracking
int8_t lostDirection = 0;
int8_t captureDirection = UTURN_DIR;
uint8_t lostCaptureCount = 0;

/* ============================================================
                     FORWARD DECLARATIONS
   ============================================================ */

void setMotors(int left, int right);
void stopMotors();
void resumeFollow();
void startLost();
void printCalibrationReport();

/* ============================================================
                          MOTOR DRIVER
   ============================================================ */

void setMotor(uint8_t pwmPin, uint8_t in1Pin, uint8_t in2Pin, int speed, bool invert) {
  if (invert) speed = -speed;
  speed = constrain(speed, -255, 255);

  if (speed > 0) {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
    analogWrite(pwmPin, speed);
  } else if (speed < 0) {
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
    analogWrite(pwmPin, -speed);
  } else {
    // TB6612 Active Brake
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, HIGH);
    analogWrite(pwmPin, 255);
  }
}

void setMotors(int left, int right) {
  setMotor(LEFT_PWM, LEFT_IN1, LEFT_IN2, left, LEFT_INVERT);
  setMotor(RIGHT_PWM, RIGHT_IN1, RIGHT_IN2, right, RIGHT_INVERT);
}

void driveStraight(int speed) {
  setMotors(speed, speed);
}

void spin(int8_t direction, int speed) {
  // direction: -1 = Left | +1 = Right
  setMotors(direction * speed, -direction * speed);
}

void stopMotors() {
  setMotors(0, 0);
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
    uint8_t idx = SENSOR_REVERSED ? (7 - i) : i;
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

  // Universal surface layer:
  // readLineWhite maps White line = 1000, Black background = 0
  // readLineBlack maps Black line = 1000, White background = 0
  // In both modes, sensorValues[0..7] is normalized so 1000 = ON-LINE, 0 = OFF-LINE.
  if (isWhiteLine) {
    rawPos = qtr.readLineWhite(sensorValues);
  } else {
    rawPos = qtr.readLineBlack(sensorValues);
  }

  if (SENSOR_REVERSED) {
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
#if TRACK_MODE == TRACK_WHITE_ON_BLACK
  isWhiteLine = true;
#elif TRACK_MODE == TRACK_BLACK_ON_WHITE
  isWhiteLine = false;
#else
  // AUTO DETECT using the two additional side-facing digital IR sensors.
  // The sensors must both be over the track BACKGROUND, not over the line.
  // Confirmed module polarity: LOW = black, HIGH = white.
  bool detected = false;
  bool backgroundIsBlack = false;

  for (uint8_t attempt = 0; attempt < BG_SENSOR_DETECT_RETRIES && !detected; attempt++) {
    uint8_t leftBlackReads = 0;
    uint8_t leftWhiteReads = 0;
    uint8_t rightBlackReads = 0;
    uint8_t rightWhiteReads = 0;

    for (uint8_t sample = 0; sample < BG_SENSOR_SAMPLE_COUNT; sample++) {
      const uint8_t leftReading = digitalRead(LEFT_BG_SENSOR_PIN);
      const uint8_t rightReading = digitalRead(RIGHT_BG_SENSOR_PIN);

      if (leftReading == BG_SENSOR_BLACK_LEVEL) leftBlackReads++;
      else if (leftReading == BG_SENSOR_WHITE_LEVEL) leftWhiteReads++;

      if (rightReading == BG_SENSOR_BLACK_LEVEL) rightBlackReads++;
      else if (rightReading == BG_SENSOR_WHITE_LEVEL) rightWhiteReads++;

      delay(BG_SENSOR_SAMPLE_DELAY_MS);
    }

    const bool leftStableBlack = (leftBlackReads >= BG_SENSOR_MIN_STABLE_READS);
    const bool leftStableWhite = (leftWhiteReads >= BG_SENSOR_MIN_STABLE_READS);
    const bool rightStableBlack = (rightBlackReads >= BG_SENSOR_MIN_STABLE_READS);
    const bool rightStableWhite = (rightWhiteReads >= BG_SENSOR_MIN_STABLE_READS);

#if DEBUG_SERIAL
    Serial.print(F("[BG] Attempt "));
    Serial.print(attempt + 1);
    Serial.print(F(" | L black/white: "));
    Serial.print(leftBlackReads);
    Serial.print('/');
    Serial.print(leftWhiteReads);
    Serial.print(F(" | R black/white: "));
    Serial.print(rightBlackReads);
    Serial.print('/');
    Serial.println(rightWhiteReads);
#endif

    // Accept only if each sensor is stable and both sensors identify the same colour.
    if (leftStableBlack && rightStableBlack) {
      backgroundIsBlack = true;
      detected = true;
    } else if (leftStableWhite && rightStableWhite) {
      backgroundIsBlack = false;
      detected = true;
    } else {
      delay(100);
    }
  }

  if (!detected) {
#if DEBUG_SERIAL
    Serial.println(F("[BG ERROR] Sensors did not agree on a stable background colour."));
    Serial.println(F("[BG ERROR] Check sensor alignment, wiring, and that BOTH sensors face the background."));
    Serial.println(F("[BG ERROR] Robot will remain stopped. Set TRACK_MODE manually only after checking the track."));
#endif
    return false;
  }

  // A black background requires reading the white line; a white background requires a black line.
  isWhiteLine = backgroundIsBlack;
#endif

#if DEBUG_SERIAL
  if (isWhiteLine) {
    Serial.println(F("[TRACK] Detected BLACK background -> WHITE line mode"));
  } else {
    Serial.println(F("[TRACK] Detected WHITE background -> BLACK line mode"));
  }
#endif
  return true;
}

bool checkCalibration() {
  bool valid = true;
  for (uint8_t i = 0; i < 8; i++) {
    uint16_t span = qtr.calibrationOn.maximum[i] - qtr.calibrationOn.minimum[i];
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
    Serial.print(i);
    if (i == 0) Serial.print(F(" (Far L)|   "));
    else if (i == 7) Serial.print(F(" (Far R)|   "));
    else Serial.print(F("        |   "));

    if (minVal < 1000) Serial.print(' ');
    if (minVal < 100) Serial.print(' ');
    Serial.print(minVal);

    Serial.print(F("   |   "));
    if (maxVal < 1000) Serial.print(' ');
    if (maxVal < 100) Serial.print(' ');
    Serial.print(maxVal);

    Serial.print(F("   |   "));
    if (span < 1000) Serial.print(' ');
    if (span < 100) Serial.print(' ');
    Serial.print(span);

    Serial.print(F("   |   "));
    if (passed) {
      Serial.println(F("OK"));
    } else {
      Serial.println(F("WARN: LOW"));
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

  if (allPassed && leftAvg >= MIN_CAL_SPAN && rightAvg >= MIN_CAL_SPAN) {
    Serial.println(F(" [CAL RESULT] SUCCESS: All sensors calibrated both sides!"));
  } else {
    Serial.println(F(" [CAL RESULT] WARNING: One or more sensors have low span!"));
  }
  Serial.println(F("========================================================\n"));
#endif
}

void calibrateQTR() {
#if DEBUG_SERIAL
  Serial.println(F("[CAL] Calibrating QTR Sensors..."));
#if AUTO_CAL_SWEEP
  Serial.println(F("[CAL] Executing programmed Left-Right sweep..."));
#else
  Serial.println(F("[CAL] Keep the motors stopped. Manually move the QTR array over BOTH the line and background for 4 seconds."));
  Serial.println(F("[CAL] Keep the sensor at its normal track height while moving it."));
#endif
#endif

#if AUTO_CAL_SWEEP
  // Symmetrical Left-Right Calibration Sweep
  // Robot starts centered on the line:
  // 1. Sweep Right for HALF period (moves Center -> Right extreme)
  // 2. Sweep Left for FULL period (moves Right extreme -> Left extreme across all sensors)
  // 3. Sweep Right for FULL period (moves Left extreme -> Right extreme)
  // 4. Sweep Left for FULL period (moves Right extreme -> Left extreme)
  // 5. Sweep Right for FULL period (moves Left extreme -> Right extreme)
  // 6. Sweep Left for FULL period (moves Right extreme -> Left extreme)
  // 7. Sweep Right for HALF period (returns Left extreme -> Center)
  
  const uint16_t HALF_SWEEP_MS = 280;
  const uint16_t FULL_SWEEP_MS = 560;
  const uint8_t CAL_SPEED = 95; // Stable turning speed for reliable reflectance sampling

  const uint16_t SWEEP_STAGES[] = {
    HALF_SWEEP_MS,                                    // Stage 0: 0 -> 280 ms (Right)
    (uint16_t)(HALF_SWEEP_MS + FULL_SWEEP_MS),        // Stage 1: 280 -> 840 ms (Left)
    (uint16_t)(HALF_SWEEP_MS + 2 * FULL_SWEEP_MS),    // Stage 2: 840 -> 1400 ms (Right)
    (uint16_t)(HALF_SWEEP_MS + 3 * FULL_SWEEP_MS),    // Stage 3: 1400 -> 1960 ms (Left)
    (uint16_t)(HALF_SWEEP_MS + 4 * FULL_SWEEP_MS),    // Stage 4: 1960 -> 2520 ms (Right)
    (uint16_t)(HALF_SWEEP_MS + 5 * FULL_SWEEP_MS),    // Stage 5: 2520 -> 3080 ms (Left)
    (uint16_t)(2 * HALF_SWEEP_MS + 5 * FULL_SWEEP_MS) // Stage 6: 3080 -> 3360 ms (Right to Center)
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
  }
#else
  uint32_t calStart = millis();
  stopMotors();
  while (millis() - calStart < 4000) {
    qtr.calibrate();
    delay(4);
  }
#endif

  stopMotors();
  delay(150);

  printCalibrationReport();
}

/* ============================================================
                          PID FOLLOW
   ============================================================ */

void resetPID() {
  integral = 0.0f;
  derivativeFiltered = 0.0f;
  lastError = (float)position - 3500.0f;
  currentBaseSpeed = SPEED_MIN_BASE;
}

void followPID() {
  if (!lineVisible) return;

  float rawError = (float)position - 3500.0f;
  float error = rawError;

  // 1. Error deadband for straight line stability
  if (fabs(error) < ERROR_DEADBAND) {
    error = 0.0f;
  }

  // 2. Integral term with intelligent anti-windup
  // Only integrate when error is moderate (steady-state curve tracking),
  // but NOT during sharp turns, straight line, or when crossing center.
  if (fabs(error) > ERROR_DEADBAND && fabs(error) < INTEGRAL_ACTIVE_ZONE) {
    integral += error * dt;
    integral = constrain(integral, -MAX_INTEGRAL, MAX_INTEGRAL);
  }

  // Zero-crossing anti-windup: if error changes sign, immediately dump integral
  if ((error > 0.0f && lastError < 0.0f) || (error < 0.0f && lastError > 0.0f)) {
    integral = 0.0f;
  }

  // 3. Derivative term with low-pass filter
  float derivative = (error - lastError) / dt;
  derivativeFiltered = (D_FILTER_ALPHA * derivative) + ((1.0f - D_FILTER_ALPHA) * derivativeFiltered);
  lastError = error;

  // 4. Proportional term with progressive curve scaling
  float absErr = fabs(error);
  float pTerm = Kp * error;

  // Add progressive boost for sharp curves to ensure high steering authority
  if (absErr > 1500.0f) {
    float excess = absErr - 1500.0f;
    float boost = excess * excess * PROGRESSIVE_GAIN;
    if (error > 0.0f) {
      pTerm += boost;
    } else {
      pTerm -= boost;
    }
  }

  float iTerm = Ki * integral;
  float dTerm = Kd * derivativeFiltered;

  float correction = pTerm + iTerm + dTerm;
  correction = constrain(correction, -MAX_CORRECTION, MAX_CORRECTION);

  // 5. Intelligent dynamic speed control based on curve severity
  int targetSpeed;
  if (absErr > 2200.0f) {
    // Sharp curve: drop to controlled low speed
    targetSpeed = SPEED_SHARP_CURVE;
  } else if (absErr > 1400.0f) {
    // Medium-to-sharp curve interpolation
    float factor = (absErr - 1400.0f) / 800.0f;
    targetSpeed = (int)(SPEED_MEDIUM_CURVE - factor * (SPEED_MEDIUM_CURVE - SPEED_SHARP_CURVE));
  } else if (absErr > 600.0f) {
    // Gentle-to-medium curve interpolation
    float factor = (absErr - 600.0f) / 800.0f;
    targetSpeed = (int)(SPEED_GENTLE_CURVE - factor * (SPEED_GENTLE_CURVE - SPEED_MEDIUM_CURVE));
  } else if (absErr > ERROR_DEADBAND) {
    // Straight-to-gentle curve interpolation
    float factor = (absErr - ERROR_DEADBAND) / (600.0f - ERROR_DEADBAND);
    targetSpeed = (int)(SPEED_STRAIGHT - factor * (SPEED_STRAIGHT - SPEED_GENTLE_CURVE));
  } else {
    // Straight line: maximum straight speed
    targetSpeed = SPEED_STRAIGHT;
  }

  // Anticipatory braking if derivative indicates rapid line deviation
  if (fabs(derivativeFiltered) > 35000.0f) {
    targetSpeed = min(targetSpeed, (int)(SPEED_SHARP_CURVE + 15));
  }

  // Smooth acceleration & aggressive deceleration
  if (currentBaseSpeed < targetSpeed) {
    currentBaseSpeed += ACCEL_RATE * dt;
    if (currentBaseSpeed > targetSpeed) currentBaseSpeed = targetSpeed;
  } else if (currentBaseSpeed > targetSpeed) {
    currentBaseSpeed -= DECEL_RATE * dt;
    if (currentBaseSpeed < targetSpeed) currentBaseSpeed = targetSpeed;
  }

  // 6. Motor command generation with controlled differential braking
  int leftSpd  = (int)(currentBaseSpeed + correction);
  int rightSpd = (int)(currentBaseSpeed - correction);

  // Clamp motors within hardware and differential reverse limits
  leftSpd  = constrain(leftSpd,  MOTOR_MAX_REVERSE, MOTOR_MAX_FORWARD);
  rightSpd = constrain(rightSpd, MOTOR_MAX_REVERSE, MOTOR_MAX_FORWARD);

  setMotors(leftSpd, rightSpd);
}

/* ============================================================
                    JUNCTION RESOLUTION
   ============================================================ */

Turn chooseTurn(bool left, bool straight, bool right) {
  Turn pref[3];

#if JUNCTION_POLICY == POLICY_LEFT
  pref[0] = T_LEFT;     pref[1] = T_STRAIGHT; pref[2] = T_RIGHT;
#elif JUNCTION_POLICY == POLICY_RIGHT
  pref[0] = T_RIGHT;    pref[1] = T_STRAIGHT; pref[2] = T_LEFT;
#else
  pref[0] = T_STRAIGHT; pref[1] = T_LEFT;     pref[2] = T_RIGHT;
#endif

  for (uint8_t i = 0; i < 3; i++) {
    if (pref[i] == T_LEFT && left)         return T_LEFT;
    if (pref[i] == T_STRAIGHT && straight) return T_STRAIGHT;
    if (pref[i] == T_RIGHT && right)       return T_RIGHT;
  }

  return T_BACK;
}

void startJunction() {
  state = ST_JUNC_PROBE;
  stateTimer = millis();

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
    pivotDir = UTURN_DIR;
  }

  pivotPhase = 0;
  pivotClearCount = 0;
  pivotCaptureCount = 0;
  stateTimer = millis();
  state = ST_PIVOT;
}

void evaluateJunction() {
  if (jChecker) {
    state = ST_CHECKER;
    stateTimer = millis();
    return;
  }

  // Solid finish box region persisting after probe
  if (strongCount >= 7) {
    state = ST_FINISH_BOX;
    stateTimer = millis();
    return;
  }

  // Verify if straight line actually continues beyond the cross bar
  bool straightStillExists = (mask & CENTER_MASK) && lineVisible;
  Turn selected = chooseTurn(jSeenL, straightStillExists, jSeenR);

  if (selected == T_STRAIGHT) {
    resumeFollow();
  } else {
    pendingTurn = selected;
    state = ST_JUNC_ALIGN;
    stateTimer = millis();
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

void startLost() {
  state = ST_LOST;
  stateTimer = millis();
  lostCaptureCount = 0;

  if (lastPosition < 2800) {
    lostDirection = -1;
  } else if (lastPosition > 4200) {
    lostDirection = 1;
  } else {
    lostDirection = UTURN_DIR;
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
      spin(lostDirection, SEARCH_SPEED);
      if (isLineCentered()) {
        lostPhase = LP_CAPTURE;
        captureDirection = -lostDirection;
        stateTimer = millis();
      } else if (elapsed > 260) {
        lostPhase = LP_SEARCH2;
        stateTimer = millis();
      }
      break;

    case LP_SEARCH2:
      spin(-lostDirection, SEARCH_SPEED);
      if (isLineCentered()) {
        lostPhase = LP_CAPTURE;
        captureDirection = lostDirection;
        stateTimer = millis();
      } else if (elapsed > 520) {
        lostPhase = LP_UTURN;
        stateTimer = millis();
      }
      break;

    case LP_UTURN:
      spin(UTURN_DIR, UTURN_SPEED);
      if (elapsed > 350 && isLineCentered()) {
        lostPhase = LP_CAPTURE;
        captureDirection = -UTURN_DIR;
        stateTimer = millis();
      } else if (elapsed > UTURN_TIMEOUT_MS) {
        state = ST_STOP;
      }
      break;

    case LP_CAPTURE:
      stopMotors();
      if (isLineCentered()) {
        if (++lostCaptureCount >= REACQUIRE_CONFIRM) {
          resumeFollow();
        }
      } else {
        spin(captureDirection, 60);
      }

      if (elapsed > 400) {
        lostPhase = LP_SEARCH1;
        stateTimer = millis();
      }
      break;
  }
}

/* ============================================================
                     SUPERVISOR LOOP
   ============================================================ */

void processFollowState() {
  // 1. FINISH BOX CHECK
  if (strongCount >= 7) {
    if (++finishCounter >= FINISH_CONFIRM) {
      state = ST_FINISH_BOX;
      stateTimer = millis();
      return;
    }
  } else {
    finishCounter = 0;
  }

  // 2. JUNCTION CHECK
  bool leftBranch  = (mask & LEFT_MASK);
  bool rightBranch = (mask & RIGHT_MASK);
  bool centerTrack = (mask & CENTER_MASK);

  bool branchCandidate = (millis() > junctionCooldownTimer) &&
                         (((leftBranch || rightBranch) && centerTrack && activeCount >= 4) ||
                          (runs >= 2 && activeCount >= 4) ||
                          (activeCount >= 6));

  if (branchCandidate) {
    if (++junctionCounter >= JUNCTION_CONFIRM) {
      startJunction();
      return;
    }
  } else {
    junctionCounter = 0;
  }

  // 3. CHECKER PATTERN
  if (runs >= 2 && activeCount >= 4 && !leftBranch && !rightBranch) {
    if (++checkerCounter >= CHECKER_CONFIRM) {
      state = ST_CHECKER;
      stateTimer = millis();
      return;
    }
  } else {
    checkerCounter = 0;
  }

  // 4. LOST LINE
  static uint32_t lostSince = 0;
  if (!lineVisible) {
    if (lostSince == 0) lostSince = millis();
    if (millis() - lostSince > 25) {
      lostSince = 0;
      startLost();
      return;
    }
  } else {
    lostSince = 0;
  }

  // 5. NORMAL TRACKING
  followPID();
}

void resumeFollow() {
  state = ST_FOLLOW;
  resetPID();
  junctionCounter = 0;
  checkerCounter = 0;
  finishCounter = 0;
  lostCaptureCount = 0;
  junctionCooldownTimer = millis() + JUNC_COOLDOWN_MS;
}

/* ============================================================
                        SETUP & INIT
   ============================================================ */

void waitForButton() {
  while (digitalRead(BTN) == HIGH) {
    delay(10);
  }
  delay(50);
  while (digitalRead(BTN) == LOW) {
    delay(10);
  }
  delay(50);
}

void setup() {
#if DEBUG_SERIAL
  Serial.begin(115200);
  Serial.println(F("\n=== UNIVERSAL QTR LINE FOLLOWER (V2.3 - DUAL IR BACKGROUND DETECTION) ==="));
#endif

  // Motor timer high-frequency configuration (~31 kHz for silent, smooth drive)
  TCCR1B = (TCCR1B & 0xF8) | 0x01;   // D9  ~31 kHz
  TCCR2B = (TCCR2B & 0xF8) | 0x01;   // D3  ~31 kHz

  pinMode(LEFT_PWM, OUTPUT);
  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);

  // TB6612FNG STBY is wired directly to +5V, so it is NOT controlled by the Nano.
  // D7 and D8 are dedicated digital background-colour sensor inputs.
  pinMode(LEFT_BG_SENSOR_PIN, INPUT_PULLUP);
  pinMode(RIGHT_BG_SENSOR_PIN, INPUT_PULLUP);

  pinMode(BTN, INPUT_PULLUP);
  stopMotors();

  // Fast ADC prescaler (32) for high-frequency QTR sampling (~26 µs / conversion)
  ADCSRA = (ADCSRA & 0xF8) | 0x05;

  // QTR Setup
  qtr.setTypeAnalog();
  qtr.setSensorPins(QTR_PINS, 8);
  qtr.setEmitterPin(EMITTER_PIN);
  qtr.setSamplesPerSensor(4); // 4 samples * 8 sensors = 32 conversions (~830 µs total)

  delay(300);

#if !AUTO_START
#if DEBUG_SERIAL
  Serial.println(F("[USER] Press D12 button to start calibration..."));
#endif
  waitForButton();
#else
  delay(800);
#endif

  calibrateQTR();

  // Do not run PID with an unusable QTR calibration.
  if (!checkCalibration()) {
#if DEBUG_SERIAL
    Serial.println(F("[CAL ERROR] QTR calibration span is too small. Repeat calibration while sweeping over line AND background."));
#endif
    stopMotors();
    while (true) {
      stopMotors();
      delay(100);
    }
  }

#if !AUTO_START
#if DEBUG_SERIAL
  Serial.println(F("[USER] Place on line & press D12 to detect track and race..."));
#endif
  waitForButton();
#else
#if DEBUG_SERIAL
  Serial.println(F("[AUTO] Starting in 2 seconds..."));
#endif
  delay(2000);
#endif

  // Identify track surface before driving. In AUTO mode, D7 and D8 must both
  // report a stable, matching background colour. Never start with an uncertain mode.
  if (!detectTrackType()) {
    stopMotors();
    while (true) {
      stopMotors();
      delay(100);
    }
  }

  readQTRLine();
  resetPID();
  lastLoopMicros = micros();
}

/* ============================================================
                         MAIN LOOP
   ============================================================ */

void loop() {
  uint32_t now = micros();
  if (now - lastLoopMicros < LOOP_INTERVAL_US) return;
  dt = (now - lastLoopMicros) * 1e-6f;
  dt = constrain(dt, 0.0010f, 0.0100f);
  lastLoopMicros = now;

  readQTRLine();

  switch (state) {
    case ST_FOLLOW:
      processFollowState();
      break;

    case ST_JUNC_PROBE:
      driveStraight(PROBE_SPEED);

      if (mask & LEFT_MASK)  jSeenL = true;
      if (mask & RIGHT_MASK) jSeenR = true;
      if (runs >= 2 && activeCount >= 4) jChecker = true;

      if (millis() - stateTimer >= PROBE_MS) {
        evaluateJunction();
      }
      break;

    case ST_JUNC_ALIGN:
      driveStraight(ALIGN_SPEED);
      if (millis() - stateTimer >= ALIGN_MS) {
        startPivot(pendingTurn);
      }
      break;

    case ST_PIVOT: {
      uint32_t elapsed = millis() - stateTimer;
      bool centerCleared = !(mask & CENTER_MASK);

      if (pivotPhase == 0) {
        spin(pivotDir, PIVOT_SPEED);
        if (centerCleared) {
          if (++pivotClearCount >= 2) {
            pivotClearCount = 0;
            pivotPhase = 1;
          }
        } else {
          pivotClearCount = 0;
        }
      } else if (pivotPhase == 1) {
        spin(pivotDir, PIVOT_SPEED);
        if (isLineCentered()) {
          pivotPhase = 2;
        }
      } else {
        spin(pivotDir, PIVOT_CAPTURE_SPEED);
        if (isLineCentered()) {
          if (++pivotCaptureCount >= REACQUIRE_CONFIRM) {
            pivotCaptureCount = 0;
            resumeFollow();
          }
        } else {
          pivotCaptureCount = 0;
        }
      }

      if (elapsed > PIVOT_TIMEOUT_MS) {
        startLost();
      }
      break;
    }

    case ST_CHECKER:
      followPID();
      if (lineVisible && runs == 1 && activeCount <= 4) {
        resumeFollow();
      } else if (millis() - stateTimer > 1200) {
        startLost();
      }
      break;

    case ST_FINISH_BOX:
#if STOP_AT_FINISH
      stopMotors();
      state = ST_STOP;
#else
      driveStraight(80);
      if (activeCount < 4 && lineVisible) {
        resumeFollow();
      } else if (millis() - stateTimer > 1800) {
        state = ST_STOP;
      }
#endif
      break;

    case ST_LOST:
      runLost();
      break;

    case ST_STOP:
    default:
      stopMotors();
      break;
  }
}