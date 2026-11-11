/*
  DIGITAL 8-SENSOR LINE FOLLOWER — V1.5

  Hardware: Arduino Nano, TB6612FNG, two N20 motors, 8-channel digital
  reflectance array (outputs D0..D7).  This is intentionally NOT a QTR-8A
  analog sketch: do not include QTRSensors or call analog readLine methods.

  Wiring
    Sensor D0..D7 (left to right) -> A0..A7
    Sensor VCC/GND              -> regulated 5 V/common GND
    TB6612 AIN1/AIN2/PWMA       -> D10/D11/D3  (left motor)
    TB6612 BIN1/BIN2/PWMB       -> D5/D6/D9    (right motor)
    TB6612 STBY                 -> 5 V
    Start button                -> D12 to GND
*/

#include <Arduino.h>

/* ----------------------------- CONFIG ----------------------------- */

// Most LM393/TCRT5000 digital arrays output LOW over a black line and HIGH
// over a white floor.  Change to HIGH only after verifying D0..D7 with LEDs
// or a serial diagnostic.
#define SENSOR_LINE_LEVEL LOW
#define SENSOR_REVERSED 0       // Set to 1 if D0 is physically on the right.
#define LEFT_INVERT 0
#define RIGHT_INVERT 0
#define AUTO_START 0             // 0 = wait for D12; safest for competition.
#define DEBUG_SERIAL 1

// Course-specific features are OFF by default: an all-active pattern can be a
// wide curve, intersection, or finish box. Enable only when rules require it.
#define STOP_AT_FINISH 0
#define FINISH_IS_ALL_SENSORS 1

const uint8_t SENSOR_PINS[8] = {A0, A1, A2, A3, A4, A5, A6, A7};
const uint8_t LEFT_PWM = 3, LEFT_IN1 = 10, LEFT_IN2 = 11;
const uint8_t RIGHT_PWM = 9, RIGHT_IN1 = 5, RIGHT_IN2 = 6;
const uint8_t BUTTON_PIN = 12;

const uint32_t LOOP_INTERVAL_US = 2000;
const uint8_t SENSOR_SAMPLES = 3;       // majority vote rejects comparator chatter
const uint8_t LOST_CONFIRM_FRAMES = 5;  // 10 ms at a 2 ms loop
const uint8_t REACQUIRE_FRAMES = 3;
const uint8_t FINISH_CONFIRM_FRAMES = 15;

// Starting gains for a digital array. Tune on the actual track, in this order:
// Kp until it starts oscillating, Kd until oscillation settles, then Ki only
// if it keeps a consistent left/right bias on long straights.
float Kp = 0.040f;
float Ki = 0.0005f;
float Kd = 0.00045f;
const float ERROR_DEADBAND = 100.0f;
const float INTEGRAL_ZONE = 900.0f;
const float INTEGRAL_LIMIT = 2200.0f;
const float MAX_DERIVATIVE = 80000.0f;
const float DERIVATIVE_ALPHA = 0.30f;
const float MAX_CORRECTION = 180.0f;

// Conservative defaults suit a tall, battery-heavy chassis. Raise only after
// the robot consistently handles the tightest corner without losing the line.
const int SPEED_STRAIGHT = 105;
const int SPEED_GENTLE = 88;
const int SPEED_SHARP = 68;
const int SPEED_SEARCH = 95;
const int SPEED_PIVOT = 120;
const int MOTOR_MAX_FORWARD = 255;
const int MOTOR_MAX_REVERSE = -90;
const float ACCEL_RATE = 500.0f;
const float DECEL_RATE = 5000.0f;

const uint16_t LOST_COAST_MS = 45;
const uint16_t SEARCH_FIRST_MS = 280;
const uint16_t SEARCH_SECOND_MS = 550;
const uint16_t SEARCH_TIMEOUT_MS = 2200;

enum RobotState : uint8_t { FOLLOW, LOST, STOPPED };
enum LostPhase : uint8_t { COAST, SEARCH_PRIMARY, SEARCH_SECONDARY };

RobotState state = FOLLOW;
LostPhase lostPhase = COAST;
uint8_t sensorMask = 0;
uint8_t activeCount = 0;
bool lineVisible = false;
bool lineWasVisible = false;
int16_t position = 3500;
int16_t lastPosition = 3500;
float filteredPosition = 3500.0f;
float lastError = 0.0f;
float integral = 0.0f;
float derivativeFiltered = 0.0f;
float baseSpeed = SPEED_SHARP;
float dt = 0.002f;
uint32_t lastLoopUs = 0;
uint32_t stateSinceMs = 0;
uint32_t lostStartedMs = 0;
uint8_t lostFrames = 0;
uint8_t reacquireFrames = 0;
uint8_t finishFrames = 0;
int8_t searchDirection = -1;

/* --------------------------- MOTOR CONTROL ------------------------- */

void setMotor(uint8_t pwm, uint8_t in1, uint8_t in2, int speed, bool invert) {
  if (invert) speed = -speed;
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    digitalWrite(in1, LOW); digitalWrite(in2, HIGH); analogWrite(pwm, speed);
  } else if (speed < 0) {
    digitalWrite(in1, HIGH); digitalWrite(in2, LOW); analogWrite(pwm, -speed);
  } else {
    digitalWrite(in1, LOW); digitalWrite(in2, LOW); analogWrite(pwm, 0);
  }
}

void setMotors(int left, int right) {
  setMotor(LEFT_PWM, LEFT_IN1, LEFT_IN2, left, LEFT_INVERT);
  setMotor(RIGHT_PWM, RIGHT_IN1, RIGHT_IN2, right, RIGHT_INVERT);
}

void coastMotors() { setMotors(0, 0); }
void spin(int8_t direction, int speed) { setMotors(direction * speed, -direction * speed); }

/* -------------------------- SENSOR PROCESSING ----------------------- */

void readDigitalArray() {
  uint8_t votes[8] = {0};
  for (uint8_t sample = 0; sample < SENSOR_SAMPLES; ++sample) {
    for (uint8_t physical = 0; physical < 8; ++physical) {
      if (digitalRead(SENSOR_PINS[physical]) == SENSOR_LINE_LEVEL) ++votes[physical];
    }
  }

  sensorMask = 0;
  activeCount = 0;
  uint32_t weightedSum = 0;
  for (uint8_t logical = 0; logical < 8; ++logical) {
    uint8_t physical = SENSOR_REVERSED ? (7 - logical) : logical;
    if (votes[physical] >= ((SENSOR_SAMPLES / 2) + 1)) {
      sensorMask |= (1 << logical);
      ++activeCount;
      weightedSum += (uint32_t)logical * 1000UL;
    }
  }

  lineVisible = activeCount > 0;
  if (!lineVisible) return;

  uint16_t rawPosition = weightedSum / activeCount;
  // A digital array is quantized; light filtering suppresses one-sensor jumps
  // without adding enough lag to miss a sharp turn.
  const float POSITION_ALPHA = 0.70f;
  // Reset the filter on reacquisition; otherwise its old endpoint can make a
  // genuine center hit look offset for several control frames.
  if (!lineWasVisible) filteredPosition = rawPosition;
  else filteredPosition = POSITION_ALPHA * rawPosition + (1.0f - POSITION_ALPHA) * filteredPosition;
  position = (int16_t)(filteredPosition + 0.5f);
  lastPosition = position;
  lineWasVisible = true;
}

bool centeredOnLine() {
  const bool centerActive = sensorMask & ((1 << 3) | (1 << 4));
  return lineVisible && centerActive && abs(position - 3500) <= 900;
}

/* ----------------------------- PID FOLLOW --------------------------- */

void resetPid() {
  integral = 0.0f;
  derivativeFiltered = 0.0f;
  lastError = lineVisible ? position - 3500.0f : 0.0f;
  baseSpeed = SPEED_SHARP;
}

int speedForError(float absoluteError) {
  if (absoluteError >= 2200.0f) return SPEED_SHARP;
  if (absoluteError <= 450.0f) return SPEED_STRAIGHT;
  float ratio = (absoluteError - 450.0f) / 1750.0f;
  return (int)(SPEED_STRAIGHT - ratio * (SPEED_STRAIGHT - SPEED_SHARP));
}

void followPid() {
  float error = position - 3500.0f;
  if (fabs(error) < ERROR_DEADBAND) error = 0.0f;

  if (error == 0.0f || (error > 0.0f && lastError < 0.0f) ||
      (error < 0.0f && lastError > 0.0f)) integral = 0.0f;

  float derivative = constrain((error - lastError) / dt, -MAX_DERIVATIVE, MAX_DERIVATIVE);
  derivativeFiltered = DERIVATIVE_ALPHA * derivative +
                       (1.0f - DERIVATIVE_ALPHA) * derivativeFiltered;
  lastError = error;

  const float pTerm = Kp * error;
  const float dTerm = Kd * derivativeFiltered;
  float candidateIntegral = integral;
  if (fabs(error) > ERROR_DEADBAND && fabs(error) < INTEGRAL_ZONE) {
    candidateIntegral = constrain(integral + error * dt, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
  }
  float candidateOutput = pTerm + Ki * candidateIntegral + dTerm;
  if (fabs(candidateOutput) < MAX_CORRECTION) integral = candidateIntegral;

  float correction = constrain(pTerm + Ki * integral + dTerm, -MAX_CORRECTION, MAX_CORRECTION);
  int target = speedForError(fabs(error));
  if (fabs(derivativeFiltered) > 30000.0f) target = min(target, SPEED_GENTLE);
  if (baseSpeed < target) baseSpeed = min((float)target, baseSpeed + ACCEL_RATE * dt);
  else baseSpeed = max((float)target, baseSpeed - DECEL_RATE * dt);

  int left = constrain((int)(baseSpeed + correction), MOTOR_MAX_REVERSE, MOTOR_MAX_FORWARD);
  int right = constrain((int)(baseSpeed - correction), MOTOR_MAX_REVERSE, MOTOR_MAX_FORWARD);
  setMotors(left, right);
}

/* ---------------------------- RECOVERY ------------------------------ */

void startLost() {
  state = LOST;
  lostPhase = COAST;
  stateSinceMs = millis();
  lostStartedMs = stateSinceMs;
  lostFrames = 0;
  reacquireFrames = 0;
  integral = 0.0f;
  derivativeFiltered = 0.0f;
  lastError = 0.0f;
  searchDirection = (lastPosition < 3500) ? -1 : 1;
}

void runLost() {
  uint32_t elapsed = millis() - stateSinceMs;
  if (centeredOnLine()) {
    if (++reacquireFrames >= REACQUIRE_FRAMES) {
      state = FOLLOW;
      resetPid();
      return;
    }
  } else {
    reacquireFrames = 0;
  }

  switch (lostPhase) {
    case COAST:
      setMotors(SPEED_SHARP, SPEED_SHARP);
      if (elapsed >= LOST_COAST_MS) { lostPhase = SEARCH_PRIMARY; stateSinceMs = millis(); }
      break;
    case SEARCH_PRIMARY:
      spin(searchDirection, SPEED_SEARCH);
      if (elapsed >= SEARCH_FIRST_MS) { lostPhase = SEARCH_SECONDARY; stateSinceMs = millis(); }
      break;
    case SEARCH_SECONDARY:
      spin(-searchDirection, SPEED_PIVOT);
      if (elapsed >= SEARCH_SECOND_MS) { state = STOPPED; coastMotors(); }
      break;
  }

  if (millis() - lostStartedMs >= SEARCH_TIMEOUT_MS) { state = STOPPED; coastMotors(); }
}

void waitForStart() {
#if AUTO_START
  delay(1500);
#else
  while (digitalRead(BUTTON_PIN) == HIGH) delay(5);
  while (digitalRead(BUTTON_PIN) == LOW) delay(5);
  delay(50);
#endif
}

void printSensors() {
#if DEBUG_SERIAL
  Serial.print(F("[S] "));
  for (int8_t i = 7; i >= 0; --i) Serial.print((sensorMask >> i) & 1);
  Serial.print(F(" pos=")); Serial.print(position);
  Serial.print(F(" active=")); Serial.println(activeCount);
#endif
}

void setup() {
#if DEBUG_SERIAL
  Serial.begin(115200);
  Serial.println(F("Digital 8-sensor line follower V1.5"));
  Serial.println(F("Set each sensor potentiometer so its LED changes cleanly at the line edge."));
#endif
  TCCR1B = (TCCR1B & 0xF8) | 0x01;
  TCCR2B = (TCCR2B & 0xF8) | 0x01;
  pinMode(LEFT_PWM, OUTPUT); pinMode(LEFT_IN1, OUTPUT); pinMode(LEFT_IN2, OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT); pinMode(RIGHT_IN1, OUTPUT); pinMode(RIGHT_IN2, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  for (uint8_t i = 0; i < 8; ++i) pinMode(SENSOR_PINS[i], INPUT);
  coastMotors();
  waitForStart();
  readDigitalArray();
  if (!lineVisible) lineWasVisible = false;
  resetPid();
  lastLoopUs = micros();
}

void loop() {
  uint32_t now = micros();
  if ((uint32_t)(now - lastLoopUs) < LOOP_INTERVAL_US) return;
  dt = constrain((now - lastLoopUs) * 1e-6f, 0.001f, 0.015f);
  lastLoopUs = now;
  readDigitalArray();

  if (state == STOPPED) { coastMotors(); return; }

  if (STOP_AT_FINISH && FINISH_IS_ALL_SENSORS && activeCount == 8) {
    if (++finishFrames >= FINISH_CONFIRM_FRAMES) { state = STOPPED; coastMotors(); return; }
  } else {
    finishFrames = 0;
  }

  if (state == FOLLOW) {
    if (!lineVisible) {
      if (++lostFrames >= LOST_CONFIRM_FRAMES) startLost();
      else setMotors(SPEED_SHARP, SPEED_SHARP);
    } else {
      lostFrames = 0;
      followPid();
    }
  } else {
    runLost();
  }
}
