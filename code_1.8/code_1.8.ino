/*

       UNIVERSAL HYBRID QTR LINE FOLLOWER (V3.3 - MEGA 2560)
       Supports: Black-on-White & White-on-Black


============================================================
BOARD:   Arduino Mega 2560 (ATmega2560)
SENSOR:  Pololu QTR-8A (Analog array on A0-A7, Emitter D13)
DRIVER:  TB6612FNG (Dual H-Bridge)

CHANGES FROM V3.2:
- Ultrasonic obstacle detection (HC-SR04, non-blocking, Timer5 ISR samples ECHO only during a ping).
- Blockage within OBSTACLE_CM -> ST_OBSTACLE_UTURN: brake, encoder-timed pivot (UTURN_DIR), then
  slow spin until the line is re-acquired -> resumeFollow(). No reverse driving.
- Geometry/encoder constants (wheel, track, caster, spin counts L=109 / R=-106) and tick helpers:
  degToTicksL/R(), cmToTicks(), startEncMove() / startEncPivot() / startEncDrive() / runEncMove().
- Telemetry: new state UTURN and new last column "us" (distance in cm, 0 = no echo).
- Removed the Serial.println(rawPos) left in readQTRLine() (a USB print every 2 ms tick can block the loop).

CHANGES FROM V3.1:
- Guarded ST_PIVOT timeout with (state == ST_PIVOT) so it cannot override resumeFollow().
- Cleared jChecker = false whenever LEFT_MASK or RIGHT_MASK is seen during ST_JUNC_PROBE.
- Handled !lineVisible inside ST_CHECKER (stops motors and transitions to ST_LOST after 25 ms).
- Corrected QTR reflectance vs. calibrated value comments in detectTrackType().
- Added hardware note on QTR-8A optimal mounting height (3 mm optimal, 6 mm max).

# PIN ASSIGNMENT:
- QTR Array: A0 (Far Left) -> A7 (Far Right), Emitter D13
- Left Mot:  PWM=D4 (Timer0), IN1=D31, IN2=D33 (IN1 LOW + IN2 HIGH = FWD)
- Right Mot: PWM=D5 (Timer3), IN1=D35, IN2=D37 (IN1 LOW + IN2 HIGH = FWD)
- TB6612:    STBY=D7 (HIGH = Enabled)
- Encoders:  L: A=D2 (INT4), B=D22 | R: A=D3 (INT5), B=D24
- BG IR:     Left=D30, Right=D32
- Ultrasonic (HC-SR04): TRIG=D40, ECHO=D41 (ECHO sampled by Timer5 ISR; D44-D46 PWM no longer usable)
- Button:    D12 (Active LOW / Internal Pullup)
- Wi-Fi UART: Serial3 TX=D14, RX=D15

*/

#include <Arduino.h>
#include <QTRSensors.h>

/* ============================================================
USER CONFIG
============================================================ */

// ---------------- TRACK SURFACE COLOR ----------------
#define TRACK_BLACK_ON_WHITE  0  // Black line on White surface
#define TRACK_WHITE_ON_BLACK  1  // White line on Black surface
#define TRACK_AUTO_DETECT     2  // Automatically detect based on contrast at startup

// Active track mode selection:
#define TRACK_MODE 0

// 1 = Auto-start after countdown | 0 = Wait for D12 button press
#define AUTO_START 1

// 1 = Enable USB Serial diagnostics during setup/calibration (no runtime prints)
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

// ---------------- OBSTACLE (ULTRASONIC) -> U-TURN ----------------
// 1 = Enable ultrasonic obstacle detection and the U-turn reaction
#define OBSTACLE_ENABLED 1

const uint8_t  OBSTACLE_CM          = 20;   // Blockage threshold, measured from the sensor face
const uint8_t  US_FWD_OFFSET_CM     = 0;    // How far the sensor sits ahead of the tyre midpoint (cm)
const uint8_t  US_CONFIRM           = 2;    // Consecutive pings under threshold before reacting
const uint16_t US_PING_MS           = 50;   // Ping period (HC-SR04 needs >= ~40-60 ms between pings)
const uint16_t OBSTACLE_SETTLE_MS   = 80;   // Brake time before the pivot starts
const uint16_t OBSTACLE_COOLDOWN_MS = 800;  // Ignore the sensor this long after a finished U-turn

// U-turn angles (degrees of bot rotation, converted to encoder ticks by degToTicksL/R)
const float UTURN_ARM_DEG = 140.0f;  // Blind encoder pivot; the line sensor is only trusted after this
const float UTURN_MAX_DEG = 250.0f;  // Total pivot limit: line not found by here -> ST_LOST

// ---------------- ENCODERS ----------------
// Set to 1 for a channel whose count decreases when the robot drives forward.
#define ENC_LEFT_INVERT   0
#define ENC_RIGHT_INVERT  0

// Pin mode for encoder inputs (INPUT_PULLUP is safe for push-pull and open-collector outputs)
#define ENC_PIN_MODE INPUT_PULLUP

// ---------------- BACKGROUND IR SENSORS ----------------
// Logic level that each sensor outputs when it sees a BLACK surface.
// 1 = HIGH on black (active-HIGH) | 0 = LOW on black (active-LOW)
#define IR_LEFT_BLACK_IS_HIGH   1
#define IR_RIGHT_BLACK_IS_HIGH  1

// Pin mode for the IR digital outputs
#define IR_PIN_MODE INPUT

// Consecutive identical readings (control-loop ticks) required to change a sensor's state
const uint8_t BG_CONFIRM_COUNT = 8;

// ---------------- SERIAL3 TELEMETRY (Wi-Fi module UART) ----------------
// 1 = Enable telemetry on Serial3 | 0 = Disable
#define TELEMETRY_ENABLED 1
const uint32_t TELEMETRY_BAUD        = 115200UL;
const uint16_t TELEMETRY_INTERVAL_MS = 100;   // Minimum time between frame starts

/* ============================================================
PINS
============================================================ */

/* ============================================================
PINS
============================================================ */

const uint8_t QTR_PINS[8] = { A0, A1, A2, A3, A4, A5, A6, A7 };
const uint8_t EMITTER_PIN = 13;

// D4 = Timer0 OC0B, D5 = Timer3 OC3A. Both support analogWrite() out of the box.
const uint8_t LEFT_PWM = 4;
const uint8_t LEFT_IN1 = 33;
const uint8_t LEFT_IN2 = 31;

const uint8_t RIGHT_PWM = 5;
const uint8_t RIGHT_IN1 = 37;
const uint8_t RIGHT_IN2 = 35;

const uint8_t STBY = 7;
const uint8_t BTN  = 12;

// Encoders (A channels on Mega external-interrupt pins)
const uint8_t ENC_L_A = 2;    // INT4
const uint8_t ENC_L_B = 22;
const uint8_t ENC_R_A = 3;    // INT5
const uint8_t ENC_R_B = 24;

// Background IR sensors
const uint8_t IR_LEFT_PIN  = 30;
const uint8_t IR_RIGHT_PIN = 32;

// Ultrasonic sensor (HC-SR04)
const uint8_t US_TRIG = 40;
const uint8_t US_ECHO = 41;
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

// Primary PID gains (8cm wheelbase, 500 Hz loop; target QTR-8A height: 3mm optimal, 6mm max)
float Kp = 0.068f;    // Base proportional gain
float Ki = 0.0008f;   // Small integral gain for long sweeping curves
float Kd = 0.0018f;   // Derivative gain for anticipation & damping (0.90 effective discrete gain at 500 Hz)

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

// Last PID error / output of the current tick (0 when PID is not running) - telemetry only
int16_t pidErrorOut = 0;
int16_t pidOutputOut = 0;

/* ============================================================
SPEED PROFILES
============================================================ */

// Dynamic speed scaling thresholds (PWM limits)
const int SPEED_STRAIGHT      = 190; // High speed on straight sections (controlled competition speed)
// const int SPEED_STRAIGHT      = 255; // High speed on straight sections (controlled competition speed)
const int SPEED_GENTLE_CURVE  = 90;  // Moderate speed on gentle curves
const int SPEED_MEDIUM_CURVE  = 80;  // Controlled speed on medium curves
const int SPEED_SHARP_CURVE   = 70;  // Low speed for sharp curves / rapid error changes
const int SPEED_MIN_BASE      = 65;  // Minimum forward tracking base speed

// Motor output constraints
const int MOTOR_MAX_FORWARD   = 255;  // TB6612 maximum PWM
const int MOTOR_MAX_REVERSE   = -100; // Allow controlled active counter-rotation for sharp pivots

// Maneuver speeds for autonomous routines
const int PROBE_SPEED         = 130;
const int ALIGN_SPEED         = 120;
const int PIVOT_SPEED         = 160;
const int PIVOT_CAPTURE_SPEED = 120;
const int SEARCH_SPEED        = 155;
const int UTURN_SPEED         = 160;

float currentBaseSpeed = SPEED_MIN_BASE;
const float ACCEL_RATE = 600.0f;  // PWM units / second (smooth pickup out of corners)
const float DECEL_RATE = 8000.0f; // PWM units / second (rapid braking into sharp curves)

/* ============================================================
THRESHOLDS & TIMINGS
============================================================ */

const uint16_t ACTIVE_THR       = 280; // Sensor sees meaningful line (calibrated 0-1000)
const uint16_t STRONG_THR       = 620; // Strong solid line presence
const uint16_t MIN_LINE_SIGNAL  = 160; // Minimum threshold to declare line presence
const uint16_t MIN_CAL_SPAN     = 120; // Minimum sensor calibration span

// Automatic track surface detection safeguards
const uint16_t AUTO_DETECT_MIN_GAP         = 250;
const uint16_t AUTO_DETECT_MAX_PAIR_SPREAD = 350;

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
  ST_STOP,
  ST_OBSTACLE_UTURN,
  ST_IDLE            // Setup / calibration phase (before the control loop starts)
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
  CAL_NONE,          // Not calibrated yet
  CAL_RUNNING,       // Calibration in progress
  CAL_OK,            // All sensors reached MIN_CAL_SPAN
  CAL_LOW            // One or more sensors have too little span
};

enum BgColor : uint8_t {
  BG_UNKNOWN,
  BG_WHITE,
  BG_BLACK
};

// Debounced state of one digital background IR sensor
struct IrFilter {
  bool stableBlack;  // Confirmed reading (true = black)
  bool valid;        // True once a first reading has been confirmed
  bool candidate;    // Reading currently being confirmed
  uint8_t count;     // Consecutive candidate readings
};

// Frozen copy of all telemetry fields, so a frame is internally consistent
struct TlmSnap {
  uint32_t t;
  State    state;
  CalStatus cal;
  BgColor  bg;
  char     irL, irR, line;
  uint16_t q[8];
  int16_t  pos, err, out, pwmL, pwmR;
  int32_t  encL, encR;
  uint8_t  junc;
  bool     jL, jR;
  uint16_t us;
};

State state = ST_IDLE;
LostPhase lostPhase = LP_COAST;
CalStatus calStatus = CAL_NONE;

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
uint32_t junctionCooldownStart = 0;

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

// Last motor commands (logical, before inversion) - telemetry only
int16_t pwmCmdLeft = 0;
int16_t pwmCmdRight = 0;

// Encoders (updated in ISRs)
volatile int32_t encLeftCount = 0;
volatile int32_t encRightCount = 0;
const int8_t ENC_LEFT_SIGN  = ENC_LEFT_INVERT  ? -1 : 1;
const int8_t ENC_RIGHT_SIGN = ENC_RIGHT_INVERT ? -1 : 1;
// Direct port access for fast ISR reads (resolved once in setup from the pin constants)
static volatile uint8_t *encLAport, *encLBport, *encRAport, *encRBport;
static uint8_t encLAmask, encLBmask, encRAmask, encRBmask;

// Background IR
IrFilter irLeft  = { false, false, false, 0 };
IrFilter irRight = { false, false, false, 0 };
BgColor bgColor = BG_UNKNOWN;

// Telemetry frame buffer (built in small slices, drained without blocking)
const uint8_t TLM_BUF_SIZE = 192;
static char tlmBuf[TLM_BUF_SIZE];
static uint8_t tlmLen = 0;        // Bytes written while building
static uint8_t tlmSendLen = 0;    // Bytes of a completed frame ready to send
static uint8_t tlmPos = 0;        // Bytes already pushed into the UART buffer
static uint8_t tlmStage = 0;      // 0 = waiting for next interval, 1.. = building
static uint32_t tlmLastStartMs = 0;
static TlmSnap snap;

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
  pwmCmdLeft  = constrain(left,  -255, 255);
  pwmCmdRight = constrain(right, -255, 255);
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
GEOMETRY & ENCODER CALIBRATION
============================================================

Measured (cm): wheel diameter 4.0 | track (tyre to tyre) 18.0 | caster 15.0 from the tyre midpoint

  wheel circumference    = pi * 4.0             = 12.566 cm
  in-place spin circle   = pi * 18.0            = 56.549 cm  ->  56.549 / 12.566 = 4.5 wheel revs per 360 deg
  encoder                = 12-line hub disc, both edges of channel A counted -> 24 counts / wheel rev
  1 cm of travel         = 24 / 12.566          = 1.910 counts
  theoretical 360 spin   = 4.5 * 24             = 108 counts per wheel (about 3.3 deg per count)

Reference reading L = +109, R = -106:
  - opposite signs: the wheels turned against each other, i.e. an in-place spin (a straight drive reads the
    same sign on both wheels)
  - |109| and |106| are within 2% of 108: that is ONE FULL 360 DEG BOT SPIN, not one wheel revolution
    (one wheel revolution would read about 24).
  - The L/R difference is kept: each wheel gets its own counts-per-spin (SPIN_TICKS_L / SPIN_TICKS_R), so every
    turn target is  ticks = SPIN_TICKS_x * angle / 360  per wheel and a wheel stops when ITS target is reached.
  - Sign convention: forward = positive counts (fix with ENC_LEFT_INVERT / ENC_RIGHT_INVERT). L+ / R- is then
    a RIGHT (clockwise) spin; UTURN_DIR = -1 (left) therefore produces L- / R+.

The caster does not enter the tick maths (it only swings freely). It sets the pivot swing radius: while turning
in place the body sweeps a circle of at least CASTER_CM around the tyre midpoint, so an obstacle must be
farther than that from the midpoint when the U-turn starts (see the static_assert below).
============================================================ */

constexpr float WHEEL_DIA_CM      = 4.0f;
constexpr float TRACK_CM          = 18.0f;
constexpr float CASTER_CM         = 15.0f;
constexpr float WHEEL_CIRC_CM     = (float)PI * WHEEL_DIA_CM;                        // 12.566
constexpr float ENC_TICKS_PER_REV = 24.0f;                                           // 12 lines x 2 edges
constexpr float ENC_TICKS_PER_CM  = ENC_TICKS_PER_REV / WHEEL_CIRC_CM;               // 1.910
constexpr float SPIN_TICKS_THEORY = (TRACK_CM / WHEEL_DIA_CM) * ENC_TICKS_PER_REV;   // 108

// Measured counts for one full 360 deg in-place spin (magnitudes)
constexpr float SPIN_TICKS_L = 109.0f;
constexpr float SPIN_TICKS_R = 106.0f;

// Catches one-wheel-revolution counts (~24) or a wrong dimension typed in above
static_assert(SPIN_TICKS_L > 0.9f * SPIN_TICKS_THEORY && SPIN_TICKS_L < 1.1f * SPIN_TICKS_THEORY &&
  SPIN_TICKS_R > 0.9f * SPIN_TICKS_THEORY && SPIN_TICKS_R < 1.1f * SPIN_TICKS_THEORY,
"SPIN_TICKS_L/R must be within 10% of the geometric 360 deg spin (TRACK/WHEEL * ticks per rev)");

// The pivot must not hit the obstacle: its distance from the tyre midpoint must exceed the swing radius
static_assert(OBSTACLE_CM + US_FWD_OFFSET_CM >= CASTER_CM + 3.0f,
"OBSTACLE_CM + US_FWD_OFFSET_CM must be >= CASTER_CM + 3 cm so the U-turn clears the obstacle");

int32_t degToTicksL(float deg) { return lroundf(SPIN_TICKS_L * deg / 360.0f); }
int32_t degToTicksR(float deg) { return lroundf(SPIN_TICKS_R * deg / 360.0f); }
int32_t cmToTicks(float cm)    { return lroundf(cm * ENC_TICKS_PER_CM); }


/* ============================================================
ENCODERS
============================================================ */

// Quadrature decoding on the A channel (CHANGE); direction from the B channel.
// If A == B after an edge the shaft turns one way, otherwise the other.
void isrEncLeft() {
  bool a = (*encLAport & encLAmask) != 0;
  bool b = (*encLBport & encLBmask) != 0;
  if (a == b) encLeftCount -= ENC_LEFT_SIGN;
  else        encLeftCount += ENC_LEFT_SIGN;
}

void isrEncRight() {
  bool a = (*encRAport & encRAmask) != 0;
  bool b = (*encRBport & encRBmask) != 0;
  if (a == b) encRightCount -= ENC_RIGHT_SIGN;
  else        encRightCount += ENC_RIGHT_SIGN;
}

void initEncoders() {
  pinMode(ENC_L_A, ENC_PIN_MODE);
  pinMode(ENC_L_B, ENC_PIN_MODE);
  pinMode(ENC_R_A, ENC_PIN_MODE);
  pinMode(ENC_R_B, ENC_PIN_MODE);

  encLAport = portInputRegister(digitalPinToPort(ENC_L_A));
  encLBport = portInputRegister(digitalPinToPort(ENC_L_B));
  encRAport = portInputRegister(digitalPinToPort(ENC_R_A));
  encRBport = portInputRegister(digitalPinToPort(ENC_R_B));
  encLAmask = digitalPinToBitMask(ENC_L_A);
  encLBmask = digitalPinToBitMask(ENC_L_B);
  encRAmask = digitalPinToBitMask(ENC_R_A);
  encRBmask = digitalPinToBitMask(ENC_R_B);

  attachInterrupt(digitalPinToInterrupt(ENC_L_A), isrEncLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), isrEncRight, CHANGE);
}

// Atomic snapshot (32-bit reads are not atomic on AVR)
void readEncoders(int32_t &left, int32_t &right) {
  uint8_t sreg = SREG;
  cli();
  left  = encLeftCount;
  right = encRightCount;
  SREG = sreg;
}

void resetEncoders() {
  uint8_t sreg = SREG;
  cli();
  encLeftCount = 0;
  encRightCount = 0;
  SREG = sreg;
}

/* ============================================================
ENCODER-TARGETED MOVES (non-blocking)
============================================================ */

// Each wheel has its own signed target (counts, forward positive), measured from the moment startEncMove()
// was called. This also covers asymmetric moves such as an arc: pass different L and R targets.
static int32_t moveL0, moveR0, moveTgtL, moveTgtR;

void startEncMove(int32_t tgtL, int32_t tgtR) {
  readEncoders(moveL0, moveR0);
  moveTgtL = tgtL;
  moveTgtR = tgtR;
}

// Call once per control tick. Drives each wheel toward its own target and actively brakes it on arrival.
// Returns true when both wheels have arrived. shortcut: no speed ramp or overshoot compensation,
// the U-turn finishes with the line sensor so 1-2 counts of coast do not matter; add a ramp for precise moves.
bool runEncMove(int speed) {
  int32_t l, r;
  readEncoders(l, r);
  int32_t remL = moveTgtL - (l - moveL0);
  int32_t remR = moveTgtR - (r - moveR0);
  bool doneL = (moveTgtL >= 0) ? (remL <= 0) : (remL >= 0);
  bool doneR = (moveTgtR >= 0) ? (remR <= 0) : (remR >= 0);
  setMotors(doneL ? 0 : (moveTgtL > 0 ? speed : -speed),
  doneR ? 0 : (moveTgtR > 0 ? speed : -speed));
  return doneL && doneR;
}

// In-place pivot by an angle: dir = -1 left, +1 right (same convention as spin()). 90 / 180 / any angle.
void startEncPivot(float deg, int8_t dir) {
  startEncMove(dir * degToTicksL(deg), -dir * degToTicksR(deg));
}

// Straight move: cm > 0 forward, cm < 0 backward
void startEncDrive(float cm) {
  int32_t t = cmToTicks(cm);
  startEncMove(t, t);
}


/* ============================================================
ULTRASONIC (HC-SR04) - NON-BLOCKING
============================================================ */
// D41 (PG0) has no pin-change / external interrupt on the Mega, and polling it from loop() cannot work:
// the QTR read keeps the CPU busy for ~1 ms of every 2 ms tick, so an echo edge could be timestamped up to
// ~17 cm late. Instead Timer5 samples ECHO every 50 us, but ONLY while a ping is in flight (<= ~4 ms per
// ping), and counts how long it stays high. Resolution 0.86 cm. The control loop only reads the result.

const uint16_t US_MAX_TICKS          = 80;   // 80 x 50 us = 4 ms echo = ~68 cm; longer = nothing in range
const uint8_t  US_RISE_TIMEOUT_TICKS = 100;  // 5 ms without any echo = sensor silent, treated as clear

enum : uint8_t { US_IDLE, US_WAIT_RISE, US_ECHO_HIGH, US_DONE };
static volatile uint8_t  usPhase = US_IDLE;
static volatile uint16_t usTicks = 0;        // samples waiting for the rise, then samples ECHO stayed high
static volatile uint8_t *usEchoPort;
static uint8_t usEchoMask;

uint16_t usCm = 0;              // Last result in cm (0 = no echo / out of range) - telemetry
uint8_t  usHits = 0;            // Consecutive pings at or below OBSTACLE_CM
uint32_t usLastPingMs = 0;
uint32_t obstacleCooldownStart = 0;

ISR(TIMER5_COMPA_vect) {
  bool hi = (*usEchoPort & usEchoMask) != 0;
  if (usPhase == US_WAIT_RISE) {
    if (hi) {
      usTicks = 1;
      usPhase = US_ECHO_HIGH;
    } else if (++usTicks > US_RISE_TIMEOUT_TICKS) {
      usTicks = 0;
      usPhase = US_DONE;
      TIMSK5 = 0;
    }
  } else if (usPhase == US_ECHO_HIGH) {
    if (hi && usTicks <= US_MAX_TICKS) {
      usTicks++;
    } else {
      if (hi) usTicks = 0;   // Still high past the range limit: nothing in range
      usPhase = US_DONE;
      TIMSK5 = 0;
    }
  }
}

// Call every loop() pass (cheap). Starts a ping when due and collects the finished one.
void usService() {
#if OBSTACLE_ENABLED
  if (usPhase == US_DONE) {
    uint16_t t = usTicks;                        // ISR is stopped, safe to read
    usPhase = US_IDLE;
    usCm = (uint16_t)((t * 6UL) / 7UL);          // 50 us x 343 m/s / 2 = 0.857 cm per sample
    if (usCm != 0 && usCm <= OBSTACLE_CM) {
      if (usHits < 255) usHits++;
    } else {
      usHits = 0;
    }
  } else if (usPhase == US_IDLE && (uint32_t)(millis() - usLastPingMs) >= US_PING_MS) {
    usLastPingMs = millis();
    digitalWrite(US_TRIG, HIGH);
    delayMicroseconds(10);                       // The only blocking part: the 10 us trigger pulse
    digitalWrite(US_TRIG, LOW);
    usTicks = 0;
    usPhase = US_WAIT_RISE;
    TCNT5 = 0;
    TIFR5 = _BV(OCF5A);
    TIMSK5 = _BV(OCIE5A);
  }
#endif
}


/* ============================================================
BACKGROUND COLOUR (DIGITAL IR SENSORS)
============================================================ */

// Telemetry/diagnostics only. Never used by the PID or the state machine.

bool irReadBlack(uint8_t pin, bool blackIsHigh) {
  bool level = (digitalRead(pin) == HIGH);
  return blackIsHigh ? level : !level;
}

// Consecutive-reading confirmation: a new reading must repeat BG_CONFIRM_COUNT times in a row
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

// Sensor placement is unspecified: one sensor may sit over the line while the other
// sees the background. The background is therefore updated only when BOTH confirmed
// readings agree; on disagreement the last agreed value is held.
void updateBackground() {
  irFilterUpdate(irLeft,  irReadBlack(IR_LEFT_PIN,  IR_LEFT_BLACK_IS_HIGH));
  irFilterUpdate(irRight, irReadBlack(IR_RIGHT_PIN, IR_RIGHT_BLACK_IS_HIGH));

  if (irLeft.valid && irRight.valid && irLeft.stableBlack == irRight.stableBlack) {
    bgColor = irLeft.stableBlack ? BG_BLACK : BG_WHITE;
  }
}

/* ============================================================
SERIAL3 TELEMETRY
============================================================ */
/*
Frame (one line, CSV, '\n' terminated):
$TEL,ms,state,cal,bg,irL,irR,q0..q7,pos,err,out,pwmL,pwmR,encL,encR,junc,jL,jR,line,us

state : IDLE FOLLOW JPROBE JALIGN PIVOT CHECKER FINISH LOST UTURN STOP
cal   : NONE RUN OK LOW
bg    : W / B / U(nknown)      - background from the IR pair
irL/R : W / B / -(not yet confirmed)
q0..7 : normalised QTR values 0..1000 (1000 = on line), physical order A0..A7
pos   : line position 0..7000
err   : PID error (post deadband) | out: PID steering correction
pwmL/R: last commanded motor PWM (-255..255)
encL/R: encoder counts
junc  : NONE CAND PROBE TURN | jL/jR: branches seen at the last junction (0/1)
line  : W / B - line colour used by the QTR layer
us    : last ultrasonic distance in cm (0 = no echo / out of range)


A frame is formatted in small slices over several control ticks and drained into the
UART buffer only as space allows, so Serial3 never blocks the control loop.
*/

const __FlashStringHelper* stateName(State s) {
  switch (s) {
    case ST_FOLLOW:     return F("FOLLOW");
    case ST_JUNC_PROBE: return F("JPROBE");
    case ST_JUNC_ALIGN: return F("JALIGN");
    case ST_PIVOT:      return F("PIVOT");
    case ST_CHECKER:    return F("CHECKER");
    case ST_FINISH_BOX: return F("FINISH");
    case ST_LOST:       return F("LOST");
    case ST_STOP:       return F("STOP");
    case ST_OBSTACLE_UTURN: return F("UTURN");
    case ST_IDLE:       return F("IDLE");
    default:            return F("?");
  }
}

const __FlashStringHelper* calName(CalStatus c) {
  switch (c) {
    case CAL_NONE:    return F("NONE");
    case CAL_RUNNING: return F("RUN");
    case CAL_OK:      return F("OK");
    case CAL_LOW:     return F("LOW");
    default:          return F("?");
  }
}

const __FlashStringHelper* junctionName(uint8_t j) {
  switch (j) {
    case 1:  return F("CAND");
    case 2:  return F("PROBE");
    case 3:  return F("TURN");
    default: return F("NONE");
  }
}

// 0 = none, 1 = candidate being confirmed, 2 = probing, 3 = aligning / pivoting
uint8_t junctionStatusCode() {
  if (state == ST_JUNC_PROBE) return 2;
  if (state == ST_JUNC_ALIGN || state == ST_PIVOT) return 3;
  if (state == ST_FOLLOW && junctionCounter > 0) return 1;
  return 0;
}

void tlmChar(char c) {
  if (tlmLen < TLM_BUF_SIZE) tlmBuf[tlmLen++] = c;
}

void tlmFlash(const __FlashStringHelper *s) {
  const char *p = reinterpret_cast<const char *>(s);
  char c;
  while ((c = pgm_read_byte(p++)) != 0) tlmChar(c);
}

void tlmInt16(int16_t v) {
  char t[8];
  itoa(v, t, 10);
  for (const char *p = t; *p; p++) tlmChar(*p);
}

void tlmInt32(int32_t v) {
  char t[13];
  ltoa(v, t, 10);
  for (const char *p = t; *p; p++) tlmChar(*p);
}

void tlmUInt32(uint32_t v) {
  char t[12];
  ultoa(v, t, 10);
  for (const char *p = t; *p; p++) tlmChar(*p);
}

void tlmCommaInt16(int16_t v) { tlmChar(','); tlmInt16(v); }
void tlmCommaInt32(int32_t v) { tlmChar(','); tlmInt32(v); }

void tlmCaptureSnapshot() {
  snap.t     = millis();
  snap.state = state;
  snap.cal   = calStatus;
  snap.bg    = bgColor;
  snap.irL   = irLeft.valid  ? (irLeft.stableBlack  ? 'B' : 'W') : '-';
  snap.irR   = irRight.valid ? (irRight.stableBlack ? 'B' : 'W') : '-';
  snap.line  = isWhiteLine ? 'W' : 'B';
  memcpy(snap.q, sensorValues, sizeof(snap.q));
  snap.pos   = position;
  snap.err   = pidErrorOut;
  snap.out   = pidOutputOut;
  snap.pwmL  = pwmCmdLeft;
  snap.pwmR  = pwmCmdRight;
  readEncoders(snap.encL, snap.encR);
  snap.junc  = junctionStatusCode();
  snap.jL    = jSeenL;
  snap.jR    = jSeenR;
  snap.us    = usCm;
}

// Push as many pending bytes as the UART buffer can take right now (never blocks)
void telemetryPump() {
#if TELEMETRY_ENABLED
  if (tlmPos >= tlmSendLen) return;
  int room = Serial3.availableForWrite();
  if (room <= 0) return;
  uint8_t remaining = tlmSendLen - tlmPos;
  uint8_t n = (room < remaining) ? (uint8_t)room : remaining;
  Serial3.write((const uint8_t *)(tlmBuf + tlmPos), n);
  tlmPos += n;
#endif
}

// Format one slice of the next frame (call once per control tick)
void telemetryBuildStep() {
#if TELEMETRY_ENABLED
  if (tlmStage == 0) {
    if (tlmPos < tlmSendLen) return;  // Previous frame is still draining
    if ((uint32_t)(millis() - tlmLastStartMs) < TELEMETRY_INTERVAL_MS) return;
    tlmLastStartMs = millis();
    tlmCaptureSnapshot();
    tlmLen = 0;
    tlmPos = 0;
    tlmSendLen = 0;
  }

  switch (tlmStage) {
    case 0:
      tlmFlash(F("$TEL,"));
      tlmUInt32(snap.t);
      tlmChar(','); tlmFlash(stateName(snap.state));
      tlmChar(','); tlmFlash(calName(snap.cal));
      tlmChar(',');
      tlmChar(snap.bg == BG_BLACK ? 'B' : (snap.bg == BG_WHITE ? 'W' : 'U'));
      tlmChar(','); tlmChar(snap.irL);
      tlmChar(','); tlmChar(snap.irR);
      tlmStage = 1;
      break;


    case 1:
      for (uint8_t i = 0; i < 4; i++) tlmCommaInt16((int16_t)snap.q[i]);
      tlmStage = 2;
      break;

    case 2:
      for (uint8_t i = 4; i < 8; i++) tlmCommaInt16((int16_t)snap.q[i]);
      tlmCommaInt16(snap.pos);
      tlmStage = 3;
      break;

    case 3:
      tlmCommaInt16(snap.err);
      tlmCommaInt16(snap.out);
      tlmCommaInt16(snap.pwmL);
      tlmCommaInt16(snap.pwmR);
      tlmStage = 4;
      break;

    default:
      tlmCommaInt32(snap.encL);
      tlmCommaInt32(snap.encR);
      tlmChar(','); tlmFlash(junctionName(snap.junc));
      tlmChar(','); tlmChar(snap.jL ? '1' : '0');
      tlmChar(','); tlmChar(snap.jR ? '1' : '0');
      tlmChar(','); tlmChar(snap.line);
      tlmChar(','); tlmInt16((int16_t)snap.us);
      tlmChar('\n');
      tlmSendLen = tlmLen;  // Frame complete: hand over to the pump
      tlmStage = 0;
      break;



  }
#endif
}

void telemetryService() {
  telemetryPump();
  telemetryBuildStep();
}

void telemetryBegin() {
#if TELEMETRY_ENABLED
  Serial3.begin(TELEMETRY_BAUD);
  // One-time column header (setup only, so a short blocking write is acceptable here)
  Serial3.println(F("$HDR,ms,state,cal,bg,irL,irR,q0,q1,q2,q3,q4,q5,q6,q7,pos,err,out,pwmL,pwmR,encL,encR,junc,jL,jR,line,us"));
  tlmLastStartMs = millis();
#endif
}

// Used by blocking setup / calibration phases to keep IR filtering and telemetry alive
void setupService() {
  updateBackground();
  telemetryService();
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
#if DEBUG_SERIAL
  Serial.println(F("[TRACK] Surface Mode: WHITE Line on BLACK Background"));
#endif
  return true;

#elif TRACK_MODE == TRACK_BLACK_ON_WHITE

  isWhiteLine = false;
#if DEBUG_SERIAL
  Serial.println(F("[TRACK] Surface Mode: BLACK Line on WHITE Background"));
#endif
  return true;

#else

  // AUTO DETECT:
  // For typical white/black surfaces:
  // White (high reflectance) -> lower analog reading -> calibrated value near 0
  // Black (low reflectance)  -> higher analog reading -> calibrated value near 1000
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

  if (centerSpread <= AUTO_DETECT_MAX_PAIR_SPREAD &&
    edgeSpread <= AUTO_DETECT_MAX_PAIR_SPREAD) {


    if (centerAverage + AUTO_DETECT_MIN_GAP <= edgeAverage) {
      isWhiteLine = true;
      confident = true;
    } else if (edgeAverage + AUTO_DETECT_MIN_GAP <= centerAverage) {
      isWhiteLine = false;
      confident = true;
    }



  }

  // Fallback for slightly off-center line placement across S2..S5:
  // compare the two most extreme inner sensors against the outer edges if S3/S4 spread was too wide
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
#endif
    return false;
  }

#if DEBUG_SERIAL
  if (isWhiteLine) {
    Serial.println(F("[TRACK] Surface Mode: WHITE Line on BLACK Background"));
  } else {
    Serial.println(F("[TRACK] Surface Mode: BLACK Line on WHITE Background"));
  }
#endif

  return true;
#endif
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
  calStatus = CAL_RUNNING;

#if DEBUG_SERIAL
  Serial.println(F("[CAL] Calibrating QTR Sensors..."));
  Serial.println(F("[CAL] Executing balanced Left-Right sweep..."));
#endif

#if AUTO_CAL_SWEEP
  // Symmetrical Left-Right Calibration Sweep
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
    setupService();


  }
#else
  uint32_t calStart = millis();
  stopMotors();
  while (millis() - calStart < 4000) {
    qtr.calibrate();
    setupService();
    delay(4);
  }
#endif

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

  // Telemetry copies (read-only for the rest of the program)
  pidErrorOut  = (int16_t)error;
  pidOutputOut = (int16_t)correction;

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
  bool straightStillExists = (mask & STRAIGHT_MASK) && lineVisible;
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

void beginLostCapture(int8_t direction) {
  lostPhase = LP_CAPTURE;
  captureDirection = direction;
  lostCaptureCount = 0;
  stateTimer = millis();
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
        beginLostCapture(-lostDirection);
      } else if (elapsed > 260) {
        lostPhase = LP_SEARCH2;
        stateTimer = millis();
      }
      break;

    case LP_SEARCH2:
      spin(-lostDirection, SEARCH_SPEED);
      if (isLineCentered()) {
        beginLostCapture(lostDirection);
      } else if (elapsed > 520) {
        lostPhase = LP_UTURN;
        stateTimer = millis();
      }
      break;

    case LP_UTURN:
      spin(UTURN_DIR, UTURN_SPEED);
      if (elapsed > 350 && isLineCentered()) {
        beginLostCapture(-UTURN_DIR);
      } else if (elapsed > UTURN_TIMEOUT_MS) {
        state = ST_STOP;
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
OBSTACLE U-TURN
============================================================ */

static uint8_t  uPhase = 0;        // 0 = brake, 1 = blind encoder pivot, 2 = slow capture
static uint8_t  uCaptureCount = 0;

void startObstacleUTurn() {
  state = ST_OBSTACLE_UTURN;
  uPhase = 0;
  uCaptureCount = 0;
  stateTimer = millis();
  stopMotors();
}

void runObstacleUTurn() {
  if (millis() - stateTimer > UTURN_TIMEOUT_MS) {   // Stalled wheel / dead encoder / line never found
    startLost();
    return;
  }

  switch (uPhase) {
    case 0:
      stopMotors();
      if (millis() - stateTimer >= OBSTACLE_SETTLE_MS) {
        startEncPivot(UTURN_ARM_DEG, UTURN_DIR);
        uPhase = 1;
      }
      break;

    case 1:
      if (runEncMove(UTURN_SPEED)) {
        startEncPivot(UTURN_MAX_DEG - UTURN_ARM_DEG, UTURN_DIR);   // Remaining pivot budget
        uPhase = 2;
      }
      break;

    default: {
        bool budgetUsed = runEncMove(PIVOT_CAPTURE_SPEED);
        if (isLineCentered()) {
          if (++uCaptureCount >= REACQUIRE_CONFIRM) {
            usHits = 0;
            obstacleCooldownStart = millis();
            resumeFollow();
            return;
          }
        } else {
          uCaptureCount = 0;
        }
        if (budgetUsed) startLost();
        break;
      }
  }
}


/* ============================================================
SUPERVISOR LOOP
============================================================ */

void processFollowState() {
  static uint32_t lostSince = 0;

#if OBSTACLE_ENABLED
  // 0. OBSTACLE: confirmed blockage ahead -> U-turn
  if (usHits >= US_CONFIRM &&
    (uint32_t)(millis() - obstacleCooldownStart) >= OBSTACLE_COOLDOWN_MS) {
    lostSince = 0;
    startObstacleUTurn();
    return;
  }
#endif

  // 1. FINISH BOX CHECK
  if (strongCount >= 7) {
    if (++finishCounter >= FINISH_CONFIRM) {
      lostSince = 0;
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

  bool cooldownElapsed =
    (uint32_t)(millis() - junctionCooldownStart) >= JUNC_COOLDOWN_MS;

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

  // 3. CHECKER PATTERN
  if (runs >= 2 && activeCount >= 4 && !leftBranch && !rightBranch) {
    if (++checkerCounter >= CHECKER_CONFIRM) {
      lostSince = 0;
      state = ST_CHECKER;
      stateTimer = millis();
      return;
    }
  } else {
    checkerCounter = 0;
  }

  // 4. LOST LINE
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
  junctionCooldownStart = millis();
}

/* ============================================================
SETUP & INIT
============================================================ */

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
#if DEBUG_SERIAL
  Serial.begin(115200);
  Serial.println(F("\n=== UNIVERSAL QTR LINE FOLLOWER (V3.2 MEGA 2560) ==="));
#endif

  // No timer register changes: Timer0 stays untouched (D4 PWM, millis(), micros()),
  // Timer3 (D5 PWM) runs at its Arduino default.
  pinMode(LED_BUILTIN, OUTPUT);
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

  // Ultrasonic: pins + Timer5 in CTC mode, 2 MHz (/8), compare every 100 counts = 50 us.
  // The compare interrupt stays disabled (TIMSK5 = 0) until usService() starts a ping.
  pinMode(US_TRIG, OUTPUT);
  digitalWrite(US_TRIG, LOW);
  pinMode(US_ECHO, INPUT);
  usEchoPort = portInputRegister(digitalPinToPort(US_ECHO));
  usEchoMask = digitalPinToBitMask(US_ECHO);
  TCCR5A = 0;
  TCCR5B = _BV(WGM52) | _BV(CS51);
  OCR5A  = 99;
  TIMSK5 = 0;

  // Fast ADC prescaler for 500 Hz QTR sampling (~26 us / conversion).
  // ATmega2560 ADC clock at 16 MHz: Prescaler 32 = 500 kHz, Prescaler 64 = 250 kHz, Prescaler 128 = 125 kHz.
  // The recommended 50-200 kHz range for full 10-bit ADC accuracy is met by prescaler 128 at 16 MHz.
  ADCSRA = (ADCSRA & 0xF8) | 0x05;

  // QTR Setup
  qtr.setTypeAnalog();
  qtr.setSensorPins(QTR_PINS, 8);
  qtr.setEmitterPin(EMITTER_PIN);
  qtr.setSamplesPerSensor(4); // 4 samples * 8 sensors = 32 conversions (~830 us total)

  telemetryBegin();

  setupWait(300);

#if !AUTO_START
#if DEBUG_SERIAL
  Serial.println(F("[USER] Press D12 button to start calibration..."));
#endif
  waitForButton();
#else
  setupWait(800);
#endif

  calibrateQTR();

  if (calStatus != CAL_OK) {
    stopMotors();
    state = ST_STOP;

#if DEBUG_SERIAL
    Serial.println(F("[ERROR] Calibration failed!"));
    Serial.println(F("[ERROR] Robot will remain stopped."));
#endif


    while (true) {
      setupService();
      delay(1);
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
  setupWait(2000);
#endif

  // Identify track surface before driving
  if (!detectTrackType()) {
    stopMotors();
    state = ST_STOP;

#if DEBUG_SERIAL
    Serial.println(F("[ERROR] Automatic track detection failed!"));
    Serial.println(F("[ERROR] Robot will remain stopped."));
    digitalWrite(LED_BUILTIN, HIGH);
#endif


    while (true) {
      setupService();
      delay(1);
    }



  }

#if DEBUG_SERIAL
  Serial.print(F("[GEOM] ticks/cm: "));      Serial.println(ENC_TICKS_PER_CM, 3);
  Serial.print(F("[GEOM] 90 deg  L/R: "));   Serial.print(degToTicksL(90));  Serial.print('/'); Serial.println(degToTicksR(90));
  Serial.print(F("[GEOM] 180 deg L/R: "));   Serial.print(degToTicksL(180)); Serial.print('/'); Serial.println(degToTicksR(180));
#endif

  readQTRLine();
  resetPID();
  resetEncoders();
  junctionCooldownStart = millis() - JUNC_COOLDOWN_MS;
  obstacleCooldownStart = millis() - OBSTACLE_COOLDOWN_MS;
  state = ST_FOLLOW;
  lastLoopMicros = micros();
}

/* ============================================================
MAIN LOOP
============================================================ */

void loop() {
  // Use idle time between control ticks to drain pending telemetry bytes and run the ultrasonic ping (non-blocking)
  telemetryPump();
  usService();

  uint32_t now = micros();
  if (now - lastLoopMicros < LOOP_INTERVAL_US) return;
  dt = (now - lastLoopMicros) * 1e-6f;
  dt = constrain(dt, 0.0010f, 0.0100f);
  lastLoopMicros = now;

  readQTRLine();
  updateBackground();   // Telemetry only; does not influence control

  pidErrorOut = 0;
  pidOutputOut = 0;

  switch (state) {
    case ST_FOLLOW:
      processFollowState();
      break;


    case ST_JUNC_PROBE:
      driveStraight(PROBE_SPEED);

      if (mask & LEFT_MASK) {
        jSeenL = true;
        jChecker = false;
      }

      if (mask & RIGHT_MASK) {
        jSeenR = true;
        jChecker = false;
      }

      if (runs >= 2 &&
        activeCount >= 4 &&
        !jSeenL &&
        !jSeenR) {
        jChecker = true;
      }

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

    case ST_OBSTACLE_UTURN:
      runObstacleUTurn();
      break;

    case ST_STOP:
    default:
      stopMotors();
      break;



  }

  // Format one telemetry slice and push whatever the UART buffer can accept
  telemetryService();
}
