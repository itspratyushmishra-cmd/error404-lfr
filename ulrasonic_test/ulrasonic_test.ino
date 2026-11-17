#define TRIG_PIN 8
#define ECHO_PIN 9

const unsigned long MEASURE_INTERVAL = 25;
const unsigned long ECHO_TIMEOUT = 30000;

enum EchoState {
  IDLE,
  WAIT_RISE,
  WAIT_FALL
};

EchoState state = IDLE;

unsigned long lastMeasurement = 0;
unsigned long triggerStart = 0;
unsigned long echoStart = 0;
unsigned long echoDuration = 0;

float distance = 0.0;

void setup() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  digitalWrite(TRIG_PIN, LOW);

  Serial.begin(115200);
}

void loop() {
  unsigned long nowMs = millis();
  unsigned long nowUs = micros();

  if (state == IDLE &&
      nowMs - lastMeasurement >= MEASURE_INTERVAL) {

    lastMeasurement = nowMs;

    digitalWrite(TRIG_PIN, HIGH);
    triggerStart = nowUs;
    state = WAIT_RISE;
  }

  if (state == WAIT_RISE &&
      nowUs - triggerStart >= 10) {

    digitalWrite(TRIG_PIN, LOW);
    echoStart = nowUs;
    state = WAIT_FALL;
  }

  if (state == WAIT_FALL) {

    if (digitalRead(ECHO_PIN) == HIGH) {
      if (echoStart == triggerStart) {
        echoStart = nowUs;
      }
    }

    if (digitalRead(ECHO_PIN) == LOW &&
        echoStart != triggerStart) {

      echoDuration = nowUs - echoStart;
      distance = echoDuration * 0.0343 / 2.0;

      Serial.println(distance, 2);

      state = IDLE;
    }

    if (nowUs - echoStart > ECHO_TIMEOUT) {
      Serial.println("Timeout");
      state = IDLE;
    }
  }
}