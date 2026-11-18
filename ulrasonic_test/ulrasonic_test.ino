#define TRIG_PIN 8
#define ECHO_PIN 9

const unsigned long MEASURE_INTERVAL = 60;
const unsigned long RISE_TIMEOUT = 10000;
const unsigned long FALL_TIMEOUT = 30000;

enum EchoState {
  IDLE,
  TRIGGERING,
  WAIT_RISE,
  WAIT_FALL
};

EchoState state = IDLE;

unsigned long lastMeasurement = 0;
unsigned long triggerStart = 0;
unsigned long echoStart = 0;

void setup() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  digitalWrite(TRIG_PIN, LOW);

  Serial3.begin(115200);
}

void loop() {
  unsigned long nowMs = millis();
  unsigned long nowUs = micros();

  switch (state) {
    case IDLE:
      if (nowMs - lastMeasurement >= MEASURE_INTERVAL) {
        lastMeasurement = nowMs;
        digitalWrite(TRIG_PIN, HIGH);
        triggerStart = nowUs;
        state = TRIGGERING;
      }
      break;

    case TRIGGERING:
      if (nowUs - triggerStart >= 10) {
        digitalWrite(TRIG_PIN, LOW);
        state = WAIT_RISE;
      }
      break;

    case WAIT_RISE:
      if (digitalRead(ECHO_PIN) == HIGH) {
        echoStart = nowUs;
        state = WAIT_FALL;
      } else if (nowUs - triggerStart > RISE_TIMEOUT) {
        Serial3.println("Timeout");
        state = IDLE;
      }
      break;

    case WAIT_FALL:
      if (digitalRead(ECHO_PIN) == LOW) {
        unsigned long echoDuration = nowUs - echoStart;
        float distance = echoDuration * 0.0343 / 2.0;

        Serial3.println(distance, 2);

        state = IDLE;
      } else if (nowUs - echoStart > FALL_TIMEOUT) {
        Serial3.println("Timeout");
        state = IDLE;
      }
      break;
  }
}