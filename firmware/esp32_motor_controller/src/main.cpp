#include <Arduino.h>
#include <ArduinoJson.h>
#include <math.h>

// CHANGE THESE PINS TO MATCH YOUR ESP32-S3 BOARD/WIRING.
// Cytron MDD10A is driven in PWM + DIR mode: one PWM and one DIR per side.
constexpr int LEFT_PWM_PIN = 5;
constexpr int LEFT_DIR_PIN = 6;
constexpr int RIGHT_PWM_PIN = 7;
constexpr int RIGHT_DIR_PIN = 8;
constexpr int ESTOP_SENSE_PIN = 9;   // auxiliary contact only; physical E-stop must cut motor power in hardware
constexpr int LEFT_ENC_A_PIN = 10;
constexpr int RIGHT_ENC_A_PIN = 11;

// Optional quadrature B channels. Keep false for the starter single-channel encoder wiring.
// When enabled, direction comes from A/B phase instead of the commanded motor direction.
constexpr bool ENABLE_QUADRATURE_ENCODERS = false;
constexpr int LEFT_ENC_B_PIN = 16;
constexpr int RIGHT_ENC_B_PIN = 17;

// Flip either side if forward motion produces negative signed ticks after wiring/calibration.
constexpr bool LEFT_ENCODER_INVERT = false;
constexpr bool RIGHT_ENCODER_INVERT = false;

// Optional V1.5 front-safety inputs. They are OFF by default so unverified pins cannot
// affect a new build. Verify your board and wiring before changing either flag to true.
constexpr bool ENABLE_FRONT_BUMPERS = false;
constexpr int FRONT_BUMPER_LEFT_PIN = 12;
constexpr int FRONT_BUMPER_RIGHT_PIN = 13;

constexpr bool ENABLE_FRONT_ULTRASONIC = false;
constexpr int FRONT_ULTRASONIC_TRIG_PIN = 14;
constexpr int FRONT_ULTRASONIC_ECHO_PIN = 15;
constexpr float FIRMWARE_STOP_DISTANCE_CM = 35.0f;

constexpr int PWM_FREQ = 18000;
constexpr int PWM_BITS = 8;
constexpr uint32_t COMMAND_TIMEOUT_MS = 750;
constexpr uint32_t TELEMETRY_INTERVAL_MS = 250;
constexpr uint32_t PROXIMITY_INTERVAL_MS = 80;

volatile long leftTicks = 0;
volatile long rightTicks = 0;
volatile int currentLeft = 0;
volatile int currentRight = 0;
uint32_t lastCommandMs = 0;
uint32_t lastTelemetryMs = 0;
uint32_t lastProximityMs = 0;
bool frontBumperLeft = false;
bool frontBumperRight = false;
float frontDistanceCm = NAN;

int tickSign(int command, bool invert) {
  int sign = command > 0 ? 1 : (command < 0 ? -1 : 0);
  return invert ? -sign : sign;
}

int IRAM_ATTR quadratureDelta(int bPin, bool invert) {
  int delta = digitalRead(bPin) == HIGH ? -1 : 1;
  return invert ? -delta : delta;
}

void IRAM_ATTR onLeftEncoder() {
  leftTicks += ENABLE_QUADRATURE_ENCODERS
    ? quadratureDelta(LEFT_ENC_B_PIN, LEFT_ENCODER_INVERT)
    : tickSign(currentLeft, LEFT_ENCODER_INVERT);
}

void IRAM_ATTR onRightEncoder() {
  rightTicks += ENABLE_QUADRATURE_ENCODERS
    ? quadratureDelta(RIGHT_ENC_B_PIN, RIGHT_ENCODER_INVERT)
    : tickSign(currentRight, RIGHT_ENCODER_INVERT);
}

bool estopActive() {
  return digitalRead(ESTOP_SENSE_PIN) == LOW;
}

float measureFrontDistanceCm() {
  if (!ENABLE_FRONT_ULTRASONIC) {
    return NAN;
  }

  digitalWrite(FRONT_ULTRASONIC_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(FRONT_ULTRASONIC_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(FRONT_ULTRASONIC_TRIG_PIN, LOW);

  unsigned long duration = pulseIn(FRONT_ULTRASONIC_ECHO_PIN, HIGH, 25000);
  if (duration == 0) {
    return NAN;
  }
  return (duration * 0.0343f) / 2.0f;
}

void updateSafetySensors(bool forceDistance = false) {
  if (ENABLE_FRONT_BUMPERS) {
    frontBumperLeft = digitalRead(FRONT_BUMPER_LEFT_PIN) == LOW;
    frontBumperRight = digitalRead(FRONT_BUMPER_RIGHT_PIN) == LOW;
  } else {
    frontBumperLeft = false;
    frontBumperRight = false;
  }

  if (ENABLE_FRONT_ULTRASONIC &&
      (forceDistance || millis() - lastProximityMs >= PROXIMITY_INTERVAL_MS)) {
    frontDistanceCm = measureFrontDistanceCm();
    lastProximityMs = millis();
  } else if (!ENABLE_FRONT_ULTRASONIC) {
    frontDistanceCm = NAN;
  }
}

bool frontObstacleActive() {
  if (frontBumperLeft || frontBumperRight) {
    return true;
  }
  return !isnan(frontDistanceCm) && frontDistanceCm <= FIRMWARE_STOP_DISTANCE_CM;
}

bool commandMovesForward(int left, int right) {
  // Differential-drive approximation: the average wheel command is the forward component.
  return ((left + right) / 2.0f) > 0.0f;
}

void setMotor(int percent, int pwmPin, int dirPin) {
  percent = constrain(percent, -100, 100);
  digitalWrite(dirPin, percent >= 0 ? HIGH : LOW);
  int duty = map(abs(percent), 0, 100, 0, 255);
  ledcWrite(pwmPin, duty);
}

void stopMotors() {
  currentLeft = 0;
  currentRight = 0;
  ledcWrite(LEFT_PWM_PIN, 0);
  ledcWrite(RIGHT_PWM_PIN, 0);
}

void applyDrive(int left, int right) {
  int requestedLeft = constrain(left, -100, 100);
  int requestedRight = constrain(right, -100, 100);
  updateSafetySensors(true);

  if (estopActive() ||
      (commandMovesForward(requestedLeft, requestedRight) && frontObstacleActive())) {
    stopMotors();
    return;
  }

  currentLeft = requestedLeft;
  currentRight = requestedRight;
  setMotor(currentLeft, LEFT_PWM_PIN, LEFT_DIR_PIN);
  setMotor(currentRight, RIGHT_PWM_PIN, RIGHT_DIR_PIN);
}

void sendTelemetry() {
  JsonDocument doc;
  doc["type"] = "telemetry";
  doc["estop"] = estopActive();
  doc["left_ticks"] = leftTicks;
  doc["right_ticks"] = rightTicks;
  doc["encoder_direction_mode"] = ENABLE_QUADRATURE_ENCODERS
    ? "quadrature_a_rising_b_direction"
    : "command_signed_single_channel";
  doc["left"] = currentLeft;
  doc["right"] = currentRight;
  doc["front_bumper_left"] = frontBumperLeft;
  doc["front_bumper_right"] = frontBumperRight;
  if (!isnan(frontDistanceCm)) {
    doc["front_distance_cm"] = frontDistanceCm;
  }
  serializeJson(doc, Serial);
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  pinMode(LEFT_DIR_PIN, OUTPUT);
  pinMode(RIGHT_DIR_PIN, OUTPUT);
  pinMode(ESTOP_SENSE_PIN, INPUT_PULLUP);
  pinMode(LEFT_ENC_A_PIN, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A_PIN, INPUT_PULLUP);
  if (ENABLE_QUADRATURE_ENCODERS) {
    pinMode(LEFT_ENC_B_PIN, INPUT_PULLUP);
    pinMode(RIGHT_ENC_B_PIN, INPUT_PULLUP);
  }

  if (ENABLE_FRONT_BUMPERS) {
    pinMode(FRONT_BUMPER_LEFT_PIN, INPUT_PULLUP);
    pinMode(FRONT_BUMPER_RIGHT_PIN, INPUT_PULLUP);
  }
  if (ENABLE_FRONT_ULTRASONIC) {
    pinMode(FRONT_ULTRASONIC_TRIG_PIN, OUTPUT);
    pinMode(FRONT_ULTRASONIC_ECHO_PIN, INPUT);
  }

  ledcAttach(LEFT_PWM_PIN, PWM_FREQ, PWM_BITS);
  ledcAttach(RIGHT_PWM_PIN, PWM_FREQ, PWM_BITS);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A_PIN), onLeftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A_PIN), onRightEncoder, RISING);
  stopMotors();
  lastCommandMs = millis();
  updateSafetySensors(true);
}

void loop() {
  updateSafetySensors();

  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, line);
    if (!error) {
      const char* cmd = doc["cmd"] | "";
      if (strcmp(cmd, "drive") == 0) {
        applyDrive(doc["left"] | 0, doc["right"] | 0);
        lastCommandMs = millis();
      } else if (strcmp(cmd, "stop") == 0) {
        stopMotors();
        lastCommandMs = millis();
      } else if (strcmp(cmd, "reset_encoders") == 0) {
        noInterrupts();
        leftTicks = 0;
        rightTicks = 0;
        interrupts();
        lastCommandMs = millis();
      }
    }
  }

  // Independent software watchdog: loss of the host connection means stop.
  if (millis() - lastCommandMs > COMMAND_TIMEOUT_MS || estopActive()) {
    stopMotors();
  }

  // If a new obstacle appears after a forward command, stop without waiting for the host.
  if (commandMovesForward(currentLeft, currentRight) && frontObstacleActive()) {
    stopMotors();
  }

  if (millis() - lastTelemetryMs >= TELEMETRY_INTERVAL_MS) {
    sendTelemetry();
    lastTelemetryMs = millis();
  }
  delay(2);
}
