#include <Arduino.h>
#include <ArduinoJson.h>

// CHANGE THESE PINS TO MATCH YOUR ESP32-S3 BOARD/WIRING.
// Cytron MDD10A is driven in PWM + DIR mode: one PWM and one DIR per side.
constexpr int LEFT_PWM_PIN = 5;
constexpr int LEFT_DIR_PIN = 6;
constexpr int RIGHT_PWM_PIN = 7;
constexpr int RIGHT_DIR_PIN = 8;
constexpr int ESTOP_SENSE_PIN = 9;   // optional auxiliary contact; physical E-stop must cut motor power in hardware
constexpr int LEFT_ENC_A_PIN = 10;
constexpr int RIGHT_ENC_A_PIN = 11;

constexpr int PWM_FREQ = 18000;
constexpr int PWM_BITS = 8;
constexpr uint32_t COMMAND_TIMEOUT_MS = 750;
constexpr uint32_t TELEMETRY_INTERVAL_MS = 250;

volatile long leftTicks = 0;
volatile long rightTicks = 0;
uint32_t lastCommandMs = 0;
uint32_t lastTelemetryMs = 0;
int currentLeft = 0;
int currentRight = 0;

void IRAM_ATTR onLeftEncoder() { leftTicks++; }
void IRAM_ATTR onRightEncoder() { rightTicks++; }

bool estopActive() {
  return digitalRead(ESTOP_SENSE_PIN) == LOW;
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
  if (estopActive()) {
    stopMotors();
    return;
  }
  currentLeft = constrain(left, -100, 100);
  currentRight = constrain(right, -100, 100);
  setMotor(currentLeft, LEFT_PWM_PIN, LEFT_DIR_PIN);
  setMotor(currentRight, RIGHT_PWM_PIN, RIGHT_DIR_PIN);
}

void sendTelemetry() {
  JsonDocument doc;
  doc["type"] = "telemetry";
  doc["estop"] = estopActive();
  doc["left_ticks"] = leftTicks;
  doc["right_ticks"] = rightTicks;
  doc["left"] = currentLeft;
  doc["right"] = currentRight;
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

  ledcAttach(LEFT_PWM_PIN, PWM_FREQ, PWM_BITS);
  ledcAttach(RIGHT_PWM_PIN, PWM_FREQ, PWM_BITS);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A_PIN), onLeftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A_PIN), onRightEncoder, RISING);
  stopMotors();
  lastCommandMs = millis();
}

void loop() {
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
      }
    }
  }

  // Independent software watchdog: loss of the host connection means stop.
  if (millis() - lastCommandMs > COMMAND_TIMEOUT_MS || estopActive()) {
    stopMotors();
  }

  if (millis() - lastTelemetryMs >= TELEMETRY_INTERVAL_MS) {
    sendTelemetry();
    lastTelemetryMs = millis();
  }
  delay(2);
}
