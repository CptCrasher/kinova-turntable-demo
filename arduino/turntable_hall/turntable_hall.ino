#include <AccelStepper.h>

// ULN2003: IN1=D8, IN2=D9, IN3=D10, IN4=D11
// KY-024: D0=D2, VCC=5V, GND=GND
AccelStepper motor(AccelStepper::HALF4WIRE, 8, 10, 9, 11);

const int HALL_PIN = 2;
const int MAGNET_LEVEL = LOW;          // Change to HIGH if your sensor is inverted
const float MOTOR_SPEED = 136.0;      // Half-steps/second (~30 s per revolution)
const unsigned long STABLE_MS = 30;   // Filter Hall signal flicker
const unsigned long LOCKOUT_MS = 8000; // Ignore repeat hits from wide magnet

bool lastRaw, stable;
unsigned long changedAt, lastRef = 0;
bool seenRef = false;

void setup() {
  Serial.begin(115200);
  pinMode(HALL_PIN, INPUT_PULLUP);
  lastRaw = stable = (digitalRead(HALL_PIN) == MAGNET_LEVEL);
  changedAt = millis();
  motor.setMaxSpeed(400);
  motor.setSpeed(MOTOR_SPEED);
  Serial.println("READY");
}

void loop() {
  motor.runSpeed(); // Keep rotating; do not add delay()

  unsigned long now = millis();
  bool raw = (digitalRead(HALL_PIN) == MAGNET_LEVEL);
  if (raw != lastRaw) {
    lastRaw = raw;
    changedAt = now;
  }

  if (raw != stable && now - changedAt >= STABLE_MS) {
    stable = raw;
    if (stable && (!seenRef || now - lastRef >= LOCKOUT_MS)) {
      lastRef = now;
      seenRef = true;
      Serial.print("REF,");
      Serial.println(micros());
    }
  }
}
