#include "motors.h"

#include <Arduino.h>
#include <math.h>

#include "config.h"

namespace {

constexpr int PWM_MAX = (1 << LEDC_RESOLUTION_BITS) - 1;

// En dessous de ce rapport cyclique les moteurs ne tournent pas et se
// contentent de siffler.
constexpr float MIN_DUTY_RATIO = 0.08f;

}  // namespace

void Motors::begin() {
  pinMode(PIN_MOTOR_LEFT_IN1, OUTPUT);
  pinMode(PIN_MOTOR_LEFT_IN2, OUTPUT);
  pinMode(PIN_MOTOR_RIGHT_IN1, OUTPUT);
  pinMode(PIN_MOTOR_RIGHT_IN2, OUTPUT);

  if (PIN_MOTOR_STANDBY >= 0) {
    pinMode(PIN_MOTOR_STANDBY, OUTPUT);
    digitalWrite(PIN_MOTOR_STANDBY, HIGH);
  }

  ledcSetup(LEDC_CHANNEL_LEFT, LEDC_FREQUENCY_HZ, LEDC_RESOLUTION_BITS);
  ledcAttachPin(PIN_MOTOR_LEFT_PWM, LEDC_CHANNEL_LEFT);
  ledcSetup(LEDC_CHANNEL_RIGHT, LEDC_FREQUENCY_HZ, LEDC_RESOLUTION_BITS);
  ledcAttachPin(PIN_MOTOR_RIGHT_PWM, LEDC_CHANNEL_RIGHT);

  stop();
}

void Motors::setSpeeds(float left, float right) {
  driveOne(LEDC_CHANNEL_LEFT, PIN_MOTOR_LEFT_IN1, PIN_MOTOR_LEFT_IN2, left);
  driveOne(LEDC_CHANNEL_RIGHT, PIN_MOTOR_RIGHT_IN1, PIN_MOTOR_RIGHT_IN2, right);
}

void Motors::stop() {
  digitalWrite(PIN_MOTOR_LEFT_IN1, LOW);
  digitalWrite(PIN_MOTOR_LEFT_IN2, LOW);
  digitalWrite(PIN_MOTOR_RIGHT_IN1, LOW);
  digitalWrite(PIN_MOTOR_RIGHT_IN2, LOW);
  ledcWrite(LEDC_CHANNEL_LEFT, 0);
  ledcWrite(LEDC_CHANNEL_RIGHT, 0);
}

void Motors::driveOne(int channel, int pin_in1, int pin_in2, float speed) {
  speed = constrain(speed, -1.0f, 1.0f);
  const float magnitude = fabsf(speed);

  if (magnitude < MIN_DUTY_RATIO) {
    digitalWrite(pin_in1, LOW);
    digitalWrite(pin_in2, LOW);
    ledcWrite(channel, 0);
    return;
  }

  const bool forward = speed > 0.0f;
  digitalWrite(pin_in1, forward ? HIGH : LOW);
  digitalWrite(pin_in2, forward ? LOW : HIGH);
  ledcWrite(channel, static_cast<uint32_t>(magnitude * PWM_MAX));
}
