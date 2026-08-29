#include "pid.h"

#include <Arduino.h>

Pid::Pid(float kp, float ki, float kd, float output_limit)
    : kp_(kp), ki_(ki), kd_(kd), output_limit_(output_limit) {}

float Pid::update(float setpoint, float measurement, float measurement_rate, float dt) {
  const float error = setpoint - measurement;

  const float proportional = kp_ * error;
  const float derivative = -kd_ * measurement_rate;

  // L'intégrale n'est accumulée que si la sortie qui en résulte reste dans les
  // limites, sinon elle continuerait à grossir pendant la saturation.
  const float candidate_integral = integral_ + ki_ * error * dt;
  const float candidate_output = proportional + candidate_integral + derivative;
  if (candidate_output > -output_limit_ && candidate_output < output_limit_) {
    integral_ = candidate_integral;
  }

  return constrain(proportional + integral_ + derivative, -output_limit_, output_limit_);
}

void Pid::reset() { integral_ = 0.0f; }

void Pid::setGains(float kp, float ki, float kd) {
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
  integral_ = 0.0f;
}
