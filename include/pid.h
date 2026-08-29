#pragma once

// Régulateur PID avec sortie saturée et anti-emballement de l'intégrale.
class Pid {
 public:
  Pid(float kp, float ki, float kd, float output_limit);

  // measurement_rate est la dérivée mesurée de la grandeur régulée : l'utiliser
  // évite le pic de dérivée lors des changements de consigne.
  float update(float setpoint, float measurement, float measurement_rate, float dt);

  void reset();

  void setGains(float kp, float ki, float kd);

  float kp() const { return kp_; }
  float ki() const { return ki_; }
  float kd() const { return kd_; }

 private:
  float kp_;
  float ki_;
  float kd_;
  float output_limit_;
  float integral_ = 0.0f;
};
