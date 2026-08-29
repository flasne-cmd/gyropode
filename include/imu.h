#pragma once

#include <stdint.h>

// Lecture du MPU6050 et estimation de l'angle d'inclinaison (tangage).
class Imu {
 public:
  // Initialise le bus I2C et réveille le capteur. Renvoie false si le MPU6050
  // ne répond pas.
  bool begin();

  // Mesure le biais du gyroscope, gyropode maintenu immobile et vertical.
  void calibrate(int samples = 1000);

  // Met à jour l'estimation d'angle. dt en secondes.
  void update(float dt);

  // Angle de tangage estimé, en degrés (0 = vertical).
  float pitchDeg() const { return pitch_deg_; }

  // Vitesse angulaire de tangage, en degrés par seconde.
  float pitchRateDps() const { return pitch_rate_dps_; }

 private:
  struct RawSample {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
  };

  bool readRaw(RawSample& sample) const;

  float gyro_bias_dps_ = 0.0f;
  float pitch_deg_ = 0.0f;
  float pitch_rate_dps_ = 0.0f;
};
