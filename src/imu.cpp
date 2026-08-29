#include "imu.h"

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#include "config.h"

namespace {

constexpr uint8_t MPU6050_ADDRESS = 0x68;
constexpr uint8_t REG_SMPLRT_DIV = 0x19;
constexpr uint8_t REG_CONFIG = 0x1A;
constexpr uint8_t REG_GYRO_CONFIG = 0x1B;
constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;
constexpr uint8_t REG_WHO_AM_I = 0x75;

// Sensibilités pour les pleines échelles configurées (±2 g et ±250 °/s).
constexpr float ACCEL_LSB_PER_G = 16384.0f;
constexpr float GYRO_LSB_PER_DPS = 131.0f;

void writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

}  // namespace

bool Imu::begin() {
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);

  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(REG_WHO_AM_I);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  if (Wire.requestFrom(static_cast<uint8_t>(MPU6050_ADDRESS), static_cast<uint8_t>(1)) != 1) {
    return false;
  }
  Wire.read();

  writeRegister(REG_PWR_MGMT_1, 0x01);   // sortie de veille, horloge sur gyro X
  writeRegister(REG_CONFIG, 0x03);       // filtre passe-bas interne à 44 Hz
  writeRegister(REG_SMPLRT_DIV, 0x00);   // 1 kHz
  writeRegister(REG_GYRO_CONFIG, 0x00);  // ±250 °/s
  writeRegister(REG_ACCEL_CONFIG, 0x00);  // ±2 g
  delay(100);

  RawSample sample;
  if (!readRaw(sample)) {
    return false;
  }
  pitch_deg_ = atan2f(static_cast<float>(sample.accel_y), static_cast<float>(sample.accel_z)) *
               180.0f / PI;
  return true;
}

void Imu::calibrate(int samples) {
  float sum_dps = 0.0f;
  int taken = 0;
  for (int i = 0; i < samples; ++i) {
    RawSample sample;
    if (readRaw(sample)) {
      sum_dps += static_cast<float>(sample.gyro_x) / GYRO_LSB_PER_DPS;
      ++taken;
    }
    delay(2);
  }
  if (taken > 0) {
    gyro_bias_dps_ = sum_dps / static_cast<float>(taken);
  }
}

void Imu::update(float dt) {
  RawSample sample;
  if (!readRaw(sample)) {
    return;
  }

  // Angle absolu donné par la gravité : précis en moyenne, bruité en dynamique.
  const float accel_pitch_deg =
      atan2f(static_cast<float>(sample.accel_y), static_cast<float>(sample.accel_z)) * 180.0f / PI;

  // Vitesse angulaire : peu bruitée mais dérive une fois intégrée.
  pitch_rate_dps_ = static_cast<float>(sample.gyro_x) / GYRO_LSB_PER_DPS - gyro_bias_dps_;

  // Filtre complémentaire : le gyroscope porte le court terme, l'accéléromètre
  // recale le long terme.
  pitch_deg_ = COMPLEMENTARY_ALPHA * (pitch_deg_ + pitch_rate_dps_ * dt) +
               (1.0f - COMPLEMENTARY_ALPHA) * accel_pitch_deg;
}

bool Imu::readRaw(RawSample& sample) const {
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(REG_ACCEL_XOUT_H);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  if (Wire.requestFrom(static_cast<uint8_t>(MPU6050_ADDRESS), static_cast<uint8_t>(14)) != 14) {
    return false;
  }

  auto read16 = []() -> int16_t {
    const uint8_t high = Wire.read();
    const uint8_t low = Wire.read();
    return static_cast<int16_t>((high << 8) | low);
  };

  sample.accel_x = read16();
  sample.accel_y = read16();
  sample.accel_z = read16();
  read16();  // température, inutilisée
  sample.gyro_x = read16();
  sample.gyro_y = read16();
  sample.gyro_z = read16();
  return true;
}
