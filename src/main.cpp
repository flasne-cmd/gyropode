#include <Arduino.h>
#include <math.h>

#include "config.h"
#include "imu.h"
#include "motors.h"
#include "pid.h"
#include "remote.h"

namespace {

Imu imu;
Motors motors;
Pid balance_pid(PID_KP, PID_KI, PID_KD, 1.0f);
Remote remote(balance_pid);

bool fallen = true;  // au démarrage on attend d'être redressé à la main
uint32_t last_loop_us = 0;

}  // namespace

void setup() {
  Serial.begin(115200);

  motors.begin();

  if (!imu.begin()) {
    Serial.println("MPU6050 introuvable : verifier le cablage I2C (SDA/SCL/3V3/GND).");
    while (true) {
      motors.stop();
      delay(1000);
    }
  }

  Serial.println("Calibration du gyroscope : ne pas bouger le gyropode...");
  imu.calibrate();
  Serial.println("Calibration terminee.");

  remote.begin();
  Serial.printf("Wi-Fi \"%s\" actif, interface sur http://192.168.4.1\n", WIFI_AP_SSID);

  last_loop_us = micros();
}

void loop() {
  remote.poll();

  const uint32_t now_us = micros();
  const float elapsed_s = static_cast<float>(now_us - last_loop_us) * 1e-6f;
  if (elapsed_s < LOOP_PERIOD_S) {
    return;
  }
  last_loop_us = now_us;

  imu.update(elapsed_s);
  const float pitch_deg = imu.pitchDeg();

  // Redressement manuel : on ne relance les moteurs qu'une fois le gyropode
  // ramené proche de la verticale.
  if (fabsf(pitch_deg) > FALL_ANGLE_DEG) {
    fallen = true;
  } else if (fallen && fabsf(pitch_deg) < 2.0f) {
    fallen = false;
    balance_pid.reset();
  }

  const DriveCommand command = remote.command();

  if (fallen || !command.enabled) {
    motors.stop();
    balance_pid.reset();
    remote.setTelemetry({pitch_deg, 0.0f, fallen});
    return;
  }

  // Avancer revient à demander une inclinaison : le gyropode se penche puis
  // « court après » son propre déséquilibre.
  const float setpoint_deg = BALANCE_OFFSET_DEG - command.throttle * MAX_TILT_COMMAND_DEG;

  const float output =
      balance_pid.update(setpoint_deg, pitch_deg, imu.pitchRateDps(), elapsed_s);

  const float turn = command.steering * MAX_TURN_COMMAND;
  motors.setSpeeds(constrain(output + turn, -1.0f, 1.0f),
                   constrain(output - turn, -1.0f, 1.0f));

  remote.setTelemetry({pitch_deg, output, fallen});
}
