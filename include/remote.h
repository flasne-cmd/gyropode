#pragma once

#include <Arduino.h>

class Pid;

// Consigne de pilotage partagée entre le Bluetooth, l'interface web et la
// boucle d'asservissement.
struct DriveCommand {
  float throttle = 0.0f;  // [-1, 1], positif = avant
  float steering = 0.0f;  // [-1, 1], positif = droite
  bool enabled = true;    // false = moteurs coupés
};

// Télémétrie renvoyée à l'interface web.
struct Telemetry {
  float pitch_deg;
  float output;
  bool fallen;
};

// Reçoit les commandes du pilote par Bluetooth série et par une page web
// servie sur le point d'accès Wi-Fi de l'ESP32.
class Remote {
 public:
  // Le PID est utilisé pour appliquer les gains réglés depuis l'interface web.
  explicit Remote(Pid& balance_pid) : balance_pid_(balance_pid) {}

  void begin();

  // À appeler régulièrement : traite les trames Bluetooth et les requêtes HTTP.
  void poll();

  // Commande courante ; remise à zéro si plus rien n'est reçu (sécurité).
  DriveCommand command();

  void setTelemetry(const Telemetry& telemetry) { telemetry_ = telemetry; }

 private:
  void handleBluetooth();
  void applyCharCommand(char c);
  void registerWebRoutes();

  Pid& balance_pid_;
  DriveCommand command_;
  Telemetry telemetry_{};
  uint32_t last_command_ms_ = 0;
};
