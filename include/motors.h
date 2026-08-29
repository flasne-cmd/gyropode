#pragma once

// Pilotage des deux moteurs à courant continu via un driver en pont en H
// (TB6612FNG ou L298N).
class Motors {
 public:
  void begin();

  // Commandes normalisées dans [-1, 1] : positif = marche avant.
  void setSpeeds(float left, float right);

  // Coupe les deux moteurs (roue libre).
  void stop();

 private:
  void driveOne(int channel, int pin_in1, int pin_in2, float speed);
};
