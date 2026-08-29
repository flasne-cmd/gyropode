#pragma once

// ---------------------------------------------------------------------------
// Brochage ESP32
// ---------------------------------------------------------------------------

// Bus I2C vers la centrale inertielle MPU6050
constexpr int PIN_I2C_SDA = 21;
constexpr int PIN_I2C_SCL = 22;

// Driver moteurs (TB6612FNG ou L298N)
constexpr int PIN_MOTOR_LEFT_IN1 = 25;
constexpr int PIN_MOTOR_LEFT_IN2 = 26;
constexpr int PIN_MOTOR_LEFT_PWM = 27;
constexpr int PIN_MOTOR_RIGHT_IN1 = 32;
constexpr int PIN_MOTOR_RIGHT_IN2 = 33;
constexpr int PIN_MOTOR_RIGHT_PWM = 14;
constexpr int PIN_MOTOR_STANDBY = 12;  // broche STBY du TB6612, -1 si absente

// Canaux LEDC (PWM matériel de l'ESP32)
constexpr int LEDC_CHANNEL_LEFT = 0;
constexpr int LEDC_CHANNEL_RIGHT = 1;
constexpr int LEDC_FREQUENCY_HZ = 20000;  // au-delà de l'audible
constexpr int LEDC_RESOLUTION_BITS = 10;  // rapport cyclique 0..1023

// ---------------------------------------------------------------------------
// Asservissement
// ---------------------------------------------------------------------------

constexpr float LOOP_PERIOD_S = 0.005f;  // 200 Hz

// Gains de la boucle d'équilibre (à affiner sur le châssis réel)
constexpr float PID_KP = 22.0f;
constexpr float PID_KI = 90.0f;
constexpr float PID_KD = 0.6f;

// Angle pour lequel le gyropode est parfaitement vertical, en degrés.
// Se calibre en observant la dérive : si l'engin part vers l'avant, augmenter.
constexpr float BALANCE_OFFSET_DEG = 0.0f;

// Au-delà de cet angle le gyropode est considéré comme tombé : moteurs coupés.
constexpr float FALL_ANGLE_DEG = 45.0f;

// Consignes envoyées par le pilote (Bluetooth ou Wi-Fi)
constexpr float MAX_TILT_COMMAND_DEG = 8.0f;   // inclinaison demandée en marche
constexpr float MAX_TURN_COMMAND = 0.35f;      // différentiel gauche/droite

// Filtre complémentaire : poids du gyroscope face à l'accéléromètre
constexpr float COMPLEMENTARY_ALPHA = 0.98f;

// ---------------------------------------------------------------------------
// Connectivité
// ---------------------------------------------------------------------------

constexpr char BLUETOOTH_DEVICE_NAME[] = "Gyropode";

// Point d'accès Wi-Fi créé par l'ESP32 (l'interface web est sur http://192.168.4.1)
constexpr char WIFI_AP_SSID[] = "Gyropode";
constexpr char WIFI_AP_PASSWORD[] = "gyropode123";  // 8 caractères minimum
