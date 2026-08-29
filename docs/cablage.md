# Câblage

## Matériel

| Élément | Référence conseillée |
| --- | --- |
| Carte | ESP32 DevKit v1 (30 broches) |
| Centrale inertielle | MPU6050 (GY-521) |
| Driver moteurs | TB6612FNG (ou L298N) |
| Moteurs | 2 motoréducteurs 6–12 V avec roues |
| Batterie | 2S/3S LiPo ou 6 × AA, selon les moteurs |
| Régulateur | 5 V vers l'ESP32 si la batterie dépasse 5 V |

## MPU6050 → ESP32

| MPU6050 | ESP32 |
| --- | --- |
| VCC | 3V3 |
| GND | GND |
| SDA | GPIO 21 |
| SCL | GPIO 22 |

Le capteur doit être fixé **rigidement** au châssis, à plat, l'axe X aligné avec
l'axe des roues. Un capteur qui vibre ou qui bouge rend l'asservissement
impossible.

## TB6612FNG → ESP32

| TB6612 | ESP32 |
| --- | --- |
| PWMA | GPIO 27 |
| AIN1 | GPIO 25 |
| AIN2 | GPIO 26 |
| PWMB | GPIO 14 |
| BIN1 | GPIO 32 |
| BIN2 | GPIO 33 |
| STBY | GPIO 12 |
| VCC | 3V3 |
| VM | + batterie |
| GND | GND commun avec l'ESP32 et la batterie |

Avec un L298N, PWMA/PWMB correspondent à ENA/ENB et il n'y a pas de broche STBY :
mettre `PIN_MOTOR_STANDBY` à `-1` dans `include/config.h`.

## Points de vigilance

- La masse de la batterie, celle du driver et celle de l'ESP32 doivent être reliées.
- Ne pas alimenter les moteurs depuis le port USB de l'ESP32.
- Placer la batterie le plus haut possible sur le châssis : un centre de gravité
  élevé rend le gyropode plus facile à stabiliser.
- Prévoir un interrupteur qui coupe l'alimentation moteurs, indépendant de l'ESP32.
