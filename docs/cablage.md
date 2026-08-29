# Brochage

Reprend les broches déclarées en tête de `src/main.cpp`, telles que câblées et
vérifiées sur le robot.

## Moteurs (pont en H, PWM sur les deux entrées de chaque moteur)

| Fonction | GPIO |
| --- | --- |
| Gauche, sens + (MOTOR-A) | 32 |
| Gauche, sens − | 33 |
| Droit, sens + (MOTOR-B) | 25 |
| Droit, sens − | 26 |

PWM à 20 kHz, résolution 10 bits : le repos correspond à un rapport cyclique de
512 sur les deux entrées, la commande est appliquée en `512 ± Ecc`, saturée à
±470.

Attention : le câblage réel est **inversé** par rapport aux premières étapes du
projet (gauche = 25/26, droit = 32/33). Vérifier avec `TestOn` / `TestG 300` /
`TestD 300` avant tout essai au sol.

## Codeurs (diagnostic uniquement)

| Signal | GPIO |
| --- | --- |
| Codeur gauche | 16 |
| Codeur droit A | 18 |
| Codeur droit B | 19 |

Ces mesures sont affichées (`vG`, `vD`) mais n'entrent dans aucun
asservissement.

## MPU6050

Bus I2C par défaut de l'ESP32 : SDA = GPIO 21, SCL = GPIO 22, alimentation 3V3.
Le capteur doit être fixé rigidement au châssis.

## LED d'état et batterie

| Fonction | GPIO |
| --- | --- |
| LED rouge | 2 |
| LED bleue | 0 |
| LED verte | 4 |
| Mesure batterie (ADC) | 34 |

Seuils sur la mesure ADC : 3500 (~6,2 V) → LED bleue, avertissement ; 3000
(~6,0 V) → LED rouge et coupure des moteurs. Les commandes `TestBatOn`,
`TestBat <val>` et `TestBatOff` permettent de simuler la tension pour vérifier
les LED et la coupure sans décharger la batterie.
