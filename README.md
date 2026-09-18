# Gyropode ESP32

Firmware d'un gyropode (robot deux roues auto-équilibré) sur ESP32 avec MPU6050,
piloté depuis un téléphone ou un PC via le point d'accès Wi-Fi de la carte
(WebSocket), et interface de pilotage/réglage `web/pilotage.html`.

Le code de `src/main.cpp` est celui mis au point et validé sur le robot réel :
tous les réglages (gains, calibrage, seuils) et l'historique des corrections
sont documentés en commentaires dans le fichier.

## Fonctionnement

- Filtre complémentaire sur le MPU6050 → angle `thetaF`.
- Régulateur PD (`Kp`, `Kd`) angle → PWM moteurs, dans une tâche FreeRTOS
  dédiée (période `Te` = 5 ms, coeur 1), pendant que `loop()` (100 ms) gère le
  réseau, la batterie et le diagnostic.
- Boucle de vitesse cascadée (`Kpv`, `Kdv`) et biais direct `KVcons` pour
  avancer/reculer, avec rampe sur la consigne (`VconsRampeMax`).
- Sécurités : coupure au-delà de `seuilChute` (±50°), zone morte `seuilRepos`
  contre les vibrations, coupure sur batterie faible, garde-fou anti-NaN.
- LED d'état batterie (vert / bleu / rouge) et coupure moteur sous ~6,0 V.

## Compilation et téléversement

```bash
pio run                 # compiler
pio run --target upload # téléverser
pio device monitor      # console série, 115200 bauds
```

`platformio.ini` utilise le fork **pioarduino** de la plateforme ESP32 : le
firmware s'appuie sur le coeur Arduino ESP32 3.x (API
`ledcAttach(broche, fréquence, résolution)`), absent de la plateforme
`espressif32` officielle de PlatformIO. Bibliothèques installées
automatiquement : *Adafruit MPU6050* et *WebSockets* (Links2004).

Sous Arduino IDE, installer ces deux bibliothèques et le paquet ESP32 3.x, puis
copier `src/main.cpp` dans un croquis.

## Pilotage

1. Connecter le téléphone ou le PC au réseau Wi-Fi **Gyropode** (mot de passe
   `gyropode1`).
2. Ouvrir `web/pilotage.html` localement dans le navigateur (`file://`), saisir
   l'IP `192.168.4.1` et cliquer **Connecter**.

La page donne les commandes de déplacement, les curseurs de réglage en direct
(gains, calibrage, seuils) et les courbes de diagnostic. Les mêmes commandes
sont acceptées sur le port série USB, une par ligne — la liste complète est en
tête de `src/main.cpp`.

Ouvrir la page en `https://` empêche la connexion `ws://` vers le réseau local :
l'ouvrir en fichier local ou en `http://`.

## Avant tout essai au sol

Vérifier le sens des moteurs avec `TestOn`, `TestG 300`, `TestD 300`, puis
`TestOff`. Une inversion gauche/droite ou de polarité transforme
l'asservissement en réaction positive : le robot accélérerait sa chute.

Le brochage est détaillé dans [docs/cablage.md](docs/cablage.md) et la méthode
de réglage dans [docs/reglage-pid.md](docs/reglage-pid.md).

## Organisation du dépôt

| Chemin | Rôle |
| --- | --- |
| `src/main.cpp` | firmware complet (équilibrage, Wi-Fi/WebSocket, batterie) |
| `web/pilotage.html` | interface de pilotage et de réglage, à ouvrir en local |
| `platformio.ini` | plateforme, bibliothèques et partitions |
| `.vscode/gyropode.code-workspace` | espace de travail VS Code |
| `docs/` | câblage, réglage, travail hors ligne |
| `outils/nouveau-projet-esp32.ps1` | crée un nouveau projet ESP32 avec la même structure |

Pour la copie locale, la compilation sans Internet et l'organisation commune à
plusieurs robots, voir
[docs/travail-hors-ligne.md](docs/travail-hors-ligne.md).
