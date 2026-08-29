# Gyropode ESP32

Firmware d'un gyropode (robot à deux roues auto-équilibré) basé sur un ESP32 et
une centrale inertielle MPU6050, pilotable par Bluetooth ou depuis un
navigateur via le Wi-Fi de l'ESP32.

## Fonctionnement

L'ESP32 lit 200 fois par seconde l'accéléromètre et le gyroscope du MPU6050. Un
filtre complémentaire combine les deux mesures pour estimer l'angle
d'inclinaison, et un régulateur PID transforme cet angle en vitesse des roues :
quand le gyropode penche en avant, les roues avancent pour se replacer sous lui.

Piloter revient simplement à demander une inclinaison : le PID fait le reste.

## Matériel

ESP32 DevKit v1, MPU6050, driver TB6612FNG (ou L298N), deux motoréducteurs et
une batterie. Le détail du câblage est dans [docs/cablage.md](docs/cablage.md).

## Compilation et téléversement

### PlatformIO (recommandé)

```bash
pip install platformio
pio run --target upload
pio device monitor
```

### Arduino IDE

Installer le support ESP32 (Boards Manager → *esp32* d'Espressif), puis copier
le contenu de `src/` et `include/` dans un dossier de croquis, en renommant
`main.cpp` en `gyropode.ino`. Carte : *ESP32 Dev Module*.

## Première mise en route

1. Poser le gyropode **à plat et immobile**, puis alimenter : le gyroscope se
   calibre pendant les deux premières secondes.
2. Le tenir à la main, roues en l'air, et vérifier sur le moniteur série que
   l'angle affiché est proche de 0° à la verticale.
3. Vérifier que les roues tournent dans le sens du redressement quand on penche
   le gyropode. Si elles tournent à l'envers, inverser les fils d'un moteur.
4. Régler les gains PID en suivant [docs/reglage-pid.md](docs/reglage-pid.md).

## Pilotage

### Wi-Fi

L'ESP32 crée un point d'accès `Gyropode` (mot de passe `gyropode123`, à changer
dans `include/config.h`). S'y connecter et ouvrir <http://192.168.4.1> : la page
offre les commandes de direction, les curseurs de réglage PID et l'angle en
temps réel.

### Bluetooth

Appairer l'appareil `Gyropode` et utiliser n'importe quel terminal série
Bluetooth :

| Touche | Action |
| --- | --- |
| `z` | avancer |
| `s` | reculer |
| `q` / `d` | tourner à gauche / à droite |
| espace | arrêt |
| `x` / `e` | couper / réactiver les moteurs |

Sans commande reçue pendant 600 ms, le gyropode revient à l'arrêt de lui-même.

## Sécurité

Au-delà de 45° d'inclinaison, les moteurs sont coupés et ne repartent qu'une
fois le gyropode redressé à la main. Faire les premiers essais roues en l'air ou
au-dessus d'une surface dégagée, et garder un moyen de couper l'alimentation
moteurs à portée de main.

## Organisation du code

| Fichier | Rôle |
| --- | --- |
| `src/main.cpp` | boucle d'asservissement et gestion des chutes |
| `src/imu.cpp` | lecture du MPU6050 et filtre complémentaire |
| `src/pid.cpp` | régulateur PID avec anti-emballement |
| `src/motors.cpp` | pilotage PWM des moteurs |
| `src/remote.cpp` | Bluetooth, point d'accès Wi-Fi et interface web |
| `include/config.h` | brochage et paramètres de réglage |
