# Travailler hors ligne sur le PC, avec la même structure

Objectif : avoir sur le PC une copie complète du projet, utilisable sans
connexion Internet, et réutiliser exactement la même organisation pour les
autres robots ESP32.

## 1. Emplacement des projets

Un dossier racine unique, un sous-dossier (= un dépôt Git) par robot :

```
C:\Users\fabrice\robots\
    gyropode\
    bras-robot\
    suiveur-ligne\
```

Éviter le Bureau et les dossiers synchronisés (OneDrive, Dropbox) : PlatformIO
y écrit des milliers de fichiers dans `.pio` et la synchronisation ralentit
fortement la compilation.

## 2. Copie locale du gyropode

```
mkdir C:\Users\fabrice\robots
cd C:\Users\fabrice\robots
git clone https://github.com/flasne-cmd/code.git gyropode
cd gyropode
pio run
```

Le premier `pio run` doit être fait **connecté** : il télécharge la plateforme
ESP32, la toolchain et les bibliothèques. Ensuite tout est en cache dans
`C:\Users\fabrice\.platformio` et la compilation fonctionne hors ligne.

Pour ne rien avoir à télécharger lors du premier essai hors ligne, lancer une
fois, en ligne :

```
pio run                     # plateforme + bibliothèques
pio pkg install             # dépendances de platformio.ini
pio run --target upload     # vérifie aussi l'outil de flash esptool
```

## 3. Ce qui marche hors ligne, ce qui ne marche pas

Fonctionne sans Internet : compilation, téléversement sur COM3, moniteur série,
commits Git locaux, toute la documentation du dépôt.

Ne fonctionne pas sans Internet :

- `git pull` / `git push` (à faire quand la connexion revient) ;
- l'ajout d'une **nouvelle** bibliothèque dans `lib_deps` ;
- le changement de version de plateforme dans `platformio.ini` ;
- les graphiques de `web/pilotage.html`, qui chargent Chart.js depuis un CDN.

Pour les graphiques hors ligne, télécharger une fois
`https://cdn.jsdelivr.net/npm/chart.js` dans `web/chart.min.js` et remplacer
l'URL du `<script src=...>` par `chart.min.js`. Le pilotage lui-même (WebSocket
vers `192.168.4.1:81`) n'a jamais besoin d'Internet : le PC est connecté au
point d'accès de l'ESP32, donc coupé du réseau pendant les essais.

## 4. Structure commune à tous les projets

```
<projet>/
    platformio.ini              plateforme, carte, partitions, lib_deps
    README.md                   but du robot, matériel, mode d'emploi
    .gitignore                  ignore .pio/, *.bin, .vscode sauf le workspace
    src/main.cpp                firmware
    web/pilotage.html           interface WebSocket (si pilotage sans fil)
    docs/cablage.md             brochage réel, alimentation, moteurs
    docs/reglage-pid.md         paramètres et méthode de réglage
    .vscode/<projet>.code-workspace
```

Garder ces noms à l'identique d'un projet à l'autre : la documentation, les
scripts et les sessions Devin s'y retrouvent sans explication supplémentaire.

## 5. Créer un nouveau projet avec la même structure

Deux possibilités.

**Script local** — depuis le dossier racine des robots :

```
powershell -ExecutionPolicy Bypass -File C:\Users\fabrice\robots\gyropode\outils\nouveau-projet-esp32.ps1 bras-robot
```

Le script crée l'arborescence, un `platformio.ini` identique à celui du
gyropode, un `README.md`, le `.gitignore`, les squelettes de `docs/` et
initialise le dépôt Git local.

**Depuis Devin** — demander le playbook « Nouveau projet robotique ESP32 » :
le dépôt GitHub est créé avec la même structure, déjà compilé, et il n'y a plus
qu'à cloner.

## 6. Aller-retour entre le PC et Devin

Un seul principe : **GitHub est la référence**, le PC et Devin sont deux copies.

Avant de commencer à travailler sur le PC :

```
git pull
```

Après une séance de réglage, pour que Devin voie vos modifications :

```
git add -A
git commit -m "Reglages valides sur le robot : Kp -18, Kd -40"
git push
```

Si Devin a poussé une branche (`devin/...`), la récupérer par :

```
git fetch
git checkout devin/nom-de-la-branche
```

Ce qui n'a pas besoin de Git : les valeurs de réglage testées à chaud depuis
`web/pilotage.html`. Notez-les, et reportez les bonnes dans `src/main.cpp` ou
dans `docs/reglage-pid.md` — sinon elles sont perdues au prochain flash.
