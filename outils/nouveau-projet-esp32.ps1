# Cree un nouveau projet robotique ESP32 avec la meme structure que le gyropode.
#
# Usage, depuis le dossier racine des robots :
#   powershell -ExecutionPolicy Bypass -File chemin\nouveau-projet-esp32.ps1 bras-robot
#
# Le projet est cree dans le dossier courant. Aucune connexion Internet n'est
# necessaire, sauf pour la premiere compilation (telechargement de la plateforme).

param(
    [Parameter(Mandatory = $true)]
    [string]$Nom
)

$ErrorActionPreference = "Stop"

$racine = Join-Path (Get-Location) $Nom
if (Test-Path $racine) {
    Write-Error "Le dossier $racine existe deja."
}

New-Item -ItemType Directory -Path $racine | Out-Null
foreach ($d in @("src", "web", "docs", ".vscode")) {
    New-Item -ItemType Directory -Path (Join-Path $racine $d) | Out-Null
}

@"
[env:esp32dev]
; Fork pioarduino : coeur Arduino ESP32 3.x (API ledcAttach).
platform = https://github.com/pioarduino/platform-espressif32/releases/download/55.03.311/platform-espressif32.zip
board = esp32dev
framework = arduino
monitor_speed = 115200
; Decommenter si Wi-Fi + WebSocket depassent la partition par defaut
;board_build.partitions = huge_app.csv
lib_deps =
"@ | Set-Content -Encoding UTF8 (Join-Path $racine "platformio.ini")

@"
.pio/
.vscode/*
!.vscode/$Nom.code-workspace
*.bin
*.elf
"@ | Set-Content -Encoding UTF8 (Join-Path $racine ".gitignore")

@"
# $Nom

Projet robotique ESP32.

## Materiel

A completer : carte, capteurs, driver moteur, alimentation.

## Compilation

``````
pio run
pio run --target upload --upload-port COM3
pio device monitor --port COM3 --baud 115200
``````

## Documentation

- ``docs/cablage.md`` : brochage reel
- ``docs/reglage-pid.md`` : parametres et methode de reglage
"@ | Set-Content -Encoding UTF8 (Join-Path $racine "README.md")

@"
#include <Arduino.h>

void setup()
{
    Serial.begin(115200);
}

void loop()
{
    delay(100);
}
"@ | Set-Content -Encoding UTF8 (Join-Path $racine "src\main.cpp")

"# Cablage $Nom`n`n| Fonction | GPIO | Remarque |`n| --- | --- | --- |`n" |
    Set-Content -Encoding UTF8 (Join-Path $racine "docs\cablage.md")

"# Reglage $Nom`n`n| Parametre | Valeur | Role |`n| --- | --- | --- |`n" |
    Set-Content -Encoding UTF8 (Join-Path $racine "docs\reglage-pid.md")

@"
{
  "folders": [
    {
      "name": "$Nom",
      "path": ".."
    }
  ],
  "settings": {}
}
"@ | Set-Content -Encoding UTF8 (Join-Path $racine ".vscode\$Nom.code-workspace")

Push-Location $racine
git init -q
git add -A
git commit -q -m "Structure initiale du projet $Nom"
Pop-Location

Write-Host "Projet cree : $racine"
Write-Host "Ouvrir .vscode\$Nom.code-workspace dans VS Code, puis : pio run"
