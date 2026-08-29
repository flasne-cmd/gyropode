# Réglage

Tous les paramètres ci-dessous se modifient en direct, sans reflasher, depuis
`web/pilotage.html` ou le port série (une commande par ligne). Les valeurs
retenues doivent ensuite être reportées dans `src/main.cpp`, sinon elles sont
perdues à l'extinction.

## Paramètres

| Commande | Rôle | Valeur par défaut |
| --- | --- | --- |
| `Kp` / `Kd` | PD d'équilibrage (angle → PWM) | −13 / −30 |
| `thetaeq` | angle d'équilibre, verticale réelle du châssis | −9.962 |
| `Kpv` / `Kdv` | boucle de vitesse cascadée | −0.015 / 0.3 |
| `KVcons` | biais direct consigne → inclinaison cible | 0.03 |
| `ConsAvance` / `ConsVirage` | amplitudes des commandes F/B et L/R | 80 / 60 |
| `Vcons` / `Dec` | consigne d'avance et différentiel de virage en continu | 0 |
| `VconsRampeMax` | pente maximale de la rampe sur `Vcons`, par cycle de 5 ms | 2 |
| `Tau1` / `Tau2` | constantes de temps des filtres complémentaires | 100 / 35 |
| `SeuilChute` | coupure de sécurité, en degrés | 50 |
| `SeuilRepos` | zone morte anti-vibration avant compensation de frottement | 15 |

`Kp` et `Kd` doivent changer de signe ensemble, sinon le terme dérivé s'oppose
au terme proportionnel au lieu de l'amortir.

`Ecfd` / `Ecfg` (compensation de frottement sec, 190/190) ne sont pas exposés en
commande : les modifier demande un reflash.

## Calibrage de `thetaeq`

Placer le robot à sa position d'équilibre réelle et lire `thetaF` :
`thetaeq += thetaF`. Le nouveau `thetaeq` ramène `thetaF` à 0 à cette position.

## Précautions

- Faire les réglages robot tenu à la main ou sur support, pas au sol.
- Vérifier le sens des moteurs (`TestOn`, `TestG`, `TestD`, `TestOff`) après
  toute intervention sur le câblage.
- `Kpv` / `Kdv` sont en cours de validation : les faire varier à très petite
  magnitude et jamais au sol tant que le comportement n'est pas confirmé.
- Vibrations à l'arrêt : augmenter `SeuilRepos`. Chutes en accélérant :
  réduire `VconsRampeMax`.
