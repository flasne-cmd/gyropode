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
| `Ecfg` / `Ecfd` | compensation de frottement sec gauche / droite | 190 / 190 |

`Kp` et `Kd` doivent changer de signe ensemble, sinon le terme dérivé s'oppose
au terme proportionnel au lieu de l'amortir.

## Méthode, dans cet ordre

Robot **tenu à la main ou sur support** pour les étapes 0 à 4. Ne changer qu'un
paramètre à la fois, et attendre 5 à 10 s avant de juger.

**0. Câblage.** `TestOn`, `TestG 300`, `TestD 300`, `TestOff` : la bonne roue,
dans le bon sens. Tout le reste est inutile tant que ce point n'est pas sûr.

**1. `thetaeq`.** Tenir le robot exactement à sa verticale d'équilibre, lire
`thetaF` : nouveau `thetaeq` = ancien + `thetaF` lu. Refaire jusqu'à obtenir
`thetaF` ≈ 0 (±0,2°) à cette position. Une erreur de 1° ici se traduit par une
dérive permanente qu'aucun gain ne rattrape.

**2. Neutraliser tout le reste** pendant le réglage du PD :
```
Kpv 0    Kdv 0    KVcons 0    Vcons 0    Dec 0
```
Ainsi `thetaC = 0` et le PD travaille seul, sur la verticale.

**3. `Kp` seul, `Kd 0`.** Partir volontairement trop bas et monter :
`Kp -6`, `-9`, `-13`, `-18`, `-25`. À chaque valeur, incliner le robot de ~5° à
la main et lâcher légèrement : il doit pousser dans le sens qui le redresse, de
plus en plus fort. Arrêter à la valeur où il commence à osciller franchement,
puis revenir à environ 70 % de celle-ci.

**4. `Kd` ensuite.** Monter par pas de 5 en gardant le même signe que `Kp` :
`-10`, `-15`, `-20`, `-30`, `-40`. `Kd` doit supprimer l'oscillation laissée par
l'étape 3. Trop de `Kd` = robot nerveux, roues qui grésillent et bruit haute
fréquence : redescendre. Ordre de grandeur utile : `Kd` ≈ 2 à 3 × `Kp`.

**5. Frottement sec `Ecfg` / `Ecfd` et `SeuilRepos`.** C'est souvent la vraie
cause d'un réglage qui « ne prend pas » : avec 190, la commande saute
directement à ±190 sur une plage utile de ±470, ce qui rend le PD presque
binaire près de l'équilibre. Si le robot oscille avec une amplitude à peu près
constante, quelle que soit la valeur de `Kp`/`Kd`, descendre par pas de 20 :
`Ecfg 170` / `Ecfd 170`, puis 150, 130, jusqu'à la valeur la plus basse à
laquelle les roues démarrent encore sans à-coup. Ensuite seulement retoucher
`SeuilRepos` : l'augmenter (20, 25) si des vibrations subsistent à l'arrêt, le
réduire s'il devient mou près de l'équilibre. Puis refaire un passage rapide sur
`Kp`/`Kd`, qui peuvent souvent remonter une fois `Ecf` réduit.

**6. Avance/recul, seulement une fois l'équilibre stable.** Remettre
`KVcons 0.03`, laisser `Kpv 0` / `Kdv 0`, et essayer `Vcons 20`, `40`, `80`.
`KVcons` fixe l'inclinaison prise par unité de consigne : trop grand, le robot
part en avant et tombe ; trop petit, il n'avance pas. `thetaC` (visible dans le
diagnostic) doit rester dans ±5° environ. Si le robot tombe en accélérant,
réduire `VconsRampeMax` à 1 avant de toucher à `KVcons`.

**7. `Kpv` / `Kdv` en tout dernier**, et uniquement si l'avance est déjà
correcte sans eux. Par pas de 0,005 sur `Kpv`, robot tenu, jamais au sol tant
que le signe n'est pas confirmé.

## Notes

- Les valeurs réglées en direct sont perdues à l'extinction : reporter dans
  `src/main.cpp` celles que vous gardez.
- Si le comportement devient incompréhensible, revenir au jeu de valeurs par
  défaut du tableau ci-dessus et reprendre à l'étape 1.
- Le diagnostic affiche `thetaF`, `ec`, `Eccg`/`Eccd`, `thetaC`, `Ecf` : `ec`
  saturé en permanence à ±470 signale un gain trop fort ou un `thetaeq` faux,
  pas un manque de puissance.
