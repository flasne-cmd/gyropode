# Réglage du PID

Le gyropode tient debout grâce à un régulateur PID qui transforme l'angle
d'inclinaison en vitesse des roues. Les gains dépendent du poids, de la hauteur
et des moteurs : ceux de `include/config.h` ne sont qu'un point de départ.

Les trois gains se règlent en direct depuis l'interface web
(<http://192.168.4.1>), gyropode tenu à la main au-dessus du sol.

## Méthode

1. **Mettre Ki et Kd à 0.** Augmenter **Kp** jusqu'à ce que le gyropode réagisse
   franchement quand on le penche. Trop bas : il tombe mollement. Trop haut : il
   oscille rapidement et vibre.
2. Augmenter **Kd** pour amortir ces oscillations. Trop haut : le gyropode
   devient nerveux et bruyant, les moteurs saccadent.
3. Augmenter **Ki** pour supprimer la dérive lente (le gyropode s'éloigne
   doucement dans une direction). Trop haut : oscillation lente de grande
   amplitude.
4. Reporter les valeurs trouvées dans `include/config.h` (`PID_KP`, `PID_KI`,
   `PID_KD`) puis reflasher, sinon elles sont perdues à l'extinction.

## Réglage de l'aplomb

Si le gyropode part systématiquement du même côté alors qu'il paraît vertical,
c'est que le capteur n'est pas parfaitement à plat. Ajuster
`BALANCE_OFFSET_DEG` par pas de 0,5° : positif s'il fuit vers l'avant.

## Symptômes courants

| Symptôme | Cause probable |
| --- | --- |
| Vibration rapide sur place | Kp ou Kd trop élevé |
| Chute molle sans réaction | Kp trop faible, ou moteurs sous-alimentés |
| Dérive lente constante | `BALANCE_OFFSET_DEG` mal réglé, ou Ki trop faible |
| Balancement lent d'avant en arrière | Ki trop élevé |
| Réaction dans le mauvais sens | Moteurs câblés à l'envers, ou capteur retourné |
