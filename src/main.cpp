#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <WiFi.h>
#include <WebSocketsServer.h>
// Bibliotheque "WebSockets" de Markus Sattler (Links2004) -- lib_deps :
// links2004/WebSockets@^2.4.1 (PlatformIO/pioarduino) ou Gestionnaire de
// bibliotheques Arduino IDE (chercher "WebSockets").

/*
 * PROGRAMME FINAL : équilibrage (angle, PD) + pilotage WiFi/WebSocket + batterie
 * -------------------------------------------------------------------------------
 * Fusionne :
 *  - la boucle d'ÉQUILIBRAGE (angle -> PWM moteurs), reprise TELLE QUELLE de
 *    main.cpp car déjà validée par test physique (robot tenu à la main, voir
 *    claude/gyropode-notes.md section "Mise au point de l'asservissement") :
 *    filtre complémentaire MPU6050 + PD (Kp=-12.4, Kd=-88), thetaeq calibré
 *    (voir plus bas), garde-fous anti-NaN, saturation ±470 appliquée juste
 *    avant ledcWrite(), compensation de frottement sec.
 *  - le pilotage à distance WiFi + WebSocket et la surveillance batterie à
 *    3 états (vert/bleu/rouge) d'etape4_wifi_batterie_2.cpp, pour piloter le
 *    robot depuis un téléphone (Web Serial, utilisé par les étapes précédentes,
 *    ne fonctionne pas sur Chrome mobile).
 *
 * CE QUI CHANGE PAR RAPPORT À etape4_wifi_batterie_2.cpp (c'est le coeur de la
 * demande "rajouter la commande pour maintenir le robot stable") :
 *  - etape4 mesurait thetaF (tâche indépendante) mais NE S'EN SERVAIT PAS pour
 *    commander les moteurs -- voir son commentaire d'architecture : "Les deux
 *    boucles ne sont PAS encore couplées [...] rien ne s'en sert dans le calcul
 *    des consignes moteur." Ici thetaF pilote réellement les moteurs : c'est
 *    l'équilibrage proprement dit.
 *  - le PID de vitesse PAR ROUE (KpV/KiV/KdV, asservissement sur une consigne en
 *    imp/s) d'etape4 est retiré. On ne peut pas laisser un asservissement de
 *    VITESSE par roue et la boucle d'ÉQUILIBRE se disputer la même sortie PWM
 *    sans les cascader proprement -- et cette cascade (Kpv/Kdv, angle <- vitesse)
 *    n'a justement pas encore été validée au sol (voir notes). Les codeurs
 *    restent câblés et LUS (vitesseG/vitesseD) uniquement à titre de DIAGNOSTIC
 *    (affichés en série/WebSocket) : ils n'entrent plus dans le calcul de la
 *    commande moteur.
 *  - Kpv/Kdv (boucle de vitesse cascadée, angle <- vitesse) restent à 0 par
 *    défaut, comme dans main.cpp, tant que la boucle d'angle seule n'a pas été
 *    validée au sol. Vcons (consigne "avance") est piloté par les commandes
 *    F/B/S (voir plus bas) plutôt que par les codeurs.
 *  - la commande moteur tourne dans la tâche FreeRTOS dédiée (Te=5 ms, coeur 1,
 *    priorité 2), PAS dans loop() (100 ms) : l'équilibrage a besoin de cette
 *    fréquence pour rester stable, c'est déjà l'architecture (validée) de
 *    main.cpp. loop() ne s'occupe plus que de batterie/diagnostic/réseau.
 *
 * CORRECTION (2026-08-26) : F/B (avance/recule) étaient sans effet -- Vcons ne
 * passait QUE par thetaC = Kpv*erreurV + Kdv*deltaV/Te, toujours nul tant que
 * Kpv=Kdv=0 (défaut de sécurité hérité de main.cpp, jamais mis en évidence
 * faute d'avoir testé F/B avec Kpv/Kdv=0 en même temps). Ajout d'un biais
 * DIRECT (thetaC += Vcons*KVcons) indépendant de cette boucle cascadée
 * toujours désactivée -- voir le commentaire à la déclaration de KVcons.
 *
 * CORRECTION (2026-08-26) : thetaeq recalibré à 9.348 (ancienne valeur
 * -1.652) suite à une mesure physique -- thetaF affichait 11° à la position
 * d'équilibre réelle du châssis au lieu de 0°. Voir le commentaire à la
 * déclaration de thetaeq pour le calcul si un nouveau calibrage est requis.
 *
 * CORRECTION (2026-08-27) : bande morte sur la commande moteur (Eccg/Eccd)
 * observée jusqu'à ±170 -- la compensation de frottement sec (Ecfd/Ecfg,
 * voir plus bas) était calibrée à 115/105, en dessous du seuil réel de
 * décrochage statique du robot assemblé (~170 mesuré par test). Relevé à
 * 190/190 (marge d'environ 20 au-dessus du seuil mesuré). Voir
 * claude/gyropode-notes.md section "Diagnostic bande morte sur la commande"
 * pour le détail du diagnostic. Au passage, le bloc "frottements secs" est
 * réordonné pour être calculé APRÈS "Commande gauche droite" (Ecg/Ecd) au
 * lieu d'avant : dans l'ordre d'origine, la compensation utilisait les
 * valeurs de Ecg/Ecd du cycle précédent (décalage d'un cycle, Te=5 ms,
 * négligeable en pratique mais même famille de bug d'ordre déjà corrigée
 * deux fois ailleurs dans ce fichier -- voir historique ci-dessous).
 *
 * AJOUT (2026-08-27) : thetaC et efv ajoutés à la ligne de diagnostic série/
 * WebSocket, pour permettre de réactiver Kpv/Kdv (boucle de vitesse
 * cascadée, toujours désactivée par défaut) en observant directement son
 * comportement plutôt qu'en le déduisant indirectement de thetaF/ec. Aucun
 * changement de comportement de la commande moteur -- purement diagnostic.
 *
 * AJOUT (2026-08-27) : coupure de sécurité en cas de chute. Si |thetaF|
 * dépasse seuilChute (+/-50° par défaut, réglable en direct via "SeuilChute
 * <val>"), le moteur est coupé et les états internes de la boucle (ec/efv/
 * erreurV/Eccg/Eccd/Vcons/Dec) réinitialisés -- évite de laisser les roues
 * tourner à plein régime contre le sol après une vraie chute, et évite un
 * effet de "windup" (redémarrage en plein régime) quand le robot est
 * redressé. Pas de latch : la commande reprend automatiquement dès que
 * |thetaF| repasse sous le seuil. Exposé au diagnostic (champ "chute=0/1").
 *
 * CORRECTION (2026-08-29) : valeurs par défaut du PD et du calibrage mises à
 * jour suite aux derniers essais (interface de réglage) : Kp=-13, Kd=-30
 * (au lieu de -12.4/-88), thetaeq=-9.962 (au lieu de 9.348 -- nouveau
 * calibrage physique). Kpv/Kdv sortent de leur valeur de sécurité 0/0 pour
 * une réactivation EN COURS DE VALIDATION (Kpv=-0.015, Kdv=0.3) : signe et
 * amplitude encore à confirmer empiriquement à très petite magnitude, robot
 * tenu à la main ou sur support, JAMAIS au sol pour ce test -- voir Tau1/Tau2
 * (déjà 100/35, inchangés) et le seuil de coupure chute (déjà 50, inchangé).
 *
 * ATTENTION BROCHAGE MOTEURS (important, à vérifier avant tout test au sol) :
 * etape4_wifi_batterie_2.cpp utilisait gauche=GPIO25/26, droit=GPIO32/33
 * (convention etape1/etape2 d'origine, avec un commentaire "à vérifier si le
 * câblage a changé"). claude/gyropode-notes.md documente un câblage réel
 * INVERSE, corrigé et confirmé le 2026-08-25, déjà utilisé dans main.cpp :
 * gauche=GPIO32/33, droit=GPIO25/26. C'est CETTE affectation (celle de
 * main.cpp, la plus récente et documentée comme vérifiée sur le typon réel)
 * qui est reprise ici.
 * Avant de poser le robot au sol : relance TestOn puis TestG 300 (doit faire
 * tourner la roue gauche) et TestD 300 (roue droite), TestOff pour repasser en
 * mode normal (voir "Commandes série/WebSocket" plus bas). Une inversion
 * gauche/droite ou +/- sur un moteur transformerait la contre-réaction
 * d'équilibrage en RÉACTION POSITIVE (le robot accélérerait sa chute au lieu de
 * la corriger) -- ne pas tester au sol sans avoir vérifié.
 *
 * Commandes série (USB, 115200 bauds) ou WebSocket (port 81) -- une commande
 * par ligne :
 *   F / B / L / R / S      -> avance / recule / tourne gauche / tourne droite / stop
 *   Kp <val>   Kd <val>    -> gains du PD d'équilibrage (angle -> PWM)
 *   thetaeq <val>          -> angle d'équilibre (verticale réelle du châssis)
 *   Kpv <val>  Kdv <val>   -> gains de la boucle de vitesse cascadée (désactivée par défaut, =0)
 *   KVcons <val>           -> gain du biais direct Vcons -> inclinaison cible (voir CORRECTION ci-dessus)
 *   ConsAvance <val>       -> amplitude de la consigne "avance" (F/B)
 *   ConsVirage <val>       -> amplitude du différentiel de virage (L/R)
 *   Vcons <val>            -> fixe directement la consigne "avance" en continu (curseur UI),
 *                             indépendant de F/B/S -- voir commentaire à son traitement plus bas
 *   VconsRampeMax <val>    -> pente max de la rampe appliquée sur Vcons, en unités par cycle
 *                             Te=5ms (défaut 2) -- augmenter si trop mou à accélérer, réduire
 *                             si le robot tombe encore aux vitesses élevées
 *   Dec <val>              -> fixe directement le différentiel de virage, sans toucher Vcons
 *                             (utilisé par l'UI au relâché de L/R, à la place de S)
 *   Tau1 <val>  Tau2 <val> -> constantes de temps des filtres complémentaires
 *   TestOn / TestOff       -> active/désactive le mode test moteur (coupe l'équilibrage)
 *   TestG <val> TestD <val>-> commande PWM fixe d'une roue, mode test uniquement
 *   TestBatOn / TestBatOff -> active/désactive la simulation de tension batterie
 *   TestBat <val>          -> valeur ADC batterie simulée (mode test uniquement)
 *   SeuilChute <val>       -> seuil de coupure sécurité en cas de chute, +/-deg (défaut 50)
 *   SeuilRepos <val>       -> zone morte anti-vibration sur Ecg/Ecd avant compensation de
 *                             frottement sec (défaut 15) -- augmenter si vibrations à l'arrêt
 *
 *   NOTE : Ecfd/Ecfg (compensation de frottement sec) ne sont pour l'instant
 *   PAS exposés en commande série/WebSocket -- seulement réglables en dur
 *   ci-dessous (donc reflash nécessaire pour les retoucher).
 */

// ---------- Broches moteurs (cf. note brochage ci-dessus) ----------
const int PWMGp = 32; // gauche, sens + (sortie MOTOR-A du driver)
const int PWMGm = 33; // gauche, sens -
const int PWMDp = 25; // droit,  sens + (sortie MOTOR-B du driver)
const int PWMDm = 26; // droit,  sens -

const int frequencePWM = 20000;
const int resolutionPWM = 10;

// ---------- Broches codeurs (DIAGNOSTIC uniquement, voir note d'architecture) ----------
const int PinCodeurG = 16;
const int PinCodeurD_A = 18;
const int PinCodeurD_B = 19;

volatile long compteurG = 0;
volatile long compteurD = 0;

void IRAM_ATTR isrCodeurG()
{
    compteurG++;
}

void IRAM_ATTR isrCodeurD()
{
    if (digitalRead(PinCodeurD_B) == HIGH)
        compteurD++;
    else
        compteurD--;
}

long dernierCompteurG = 0;
long dernierCompteurD = 0;
float vitesseG = 0; // imp/s, affichage diagnostic uniquement (n'entre pas dans la commande)
float vitesseD = 0;

// ---------- Broches LED + batterie (identiques main.cpp/etape4) ----------
const int redpin = 2;
const int bluepin = 0;
const int greenpin = 4;
const int pinbat = 34;

float mesurebat;
volatile bool batterieFaible = false;

const int SEUIL_BATTERIE_FAIBLE = 3500;  // ~6,2V -- LED bleue (avertissement)
const int SEUIL_BATTERIE_COUPURE = 3000; // ~6,0V -- LED rouge + coupure moteur

// Mode de test batterie simulée (voir etape4) : verifier le cablage des 3 LED
// et le seuil de coupure sans avoir a reellement decharger la batterie.
bool modeTestBat = false;
float mesurebatTest = 4095;

// ---------- WiFi (point d'accès) + WebSocket ----------
const char *wifiSSID = "Gyropode";
const char *wifiPassword = "gyropode1"; // >= 8 caracteres pour WPA2
WebSocketsServer webSocket(81);         // port 81 : garde le port 80 libre si on sert du HTML depuis l'ESP32 plus tard

/*****************************************************************************
 * MPU6050 + filtre complémentaire -- identique main.cpp/etape4
 *****************************************************************************/
Adafruit_MPU6050 mpu;
volatile bool imuDisponible = false;
float accX, accY;
sensors_event_t a, g, temp;

float Te = 5, Tau = 100, Tau2 = 35;
float angle_nf, angle_f;
float A, C, A2, C2;
float thetaWF, wz, thetaF;

// CALIBRÉ le 2026-08-29 (interface de réglage) : thetaeq = -9.962. Ancienne
// valeur (9.348, calibrage du 2026-08-26) conservée ici en commentaire pour
// mémoire -- si le calibrage bouge encore, même calcul : thetaeq += thetaF lu
// à la position d'équilibre voulue (thetaF = angle_brut - thetaeq, donc pour
// ramener thetaF à 0 à une position donnée il faut thetaeq = angle_brut =
// ancien_thetaeq + thetaF_mesuré à cette position).
float thetaeq = -9.962; // angle cible (verticale) -- qd trop négatif penche vers l'avant

/*****************************************************************************
 * Boucle de vitesse cascadée (angle <- vitesse).
 *
 * CORRECTION (2026-08-26) : avec Kpv=Kdv=0, thetaC = Kpv*erreurV +
 * Kdv*deltaV/Te valait TOUJOURS 0, quelle que soit Vcons -- les commandes
 * F/B (qui ne font que changer Vcons) n'avaient donc strictement AUCUN effet
 * sur la commande moteur. Ce n'était pas un bug d'etape4/programme_final,
 * c'est une limite héritée telle quelle de main.cpp (jamais remarquée car
 * jamais testée avec Vcons != 0 et Kpv/Kdv = 0 en même temps). Corrigé en
 * ajoutant un biais DIRECT et INDÉPENDANT de la boucle cascadée : KVcons
 * convertit Vcons en quelques degrés de consigne d'inclinaison cible,
 * appliqués en plus de Kpv*erreurV + Kdv*deltaV/Te. Avancer/reculer passe
 * donc par le PD d'équilibrage déjà validé (Kp/Kd), pas seulement par la
 * boucle cascadée.
 *
 * CORRECTION (2026-08-29) : Kpv/Kdv sortent de 0/0 (valeur de sécurité tant
 * que la boucle d'angle seule n'était pas validée au sol) pour une
 * réactivation EN COURS DE VALIDATION (Kpv=-0.015, Kdv=0.3, voir
 * déclaration plus bas) -- signe et amplitude encore à redéterminer
 * empiriquement à très petite magnitude. Robot tenu à la main ou sur
 * support, JAMAIS au sol tant que ce test n'est pas concluant.
 *
 * CORRECTION (2026-08-29) : chutes observées quand Vcons change vite/fort
 * (curseur UI déplacé rapidement vers une vitesse élevée) -- Kdv*deltaV/Te
 * divise par Te=5 ms, donc amplifie x200 tout saut de Vcons d'un cycle à
 * l'autre, ce qui sature thetaC instantanément au lieu de monter
 * progressivement. Ajout de VconsRampe (voir sa déclaration plus bas) :
 * c'est elle, pas Vcons brut, qui alimente erreurV/deltaV/le biais direct
 * ci-dessous.
 *****************************************************************************/
float efv, erreurV, erreurVp, deltaV, thetaC;
float Vcons = 0;
float KVcons = 0.03; // degrés d'inclinaison cible par unité de Vcons (ConsAvance=80 -> ~2.4°)

// ---------- Rampe sur Vcons (2026-08-29) ----------
// Vcons peut changer d'un coup (curseur déplacé vite, ou commande brute) --
// or deltaV/Te (voir plus bas dans controle()) DIVISE par Te=0.005 s, donc
// AMPLIFIE x200 tout saut de Vcons d'un cycle à l'autre. Un saut brutal de
// Vcons fait donc saturer thetaC instantanément à sa limite (+/-10°, voir
// plus bas) -- le robot doit se pencher d'un coup au lieu d'accélérer
// progressivement, et les roues n'ont pas le temps de suivre : chute.
// VconsRampe est la valeur RÉELLEMENT utilisée dans la boucle (au lieu de
// Vcons brut) ; elle progresse vers Vcons d'au plus VconsRampeMax par cycle
// Te, ce qui lisse à la fois le terme dérivé et l'accélération physique,
// quelle que soit la vitesse à laquelle Vcons est modifié. Réglable en
// direct via "VconsRampeMax <val>" -- augmenter si le robot est trop mou à
// accélérer, réduire s'il tombe encore aux vitesses élevées.
float VconsRampe = 0;
float VconsRampeMax = 2; // unités de Vcons max par cycle Te (5 ms) -> ici 400/s

/*****************************************************************************
 * Contrôleur PD d'équilibrage (voir claude/gyropode-notes.md). Kp et Kd
 * doivent changer de signe ENSEMBLE si on les retouche, sinon le terme
 * dérivé s'oppose au terme proportionnel au lieu de l'amortir. Valeurs mises
 * à jour le 2026-08-29 (interface de réglage) : Kp=-13, Kd=-30 (au lieu de
 * -12.4/-88).
 *
 * Kpv/Kdv (boucle de vitesse cascadée ci-dessus) : réactivation en cours de
 * validation à petite magnitude, voir commentaire à leur bloc de
 * déclaration -- pas encore testés au sol.
 *****************************************************************************/
float Kp = -13,
      Kd = -30,
      Kpv = -0.015,
      Kdv = 0.3;

// CORRECTION (2026-08-27) : Ecfd/Ecfg (compensation de frottement sec, voir
// controle() plus bas) relevés de 115/105 à 190/190 -- l'ancienne valeur
// laissait une bande morte sur la commande moteur effective (Eccg/Eccd)
// jusqu'à ±170 (mesurée par test réel), le seuil de décrochage statique réel
// du robot assemblé étant supérieur à l'ancienne compensation. 190 = seuil
// mesuré (~170) + marge d'environ 20. Uniformisé sur les deux roues (pas de
// distinction gauche/droite mesurée cette fois) -- à ajuster séparément si un
// écart net apparaît à l'usage entre les deux moteurs. Voir
// claude/gyropode-notes.md, section "Diagnostic bande morte sur la commande".
float ec, Ecg, Ecd, Eccd, Eccg, Ecfd = 190, Ecfg = 190, Dec = 0;

// ---------- Zone morte anti-vibration (2026-08-29) ----------
// La compensation de frottement sec ci-dessus (Ecfd/Ecfg) est appliquée par
// SAUT selon le signe de Ecg/Ecd (+190 ou -190, pas de valeur intermédiaire).
// Au repos (robot équilibré, Ecg/Ecd proches de 0), le bruit capteur suffit à
// faire osciller ce signe en permanence -> la commande moteur bascule sans
// arrêt entre +190 et -190 (~380 d'amplitude sur une échelle de ±470) alors
// que le robot n'a besoin d'aucune correction réelle : c'est un "limit cycle"
// classique de compensation de frottement, cause probable des vibrations
// observées à l'arrêt. seuilRepos coupe la compensation (Eccg/Eccd = 0, donc
// PWM neutre) tant que |Ecg|/|Ecd| reste sous ce seuil -- au-delà (vraie
// correction demandée), la compensation s'applique normalement, la
// réactivité aux inclinaisons réelles n'est pas affectée. Réglable en direct
// via "SeuilRepos <val>" -- augmenter si les vibrations persistent, réduire
// si le robot devient mou près de l'équilibre.
float seuilRepos = 15;

// ---------- Coupure de sécurité en cas de chute (2026-08-27) ----------
// Au-delà de ce seuil, le robot est considéré hors de portée du PD (pas une
// question de réglage, c'est une chute) -- couper le moteur plutôt que de
// laisser les roues tourner à plein régime contre le sol ou un obstacle.
// float (pas const) pour rester réglable en direct comme les autres seuils,
// commande "SeuilChute <val>".
float seuilChute = 50; // degrés, symétrique (+/- seuilChute)
volatile bool chute = false; // exposé au diagnostic (chute=1/0)

// ---------- Consignes de pilotage (F/B/L/R/S) ----------
float ConsAvance = 80; // amplitude de la consigne "avance/recule" (échelle de ec, pas m/s)
float ConsVirage = 60; // différentiel gauche/droite pour tourner

// ---------- Mode test moteur (bypass équilibrage, cf. main.cpp) ----------
// Permet de diagnostiquer le câblage moteur (voir note brochage) sans que la
// boucle d'équilibrage ne réécrive la commande PWM par-dessus.
volatile bool modeTest = false;
float testDutyG = 0;
float testDutyD = 0;

unsigned long dernierTempsMesureMs = 0;
const unsigned long periodeMs = 100; // période de la boucle batterie/diagnostic/réseau

void controle(void *pvParameters);

/*****************************************************************************
 * Traitement d'une commande complète (une ligne déjà isolée) -- indépendant
 * du transport : appelé par traiterCommandeSerie() (USB) et par
 * onWebSocketEvent() (chaque trame WebSocket TEXTE est déjà une commande
 * complète).
 *****************************************************************************/
void traiterCommande(String ligne)
{
    ligne.trim();
    if (ligne.length() == 0)
        return;

    // ATTENTION à l'ordre : "Kpv"/"Kdv" doivent être testés AVANT "Kp"/"Kd"
    // (sinon "Kpv 1.0".startsWith("Kp") est vrai et c'est Kp qui est modifié à
    // la place de Kpv) -- même piège que TestBat/TestBatOn/TestBatOff plus bas.
    if (ligne.startsWith("Kpv"))
    {
        Kpv = ligne.substring(3).toFloat();
    }
    else if (ligne.startsWith("Kdv"))
    {
        Kdv = ligne.substring(3).toFloat();
    }
    else if (ligne.startsWith("Kp"))
    {
        Kp = ligne.substring(2).toFloat();
    }
    else if (ligne.startsWith("Kd"))
    {
        Kd = ligne.substring(2).toFloat();
    }
    else if (ligne.startsWith("thetaeq"))
    {
        thetaeq = ligne.substring(7).toFloat();
    }
    else if (ligne.startsWith("ConsAvance"))
    {
        ConsAvance = ligne.substring(10).toFloat();
    }
    else if (ligne.startsWith("ConsVirage"))
    {
        ConsVirage = ligne.substring(10).toFloat();
    }
    else if (ligne.startsWith("VconsRampeMax"))
    {
        // ATTENTION à l'ordre : doit être testé AVANT "Vcons" (sinon
        // "VconsRampeMax 5".startsWith("Vcons") est vrai et c'est Vcons qui
        // est modifié à la place -- même piège que Kpv/Kp, TestBat/TestBatOn).
        VconsRampeMax = ligne.substring(13).toFloat();
    }
    else if (ligne.startsWith("Vcons"))
    {
        // AJOUT (2026-08-29) : commande continue pour un pilotage au curseur
        // (voir pilotage_programme_final_3.html) -- contrairement à F/B qui ne
        // peuvent mettre Vcons qu'à +/-ConsAvance ou 0 (S), celle-ci accepte
        // n'importe quelle valeur intermédiaire envoyée en direct pendant que
        // l'utilisateur déplace le curseur. Ne touche pas Dec (virage) : les
        // deux axes restent indépendants, comme F/B/S et L/R le sont déjà.
        Vcons = ligne.substring(5).toFloat();
    }
    else if (ligne.startsWith("Dec"))
    {
        // AJOUT (2026-08-29) : pendant du "Dec 0" envoyé au relâché de L/R par
        // l'UI curseur -- contrairement à "S", ne touche PAS Vcons. Nécessaire
        // depuis que Vcons est piloté en continu par le curseur avance/recule :
        // relâcher L/R ne doit annuler QUE le virage, pas la vitesse en cours.
        Dec = ligne.substring(3).toFloat();
    }
    else if (ligne.startsWith("KVcons"))
    {
        KVcons = ligne.substring(6).toFloat();
    }
    else if (ligne.startsWith("SeuilChute"))
    {
        seuilChute = ligne.substring(10).toFloat();
    }
    else if (ligne.startsWith("SeuilRepos"))
    {
        seuilRepos = ligne.substring(10).toFloat();
    }
    else if (ligne.startsWith("Tau1"))
    {
        // CORRECTION par rapport à main.cpp : la commande "Tau1" y modifiait Tau
        // sans jamais recalculer A/C (contrairement à "Tau2" qui recalcule bien
        // A2/C2) -- la nouvelle constante de temps n'avait donc aucun effet tant
        // qu'on ne relançait pas le programme. Corrigé ici par cohérence.
        Tau = ligne.substring(4).toFloat();
        A = 1 / (1 + Tau / Te);
        C = A * Tau / Te;
    }
    else if (ligne.startsWith("Tau2"))
    {
        Tau2 = ligne.substring(4).toFloat();
        A2 = 1 / (1 + Tau2 / Te);
        C2 = A2 * Tau2 / Te;
    }
    else if (ligne.startsWith("TestBatOn"))
    {
        modeTestBat = true;
    }
    else if (ligne.startsWith("TestBatOff"))
    {
        modeTestBat = false;
    }
    else if (ligne.startsWith("TestBat"))
    {
        // Doit rester APRES TestBatOn/TestBatOff (même piège que ci-dessus).
        mesurebatTest = ligne.substring(7).toFloat();
    }
    else if (ligne.startsWith("TestOn"))
    {
        modeTest = true;
    }
    else if (ligne.startsWith("TestOff"))
    {
        modeTest = false;
        testDutyG = 0;
        testDutyD = 0;
    }
    else if (ligne.startsWith("TestG"))
    {
        // Même saturation que Eccg/Eccd en fonctionnement normal (±470 sur une
        // plage matérielle de ±511) -> évite un rebouclage de duty cycle 10 bits.
        testDutyG = constrain(ligne.substring(5).toFloat(), -470, 470);
    }
    else if (ligne.startsWith("TestD"))
    {
        testDutyD = constrain(ligne.substring(5).toFloat(), -470, 470);
    }
    else if (ligne == "F" || ligne == "f")
    {
        Vcons = ConsAvance;
        Dec = 0;
    }
    else if (ligne == "B" || ligne == "b")
    {
        Vcons = -ConsAvance;
        Dec = 0;
    }
    else if (ligne == "L" || ligne == "l")
    {
        Dec = ConsVirage;
    }
    else if (ligne == "R" || ligne == "r")
    {
        Dec = -ConsVirage;
    }
    else if (ligne == "S" || ligne == "s")
    {
        Vcons = 0;
        Dec = 0;
    }
}

// ---------- Réception USB : accumule les caractères jusqu'à fin de ligne ----------
void traiterCommandeSerie()
{
    static String ligne = "";
    while (Serial.available())
    {
        char c = Serial.read();
        if (c == '\n' || c == '\r')
        {
            traiterCommande(ligne);
            ligne = "";
        }
        else
        {
            ligne += c;
        }
    }
}

// ---------- Réception WiFi : chaque trame texte est déjà une commande complète ----------
void onWebSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length)
{
    switch (type)
    {
    case WStype_CONNECTED:
        Serial.printf("[WS] client %u connecte\n", num);
        break;
    case WStype_DISCONNECTED:
        Serial.printf("[WS] client %u deconnecte\n", num);
        break;
    case WStype_TEXT:
        traiterCommande(String((char *)payload).substring(0, length));
        break;
    default:
        break;
    }
}

/*****************************************************************************
 * Setup
 *****************************************************************************/
void setup()
{
    Serial.begin(115200);

    // WiFi/WebSocket en PREMIER (même choix qu'etape4) : si quoi que ce soit
    // plus loin dans setup() pose problème (capteur absent, etc.), le point
    // d'accès est déjà actif et joignable pour diagnostiquer à distance.
    WiFi.softAP(wifiSSID, wifiPassword);
    Serial.print("Point d'acces WiFi actif -- SSID: ");
    Serial.print(wifiSSID);
    Serial.print(" IP: ");
    Serial.println(WiFi.softAPIP());

    webSocket.begin();
    webSocket.onEvent(onWebSocketEvent);

    ledcAttach(PWMGp, frequencePWM, resolutionPWM);
    ledcAttach(PWMGm, frequencePWM, resolutionPWM);
    ledcAttach(PWMDp, frequencePWM, resolutionPWM);
    ledcAttach(PWMDm, frequencePWM, resolutionPWM);

    pinMode(PinCodeurG, INPUT_PULLUP);
    pinMode(PinCodeurD_A, INPUT_PULLUP);
    pinMode(PinCodeurD_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PinCodeurG), isrCodeurG, RISING);
    attachInterrupt(digitalPinToInterrupt(PinCodeurD_A), isrCodeurD, RISING);

    pinMode(redpin, OUTPUT);
    pinMode(bluepin, OUTPUT);
    pinMode(greenpin, OUTPUT);

    A = 1 / (1 + Tau / Te);
    C = A * Tau / Te;
    A2 = 1 / (1 + Tau2 / Te);
    C2 = A2 * Tau2 / Te;

    while (!Serial)
        delay(10);

    // CORRECTION (reprise d'etape4) : timeout I2C explicite -- sans ça, un bus
    // bloqué (MPU absent, SDA/SCL inversés ou flottants) fait attendre
    // mpu.begin() indéfiniment, y compris le WiFi/WebSocket déjà démarrés.
    Wire.begin(); // SDA=GPIO21, SCL=GPIO22 par defaut sur ESP32
    Wire.setTimeOut(1000);

    // CORRECTION (reprise d'etape4, importante) : un MPU6050 absent/mal câblé
    // ne bloque plus tout le programme (while(1) de main.cpp d'origine) --
    // WiFi/batterie/diagnostics restent actifs pour permettre de diagnostiquer
    // à distance. En revanche, sans IMU il n'y a PAS d'équilibrage possible :
    // la tâche controle() force alors les moteurs à l'arrêt (voir plus bas),
    // sauf en mode test manuel (TestOn/TestG/TestD) qui ne dépend pas de l'IMU.
    imuDisponible = mpu.begin();
    if (!imuDisponible)
    {
        Serial.println("MPU6050 introuvable -- verifier le cablage I2C (SDA=GPIO21, SCL=GPIO22). Poursuite SANS equilibrage (WiFi/batterie/diagnostics restent actifs, moteurs a l'arret hors mode test).");
    }
    else
    {
        mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
        mpu.setGyroRange(MPU6050_RANGE_500_DEG);
        mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);
        Serial.println("MPU6050 detecte -- plage +/-8G, +/-500 deg/s, filtre 260 Hz");
    }

    // Tâche épinglée coeur 1, priorité 2 (> loopTask, priorité 1), période
    // Te=5 ms -- identique à main.cpp : c'est la fréquence qui a été validée
    // pour la stabilité de l'équilibrage.
    xTaskCreatePinnedToCore(
        controle,
        "controle",
        10000,
        NULL,
        2,
        NULL,
        1);

    dernierTempsMesureMs = millis();

    Serial.println("");
    Serial.println("Programme final : equilibrage (angle, PD) + pilotage WiFi/WebSocket + batterie");
    Serial.println("Commandes : F  B  L  R  S  |  Vcons <val>  VconsRampeMax <val>  Dec <val>  |  Kp <val>  Kd <val>  thetaeq <val>  |  Kpv <val>  Kdv <val>  KVcons <val>  |  ConsAvance <val>  ConsVirage <val>  |  Tau1 <val>  Tau2 <val>  |  SeuilChute <val>  SeuilRepos <val>  |  TestOn  TestG <val>  TestD <val>  TestOff  |  TestBatOn  TestBat <val>  TestBatOff");
    delay(100);
}

/*****************************************************************************
 * Tâche de récupération des données et de commande moteur (équilibrage) --
 * reprise de main.cpp, comportement inchangé (déjà validé par test physique) :
 * seule différence, la coupure batterie (batterieFaible) est maintenant
 * calculée dans loop() à partir de la mesure 3 états d'etape4 au lieu du
 * seuil unique d'origine -- le flag lui-même est utilisé exactement pareil.
 *****************************************************************************/
void controle(void *pvParameters)
{
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    while (1) // <- boucle infinie
    {
        // Mode test moteur : court-circuite tout l'équilibrage, applique
        // directement testDutyG/testDutyD -- utile pour vérifier le câblage
        // moteur (voir note brochage en tête de fichier) indépendamment de
        // l'IMU/de l'équilibre.
        if (modeTest)
        {
            ledcWrite(PWMGp, 512 + testDutyG);
            ledcWrite(PWMGm, 512 - testDutyG);
            ledcWrite(PWMDp, 512 - testDutyD);
            ledcWrite(PWMDm, 512 + testDutyD);
            vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(Te));
            continue;
        }

        // Pas d'IMU disponible -> pas d'équilibrage possible : moteurs à
        // l'arrêt par sécurité (voir commentaire dans setup()).
        if (!imuDisponible)
        {
            ledcWrite(PWMGp, 512);
            ledcWrite(PWMGm, 512);
            ledcWrite(PWMDp, 512);
            ledcWrite(PWMDm, 512);
            vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(Te));
            continue;
        }

        // récupère les valeurs de l'accéléromètre, du gyroscope et de la température
        mpu.getEvent(&a, &g, &temp);

        accX = a.acceleration.x;
        accY = a.acceleration.y;
        wz = g.gyro.z * Tau / 1000;
        /***************************filtre complémentaire**************************/
        angle_nf = (atan2((double)-accY, (double)accX)); // angle non filtré, en radian
        angle_f = A * angle_nf + C * angle_f;             // angle filtré, en radian
        thetaWF = A * wz + C * thetaWF;
        thetaF = (thetaWF + angle_f) * 180 / PI - thetaeq;

        // AJOUT (2026-08-27) : coupure de sécurité en cas de chute (|thetaF| >
        // seuilChute, +/-50° par défaut). Au-delà de ce seuil le robot n'est
        // plus dans une plage récupérable par le PD -- ce n'est pas un
        // problème de réglage, c'est une chute réelle (posé/tombé). Sortie
        // anticipée façon modeTest/imuDisponible ci-dessus : coupe le moteur
        // ET réinitialise les états internes de la boucle (ec/efv/erreurV/
        // Eccg/Eccd/Vcons/Dec) plutôt que de laisser seulement Eccg/Eccd à 0
        // pendant que ec continue d'être recalculé -- sans ça, ec resterait
        // saturé à +/-470 (et efv dériverait vers une valeur saturée aussi)
        // pendant toute la durée de la chute, et le moteur repartirait à
        // plein régime dès que le robot repasse sous le seuil (effet de
        // windup) -- même précaution que le garde-fou isfinite() plus bas,
        // appliquée en amont ici. Redémarre automatiquement (pas de latch) :
        // dès que |thetaF| repasse sous le seuil (robot redressé), l'asser-
        // vissement normal reprend au cycle suivant à partir d'un état propre.
        chute = (fabs(thetaF) > seuilChute);
        if (chute)
        {
            ec = 0;
            efv = 0;
            erreurV = 0;
            erreurVp = 0;
            Ecg = 0;
            Ecd = 0;
            Eccg = 0;
            Eccd = 0;
            Vcons = 0;
            VconsRampe = 0;
            Dec = 0;
            ledcWrite(PWMGp, 512);
            ledcWrite(PWMGm, 512);
            ledcWrite(PWMDp, 512);
            ledcWrite(PWMDm, 512);
            vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(Te));
            continue;
        }

        // Rampe sur Vcons (voir déclaration de VconsRampe/VconsRampeMax) --
        // fait AVANT le calcul de la boucle de vitesse pour que erreurV/deltaV
        // ci-dessous voient une consigne qui progresse par petits pas, jamais
        // un saut brutal.
        if (VconsRampe < Vcons)
        {
            VconsRampe += VconsRampeMax;
            if (VconsRampe > Vcons)
                VconsRampe = Vcons;
        }
        else if (VconsRampe > Vcons)
        {
            VconsRampe -= VconsRampeMax;
            if (VconsRampe < Vcons)
                VconsRampe = Vcons;
        }

        // calcul asservissement de vitesse (boucle cascadée -- réactivation en
        // cours de validation, Kpv/Kdv non nuls, voir note d'architecture)
        efv = A2 * ec + efv * C2;
        erreurV = (VconsRampe - efv);
        deltaV = erreurV - erreurVp;
        // CORRECTION (2026-08-26) : + VconsRampe*KVcons -- biais direct qui rend
        // F/B fonctionnels sans dépendre de Kpv/Kdv (voir commentaire à la
        // déclaration de KVcons). Avec Kpv=Kdv=0, thetaC = VconsRampe*KVcons.
        thetaC = Kpv * erreurV + Kdv * deltaV / Te + VconsRampe * KVcons;
        erreurVp = erreurV;

        // Garde-fou (2026-08-26) : borne la consigne d'inclinaison cible à un
        // écart raisonnable de la verticale -- une valeur de KVcons ou de
        // ConsAvance mal réglée ne doit pas pouvoir demander une inclinaison
        // extrême. A ajuster si besoin, mais tester à la main avant le sol,
        // comme pour Kp/Kd (voir claude/gyropode-notes.md).
        if (thetaC > 10)
            thetaC = 10;
        else if (thetaC < -10)
            thetaC = -10;

        /************************commande d'équilibrage*******************************/
        float erreur = thetaC - thetaF;
        ec = (erreur * Kp - Kd * g.gyro.z);

        // Garde-fou anti-NaN/infini (voir main.cpp pour l'historique du bug) :
        // sans ça, une divergence quelque part en amont pollue ec/efv en NaN,
        // qui se propage indéfiniment dans le filtre récursif efv sans jamais
        // être rattrapé par les tests de saturation (toute comparaison avec
        // NaN est fausse).
        if (!isfinite(ec) || !isfinite(efv) || !isfinite(thetaF))
        {
            ec = 0;
            efv = 0;
            erreurV = 0;
            erreurVp = 0;
            angle_f = 0;
            thetaWF = 0;
            Ecg = 0;
            Ecd = 0;
            Eccg = 0;
            Eccd = 0;
        }

        // ec borné avant de reboucler dans efv au cycle suivant -- empêche la
        // dérive numérique quelle que soit la configuration de Kpv/Kdv.
        if (ec > 470)
            ec = 470;
        else if (ec < -470)
            ec = -470;

        /*****************************Commande gauche droite**************/
        // CORRECTION (2026-08-27) : ce bloc est maintenant calculé AVANT la
        // compensation de frottement sec (au lieu d'après) -- dans l'ordre
        // d'origine, "frottements secs" utilisait encore les Ecg/Ecd du cycle
        // précédent, puisqu'ils n'étaient recalculés qu'ici, juste après.
        // Décalage d'un cycle (Te=5 ms) sans conséquence pratique observée,
        // mais même famille de bug d'ordre déjà corrigée deux fois ailleurs
        // dans ce fichier (voir historique en tête de fichier) -- corrigé par
        // cohérence en même temps que le réglage Ecfd/Ecfg ci-dessous.
        Ecg = ec + Dec;
        Ecd = ec - Dec;

        /***************************frottements secs**********************/
        // CORRECTION (2026-08-27) : Ecfd/Ecfg relevés à 190/190 (voir
        // déclaration en tête de fichier) -- l'ancienne valeur (115/105)
        // laissait une bande morte sur la commande moteur effective jusqu'à
        // ±170, mesurée par test réel (le seuil de décrochage statique réel
        // du robot assemblé dépasse l'ancienne compensation).
        //
        // CORRECTION (2026-08-29) : compensation coupée sous seuilRepos (voir
        // déclaration plus haut) -- sans ça, cette compensation est un saut
        // brutal de +/-Ecfd selon le SIGNE de Ecd/Ecg, et le bruit capteur au
        // repos (Ecd/Ecg proches de 0) le fait osciller en permanence -> la
        // commande moteur bascule sans arrêt entre +190 et -190, vibrations à
        // l'arrêt. Au-delà du seuil (vraie correction demandée), comportement
        // inchangé.
        if (Ecd > seuilRepos)
            Eccd = Ecd + Ecfd;
        else if (Ecd < -seuilRepos)
            Eccd = Ecd - Ecfd;
        else
            Eccd = 0;
        if (Ecg > seuilRepos)
            Eccg = Ecg + Ecfg;
        else if (Ecg < -seuilRepos)
            Eccg = Ecg - Ecfg;
        else
            Eccg = 0;

        // Coupure moteur batterie faible -- flag mis à jour par loop() (mesure
        // 3 états), appliqué ici juste avant l'écriture PWM.
        if (batterieFaible)
        {
            Eccg = 0;
            Eccd = 0;
        }

        // Saturation appliquée sur les valeurs réellement envoyées aux moteurs
        // (±470 sur une plage matérielle de ±511 -- ledcWrite() attend un
        // rapport cyclique non signé, "512 + Eccg" négatif provoquerait un
        // rebouclage imprévisible sur 10 bits).
        if (Eccg > 470)
            Eccg = 470;
        else if (Eccg < -470)
            Eccg = -470;
        if (Eccd > 470)
            Eccd = 470;
        else if (Eccd < -470)
            Eccd = -470;

        /********************commandes moteurs******************************/
        ledcWrite(PWMGp, 512 + Eccg);
        ledcWrite(PWMGm, 512 - Eccg);
        ledcWrite(PWMDp, 512 - Eccd);
        ledcWrite(PWMDm, 512 + Eccd);

        // pour garder le même temps de (tache + sommeil)
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(Te));
    }
}

/*****************************************************************************
 * loop() : plus de temps réel ici (déplacé dans controle()) -- juste réseau,
 * commandes, batterie et diagnostic, à 100 ms.
 *****************************************************************************/
void loop()
{
    webSocket.loop(); // à appeler fréquemment -- non bloquant
    traiterCommandeSerie();

    unsigned long maintenant = millis();
    if (maintenant - dernierTempsMesureMs >= periodeMs)
    {
        noInterrupts();
        long cG = compteurG;
        long cD = compteurD;
        interrupts();

        float dt = (maintenant - dernierTempsMesureMs) / 1000.0;

        // DIAGNOSTIC uniquement (voir note d'architecture en tête de fichier) :
        // magnitude/sens bruts du codeur, sans la reconstruction de signe côté
        // gauche qu'utilisaient etape1/etape2 (pas nécessaire ici puisque ces
        // valeurs ne pilotent plus aucun asservissement).
        vitesseG = (cG - dernierCompteurG) / dt;
        vitesseD = (cD - dernierCompteurD) / dt;

        dernierCompteurG = cG;
        dernierCompteurD = cD;
        dernierTempsMesureMs = maintenant;

        // Surveillance batterie + LED 3 états (reprise d'etape4). En mode
        // test (TestBatOn), on utilise la valeur forcée par commande au lieu
        // de l'ADC réel.
        mesurebat = modeTestBat ? mesurebatTest : analogRead(pinbat);
        if (mesurebat < SEUIL_BATTERIE_COUPURE)
        {
            digitalWrite(greenpin, LOW);
            digitalWrite(bluepin, LOW);
            digitalWrite(redpin, HIGH);
        }
        else if (mesurebat < SEUIL_BATTERIE_FAIBLE)
        {
            digitalWrite(greenpin, LOW);
            digitalWrite(bluepin, HIGH);
            digitalWrite(redpin, LOW);
        }
        else
        {
            digitalWrite(greenpin, HIGH);
            digitalWrite(bluepin, LOW);
            digitalWrite(redpin, LOW);
        }
        batterieFaible = (mesurebat < SEUIL_BATTERIE_COUPURE);

        char etatLed = (mesurebat < SEUIL_BATTERIE_COUPURE) ? 'R' : (mesurebat < SEUIL_BATTERIE_FAIBLE) ? 'B'
                                                                                                          : 'V';

        // AJOUT (2026-08-27) : thetaC/efv rendus visibles -- nécessaires pour
        // réactiver Kpv/Kdv (boucle de vitesse cascadée) en connaissance de
        // cause. Jusqu'ici seuls thetaF/ec étaient visibles, ce qui ne permet
        // de voir la boucle cascadée qu'indirectement (via ses effets sur la
        // boucle d'angle). thetaC = consigne d'inclinaison générée par la
        // boucle de vitesse (Kpv*erreurV + Kdv*deltaV/Te + VconsRampe*KVcons) ;
        // efv = proxy filtré de la "vitesse" (filtre A2/C2 appliqué à ec, PAS
        // une mesure encodeur -- voir note d'architecture en tête de fichier).
        // AJOUT (2026-08-29) : VconsRampe rendu visible à côté de Vcons --
        // permet de voir la rampe suivre (ou pas) la consigne brute pendant le
        // réglage de VconsRampeMax (voir sa déclaration).
        // Buffer élargi (300 -> 350) pour thetaC/efv, (350 -> 380) pour
        // chute=%d, (380 -> 400) pour VconsRampe.
        char diag[400];
        snprintf(diag, sizeof(diag),
                 "thetaF=%.3f ec=%.1f Eccg=%.1f Eccd=%.1f | thetaC=%.2f efv=%.1f | Vcons=%.0f VconsRampe=%.1f Dec=%.0f | vG=%.1f vD=%.1f | bat=%.0f led=%c coupure=%d chute=%d%s%s",
                 thetaF, ec, Eccg, Eccd, thetaC, efv, Vcons, VconsRampe, Dec, vitesseG, vitesseD, mesurebat, etatLed,
                 batterieFaible ? 1 : 0, chute ? 1 : 0, modeTestBat ? " (testBat)" : "", modeTest ? " (testMoteur)" : "");

        // Même diagnostic sur les deux canaux : USB (débogage local) et
        // WebSocket (téléphone connecté au point d'accès). Aucun client WS
        // connecté -> broadcastTXT() ne fait simplement rien.
        Serial.println(diag);
        webSocket.broadcastTXT(diag);
    }
}
