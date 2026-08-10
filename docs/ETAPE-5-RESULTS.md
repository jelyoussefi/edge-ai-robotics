# Étape 5 : le plancher de vitesse était du mauvais côté d'une hystérésis

Compte rendu de l'étape 5 du plan `docs/CLAUDE-CODE-PROMPT-simplification.md`,
traitée en premier plutôt qu'en quatrième position parce que la base de
comparaison de l'étape 2 n'était pas reproductible tant que le robot pouvait
geler au milieu d'une mesure.

Scène et configuration, inchangées pendant toute l'étape :

```
salon, D455 720p, height=1.48, tangage 14,5 deg
LANE=0 DETOUR_MAX=2.4 RETURN_TO=1.2 STOP_AT=4.0 RUNUP_MIN=0.7 \
  PERCEPTION_CONF=0.25 ROI_MARGIN=0.10 SHOW_FLOOR=1 SHOW_OBJECTS=1
```

---

## 1. Le symptôme

Le navigateur écrivait une fois par seconde, indéfiniment :

```
walking back: toes at 1.21 m (limit 1.20), +0.065 m off the axis (lane +0.12),
heading +180.9 deg, vx 0.26 m/s, lap 18
```

Le robot ne bougeait pas. Position figée à (1,24 ; 0,07), 0,00 m parcouru en
60 s. Aucune ligne de journal n'était fausse et aucune n'était une erreur :
personne ne comparait la commande au déplacement.

## 2. La mesure : deux seuils, pas un

`scripts/startup_probe.py` construit le même modèle et le même contrôleur que
le sim, puis rampe la vitesse commandée par pas de 0,02 m/s, deux fois : en
montant depuis l'arrêt, puis en descendant depuis la marche. Il mesure la
politique seule, donc le résultat ne dépend ni du mobilier ni de ce que le
navigateur a décidé cette seconde-là.

| | |
|---|---|
| seuil de **DÉMARRAGE** | **0,42 m/s** |
| seuil d'**ARRÊT** | **0,26 m/s** |
| **hystérésis** | **+0,16 m/s** |

Extrait de la rampe montante, 2 s par barreau, un barreau compte comme marche
au-delà de 0,02 m parcourus :

```
  vx cmd   advanced  mean speed   verdict
    0.38      0.006       0.003   stationary
    0.40      0.011       0.005   stationary
    0.42      0.064       0.032   walks
```

Et la descendante :

```
    0.28      0.192       0.096   walks
    0.26      0.252       0.126   walks
    0.24      0.003       0.001   stationary
```

**`TURN_VX=0.26` est exactement le seuil d'arrêt.** Le commentaire du code qui
l'appelle « la vitesse à laquelle la politique marche encore » est vrai d'un
robot qui marche et faux d'un robot arrêté. Tout ce qui tombe dans
[0,26 ; 0,42) tient un robot en mouvement et ne peut pas en relancer un
immobile : la commande (0,26 ; 0) est un point fixe stable.

## 3. Le correctif

`START_VX`, défaut **0,45 m/s**, soit le 0,42 mesuré plus une marge, exprimé en
**m/s absolus**. La forme précédente, `RUNUP_MIN x CRUISE_VX`, est exactement ce
qui l'avait fait passer sous le seuil : 0,25 x 0,6 = 0,15 m/s. Un plancher
défini comme fraction d'un autre réglage peut toujours être tiré sous le seuil
par ce réglage-là.

Le plancher et le chien de garde vivent dans `_smooth`, l'entonnoir unique par
lequel passe chaque sortie du navigateur. Les trois sites qui demandent
`TURN_VX` sont laissés tels quels : poser un garde-fou aux sites d'appel est
précisément ce qui a laissé passer le dernier, deux fois dans ce dépôt, dont
l'empreinte calculée à deux endroits et corrigée d'un seul.

**Chien de garde.** Une commande `vx` au-dessus de 0,01 qui déplace le robot de
moins de `STALL_MIN_MOVE` (0,05 m) sur `STALL_WINDOW` (2,0 s) écrit un WARNING
nommant la valeur commandée, les deux seuils et la valeur vers laquelle il
relève. Mesuré sur `pose.centre` et non `pose.lead` : l'orteil oscille à chaque
pas et se lirait comme un déplacement alors que le robot fait du surplace.

## 4. Avant / après

| | avant | après |
|---|---|---|
| état en fin de mesure | **figé à (1,24 ; 0,07)** | en marche |
| distance parcourue, 60 s | **0,00 m** | **2,31 m** avant, 1,07 m latéral |
| laps couverts | figé au **lap 18**, définitif | **5 à 8**, quatre laps |
| gel détecté | non, aucun message | **oui**, 2 échantillons `STALLED` |
| clairance minimale | +1,310 m (robot immobile, hors obstacles) | **+0,760 m** |
| clairance médiane | +1,364 m | +1,433 m |
| raclages | 0 % | **0 / 3470 = 0,0 %** |

Les clairances d'avant sont celles d'un robot arrêté loin de tout, donc
flatteuses et sans intérêt : c'est la ligne « distance parcourue » qui porte le
résultat.

**Ce qui n'est pas démontré.** Soixante secondes montrent que le gel est détecté
et surmonté deux fois, pas que le robot ne gèlera plus jamais. Le blocage
précédent est survenu au lap 18 ; cette mesure s'arrête au lap 8. Une passe
longue reste à faire.

Une comparaison à ne pas tirer : la passe d'avant correctif où le robot marchait
encore parcourait 2,83 m en 60 s, contre 2,31 m après. Ce n'est pas une
régression mesurée, les deux passes ne couvrent pas les mêmes laps et l'une
d'elles s'est terminée par un gel définitif. La distance par minute n'est pas le
critère ; la capacité à repartir l'est.

## 5. Deux corrections de mesure faites au passage

Elles ne font pas partie de l'étape 5 du plan mais sans elles ses chiffres ne
voulaient rien dire.

**`nav_probe` mesurait contre la mauvaise représentation.** Il comparait le
robot aux rectangles publiés alors que `OBSTACLE_REP=grid` le fait piloter sur
les cellules. Même passe, même robot, deux règles :

| mesuré contre | clairance min | p05 | médiane | raclages |
|---|---|---|---|---|
| rectangles | **-1,212 m** | -0,535 | +0,887 | **21,0 %** |
| **grille** | **+0,341 m** | +0,520 | +1,077 | **0,0 %** |

Distance parcourue 2,83 m contre 2,92 m : le robot faisait la même chose, seule
la règle a changé. Les 21 % de raclages de la ligne de base étaient entièrement
un artefact de mesure. C'est le défaut de l'empreinte calculée deux fois, vu
depuis l'outil de mesure.

**`lane_probe` lisait sa propre configuration.** Il annonçait `LANE=0.39
DETOUR_MAX=1.80 STOP_AT=6.00` pendant que la démo tournait en 0 / 2,4 / 4,0, et
concluait sur une patrouille qui ne tournait pas. Le sim publie désormais les
valeurs vives de son navigateur sur `SIM_TELEMETRY` et la sonde les adopte avant
de rapporter quoi que ce soit, en affichant `knobs LIVE from the sim`, avec un
avertissement bruyant quand elle a dû retomber sur ses défauts.

**Toute mesure porte maintenant ses laps.** `lap` et `stalled` voyagent sur
`SIM_TELEMETRY`, `nav_probe` imprime la plage couverte. Une fenêtre de 60 s peut
enjamber un gel, et deux passes qui rapportent les mêmes chiffres sur des laps
différents ne sont pas la même passe.

## 6. Le profil du FPS, mesuré et non traité

Le plan désignait `points_to_grid`, la morphologie sur 160x160, `findContours`
et `shrink`. `scripts/grid_profile.py` les chronomètre sur une vraie image de
profondeur et une vraie silhouette :

| étape | ms | part |
|---|---|---|
| déprojection de l'image entière | **12,21** | 51 % |
| `points_to_grid`, sol | **9,75** | 41 % |
| `points_to_grid`, objets | 1,28 | 5 % |
| `polygon_from_mask` | 0,33 | 1 % |
| `findContours` | 0,15 | 1 % |
| fermeture morphologique | **0,01** | 0 % |
| dilatation | **0,01** | 0 % |
| total chronométré | 23,73 | |

La morphologie soupçonnée coûte **0,01 ms** : une grille 160x160 n'est rien.
`findContours` et le polygone font un demi-milliseconde à eux deux. Tout est
dans deux opérations plein cadre sur 921 600 pixels : la déprojection, qui
reconstruit un `meshgrid` constant à chaque image, et la rastérisation du
**sol**, qui verse environ 900 000 points dans 25 600 cellules, soit 35 points
par cellule.

Les 23,7 ms chronométrées sur une image de 54 à 57 ms laissent une trentaine de
millisecondes qui sont le rendu, le blit, le shader, la relecture et le JPEG,
présents depuis toujours.

**Rien n'est appliqué** au moment où ces lignes sont écrites. Les deux remèdes
sont l'objet du travail suivant.

> **Correction, ajoutée après coup.** Deux chiffres de ce tableau étaient faux.
> La ligne `points_to_grid`, sol mesurait un appel que le code ne fait pas : la
> sonde rastérisait `valid & ~silhouette` sur l'image entière, alors que le
> chemin réel passe par `nonzero` du masque de sol, `project_many` sur les
> indices, puis une rastérisation de points 1-D. Le total honnête du chemin sol
> est 8,90 ms en trois postes, et il est payé une fois par publication de ROI à
> 0,99 Hz, pas par image. Et 35 points par cellule était une moyenne trompeuse :
> c'est 55,6, réparti en milliers par cellule au près et un ou deux au loin, ce
> qui a fait échouer le sous-échantillonnage sur son propre critère. Le tableau
> corrigé et l'A/B/A sont dans `docs/SIMPLIFICATION-BASELINE.md` §6 et §7.

## 7. Commits

| commit | contenu |
|---|---|
| `d6f9e0a` | point de sauvegarde du travail grille trouvé non commité dans l'arbre |
| `3123b85` | étape 0, ligne de base dans `docs/SIMPLIFICATION-BASELINE.md` |
| `1cb3d22` | étape 1, trois knobs morts retirés, 125 -> 122 variables, détection ajoutée à `check_compose.py` |
| `3676778` | `nav_probe` contre la grille, profil FPS, `lane_probe` sur la config vive |
| `34d49a0` | étape 5, `START_VX` et le chien de garde |

## 8. Reproduire

```bash
docker compose run --rm --no-deps --entrypoint python3 \
  -e POLICY=rl -e ROBOT=g1_walker -e OV_DEVICE=CPU \
  -e POLICY_PATH=/policies/g1_walker/walker.onnx \
  -e PYTHONPATH=/app:/opt/edgebot -v $PWD/scripts:/scripts:ro \
  sim /scripts/startup_probe.py

docker compose run --rm --no-deps --entrypoint python3 \
  -v $PWD/scripts:/scripts:ro -v $PWD/common:/opt/edgebot:ro \
  perception /scripts/nav_probe.py --seconds 60 --label APRES

docker compose run --rm --no-deps --entrypoint python3 \
  -v $PWD/scripts:/scripts:ro -v $PWD/common:/opt/edgebot:ro \
  -v $PWD/config:/config:ro perception /scripts/grid_profile.py
```

## 9. Suite

Ordre révisé : **FPS**, puis **étape 2** (budget de marge unique), puis 6, 3, 4.

Le FPS est traité : `24153ec` cache la grille de rayons et fait passer le
compositeur de 14,1 à 37,0 fps, `fc252e9` mesure le sous-échantillonnage du sol
et le livre désactivé. La ligne de base a été refaite là-dessus, avec les laps,
dans `docs/SIMPLIFICATION-BASELINE.md` — c'est elle qui sert de référence à
partir de l'étape 2, et non les 14 fps de l'étape 0.
