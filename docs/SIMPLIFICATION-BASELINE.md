# Ligne de base avant simplification

> **CADUC pour tout ce qui touche la grille : mesures prises a
> `GRID_CELL=0,05 m`,** qui vaut 0,02 m depuis le 2026-08-11. Les sections
> FPS et taille de code restent valables ; occupation, sol libre et largeurs
> de voie ne se comparent plus. Voir `docs/SURPROJECTION-RESULTS.md` III.


Refaite le 2026-08-10 après le travail FPS, parce que la première mouture n'était
comparable à rien : elle portait 14 fps, un taux de raclage entièrement
artefactuel, et aucun numéro de lap. Les chiffres d'origine sont conservés en §8
pour que la trajectoire reste lisible, mais **c'est celle-ci qui sert de
référence** à partir de l'étape 2.

Tout ce qui suit se compare à ces chiffres. Une simplification qui dégrade la
clairance, le sol praticable ou le FPS est refusée.

## 1. Ce qui tournait pendant la mesure

Lues sur le matériel et sur le bus, pas recopiées d'un document. La sonde
`lane_probe` affiche `knobs LIVE from the sim`, ce qui est la seule preuve que
la configuration mesurée est celle qui tourne.

```
calibration : hauteur 1,470 m, tangage -14,53 deg, raster 1280x720, fx 643,6
sim (vif)   : LANE=0.00 DETOUR_MAX=2.40 RETURN_TO=1.00 STOP_AT=3.10
              ROBOT_HALF_WIDTH=0.22 LANE_SLACK=0.08 CRUISE_VX=0.6 TURN_VX=0.26
              RUNUP_MIN=0.7 OBSTACLE_REP=grid OBSTACLE_SOURCE=ours
compositeur : ROI_MARGIN=0.10 OBSTACLE_MARGIN=0.12 GRID_PASSABLE=0.44
              FLOOR_STRIDE=1 SHOW_FLOOR=1 SHOW_OBJECTS=1
```

**Ne pas changer la scène ni ces valeurs entre deux mesures.** Si l'une bouge, la
mesure suivante est une autre mesure et doit le dire.

> **Résolu.** Les blocs de scène de `docs/ETAPE-5-RESULTS.md` et de la première
> ligne de base annonçaient `RETURN_TO=1.2 STOP_AT=4.0` et `height=1.47/1.48`.
> Rien de cela n'a tourné : c'était une configuration antérieure recopiée de
> mémoire. `.env` a été écrit pour la dernière fois à 18h42 et la calibration à
> 19h06, alors que l'étape 0 commence à 20h14 — **ni l'une ni l'autre n'a bougé
> pendant la session**. Toutes les mesures, de l'étape 0 à aujourd'hui, portent
> donc la même scène et les mêmes réglages, et les blocs de scène ont été
> corrigés. Ce qui rend l'étape 0 inutilisable est ailleurs : voir §8.

## 2. Taille du code et de la configuration

| | étape 0 | maintenant |
|---|---|---|
| `services/compositor/compositor.py` | 3393 | **3460** |
| `services/sim/navigator.py` | 1327 | **1415** |
| `common/edgebot/floor.py` | 729 | **729** |
| `docker-compose.yml` | 879 | **891** |
| clés d'environnement distinctes | 125 | **123** |

Le code a grossi de 155 lignes pendant une étape de simplification. C'est
assumé : les ajouts sont le chien de garde du défaut 5, le cache de rayons, le
knob `FLOOR_STRIDE` et surtout les commentaires qui portent les mesures. Les
étapes 2, 6, 3 et 4 sont celles qui doivent faire redescendre ce tableau.

## 3. Patrouille, 60 s, `scripts/nav_probe.py`

Mesurée **contre la grille d'occupation**, celle sur laquelle le navigateur
pilote, et non contre les rectangles publiés.

| | |
|---|---|
| poses échantillonnées | 3457, 59 mises à jour de ROI |
| **laps couverts** | **252 à 254** |
| **raclages** | **0 / 3438 = 0,0 %** |
| **clairance minimale** | **+0,433 m** |
| clairance p05 / médiane | +0,727 m / +1,438 m |
| **distance parcourue** | **2,27 m** en avant, **1,22 m** en latéral |
| empreintes publiées | médiane 24 (21 à 37), plus grand côté 2,94 m |

Largeur du sol praticable, extension latérale du polygone `roi` :

| distance | médiane | n |
|---|---|---|
| 1,5 m | 3,52 m | 59 |
| 2,0 m | **4,40 m** | 59 |
| 2,5 m | 1,20 m | 2 |
| 3,0 m | 1,20 m | 4 |
| 3,5 m | 0,32 m | 1 |
| 4,0 m | 0,30 m | 1 |

Boîte englobante du ROI : x 1,62 à 5,20 m, y -2,19 à +2,17 m.

Le `n` est la mesure la plus utile de ce tableau. Au-delà de 2,5 m le polygone
n'atteint la distance échantillonnée que dans 2 à 4 publications sur 59 : le sol
praticable est large près du robot et essentiellement absent au-delà. C'est le
problème que l'étape 2 doit attaquer, et il est ici sous forme de chiffre.

## 4. Sol praticable, `make lane-probe`

| | |
|---|---|
| **occupation** | **2656 cellules = 6,64 m2** sur 2,0-6,0 x -2,3-1,8 m |
| **sol libre** | **4251 cellules = 10,63 m2** sur 1,5-5,7 x -2,5-2,5 m |
| polygone `roi` | 10 sommets, 1,62-5,50 m devant, -2,18-2,22 m en travers |
| polygone `raw` | 10 sommets, 1,54-5,68 m devant, -2,42-2,47 m |
| rectangles publiés | 24 |
| **meilleure voie absolue** | **y +2,00 m, dégagée jusqu'à 6,40 m** |
| **meilleure voie atteignable** | **y +2,00 m, dégagée jusqu'à 6,40 m** |
| patrouille configurée | RETURN_TO 1,00 -> STOP_AT 3,10 |
| suggéré pour qu'elle marche | RETURN_TO=1,00 STOP_AT=5,80 |
| grille | 160x160, cellule 0,05 m, bornes (0,0 ; 8,0 ; -4,0 ; 4,0) |

Voie absolue et voie atteignable coïncident maintenant, alors qu'à l'étape 0 la
meilleure voie atteignable s'arrêtait à 4,50 m contre 6,40 m pour la meilleure
absolue. **Ce n'est pas une amélioration du robot, c'est la sonde qui a cessé de
mentir.** La bande atteignable se calcule à partir de `LANE` et `DETOUR_MAX` ; à
l'étape 0 `lane_probe` lisait ceux de son propre conteneur (`LANE=0.39`,
`DETOUR_MAX=1.80`, soit -1,41 à +2,19 m) au lieu de ceux du sim (`LANE=0`,
`DETOUR_MAX=2.40`, soit -2,79 à +2,01 m). Le sim n'a pas changé ; la sonde lit
maintenant ses réglages sur le bus. Rien de ceci n'est porté au crédit du travail
FPS.

La sonde signale que la patrouille s'arrête à 3,10 m alors que la voie est
dégagée jusqu'à 6,40 m. Le réglage laisse 3,30 m de sol libre inutilisés.

## 5. Compositeur

Vingt fenêtres de 30 s, même scène, `scripts/fps_probe.py`.

| | étape 0 | **maintenant** |
|---|---|---|
| **FPS composite** | 13,9 à 14,2 | **36,0** (34,8 à 38,2) |
| **travail par image, médiane** | 54,0 à 57,1 ms | **22,9 ms** (20,5 à 23,6) |
| travail par image, p95 | 175 à 188 ms | **31,8 ms** (29,7 à 33,4) |
| encodage JPEG | 1,7 à 1,8 ms | 3,3 ms |
| laps couverts | non relevés | **215 à 249** |

C'est la seule section comparable de bout en bout avec l'étape 0 : le
compositeur ne lit aucun réglage de patrouille, donc le changement de
`RETURN_TO` / `STOP_AT` ne la touche pas.

Le p95 est la colonne qui a le plus bougé, d'un facteur 5,5. Le budget par image
n'est plus bimodal : les pires images étaient celles qui portaient aussi une
publication de ROI et payaient donc la grille de rayons deux fois de plus.

Pour mémoire, avant l'arrivée du chemin grille, le même compositeur tournait à
40-50 fps pour 13-23 ms. **Le chemin grille n'est donc plus le poste dominant,
mais il n'est pas gratuit non plus** : il reste 10 à 20 fps d'écart avec l'état
antérieur, et ils ne sont pas attribués.

L'encodage JPEG passe de 1,8 à 3,3 ms et ce n'est pas une régression de
l'encodeur : c'est le même encodeur appelé 2,6 fois plus souvent sur une machine
qui n'est plus oisive entre deux images.

## 6. Où passe le temps par image, `scripts/grid_profile.py`

| étape | ms | part |
|---|---|---|
| déprojection de l'image entière | 9,24 | 47 % |
| `points_to_grid`, sol (points 1-D) | **4,67** | 24 % |
| `project_many` sur les pixels de sol | **2,27** | 11 % |
| `nonzero` sur le masque de sol | **1,96** | 10 % |
| `points_to_grid`, objets (avec marge) | 0,80 | 4 % |
| `polygon_from_mask` | 0,51 | 3 % |
| `findContours` | 0,22 | 1 % |
| `shrink` du polygone | 0,18 | 1 % |
| fermeture morphologique | 0,02 | 0 % |
| dilatation | 0,01 | 0 % |
| total chronométré | 19,87 | |

**Deux lectures obligatoires de ce tableau.**

La ligne « déprojection » ne représente plus le code livré. La sonde reconstruit
son propre `meshgrid` ; le compositeur ne le fait plus. Dans le code livré cette
ligne coûte les trois multiplications qui restent, pas 9,24 ms.

Les trois lignes du chemin sol totalisent 8,90 ms et **ne sont pas payées par
image** : elles tournent une fois par publication de ROI, mesuré à 0,99 Hz sur
10 minutes (592 reconstructions). 8,9 ms une fois par seconde, c'est 0,9 % du
temps, sur une image sur 37. C'est ce qui reste du p95.

La morphologie, `findContours` et `shrink` que le plan désignait totalisent
0,43 ms. Ils sont hors sujet et le resteront.

## 7. Ce qui a été tenté et refusé

Le sous-échantillonnage du sol, mesuré et rejeté sur le critère annoncé à
l'avance (« identique à quelques cellules près ») :

| pas | points | projection + rastérisation | cellules | perdues |
|---|---|---|---|---|
| 1 | 284 656 | 8,90 ms | 5120 | — |
| 2 | 71 225 | 1,52 ms | 4975 | 145 (2,8 %) |
| 4 | 17 725 | 0,38 ms | 4045 | 1075 (21,0 %) |
| 8 | 4 390 | 0,16 ms | 2604 | 2516 (49,1 %) |

Et en A/B/A sur la pile vive, trois fenêtres de 10 minutes consécutives :

| | pas 1 | pas 4 | pas 1 |
|---|---|---|---|
| FPS | 37,0 | 37,1 | 36,0 |
| travail par image | 22,6 ms | 22,4 ms | 22,9 ms |
| p95 | 31,8 ms | 31,7 ms | 31,8 ms |
| cellules de sol publiées | ~4290 | **3559** | 4221 |
| laps | 141-174 | 182-215 | 215-249 |

Aucun chiffre de temps ne sort de sa propre dispersion, et la grille de sol perd
un sixième d'elle-même puis le récupère. `FLOOR_STRIDE` reste donc à 1.

## 8. La première ligne de base, pour mémoire

Prise le même jour avant tout travail, sur **la même scène et les mêmes réglages**
qu'aujourd'hui (§1). Conservée parce qu'elle documente d'où l'on part, **pas
utilisable comme référence**, et pour trois raisons qui tiennent toutes aux
sondes et non à la configuration :

- ses 21 % de raclages étaient mesurés contre les rectangles publiés alors que
  le navigateur pilote sur la grille. Contre la grille, la même passe donne
  0,0 %. Le chiffre ne mesurait pas le robot, il mesurait la sonde.
- sa clairance minimale de -1,212 m vient de la même erreur.
- sa « meilleure voie atteignable » venait des réglages du conteneur de la
  sonde et non de ceux du sim (§4).
- elle ne porte aucun numéro de lap, donc sa fenêtre de 60 s peut avoir enjambé
  un gel de la politique : c'est précisément ce qui arrivait alors, et c'est la
  seule de ces quatre réserves qui reste sans remède rétroactif.

Deux de ses chiffres survivent au changement de sonde et servent de contrôle
d'invariance : l'occupation passe de 2726 à 2656 cellules et le sol libre de
4214 à 4251. La pièce n'a pas bougé, et aucun des remèdes FPS ne l'a déplacée.

| | étape 0 |
|---|---|
| poses / ROI | 3479 / 58 |
| raclages | 721 / 3432 = 21,0 % (artefact) |
| clairance min / p05 / médiane | -1,212 / -0,535 / +0,887 m |
| distance parcourue | 2,92 m avant, 1,83 m latéral |
| occupation / sol libre | 2726 cellules (6,82 m2) / 4214 (10,54 m2) |
| meilleure voie atteignable | y +1,10 m, dégagée jusqu'à 4,50 m |
| FPS / travail par image | 13,9-14,2 / 54,0-57,1 ms |

## 9. Reproduire

```bash
python3 scripts/fps_probe.py --since 11m --label BASE      # sur l'hote

docker compose run --rm --no-deps --entrypoint python3 \
  -v $PWD/scripts:/scripts:ro -v $PWD/common:/opt/edgebot:ro \
  perception /scripts/nav_probe.py --seconds 60 --label BASE

make lane-probe

docker compose run --rm --no-deps --entrypoint python3 \
  -v $PWD/scripts:/scripts:ro -v $PWD/common:/opt/edgebot:ro \
  -v $PWD/config:/config:ro perception /scripts/grid_profile.py
```

## 10. Suite

Étape 2, le budget de marge unique : remplacer `OBSTACLE_MARGIN`, `LANE_SLACK` et
`GAP_CLEAR` par un seul `CLEARANCE`, et journaliser au démarrage le couloir
minimal en mètres qui en découle. Le critère porte sur la **grille**, pas sur les
rectangles, et se compare aux §3 et §4 ci-dessus.

Puis les étapes 6, 3 et 4.
