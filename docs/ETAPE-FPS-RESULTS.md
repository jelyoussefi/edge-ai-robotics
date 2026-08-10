# Étape FPS : un `meshgrid` mémoïsé, et un sous-échantillonnage refusé

Le plan de simplification désignait quatre suspects pour la chute de 40-50 fps à
14 fps : `points_to_grid`, la fermeture morphologique sur 160x160, `findContours`
et `shrink`. Les quatre sont innocents — ils totalisent 0,43 ms. Le coupable
était ailleurs, et le second remède proposé n'en était pas un.

Scène et configuration, lues sur le sim vif et sur la calibration, identiques
d'un bout à l'autre :

```
calibration : hauteur 1,470 m, tangage -14,53 deg, raster 1280x720
sim         : LANE=0 DETOUR_MAX=2.4 RETURN_TO=1.0 STOP_AT=3.1 RUNUP_MIN=0.7
              CRUISE_VX=0.6 TURN_VX=0.26 OBSTACLE_REP=grid OBSTACLE_SOURCE=ours
compositeur : ROI_MARGIN=0.10 OBSTACLE_MARGIN=0.12 GRID_PASSABLE=0.44
              SHOW_FLOOR=1 SHOW_OBJECTS=1
```

Chaque mesure porte 20 fenêtres de 30 s et sa plage de laps. Une seule ligne de
30 s ne vaut rien : à scène immobile, deux lignes consécutives s'étalent de 12,4
à 14,6 fps. C'est la raison d'être de `scripts/fps_probe.py`, qui prend la
médiane et imprime la dispersion à côté.

---

## 1. Remède 1 : le cache de la grille de rayons — `24153ec`

`FloorDetector._rays()` rend les composantes du rayon caméra de chaque pixel. Il
ne prend aucune profondeur en entrée : c'est une fonction pure des intrinsèques
et du raster. Les intrinsèques sont posées une fois dans `__init__` et ne sont
réassignées nulle part dans le fichier, et une nouvelle calibration impose un
`make calibrate` puis un redémarrage. **Rien ne peut donc l'invalider en cours
de run sauf le raster de profondeur**, qui est la clé du cache.

Il était reconstruit à chaque appel : un `meshgrid` 1280x720 plus deux divisions,
921 600 pixels, pour produire un tableau inchangé depuis le démarrage. Appelé

- une fois **par image** depuis `height_map()`, dès que l'overlay silhouette est
  affiché, ce qui est le cas de la configuration de démonstration ;
- deux fois de plus **par publication de ROI** depuis `deproject()`.

### Avant / après

| 20 fenêtres de 30 s | avant (`e12f4c1`) | après (`24153ec`) |
|---|---|---|
| **FPS composite** | **14,1** (13,3 à 14,6) | **37,0** (36,1 à 37,8) |
| **travail par image, médiane** | **55,2 ms** (50,1 à 59,6) | **22,6 ms** (20,9 à 23,5) |
| **travail par image, p95** | **173,8 ms** (163,6 à 189,7) | **31,8 ms** (30,1 à 33,2) |
| encodage JPEG | 1,8 ms | 3,3 ms |
| laps couverts | 112 à 139 | 141 à 174 |

**Le p95 est la colonne intéressante.** Il tombe de 142 ms, soit quatre fois plus
que la médiane, parce que les pires images étaient celles qui portaient aussi une
publication de ROI et payaient donc la grille deux fois de plus. Le budget par
image n'est plus bimodal : l'écart médiane-p95 passe de 118 ms à 9 ms.

L'encodage JPEG monte de 1,8 à 3,3 ms et **ce n'est pas une régression de
l'encodeur** : c'est le même encodeur appelé 2,6 fois plus souvent sur une
machine qui n'est plus oisive entre deux images.

### Ce qui protège le cache

Les tableaux sortent en lecture seule (`setflags(write=False)`). Un appelant qui
écrirait dedans corromprait toutes les images suivantes d'une manière qui se lit
comme une dérive de calibration, et c'est un chemin chaud où allouer paraîtra un
jour du gaspillage ; mieux vaut que ça lève. Aucune exception en plusieurs
dizaines de minutes de fonctionnement : rien n'y écrit aujourd'hui.

Un changement de raster écrit une ligne de journal plutôt que de passer
inaperçu. Ce n'est pas une erreur — redémarrer la source à un autre `STREAM_RES`
le fait légitimement — mais c'est aussi ce qui arrive quand quelque chose
rééchantillonne la profondeur en silence, et alors toutes les distances calculées
en dessous le sont pour le mauvais objectif.

`_expected_floor()` prend désormais `x` et `y` de `_rays()` au lieu de les
redériver. Les deux dérivations étaient identiques caractère pour caractère,
ce qui est exactement comment une correction d'échelle finit appliquée à une
seule des deux.

## 2. La correction du tableau §6 d'ETAPE-5-RESULTS

Le profil publié dans `docs/ETAPE-5-RESULTS.md` §6 portait deux erreurs. Les
deux allaient dans le sens de justifier le remède 2.

**Il chronométrait un appel que le code ne fait pas.** La sonde mesurait
`points_to_grid(f, l, valid & ~silhouette)` sur l'image entière. Le chemin réel
du sol ne passe pas par là :

```python
ys, xs = np.nonzero(floor_mask)
ff, ll = floor_det.project_many(xs, ys, dw, dh)   # par le PLAN, pas la profondeur
flr = points_to_grid(ff, ll, np.isfinite(ff) & np.isfinite(ll))
```

Fonction différente, taille de tableau différente, coût différent. C'est la
deuxième fois sur ce même code qu'une mesure porte sur un appel que personne ne
fait ; la première était `nav_probe` mesurant contre les rectangles.

**Et 35 points par cellule était une moyenne trompeuse.** Le vrai chiffre est
55,6, mais il ne décrit rien : voir §3.

### Le tableau corrigé

`scripts/grid_profile.py` décompose maintenant le chemin sol en trois postes qui
suivent la source ligne à ligne.

| étape | ms | part | payé |
|---|---|---|---|
| déprojection de l'image entière | 9,24 | 47 % | **plus par le code livré** |
| `points_to_grid`, sol (points 1-D) | **4,67** | 24 % | 0,99 Hz |
| `project_many` sur les pixels de sol | **2,27** | 11 % | 0,99 Hz |
| `nonzero` sur le masque de sol | **1,96** | 10 % | 0,99 Hz |
| `points_to_grid`, objets (avec marge) | 0,80 | 4 % | 0,99 Hz |
| `polygon_from_mask` | 0,51 | 3 % | à la reconstruction |
| `findContours` | 0,22 | 1 % | à la reconstruction |
| `shrink` du polygone | 0,18 | 1 % | à la reconstruction |
| fermeture morphologique | 0,02 | 0 % | 0,99 Hz |
| dilatation | 0,01 | 0 % | 0,99 Hz |
| total chronométré | 19,87 | | |

Trois lectures.

La ligne « déprojection » **ne représente plus le code livré** : la sonde
reconstruit son propre `meshgrid`, le compositeur ne le fait plus. C'est le
remède 1, vu depuis l'outil de mesure.

Le chemin sol totalise **8,90 ms**, et il n'est **pas payé par image**. Il tourne
une fois par publication de ROI, mesuré à **0,99 Hz** sur 10 minutes
(592 reconstructions). 8,9 ms une fois par seconde, c'est 0,9 % du temps, sur une
image sur 37.

Les quatre suspects du plan — morphologie, `findContours`, `shrink` — pèsent
**0,43 ms** à eux tous. Ils étaient hors sujet dès le départ.

## 3. Pourquoi le sous-échantillonnage reste à 1 — `fc252e9`

Le plan proposait un pas de 4 en ligne et en colonne, « qui laisse encore 2 points
par cellule ». Le critère de recette avait été fixé à l'avance : **la grille de
sol doit rester identique à quelques cellules près, pas seulement être plus
rapide.** C'est ce critère qui tranche, et il tranche contre.

### La moyenne cache la distribution

Sur ce salon, le masque de sol porte 284 656 pixels qui atterrissent sur
5120 cellules, soit 55,6 points par cellule **en moyenne**. Mais la densité de
points au sol décroît en 1/z² : le sol proche reçoit des milliers de points par
cellule, le sol lointain en reçoit un ou deux. Un pas uniforme en pixels supprime
donc les cellules lointaines en entier. Il n'en **ajoute jamais** une seule — la
colonne « gagnées » vaut zéro à tous les pas — et il les retire au bord lointain,
exactement là où se mesure le sol praticable.

| pas | points | % | projection + rastérisation | cellules | perdues | gagnées |
|---|---|---|---|---|---|---|
| 1 | 284 656 | 100 | 8,90 ms | 5120 | — | — |
| 2 | 71 225 | 25,0 | 1,52 ms | 4975 | 145 (2,8 %) | 0 |
| **4** | 17 725 | 6,2 | 0,38 ms | 4045 | **1075 (21,0 %)** | 0 |
| 8 | 4 390 | 1,5 | 0,16 ms | 2604 | 2516 (49,1 %) | 0 |

1075 cellules ne sont pas « quelques ». 145 non plus.

### Et le gain n'était pas là

A/B/A sur la pile vive, trois fenêtres de 10 minutes consécutives, scène
inchangée. L'aller-retour est délibéré : une mesure unique après changement ne
distingue pas un effet d'une dérive.

| | pas 1 | pas 4 | pas 1 |
|---|---|---|---|
| **FPS composite** | 37,0 | 37,1 | 36,0 |
| travail par image, médiane | 22,6 ms | 22,4 ms | 22,9 ms |
| travail par image, p95 | 31,8 ms | 31,7 ms | 31,8 ms |
| **cellules de sol publiées** | ~4290 | **3559** | 4221 |
| laps couverts | 141-174 | 182-215 | 215-249 |

**Aucun chiffre de temps ne sort de sa propre dispersion.** La grille de sol, elle,
perd un sixième d'elle-même puis le récupère. Le pas 4 échange un sixième du sol
observé contre un percentile.

`FLOOR_STRIDE` existe donc et vaut **1**. Le knob reste — pas comme un réglage à
essayer, mais comme le support du tableau ci-dessus : c'est le levier le moins
cher si le raster grossit un jour, et l'argument écrit pour ne pas y toucher
aujourd'hui. Les deux se suppriment ensemble ou pas du tout.

**Les objets ne sont délibérément pas sous-échantillonnés.** L'asymétrie est le
fond de l'affaire : une cellule de sol perdue coûte au robot un endroit où il
aurait pu marcher, une cellule d'objet perdue l'envoie dans le meuble.

## 4. Ce qui reste, non attribué

Avant l'arrivée du chemin grille, le même compositeur tournait à **40-50 fps pour
13-23 ms** par image. Il est à 36-37 fps pour 22,9 ms. L'écart s'est refermé mais
n'a pas disparu, et **il n'est pas expliqué** : le total chronométré par
`grid_profile` ne rend compte que de 19,87 ms dont l'essentiel est payé à 1 Hz.
Le reste est le rendu, le blit, le shader, la relecture et le JPEG, présents
depuis toujours, jamais décomposés.

Ce n'est pas une conclusion, c'est une case vide. Personne n'a mesuré le coût de
l'overlay silhouette par image, et c'est le premier endroit où regarder si le
FPS redevient un sujet.

## 5. Commits

| commit | contenu |
|---|---|
| `24153ec` | cache de la grille de rayons, `scripts/fps_probe.py` |
| `fc252e9` | `FLOOR_STRIDE` mesuré et livré à 1, `grid_profile` corrigé |
| `97f2445` | ligne de base refaite sur ce code, avec les laps |

## 6. Reproduire

```bash
python3 scripts/fps_probe.py --since 11m --label ETAT       # sur l'hote

docker compose run --rm --no-deps --entrypoint python3 \
  -v $PWD/scripts:/scripts:ro -v $PWD/common:/opt/edgebot:ro \
  -v $PWD/config:/config:ro perception /scripts/grid_profile.py

# le A/B/A, en changeant une seule chose a la fois
FLOOR_STRIDE=4 docker compose up -d --build compositor
docker compose up -d --build compositor        # retour au defaut
```

`--build` est obligatoire : `docker compose up -d <service>` relance le conteneur
avec l'image existante et mesure du code qui n'est plus dans l'arbre.
