# Ligne de base avant simplification

Mesures prises le 2026-08-10 sur le salon, avant toute modification. Tout ce qui
suit se compare a ces chiffres. Une simplification qui degrade la clairance ou
le FPS est refusee.

Scene et configuration, a ne pas changer entre deux mesures :

```
salon, D455 720p, height=1.47, tangage 14,5 deg
LANE=0 DETOUR_MAX=2.4 RETURN_TO=1.2 STOP_AT=4.0 RUNUP_MIN=0.7 \
  PERCEPTION_CONF=0.25 ROI_MARGIN=0.10 SHOW_FLOOR=1 SHOW_OBJECTS=1
```

## Taille du code et de la configuration

| | lignes |
|---|---|
| `services/compositor/compositor.py` | **3393** |
| `services/sim/navigator.py` | **1327** |
| `common/edgebot/floor.py` | **729** |
| `docker-compose.yml` | **879** |

**125 variables d'environnement distinctes** dans `docker-compose.yml`.

## Patrouille, 60 s, `scripts/nav_probe.py`

| | |
|---|---|
| poses echantillonnees | 3479 |
| mises a jour de ROI | 58 |
| **raclages** | **721 / 3432 = 21,0 %** |
| **clairance minimale** | **-1,212 m** (au plus mauvais point, (3,50 ; 0,10)) |
| clairance p05 / mediane | -0,535 m / +0,887 m |
| **distance parcourue** | **2,92 m** en avant, **1,83 m** en lateral |
| **`no way round`** | **0** |
| empreintes publiees | mediane 24 (20 a 30), plus grand cote 3,00 m |

Le robot marche, ce qui n'etait pas le cas aux mesures precedentes de la
session : la configuration de reference place le but dans le sol libre.

**Reserve sur les raclages.** `nav_probe` mesure la clairance contre les
rectangles publies, alors que `OBSTACLE_REP=grid` fait piloter le navigateur sur
la grille. Les 21 % sont donc mesures contre une representation qui n'est pas
celle qui decide. Le chiffre reste valable comme reference tant qu'on le compare
a lui-meme, mais il ne dit pas ce que le robot a evite.

## Sol praticable, `make lane-probe`

| | |
|---|---|
| **occupation** | **2726 cellules = 6,82 m2** sur 2,0-6,2 x -2,3-1,8 m |
| **sol libre** | **4214 cellules = 10,54 m2** sur 1,5-5,9 x -2,5-2,6 m |
| polygone `roi` | 12 sommets, 1,62-5,40 m devant, -2,21-2,23 m en travers |
| polygone `raw` | 11 sommets, 1,54-5,83 m devant, -2,43-2,50 m |
| rectangles publies | 24 |
| **meilleure voie absolue** | **y +2,00 m, degagee jusqu'a 6,40 m** |
| **meilleure voie atteignable** | **y +1,10 m, degagee jusqu'a 4,50 m** |
| grille | 160x160, cellule 0,05 m, bornes (0,0 ; 8,0 ; -4,0 ; 4,0) |

## Compositeur

| | |
|---|---|
| **FPS composite** | **13,9 a 14,2** |
| **travail par image, mediane** | **54,0 a 57,1 ms** |
| travail par image, p95 | 175 a 188 ms |
| encodage JPEG | 1,7 a 1,8 ms |

Pour comparaison, plus tot dans la meme session et avant le chemin grille, le
meme compositeur tournait a **40-50 fps pour 13-23 ms** par image. Le cout est
donc dans ce qui a ete ajoute depuis, pas dans le compositing lui-meme. C'est le
chiffre le plus fragile de cette ligne de base et celui qu'aucune etape ne doit
aggraver.

## Un piege trouve en mesurant

`make lane-probe` affiche `knobs in THIS container: LANE=0.39 DETOUR_MAX=1.80
RETURN_TO=1.90 STOP_AT=6.00` alors que la demo tourne avec `LANE=0
DETOUR_MAX=2.4 RETURN_TO=1.2 STOP_AT=4.0`. La sonde demarre un conteneur neuf
sans les surcharges d'environnement et lit donc les defauts de compose, pas la
configuration en cours.

Ses conclusions sur la patrouille -- « the reachable lane ends 1.50 m short of
STOP_AT », « suggested RETURN_TO=1.90 STOP_AT=3.90 » -- portent par consequent
sur une patrouille qui ne tourne pas. Les chiffres de grille et de sol libre,
eux, viennent du bus et restent valables.

C'est le meme defaut que la ligne de base cherche a eviter : lire une valeur
autre que celle qui s'execute.

## Reproduire

```bash
make lane-probe
docker compose run --rm --no-deps --entrypoint python3 \
  -v $PWD/scripts:/scripts:ro -v $PWD/common:/opt/edgebot:ro \
  perception /scripts/nav_probe.py --seconds 60 --label BASELINE
docker compose logs --since 200s compositor | grep composited
```
