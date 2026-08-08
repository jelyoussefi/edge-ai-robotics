# Étape E2 — le robot suit le chemin ITS

Note de mesure, pas un compte rendu d'étape close : **E2 n'est pas terminée**, et
ce document dit exactement où elle s'arrête et pourquoi.

Ce qui marche : le robot suit un chemin planifié par ITS et atteint des buts. Ce
qui bloque : la carte n'oublie rien, et une brique de planification posée sur une
carte qui n'oublie rien perd ses routes avec le temps.

---

## 1. Ce qui est en place

- **Repères.** `map -> base_link` reste le monde, fixe. Nav2 reçoit `robot_base`
  et `odom` sous des noms à lui. Renommer `base_link` aurait déplacé en silence
  toutes les empreintes, bornes d'arène et valeurs de calibration écrites depuis
  l'étape B — c'est la prédiction de E1 qu'il a fallu défaire.
- **Pose.** Le sim ne publie pas de TF. Il n'a pas de ROS dedans et n'en aura
  pas : le pont `itsplanner`, déjà un nœud ROS, lit `ROBOT_STATE` sur le bus et
  en fait `odom -> robot_base` à 20 Hz plus `/odom`.
- **`NAV_MODE=goal`.** Poursuite pure sur `SUITE_PATH`, puis la **même** loi de
  cap, le même plafond de lacet, le même plancher `TURN_VX` et le même lissage
  que la patrouille. Ce sont des propriétés du robot, pas de la mission.
- **Amorçage.** Le robot naît sous la caméra, hors du sol cartographié. Sans
  amorçage il attendait indéfiniment un plan qui ne pouvait pas exister tant
  qu'il restait là. Le mode but avance donc droit devant jusqu'au sol
  cartographié, ce que la patrouille faisait déjà implicitement.

## 2. Le sélecteur de buts par connexité

La première tentative a échoué parce que les buts étaient choisis un par un :
chacun atteignable depuis le départ, le deuxième inatteignable depuis là où le
premier laissait le robot. **L'atteignabilité est ici deux à deux.**

`make pick-goals` traite le problème comme il se pose : une cellule est
traversable si sa distance à l'occupation la plus proche dépasse le dégagement
demandé, et deux cellules ne sont mutuellement planifiables que dans la **même
composante connexe**. Condition nécessaire, pas suffisante — le roadmap est
probabiliste — mais la réciproque est exacte : deux composantes, aucun chemin.

Il a corrigé ce pour quoi il a été écrit : le but 2, inatteignable à la passe
précédente, a été atteint en 15 s.

### Le robot tient-il dans cette pièce ?

Mesuré dans l'arène de 20 m² :

| dégagement exigé | surface traversable | composantes | plus grande |
|---|---|---|---|
| 0,22 m | 6,53 m² | 3 | 6,45 m² |
| 0,25 m | 5,89 m² | 6 | 5,80 m² |
| **0,30 m** | **4,92 m²** | **1** | 4,92 m² |
| **0,35 m** | **4,09 m²** | **1** | 4,09 m² |
| 0,40 m | 3,22 m² | 3 | 3,20 m² |

À 0,35 m le robot tient sur **4,09 m²**, moins du tiers des ~13 m² de sol libre
cartographié. Il tient, et de justesse. La connexité n'est pas monotone : trois
composantes à 0,22, une à 0,30 et 0,35, trois à 0,40. Le bas fragmente parce que
des cellules marginales ouvrent des poches isolées, le haut parce que les
couloirs se ferment.

**Donc l'inflation à 0,35 m n'affame pas l'ensemble des buts : c'est la valeur
qui garde la pièce connexe.** La baisser ne gagnerait rien et coûterait de la
marge. Laissée telle quelle.

## 3. Le balayage de `min_samples` : l'hypothèse était fausse

Dix requêtes sur le tronçon qui échouait, (6,10 ; 0,42) → (4,14 ; −1,34) :

| `min_samples` | succès | temps (médiane / max) | longueur |
|---|---|---|---|
| **250** | **10/10** | 4 ms / 5 ms | 2,18 m |
| 500 | 10/10 | 3 ms / 6 ms | 2,18 m |
| 1000 | 10/10 | 3 ms / 4 ms | 2,18 m |
| 2000 | 10/10 | 5 ms / 7 ms | 2,18 m |

Leur 250 réussit déjà à tous les coups, en 4 ms, et **tous** les comptes rendent
le même chemin de 2,18 m. La densité du roadmap n'a jamais été la contrainte.
Monter la valeur n'aurait rien acheté tout en ayant l'air d'un correctif : c'est
l'argument entier pour mesurer avant de tourner un bouton. Gardé à 250.

## 4. Le couplage tolérance / accrochage

Le but 3 échouait autrement, et la nuance est le résultat : ni « obstacle », ni
chemin vide, mais un chemin d'**un seul point**, en boucle. C'est le
planificateur qui dit que départ et but tombent sur le même nœud du roadmap.

C'est l'accrochage de départ de E1 vu par l'autre bout. Le même espacement de
nœuds qui pose le départ à 0,30-0,45 m fixe la finesse avec laquelle le
planificateur peut approcher **quoi que ce soit**. La tolérance de 0,45 m et
l'accrochage ne sont pas deux quantités indépendantes : la première est une
conséquence du second.

**Le dernier tronçon est donc le nôtre**, pas le leur : une ligne droite de la
fin de leur plan jusqu'au but exact, **vérifiée contre la carte** à 0,22 m de
dégagement. Si la carte dit que la ligne n'est pas libre, le plan est laissé tel
quel et le robot s'arrête court — un planificateur qui ne peut pas atteindre un
but doit le dire, pas nous voir dessiner le dernier mètre à sa place.

## 5. La carte n'oublie rien, et c'est ce qui bloque maintenant

FastMapping accumule et **ne nettoie jamais**. Sur une seule session :

| | cellules occupées |
|---|---|
| début de session | 8 788 |
| une heure plus tard | 10 278 |
| fin de session | **15 767** |

L'occupation ne fait que croître. Une personne qui traverse la pièce est inscrite
définitivement, et l'espace libre se réduit à mesure que la démo tourne.

Conséquence observée directement : le tronçon de départ, planifié **5 fois sur 5
en 3 ms** au moment du balayage, ne se planifie plus quarante minutes plus tard.
Le sélecteur, le balayage et le tronçon d'approche sont tous corrects et tous
insuffisants, parce qu'ils opèrent sur une carte dont la connexité se dégrade
sous eux.

Le tronçon d'approche l'a d'ailleurs signalé proprement plutôt que de le masquer :
`final 3.95 m to the goal is not clear, stopping short`.

**C'est le point de blocage de E2 et le premier travail de E3.** Il n'appartient
pas au planificateur. Il faut soit un modèle de nettoyage (raytracing des
cellules libres traversées, ce que fait la couche obstacle de Nav2 avec un
`/scan`), soit une décroissance temporelle de l'occupation, soit une remise à
zéro périodique de la carte. Aucun n'est écrit.

## 6. Résultat d'acceptation

Passe complète, buts choisis par le sélecteur, `min_samples` 250 :

| but | longueur planifiée | temps | distance finale | |
|---|---|---|---|---|
| 1 (6,10 ; 0,42) | 3,69 m | 20,0 s | **0,448 m** | atteint |
| 2 (4,14 ; −1,34) | 2,07 m | 15,0 s | **0,449 m** | atteint |
| 3 (3,62 ; 0,34) | 1,33 m | — | — | non atteint |

Zéro « no way round » sur toute la course. La couche réactive s'est déclenchée
trois fois et a relâché à chaque fois.

Deux distances finales par but existent dans les journaux, celle du navigateur et
celle du pont, échantillonnées à des instants différents pendant que le robot
s'arrête. Celle du navigateur est la conservatrice et c'est celle du tableau.

**Non fait :** le troisième but après ajout du tronçon d'approche, et le test de
la chaise. La reprise ultérieure a échoué au premier tronçon pour la raison du
§5, avant que le robot ne bouge, donc il n'y avait aucun chemin actif sur lequel
poser une chaise. Rien n'est affirmé sur l'un ni sur l'autre.

## 7. Reproduire

```bash
make fastmapping                      # la carte, et groundfloor avec
make itsplanner                       # le planificateur
make pick-goals                       # trois buts mutuellement atteignables
NAV_MODE=goal OBSTACLE_SOURCE=union ITS_GOALS="..." \
  docker compose --profile suite up -d --build sim itsplanner
```

Le balayage, dans le conteneur `itsplanner` :

```bash
docker compose exec -T itsplanner bash -lc \
  'source /opt/ros-env.sh && python3 /scripts/its_sweep.py \
     --start 6.10,0.42 --goal 4.14,-1.34 -n 10'
```

Sur une carte déjà longuement accumulée, redémarrer `fastmapping` avant de
mesurer : sinon c'est le §5 qui est mesuré, pas ce qu'on croit mesurer.
