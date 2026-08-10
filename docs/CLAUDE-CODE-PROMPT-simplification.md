# Prompt Claude Code : simplifier edge-ai-robotics sur ROS 2 standard

Colle ce document comme premier message. Conventions du dépôt : français concis
pour la discussion, sans tirets cadratins ; code et commentaires en anglais ;
git est la référence, push après chaque commit ; `t` interdit. Mesurer AVANT de
régler, chaque affirmation porte son chiffre, non mesuré = "unmeasured".

Livrable : une seule archive `edge-ai-robotics.tar.gz` avec un répertoire
`edge-ai-robotics/` au premier niveau.

---

## Objectif

Réduire le code maison au strict minimum et déléguer à ROS 2, Nav2 et aux
briques Intel Robotics AI Suite tout ce qu'elles savent déjà faire. Réduire
aussi la surface de configuration.

Ce n'est PAS une réécriture. Le pipeline tourne aujourd'hui : compositing GPU,
détection, grille d'occupation, patrouille avec évitement, quatre briques suite
en `OBSTACLE_SOURCE=union`. Chaque étape ci-dessous doit laisser la démo
fonctionnelle et mesurée.

Points de départ chiffrés, mesurés le 2026-08-10 :

- `services/compositor/compositor.py` 3393 lignes
- `services/sim/navigator.py` 1327 lignes
- `common/edgebot/floor.py` 729 lignes
- `docker-compose.yml` 879 lignes, **125 variables d'environnement distinctes**

---

## Étape 0 : mesurer avant de toucher

Établir la ligne de base et la commiter dans `docs/SIMPLIFICATION-BASELINE.md` :

- nombre de lignes par fichier, nombre de variables d'environnement
- 60 s de patrouille avec `scripts/nav_probe.py` : raclages, clairance minimale,
  distance parcourue, nombre de `no way round`
- `make lane-probe` : occupation en m², sol libre en m², meilleure voie
- FPS composité et travail par image médian (le compositor le loggue)

Toute étape suivante se compare à ces chiffres. Une simplification qui dégrade
la clairance ou le FPS est refusée.

---

## Étape 1 : supprimer les knobs morts (aucun risque)

Vérifié par grep : ces variables sont déclarées dans `docker-compose.yml` et
lues par **aucun** Python.

- `OBSTACLE_CONF` : 0 référence hors compose. Piège actif, on a cru pendant des
  heures qu'elle réglait le seuil du détecteur. Le vrai seuil est
  `confidence` dans `config/streams.d455.json`, surchargé par `PERCEPTION_CONF`.
- `PATROL_MODE` : 0 référence. Le mode `perimeter` annoncé n'existe pas.
- `CORNER_R` : 0 référence.

À traiter au cas par cas, certaines sont légitimes (ROS, Mesa, Python) :
`ADBSCAN_Z_TOL`, `FLOOR_OCCLUDE_H`, `FM_*`, `GF_PARAMS`, `ITS_GOALS`,
`ITS_MIN_SAMPLES`, `INTEL_FORCE_PROBE`, `LIBVA_DRIVER_NAME`, `MUJOCO_GL`,
`PYTHONUNBUFFERED`, `ROS_DISTRO`, `ROS_DOMAIN_ID`, `XDG_RUNTIME_DIR`.

Pour chacune : soit elle est lue quelque part (fichier de paramètres ROS,
Dockerfile, script), soit elle disparaît. Rapporter le décompte avant/après.

Ajouter `scripts/check_compose.py` un test qui échoue si une variable déclarée
dans compose n'est lue nulle part. Ce défaut doit être impossible à réintroduire.

---

## Étape 2 : un seul budget de marge

Trois marges empilées à trois endroits différents, mesurées aujourd'hui :

```
OBSTACLE_MARGIN 0.12  appliquee par cellule dans points_to_grid, 2 cotes = 0.24
LANE_SLACK      0.08  ajoutee a la demi-largeur dans free_lane,  2 cotes = 0.16
robot                                                                    = 0.44
                                                                   total = 0.84
```

Un couloir réel de 0,9 m laisse donc 3 cm par côté, et le robot refusait des
passages qu'un humain traverse sans y penser. Le commentaire de `navigator.py`
raconte que `GAP_CLEAR` avait déjà causé ce défaut et n'a été qu'à moitié
corrigé.

Exposer **une seule** grandeur, par exemple `CLEARANCE`, calculée une fois et
consommée partout. Supprimer `LANE_SLACK` et `GAP_CLEAR` de l'interface. Logger
au démarrage le couloir minimal exigé en mètres, en toutes lettres.

Critère : `nav_probe` sur 60 s, raclages et clairance minimale au moins aussi
bons que la ligne de base, et un couloir réel de 0,9 m franchi.

---

## Étape 3 : supprimer le chemin polygone, ou l'assumer comme affichage

Mesuré : `self._roi` est écrit par `set_floor` dans `navigator.py` et **relu
nulle part** en `OBSTACLE_REP=grid`, qui est le défaut. Le polygone ne pilote
donc rien.

Et il est instable : `polygon_from_mask` ne garde que la plus grande composante
connexe, `shrink` refait pareil après érosion. Sur une trame sur quatre le sol
praticable passe de 1,5-5,6 m à 1,5-2,4 m alors que la grille reste à
2661-3012 cellules sur 1,9-6,2 m. L'overlay clignote, pas le robot.

Deux options, à trancher par la mesure et non par le goût :

- si rien ne consomme `roi` (vérifier les ponts suite, `suite_compare.py`, la
  console web), le sortir du chemin de décision et le marquer explicitement
  comme diagnostic
- sinon, publier toutes les composantes connexes plutôt que la plus grande

Dans les deux cas le clignotement doit disparaître, et il faut le montrer :
écart-type de la portée du polygone sur 60 s, avant et après.

---

## Étape 4 : déléguer la navigation à Nav2

C'est le gros morceau, et c'est le vrai sujet.

Mesuré aujourd'hui sur le salon : entre x=2,5 et x=4,25 m un couloir libre
existe et fait 0,50 à 0,70 m, mais son axe dérive de -0,95 à -1,08 m. Il ne
reste que 0,35 m de largeur commune sur toute la longueur, pour un robot de
0,44 m. `free_lane` ne teste que des couloirs **rectilignes parallèles à
l'axe**, donc aucun passage courbe n'est franchissable, alors qu'un humain
passe. Ce n'est pas un réglage, c'est la limite de l'algorithme.

Nav2 et le planificateur ITS produisent exactement ces trajectoires. L'étape E1
est close, ITS tourne comme plugin Nav2 et sort un chemin qui contourne.

Travail :

- publier la grille d'occupation comme `nav_msgs/OccupancyGrid` standard plutôt
  que par le format maison de `PATROL_ROI`
- laisser le costmap Nav2 porter l'inflation, à la place de
  `OBSTACLE_MARGIN` côté grille
- `NAV_MODE=goal` suit déjà `SUITE_PATH` ; mesurer ce qui reste de
  `navigator.py` une fois le suivi de chemin délégué

Ce que `navigator.py` doit garder, et ce n'est pas négociable : la loi de cap,
le plafond de lacet, le plancher `TURN_VX` que la politique RL exige pour
continuer à lever les pieds, et le lissage. Ce sont des propriétés du ROBOT,
pas de la mission, et elles ont huit tours de validation derrière elles.

Bloqueur connu à traiter avant : **FastMapping accumule et n'oublie jamais**.
Une personne qui traverse se grave dans la carte. C'est l'étape E3, mémoire qui
oublie, par raytracing, décroissance ou reset.

---

## Étape 5 : corriger deux défauts trouvés aujourd'hui

**Plancher de frein d'élan du mauvais côté de l'hystérésis.**
`RUNUP_MIN=0.25` fois `CRUISE_VX=0.6` donne 0,15 m/s, sous le `TURN_VX=0.26`
auquel la politique cesse de lever les pieds. Le robot s'arrête, la commande
(0,26 ; 0) est un point fixe stable, et il ne repart jamais. Aucun message
d'erreur, il ressemble à un robot bloqué. Le commentaire du code affirme que
0,26 est la vitesse à laquelle la politique marche encore : vrai en régime
établi, faux au redémarrage. Le plancher doit être exprimé en vitesse absolue,
au-dessus du seuil de DÉMARRAGE, qui reste à mesurer.

**Appariement masque/détection cassé.** Dans `detector.py`, `det_masks` est
rempli dans l'ordre de peinture puis `results.sort(key=lambda o: o.range_m)`
réordonne `results` sans réordonner les masques. Le commentaire promet
`mask[k]` apparié à `obstacle[k]`. Personne ne consomme `keep_masks`
aujourd'hui, donc c'est latent. `inst_map` et `inst_meta`, ajoutés le
2026-08-10, contournent le problème en gardant l'ordre de peinture.

---

## Étape 6 : test de non-régression sur les bornes

Motif de bug récurrent du dépôt, trois occurrences documentées : un garde-fou
appliqué sur un seul des deux chemins.

- l'empreinte calculée deux fois, corrigée d'un côté
- `clip_footprints` protégeait `blocked` avec `FOOTPRINT_X_MAX` et rien ne
  protégeait la grille, qui est pourtant ce sur quoi le navigateur pilote.
  Résultat mesuré : `ground grid 1.2-7.8 m` dans une pièce dont le mur du fond
  est à 6,2 m. Corrigé le 2026-08-10 par `OBSTACLE_X_MIN`.

`scripts/grid_probe.py` tourne déjà hors matériel. Y ajouter un cas : masque
contenant des pixels à 1,0 m et à 7,5 m, vérifier que la grille publiée ET les
rectangles publiés restent dans les bornes. Puis auditer les autres garde-fous
pour la même asymétrie.

---

## Ordre et règles

1, 2, 6, 5, 3, puis 4. Les cinq premières sont peu risquées et réduisent la
surface avant de toucher à la navigation.

- Un commit par étape, poussé. Le keyring GNOME refuse de signer sans prompteur
  graphique : `eval "$(ssh-agent -s)" && ssh-add ~/.ssh/id_ed25519 && git push`.
- `--build` obligatoire après toute modification de code. Toucher à `common/`
  invalide toutes les images.
- Une seule source sur la D455 : fermer `realsense-viewer` avant `make`.
  `docker compose run` n'accepte pas `--privileged`, utiliser `docker run`.
- Ne pas régler à l'aveugle. Si un chiffre manque, écrire le script qui le
  mesure, comme `scripts/lane_probe.py`.
- Ne pas modifier la scène entre deux mesures. Chaque déplacement de meuble
  invalide la mesure précédente, ce qui a coûté plusieurs heures le 2026-08-10.

---

## Configuration de référence

Salon, caméra 720p, `height=1.47`, tangage 14,5°. Démo qui tourne :

```
LANE=0 DETOUR_MAX=2.4 RETURN_TO=1.2 STOP_AT=4.0 RUNUP_MIN=0.7 \
  PERCEPTION_CONF=0.25 ROI_MARGIN=0.10 SHOW_FLOOR=1 SHOW_OBJECTS=1 make
```

`make full` ajoute les quatre briques suite avec `GF_DEPTH_RELIABLE=1` et
`OBSTACLE_SOURCE=union`. Après simplification, cette ligne doit être plus
courte, pas plus longue.
