# Contexte pour reprendre le projet edge-ai-robotics

Colle ce document en premier message du nouveau chat, puis ajoute ta demande.
Ce document sert aussi de base au `CLAUDE.md` de Claude Code sur la machine.

---

## Le projet

Démo Intel : un Unitree G1 simulé dans MuJoCo est composité en temps réel
par-dessus le flux d'une caméra RealSense D455 fixe, de façon à donner
l'illusion qu'un robot marche dans la pièce filmée. Tout tourne sur un NUC
Panther Lake (iGPU Arc, NPU), en Docker Compose.

Répertoire : `/opt/projects/edge-ai-robotics`

## Direction du projet

1. **Mise en commun avec l'équipe Robotics AI Suite d'Intel** (contact Umesh N
   Papadkar, via Alex Klimovitski). Guide développeur demandé, livre blanc en
   bonus, intégration au catalogue.
2. **Le robot virtuel sera peut-être remplacé par un vrai G1.** Réutiliser un
   maximum de briques de leur suite plutôt que de tout écrire.

La suite est publique et **open source** : dépôt
`github.com/open-edge-platform/edge-ai-suites`, sous-arbre `robotics-ai-suite`,
licence Apache-2.0. Ce point, vérifié à la source, règle la question de licence
pour le guide et rend les briques compilables sans leur dépôt APT.

Le chemin retenu est un service passerelle entre le bus ZeroMQ et ROS 2,
permettant d'adopter leurs briques une par une. Ne rien demander à leur équipe
pour l'instant : arriver à l'échange avec un résultat, pas avec des questions.

## Conventions de travail

- Réponses en **français**, concises, sans tirets cadratins.
- Livraison du code **toujours** en une archive nommée d'après l'étape,
  `edge-ai-robotics-<etape>.tar.gz`, par exemple
  `edge-ai-robotics-B-ros2-bridge.tar.gz`. Répertoire de premier niveau
  `edge-ai-robotics/` dans tous les cas. Extraction :
  `tar xvf "$(ls -t ~/Downloads/edge-ai-robotics-*.tar.gz | head -1)" --strip-components=1`
- Ne pas inclure `models/`, `policies/`, `assets/`, `data/`,
  `config/camera_calibration.json` ni `config/floor_mask.png` dans l'archive.
  Ni `.env`, `.make`, `__pycache__`.
- Vérifier par la mesure, pas par l'inspection visuelle. Chaque correctif est
  étayé par un calcul ou une simulation, et le chiffre est donné.
- Commentaires de code en anglais, expliquant le *pourquoi*, y compris ce qui a
  été essayé et écarté.
- **Toujours lancer les trois vérificateurs avant de livrer** : `py_compile`,
  `scripts/check_compose.py`, `scripts/check_names.py`.

## Répartition chat / Claude Code

Claude Code est installé sur la machine de dev, `CLAUDE.md` à la racine.

- Les **archives d'étape** viennent du chat, qui porte le fil long du projet.
- Les **correctifs de terrain** (une ligne identifiée par un message d'erreur,
  ajustement de réglage) se font sur place avec Claude Code.
- Jamais les deux sur le même fichier en parallèle : une extraction d'archive
  écrase sans avertissement. Un commit git avant chaque intervention. Un
  correctif local qui doit entrer dans la suite revient au chat en `git diff`.

## Architecture

Six services : `bus`, `source`, `sim`, `compositor`, `perception`, `recorder`,
plus `groundfloor` sous profil optionnel `suite` (étape B). Bus ZeroMQ en
étoile, messages msgpack, topics dans `common/edgebot/topics.py`.

- `source` : RealSense D455, 640x480@30, couleur JPEG sur `CAMERA_RGB`,
  profondeur brute sur `CAMERA_DEPTH`. Contient `calibrate.py`.
- `perception` : YOLOv11m-seg sur NPU, ~28 ms/trame. Boîtes sur `DETECTIONS`,
  silhouette combinée 160x120 empaquetée en bits sur `OBSTACLE_MASK`.
- `compositor` : rendu MuJoCo hors écran, composition GPU GLSL, affichage GLFW.
  Calcule le sol praticable, publie sur `PATROL_ROI`.
- `sim` : incarnation MuJoCo + `navigator.py`, navigation pure.
- `groundfloor` : la brique `pointcloud_groundfloor_segmentation` d'Intel plus
  `bridge.py`. `make groundfloor` la démarre, la démo tourne sans elle.

**Bus** : publieurs sur `BUS_PUB` (5555, XSUB), abonnés sur `BUS_SUB` (5556,
XPUB). Utiliser `edgebot.bus.Publisher` et `Subscriber`.

Commandes : `make`, Ctrl-C arrête tout, `make calibrate HEIGHT=1.56`,
`make groundfloor`, `make suite-compare`, `make seg-test`.
Touches : `f` sol + anneau, `s` silhouettes et boîtes, `h` échelle, `r` reset.

## Conventions géométriques, à ne pas réinventer

- Origine du monde : le point du sol à la verticale de la caméra.
- `+x` : axe optique vers l'avant. `+y` : la gauche. `+z` : le haut.
- Caméra libre MuJoCo à azimut 0 : regarde `+x`, `up = +z`, sa droite est `-y`,
  d'où le signe dans `lat = -x * Ze`.
- Profondeur du tampon MuJoCo **inversée sur ce pilote** : le fond vaut 0,0.
  Détecté au démarrage.
- En GLFW, toute annotation sur la copie CPU passe par `gpu.present_image()`.
- Le navigateur raisonne entièrement en coordonnées monde.
- Côté ROS, la passerelle publie la TF `base_link` vers le repère caméra depuis
  la calibration. Le signe du tangage est vérifié par l'intersection de l'axe
  optique avec le sol : `1.56/tan(14.1°) = 6.21 m` devant la caméra.

## Calibration actuelle, étape A faite

La caméra a été inclinée de 14,1 deg (mesure IMU, reproductible à 0,1 deg sur
trois lectures). `config/camera_calibration.json` : hauteur 1,56 m mesurée au
mètre, tangage 14,1, vfov 63,7, `fx=386.4 fy=386.5 ppx=325.6 ppy=239.6`,
`floor_h_tol_m = 0.08`.

- Bord proche du sol : 1,5 m mesuré contre 1,506 calculé par
  `h / tan(tangage + vfov/2)`. Accord à 4 mm.
- Bande utile 1,5 à ~6,1 m contre 2,5 à 6,1 avant, soit 28 % de plus.
- Masque léger : la détection automatique fait le travail (24,6 % de
  couverture), la peinture n'ajuste que ~1 point. Ne jamais repeindre de
  grande surface, un masque lourd avait déclaré 8,4 m de sol dont 3 m de
  mobilier.
- **Le critère se lit sur `floor polygon`**, pas sur `walkable floor` : les
  deux diffèrent de `ROI_MARGIN` (0,25 m), donc 1,5 contre 1,7 m.
- **Ne pas toucher au bouton `MEASURE`** de l'interface de calibration : il
  écrase hauteur ET tangage par la médiane de neuf RANSAC, or le RANSAC est
  non fiable dans cette pièce (19-25 % d'inliers, 79 cm de dispersion). La
  règle, c'est la ligne verte de l'axe optique, tirets tous les mètres.

Incliner davantage reste possible, l'horizon ne sort du cadre qu'à 31,9 deg.
Gain marginal, non prioritaire.

## L'affaire du robot figé, résolue mais à connaître

Symptôme : la ligne `walking out/back` se répète à l'identique, position au
millimètre, pendant des dizaines de secondes, avec une commande `vx` positive.
Ce n'est pas une lenteur, c'est **définitif** : `reached` ne devient jamais
vrai, donc le demi-tour qui relancerait la marche ne part jamais.

Diagnostic mesuré : la politique a une **hystérésis**. Elle marche à 0,35 m/s
en croisière, mais une fois les deux pieds plantés, une commande faible ne la
relance pas. Les quatre blocages observés étaient tous dans la rampe `EASE_IN`
de décélération d'approche, et la cause racine était un échappement d'empreinte
qui commandait `vx = 0` en pleine ligne (défaut corrigé). Depuis, huit tours
de validation sans blocage.

Si le symptôme revient : comparer la commande reconstruite
`TURN_VX + (vx - TURN_VX) * to_go/EASE_IN` au journal, et forcer le demi-tour
est la relance. Un chien de garde (5 cm / 3 s) a existé et a été retiré une
fois la cause corrigée, son code est dans l'historique du chat du 6 août.

## Étape B en cours : la brique groundfloor

### Ce qui est en place

- `services/groundfloor/` : Dockerfile, `bridge.py` (234 lignes),
  `entrypoint.sh`. Profil compose `suite`.
- La passerelle publie profondeur + `camera_info` sur `/<sensor>/depth/...`,
  la TF statique, et convertit le nuage d'obstacles retour en empreintes au
  même format que celles du projet. Lecteur `PointCloud2` écrit contre le
  format binaire, testé au bit près, 60 000 points en 18 ms.
- `make suite-compare` écoute les deux côtés du bus et appose les empreintes.

### L'histoire du paquet, importante pour le guide

1. Le .deb AMR `ros-jazzy-pointcloud-groundfloor-segmentation` existe, mais il
   est lié contre PCL oneAPI, dont les runtimes vivent dans
   `apt.repos.intel.com`.
2. Ce dépôt a fait tourner ses versions : seule la 2026.1.1 reste, et le
   paquet AMR borne à `< 2026.0.0`. **Le binaire est ininstallable pour tout
   le monde** tant qu'Intel ne reconstruit pas un des deux côtés.
3. La solution est le **build source** : le CMakeLists veut le PCL standard
   d'Ubuntu, la liaison oneAPI n'existe que derrière `FUZZTEST_FUZZING_MODE`.
   Le Dockerfile fait un checkout épars épinglé au SHA
   `d35ad014d42e270630cd7866f38e679b7bd8ea4a` (revu le 2026-08-06), 20 Mo avec
   `--filter=blob:none`. La compilation colcon passe en ~16 s.
4. Le fichier de lancement `realsense_groundfloor_segmentation_launch.py`
   existe sous ce nom, arguments `standalone`, `camera_name`, `with_rviz`.
   Paramètres par défaut du nœud : `base_frame: base_link`,
   `max_surface_height: 0.05`, capteur `camera`.
5. `nav2-common` est requis (le launch importe `RewrittenYaml`), pas
   `nav2-bringup` qui tirerait tout Nav2 pour le seul lancement AAEON.
6. Les setup.bash de ROS ne sont pas propres sous `set -u` : l'entrypoint
   relâche `-u` le temps des deux `source` puis le rétablit.

### Où on en est exactement

L'image se construit de bout en bout. Le prochain événement est le **premier
contact réel avec le nœud** : lancement, découverte des topics, premier nuage.
La sortie des trente premières secondes du conteneur est la donnée attendue.
Points encore non vérifiés : que le nœud accepte une caméra qui n'est pas un
vrai driver RealSense, et la correspondance exacte des noms de topics.

### Critère d'arrêt de B

Plan de sol superposable au nôtre à quelques centimètres via
`make suite-compare`, latence ajoutée sous 50 ms.

## Points ouverts, par ordre d'importance

1. **Premier lancement du nœud groundfloor** (ci-dessus).
2. **La fusion d'empreintes engloutit parfois le robot** : `walkable floor`
   s'effondre par moments à une bande 1,7-2,1 m alors que le polygone brut en
   fait 6,9, et les `inside an obstacle footprint by 2.6 m` s'enchaînent quand
   le robot est loin dans la pièce, sa propre silhouette et le mobilier
   fusionnant en un bloc. Sept déclenchements en 40 s observés. À traiter
   après la mesure de B, probablement en excluant la silhouette du robot par
   sa position connue.
3. **`RETURN_TO` par défaut** : vérifier qu'il vaut 1,9 dans
   `docker-compose.yml`. À 1,5 le demi-tour proche se fait 20 cm hors du sol
   praticable, qui commence à 1,7.
4. Commentaire égaré ligne ~76 de `navigator.py` : `# m each side in a gap`
   appartient à `GAP_CLEAR`, pas à `CONFIRM_MIN`.

## Limites connues de la politique RL

Politique de `luckyrobots/g1-manipulation-challenge`, socle manipulation, pas
une marche optimisée. Vérifier sa licence avant publication.

- Suit 0,9 rad/s de lacet à ~75 %. À 1,2 rad/s elle décroche.
- Elle tourne en faisant des pas. `TURN_VX` ne doit pas descendre sous 0,26.
- Hystérésis marche/arrêt, voir plus haut.
- Rayon de virage minimal 0,38 m, écart latéral 0,76 m, demi-tour ~5 s.
- Le G1 mesure 1,31 m en marche.

## Réglages

Navigator : `CRUISE_VX=0.6`, `TURN_VX=0.26`, `TURN_WZ=0.9`, `LOOKAHEAD=2.0`,
`YAW_DAMP=0.5`, `CROSS_MAX=0.35`, `SMOOTH_TAU=0.5`, `OBSTACLE_LOOK=3.5`,
`OBSTACLE_CLEAR=0.45`, `DETOUR_MAX=1.8`, `DETOUR_RUNUP=1.6`, `DETOUR_GAIN=1.25`,
`ROBOT_HALF_WIDTH=0.22`, `GAP_CLEAR=0.20`, `STOP_AT=5.2`, `RETURN_TO=1.9`,
`EASE_IN=0.8`.

Compositor : `RENDER_SCALE=3`, `ROI_MARGIN=0.25`, `OBSTACLE_MARGIN=0.20`,
`STRAIGHTEN_TOL=0.012`, `ROBOT_HEIGHT=1.31`.

Groundfloor : `ROS_DOMAIN_ID=42`, `GF_SENSOR_NAME=camera`, `OBSTACLE_MARGIN`
partagé avec le reste pour que les empreintes soient comparables.

## NPU

Politique et détection sur NPU, `OV_DEVICE=NPU` par défaut. Latence politique
0,48 ms, détection ~28 ms. Pièges déjà levés : reshape `[1, 99]`, `/dev/accel`
à monter, pilote NPU dans l'image.

## Feuille de route

- **A. Incliner la caméra et recalibrer : fait.** 14,1 deg, sol dès 1,5 m,
  +28 % de bande, huit tours de validation.
- **B. Passerelle ROS 2 + brique groundfloor : en cours.** Build OK, premier
  lancement du nœud à faire, puis `suite-compare`.
- **C. ADBSCAN et FastMapping**, presque gratuits une fois B faite. Les
  sources sont dans le même dépôt (`components/adbscan`,
  `components/fast-mapping`), même méthode de build épinglé.
- **D. Matière du guide, au fil de B et C.** L'histoire du paquet cassé et du
  build source en fait déjà partie : c'est exactement le genre de friction
  qu'un guide développeur doit documenter.
- **E. Nav2 et ITS Path Planner, reporté délibérément.**

Le passage au G1 réel n'est pas une étape : `navigator.py` ne connaît ni
MuJoCo ni le rendu, piloter un vrai G1 est un `real_robot.py` qui lit
l'odométrie dans une `Pose` et écrit les vitesses au robot.
