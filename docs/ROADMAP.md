# Feuille de route

## Nom des livraisons

Chaque archive porte l'étape à laquelle elle appartient :

    edge-ai-robotics-A-camera-tilt.tar.gz
    edge-ai-robotics-B-ros2-bridge.tar.gz
    edge-ai-robotics-C-adbscan-fastmapping.tar.gz
    edge-ai-robotics-D-guide.tar.gz
    edge-ai-robotics-E-nav2.tar.gz

Le répertoire de premier niveau reste `edge-ai-robotics/` dans tous les cas, donc
l'extraction ne change pas :

    tar xvf "$(ls -t ~/Downloads/edge-ai-robotics-*.tar.gz | head -1)" \
        --strip-components=1

---

Ordre décidé, du moins coûteux au plus engageant. Chaque étape a un critère
d'arrêt clair, et aucune ne demande quoi que ce soit à une équipe extérieure.

---

## Étape A. Incliner la caméra et recalibrer

**Pourquoi en premier.** C'est une demi-heure de travail, aucun code, et cela
change tout ce qui suit. À 0,1 degré le sol ne commence qu'à 2,50 m et la bande
utile fait 3,50 m de profondeur. À 15 degrés le sol commence à 1,46 m et la
bande utile passe à 4,54 m, soit **30 % de sol praticable en plus**. La portée
lointaine ne perd rien : elle est bornée par le capteur, autour de 6 m, pas par
le champ de vision.

Cela desserre aussi la contrainte de scène. Aujourd'hui la table, les chaises et
le comptoir forment une barrière que la fusion agglomère sur 2,8 m et devant
laquelle le robot s'arrête. Avec une bande plus profonde, un parcours passant
devant le mobilier redevient possible.

Et toute évaluation de brique faite ensuite portera sur une scène correcte,
plutôt que sur une scène qui bride les résultats.

**Comment.** Incliner physiquement la caméra de 15 degrés vers le bas, puis
`make calibrate HEIGHT=1.56`. Le tangage vient de l'IMU, donc il sera mesuré
seul. Repeindre le masque de sol, l'ancien ne correspondra plus.

**Critère d'arrêt.** La ligne `walkable floor` annonce une zone commençant sous
1,6 m, et le robot fait au moins deux tours complets sans `no way round`.

---

## Étape B. Passerelle ROS 2 et une seule brique

**Pourquoi ensuite.** Il reste une inconnue qui coûterait cher à découvrir tard :
leurs paquets AMR sont des Debian **ROS 2 Humble**, donc Ubuntu 22.04, alors que
les images du projet sont en 24.04 et Python slim. Une soirée de test lève le
doute définitivement, sans rien demander à personne.

**Comment.** Un service `ubuntu:22.04` avec Humble et
`ros-humble-*-groundfloor-segmentation`, à côté des services actuels, plus une
passerelle traduisant entre le bus ZeroMQ et les topics ROS. Cette brique est
choisie parce qu'elle consomme un nuage de points et rend un plan, donc son
résultat est directement comparable à la détection de sol actuelle.

**Critère d'arrêt.** Le plan de sol calculé par leur brique est superposable au
tien à quelques centimètres près, et la latence ajoutée reste sous 50 ms.

**Si ça échoue.** On le sait pour un coût faible, la démo n'a pas bougé, et le
diagramme cible devient une intention plutôt qu'un plan.

---

## Étape C. Deux briques de plus, presque gratuites

Une fois la passerelle en place, `ADBSCAN` et `FastMapping` consomment le même
nuage de points et ne demandent que leur câblage. FastMapping donne une carte
d'occupation persistante au lieu d'un polygone recalculé à chaque changement de
scène.

**ADBSCAN : mesuré, voir `docs/ETAPE-C-RESULTS.md`.** L'hypothèse « ADBSCAN
remplace la fusion d'empreintes » est **rejetée par la mesure**. Branché tel
quel il rend un amas de la taille de l'arène dans 60 % des trames ; chaîné
derrière `groundfloor`, rogné à l'arène et débarrassé du résidu de plan
(`GF_Z_LOW`), il atteint 39–42 % d'appariement à 0,50 de recouvrement, et ce qui
reste n'est pas rattrapable par du réglage — les deux détecteurs voient des
choses différentes. Nous tenons les objets sémantiques (table, bloc cuisine)
qu'ils fondent dans une masse ; ils tiennent le pilier et le comptoir proches à
70–75 %, dont nous n'avons aucune empreinte parce qu'aucune classe COCO ne les
couvre.

**Critère d'arrêt, révisé.** ~~Le robot évite les mêmes obstacles
qu'aujourd'hui, avec les obstacles venant de chez eux.~~ C'était un critère de
**substitution**, et la substitution échoue. À la place : le navigateur consomme
l'**union** de nos empreintes et de leurs clusters, et le robot évite au moins
les obstacles qu'il évite aujourd'hui, **plus** le pilier proche droit qu'il ne
voit pas. Un obstacle vu par l'un des deux est un obstacle : un faux positif
coûte un détour, un faux négatif une collision.

---

## Étape D. Rassembler la matière du guide

À faire au fil des étapes B et C, pas à la fin. Un tableau des dispositifs :
quel modèle, quelle cible, quelle latence, quel obstacle rencontré. Plus les
pièges déjà documentés, forme d'entrée dynamique refusée par le NPU, `/dev/accel`
non monté, format de profondeur MuJoCo, convention de profondeur inversée.

C'est exactement ce qu'un guide développeur doit contenir, et c'est ce qui a été
demandé. L'écrire au fil de l'eau évite de le reconstituer de mémoire.

**Première version écrite : [`DEVELOPER-GUIDE.md`](DEVELOPER-GUIDE.md)**, en
anglais, compilée depuis les deux comptes rendus de mesure, les pièges de
`CLAUDE.md` et l'historique des commits. Elle couvre la compilation des briques
depuis les sources (le .deb AMR ininstallable, le checkout épars épinglé, l'image
de base partagée), les pièges d'intégration (QoS, `set -u`, invalidation par
`common/`, session X), le réglage au capteur (`max_surface_height` contre le
bruit du D455, la bande de résidu de plan, le rognage d'arène), la méthodologie
de comparaison (politique contre perception, validité par session, dérive
temporelle) et le résultat de composition (chaînage, union, confirmation par
source).

La section « dispositifs » (§6) est écrite et **mesurée**, pas reconstituée de
mémoire : latence de la politique et du détecteur sur NPU / iGPU / CPU (la
politique est 4× plus rapide sur CPU que sur NPU, le détecteur 22× plus rapide
sur NPU que sur CPU), coût CPU par conteneur, épinglage pilote NPU 1.35.0 /
`libze1` 1.28.2 avec son mode de panne, et les pièges MuJoCo / GLFW. Ce qui n'a
pas pu être mesuré y est marqué comme tel : coût GPU par trame, attribution CPU
dans `perception`, latence par trame, consommation, thermique.

---

## Étape E. ITS Path Planner, en trois phases

Le report était justifié et la raison tient toujours : un greffon Nav2 n'arrive
pas seul, il demande une carte de coûts, un arbre TF complet, une localisation,
une empreinte et une cinématique de robot, et Nav2 pilote par `cmd_vel`, ce qui
recouvre la patrouille actuelle. La réponse n'est pas d'attendre, c'est de
**découper par ce que chaque phase met en jeu**. E1 ne risque rien, E2 met la TF
en jeu, E3 met le robot en jeu.

Ce que le repérage a établi avant d'écrire la moindre ligne :
`its_planner::ITSPlanner` est un **greffon pluginlib** `nav2_core::GlobalPlanner`,
pas un nœud. Il ne tourne pas seul, il est chargé par `planner_server` (nœud
lifecycle) et lit un `Costmap2DROS` reçu dans `configure()`, sans s'abonner à
quoi que ce soit lui-même. La moitié planification de Nav2 est donc nécessaire
(`nav2_planner`, `nav2_costmap_2d`, `nav2_lifecycle_manager`, `nav2_msgs`) ;
`nav2_bringup` reste exclu.

### E1. Planifier sur la carte, sans robot dans la boucle — **faite**

Quatrième brique, `services/itsplanner/`, quatrième consommateur de `ros-base`.
Le planificateur produit un chemin sur la carte FastMapping, publié sur
`SUITE_PATH` et dessiné sous la touche `m` avec la carte contre laquelle il a
été calculé. **Personne ne le consomme** : le navigateur ne le lit pas.

Critère : chemin dans l'espace libre avec au moins 0,22 m de dégagement.
**Atteint**, 0,267 à 0,341 m sur cinq requêtes. Détail et réserves dans le
compte rendu de commit ; voir `docs/images/etape-e1-its-path.png`.

**Décision de nommage, à défaire en E2.** `base_link` est ici le MONDE, fixe,
au sol sous la caméra. Nav2 entend par `base_link` le CORPS DU ROBOT et attend
`map -> odom -> base_link`. E1 réconcilie les deux en les rendant numériquement
identiques : FastMapping publie sa grille dans `map` (`FM_MAP_FRAME`) et le pont
publie une TF statique **identité** `map -> base_link`. Rien ne bouge
géométriquement, seule l'étiquette change, et Nav2 voit un robot immobile à
l'origine du monde -- ce qui est vrai, il n'y a pas de robot.

### E2. Mettre le robot dans le repère, toujours sans le piloter

Remplacer l'identité par `map -> odom -> base_link` alimentée par la pose du
sim, pour que `base_link` redevienne le corps du robot au sens de Nav2 et que le
costmap suive le robot. Publier `/odom` depuis la même source.

**Fait, sauf le test de la chaise et le troisième but.** `map -> base_link` reste
le monde ; Nav2 reçoit `robot_base` et `odom` sous des noms à lui. La pose vient
du bus, pas d'un ROS embarqué dans le sim. `NAV_MODE=goal` suit `SUITE_PATH`
avec la même loi de cap, le même plafond de lacet et le même plancher `TURN_VX`
que la patrouille.

Buts choisis par `make pick-goals`, qui les sélectionne comme un ENSEMBLE
connexe : c'est ce que la première tentative avait raté, chaque but étant
atteignable depuis le départ mais pas depuis le précédent.

| but | longueur planifiée | temps | distance finale | |
|---|---|---|---|---|
| 1 (6,10 ; 0,42) | 3,69 m | 20,0 s | 0,448 m | atteint |
| 2 (4,14 ; −1,34) | 2,07 m | 15,0 s | 0,449 m | atteint |
| 3 (3,62 ; 0,34) | 1,33 m | — | — | non atteint |

Zéro « no way round » sur toute la course. La couche réactive s'est déclenchée
trois fois et a relâché à chaque fois.

**Le but 3 échoue autrement, et la nuance compte** : le planificateur ne rend ni
« obstacle » ni chemin vide, mais un chemin d'UN SEUL point, en boucle. C'est le
planificateur qui dit que départ et but tombent sur le même nœud du roadmap : le
robot est près du but mais hors des 0,45 m, et il n'y a aucun nœud entre les deux
pour faire un chemin. C'est le décalage de départ de E1 vu par l'autre bout — le
même espacement de nœuds qui pose le départ à 0,30-0,45 m fixe aussi la finesse
avec laquelle le planificateur peut approcher quoi que ce soit. La tolérance et
le décalage ne sont donc pas indépendants. `min_samples` n'y change rien, c'est
mesuré (voir `params.yaml.in`) : il faut une approche finale, pas plus
d'échantillons.

**Le test de la chaise n'a pas été fait.** Il demande quelqu'un devant la
machine pour poser une chaise sur le chemin.

### E3. Donner l'autorité au planificateur, ou ne pas la donner

C'est la seule phase qui change le comportement du robot, et elle n'est pas
acquise. La question à trancher avec des chiffres, pas d'avance : le chemin ITS
conduit-il mieux que la patrouille actuelle, et que devient la mission
(aller-retour sur l'axe) si Nav2 pilote par `cmd_vel` ? Décider après E2.

---

## Ce qui n'est pas dans la liste, et pourquoi

Le passage au G1 réel n'est pas une étape : la préparation est déjà faite. La
navigation est séparée du simulateur et ne connaît qu'une `Pose` en entrée et une
vitesse en sortie. Le jour venu, il s'agit d'écrire une seconde incarnation, pas
de refondre quoi que ce soit.
