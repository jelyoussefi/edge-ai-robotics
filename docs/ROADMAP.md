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

## Étape E. Nav2 et ITS Path Planner, à décider plus tard

Reporté délibérément. Ce n'est pas un ajout mais un changement d'architecture :
un greffon Nav2 n'arrive pas seul, il demande une carte de coûts, un arbre TF
complet, une localisation, une empreinte et une cinématique de robot. Nav2 pilote
aussi le robot par `cmd_vel`, ce qui recouvre la mission et la patrouille
actuelles.

À rouvrir seulement quand B et C auront montré ce que la passerelle coûte
réellement, ou quand un robot réel entrera dans le tableau.

---

## Ce qui n'est pas dans la liste, et pourquoi

Le passage au G1 réel n'est pas une étape : la préparation est déjà faite. La
navigation est séparée du simulateur et ne connaît qu'une `Pose` en entrée et une
vitesse en sortie. Le jour venu, il s'agit d'écrire une seconde incarnation, pas
de refondre quoi que ce soit.
