# Cellules plutot que rectangles

Mesure du 10 aout 2026. Remplace la representation des obstacles dans le
navigator. `OBSTACLE_REP=rects` restaure exactement l'ancien comportement.

## Le probleme, chiffre

Le canape en L de ce salon, reduit a un rectangle aligne sur les axes :

- rectangle publie 3,35 x 3,19 m = 10,7 m2
- canape reel, deux bras de 0,9 m de profondeur, environ 5,1 m2
- **5,6 m2 de sol libre avales**, qui sont la table basse et les deux couloirs

Consequence mesuree sur le bus : `obstacle 4.4 x 3.8 m at (4.4, -0.4)`, soit
un obstacle de 4,4 m de long centre a 4,4 m dans une piece dont le mur du fond
est a 6,2 m, occupant toute la largeur et centre a -0,01 m de l'axe du robot.
Aucun detour lateral n'existe. `no way round: needs a 2.17 m detour and only
1.80 m is available`, 0,01 m parcouru en 60 s.

## Ce qui a ete essaye avant, et pourquoi ca ne suffisait pas

1. **Contour du masque au lieu de la boite image.** Necessaire, insuffisant :
   `mask_footprints` suivait deja la ligne de contact par colonne, puis
   reduisait la composante a un rectangle englobant.
2. **Bornage par la profondeur mesuree** (`FOOTPRINT_FROM_DEPTH`). Reel gain :
   x_max 6,62 -> 5,80 m, 45/45 empreintes au-dela du mur -> 0/44. Mais le
   rectangle restait un rectangle autour d'un L.
3. **Decomposition en plusieurs rectangles** (`FOOTPRINT_MAX_RECTS`). Meilleure
   couverture, jamais l'interieur du L : couvrir exactement une forme dont le
   bord est bruite par la profondeur demande tant de rectangles que le budget
   finit toujours par une boite qui traverse le passage. Mesure a 27
   rectangles, sol praticable 0,93 m2, robot toujours immobile.
4. **Fusion des empreintes** (`_merge`). Mesuree sans effet : six lignes
   identiques a 0,64 m et fusion coupee. Ce n'etait pas le blocage.

Une cellule est occupee ou elle ne l'est pas. La question ne se pose plus.

## La chaine

- `perception` publie les silhouettes, inchange.
- `compositor._ground_grids()` construit deux grilles au sol : `occ` (objets,
  par leur PROPRE profondeur mesuree) et `flr` (sol vu, par le plan sol). Le
  plan n'est vrai que pour les pixels qui sont dessus ; applique a un objet il
  repond "ou serait ce pixel s'il etait du sol", ce qui fuit vers l'horizon.
- Marge appliquee par dilatation en metres monde, donc la concavite survit.
- `GRID_PASSABLE=0.44` ferme par morphologie les fentes plus etroites que le
  robot. C'est le seul role legitime que `_merge` remplissait, fait ici ou
  c'est un fait local et non une fusion globale qui reconstruit la boite
  englobante que la decomposition existait pour eviter.
- Les grilles voyagent sur `PATROL_ROI` a cote de `blocked`, jamais a la place.
  3200 octets par message a 5 cm sur 8 x 8 m, moins que le masque de
  silhouettes deja publie a chaque trame.
- `navigator._detour_grid()` ne demande plus "de combien dois-je depasser le
  bord de cette boite", question sans reponse sur un objet concave, mais
  "existe-t-il un couloir de ma largeur", et prend le plus proche.

## Quatre derives corrigees au passage

Aucune n'etait le sujet, chacune bloquait a elle seule.

1. **`GAP_CLEAR=0.84` n'existe pas.** La valeur reelle est 0,10, et le seuil
   0,64 m du journal est `2 x (ROBOT_HALF_WIDTH + GAP_CLEAR)`.
2. **`STOP_AT=6.0` datait d'une scene dont le sol allait plus loin.** Sur
   celle-ci il ordonnait quatre metres de marche entierement dans les
   empreintes : 100 % de poses en raclage et -0,522 m de clairance mesuraient
   l'obeissance a un ordre impossible, pas un echec de contournement.
   `PATROL_CLAMP` ramene le demi-tour a la portee du dernier couloir libre,
   trouve en y marchant. Borner par le bord du sol visible ne suffit pas : une
   bande de sol derriere le canape est du sol, est rapportee, et est
   inatteignable.
3. **`OBSTACLE_LOOK=3.5` fixe depuis x=1,9 atteint 5,4 m**, donc le canape du
   fond, donc toute voie etait declaree bloquee. La portee est desormais bornee
   a la fin de la jambe en cours : un obstacle au-dela du demi-tour n'est pas
   sur le chemin.
4. **Les deux jambes contournaient par des cotes opposes.** Chacune prenait le
   couloir le plus proche de SA ligne centrale, et ces lignes sont symetriques,
   donc le robot traversait toute la largeur de la piece a chaque demi-tour.
   Les couloirs sont maintenant ordonnes par distance a la voie deja tenue, en
   y monde absolu.

## Le retard de poursuite

Le couloir bouge d'un coup, le robot non : en tenant au plus CROSS_MAX du cap,
fermer E metres d'ecart lateral demande E / tan(CROSS_MAX) metres d'elan.

## L'erreur qu'il faut lire avant les chiffres

La premiere version de ce travail exigeait un couloir de
`2 x (ROBOT_HALF_WIDTH + GAP_CLEAR + LANE_SLACK)` = 0,94 m. La grille porte
DEJA `OBSTACLE_MARGIN`, applique cellule par cellule, donc le couloir de 0,90 m
de ce salon n'y mesure plus que 0,66 m. Aucune voie ne pouvait etre libre, et
le robot rapportait correctement "no way round" a propos d'un passage qu'un
humain traverse. `navigator.py` avertit de ce piege exactement, pour le chemin
rectangle ; il a ete reproduit un fichier plus loin.

Le premier banc de mesure ne pouvait pas l'attraper : la piece synthetique
n'avait pas de mur a gauche, donc toute voie poussee de ce cote etait libre
jusqu'au bord de la grille et la largeur demandee n'etait confrontee a rien.
Une piece de test ouverte d'un cote n'est pas une piece. Le tableau LANE_SLACK
publie avant cette correction est invalide.

Mesure en piece fermee des deux cotes, 60 s :

| LANE_SLACK | parcouru | raclage | pire clairance | plage x |
|---|---|---|---|---|
| 0,05 | 17,01 m | 19,9 % | -0,211 m | 1,85-4,14 |
| 0,08 | 15,59 m | 17,6 % | -0,215 m | 1,73-4,09 |
| 0,11 | 15,57 m | 0,0 % | +0,338 m | 1,66-2,24 |

Deux regimes et aucun compromis a cette heure : au-dessus de 0,11 le robot ne
racle plus mais ne depasse jamais x = 2,24 m, donc il ne passe pas la table ;
en dessous il passe et racle. Elargir OBSTACLE_MARGIN ne deplace rien, les
memes chiffres sortent a 0,12, 0,08 et 0,05, ce qui dit que le raclage du
modele ne vient pas de la largeur du couloir. Ou il vient exactement n'est pas
mesure. Ce n'est pas resolu et ce n'est pas presente comme tel.

Le sens du demi-tour est desormais choisi sur l'espace libre. Tourner toujours
du meme cote etait juste tant que le demi-tour se faisait en sol degage loin de
tout ; la patrouille tourne maintenant aussi tard que le sol le permet, donc le
balayage se fait a cote des meubles.

## Resultat, meme scene, meme 60 s

| | rects | grid |
|---|---|---|
| parcouru | 0,00 m | 16,38 m |
| plage x | 1,90 seule | 1,82 a 4,14 |
| raclage | 0,0 % (immobile) | 0,0 % |
| aire obstacle | 14,72 m2 | 9,22 m2 |

Le temoin `rects` reproduit le journal du terrain mot pour mot, `no way round`
inclus, ce qui est la raison de le garder : il dit que le banc mesure la bonne
chose.

## Ce que ceci N'EST PAS

`scripts/grid_probe.py` est un MODELE. La cinematique est un unicycle, pas la
politique RL, donc les distances ne sont pas celles que le vrai robot parcourt.
Ce qui est mesure honnetement est la DECISION : un couloir existe-t-il, lequel
est pris, la ligne prise traverse-t-elle le contour reel d'un obstacle. C'est
la partie qui etait cassee et c'est la partie testable sans materiel.

Non mesure sur la vraie scene a cette heure : tout. `make grid-probe` puis un
run de 60 s avec la camera, sans `up -d sim`, et les memes quatre chiffres.
