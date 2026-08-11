# Étape 2 : un seul budget de marge

> **CADUC depuis le 2026-08-11 : toutes les mesures de grille de ce document
> ont ete prises a `GRID_CELL=0,05 m`.** La cellule vaut 0,02 m depuis
> `docs/SURPROJECTION-RESULTS.md` partie III. Les comptes de cellules sont
> multiplies par ~7 et les largeurs de travee changent : la bande d'identite
> de scene passe de 3251-3716 a 18 092-22 468, et le cote TV lit 0,70 m au
> lieu de 0,50. Les VERDICTS tiennent, les CHIFFRES ne se comparent pas.


**État : ÉTAPE CLOSE.** Budget de marge démontré dans les deux sens ; le
franchissement par le patrouilleur est hors de portée sur cette géométrie et
relève de l'étape 4. Détail en §7.4.

**Note historique :** Le couloir mesure 0,40 à
0,90 m par tranche mais son axe dérive de 0,27 m, ce qui ne laisse que **0,35 m
de largeur commune** contre 0,68 m demandés. Ce n'est pas un problème de budget —
`CLEARANCE=0` ne suffirait pas non plus — c'est que `free_lane` ne teste que des
couloirs droits. Renvoyé à l'étape 4, Nav2 et ITS. Voir §7.

---

## 1. Le défaut

Trois marges empilées à trois endroits, qu'aucune ligne de code n'additionnait :

```
CLEARANCE       0.12  dilatation par cellule dans points_to_grid, 2 cotes = 0.24
LANE_SLACK      0.08  ajoutee a la demi-largeur dans free_lane,    2 cotes = 0.16
robot                                                                     = 0.44
                                                                    total = 0.84
```

Un couloir réel de 0,90 m laissait donc 3 cm par côté. Et un fichier plus loin,
la fusion posait une autre question encore : `need = 2 x (ROBOT_HALF_WIDTH +
GAP_CLEAR) = 0,64 m` mesurés entre des rectangles portant déjà 0,12 chacun, soit
**0,88 m** de sol réel — 3 cm d'un côté, -2 cm de l'autre. Deux chemins, deux
réponses, aucune écrite en mètres nulle part.

`GAP_CLEAR` avait déjà causé ce défaut et n'avait été corrigé qu'à moitié ; le
commentaire de `navigator.py` le racontait, à côté du code qui le répétait.

Un quatrième s'est trouvé au passage : **`OBSTACLE_CLEAR`**, lu de
l'environnement, affecté à un attribut de classe, consommé nulle part. Le
détecteur de knobs morts le ratait parce qu'un knob qui se lit lui-même paraît
utilisé.

## 2. Interface

Une seule grandeur, `CLEARANCE`, en mètres. `LANE_SLACK`, `GAP_CLEAR` et
`OBSTACLE_CLEAR` ont disparu de l'interface. Une seule déclaration dans
`docker-compose.yml` pilote les quatre services qui la lisent — ce que
`OBSTACLE_MARGIN`, répété trois fois avec à chaque fois un commentaire demandant
au lecteur suivant de les garder égaux, ne donnait pas.

Au démarrage, en clair :

```
clearance budget: the robot is 0.44 m wide and keeps 0.12 m of clearance on
each side, so it will only walk through a gap of 0.68 m or more of REAL FLOOR,
measurable with a tape. The margin lives in the query (raw grid), so the grid
is asked for 0.34 m of half-width. Nothing adds to this later.
```

## 3. Où appliquer la marge : mesuré, pas choisi

`CLEARANCE_MODE` expose les deux formes pour qu'elles soient comparables :

- **`dilate`** — `points_to_grid` grossit chaque cellule occupée, la requête
  demande la demi-largeur nue. L'ancien comportement.
- **`query`** — la grille publiée reste une carte **brute** des obstacles, la
  requête demande demi-largeur + `CLEARANCE`. **Le nouveau défaut.**

Comparaison **hors ligne, sur un seul message `PATROL_ROI`**, donc les deux
formes voient la même grille au même instant. Deux passes vives ne peuvent pas
trancher : la scène dérive entre elles de plus que l'effet mesuré.

```
lane decisions over 41 distances x 105 lanes = 4305:
  agree                      4275  (99.30 %)
  clear for DILATE only        30  (0.70 %)
  clear for QUERY only          0  (0.00 %)
  disagreements span x 1.70..5.50 m, y -2.60..+1.80 m
```

Trois constats, dont un seul était celui qu'on attendait.

### 3.1 `dilate` ne livre pas la marge qu'on lui donne

Le noyau vaut `2 x round(c / cell) + 1`. Sur une grille de 0,05 m :

| CLEARANCE demandé | noyau | réellement grossi | écart |
|---|---|---|---|
| **0,12 m** | 5x5 | **0,10 m** | **-17 %** |
| 0,10 m | 5x5 | 0,10 m | 0 % |
| 0,15 m | 7x7 | 0,15 m | 0 % |
| 0,20 m | 9x9 | 0,20 m | 0 % |

Silencieusement, et seulement aux valeurs qui ne tombent pas sur un multiple de
la cellule — donc invisible à qui teste avec des nombres ronds. Le mode `query`
ne quantise jamais. Cela explique à soi seul une bonne part des 30.

### 3.2 `query` était anisotrope, et j'ai failli le livrer ainsi

Élargir `half_width` gonfle le couloir **en travers** et pas **en long**. La
première version gardait donc la marge d'un mur sur le côté et zéro de la table
devant.

Le signe du désaccord l'a trahi : **61 voies libres pour `query` seul et zéro
pour `dilate`**, toutes là où un obstacle était devant plutôt qu'à côté. Ce
n'était pas un effet de coin, c'était un axe manquant.

`corridor_blocked` prend maintenant un `pad` longitudinal, `query_pad()` le
règle, et le signe s'est inversé pour devenir celui que la géométrie prédit :
30 pour `dilate`, 0 pour `query`. **La dilatation grossit un obstacle sur tous
les axes ; ce qui prétend la remplacer doit le faire aussi.**

### 3.3 La dilatation n'est pas inversible — la raison architecturale

Une fois les cellules gonflées, la grille ne peut plus être interrogée à une
autre largeur. Le balayage « quelle largeur de voie AURAIT été libre » de
`free_lane`, et toute sonde qui tente de reproduire une décision, interrogent
alors une grille qui ne décrit plus les obstacles. **C'est le mécanisme par
lequel `lane_probe` et le navigateur ne pouvaient pas être d'accord**, et garder
deux knobs égaux n'aurait jamais pu le corriger.

Sur la couche brute, la même grille répond :

| couloir demandé | voies libres depuis x=2,0 m |
|---|---|
| 0,44 m | 71 / 105 |
| 0,56 m | 70 / 105 |
| 0,68 m | 69 / 105 |
| 0,80 m | 67 / 105 |
| 0,90 m | 66 / 105 |
| 1,00 m | 65 / 105 |

C'est aussi la forme de Nav2 — couche obstacle brute, couche d'inflation
séparée — donc l'étape 4 y gagne.

### 3.4 Verdict

`query` par défaut. Marge exacte, information conservée, jamais plus permissif
que `dilate` dans cette pièce, et la forme que l'étape 4 voudra. `dilate` reste
disponible et documenté par le tableau ci-dessus, pas comme un réglage à essayer.

## 4. L'accord sonde / robot, vérifié par la machine

`lane_probe` et `nav_probe` calculent `GRID_HALF`, `GRID_PAD` et `MIN_CORRIDOR`
avec les **mêmes helpers** que le navigateur, comparent les trois à ce qu'il
publie, et **sortent en erreur plutôt que d'imprimer**.

Falsifié plutôt qu'affirmé — en forçant `CLEARANCE=0.30` dans la sonde :

```
lane_probe would ask the grid a different question from the robot --
GRID_HALF: lane_probe 0.520 vs navigator 0.340, GRID_PAD: 0.300 vs 0.120,
MIN_CORRIDOR: 1.040 vs 0.680. Refusing to report numbers about a question the
robot is not asking.
```

Ce que ça attrape et ce que ça n'attrape pas : la sonde adopte d'abord les
réglages vifs, donc ce n'est pas un test que deux environnements s'accordent,
c'est un test qu'**à entrées égales les deux posent la même question**. Ce qui
attrape la panne qui mord vraiment ici : `common/edgebot` est **cuit** dans
l'image du sim et **monté depuis l'arbre** dans une sonde, donc une édition non
reconstruite devient un écart au lieu de deux rapports plausibles.

`nav_probe` tirait aussi son seuil de raclage de `ROBOT_HALF_WIDTH` en dur, ce
qui n'est juste qu'en mode `dilate`. Il passe par `query_half` maintenant.

## 5. Les autres consommateurs, découplés explicitement

`CLEARANCE` sert aussi à `clip_footprints` et au découpage du masque de sol. Ne
corriger qu'un chemin est le motif de bug récurrent du dépôt, donc la décision
est écrite plutôt que subie :

**Les rectangles portent `CLEARANCE` dans les deux modes**, et `clip_footprints`
comme le découpage du masque, qui les consomment, ne changent pas. Raisons :
`topics.py` documente `blocked` comme publié marge appliquée, la comparaison
suite de `docs/ETAPE-C-RESULTS.md` mesure nos empreintes contre celles d'Intel
sur cette base, et le chemin `rects` du navigateur n'ajoute que sa propre
demi-largeur pour la même raison.

**Le mode change ce que signifie une CELLULE**, et seul le chemin grille lit des
cellules. Les deux représentations exigent le même couloir de 0,68 m.

## 6. Deux scènes, et une phrase que je n'aurais pas dû écrire

### 6.0 Correction

La première rédaction de ce §6 annonçait « table basse placée pour former le
couloir, mobilier figé ». **Il n'y avait aucune table.** La phrase venait du
gabarit du prompt et je l'ai recopiée comme un constat. Ce que j'ai décrit
comme le couloir table–canapé à x = 4,5 m était un renfoncement du canapé.

Les mesures elles-mêmes étaient bonnes et le restent : « 0,30 m de long » et
« aucune voie ne le traverse » décrivaient fidèlement une pièce sans table. La
grille avait raison. Le §7.2 de cette version, qui demandait un mètre pour
départager la grille et la réalité, était donc une fausse question — tranchée
sans mètre, en faveur de la grille.

Règle qui en découle, appliquée depuis : **un chiffre resté en gabarit dans une
consigne n'est pas une mesure**, et aucune phrase d'un prompt n'entre dans ce
document comme un fait constaté.

### 6.1 Scène A, sans table — ligne de base conservée pour mémoire

| | |
|---|---|
| laps couverts | 43 à 46 |
| cellules occupées (grille brute) | 2095 à 2187 |
| raclages | 0 / 3432 = 0,0 % |
| clairance minimale | +0,633 m |
| distance parcourue | 2,05 m avant, 1,19 m latéral |

### 6.2 Scène B, table en place — la ligne de base courante

Changement de scène **vérifié avant de mesurer**, et non supposé : 2540 à
3332 cellules occupées contre la bande 2095–2187 de la scène A, et l'extension
latérale passe de +1,6 à +2,4 m. La table est là.

| | scène A (sans table) | **scène B (avec table)** |
|---|---|---|
| **laps couverts** | 43 à 46 | **9 à 11**, `STALLED` dans 6 échantillons |
| **cellules occupées** | 2095 à 2187 | **2540 à 3332**, médiane 2838 |
| **raclages** | 0 / 3432 = 0,0 % | **0 / 3471 = 0,0 %** |
| **clairance minimale** | +0,633 m | **+0,574 m** |
| clairance p05 / médiane | +0,763 / +1,469 m | +0,611 / +0,946 m |
| distance parcourue | 2,05 m avant, 1,19 latéral | 2,27 m avant, 1,46 latéral |
| empreintes publiées | médiane 24 (23 à 24) | médiane 40 (29 à 54) |
| boîte du ROI | x 1,62..5,52 m | **x 1,62..4,44 m** |

**Les deux colonnes ne se comparent pas** : ce sont deux pièces. La colonne B est
la référence pour tout ce qui suit, et sa bande d'identité de scène est
**2540–3332 cellules occupées**.

Ce que la table coûte, lisible : les empreintes passent de 24 à 40, la médiane de
clairance de 1,47 à 0,95 m, et le sol praticable s'arrête un mètre plus tôt
(5,52 → 4,44 m). Le chien de garde de l'étape 5 signale 6 blocages, dont le
robot repart à chaque fois.

## 7. Le couloir n'est pas franchissable en ligne droite

C'est un **résultat**, pas un échec, et il ne tient ni au budget de marge ni à
un réglage.

### 7.0 Les quatre scènes, pour qui arrive ici

Le mobilier a bougé quatre fois pendant cette étape. **Les chiffres d'une scène
ne se comparent à ceux d'une autre sous aucun prétexte**, et chaque mesure
ci-dessous nomme la sienne.

| scène | mobilier | cellules occupées | ce qu'elle a servi à établir |
|---|---|---|---|
| **A** | aucune table ; ce que j'ai pris pour un couloir était un renfoncement du canapé | 2095–2187 | la ligne de base, et une leçon de méthode (§6.0) |
| **B** | table posée, non centrée | 2540–3332 | le couloir échoue par **dérive de l'axe** : 0,27 m, 0,35 m communs |
| **C** | table recentrée, 0,90 m dégagés | ~2637–3444 | même verdict, **cause opposée** : droit et trop étroit |
| **D** | table décalée : 1,20 m à droite et derrière, 0,70 m côté TV | 3251–3716 | **le budget démontré dans les deux sens**, et la régression |

Le compte de cellules ne suffit pas à distinguer deux scènes — il oscille de
±800 d'une trame à l'autre, plus que l'écart entre B et C. L'identité de scène
est établie sur la **géométrie** : extension occupée et structure des travées.

### 7.1 Largeur : la grille contre le mètre

Scène C, table recentrée à **3,20 m** du pied de la caméra, **0,90 m** libres sur
ses trois côtés dégagés. Mesures agrégées sur **15 trames**, pas une.

| | grille | mètre |
|---|---|---|
| travée la plus large | **0,75 m** (0,70 à 0,85) | **0,90 m** |
| travée la plus étroite | **0,45 m** (0,05 à 0,55) | 0,90 m |
| position du couloir | x 3,10 à 4,45 m, y −1,30 à −0,55 | table à x = 3,20 m |

**Manque résiduel de 0,15 m** au plus large : 0,75 m contre 0,90 m. C'est très
exactement la bande que vous aviez annoncée pour de la sur-projection du canapé,
et ce n'est pas un défaut de budget. **Noté, non corrigé ici.**

Au plus étroit l'écart est de 0,45 m, mais ce point est à x = 4,45 m, soit
1,25 m derrière la table : il n'appartient pas au dégagement de 0,90 m que le
mètre décrit et ne doit pas être compté contre lui.

**Ce que la grille ne permet pas d'affirmer.** Le couloir mesuré court de 3,10 à
4,45 m, soit 1,35 m de long, ce qui est plus qu'une table basse. Je ne peux pas
établir depuis la grille seule que ce passage EST le dégagement de 0,90 m que
vous avez mesuré, plutôt qu'un couloir entre le canapé et autre chose qui
commencerait au niveau de la table. Le rapprochement du tableau ci-dessus est
donc fait sur la position et la largeur, pas sur une identification certaine.

## 7.2 Longueur et dérive de l'axe : recalculé sur la scène C

Largeur libre **et** centre du couloir, par tranche de 0,05 m :

```
      x  largeur   centre        x  largeur   centre
   3.10     0.70    -0.95      3.90     0.60    -0.95
   3.20     0.75    -0.92      4.00     0.60    -0.95
   3.30     0.70    -0.95      4.10     0.60    -1.00
   3.40     0.75    -0.92      4.20     0.60    -1.00
   3.50     0.70    -0.95      4.30     0.55    -1.02
   3.60     0.70    -0.95      4.40     0.55    -1.02
   3.80     0.65    -0.97      4.45     0.50    -1.00
```

Sur 15 trames :

| | scène B | **scène C** | budget |
|---|---|---|---|
| travée la plus large | 0,90 m | **0,75 m** (0,70–0,85) | |
| travée la plus étroite | 0,40 m | **0,45 m** (0,05–0,55) | |
| **commune à toutes les tranches** | **0,35 m** | **0,45 m** (0,05–0,55) | **0,68 m** |
| **dérive de l'axe** | **0,27 m** | **0,10 m** | |
| trames où le budget passe | — | **0 sur 15** | |

**Le verdict tient, mais sa cause a changé, et c'est le point de ce recalcul.**

En scène B le couloir échouait par **dérive** : chaque tranche faisait au moins
0,40 m et l'axe glissait de 0,27 m, ne laissant que 0,35 m communs. En scène C
l'axe ne dérive plus que de 0,10 m et la largeur commune, 0,45 m, est
pratiquement la tranche la plus étroite : **le couloir est droit et simplement
trop étroit.**

Ce sont deux diagnostics opposés qui appellent des réponses opposées : élargir le
passage sert dans un cas et ne sert à rien dans l'autre. Recopier le verdict de
la scène B aurait donné la bonne conclusion pour la mauvaise raison.

**Correction d'outil, trouvée en le recalculant.** `clearance_probe` imprimait
« NOT PASSABLE IN A STRAIGHT LINE » dès que la largeur commune passait sous le
budget, quelle qu'en soit la cause. Faux dès la première dérive à 0,10 m. Il
distingue maintenant les deux : largeur commune proche de la tranche la plus
étroite = trop étroit ; largeur commune très en dessous = dérive.

### 7.3 Conséquence

Sur la scène C, le couloir mesure **0,45 m de largeur commune contre 0,68 m
demandés**, sur les 15 trames, sans exception. Le robot ne passe pas, et il a
raison de ne pas passer : il fait 0,44 m d'épaule.

Trois issues, et aucune n'est un réglage de cette étape :

- **Élargir le passage** de 0,25 m. C'est la seule qui rende ce couloir
  franchissable en ligne droite.
- **Descendre `CLEARANCE` à 0,01 m**, ce que la sonde calcule et affiche. Le
  robot entrerait avec 1 cm de reste par côté. Ce n'est pas proposé : le budget
  existe pour absorber l'erreur de suivi et l'erreur de perception, dont
  précisément les 0,15 m de sur-projection du §7.1.
- **Un planificateur qui suit une courbe** — étape 4, Nav2 et ITS. Utile en
  général, mais il ne fait pas passer 0,44 m de robot dans 0,45 m de couloir
  avec une marge : il traite la dérive, pas l'étroitesse.

La couche d'obstacles brute du §3 reste le bon choix pour l'étape 4, c'est la
forme d'entrée que ce planificateur voudra.

Le travail de franchissement — `STOP_AT` au-delà du couloir, vérification de
`DETOUR_MAX`, passe de 60 s — **n'est pas fait**, et il ne le sera pas sur cette
géométrie : il mesurerait la capacité du suivi rectiligne à faire ce qu'il ne
peut pas faire.

### 7.3bis Scène D : un passage qui doit passer, un qui doit être refusé

Table à **2,90 m** de la caméra. **1,20 m** réels côté droit et derrière,
**0,70 m** réels côté TV. Le budget se démontre alors dans les deux sens plutôt
que d'être seulement franchi.

Repère : `+y` est la **gauche**, donc le côté droit vu de la caméra est y négatif.

| | côté droit | côté TV | budget |
|---|---|---|---|
| position | x 2,80–4,40, centre y −0,88 | x 4,20–4,50, centre y +1,45 | |
| **mètre** | **1,20 m** | **0,70 m** | |
| grille, travée la plus large | 1,05 m | 0,60 m | |
| grille, travée la plus étroite | 0,90 m | 0,55 m | |
| **grille, largeur commune** | **0,80 m** (0,50–0,90) | **0,50 m** (0,00–0,75) | **0,68 m** |
| **trames où le budget passe** | **11 / 15** | **2 / 15** | |
| verdict | **franchissable** | **refusé** | |

Sur 15 trames chacun, pas une.

#### Laquelle des deux causes refuse le côté TV

Le manque grille-mètre vaut 0,10 à 0,20 m des deux côtés, encadrant les 0,15 m
de sur-projection du §7.1. Appliqué au côté TV :

| | largeur | verdict |
|---|---|---|
| grille telle quelle | 0,50 m | refusé de **0,18 m** |
| grille + 0,15 m de sur-projection rendus | 0,65 m | refusé de **0,03 m** |
| mètre | 0,70 m | **accepté de 0,02 m** |

**C'est la sur-projection qui refuse ce passage, pas le budget.** Sans elle,
0,70 m contre 0,68 m demandés : accepté. Le budget seul ne l'aurait pas écarté.

Deux réserves qui vont contre cette conclusion et qu'il faut lire avec elle.
L'acceptation ne tiendrait qu'à **2 cm**, et la largeur commune mesurée varie de
0,00 à 0,75 m d'une trame à l'autre : même avec une grille parfaite la décision
serait instable et non franche. Et **2 trames sur 15 franchissent déjà le budget
aujourd'hui**, donc le refus n'est pas net non plus.

Le côté TV est atteignable — bande de détour −2,79 à +2,01 m contre un trou à
+1,15..+1,80 — donc son refus porte bien sur la largeur et non sur
l'accessibilité. Vérifié avant de conclure, comme la voie de droite à −0,88 m,
qui est dans la bande avec 2,0 m de marge : **`DETOUR_MAX` n'est en cause dans
aucun des deux cas.**

### 7.3ter Le franchissement : une régression, pas un critère manqué

La passe de 60 s avec `STOP_AT=4.4` n'a pas seulement échoué à démontrer le
critère, elle a fait entrer le robot dans le mobilier — ce qu'aucune passe
précédente n'avait fait.

| | toutes les passes précédentes | **cette passe** |
|---|---|---|
| raclages (marge entamée, 0,34 m) | **0,0 %** | **54,2 %** (1867 / 3442) |
| corps dans le meuble (0,22 m) | **0,0 %** | **41,5 %** (1427 / 3442) |
| clairance minimale | +0,410 à +0,633 m | **−0,024 m** |
| étendue latérale | 1,2 à 1,5 m | **2,80 m** |
| laps | 3 à 4 par minute | 5 à 6, `STALLED` ×10 |

Le robot est allé à **y = +2,13 m** au lieu de tenir la voie de droite à
−0,88 m. Le critère « emprunte la droite » n'est donc pas atteint, mais ce n'est
pas la lecture utile : **c'est une régression à diagnostiquer.**

Ce qui est déjà écarté : `DETOUR_MAX` (ci-dessus), `no way round` compté à 0,
`escape` à 0, le navigateur trouvant bien des voies (`clear lane -0.35 m`) et la
limite de patrouille oscillant normalement entre 3,20 et 3,80 m.

Piste principale, non encore vérifiée : **la boîte du ROI est tombée à
x 1,62..2,28 m pendant que le robot marchait jusqu'à 4,10 m.** Le polygone et la
grille ne décrivent plus la même pièce. Cela ressemble au clignotement de
l'étape 3 — `polygon_from_mask` ne conserve que la plus grande composante
connexe — survenant ici pendant que le robot est loin.

Les 10 `STALLED` sont probablement une conséquence : un robot figé ne racle pas
54 % du temps.

**Correction d'outil.** `nav_probe` ne rapportait qu'un seuil et mélangeait deux
événements distincts : entamer la marge de sécurité et poser le corps dans le
meuble, séparés de `CLEARANCE` soit 0,12 m. Une traversée d'un couloir de 0,90 m
ressortait à 41,9 % de « raclage » alors que le corps ne touchait rien : vrai,
alarmant, et la mauvaise alarme. Les deux sont désormais rapportés séparément.

### 7.3quater Diagnostic de la régression

Ordre du moins cher au plus cher, et les trois réponses sont nettes.

#### 1. La grille ou le polygone ? — **le polygone, et lui seul**

`nav_probe` enregistre désormais, à **chaque** échantillon, le nombre de cellules
occupées et la boîte du ROI. Sur 59 messages :

| | |
|---|---|
| cellules occupées | médiane **3491**, plage 2272–3882 |
| profondeur du polygone ROI | médiane **0,64 m**, plage 0,44–3,62 |
| bord lointain du ROI | médiane 2,26 m, plage 2,06–5,24 |
| **ROI effondré (< 1,0 m de profondeur)** | **52 échantillons sur 59 = 88 %** |

**La carte est saine, le polygone est cassé.** Les cellules restent dans leur
bande pendant que le polygone s'effondre 88 % du temps, et son bord lointain
oscille de 2,06 à 5,24 m d'un message à l'autre. C'est le clignotement de
l'étape 3 : `polygon_from_mask` ne conserve que la plus grande composante
connexe, et la table coupe le sol en morceaux dont le plus grand change de
message en message.

#### 2. Le navigateur consomme-t-il le ROI ? — **non, toujours pas**

Revérifié sur le code courant, `pad` longitudinal compris :

```
services/sim/navigator.py:287   self._roi: list = []      declaration
services/sim/navigator.py:417   self._roi = roi           ecriture
                                (aucune lecture)
services/sim/navigator.py:299   self._flr = None          declaration
services/sim/navigator.py:464   self._flr = unpack_grid   ecriture
                                (aucune lecture)
```

Toutes les requêtes de couloir — `free_lane`, `corridor_blocked`, `clear_reach`,
`nearest_free` — lisent `self._occ` et rien d'autre.

**Conséquence, et elle contredit l'hypothèse de départ : le polygone qui
clignote n'est PAS la cause de la régression.** Il ne peut pas l'être, le
navigateur ne le lit jamais. La condition « si la cause est le polygone, l'étape
3 devient prioritaire » **ne se déclenche pas**. L'étape 3 reste un défaut
d'affichage — sévère, mesuré ici à 88 %, mais d'affichage.

#### 3. L'excursion à y = +2,13 — **non reproduite**

| | passe du §7.3ter | passe de diagnostic |
|---|---|---|
| marge entamée | 54,2 % | **23,6 %** |
| corps dans le meuble | 41,5 % | **12,7 %** |
| clairance minimale | −0,024 m | −0,023 m |
| étendue latérale | **2,80 m** | **1,78 m** |
| y atteint | **+2,13 m** | +0,50 m |
| `STALLED` | 10 | **0** |
| **voie tenue par le navigateur** | non enregistrée | **−0,62 m** (−0,63 à −0,61) |

La voie choisie est **constante à −0,62 m**, du côté du couloir. **Le navigateur
n'a envoyé le robot nulle part d'anormal.** L'excursion et les 10 `STALLED` ne se
reproduisent pas ; le sim ayant été redémarré entre les deux, la piste la plus
probable est la convergence de la voie de patrouille — `self.lane` part de
`LANE=0` et converge sur le balayage mesuré des demi-tours, que la table
perturbe. Non établi, faute de l'avoir revu.

#### Ce qui reste, et c'est réel

**12,7 % de poses avec le corps dans le mobilier, contre 0,0 % partout
ailleurs.** La pire est à **(2,87 ; −0,03)** — sur l'axe, au coin d'attaque de la
table, exactement comme dans la passe précédente à (3,38 ; −0,02).

La cause est géométrique et déjà chiffrée : `LANE=0` place l'axe de patrouille
**dans** la table, occupée en continu de x = 2,8 à 4,4 m à y = 0. Le robot doit
donc se décaler de 0,88 m avant d'atteindre 2,8 m. Avec `CROSS_MAX = 0,35` rad il
lui faut 0,88 / tan(0,35) ≈ **2,4 m d'élan**, et entre `RETURN_TO = 1,0` et la
table il n'y en a que **1,8 m**. Il arrive l'erreur encore ouverte et coupe le
coin. Le journal le disait déjà : `0.56 m of lateral error needs 2.46 m of
run-up and only 0.50 m is clear`.

Ce n'est pas un défaut de l'étape 2 : le budget de marge est correct et vérifié,
c'est la consigne de patrouille qui est infaisable sur cette scène.

**Interaction secondaire, mesurée et plus faible qu'attendu.** `RUNUP_MIN = 0,7 x
CRUISE_VX = 0,6` demande 0,42 m/s et le plancher `START_VX = 0,45` de l'étape 5
le relève à 0,45 : **le plancher écrête le frein d'élan**, de 3 cm/s. Réel, à
corriger — `START_VX` devrait plafonner le frein plutôt que l'inverse — mais ce
n'est pas la cause.

#### `LANE = −0,88` testé, et il ne peut pas marcher

La correction évidente — patrouiller **le long** du couloir au lieu de le
traverser — échoue pour trois raisons indépendantes, dont deux se lisent dans le
code et la troisième se mesure.

**1. `LANE` est symétrique par construction.** `navigator.py:1315` :
`lane = -self.lane * d`. La jambe aller est à −LANE, la jambe retour à +LANE.
`LANE=−0,88` donne donc **+0,88 à l'aller** et −0,88 au retour : la moitié de la
patrouille passe dans la table. `LANE` ne sait pas exprimer « une voie
décalée », seulement « une paire de voies ±LANE ».

**2. Le balayage du demi-tour s'effondre.** `sweep = max(0.3, 2.0 * self.lane)`
ligne 1424 : avec une voie négative, `2 x (−0,88) = −1,76` et le `max` retient
**0,3 m**. Le test du sens de demi-tour sonde alors un arc six fois plus petit
que celui qui sera réellement balayé.

**3. Une voie négative s'efface d'elle-même.** La convergence vaut
`self.lane = 0.8 x self.lane + 0.2 x (swept / 2)` avec `swept = abs(...)`, donc
toujours positif. Mesuré sur quatre demi-tours :

```
swept 0.78 m  ->  lane now -0.63 m
swept 0.53 m  ->  lane now -0.45 m
swept 0.68 m  ->  lane now -0.29 m
swept 0.66 m  ->  lane now -0.17 m
```

**−0,88 érodé à −0,17 en quatre demi-tours**, et la suite part vers +rayon.

Résultat de la passe de 60 s : **19,7 % de corps dans le mobilier**, contre
12,7 % au diagnostic. C'est pire.

`LANE` n'est donc pas une mauvaise valeur ici, c'est le **mauvais paramètre**.

### 7.4 Clôture de l'étape 2

**Le budget de marge est démontré. Le franchissement par le patrouilleur est
hors de portée sur cette géométrie, et ce n'est pas le même sujet.**

Le critère demandait de démontrer qu'un budget de marge unique laisse passer un
couloir réel. C'est fait, et dans les deux sens plutôt qu'un seul (§7.3bis) :

| | côté droit | côté TV |
|---|---|---|
| mètre | 1,20 m | 0,70 m |
| grille, largeur commune | 0,80 m | 0,50 m |
| budget 0,68 m | **franchi 11 fois sur 15** | **refusé 13 fois sur 15** |
| cause du verdict | largeur réelle | **la sur-projection, pas le budget** |

Ce qui manque n'est pas une démonstration du budget, c'est **la capacité du
patrouilleur à amener le robot dans le couloir**. Deux causes mesurées et
distinctes, aucune des deux ne relevant de cette étape :

- **L'élan insuffisant** (§7.3quater). `LANE=0` place l'axe dans la table ; il
  faut 0,88 m de décalage, donc 2,4 m d'élan à `CROSS_MAX=0,35`, et il n'y en a
  que 1,8 m entre `RETURN_TO` et la table. Le robot arrive l'erreur ouverte et
  coupe le coin : 12,7 % de poses avec le corps dans le mobilier.
- **La sémantique symétrique de `LANE`** (ci-dessus). Le navigateur ne sait pas
  représenter une voie décalée unique, donc la correction évidente n'existe pas
  dans son vocabulaire.

**Les deux sont des entrées de l'étape 4.** Ni `RETURN_TO` reculé ni l'étape 4
n'ont été tentés pour atteindre le critère : ils traiteraient le patrouilleur,
pas le budget.

#### Acquis

- [x] une seule grandeur exposée, `CLEARANCE`
- [x] `LANE_SLACK`, `GAP_CLEAR`, `OBSTACLE_CLEAR` retirés de l'interface
- [x] où appliquer la marge, tranché par la mesure : couche brute + inflation à
      la requête (§3)
- [x] couloir exigé journalisé en mètres au démarrage, robot compris
- [x] laps et cellules occupées dans chaque mesure
- [x] `lane_probe` et `nav_probe` exigent la même largeur que le navigateur,
      par assertion falsifiée
- [x] **le budget démontré dans les deux sens**, un passage franchi et un
      refusé, avec la cause du refus attribuée

#### Non atteint, et renvoyé

- [ ] **le robot amené dans le couloir par le patrouilleur.** Deux causes
      mesurées, élan et sémantique de `LANE`, toutes deux entrées de l'étape 4.

### 7.4bis Deux sujets ouverts, non traités ici

**La sur-projection de 0,15 m.** La grille lit systématiquement 0,10 à 0,20 m de
moins que le mètre, des deux côtés de la table. C'est elle qui refuse le côté TV
— sans elle, 0,70 m contre 0,68 m demandés serait accepté (§7.3bis). Un défaut
de perception, en amont du budget, qui décide aujourd'hui d'un verdict de
navigation. À mesurer et corriger pour lui-même.

**L'étape 3, le polygone effondré.** Mesuré à **88 % des messages** sous 1,0 m
de profondeur, bord lointain oscillant de 2,06 à 5,24 m (§7.3quater). Sévère, et
d'affichage : `self._roi` est écrit et relu nulle part, vérifié sur le code
courant. Le navigateur ne le lit pas, donc ce n'est pas ce qui met le robot dans
le mobilier — mais l'overlay, les captures et la comparaison suite en dépendent.

### 7.5 Un double comptage trouvé dans la sonde

`lane_probe` imprimait `corridor 0.68 m in the grid = 0.92 m of real floor` pour
un robot qui en demande 0,68. En mode `query` la grille est brute, donc
`2 x half` **est déjà** le sol réel, et rajouter `CLEARANCE` le comptait deux
fois — le défaut que cette étape supprime, commis dans l'outil qui le mesure.
Corrigé : `min_corridor()` est la seule expression autorisée à répondre.

## 8. Commits

| commit | contenu | scène |
|---|---|---|
| `ff29720` | un seul `CLEARANCE` ; `LANE_SLACK`, `GAP_CLEAR`, `OBSTACLE_CLEAR` retirés ; couloir journalisé en mètres | — |
| `7b16859` | couche d'obstacles brute, inflation à la requête, `pad` longitudinal, assertions sonde/robot, `clearance_probe` | — |
| `8d4494f` | premier compte rendu, avant toute scène | — |
| `79b7d00` | ligne de base, et le double comptage de `lane_probe` corrigé | **A** |
| `fa50481` | le couloir n'est pas droit : dérive de 0,27 m, 0,35 m communs ; `--corridor` ajouté | **B** |
| `9ec66cb` | même verdict, cause opposée : trop étroit et non tordu ; colonne mètre remplie ; agrégation `--frames` ; verdict de l'outil corrigé | **C** |
| `93f93e1` | budget démontré dans les deux sens ; la régression consignée ; `nav_probe` sépare marge entamée et corps dans le meuble | **D** |
| `37bbb92` | diagnostic : carte saine, polygone effondré à 88 %, ni l'un ni l'autre la cause ; la décision du navigateur publiée sur le bus | **D** |
| `97bde1b` | **le plancher `START_VX` plafonne le frein d'élan** au lieu d'être écrasé par lui | **D** |
| `ede25a0` | clôture : budget démontré, patrouilleur renvoyé à l'étape 4 ; `LANE=−0,88` testé et réfuté | **D** |
| `86ca4ab` | étape 6, hors de cette étape mais issu du même audit : garde de bornes unique et partagé | — |

## 9. Reproduire

```bash
# la comparaison des deux formes, hors ligne, sur une seule trame
docker compose run --rm --no-deps --entrypoint python3 \
  -v $PWD/scripts:/scripts:ro -v $PWD/common:/opt/edgebot:ro \
  perception /scripts/clearance_probe.py

# la patrouille, contre la grille, avec les laps
docker compose run --rm --no-deps --entrypoint python3 \
  -v $PWD/scripts:/scripts:ro -v $PWD/common:/opt/edgebot:ro \
  perception /scripts/nav_probe.py --seconds 60 --label BASE

make lane-probe

# l'ancienne forme, pour la comparer
CLEARANCE_MODE=dilate docker compose up -d --build sim compositor
```
