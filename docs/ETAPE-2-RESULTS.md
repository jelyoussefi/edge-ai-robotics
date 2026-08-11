# Étape 2 : un seul budget de marge

**État : architecture close et mesurée, ligne de base prise sur la scène figée,
critère du couloir NON DÉMONTRÉ.** Il ne l'est pas pour deux raisons qui ne sont
ni l'une ni l'autre le budget de marge : la patrouille s'arrête 1,4 m avant la
table, et la grille referme le couloir 0,30 m après son entrée. Le §7 les
détaille et dit ce qu'il faut pour les lever. Rien n'y est revendiqué.

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

## 6. La scène figée, et la ligne de base dessus

Table basse placée pour former le couloir, mobilier figé, 2026-08-11.

**Largeur réelle au mètre : NON RENSEIGNÉE.** La consigne portait `X,XX m`, resté
en gabarit. C'est le seul chiffre manquant du compte rendu et il n'est pas
substituable : tout le §7 se joue sur la comparaison entre ce nombre et ce que la
grille mesure.

### Ligne de base, 60 s, contre la grille

| | valeur |
|---|---|
| poses / mises à jour ROI | 3489 / 59 |
| **laps couverts** | **43 à 46** |
| **cellules occupées (grille brute)** | **2095 à 2187**, médiane 2168 |
| **raclages** | **0 / 3432 = 0,0 %** |
| **clairance minimale** | **+0,633 m** |
| clairance p05 / médiane | +0,763 m / +1,469 m |
| distance parcourue | 2,05 m avant, 1,19 m latéral |
| empreintes publiées | médiane 24 (23 à 24) |
| couloir exigé | **0,68 m de sol réel**, journalisé, vérifié par assertion |
| sol libre | 4444 cellules = 11,11 m2 |

Bande d'identité de scène pour la suite : **2095 à 2187 cellules occupées, en
mode `query`, grille brute.** Toute passe hors de cette bande porte sur une autre
scène et la base sera refaite plutôt que comparée.

C'est aussi la meilleure ligne de base de la série : clairance minimale +0,633 m
contre +0,433 m et +0,410 m aux deux précédentes, et le nombre d'empreintes ne
bouge plus que de 23 à 24 au lieu de 21 à 39.

### Ce que la grille mesure du couloir

Le couloir table–canapé, côté y négatif, largeur libre par tranche de 0,05 m :

```
  x=4.30   une seule travee de 5.35 m   (la table n'est pas encore la)
  x=4.35   1.25 m  de y -1.35 a -0.10
  x=4.45   1.20 m  de y -1.30 a -0.10
  x=4.55   1.05 m  de y -1.20 a -0.15
  x=4.60   1.05 m  de y -1.20 a -0.15
  x=4.65   ferme : 0.10 / 0.20 / 0.35 m en trois morceaux
  x=4.80   plus rien
```

**Le couloir fait 1,05 à 1,25 m de large, très au-dessus du budget de 0,68 m, et
il ne fait que 0,30 m de long.** Il se referme à x = 4,65 m.

Conséquence directe, vérifiée : aucune voie ne le traverse, à aucune largeur.

```
cote table-canape (y de -1.4 a 0.0), largeur la plus etroite qui passerait :
  0.20 m -> 0 voie      0.50 m -> 0 voie
  0.30 m -> 0 voie      0.55 m -> 0 voie
  0.40 m -> 0 voie      0.68 m -> 0 voie
  AUCUNE, jusqu'a 0.20 m : la grille dit que ce cote est bouche.
```

Les seules voies qui franchissent x = 3,5 → 5,2 m au budget de 0,68 m sont à
**y +1,90 à +2,40 m**, c'est-à-dire en contournant la table par la droite, pas
par le couloir.

## 7. Le critère du couloir : ce qui bloque, et ce n'est pas le budget

Trois obstacles, dans l'ordre où ils comptent. Aucun n'est le budget de marge :
1,20 m de couloir contre 0,68 m demandés, il y a 0,26 m de reste par côté.

### 7.1 La patrouille ne va pas jusqu'au couloir

`STOP_AT = 3,10 m`. La table est à **x = 4,5 m**. Le robot fait demi-tour
1,4 m avant de l'atteindre et ne s'en approche jamais — les 60 s ci-dessus le
montrent, la course va de 0,73 à 2,78 m. **Même un couloir parfait ne serait pas
franchi.** Il faut porter `STOP_AT` au-delà du couloir, vers 5,0 m.

### 7.2 La grille referme le couloir à 4,65 m

Et c'est le point qui a besoin du mètre. Deux lectures possibles, exclusives :

- **Le couloir se termine vraiment là.** Il longe le canapé et le canapé
  ressort à x = 4,65 m ; ce n'est pas un passage traversant dans la pièce non
  plus. Alors la grille a raison et c'est la scène qu'il faut réarranger, pour
  que le couloir débouche le long de l'axe de patrouille.
- **Le couloir est réellement ouvert et c'est la perception qui le ferme.**
  Alors le défaut n'est pas dans le budget de marge mais en amont, et l'étape 2
  n'est pas l'endroit où le corriger.

**Votre mesure au mètre tranche.** Si le mètre dit que le passage continue
au-delà de 4,65 m, c'est la seconde lecture.

### 7.3 Restent, une fois 7.1 et 7.2 levés

- [ ] le couloir réel franchi, contre la grille
- [ ] raclages et clairance minimale au moins aussi bons que **0,0 % et
      +0,633 m**, sur une passe dans la bande 2095–2187 cellules

Ce qui est déjà acquis et ne dépend plus de rien :

- [x] une seule grandeur exposée, `CLEARANCE`
- [x] `LANE_SLACK`, `GAP_CLEAR` et `OBSTACLE_CLEAR` retirés de l'interface
- [x] le couloir exigé journalisé en mètres au démarrage, robot compris
- [x] la plage de laps et le compte de cellules dans chaque mesure
- [x] `lane_probe` et `nav_probe` exigent la même largeur que le navigateur,
      par assertion falsifiée et non par relecture

### 7.4 Un double comptage trouvé dans la sonde en préparant ceci

`lane_probe` imprimait `corridor 0.68 m in the grid = 0.92 m of real floor` pour
un robot qui en demande 0,68. En mode `query` la grille est brute, donc
`2 x half` **est déjà** le sol réel, et rajouter `CLEARANCE` le comptait deux
fois — exactement le défaut que cette étape supprime, commis dans l'outil qui le
mesure. Corrigé : `min_corridor()` est la seule expression autorisée à répondre,
dans les deux modes.

## 8. Commits

| commit | contenu |
|---|---|
| `ff29720` | un seul `CLEARANCE`, `LANE_SLACK` et `GAP_CLEAR` retirés, couloir journalisé en mètres |
| `7b16859` | couche d'obstacles brute, inflation à la requête, `pad` longitudinal, assertions sonde/robot, `clearance_probe` |
| `8d4494f` | premier compte rendu, avant la scène |

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
