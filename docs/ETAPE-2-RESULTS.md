# Étape 2 : un seul budget de marge

**État : partie architecture close et mesurée, partie recette EN ATTENTE DE LA
SCÈNE.** Le critère central — un couloir réel mesuré au mètre, franchi — exige
que la table basse soit placée et mesurée, ce que ce document ne peut pas faire.
Ce qui suit sépare strictement les deux : §§1-6 sont mesurés, §7 est la liste de
ce qui manque et ne revendique rien.

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

## 6. Ce qui est mesuré à ce jour

Passe de 60 s, contre la grille, code en mode `query`, **sur la scène telle
qu'elle est aujourd'hui, sans le couloir à démontrer** :

| | ligne de base (`97f2445`, mode dilate) | maintenant (mode query) |
|---|---|---|
| poses / mises à jour ROI | 3457 / 59 | 3502 / 60 |
| **laps couverts** | **252 à 254** | **4 à 7** |
| **raclages** | **0 / 3438 = 0,0 %** | **0 / 3495 = 0,0 %** |
| **clairance minimale** | **+0,433 m** | **+0,410 m** |
| clairance p05 / médiane | +0,727 / +1,438 m | +0,683 / +1,195 m |
| distance parcourue | 2,27 m avant, 1,22 m latéral | 2,05 m avant, 1,27 m latéral |
| couloir exigé | 0,68 m (non journalisé) | **0,68 m, journalisé, vérifié** |

**Ces deux colonnes ne se comparent pas et ne doivent pas être lues comme un
avant/après.** La scène a changé entre les deux : la session du 2026-08-10 a été
interrompue, l'affichage X est passé de `:0` à `:1`, et le mobilier n'a pas été
figé. Le compte des cellules occupées le dit :

| | cellules occupées |
|---|---|
| 2026-08-10, mode dilate | 2624 / 2656 / 2744 |
| 2026-08-11, mode query, grille **brute** | 2005 à 2473, médiane 2249 sur 297 échantillons |

Et ces deux lignes ne sont pas non plus comparables entre elles : **une grille
brute et une grille dilatée n'ont pas le même nombre de cellules par
construction** — mesuré, x1,33 sur la même trame. Le contrôle d'identité de
scène demandé doit donc se faire **à mode égal**, et la bande de référence en
mode `query` est celle de la deuxième ligne.

## 7. Ce qui manque, et pourquoi

Trois critères sur quatre ne sont pas satisfaits, et aucun ne l'est par le code :
ils exigent tous une intervention physique sur la pièce.

- [ ] **La table basse placée pour former le couloir à démontrer, et sa largeur
      mesurée au mètre.** Je ne peux ni déplacer un meuble ni tendre un mètre.
      Sans cela il n'y a pas de couloir à franchir : `clearance_probe` ne trouve
      aujourd'hui aucun passage sous 1,75 m dans cette pièce.
- [ ] **La ligne de base de l'étape 0 refaite sur cette scène figée**, avec les
      laps. Elle doit être prise **après** le placement, sinon elle décrit une
      autre pièce.
- [ ] **Le couloir réel franchi**, contre la grille.
- [ ] **Raclages et clairance minimale au moins aussi bons que cette base.**

Le mobilier doit être figé avant la ligne de base et ne plus bouger jusqu'au
compte rendu. Chaque mesure reportera son nombre de cellules occupées ; deux
passes qui s'écartent de plus que la bande du §6 ne portent pas sur la même
scène et la base sera refaite plutôt que comparée.

**Arithmétique en attendant, qui n'est pas une mesure.** Un couloir de 0,90 m
passe le budget de 0,68 m avec 0,11 m par côté, là où les deux anciens chemins le
passaient de 0,03 m et de -0,02 m. C'est ce que la mesure doit confirmer ou
démentir, pas ce qu'elle remplace.

## 8. Commits

| commit | contenu |
|---|---|
| `ff29720` | un seul `CLEARANCE`, `LANE_SLACK` et `GAP_CLEAR` retirés, couloir journalisé en mètres |
| `7b16859` | couche d'obstacles brute, inflation à la requête, `pad` longitudinal, assertions sonde/robot, `clearance_probe` |

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
