# La sur-projection de 0,15 m : d'où elle vient

Établi dans `docs/ETAPE-2-RESULTS.md` §7.1 et §7.3bis : sur 15 trames, la grille
d'occupation rapporte 0,10 à 0,20 m de largeur libre de moins que le mètre, des
deux côtés de la même table, et ce manque refuse un passage de 0,70 m dans lequel
un robot de 0,44 m tient. Un défaut de perception qui décide d'un verdict de
navigation.

Trois candidats, **chiffrés séparément**. Les corriger ensemble aurait laissé
personne capable de dire lequel comptait.

---

## Le verdict, d'abord

| candidat | coût mesuré | statut |
|---|---|---|
| **quantification de cellule** | **+0,220 m** de largeur libre | **la cause dominante** |
| filtres de profondeur | **+0,000 m** sur le chiffre livré | masqué par la cellule |
| masque de segmentation | **+0,000 m** | hors de cause |

**La quantification explique tout le manque et davantage.** Ce n'est ni le
modèle ni le capteur : c'est `points_to_grid` qui plancher-arrondit chaque point
dans une cellule de 0,05 m.

---

## 1. La cellule : +0,220 m

`points_to_grid` fait `ix = ((f - x0) / cell).astype(np.int32)`, donc un plancher.
Un objet grossit vers l'extérieur d'au plus une cellule de chaque côté, et
l'**écart libre** entre deux objets se réduit donc d'au plus deux cellules, soit
0,10 m à 0,05 m de cellule.

Isolé en re-rastérisant les **points identiques** — même trame, même
déprojection, même masque — en ne changeant que la cellule :

| cellule | largeur libre (masque) | largeur libre (géométrie seule) |
|---|---|---|
| **0,050 m** (livré) | **0,750 m** | 0,750 m |
| 0,020 m | 0,760 m | 0,760 m |
| 0,010 m | 0,800 m | 0,800 m |
| 0,005 m | 0,850 m | 0,850 m |

Sur 8 trames, filtres allumés, la configuration livrée :

| | médiane | plage |
|---|---|---|
| cellule 0,05 m | **0,750 m** | 0,750 à 0,900 |
| cellule 0,005 m | **0,970 m** | 0,850 à 1,090 |
| **coût de la quantification** | **+0,220 m** | |

0,005 m sert de référence parce qu'elle est cent fois plus fine que le bruit de
bord de l'objet lui-même : ce qu'elle rend n'est que la cellule.

**0,220 m est plus que les 0,15 m constatés au §7.1.** L'écart tient à ce que les
deux mesures ne portent pas sur la même grandeur : le §7.1 comparait la travée la
plus large au mètre, celle-ci compare la même travée à elle-même à deux
résolutions. Les deux disent la même chose — la cellule mange plus que la marge
disponible.

Noter aussi la dispersion : à 0,05 m la lecture est **coincée à 0,750 m** sur 6
trames sur 8, alors qu'à 0,005 m elle varie de 0,850 à 1,090. La cellule ne fait
pas que rétrécir, elle **écrase la variation** et donne une fausse impression de
stabilité.

## 2. Les filtres de profondeur : +0,000 m sur ce qui est livré

Le soupçon était raisonnable : le filtre spatial et le remplissage de trous de la
RealSense font leur pire travail exactement à une discontinuité de profondeur,
c'est-à-dire au bord d'un objet. Mesuré, 8 trames de chaque côté :

| | cellule 0,05 m (livrée) | cellule 0,005 m |
|---|---|---|
| `DEPTH_FILTERS=1` | **0,750 m** | 0,970 m |
| `DEPTH_FILTERS=0` | **0,750 m** | 0,917 m |
| différence | **0,000 m** | −0,053 m |

**Éteindre les filtres ne change rien à ce que le robot voit.** Ils coûtent bien
quelque chose sur la mesure fine — environ 0,05 m, et dans le sens *inattendu* :
la largeur libre est plus grande **avec** les filtres qu'avec la profondeur brute,
donc le remplissage de trous ajoute du sol plutôt que de l'obstacle. Mais à la
cellule livrée, la quantification l'avale entièrement.

C'est le résultat le plus utile de la série : **il ne sert à rien de toucher aux
filtres tant que la cellule est à 0,05 m.**

**Défaut trouvé en le mesurant.** `DEPTH_FILTERS` était écrit en dur à `"1"` dans
`docker-compose.yml` alors que le commentaire juste au-dessus disait « mettre à 0
pour voir la profondeur brute ». Ce n'était pas possible sans éditer le fichier.
Corrigé en `${DEPTH_FILTERS:-1}`.

## 3. Le masque de segmentation : +0,000 m

Isolé sans toucher au capteur, en construisant la même occupation à partir d'un
critère purement **géométrique** — hauteur au-dessus du plan du sol — qui n'a
besoin d'aucun modèle. Si la silhouette YOLO était plus large que l'objet, la
version géométrique donnerait un écart libre plus grand.

| | largeur libre à la cellule livrée |
|---|---|
| silhouette YOLO | 0,750 m |
| hauteur seule, sans modèle | 0,750 m |
| **coût du masque** | **+0,000 m** |

Identiques à la cellule près, et identiques aussi aux quatre résolutions du §1.
**La silhouette n'est pas plus large que la chose qui se trouve là**, sur ce
bord. Candidat écarté.

Une réserve : ceci mesure le bord qui borne **ce passage**. Un masque qui
déborderait ailleurs — sur un objet que la géométrie voit mal, une chaise à
pieds fins — ne serait pas vu par ce test.

## 4. Ce que cela veut dire pour l'étape 2

Le refus du côté TV au §7.3bis était attribué à « la sur-projection ». C'est
juste, mais le mot est trop vague : **c'est la quantification de la grille**, pas
la perception au sens du capteur ou du modèle.

La conséquence est différente, et meilleure : un défaut de rastérisation se
corrige sans toucher ni au détecteur ni au capteur.

## 5. Ce qui n'est pas fait

**Rien n'est corrigé.** Le sujet était de trouver la cause, et les trois chiffres
sont là. Les pistes, dans l'ordre où elles se défendent :

- **Rastériser plus fin.** 0,02 m rendrait environ 0,05 m et coûterait 6,25 fois
  plus de cellules — 400x400 au lieu de 160x160. Le coût est mesurable avec
  `scripts/grid_profile.py` et ne l'a pas été.
- **Rastériser conservativement du bon côté.** Le vrai défaut n'est pas la
  finesse, c'est que l'arrondi va toujours vers *plus d'obstacle*. Un obstacle
  marqué par le centre de cellule le plus proche plutôt que par plancher perdrait
  le biais sans changer la taille de la grille — au prix d'une garantie de
  sûreté qu'il faudrait examiner, un obstacle sous-couvert étant pire qu'un
  passage refusé.
- **Ne rien faire et le documenter**, ce qui est l'état actuel : la grille est
  conservatrice de 0,22 m sur une largeur de passage, et `CLEARANCE` est un
  budget qui s'y ajoute.

Le choix n'est pas technique, il est de sûreté : les 0,22 m sont aujourd'hui une
marge cachée. Les récupérer, c'est marcher plus près des meubles.

## 6. Reproduire

```bash
docker compose run --rm --no-deps --entrypoint python3 \
  -v $PWD/scripts:/scripts:ro -v $PWD/common:/opt/edgebot:ro \
  -v $PWD/config:/config:ro \
  perception /scripts/surproj_probe.py --x 3.5 --band -1.6 -0.1 --tape 1.20

DEPTH_FILTERS=0 docker compose up -d source      # puis relancer la sonde
docker compose up -d source                      # retour au defaut
```

---

# Partie II : rendre le biais explicite — mesuré, et la réponse est non

Objectif : si le biais pire cas est **borné et connu**, `CLEARANCE` peut en tenir
compte et le budget cesse de contenir une marge invisible. La prémisse a été
testée avant d'être utilisée, et **elle ne tient pas**.

## 7. Correction : ma référence à 0,005 m était fausse

La partie I chiffrait la quantification à **+0,220 m** en comparant la grille
livrée à une grille de 0,005 m. Cette référence n'est pas valide.

Densité mesurée du nuage d'objet : 104 753 points sur 2881 cellules de 0,05 m,
soit ~14 400 points/m², soit **8 mm entre points**. Une grille de 5 mm est donc
**plus fine que le nuage** : l'objet s'y troue tout seul, chaque intervalle entre
deux points devient une cellule libre, et la « largeur libre » qu'elle rapporte
est du bruit d'échantillonnage, pas du sol.

C'est visible dans le tableau de la partie I, que je n'avais pas su lire :
0,750 → 0,760 → 0,800 → 0,850 quand la cellule descend de 0,05 à 0,005. Le saut
est dans les deux dernières lignes, celles qui passent sous l'espacement du
nuage.

**Le chiffre de +0,220 m de la partie I est donc surestimé.** Les mesures
ci-dessous le remplacent.

## 8. Le biais, contre une référence valide

706 paires de travées appariées sur 6 trames, chaque travée fine comparée à celle
de la grille livrée qui la contient.

| référence | n | moyen | médiane | p90 | p99 | pire | valide ? |
|---|---|---|---|---|---|---|---|
| **0,020 m** | 706 | **+0,076** | **+0,060** | +0,170 | +0,430 | **+0,500** | oui, cellule > espacement |
| 0,010 m | 590 | +0,120 | +0,090 | +0,250 | +0,460 | +0,490 | oui, à la limite |
| 0,005 m | 473 | +0,137 | +0,110 | +0,265 | +0,465 | +0,490 | **non**, plus fine que le nuage |

**Le biais typique est petit : 0,060 m de médiane, 0,076 m de moyenne.** Loin des
0,22 m annoncés en partie I.

**Mais la queue n'est pas bornée : p99 à +0,430 m, pire cas +0,500 m.** La borne
théorique de deux cellules, 0,100 m, est dépassée dans une travée sur trois.

### Pourquoi la borne théorique ne tient pas

« Un objet grossit d'au plus une cellule de chaque côté » décrit le déplacement
d'un **bord**. Ce n'est pas ce qui arrive à une travée étroite : une travée réelle
de 0,15 m tombe à zéro dans une grille de 0,05 m dès que ses deux bords tombent
dans la même cellule ou dans deux cellules adjacentes. La travée n'est pas
rétrécie de 0,10 m, elle est **supprimée**. Ces cas-là peuplent la queue.

Le mécanisme que j'avais supposé — des points isolés remplissant des cellules
entières — est écarté par la mesure : seules **4,3 %** des cellules occupées
tiennent à un seul point et 8,1 % à deux ou moins, la médiane étant de 19 points
par cellule. Les cellules sont bien peuplées ; c'est la géométrie des travées
étroites qui produit la queue, pas le bruit.

## 9. Conséquence : ne pas replier ce biais dans `CLEARANCE`

La prémisse du sujet était « si le biais pire cas est borné et connu ». Il n'est
ni l'un ni l'autre.

Ajouter une constante à `CLEARANCE` serait juste en moyenne et **faux exactement
là où le budget décide** : dans les passages étroits, qui sont ceux dont la queue
est faite. Un budget explicite fondé sur une moyenne serait moins honnête que la
marge cachée qu'il remplace, parce qu'il aurait l'air d'être une garantie.

## 10. Le coût de 0,02 m, mesuré avant de l'écarter

L'intuition du §6 de l'étape 5 se vérifie : c'est `points_to_grid` qui domine, pas
la taille de grille.

| cellule | grille | `points_to_grid` | dilatation | cellules | grille sur le bus |
|---|---|---|---|---|---|
| **0,05 m** (livrée) | 160x160 | **1,45 ms** | 0,009 ms | 2881 | 3,1 ko |
| **0,02 m** | 400x400 | **1,98 ms** | 0,030 ms | 13 012 | 19,5 ko |
| 0,01 m | 800x800 | 1,98 ms | 0,085 ms | 31 076 | 78,1 ko |

**+0,53 ms et +16 ko**, payés à la fréquence de publication du ROI, mesurée à
0,99 Hz. C'est 0,05 % du temps. La morphologie reste négligeable à 0,030 ms, et
`points_to_grid` ne bouge presque pas parce que son coût est dans le nombre de
**points**, pas dans le nombre de cellules.

À 0,02 m le biais médian tomberait de 0,060 m à ce que la référence 0,02 m
mesure, c'est-à-dire à peu près rien, et la queue serait divisée par 2,5 sur le
même raisonnement géométrique — une travée doit alors être sous 0,06 m pour
disparaître, au lieu de 0,15 m.

## 11. Recommandation, et ce qui n'est pas fait

**Rastériser à 0,02 m plutôt que replier le biais dans `CLEARANCE`.** Le coût est
mesuré et négligeable, la queue est traitée à la source au lieu d'être compensée
en moyenne, et le budget de marge reste ce qu'il dit être.

**Rien n'est appliqué.** `GRID_CELL` est une constante de module dans
`common/edgebot/floor.py`, pas un réglage : la changer touche le compositeur, le
navigateur, les deux ponts suite et toutes les sondes, et invalide toutes les
mesures de grille des documents précédents. Cela mérite son propre passage, avec
une nouvelle ligne de base.

