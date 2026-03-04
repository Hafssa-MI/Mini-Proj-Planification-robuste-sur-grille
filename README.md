# Planification Robuste sur Grille : A* + Chaînes de Markov

**Projet Réalisé par :** HAFSSA MIFTAH IDRISSI / SDIA 1  
**Encadrant :** Mr MESTARI <br>
**Date :** 2015/2026  

## 1. Contexte et Objectif Général

Dans de nombreux systèmes réels (robotique, navigation), un agent doit atteindre une cible en minimisant un coût de déplacement, mais ses actions sont soumises à l'incertitude (glissement, panne, vent).

Ce projet propose une solution hybride combinant :
* **La planification déterministe (A\*) :** Pour trouver un chemin optimal sur un graphe en minimisant un coût cumulé.
* **La modélisation stochastique (Chaînes de Markov) :** Pour évaluer la robustesse de ce chemin face à une dynamique incertaine modélisée par une matrice de transition $P$.

---

## 2. Structure du Projet

Ce dépôt est organisé de manière modulaire, séparant la logique algorithmique, la modélisation mathématique et les expérimentations finales :

```text
projet_planification_robuste/
│
├── planner.py          # Module de planification déterministe (A*, UCS, Greedy, Heuristiques)
├── markov.py           # Module stochastique (Construction de la matrice P, P^n, Absorption)
├── simulation.py       # Module de validation empirique (Simulation de Monte-Carlo)
├── experiments.ipynb   # Le Notebook principal regroupant les tests, les graphiques et l'analyse
└── README.md           # Ce rapport de projet
```

---

## 3. Déroulement du Projet et Modélisation

### 3.1 Phase 1 — Définition du cas d’usage et génération de grilles
* **Définir l'environnement :** L'environnement est une grille 2D où chaque cellule libre est un état $n \in S$. Nous définissons les obstacles, l'état de départ $s_0$ et l'état cible $g$ (GOAL).
* **Définir le modèle de coût :** Le coût accumulé depuis l'état initial est $g(n)$, défini ici avec un coût uniforme de 1 par déplacement.
* **Niveau d'incertitude :** Fixation du taux d'erreur $\epsilon$ pour simuler l'incertitude des mouvements de l'agent (avec des variantes : faible, moyen, fort).

<br>
<img width="727" height="770" alt="image" src="https://github.com/user-attachments/assets/b6730097-7859-4e53-8015-3cb9ea7871fe" />
<br>

---


### 3.2 Phase 2 — Planification déterministe avec A\*
* **Implémentation A\* :** Implémentation de l'algorithme sur la grille (avec OPEN = heap, CLOSED = set). La recherche utilise la fonction d'évaluation $f(n) = g(n) + h(n)$, où $h(n)$ est la distance de Manhattan.
* **Discussion sur l'heuristique :** La distance de Manhattan est admissible car elle ne surestime jamais le coût réel (elle suppose un chemin parfait sans obstacles). Elle est également cohérente (h(n) = c(n, n') + h(n')), garantissant que le premier chemin trouvé est optimal sans rouvrir la liste CLOSED.
* **Comparaison des algorithmes :** Nous avons comparé A\* avec Uniform Cost Search (UCS, où $f=g$) et Greedy Search (où $f=h$).
* **Mesures et Résultats :** **Analyse :** UCS explore uniformément dans toutes les directions, ce qui sature la mémoire. Greedy fonce vers la cible mais se fait piéger par les obstacles. A\* offre le compromis parfait : il est guidé par l'heuristique tout en garantissant le chemin le plus court.

<br>
<img width="759" height="574" alt="image" src="https://github.com/user-attachments/assets/1d7502ec-ce71-4348-bfc3-2a9843d7aa25" />
<br>


---


### 3.3 Phase 3 — Construction de la chaîne de Markov induite
À partir du chemin planifié par A\*, on définit une politique (action recommandée à chaque état). En introduisant l'incertitude $\epsilon$, l'agent a une probabilité de $1-\epsilon$ de suivre l'action, et $\epsilon/2$ de dévier latéralement. En cas de collision avec un obstacle, l'agent reste sur sa case.

* **Matrice de transition :** Construction de la matrice $P$ sur les états accessibles (cases libres + GOAL + FAIL si sortie de politique).
* **Vérification :** Contrôle systématique pour s'assurer que $P$ est bien une matrice stochastique (la somme de chaque ligne vaut 1).
* **Évolution temporelle :** Calcul de $\pi^{(n)} = \pi^{(0)}P^n$ par l'équation de Chapman-Kolmogorov pour estimer la probabilité d'être dans l'état GOAL à l'instant $n$.


---


### 3.4 Phase 4 — Analyse Markov (graphe, classes, absorption, période)
* **Graphe orienté :** Construction du graphe des transitions où un arc $i \to j$ existe si $p_{ij} > 0$.
* **Classes de communication :**
  * **États transitoires :** Les cases libres de la grille (l'agent finira par les quitter).
  * **États absorbants / persistants :** La case GOAL ($p_{ii} = 1$) et la case FAIL. Ce sont des classes communicantes fermées réduites à un seul état.
* **Analyse de l'Absorption :** Puisque GOAL et FAIL sont absorbants, nous décomposons $P$ sous sa forme canonique :

$$P = \begin{pmatrix} I & 0 \\\\ R & Q \end{pmatrix}$$

Nous calculons la matrice fondamentale $N = (I - Q)^{-1}$. L'entrée $n_{ij}$ donne le nombre moyen de passages par $j$ depuis $i$. Le temps moyen d'absorption s'obtient via $t = N \cdot \mathbf{1}$. *(Exemple : Pour epsilon = 0.1, le temps moyen théorique est de 9.003 pas).*
* **Périodicité :** La sous-chaîne est apériodique car les états absorbants bouclent sur eux-mêmes (période 1), de même que les collisions contre les murs depuis les états transitoires.


---


### 3.5 Phase 5 — Simulation Monte-Carlo et validation
* **Simulation de trajectoires :** Nous simulons empiriquement $N=1000$ trajectoires Markov à partir de $s_0$ en respectant la politique et les probabilités.
* **Estimations empiriques :** Récolte des données sur la probabilité d'atteindre le GOAL, la distribution du temps d'atteinte et le taux d'échec.<br>
* **Validation croisée (Théorie vs Pratique) :** Nous comparons le calcul matriciel exact $\pi^{(n)}$ avec la simulation Monte-Carlo pour différents taux d'erreur $\epsilon \in \{0, 0.1, 0.2, 0.3\}$.
**Analyse :** La théorie et la pratique se superposent parfaitement, validant la construction de notre matrice $P$. On observe que si le chemin A\* rase les obstacles, une légère augmentation de $\epsilon$ fait chuter drastiquement les chances de survie.
<br>
<img width="728" height="473" alt="image" src="https://github.com/user-attachments/assets/9daf7d66-5d18-4c2a-99cb-2987e8d80eb3" />
<br>


---

## 4. Limites et Pistes d'Amélioration

Bien que performante pour l'analyse, cette approche présente des limites conceptuelles :

* **Planification rigide :** A\* produit un chemin fixe. Dans un environnement incertain, si l'agent dévie de ce chemin, notre modèle actuel considère souvent qu'il échoue (état FAIL) ou tente de revenir sur ses pas aveuglément.
* **Explosion de la mémoire :** Si la grille devient immense, la matrice $P$ de taille $|S| \times |S|$ devient trop lourde à stocker et à inverser mathématiquement pour calculer l'absorption.

**Pistes d'amélioration (Re-planification et MDP) :**
Plutôt que de calculer un chemin déterministe puis de subir l'incertitude, il serait plus pertinent d'utiliser des Processus de Décision Markoviens (MDP) via l'algorithme de Value Iteration. Cela permettrait de générer une politique universelle, où l'agent sait exactement quelle action prendre depuis n'importe quelle case de la grille pour maximiser ses probabilités de survie.
