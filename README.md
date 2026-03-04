# Planification Robuste sur Grille : A* + Chaînes de Markov

**Projet Réalisé par :** HAFSSA MIFTAH IDRISSI / SDIA 1  
**Encadrant :** Mr MESTARI <br>
**Date :** Mars 2026  

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


## 3. Modélisation Mathématique

### 3.1 Espace d'états et Planification (A\*)
L'environnement est une grille 2D où chaque cellule libre est un état $n \in S$. La recherche heuristique utilise la fonction d'évaluation :
$$f(n)=g(n)+h(n)$$
* $g(n)$ : Coût accumulé depuis l'état initial $s_0$ (coût uniforme de 1 par déplacement).
* $h(n)$ : Estimation du coût restant vers le but $g$. Nous utilisons la distance de Manhattan.

**Discussion sur l'heuristique (Admissibilité et Cohérence) :**
La distance de Manhattan est admissible car elle ne surestime jamais le coût réel pour atteindre la cible (elle suppose un chemin parfait sans obstacles). Elle est également cohérente car elle respecte l'inégalité triangulaire : $h(n) \le c(n, n') + h(n')$. C'est cette cohérence qui garantit que le premier chemin trouvé par A\* vers un état est le chemin optimal, évitant ainsi de rouvrir des nœuds dans la liste CLOSED.

### 3.2 Modèle Stochastique (Chaîne de Markov)
L'agent suit le plan généré par A\*, mais ses actions sont perturbées par un taux d'erreur $\epsilon$ :
* Probabilité de $1-\epsilon$ de suivre l'action recommandée.
* Probabilité de $\epsilon/2$ de dévier sur chaque côté latéral.
* En cas de collision avec un obstacle, l'agent reste sur sa case.



L'évolution de la distribution de probabilité sur les états est donnée par l'équation de Chapman-Kolmogorov :
$$\pi^{(n)}=\pi^{(0)}P^n$$
Où $P$ est la matrice stochastique de transition.


---


## 4. Expériences et Résultats

### 4.1 Comparaison des Algorithmes Déterministes (E.1 & E.3)
Nous avons comparé UCS ($f=g$), Greedy ($f=h$) et A\* ($f=g+h$) sur une grille contenant des obstacles.

**Analyse :** UCS explore uniformément dans toutes les directions, ce qui sature la mémoire. Greedy fonce vers la cible mais se fait piéger par les obstacles (chemin non optimal). A\* offre le compromis parfait : il est guidé par l'heuristique tout en garantissant le chemin le plus court grâce au coût $g$.

*[Insérer ici une capture d'écran d'un graphique en barres comparant les nœuds explorés]*

### 4.2 Impact de l'Incertitude et Validation Monte-Carlo (E.2)
Nous avons fixé le chemin A\* et fait varier le taux d'erreur $\epsilon \in \{0, 0.1, 0.2, 0.3\}$. Nous comparons le calcul matriciel exact ($\pi^{(n)}$) avec une simulation empirique Monte-Carlo (1000 trajectoires).

**Résultats :**

*[Insérer ici la courbe superposant la probabilité théorique et empirique en fonction de $\epsilon$]*

**Analyse :** La théorie et la pratique se superposent parfaitement, validant la construction de notre matrice $P$. On observe que si le chemin A\* rase les obstacles, une légère augmentation de $\epsilon$ fait chuter drastiquement les chances de survie de l'agent.

---

##  5. Analyse Markovienne Approfondie (Phase 4)

### 5.1 Classes de communication
Notre chaîne de Markov contient plusieurs types d'états :
* **États transitoires :** Les cases libres de la grille. L'agent finira par les quitter définitivement.
* **États absorbants :** La case GOAL ($p_{ii} = 1$) et éventuellement une case FAIL (erreur fatale/sortie de politique). Ce sont des classes communicantes fermées réduites à un seul état.

### 5.2 Analyse de l'Absorption
Pour étudier le temps moyen avant que l'agent n'atteigne le but (ou n'échoue), nous décomposons la matrice $P$ sous sa forme canonique :

$$P = \begin{pmatrix} I & 0 \\ R & Q \end{pmatrix}$$

En extrayant la sous-matrice $Q$ (transitions entre états transitoires), nous calculons la matrice fondamentale $N$ :

$$N = (I - Q)^{-1}$$

L'entrée $n_{ij}$ de $N$ donne le nombre moyen de passages par l'état $j$ sachant qu'on part de $i$. Le temps moyen d'absorption s'obtient via $t = N \cdot \mathbf{1}$.

*(Exemple de résultat : Pour $\epsilon = 0.1$, le temps moyen calculé mathématiquement depuis le départ jusqu'à l'absorption est de X pas).*

---

## 6. Limites et Pistes d'Amélioration

Bien que performante pour l'analyse, cette approche présente des limites conceptuelles :

* **Planification rigide :** A\* produit un chemin fixe. Dans un environnement incertain, si l'agent dévie de ce chemin, notre modèle actuel considère souvent qu'il échoue (état FAIL) ou tente de revenir sur ses pas aveuglément.
* **Mémoire :** Si la grille devient immense, la matrice $P$ de taille $|S| \times |S|$ devient trop lourde à stocker et à inverser mathématiquement pour calculer l'absorption.

**Pistes d'amélioration (Re-planification et MDP) :**
Plutôt que de calculer un chemin déterministe puis de subir l'incertitude, il serait plus pertinent d'utiliser des Processus de Décision Markoviens (MDP) via l'algorithme de Value Iteration. Cela permettrait de générer une politique universelle, où l'agent sait exactement quelle action prendre depuis n'importe quelle case de la grille pour maximiser ses probabilités de survie.


