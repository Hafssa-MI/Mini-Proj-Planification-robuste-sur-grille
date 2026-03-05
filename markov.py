import numpy as np

def get_state_mapping(grid):
    """
    Associe chaque case libre de la grille à un index entier (0, 1, 2...).
    C'est indispensable car une matrice P prend des indices (P[i][j]), pas des coordonnées (x,y).
    """
    states = []
    rows, cols = len(grid), len(grid[0])
    for x in range(rows):
        for y in range(cols):
            if grid[x][y] != 1:  # 1 = obstacle
                states.append((x, y))
                
    # Dictionnaires pour passer facilement de la coordonnée à l'index et inversement
    state_to_idx = {state: i for i, state in enumerate(states)}
    idx_to_state = {i: state for i, state in enumerate(states)}
    
    return states, state_to_idx, idx_to_state

def get_laterals(current, intended):
    """
    Calcule les deux cases latérales par rapport à la direction voulue.
    """
    dx = intended[0] - current[0]
    dy = intended[1] - current[1]
    
    # Si on se déplace en (dx, dy), les côtés sont (dy, dx) et (-dy, -dx)
    lat1 = (current[0] + dy, current[1] + dx)
    lat2 = (current[0] - dy, current[1] - dx)
    
    return lat1, lat2

def is_valid(grid, state):
    """Vérifie si une case est dans la grille et n'est pas un obstacle."""
    x, y = state
    rows, cols = len(grid), len(grid[0])
    return 0 <= x < rows and 0 <= y < cols and grid[x][y] != 1

def build_transition_matrix(grid, policy, goal, epsilon):
    """
    Construit la matrice de transition P.
    """
    states, state_to_idx, _ = get_state_mapping(grid)
    n_states = len(states)
    P = np.zeros((n_states, n_states))
    
    for state in states:
        i = state_to_idx[state]
        
        # 1. État absorbant : Le GOAL
        if state == goal:
            P[i, i] = 1.0
            continue
            
        # 2. Si l'état n'est pas dans la politique (l'agent a dévié et s'est perdu)
        # On considère qu'il reste bloqué (état FAIL absorbant) pour simplifier
        if state not in policy:
            P[i, i] = 1.0
            continue
            
        # 3. Mouvement normal selon la politique
        intended = policy[state]
        lat1, lat2 = get_laterals(state, intended)
        
        # Fonction interne pour ajouter la probabilité à la bonne destination
        def add_prob(dest_state, prob):
            # Si on fonce dans un mur ou hors de la carte, on reste sur place (state)
            if not is_valid(grid, dest_state):
                dest_state = state 
            j = state_to_idx[dest_state]
            P[i, j] += prob
            
        # Appliquer les probabilités selon l'équation de Markov
        add_prob(intended, 1 - epsilon)   # Succès
        add_prob(lat1, epsilon / 2)       # Glissade côté 1
        add_prob(lat2, epsilon / 2)       # Glissade côté 2

    return P, state_to_idx

def is_stochastic(P):
    """
    Vérifie que P est stochastique (la somme de chaque ligne vaut 1).
    On utilise np.isclose pour éviter les erreurs d'arrondis (ex: 0.9999999 == 1.0).
    """
    row_sums = np.sum(P, axis=1)
    return np.all(np.isclose(row_sums, 1.0))

def calculate_pi_n(P, start_idx, n):
    """
    Calcule pi^(n) = pi^(0) * P^n
    """
    # Vecteur initial pi^(0) : 100% de chance d'être au départ, 0% ailleurs
    pi_0 = np.zeros(len(P))
    pi_0[start_idx] = 1.0
    
    # Élever la matrice P à la puissance n
    P_n = np.linalg.matrix_power(P, n)
    
    # Calculer la distribution à l'instant n
    pi_n = np.dot(pi_0, P_n)
    return pi_n

def analyze_absorption(P, goal_idx):
    """
    Calculs d'absorption (N, temps moyen).
    P doit être réorganisée virtuellement pour séparer les états transitoires (Q) 
    des états absorbants. 
    """
    # Identifier les états absorbants (ceux dont P[i,i] == 1)
    absorbing_states = [i for i in range(len(P)) if np.isclose(P[i, i], 1.0)]
    transient_states = [i for i in range(len(P)) if i not in absorbing_states]
    
    # Extraire la sous-matrice Q (transitions d'états transitoires vers transitoires)
    Q = P[np.ix_(transient_states, transient_states)]
    
    # Calculer la matrice fondamentale N = (I - Q)^-1
    I = np.eye(len(Q))
    try:
        N = np.linalg.inv(I - Q)
    except np.linalg.LinAlgError:
        # Si la matrice est singulière, c'est qu'il y a des boucles infinies sans fuite
        return None
        
    # Temps moyen avant absorption pour chaque état transitoire : t = N * 1
    t = np.dot(N, np.ones(len(Q)))
    
    # On renvoie le temps moyen sous forme de dictionnaire {index_transitoire: temps}
    mean_times = {transient_states[idx]: time for idx, time in enumerate(t)}
    return mean_times