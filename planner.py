import heapq
import itertools

def manhattan(p1, p2):
    """
    Heuristique : Distance de Manhattan (adaptée pour une grille 4-voisins).
    h(n) = |x - x_g| + |y - y_g|
    """
    return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])

def get_neighbors(grid, node):
    """
    Retourne les voisins valides d'une case (pas de murs, dans les limites).
    """
    x, y = node
    neighbors = []
    rows = len(grid)
    cols = len(grid[0])
    
    # Mouvements : Haut, Bas, Gauche, Droite
    for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
        nx, ny = x + dx, y + dy
        
        # On vérifie les limites et si la case est libre (différente de 1)
        # On suppose que 1 = obstacle
        if 0 <= nx < rows and 0 <= ny < cols and grid[nx][ny] != 1:
            neighbors.append((nx, ny))
            
    return neighbors

def plan(grid, start, goal, algo="A*"):
    """
    Implémente la recherche heuristique (A*, UCS, Greedy).
    
    Retourne : 
    - path : La liste des coordonnées du chemin trouvé.
    - cost : Le coût total du chemin.
    - nodes_expanded : Le nombre de nœuds mis dans CLOSED.
    - max_open_size : La taille maximale atteinte par la liste OPEN (mémoire).
    """
    
    # Compteur pour départager les nœuds ayant le même f_score dans le heap
    counter = itertools.count()
    
    # 1. Structures de données requises
    open_list = []
    closed_set = set() # CLOSED = set
    
    # Le heap stocke : (f_score, tie_breaker, g_score, noeud)
    heapq.heappush(open_list, (0, next(counter), 0, start)) # OPEN = heap
    
    # Dictionnaires de suivi
    came_from = {start: None}
    g_scores = {start: 0}
    
    # 2. Variables pour les métriques
    max_open_size = 0
    nodes_expanded = 0
    
    # 3. Boucle principale de recherche
    while open_list:
        # Mesure de la mémoire
        if len(open_list) > max_open_size:
            max_open_size = len(open_list)
            
        # Extraire le nœud avec le plus petit f_score
        current_f, _, current_g, current = heapq.heappop(open_list)
        
        # Si on a atteint le but
        if current == goal:
            path = reconstruct_path(came_from, start, goal)
            return path, current_g, nodes_expanded, max_open_size
            
        # Si le nœud a déjà été développé, on l'ignore
        if current in closed_set:
            continue
            
        # On ajoute le nœud à CLOSED
        closed_set.add(current)
        nodes_expanded += 1
        
        # 4. Exploration des voisins
        for neighbor in get_neighbors(grid, current):
            if neighbor in closed_set:
                continue
                
            # Le coût pour bouger d'une case est toujours 1 (coût uniforme P1.2)
            tentative_g = current_g + 1
            
            # Si on a trouvé un meilleur chemin vers ce voisin
            if neighbor not in g_scores or tentative_g < g_scores[neighbor]:
                came_from[neighbor] = current
                g_scores[neighbor] = tentative_g
                
                h = manhattan(neighbor, goal)
                
                # 5. Choix de l'algorithme (P2.2)
                if algo == "UCS":
                    f = tentative_g               # f(n) = g(n)
                elif algo == "Greedy":
                    f = h                         # f(n) = h(n)
                elif algo == "A*":
                    f = tentative_g + h           # f(n) = g(n) + h(n)
                else:
                    raise ValueError("Algo non reconnu ('A*', 'UCS', ou 'Greedy').")
                    
                heapq.heappush(open_list, (f, next(counter), tentative_g, neighbor))
                
    # Si la boucle se termine sans trouver le but (chemin bloqué)
    return [], 0, nodes_expanded, max_open_size

def reconstruct_path(came_from, start, goal):
    """
    Remonte le dictionnaire des parents pour créer le chemin sous forme de liste.
    """
    current = goal
    path = []
    while current is not None:
        path.append(current)
        current = came_from[current]
    path.reverse()
    return path

def get_policy_from_path(path):
    """
    Crée une 'politique' (dictionnaire d'actions) à partir du chemin.
    Nécessaire pour la Phase 3 (Markov).
    """
    policy = {}
    for i in range(len(path) - 1):
        # À l'état path[i], l'action recommandée est d'aller vers path[i+1]
        policy[path[i]] = path[i+1]
    return policy