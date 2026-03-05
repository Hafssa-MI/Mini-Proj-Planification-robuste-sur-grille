import random
import numpy as np

from markov import get_laterals, is_valid

def simulate_trajectory(grid, start, goal, policy, epsilon, max_steps=100):
    """
    Simule UNE trajectoire Markov à partir de s0.
    Retourne un booléen (succès/échec) et le nombre d'étapes effectuées.
    """
    current = start
    steps = 0
    
    while current != goal and steps < max_steps:
        # Si l'agent sort de la politique (il a dévié et ne sait plus quoi faire)
        # On considère qu'il a échoué (état FAIL absorbant)
        if current not in policy:
            break
            
        intended = policy[current]
        lat1, lat2 = get_laterals(current, intended)
        
        # Tirage aléatoire pour déterminer le mouvement réel
        rand_val = random.random()
        
        if rand_val < (1 - epsilon):
            next_state = intended       # 1 - epsilon : Succès
        elif rand_val < (1 - epsilon + epsilon / 2):
            next_state = lat1           # epsilon / 2 : Glissade côté 1
        else:
            next_state = lat2           # epsilon / 2 : Glissade côté 2
            
        # Gestion des collisions 
        if not is_valid(grid, next_state):
            next_state = current
            
        current = next_state
        steps += 1
        
    reached_goal = (current == goal)
    return reached_goal, steps

def run_monte_carlo(grid, start, goal, policy, epsilon, n_simulations=1000, max_steps=100):
    """
    Simule N trajectoires et estime empiriquement les statistiques.
    """
    success_count = 0
    times_to_goal = []
    
    for _ in range(n_simulations):
        success, steps = simulate_trajectory(grid, start, goal, policy, epsilon, max_steps)
        
        if success:
            success_count += 1
            times_to_goal.append(steps)
            
    # Calcul des statistiques (estimateurs empiriques)
    prob_success = success_count / n_simulations
    failure_rate = 1.0 - prob_success
    
    # Temps moyen (uniquement sur les trajectoires qui ont réussi)
    avg_time = np.mean(times_to_goal) if times_to_goal else float('inf')
    
    return prob_success, failure_rate, avg_time, times_to_goal