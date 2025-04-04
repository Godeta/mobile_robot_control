# Swarm Robotics pour AGV en Logistique

## Introduction

La robotique en essaim (swarm robotics) est un domaine fascinant qui s'inspire des comportements collectifs observés dans la nature, comme les colonies de fourmis, les essaims d'abeilles ou les bancs de poissons. Dans cette approche, un grand nombre de robots relativement simples interagissent entre eux et avec leur environnement pour accomplir des tâches complexes qu'un seul robot aurait du mal à réaliser.

Dans le contexte de la logistique, les véhicules à guidage automatique (AGV - Automated Guided Vehicles) sont de plus en plus utilisés pour automatiser le transport de marchandises dans les entrepôts et les centres de distribution. L'application des principes de la robotique en essaim à ces AGV permet d'optimiser les flux logistiques, d'améliorer l'efficacité et la flexibilité des opérations, et de réduire les coûts.

Dans cette section, nous allons développer un exemple concret de swarm robotics appliqué aux AGV en logistique, en nous appuyant sur les modèles et algorithmes présentés dans les sections précédentes.

## Principes de la Robotique en Essaim

### Caractéristiques Principales

La robotique en essaim se caractérise par plusieurs principes fondamentaux :

1. **Décentralisation** : Absence de contrôle centralisé, chaque robot prend ses décisions de manière autonome
2. **Auto-organisation** : Émergence de comportements collectifs complexes à partir d'interactions locales simples
3. **Simplicité individuelle** : Chaque robot possède des capacités limitées en termes de perception, de calcul et d'action
4. **Robustesse** : Le système continue de fonctionner même si certains robots tombent en panne
5. **Flexibilité** : Adaptation facile à différentes tailles de flotte et différentes tâches
6. **Évolutivité** : Possibilité d'ajouter ou de retirer des robots sans reconfiguration majeure

### Avantages pour la Logistique

L'application de ces principes aux AGV en logistique offre plusieurs avantages :

1. **Résilience** : Si un AGV tombe en panne, les autres peuvent prendre le relais
2. **Adaptabilité** : Le système peut s'adapter à des changements dans la disposition de l'entrepôt ou dans les flux de travail
3. **Optimisation des ressources** : Allocation dynamique des tâches en fonction de la charge de travail
4. **Réduction des temps d'inactivité** : Les AGV peuvent être constamment réaffectés à de nouvelles tâches
5. **Évolutivité** : Facilité d'ajout de nouveaux AGV pour augmenter la capacité

## Modélisation d'un Système d'AGV en Essaim

### Architecture du Système

Notre système d'AGV en essaim comprendra les composants suivants :

1. **Flotte d'AGV** : Ensemble de robots unicycles à deux roues différentielles
2. **Environnement** : Représentation de l'entrepôt avec des zones de stockage, des stations de chargement et des voies de circulation
3. **Tâches** : Opérations de transport à effectuer (collecte et livraison de marchandises)
4. **Système de communication** : Mécanisme permettant aux AGV d'échanger des informations
5. **Algorithmes de coordination** : Méthodes pour allouer les tâches et éviter les collisions

### Modèle de l'AGV

Chaque AGV est modélisé comme un robot unicycle à deux roues différentielles, avec les caractéristiques suivantes :

- **État** : Position (x, y), orientation θ, vitesses linéaire v et angulaire ω
- **Capacités** : Charge maximale, autonomie de la batterie, vitesse maximale
- **Capteurs** : Détection d'obstacles, localisation, identification des marchandises
- **Actionneurs** : Moteurs des roues, mécanisme de chargement/déchargement

Le modèle cinématique de chaque AGV est celui que nous avons développé précédemment :

```
ẋ = v·cos(θ)
ẏ = v·sin(θ)
θ̇ = ω
```

### Modèle de l'Environnement

L'environnement est représenté par une carte 2D de l'entrepôt, avec les éléments suivants :

- **Zones de stockage** : Emplacements où les marchandises sont stockées
- **Stations de chargement** : Emplacements où les AGV peuvent recharger leurs batteries
- **Voies de circulation** : Chemins que les AGV peuvent emprunter
- **Obstacles** : Éléments fixes ou mobiles à éviter

La carte peut être représentée par une grille d'occupation, où chaque cellule indique si elle est libre ou occupée par un obstacle.

### Modèle des Tâches

Chaque tâche de transport est définie par :

- **Point de collecte** : Emplacement où la marchandise doit être récupérée
- **Point de livraison** : Emplacement où la marchandise doit être déposée
- **Priorité** : Niveau d'urgence de la tâche
- **Charge** : Poids ou volume de la marchandise à transporter

## Algorithmes de Coordination

### Allocation des Tâches

L'allocation des tâches consiste à déterminer quel AGV doit effectuer quelle tâche. Plusieurs approches sont possibles :

1. **Allocation centralisée** : Un système central attribue les tâches aux AGV
2. **Allocation distribuée** : Les AGV négocient entre eux pour se répartir les tâches
3. **Allocation par enchères** : Les AGV enchérissent pour obtenir les tâches en fonction de leur proximité et de leur disponibilité

Nous allons implémenter une allocation par enchères, qui offre un bon compromis entre décentralisation et efficacité.

### Évitement de Collisions

L'évitement de collisions est crucial dans un système multi-robots. Plusieurs approches sont possibles :

1. **Champs de potentiel** : Chaque AGV est soumis à des forces répulsives des obstacles et des autres AGV
2. **Fenêtres dynamiques** : Chaque AGV calcule une fenêtre de vitesses admissibles en fonction des obstacles
3. **Règles de priorité** : Des règles simples déterminent quel AGV doit céder le passage
4. **Réservation d'espace-temps** : Les AGV réservent des zones de l'environnement pour des intervalles de temps spécifiques

Nous allons implémenter une approche basée sur les champs de potentiel, qui est simple à mettre en œuvre et efficace pour les environnements dynamiques.

## Implémentation en Python

Voici une implémentation en Python d'un système d'AGV en essaim pour la logistique :

```python
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
import math
import random
import heapq

class Environment:
    """
    Représentation de l'environnement (entrepôt).
    """
    def __init__(self, width=50, height=50, cell_size=1.0):
        """
        Initialise l'environnement.
        
        Args:
            width (int): Largeur de l'environnement en cellules
            height (int): Hauteur de l'environnement en cellules
            cell_size (float): Taille d'une cellule en mètres
        """
        self.width = width
        self.height = height
        self.cell_size = cell_size
        
        # Grille d'occupation (0: libre, 1: obstacle)
        self.grid = np.zeros((height, width))
        
        # Zones spéciales
        self.storage_zones = []  # [(x, y, width, height), ...]
        self.charging_stations = []  # [(x, y), ...]
        self.pickup_zones = []  # [(x, y), ...]
        self.delivery_zones = []  # [(x, y), ...]
        
        # Graphe de navigation
        self.graph = {}  # {(x, y): [(x', y', cost), ...], ...}
    
    def add_obstacle(self, x, y, width, height):
        """
        Ajoute un obstacle rectangulaire.
        
        Args:
            x (int): Coordonnée x du coin supérieur gauche
            y (int): Coordonnée y du coin supérieur gauche
            width (int): Largeur de l'obstacle en cellules
            height (int): Hauteur de l'obstacle en cellules
        """
        for i in range(y, min(y + height, self.height)):
            for j in range(x, min(x + width, self.width)):
                self.grid[i, j] = 1
    
    def add_storage_zone(self, x, y, width, height):
        """
        Ajoute une zone de stockage.
        
        Args:
            x (int): Coordonnée x du coin supérieur gauche
            y (int): Coordonnée y du coin supérieur gauche
            width (int): Largeur de la zone en cellules
            height (int): Hauteur de la zone en cellules
        """
        self.storage_zones.append((x, y, width, height))
        
        # Marquer la zone comme obstacle (sauf les bords pour l'accès)
        for i in range(y + 1, y + height - 1):
            for j in range(x + 1, x + width - 1):
                self.grid[i, j] = 1
    
    def add_charging_station(self, x, y):
        """
        Ajoute une station de chargement.
        
        Args:
            x (int): Coordonnée x
            y (int): Coordonnée y
        """
        self.charging_stations.append((x, y))
    
    def add_pickup_zone(self, x, y):
        """
        Ajoute une zone de collecte.
        
        Args:
            x (int): Coordonnée x
            y (int): Coordonnée y
        """
        self.pickup_zones.append((x, y))
    
    def add_delivery_zone(self, x, y):
        """
        Ajoute une zone de livraison.
        
        Args:
            x (int): Coordonnée x
            y (int): Coordonnée y
        """
        self.delivery_zones.append((x, y))
    
    def build_navigation_graph(self):
        """
        Construit le graphe de navigation pour la planification de chemin.
        """
        # Directions possibles (8-connectivité)
        directions = [
            (0, 1), (1, 0), (0, -1), (-1, 0),  # Horizontales et verticales
            (1, 1), (1, -1), (-1, 1), (-1, -1)  # Diagonales
        ]
        
        # Coûts associés aux directions
        costs = [1.0, 1.0, 1.0, 1.0, 1.414, 1.414, 1.414, 1.414]
        
        # Construction du graphe
        for y in range(self.height):
            for x in range(self.width):
                if self.grid[y, x] == 0:  # Cellule libre
                    self.graph[(x, y)] = []
                    for (dx, dy), cost in zip(directions, costs):
                        nx, ny = x + dx, y + dy
                        if (0 <= nx < self.width and 0 <= ny < self.height and 
                            self.grid[ny, nx] == 0):
                            self.graph[(x, y)].append((nx, ny, cost))
    
    def find_path(self, start, goal):
        """
        Trouve un chemin entre deux points en utilisant l'algorithme A*.
        
        Args:
            start (tuple): Point de départ (x, y)
            goal (tuple): Point d'arrivée (x, y)
        
        Returns:
            list: Liste de points formant le chemin [(x, y), ...]
        """
        # Vérification que les points sont dans le graphe
        if start not in self.graph or goal not in self.graph:
            return None
        
        # Fonction heuristique (distance euclidienne)
        def heuristic(a, b):
            return math.sqrt((b[0] - a[0])**2 + (b[1] - a[1])**2)
        
        # File de priorité pour A*
        open_set = []
        heapq.heappush(open_set, (0, start))
        
        # Dictionnaire pour reconstruire le chemin
        came_from = {}
        
        # Coût du chemin
        g_score = {start: 0}
        
        # Coût estimé total
        f_score = {start: heuristic(start, goal)}
        
        # Ensemble des nœuds visités
        closed_set = set()
        
        while open_set:
            _, current = heapq.heappop(open_set)
            
            if current == goal:
                # Reconstruction du chemin
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                return path
            
            closed_set.add(current)
            
            for neighbor, _, cost in self.graph[current]:
                if neighbor in closed_set:
                    continue
                
                tentative_g_score = g_score[current] + cost
                
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = g_score[neighbor] + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
        
        return None  # Pas de chemin trouvé
    
    def visualize(self, agvs=None, paths=None):
        """
        Visualise l'environnement.
        
        Args:
            agvs (list): Liste des AGV à afficher
            paths (list): Liste des chemins à afficher
        """
        fig, ax = plt.subplots(figsize=(10, 10))
        
        # Affichage de la grille
        ax.imshow(self.grid.T, cmap='binary', origin='lower')
        
        # Affichage des zones de stockage
        for x, y, w, h in self.storage_zones:
            rect = patches.Rectangle((x, y), w, h, linewidth=1, edgecolor='b', facecolor='none')
            ax.add_patch(rect)
        
        # Affichage des stations de chargement
        for x, y in self.charging_stations:
            ax.plot(x, y, 'gs', markersize=10)
        
        # Affichage des zones de collecte
        for x, y in self.pickup_zones:
            ax.plot(x, y, 'ro', markersize=8)
        
        # Affichage des zones de livraison
        for x, y in self.delivery_zones:
            ax.plot(x, y, 'go', markersize=8)
        
        # Affichage des AGV
        if agvs:
            for agv in agvs:
                x, y = agv.x, agv.y
                theta = agv.theta
                
                # Corps de l'AGV
                circle = patches.Circle((x, y), 0.4, color='r')
                ax.add_patch(circle)
                
                # Direction
                dx = 0.6 * math.cos(theta)
                dy = 0.6 * math.sin(theta)
                ax.arrow(x, y, dx, dy, head_width=0.2, head_length=0.3, fc='r', ec='r')
                
                # Identifiant
                ax.text(x, y, str(agv.id), color='w', ha='center', va='center')
        
        # Affichage des chemins
        if paths:
            for path in paths:
                if path:
                    xs, ys = zip(*path)
                    ax.plot(xs, ys, 'g-', linewidth=2, alpha=0.7)
        
        ax.set_xlim(0, self.width)
        ax.set_ylim(0, self.height)
        ax.set_title('Environnement de l\'entrepôt')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        
        plt.grid(True)
        plt.show()

class Task:
    """
    Représentation d'une tâche de transport.
    """
    def __init__(self, id, pickup, delivery, priority=1, load=1.0):
        """
        Initialise une tâche.
        
        Args:
            id (int): Identifiant de la tâche
            pickup (tuple): Point de collecte (x, y)
            delivery (tuple): Point de livraison (x, y)
            priority (int): Priorité de la tâche (1-10)
            load (float): Charge à transporter
        """
        self.id = id
        self.pickup = pickup
        self.delivery = delivery
        self.priority = priority
        self.load = load
        self.status = "pending"  # pending, assigned, in_progress, completed
        self.assigned_to = None  # AGV assigné à la tâche
        self.pickup_time = None
        self.delivery_time = None
    
    def __str__(self):
        return f"Task {self.id}: {self.pickup} -> {self.delivery} (Priority: {self.priority}, Status: {self.status})"

class AGV:
    """
    Représentation d'un AGV (Automated Guided Vehicle).
    """
    def __init__(self, id, x, y, theta=0.0, v_max=1.0, omega_max=1.0, capacity=10.0, battery_capacity=100.0):
        """
        Initialise un AGV.
        
        Args:
            id (int): Identifiant de l'AGV
            x (float): Position initiale en x
            y (float): Position initiale en y
            theta (float): Orientation initiale
            v_max (float): Vitesse linéaire maximale
            omega_max (float): Vitesse angulaire maximale
            capacity (float): Capacité de charge maximale
         
(Content truncated due to size limit. Use line ranges to read in chunks)