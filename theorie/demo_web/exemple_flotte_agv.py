# Exemple de Flotte d'AGV pour le Transport de Cartons en Entreprise

## Introduction

# Dans cet exemple concret, nous allons développer un système de flotte d'AGV (Automated Guided Vehicles) pour une entreprise de fabrication qui doit transporter des cartons entre différentes stations de travail. Ce système illustre l'application pratique des concepts de robotique mobile et de robotique en essaim que nous avons étudiés tout au long de ce cours.

## Description du Scénario

# Notre entreprise fictive fabrique des produits électroniques qui passent par plusieurs étapes de traitement :

# 1. **Station d'assemblage initial** : Les composants de base sont assemblés dans des cartons
# 2. **Station de test électronique** : Les assemblages sont testés pour vérifier leur fonctionnement
# 3. **Station de calibration** : Les produits sont calibrés selon les spécifications
# 4. **Station d'emballage** : Les produits sont emballés dans leur conditionnement final
# 5. **Zone d'expédition** : Les produits finis sont regroupés pour l'expédition

# L'usine dispose également de :
# - Une **zone de stockage temporaire** pour les produits en attente de traitement
# - Une **zone de recharge** pour les AGV
# - Des **voies de circulation** définies pour les AGV

## Architecture du Système

### Flotte d'AGV

# Notre flotte se compose de 10 AGV, chacun modélisé comme un robot unicycle à deux roues différentielles avec les caractéristiques suivantes :

# - **Capacité de charge** : 50 kg maximum
# - **Vitesse maximale** : 1.5 m/s
# - **Autonomie** : 8 heures d'opération continue
# - **Capteurs** : LiDAR pour la détection d'obstacles, caméras pour l'identification des cartons, encodeurs pour l'odométrie
# - **Système de préhension** : Fourche élévatrice pour soulever les cartons

### Environnement de l'Usine

# L'usine est modélisée comme une grille 2D de 50m × 30m, avec :

# - Des stations de travail fixes à des positions spécifiques
# - Des voies de circulation bidirectionnelles de 2m de large
# - Des zones de stockage temporaire près de chaque station
# - Des stations de recharge aux extrémités de l'usine

### Système de Gestion des Tâches

# Le système de gestion des tâches est responsable de :

# 1. **Réception des demandes de transport** : Lorsqu'un carton est prêt à être déplacé d'une station à une autre
# 2. **Allocation des tâches** : Attribution des tâches aux AGV disponibles
# 3. **Suivi de l'état** : Surveillance de l'état des AGV et des tâches
# 4. **Optimisation** : Réduction des temps d'attente et des distances parcourues

# ## Implémentation

# Voici une implémentation en Python du système de flotte d'AGV pour le transport de cartons :

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import math
import random
import heapq
from collections import deque
import time

class ManufacturingEnvironment:
    """
    Représentation de l'environnement de l'usine.
    """
    def __init__(self, width=20, height=14, cell_size=0.5):
        """
        Initialise l'environnement de l'usine.
        
        Args:
            width (int): Largeur de l'usine en mètres
            height (int): Hauteur de l'usine en mètres
            cell_size (float): Taille d'une cellule de la grille en mètres
        """
        self.width_meters = width
        self.height_meters = height
        self.cell_size = cell_size
        
        # Conversion en cellules
        self.width = int(width / cell_size)
        self.height = int(height / cell_size)
        
        # Grille d'occupation (0: libre, 1: obstacle)
        self.grid = np.zeros((self.height, self.width))
        
        # Stations de travail (positions ajustées pour le nouvel espace)
        self.stations = {
            "magasin": {"position": (4, 2), "size": (3, 2), "color": "blue"},
            "ariane": {"position": (10, 5), "size": (3, 2), "color": "green"},
            "pas_chargement": {"position": (16, 2), "size": (3, 2), "color": "purple"},
            "shipping": {"position": (16, 8), "size": (3, 3), "color": "red"}
        }
        
        # Zones de stockage temporaire (positions ajustées)
        self.storage_areas = {
            "ariane_storage": {"position": (10, 2), "size": (2, 1.5), "color": "lightgreen"},
            "pas_chargement_storage": {"position": (16, 5), "size": (2, 1.5), "color": "lavender"}
        }
        
        # Stations de recharge (positions ajustées)
        self.charging_stations = [
            {"position": (2, 11), "size": (1.5, 1.5), "color": "yellow"},
            {"position": (18, 11), "size": (1.5, 1.5), "color": "yellow"}
        ]
        
        # Voies de circulation (positions ajustées)
        self.paths = [
            # Voie horizontale principale
            {"start": (0, 6), "end": (20, 6), "width": 1.5},
            # Voies verticales
            {"start": (4, 0), "end": (4, 14), "width": 1.5},
            {"start": (10, 0), "end": (10, 14), "width": 1.5},
            {"start": (16, 0), "end": (16, 14), "width": 1.5}
        ]
        
        # Initialisation de la grille
        self._initialize_grid()
        
        # Graphe de navigation
        self.graph = {}
        self.build_navigation_graph()
    
    def _initialize_grid(self):
        """
        Initialise la grille d'occupation avec les obstacles.
        """
        # Ajout des murs extérieurs
        for i in range(self.height):
            self.grid[i, 0] = 1
            self.grid[i, self.width-1] = 1
        for j in range(self.width):
            self.grid[0, j] = 1
            self.grid[self.height-1, j] = 1
        
        # Ajout des stations de travail comme obstacles
        for station_name, station in self.stations.items():
            pos_x, pos_y = station["position"]
            size_x, size_y = station["size"]
            
            # Conversion en cellules
            cell_x = int(pos_x / self.cell_size)
            cell_y = int(pos_y / self.cell_size)
            cell_size_x = int(size_x / self.cell_size)
            cell_size_y = int(size_y / self.cell_size)
            
            for i in range(cell_y, cell_y + cell_size_y):
                for j in range(cell_x, cell_x + cell_size_x):
                    if 0 <= i < self.height and 0 <= j < self.width:
                        self.grid[i, j] = 1
        
        # Ajout des zones de stockage (non obstacles)
        for area_name, area in self.storage_areas.items():
            pos_x, pos_y = area["position"]
            size_x, size_y = area["size"]
            
            # Conversion en cellules
            cell_x = int(pos_x / self.cell_size)
            cell_y = int(pos_y / self.cell_size)
            cell_size_x = int(size_x / self.cell_size)
            cell_size_y = int(size_y / self.cell_size)
            
            # Les zones de stockage sont accessibles
            for i in range(cell_y, cell_y + cell_size_y):
                for j in range(cell_x, cell_x + cell_size_x):
                    if 0 <= i < self.height and 0 <= j < self.width:
                        self.grid[i, j] = 0
        
        # Ajout des stations de recharge (non obstacles)
        for station in self.charging_stations:
            pos_x, pos_y = station["position"]
            size_x, size_y = station["size"]
            
            # Conversion en cellules
            cell_x = int(pos_x / self.cell_size)
            cell_y = int(pos_y / self.cell_size)
            cell_size_x = int(size_x / self.cell_size)
            cell_size_y = int(size_y / self.cell_size)
            
            # Les stations de recharge sont accessibles
            for i in range(cell_y, cell_y + cell_size_y):
                for j in range(cell_x, cell_x + cell_size_x):
                    if 0 <= i < self.height and 0 <= j < self.width:
                        self.grid[i, j] = 0
        
        # Ajout des voies de circulation (non obstacles)
        for path in self.paths:
            start_x, start_y = path["start"]
            end_x, end_y = path["end"]
            width = path["width"]
            
            # Conversion en cellules
            cell_start_x = int(start_x / self.cell_size)
            cell_start_y = int(start_y / self.cell_size)
            cell_end_x = int(end_x / self.cell_size)
            cell_end_y = int(end_y / self.cell_size)
            cell_width = int(width / self.cell_size)
            
            # Les voies sont horizontales ou verticales
            if cell_start_x == cell_end_x:  # Voie verticale
                for i in range(min(cell_start_y, cell_end_y), max(cell_start_y, cell_end_y) + 1):
                    for j in range(cell_start_x - cell_width//2, cell_start_x + cell_width//2 + 1):
                        if 0 <= i < self.height and 0 <= j < self.width:
                            self.grid[i, j] = 0
            else:  # Voie horizontale
                for i in range(cell_start_y - cell_width//2, cell_start_y + cell_width//2 + 1):
                    for j in range(min(cell_start_x, cell_end_x), max(cell_start_x, cell_end_x) + 1):
                        if 0 <= i < self.height and 0 <= j < self.width:
                            self.grid[i, j] = 0
        # Double check that paths are properly set as free
        for path in self.paths:
            start_x, start_y = path["start"]
            end_x, end_y = path["end"]
            width = path["width"]
            
            # Conversion en cellules
            cell_start_x = int(start_x / self.cell_size)
            cell_start_y = int(start_y / self.cell_size)
            cell_end_x = int(end_x / self.cell_size)
            cell_end_y = int(end_y / self.cell_size)
            cell_width = int(width / self.cell_size)
            
            # Force paths to be free regardless of what was set before
            if cell_start_x == cell_end_x:  # Voie verticale
                for i in range(min(cell_start_y, cell_end_y), max(cell_start_y, cell_end_y) + 1):
                    for j in range(cell_start_x - cell_width//2, cell_start_x + cell_width//2 + 1):
                        if 0 <= i < self.height and 0 <= j < self.width:
                            self.grid[i, j] = 0
            else:  # Voie horizontale
                for i in range(cell_start_y - cell_width//2, cell_start_y + cell_width//2 + 1):
                    for j in range(min(cell_start_x, cell_end_x), max(cell_start_x, cell_end_x) + 1):
                        if 0 <= i < self.height and 0 <= j < self.width:
                            self.grid[i, j] = 0
            # Clear areas in front of stations and storage areas
        for station in list(self.stations.values()) + list(self.storage_areas.values()):
            pos_x, pos_y = station["position"]
            size_x, size_y = station["size"]
            
            # Clear area in front of the station (2 meters)
            cell_x = int(pos_x / self.cell_size)
            cell_y = int((pos_y - 2) / self.cell_size)  # 2 meters in front
            cell_size_x = int(size_x / self.cell_size)
            
            for i in range(cell_y, cell_y + 4):  # 2 meters height
                for j in range(cell_x, cell_x + cell_size_x):
                    if 0 <= i < self.height and 0 <= j < self.width:
                        self.grid[i, j] = 0
    
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
        self.graph = {}
        for y in range(self.height):
            for x in range(self.width):
                if self.grid[y, x] == 0:  # Cellule libre
                    self.graph[(x, y)] = []
                    for (dx, dy), cost in zip(directions, costs):
                        nx, ny = x + dx, y + dy
                        if (0 <= nx < self.width and 0 <= ny < self.height and 
                            self.grid[ny, nx] == 0):
                            # Store neighbor as tuple
                            self.graph[(x, y)].append(((nx, ny), None, cost))
    
    def find_path(self, start, goal):
        """
        Trouve un chemin entre deux points en utilisant l'algorithme A*.
        
        Args:
            start (tuple): Point de départ (x, y) en mètres
            goal (tuple): Point d'arrivée (x, y) en mètres
        
        Returns:
            list: Liste de points formant le chemin [(x, y), ...] en mètres
        """
        print(f"Finding path from {start} to {goal}")
        # Conversion en cellules
        start_cell = (int(start[0] / self.cell_size), int(start[1] / self.cell_size))
        goal_cell = (int(goal[0] / self.cell_size), int(goal[1] / self.cell_size))
        
        # Vérification que les points sont dans le graphe
        if start_cell not in self.graph or goal_cell not in self.graph:
            print(f"Start cell {start_cell} or goal cell {goal_cell} not in graph")
            return None
        
        # Fonction heuristique (distance euclidienne)
        def heuristic(a, b):
            return math.sqrt((b[0] - a[0])**2 + (b[1] - a[1])**2)
        
        # File de priorité pour A*
        open_set = []
        heapq.heappush(open_set, (0, start_cell))
        
        # Dictionnaire pour reconstruire le chemin
        came_from = {}
        
        # Coût du chemin
        g_score = {start_cell: 0}
        
        # Coût estimé total
        f_score = {start_cell: heuristic(start_cell, goal_cell)}
        
        # Ensemble des nœuds visités
        closed_set = set()
        
        while open_set:
            _, current = heapq.heappop(open_set)
            
            if current == goal_cell:
                # Reconstruction du chemin
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                
                # Conversion en mètres
                path_meters = [(x * self.cell_size + self.cell_size/2,
                              y * self.cell_size + self.cell_size/2) for x, y in path]
                return path_meters
            
            closed_set.add(current)
            
            for neighbor, _, cost in self.graph[current]:
                if neighbor in closed_set:
                    continue
                
                tentative_g_score = g_score[current] + cost
                
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = g_score[neighbor] + heuristic(neighbor, goal_cell)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
        
        print(f"No path found from {start} to {goal}")
        return None  # Pas de chemin trouvé
    
    def get_nearest_charging_station(self, position):
        """
        Trouve la station de recharge la plus proche.
        
        Args:
            position (tuple): Position actuelle (x, y) en mètres
        
        Returns:
            tuple: Position de la station de recharge (x, y) en mètres
        """
        min_distance = float('inf')
        nearest_station = None
        
        for station in self.charging_stations:
            pos_x, pos_y = station["position"]
            size_x, size_y = station["size"]
            
            # Centre de la station
            station_center = (pos_x + size_x/2, pos_y + size_y/2)
            
            # Distance
            distance = math.sqrt((position[0] - station_center[0])**2 + (position[1] - station_center[1])**2)
            
            if distance < min_distance:
                min_distance = distance
                nearest_station = station_center
        
        return nearest_station
    
    def get_station_position(self, station_name):
        """
        Retourne la position d'une station de travail.
        
        Args:
            station_name (str): Nom de la station
        
        Returns:
            tuple: Position de la station (x, y) en mètres
        """
        if station_name in self.stations:
            pos_x, pos_y = self.stations[station_name]["position"]
            size_x, size_y = self.stations[station_name]["size"]
            return (pos_x + size_x/2, pos_y + size_y/2 -2)
        
        if station_name in self.storage_areas:
            pos_x, pos_y = self.storage_areas[station_name]["position"]
            size_x, size_y = self.storage_areas[station_name]["size"]
            return (pos_x + size_x/2, pos_y + size_y/2)
        
        return None
    
    def visualize(self, ax=None, agvs=None, cartons=None, paths=None):
        """
        Visualise l'environnement de l'usine.
        
        Args:
            ax (matplotlib.axes.Axes): Axes matplotlib sur lesquels dessiner
            agvs (list): Liste des AGV à afficher
            cartons (list): Liste des cartons à afficher
            paths (list): Liste des chemins à afficher
        
        Returns:
            tuple: (figure, axes) contenant le dessin
        """
        if ax is None:
            fig, ax = plt.subplots(figsize=(15, 10))
        else:
            fig = ax.figure
            # fig, ax = plt.subplots(figsize=(15, 10))
        
        # Reduce the number of grid lines
        ax.grid(True, which='major', linestyle='-', linewidth=0.5, alpha=0.3)
        
        # Use a faster renderer
        ax.set_rasterization_zorder(0)
        
        # Reduce the number of ticks
        ax.set_xticks(np.arange(0, self.width_meters, 5))
        ax.set_yticks(np.arange(0, self.height_meters, 5))
        
        # Use imshow instead of individual patches where possible
        ax.imshow(self.grid.T, cmap='binary', origin='lower', 
                extent=(0, self.width_meters, 0, self.height_meters),
                interpolation='nearest')  # faster interpolation
        
        # Affichage des stations de travail
        for station_name, station in self.stations.items():
            pos_x, pos_y = station["position"]
            size_x, size_y = station["size"]
            color = station["color"]
            
            rect = patches.Rectangle((pos_x, pos_y), size_x, size_y, linewidth=1, edgecolor='black', facecolor=color, alpha=0.5)
            ax.add_patch(rect)
            ax.text(pos_x + size_x/2, pos_y + size_y/2, station_name, ha='center', va='center')
        
        # Affichage des zones de stockage
        for area_name, area in self.storage_areas.items():
            pos_x, pos_y = area["position"]
            size_x, size_y = area["size"]
            color = area["color"]
            
            rect = patches.Rectangle((pos_x, pos_y), size_x, size_y, linewidth=1, edgecolor='black', facecolor=color, alpha=0.5)
            ax.add_patch(rect)
            ax.text(pos_x + size_x/2, pos_y + size_y/2, area_name, ha='center', va='center')
        
        # Affichage des stations de recharge
        for station in self.charging_stations:
            pos_x, pos_y = station["position"]
            size_x, size_y = station["size"]
            color = station["color"]
            
            rect = patches.Rectangle((pos_x, pos_y), size_x, size_y, linewidth=1, edgecolor='black', facecolor=color, alpha=0.5)
            ax.add_patch(rect)
            ax.text(pos_x + size_x/2, pos_y + size_y/2, "Charging", ha='center', va='center')
        
        # Affichage des voies de circulation
        for path in self.paths:
            start_x, start_y = path["start"]
            end_x, end_y = path["end"]
            width = path["width"]
            
            if start_x == end_x:  # Voie verticale
                rect = patches.Rectangle((start_x - width/2, start_y), width, end_y - start_y, linewidth=1, edgecolor='black', facecolor='gray', alpha=0.2)
                ax.add_patch(rect)
            else:  # Voie horizontale
                rect = patches.Rectangle((start_x, start_y - width/2), end_x - start_x, width, linewidth=1, edgecolor='black', facecolor='gray', alpha=0.2)
                ax.add_patch(rect)
        
        # Affichage des AGV
        if agvs:
            for agv in agvs:
                x, y = agv.position
                theta = agv.orientation
                
                # Corps de l'AGV
                agv_radius = 0.5
                circle = patches.Circle((x, y), agv_radius, linewidth=1, edgecolor='black', facecolor='red' if agv.has_carton else 'green', alpha=0.7)
                ax.add_patch(circle)
                
                # Direction (flèche)
                dx = agv_radius * math.cos(theta)
                dy = agv_radius * math.sin(theta)
                ax.arrow(x, y, dx, dy, head_width=0.3, head_length=0.3, fc='black', ec='black')
                
                # Numéro de l'AGV
                ax.text(x, y, str(agv.id), ha='center', va='center', color='white', fontweight='bold')
                
                # Batterie
                battery_text = f"{int(agv.battery_level)}%"
                ax.text(x, y - 0.8, battery_text, ha='center', va='center', color='black', fontsize=8)
        
        # Affichage des cartons
        if cartons:
            for carton in cartons:
                if not carton.picked_up:  # N'affiche pas les cartons portés par les AGV
                    x, y = carton.position
                    
                    # Carton (rectangle)
                    carton_size = 0.4
                    rect = patches.Rectangle((x - carton_size/2, y - carton_size/2), carton_size, carton_size, linewidth=1, edgecolor='black', facecolor='brown', alpha=0.7)
                    ax.add_patch(rect)
                    
                    # ID du carton
                    ax.text(x, y, str(carton.id), ha='center', va='center', color='white', fontsize=8)
        
        # Affichage des chemins
        if paths:
            for path in paths:
                if path:
                    path_x = [p[0] for p in path]
                    path_y = [p[1] for p in path]
                    ax.plot(path_x, path_y, 'b-', linewidth=2, alpha=0.5)
        
        # Configuration de la figure
        ax.set_xlim(0, self.width_meters)
        ax.set_ylim(0, self.height_meters)
        ax.set_title("Simulation de la Flotte d'AGV")
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.grid(True)
        
        plt.tight_layout()
        return fig, ax

class AGV:
    """
    Représentation d'un AGV (Automated Guided Vehicle).
    """
    def __init__(self, id, position, environment, max_speed=3.5, capacity=50):
        """
        Initialise un AGV.
        
        Args:
            id (int): Identifiant unique de l'AGV
            position (tuple): Position initiale (x, y) en mètres
            environment (ManufacturingEnvironment): Environnement de l'usine
            max_speed (float): Vitesse maximale en m/s
            capacity (float): Capacité de charge en kg
        """
        self.id = id
        self.position = position
        self.orientation = 0.0  # rad
        self.environment = environment
        self.max_speed = max_speed
        self.capacity = capacity
        
        self.path = []  # Chemin à suivre
        self.current_path_index = 0
        
        self.battery_level = 100.0  # % de batterie
        self.battery_drain_rate = 0.1  # % par seconde quand en mouvement
        self.charging_rate = 1  # % par seconde quand en charge
        
        self.has_carton = False
        self.carried_carton = None
        
        self.state = "idle"  # États: idle, moving, charging, picking, dropping
        self.target = None  # Point cible (x, y) en mètres
        
        self.last_update_time = time.time()
        self.path_finding_cooldown = 0
    
    def set_target(self, target):
        """
        Définit une cible à atteindre.
        
        Args:
            target (tuple): Point cible (x, y) en mètres
        """
        if self.path_finding_cooldown > 0:
            return
            
        self.target = target
        self.path = self.environment.find_path(self.position, target)
        self.current_path_index = 0
        
        if self.path:
            self.state = "moving"
        else:
            print(f"AGV {self.id}: Pas de chemin trouvé vers {target}")
        self.path_finding_cooldown = 10  # Wait 10 frames before next pathfinding
    
    def update(self, dt, cartons=[]):
        """
        Met à jour l'état de l'AGV.
        
        Args:
            dt (float): Intervalle de temps en secondes
            cartons (list): Liste des cartons dans l'environnement
        """
        self.last_update_time = time.time()
        if self.path_finding_cooldown > 0:
            self.path_finding_cooldown -= 1
        # Mise à jour de la batterie
        if self.state in ["moving", "picking", "dropping"]:
            self.battery_level = max(0.0, self.battery_level - self.battery_drain_rate * dt)
        elif self.state == "charging":
            self.battery_level = min(100.0, self.battery_level + self.charging_rate * dt)
            
            if self.battery_level >= 100.0:
                self.state = "idle"
        
        # Vérification de la batterie faible
        if self.battery_level < 20.0 and self.state != "charging":
            charging_station = self.environment.get_nearest_charging_station(self.position)
            self.set_target(charging_station)
            self.state = "moving"
            print(f"AGV {self.id}: Batterie faible, retour à la station de recharge")
        
        # Mise à jour selon l'état
        if self.state == "moving" and self.path:
            if self.current_path_index < len(self.path):
                target_point = self.path[self.current_path_index]
                
                # Calcul de la direction et de la distance
                dx = target_point[0] - self.position[0]
                dy = target_point[1] - self.position[1]
                distance = math.sqrt(dx*dx + dy*dy)
                
                # Orientation cible
                target_orientation = math.atan2(dy, dx)
                
                # Ajustement de l'orientation (modèle unicycle simplifié)
                orientation_diff = target_orientation - self.orientation
                while orientation_diff > math.pi:
                    orientation_diff -= 2*math.pi
                while orientation_diff < -math.pi:
                    orientation_diff += 2*math.pi
                
                self.orientation += min(0.5 * dt, abs(orientation_diff)) * (8 if orientation_diff > 0 else -1)
                
                # Déplacement
                if abs(orientation_diff) < 1:  # Orientation suffisamment alignée
                    # Calcul du déplacement
                    move_distance = min(self.max_speed * dt, distance)
                    move_ratio = move_distance / distance if distance > 0 else 0
                    
                    # Nouvelle position
                    self.position = (
                        self.position[0] + dx * move_ratio,
                        self.position[1] + dy * move_ratio
                    )

                    # Vérification si le point est atteint
                    if move_distance >= distance:
                        self.current_path_index += 1
                        
                        # Si le chemin est terminé
                        if self.current_path_index >= len(self.path):
                            if self.target == self.environment.get_nearest_charging_station(self.position):
                                self.state = "charging"
                                print(f"AGV {self.id}: Début de la recharge")
                            else:
                                # self.state = "idle"
                                print(f"AGV {self.id}: Cible atteinte")
            else:
                self.state = "idle"
        
        # Recherche d'un carton à proximité si l'AGV est libre
        if self.state == "idle" and not self.has_carton:
            for carton in cartons:
                if not carton.picked_up and carton.assigned_to_agv == self.id:
                    # Distance au carton
                    carton_distance = math.sqrt(
                        (self.position[0] - carton.position[0])**2 + 
                        (self.position[1] - carton.position[1])**2
                    )
                    
                    if carton_distance < 0.5:  # À portée de ramassage
                        self.state = "picking"
                        self.has_carton = True
                        self.carried_carton = carton
                        carton.picked_up = True
                        print(f"\nAGV {self.id} a ramassé le carton {carton.id}")
                        
                        # Définir la cible pour livrer le carton
                        destination = self.environment.get_station_position(carton.destination)
                        if destination:
                            # Force recalculation of path by resetting path finding cooldown
                            self.path_finding_cooldown = 0
                            self.set_target(destination)
                            print(f"AGV {self.id}: En route vers {carton.destination}")
                        break
        
        # Mise à jour de la position du carton transporté
        if self.has_carton and self.carried_carton:
            # Only update carton position when AGV is moving
            if self.state == "moving":
                self.carried_carton.position = self.position
                
                # Vérification si l'AGV a atteint sa destination
                if self.current_path_index >= len(self.path):
                    print(f"\nAGV {self.id} a atteint {self.carried_carton.destination}")
                    print(f"Carton {self.carried_carton.id} déposé")
                    self.has_carton = False
                    # Update the carton's current station to the storage area
                    self.carried_carton.current_station = self.carried_carton.destination
                    self.carried_carton.destination = None
                    self.carried_carton.picked_up = False
                    self.carried_carton.assigned_to_agv = None
                    self.carried_carton = None
                    self.state = "idle"
        
        # Mise à jour des états temporaires
        if self.state == "picking" or self.state == "dropping":
            self.state = "idle"


class Carton:
    """
    Représentation d'un carton à transporter.
    """
    def __init__(self, id, position, weight=10, current_station=None, destination=None):
        """
        Initialise un carton.
        
        Args:
            id (int): Identifiant unique du carton
            position (tuple): Position initiale (x, y) en mètres
            weight (float): Poids du carton en kg
            current_station (str): Station actuelle du carton
            destination (str): Destination du carton
        """
        self.id = id
        self.position = position
        self.weight = weight
        self.current_station = current_station
        self.destination = destination
        
        self.picked_up = False
        self.assigned_to_agv = None
    
    def needs_transport(self):
        """
        Vérifie si le carton a besoin d'être transporté.
        
        Returns:
            bool: True si le carton a besoin d'être transporté
        """
        return self.destination is not None and not self.picked_up and self.assigned_to_agv is None


class TaskManager:
    """
    Gestion des tâches de transport.
    """
    def __init__(self, environment, agvs, workflow):
        """
        Initialise le gestionnaire de tâches.
        
        Args:
            environment (ManufacturingEnvironment): Environnement de l'usine
            agvs (list): Liste des AGV disponibles
            workflow (dict): Flux de travail définissant l'ordre des stations
        """
        self.environment = environment
        self.agvs = agvs
        self.workflow = workflow
        self.cartons = []
        self.next_carton_id = 0
    
    def create_carton(self, station=None):
        """
        Crée un nouveau carton dans le système.
        
        Args:
            station (str): Station de départ, par défaut la première du workflow
        
        Returns:
            Carton: Le carton créé
        """
        if station is None:
            station = list(self.workflow.keys())[0]
        
        # Position à la station
        position = self.environment.get_station_position(station)
        
        # Détermination de la prochaine station dans le workflow
        next_station = self.workflow.get(station)
        
        # Création du carton
        carton = Carton(
            id=self.next_carton_id,
            position=position,
            weight=random.uniform(5, 30),
            current_station=station,
            destination=next_station
        )
        
        self.cartons.append(carton)
        self.next_carton_id += 1
        # Add after creating the carton object
        print(f"DEBUG: Carton {carton.id} created at position {position}, station {station}, destination {next_station}")
        
        return carton
    
    def update_carton_status(self, carton):
        """
        Met à jour le statut d'un carton après livraison.
        
        Args:
            carton (Carton): Le carton à mettre à jour
        """
        # Si le carton a atteint une station, définir la prochaine destination
        if carton.destination is None and carton.current_station in self.workflow:
            next_station = self.workflow.get(carton.current_station)
            print(f"\n=== Carton {carton.id} a atteint la station {carton.current_station} ===")
            
            # Si c'est la dernière station, le carton est terminé
            if next_station is None:
                self.cartons.remove(carton)
                print(f"Carton {carton.id} a terminé le processus de fabrication")
            else:
                carton.destination = next_station
                print(f"Carton {carton.id} prêt pour transport vers {next_station}")
                print(f"Prochaine étape: {next_station}")
    
    def update(self, dt):
        """
        Met à jour l'état du gestionnaire de tâches.
        """
        # Mise à jour des AGV
        for agv in self.agvs:
            if agv.state == "idle" and not agv.has_carton:
                # Find an unassigned carton that needs transport
                available_carton = None
                for carton in self.cartons:
                    if (not carton.picked_up and 
                        not carton.assigned_to_agv and 
                        carton.destination):
                        available_carton = carton
                        break
                
                if available_carton:
                    available_carton.assigned_to_agv = agv.id
                    agv.set_target(available_carton.position)
                    print(f"\nAGV {agv.id} assigné pour récupérer le carton {available_carton.id} à {available_carton.current_station}")
                    print(f"Destination: {available_carton.destination}")
            
            agv.update(dt, self.cartons)
        
        # Mise à jour des cartons
        for carton in list(self.cartons):
            if not carton.picked_up and carton.destination is None:
                self.update_carton_status(carton)
        
        # Création aléatoire de nouveaux cartons
        if random.random() < 0.01:  # 1% de chance par update
            self.create_carton()


class Simulation:
    """
    Simulation de la flotte d'AGV.
    """
    def __init__(self, num_agvs=2, update_interval=0.1):
        """
        Initialise la simulation.
        
        Args:
            num_agvs (int): Nombre d'AGV dans la flotte
            update_interval (float): Intervalle de temps entre les mises à jour en secondes
        """
        self.environment = ManufacturingEnvironment()
        
        # Création des AGV
        self.agvs = []
        for i in range(num_agvs):
            # Position aléatoire sur les voies de circulation
            x = random.uniform(4, 16)  # Adjusted range
            y = random.uniform(4, 8)   # Adjusted range
            
            agv = AGV(id=i, position=(x, y), environment=self.environment)
            self.agvs.append(agv)
        
        # Définition du nouveau workflow
        self.workflow = {
            "magasin": "ariane_storage",
            "ariane_storage": "ariane",
            "ariane": "pas_chargement_storage",
            "pas_chargement_storage": "pas_chargement",
            "pas_chargement": "shipping",
            "shipping": None
        }
        
        # Gestionnaire de tâches
        self.task_manager = TaskManager(self.environment, self.agvs, self.workflow)
        
        # Création de quelques cartons initiaux
        for _ in range(2):
            self.task_manager.create_carton("magasin")
        
        self.update_interval = update_interval
        self.is_running = False
        
        # Animation
        self.fig = None
        self.ax = None
        self.animation = None
    
    def update(self, frame):
        """
        Fonction de mise à jour pour l'animation.
        
        Args:
            frame (int): Numéro de l'image
        """
        if frame%10000 == 0:
                # Print AGV positions
            print("\nAGV Positions:")
            for agv in self.agvs:
                print(f"AGV {agv.id}: position={agv.position}, orientation={agv.orientation:.2f} rad, state={agv.state}")
        

        # Mise à jour du gestionnaire de tâches
        self.task_manager.update(self.update_interval)
        
        # Mise à jour de la visualisation
        self.ax.clear()
        
        # Récupération des chemins
        paths = [agv.path for agv in self.agvs]
        
        # Visualisation de l'environnement
        self.environment.visualize(self.ax, self.agvs, self.task_manager.cartons, paths)
        
        # Titre avec statistiques
        idle_count = sum(1 for agv in self.agvs if agv.state == "idle")
        moving_count = sum(1 for agv in self.agvs if agv.state == "moving")
        charging_count = sum(1 for agv in self.agvs if agv.state == "charging")
        
        title = f"Simulation de la Flotte d'AGV - {frame} updates\n"
        title += f"AGV: {len(self.agvs)} (Idle: {idle_count}, Moving: {moving_count}, Charging: {charging_count})\n"
        title += f"Cartons: {len(self.task_manager.cartons)}"
        
        self.ax.set_title(title)
    
    def run(self, max_frames=500):
        """
        Lance la simulation.
        
        Args:
            max_frames (int): Nombre maximal d'images à afficher
        """
        self.is_running = True
        
        # Création de la figure initiale
        self.fig, self.ax = plt.subplots(figsize=(15, 10))
        # Turn off axis to reduce rendering time
        self.ax.set_axis_off()
        # Enable interactive mode for faster updates
        plt.ion()# Reduce the rendering quality for better performance
        plt.rcParams['figure.dpi'] = 80  # Lower DPI (default is usually 100)
        plt.rcParams['savefig.dpi'] = 80
        
        # Disable maximum window
        plt.rcParams['figure.max_open_warning'] = 0
        
        # Run manual updates with faster refresh
        for frame in range(max_frames):
            if frame % 50 == 0:  # Only update display every 5 frames
                print(f"\n--- Frame {frame} ---")
            self.update(frame)
            plt.draw()
            plt.pause(self.update_interval)
            if not plt.fignum_exists(self.fig.number):
                break

        plt.ioff()
        plt.show()


# Exécution de la simulation
if __name__ == "__main__":
    simulation = Simulation(num_agvs=2)
    simulation.run(max_frames=500)