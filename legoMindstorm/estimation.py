import numpy as np
import matplotlib.pyplot as plt
import math

# Paramètres
segment_length = 90 +5  # cm
arc_radius = 25  # cm
arc_perimeter = 75  # cm
direction = -1  # 1 pour droite (défaut), -1 pour gauche

# Calcul de l'angle de l'arc en radians (périmètre = rayon * angle)
arc_angle = arc_perimeter / arc_radius  # en radians

# Coordonnées du segment (vertical de 0 à segment_length)
end_x = -10
segment_x = [0, end_x]
segment_y = [0, segment_length]

# Coordonnées de l'arc de cercle
# Centre de l'arc: bout du segment (0, segment_length)
center_x, center_y = arc_radius + end_x ,  segment_length - arc_radius

# Points de l'arc (départ à angle 0, sens trigonométrique)
theta = np.linspace(math.pi/2, arc_angle, 100)
arc_x = center_x + direction * arc_radius * np.sin(theta)  # direction appliquée ici
arc_y = center_y + arc_radius * (1 - np.cos(theta))   # 1-cos pour partir vers le haut

# Point à l'extrémité de l'arc
end_x = arc_x[-1]
end_y = arc_y[-1]

# Création du plot
plt.figure(figsize=(8, 8))
plt.plot(segment_x, segment_y, 'b-', linewidth=2, label='Segment')
plt.plot(arc_x, arc_y, 'r-', linewidth=2, label='Arc de cercle')
plt.scatter([end_x], [end_y], color='green', s=100, label='Extrémité')

# Ajout des coordonnées du point final
plt.text(end_x + 2 * direction, end_y, f'({end_x:.1f}, {end_y:.1f})', fontsize=12)

# Configuration du graphique
plt.title(f'Segment avec arc de cercle (direction: {"gauche" if direction == -1 else "droite"})')
plt.xlabel('X (cm)')
plt.ylabel('Y (cm)')
plt.axis('equal')  # Même échelle pour x et y
plt.grid(True)
plt.legend()
plt.show()

# Affichage des coordonnées dans la console
print(f"Coordonnées de l'extrémité: ({end_x:.2f}, {end_y:.2f}) cm")
print(f"Direction: {'gauche' if direction == -1 else 'droite'}")