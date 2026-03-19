import numpy as np
import json
import os
import sys

# Agregar path para importar cinemática
sys.path.append(os.getcwd())
from cinematica_directa import cinematica_directa

def generate_workspace(samples_per_joint=10):
    # Límites típicos del ABB IRB 140 (en grados)
    limites = [
        [-180, 180], # J1
        [-90, 110],  # J2
        [-230, 50],  # J3
        [-200, 200], # J4
        [-115, 115], # J5
        [-400, 400]  # J6
    ]
    
    points = []
    
    # Muestrear
    j1_range = np.linspace(limites[0][0], limites[0][1], samples_per_joint)
    j2_range = np.linspace(limites[1][0], limites[1][1], samples_per_joint)
    j3_range = np.linspace(limites[2][0], limites[2][1], samples_per_joint)
    j5_range = np.linspace(limites[4][0], limites[4][1], 5) # Eje de muñeca
    
    print(f"Generando workspace con {samples_per_joint**3 * 5} combinaciones...")
    
    for q1 in j1_range:
        for q2 in j2_range:
            for q3 in j3_range:
                for q5 in j5_range:
                    # Otros ejes a 0 para simplificar el contorno
                    q = [float(q1), float(q2), float(q3), 0.0, float(q5), 0.0]
                    # cinematica_directa(q) retorna (T06, transforms)
                    T06, _ = cinematica_directa(q)
                    # La posición está en la última columna, primeras 3 filas
                    pos_x = float(T06[0, 3])
                    pos_y = float(T06[1, 3])
                    pos_z = float(T06[2, 3])
                    points.append([pos_x, pos_y, pos_z])
                    
    # Guardar en JSON
    output_path = os.path.join(os.getcwd(), 'workspace_points.json')
    with open(output_path, 'w') as f:
        json.dump(points, f)
    
    print(f"Workspace generado exitosamente: {len(points)} puntos en {output_path}")

if __name__ == "__main__":
    generate_workspace(samples_per_joint=15)
