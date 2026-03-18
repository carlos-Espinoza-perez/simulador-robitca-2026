import sys
import numpy as np

from cinematica_directa import cinematica_directa, obtener_posicion_efector
from especificaciones_robot import ROBOTS

robot = ROBOTS["ABB_IRB_140"]
tabla_dh = robot["tabla_dh"]

# ZERO POSITION
angulos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

T_final, transformaciones = cinematica_directa(angulos, tabla_dh)
pose = obtener_posicion_efector(T_final)

print("Posicion en ZERO:", pose['x'], pose['y'], pose['z'])

# P10 test
from cinematica_inversa import solve_ik_from_robtarget
robot_limits = robot["limites_articulares"]
quaternion = [4.14816E-8, 6.1133E-9, -1, -2.53589E-16]
res1 = solve_ik_from_robtarget([450, 0, 450], quaternion, robot_limits, from_robotstudio=False)
T_final_ik, transformaciones_ik = cinematica_directa(res1['joints'], tabla_dh)
pose_ik = obtener_posicion_efector(T_final_ik)
print("Posicion calculada del IK (deberia ser 450,0,450):", pose_ik['x'], pose_ik['y'], pose_ik['z'])

