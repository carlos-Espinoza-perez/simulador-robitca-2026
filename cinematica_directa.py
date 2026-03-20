import numpy as np
import math

def matriz_dh(theta, d, a, alpha):
    
    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)
    
    return np.array([
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [ 0,     sa,     ca,    d],
        [ 0,      0,      0,    1]
    ])

def cinematica_directa(angulos, tabla_dh=None, tipos_articulaciones=None):
    
    if tabla_dh is None:

        tabla_dh = [
            [0, 352.0, 0.0, 0.0],
            [0, 0.0, 360.0, 0.0],
            [0, 0.0, 70.0, -90.0],
            [0, 380.0, 0.0, 90.0],
            [0, 0.0, 0.0, -90.0],
            [0, 65.0, 0.0, 0.0]
        ]
        
    num_joints = len(angulos)
    if tipos_articulaciones is None:
        tipos_articulaciones = ["R"] * num_joints
        
    transforms = []
    T_current = np.eye(4)
    pi_2 = np.pi / 2.0
    
    for i in range(num_joints):
        val = angulos[i]
        dh = list(tabla_dh[i]) # [theta_base, d_base, a, alpha]
        
        tipo = tipos_articulaciones[i]
        if tipo == "R":

            theta = np.radians(dh[0] + val)

            d = dh[1]
        elif tipo == "P":

            theta = np.radians(dh[0])
            d = dh[1] + val
        else:
            theta = np.radians(dh[0])
            d = dh[1]
            
        a = dh[2]
        alpha = np.radians(dh[3])

        if num_joints == 6 and tabla_dh[0][1] == 352.0 and i == 0: alpha = -pi_2
        if num_joints == 6 and tabla_dh[0][1] == 352.0 and i == 1: theta -= pi_2
        if num_joints == 6 and tabla_dh[0][1] == 352.0 and i == 2: alpha = -pi_2; dh[2] = 0.0
        if num_joints == 6 and tabla_dh[0][1] == 352.0 and i == 3: alpha = pi_2
        if num_joints == 6 and tabla_dh[0][1] == 352.0 and i == 4: alpha = -pi_2
        
        T_i = matriz_dh(theta, d, a, alpha)
        T_current = T_current @ T_i
        transforms.append(T_current)
        
    T_final = transforms[-1] if transforms else np.eye(4)
    return T_final, transforms

def obtener_posicion_efector(T):
    
    posicion = T[:3, 3]
    rotacion = T[:3, :3]

    return {
        "posicion": np.round(posicion, 3).tolist(),
        "matriz_rotacion": np.round(rotacion, 5).tolist(),
        "x": round(float(posicion[0]), 3),
        "y": round(float(posicion[1]), 3),
        "z": round(float(posicion[2]), 3)
    }

def matriz_a_euler(R):
    
    sy = np.sqrt(R[0, 0]**2 + R[1, 0]**2)

    if sy > 1e-6:
        roll = np.arctan2(R[2, 1], R[2, 2])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = np.arctan2(R[1, 0], R[0, 0])
    else:
        roll = np.arctan2(-R[1, 2], R[1, 1])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = 0
    
    return (np.degrees(roll), np.degrees(pitch), np.degrees(yaw))

def matriz_a_cuaternion(R):
    
    trace = np.trace(R)
    
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    
    norm = np.sqrt(w**2 + x**2 + y**2 + z**2)

    return (round(w/norm, 5), round(x/norm, 5), round(y/norm, 5), round(z/norm, 5))