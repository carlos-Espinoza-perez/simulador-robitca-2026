import numpy as np
import math

def matriz_dh(theta, d, a, alpha):
    """
    Calcula la Matriz de Transformación Homogénea 4x4 según la convención Denavit-Hartenberg.
    """
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
    """
    Calcula la Cinemática Directa general usando parámetros DH.
    
    Args:
        angulos: Lista de N ángulos en grados (o desplazamientos en mm para prismáticas)
        tabla_dh: Lista de parámetros DH [theta, d, a, alpha]
        tipos_articulaciones: Lista de 'R' (rotacional) o 'P' (prismática)
    Returns:
        T_final: Matriz de transformación 4x4 final (Efector / TCP)
        transforms: Lista de las N matrices intermedias
    """
    if tabla_dh is None:
        # Fallback a IRB 140 por compatibilidad con scripts viejos (ej. generar_workspace.py)
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
            # Articulación rotacional: sumamos el ángulo al theta base
            # Convertir el valor que viene en "grados" a radianes y sumar
            # Pero dh[0] está en grados en la tabla_dh de especificaciones_robot.py
            theta = np.radians(dh[0] + val)
            
            # IRB 140 Legacy Support: Si no pasaron tabla_dh y i==1, a veces se restan 90 grados
            # Para evitar complicar, confiamos en tabla_dh puramente, pero en IRB140 viejo:
            # J2=q[1]-pi_2. No lo aplicamos globalmente si pasamos tabla_dh correcto.
            
            d = dh[1]
        elif tipo == "P":
            # Articulación prismática: sumamos el valor a d base
            theta = np.radians(dh[0])
            d = dh[1] + val
        else:
            theta = np.radians(dh[0])
            d = dh[1]
            
        a = dh[2]
        alpha = np.radians(dh[3])
        
        # IRB 140 Legacy Support hardcodeado solo si tabla_dh es el exacto por defecto
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
    """
    Extrae la posición y rotación de una matriz homogénea 4x4.
    """
    posicion = T[:3, 3]
    rotacion = T[:3, :3]
    
    # Redondeo a 3 decimales para evitar ruido de punto flotante en la API (ej. 1.000000000001e-16)
    return {
        "posicion": np.round(posicion, 3).tolist(),
        "matriz_rotacion": np.round(rotacion, 5).tolist(),
        "x": round(float(posicion[0]), 3),
        "y": round(float(posicion[1]), 3),
        "z": round(float(posicion[2]), 3)
    }


def matriz_a_euler(R):
    """
    Convierte una matriz de rotación 3x3 a ángulos de Euler (Roll, Pitch, Yaw).
    """
    sy = np.sqrt(R[0, 0]**2 + R[1, 0]**2)
    
    # Singularidad (Gimbal Lock) cuando Pitch es +/- 90 grados
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
    """
    Convierte una matriz de rotación 3x3 a Cuaternión.
    Formato de salida: (w, x, y, z) -> w es la parte real (q1 en RobotStudio)
    """
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
    
    # Redondeo para sanear la salida JSON hacia el frontend
    return (round(w/norm, 5), round(x/norm, 5), round(y/norm, 5), round(z/norm, 5))