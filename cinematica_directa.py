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

def cinematica_directa(angulos, tabla_dh=None):
    """
    Calcula la Cinemática Directa exacta del ABB IRB 140 usando parámetros DH.
    El origen (0,0,0) es el centro real de la base del robot.
    
    Args:
        angulos: Lista de 6 ángulos en grados [J1, J2, J3, J4, J5, J6]
    Returns:
        T06: Matriz de transformación 4x4 final (Efector / TCP)
        transforms: Lista de las 6 matrices intermedias (para debug o dibujar eslabones)
    """
    # Convertir a radianes
    q = np.radians(angulos)
    pi_2 = np.pi / 2.0
    
    # --- PARÁMETROS DH OFICIALES DEL IRB 140 ---
    # Valores: [theta, d (Z), a (X), alpha (Torsión X)]
    # Nota: J2 tiene un offset de -90° para que coincida con el "Home" vertical de RobotStudio
    
    T1 = matriz_dh(q[0],          352.0,  70.0, -pi_2)
    T2 = matriz_dh(q[1] - pi_2,     0.0, 360.0,   0.0)
    T3 = matriz_dh(q[2],            0.0,   0.0, -pi_2)
    T4 = matriz_dh(q[3],          380.0,   0.0,  pi_2)
    T5 = matriz_dh(q[4],            0.0,   0.0, -pi_2)
    T6 = matriz_dh(q[5],           65.0,   0.0,   0.0)
    
    # Multiplicación secuencial de la cadena cinemática (Acumulación de rotaciones y traslaciones)
    T01 = T1
    T02 = T01 @ T2
    T03 = T02 @ T3
    T04 = T03 @ T4
    T05 = T04 @ T5
    T06 = T05 @ T6  # Posición y orientación final del TCP
    
    # Retornamos la matriz final y el historial de transformaciones
    return T06, [T01, T02, T03, T04, T05, T06]


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