"""
Sistema de interpolación para movimientos suaves del robot
Genera puntos intermedios entre posiciones para animación fluida
"""

import numpy as np
from typing import List, Dict, Any
from scipy.spatial.transform import Slerp, Rotation


def interpolar_joints(q_inicio: List[float], q_fin: List[float], num_pasos: int = 20) -> List[List[float]]:
    """
    Interpolación lineal en espacio de articulaciones
    
    Args:
        q_inicio: Ángulos iniciales [q1, q2, q3, q4, q5, q6] en grados
        q_fin: Ángulos finales [q1, q2, q3, q4, q5, q6] en grados
        num_pasos: Número de puntos intermedios a generar
        
    Returns:
        Lista de configuraciones intermedias
    """
    q_inicio = np.array(q_inicio)
    q_fin = np.array(q_fin)
    
    # Generar interpolación lineal
    trayectoria = []
    for i in range(num_pasos + 1):
        t = i / num_pasos
        q_intermedio = q_inicio + t * (q_fin - q_inicio)
        trayectoria.append(q_intermedio.tolist())
    
    return trayectoria


def interpolar_lineal_cartesiano(
    pos_inicio: List[float], 
    quat_inicio: List[float],
    pos_fin: List[float], 
    quat_fin: List[float],
    num_pasos: int = 20
) -> List[Dict[str, Any]]:
    """
    Interpolación lineal en espacio cartesiano (MoveL)
    Posición: interpolación lineal
    Orientación: interpolación esférica (SLERP)
    
    Args:
        pos_inicio: Posición inicial [x, y, z] en mm
        quat_inicio: Cuaternión inicial [qx, qy, qz, qw]
        pos_fin: Posición final [x, y, z] en mm
        quat_fin: Cuaternión final [qx, qy, qz, qw]
        num_pasos: Número de puntos intermedios
        
    Returns:
        Lista de poses intermedias con 'position' y 'quaternion'
    """
    pos_inicio = np.array(pos_inicio)
    pos_fin = np.array(pos_fin)
    
    # Interpolación lineal de posición
    trayectoria = []
    
    # SLERP para orientación
    rot_inicio = Rotation.from_quat(quat_inicio)
    rot_fin = Rotation.from_quat(quat_fin)
    
    tiempos = np.linspace(0, 1, num_pasos + 1)
    slerp = Slerp([0, 1], Rotation.concatenate([rot_inicio, rot_fin]))
    
    for t in tiempos:
        # Posición interpolada
        pos_intermedia = pos_inicio + t * (pos_fin - pos_inicio)
        
        # Orientación interpolada (SLERP)
        rot_intermedia = slerp([t])[0]
        quat_intermedia = rot_intermedia.as_quat()
        
        trayectoria.append({
            'position': pos_intermedia.tolist(),
            'quaternion': quat_intermedia.tolist()
        })
    
    return trayectoria


def interpolar_circular(
    pos_inicio: List[float],
    quat_inicio: List[float],
    pos_via: List[float],
    quat_via: List[float],
    pos_fin: List[float],
    quat_fin: List[float],
    num_pasos: int = 30
) -> List[Dict[str, Any]]:
    """
    Interpolación circular en espacio cartesiano (MoveC)
    Genera arco circular que pasa por punto inicial, vía y final
    
    Args:
        pos_inicio: Posición inicial [x, y, z]
        quat_inicio: Cuaternión inicial
        pos_via: Punto intermedio del arco [x, y, z]
        quat_via: Cuaternión en punto vía
        pos_fin: Posición final [x, y, z]
        quat_fin: Cuaternión final
        num_pasos: Número de puntos en el arco
        
    Returns:
        Lista de poses en el arco circular
    """
    p1 = np.array(pos_inicio)
    p2 = np.array(pos_via)
    p3 = np.array(pos_fin)
    
    # Calcular centro y radio del círculo que pasa por los 3 puntos
    centro, radio = calcular_circulo_3_puntos(p1, p2, p3)
    
    # Calculate normal to the plane using the 3 points to avoid zero vectors for semi-circles
    v_12 = p2 - p1
    v_13 = p3 - p1
    cross_p = np.cross(v_12, v_13)
    
    if centro is None or np.linalg.norm(cross_p) < 1e-6:
        # Puntos colineales o error al hallar centro, usar interpolación lineal
        print("Puntos colineales en MoveC, usando interpolación lineal")
        return interpolar_lineal_cartesiano(pos_inicio, quat_inicio, pos_fin, quat_fin, num_pasos)
        
    normal = cross_p / np.linalg.norm(cross_p)
    
    # Vectores desde el centro a los puntos para ángulos
    v1 = p1 - centro
    v3 = p3 - centro
    
    sin_total = np.dot(np.cross(v1, v3), normal)
    cos_total = np.dot(v1, v3)
    angulo_total = np.arctan2(sin_total, cos_total)
    
    # El ángulo total siempre debe ser positivo debido a que la normal (cross_p)
    # se define siguiendo la regla de la mano derecha de P1 -> P2 -> P3
    if angulo_total <= 0:
        angulo_total += 2 * np.pi
    
    # Generar puntos en el arco
    trayectoria = []
    
    # SLERP para orientación
    rot_inicio = Rotation.from_quat(quat_inicio)
    rot_fin = Rotation.from_quat(quat_fin)
    slerp = Slerp([0, 1], Rotation.concatenate([rot_inicio, rot_fin]))
    
    for i in range(num_pasos + 1):
        t = i / num_pasos
        angulo = angulo_total * t
        
        # Rotar v1 alrededor de la normal
        pos_intermedia = centro + rotar_vector(v1, normal, angulo)
        
        # Orientación interpolada
        rot_intermedia = slerp([t])[0]
        quat_intermedia = rot_intermedia.as_quat()
        
        trayectoria.append({
            'position': pos_intermedia.tolist(),
            'quaternion': quat_intermedia.tolist()
        })
    
    return trayectoria


def calcular_circulo_3_puntos(p1: np.ndarray, p2: np.ndarray, p3: np.ndarray):
    """
    Calcula el centro y radio del círculo que pasa por 3 puntos en 3D
    
    Returns:
        (centro, radio) o (None, None) si los puntos son colineales
    """
    # Vectores
    v1 = p2 - p1
    v2 = p3 - p1
    
    # Verificar si son colineales
    cross = np.cross(v1, v2)
    if np.linalg.norm(cross) < 1e-6:
        return None, None
    
    # Normal al plano
    normal = cross / np.linalg.norm(cross)
    
    # Puntos medios
    m1 = (p1 + p2) / 2
    m2 = (p1 + p3) / 2
    
    # Vectores perpendiculares en el plano
    perp1 = np.cross(v1, normal)
    perp2 = np.cross(v2, normal)
    
    # Resolver sistema para encontrar centro
    # m1 + t1*perp1 = m2 + t2*perp2
    A = np.column_stack([perp1, -perp2])
    b = m2 - m1
    
    try:
        # Resolver en 2D (proyectar al plano)
        u1 = v1 / np.linalg.norm(v1)
        u2 = np.cross(normal, u1)
        
        # Proyectar al sistema 2D
        m1_2d = np.array([np.dot(m1 - p1, u1), np.dot(m1 - p1, u2)])
        m2_2d = np.array([np.dot(m2 - p1, u1), np.dot(m2 - p1, u2)])
        perp1_2d = np.array([np.dot(perp1, u1), np.dot(perp1, u2)])
        perp2_2d = np.array([np.dot(perp2, u1), np.dot(perp2, u2)])
        
        A_2d = np.column_stack([perp1_2d, -perp2_2d])
        b_2d = m2_2d - m1_2d
        
        t = np.linalg.solve(A_2d, b_2d)
        
        # Centro en 3D
        centro = m1 + t[0] * perp1
        radio = np.linalg.norm(p1 - centro)
        
        return centro, radio
    except:
        return None, None


def rotar_vector(v: np.ndarray, eje: np.ndarray, angulo: float) -> np.ndarray:
    """
    Rota un vector alrededor de un eje usando la fórmula de Rodrigues
    
    Args:
        v: Vector a rotar
        eje: Eje de rotación (normalizado)
        angulo: Ángulo en radianes
        
    Returns:
        Vector rotado
    """
    cos_ang = np.cos(angulo)
    sin_ang = np.sin(angulo)
    
    return (v * cos_ang + 
            np.cross(eje, v) * sin_ang + 
            eje * np.dot(eje, v) * (1 - cos_ang))


def calcular_num_pasos_por_velocidad(velocidad: str, distancia: float = None) -> int:
    """
    Calcula el número de pasos de interpolación realista según la velocidad (mm/s)
    y la distancia real (mm), asumiendo que el servidor actualiza a 20 FPS (50ms).
    
    Args:
        velocidad: String de velocidad (ej. v100, v500, v1000, vMAX)
        distancia: Distancia física del movimiento en mm (opcional)
        
    Returns:
        Número de pasos de interpolación (mínimo 2)
    """
    import re
    match = re.search(r'v(\d+)', velocidad.lower())
    
    if match:
        vel_mm_s = float(match.group(1))
    elif 'max' in velocidad.lower():
        vel_mm_s = 2000.0  # Asumimos vMAX en este simulador
    else:
        vel_mm_s = 1000.0  # Default fallback
        
    # Evitamos divisiones por 0
    if vel_mm_s <= 0:
        vel_mm_s = 1000.0
        
    # El simulador en `execute_rapid_stream` usa sleep(0.05) -> 20 FPS
    tiempo_por_paso = 0.05 
    
    if distancia is not None and distancia > 0:
        # Tiempo real que RobotStudio tardaría = Distancia (mm) / Velocidad (mm/s)
        tiempo_total_segundos = distancia / vel_mm_s
        pasos = tiempo_total_segundos / tiempo_por_paso
    else:
        # Modo fallback: Si no pasa distancia, aproximamos como si el robot
        # estuviera moviendo una magnitud promedio (ej. 300 milimetros)
        distancia_media = 300.0
        tiempo_total_segundos = distancia_media / vel_mm_s
        pasos = tiempo_total_segundos / tiempo_por_paso

    # Restringimos a 500 iteraciones por instrucción (tope de seguridad ~25 segundos)
    # y mínimo garantizamos 2 pasos para que exista un movimiento interpolado visual
    return min(max(int(round(pasos)), 2), 500)
