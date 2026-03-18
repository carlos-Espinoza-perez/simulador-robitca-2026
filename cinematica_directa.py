import numpy as np

import math

def apply_axis_angle(vec, axis, angle):
    v = np.array(vec)
    k = np.array(axis)
    return v * math.cos(angle) + np.cross(k, v) * math.sin(angle) + k * np.dot(k, v) * (1 - math.cos(angle))

def cinematica_directa(angulos, tabla_dh):
    """
    Sustituida la tabla DH clásica por el motor cinemático equivalente a la realidad 
    visual de Three.js, de esta forma la telemetría coincide 100% con la imagen 3D.
    """
    DX = 247.0
    DY = 203.0
    
    pivots = [
        np.array([0.0, 0.0, 0.0]),
        np.array([DX + 65.0, DY, 0.0]),
        np.array([DX + 140.0, DY, 352.0]),
        np.array([DX + 140.0, DY, 712.0]),
        np.array([DX + 70.0, DY, 712.0]),
        np.array([DX + 520.0, DY, 712.0]),
        np.array([DX + 515.0, DY, 712.0])
    ]
    
    axes = [
        np.array([0, 0, 0]),
        np.array([0, 0, 1]),
        np.array([0, 1, 0]),
        np.array([0, 1, 0]),
        np.array([1, 0, 0]),
        np.array([0, 1, 0]),
        np.array([1, 0, 0])
    ]
    
    wrist_axes = [
        np.array([1.0, 0, 0]), 
        np.array([0, 1.0, 0]), 
        np.array([1.0, 0, 0])  
    ]
    
    q_rad = [math.radians(deg) for deg in angulos]
    transformaciones = []
    
    for j in range(1, 7):
        ang = q_rad[j - 1]
            
        origin = pivots[j]
        axis = axes[j]
        
        for p_idx in range(j + 1, 7):
            offset = pivots[p_idx] - origin
            pivots[p_idx] = origin + apply_axis_angle(offset, axis, ang)
            axes[p_idx] = apply_axis_angle(axes[p_idx], axis, ang)
            
        if j >= 4:
            idx_w = j - 4
            for w in range(idx_w, 3):
                wrist_axes[w] = apply_axis_angle(wrist_axes[w], axis, ang)
                
    tcpOffset = 65.0
    tcpPosition = pivots[6] + axes[6] * tcpOffset
    
    X_local = axes[6]
    Y_local = wrist_axes[1]
    Z_local = np.cross(X_local, Y_local)
    
    R = np.column_stack([X_local, Y_local, Z_local])
    
    T_final = np.eye(4)
    T_final[:3, :3] = R
    
    # Restamos las coordenadas de alineación ficticia base para la telemetría real
    T_final[0, 3] = tcpPosition[0] - DX
    T_final[1, 3] = tcpPosition[1] - DY
    T_final[2, 3] = tcpPosition[2]
    
    return T_final, [np.eye(4)] * 6


def obtener_posicion_efector(T):
    posicion = T[:3, 3]
    rotacion = T[:3, :3]
    
    return {
        "posicion": posicion.tolist(),
        "matriz_rotacion": rotacion.tolist(),
        "x": float(posicion[0]),
        "y": float(posicion[1]),
        "z": float(posicion[2])
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
    return (w/norm, x/norm, y/norm, z/norm)
