"""
Cinemática Inversa para el simulador
Adaptado del digital twin del IRB 140
"""

import numpy as np


def transform_robotstudio_to_robot_rotated(position, quaternion=None):
    """
    Transforma coordenadas de RobotStudio al sistema del robot rotado en Three.js
    
    Three.js rota el robot -90° en X, lo que causa:
    - X_rs → X_robot (frente se mantiene)
    - Y_rs → Z_robot (izquierda → hacia arriba en visualización)
    - Z_rs → -Y_robot (arriba → hacia atrás en visualización)
    
    Para que un movimiento en +X de RobotStudio se vea hacia el frente:
    Necesitamos mapear correctamente considerando la rotación del frontend
    
    Args:
        position: [x, y, z] en coordenadas RobotStudio (mm)
        quaternion: [qx, qy, qz, qw] en coordenadas RobotStudio (opcional)
    
    Returns:
        position_robot: [x, y, z] para la IK
        quaternion_robot: [qx, qy, qz, qw] para la IK (si se proporciona quaternion)
    """
    from scipy.spatial.transform import Rotation
    
    x_rs, y_rs, z_rs = position
    
    # TRANSFORMACIÓN para compensar la rotación -90° en X del frontend
    # El robot en Three.js está rotado -90° en X para ponerlo vertical
    # Necesitamos aplicar la rotación INVERSA (+90° en X) a las coordenadas
    # 
    # Rotación +90° en X transforma:
    #   X → X (se mantiene)
    #   Y → -Z (Y va hacia atrás)
    #   Z → Y (Z va hacia arriba)
    
    x_robot = x_rs    # X se mantiene (frente)
    y_robot = z_rs    # Z de RS → Y del robot (arriba RS → lateral robot)
    z_robot = -y_rs   # Y de RS → -Z del robot (lateral RS → hacia atrás robot)
    
    position_robot = [x_robot, y_robot, z_robot]
    
    if quaternion is not None and len(quaternion) == 4:
        # Transformar orientación aplicando rotación +90° en X
        r_rs = Rotation.from_quat(quaternion)
        
        # Rotación de compensación: +90° en X (inversa de la rotación del frontend)
        r_ajuste = Rotation.from_euler('x', 90, degrees=True)
        
        # Aplicar: primero la orientación de RS, luego la compensación
        r_robot = r_ajuste * r_rs
        
        quaternion_robot = r_robot.as_quat().tolist()
        return position_robot, quaternion_robot
    
    # Si no hay cuaternión, retornar None para el cuaternión
    return position_robot, None


def transform_robotstudio_to_threejs(position, quaternion=None):
    """
    Transforma coordenadas de RobotStudio (ABB) a Three.js
    
    Sistema DH del robot:
    - En home [0,0,0,0,0,0], el TCP está en [350, 445, 352]
    - +Y_DH = frente del robot (brazo extendido)
    - +Z_DH = arriba
    - +X_DH = lateral
    
    RobotStudio (ABB):
        X = Frente/Atrás (positivo = frente)
        Y = Izquierda/Derecha (positivo = izquierda)
        Z = Arriba/Abajo (positivo = arriba)
    
    Transformación correcta:
        X_DH = -Y_abb  (izquierda ABB → -lateral DH)
        Y_DH = X_abb   (frente ABB → frente DH)
        Z_DH = Z_abb   (arriba ABB → arriba DH)
    
    Args:
        position: [x, y, z] en coordenadas ABB (mm)
        quaternion: [qx, qy, qz, qw] en coordenadas ABB (opcional)
    
    Returns:
        position_three: [x, y, z] en coordenadas DH del robot
        quaternion_three: [qx, qy, qz, qw] en coordenadas DH (si se proporciona)
    """
    from scipy.spatial.transform import Rotation
    
    x_abb, y_abb, z_abb = position
    
    # Transformación correcta basada en el sistema DH
    x_dh = -y_abb  # Izquierda ABB → -Lateral DH
    y_dh = x_abb   # Frente ABB → Frente DH (brazo extendido)
    z_dh = z_abb   # Arriba ABB → Arriba DH
    
    position_three = [x_dh, y_dh, z_dh]
    
    if quaternion is not None:
        # Transformar orientación
        r_abb = Rotation.from_quat(quaternion)
        
        # Ajuste de sistema de coordenadas
        # Rotar -90° en Z para alinear X_abb con Y_DH
        r_ajuste = Rotation.from_euler('z', -90, degrees=True)
        
        r_three = r_ajuste * r_abb
        
        quaternion_three = r_three.as_quat()
        return position_three, quaternion_three
    
    return position_three


def transform_threejs_to_robotstudio(position, quaternion=None):
    """
    Transforma coordenadas de Three.js a RobotStudio (ABB)
    Transformación inversa de transform_robotstudio_to_threejs
    
    Args:
        position: [x, y, z] en coordenadas Three.js (mm)
        quaternion: [qx, qy, qz, qw] en coordenadas Three.js (opcional)
    
    Returns:
        position_abb: [x, y, z] en coordenadas ABB
        quaternion_abb: [qx, qy, qz, qw] en coordenadas ABB (si se proporciona)
    """
    x_three, y_three, z_three = position
    
    # Transformación inversa
    x_abb = z_three   # Frente Three.js → Frente ABB
    y_abb = -x_three  # -Derecha Three.js → Izquierda ABB
    z_abb = y_three   # Arriba Three.js → Arriba ABB
    
    position_abb = [x_abb, y_abb, z_abb]
    
    if quaternion is not None:
        from scipy.spatial.transform import Rotation
        
        # Cuaternión Three.js
        r_three = Rotation.from_quat(quaternion)
        
        # Rotación de ajuste inversa: +90° en X
        r_ajuste_inv = Rotation.from_euler('x', 90, degrees=True)
        
        # Componer rotaciones
        r_abb = r_ajuste_inv * r_three
        
        quaternion_abb = r_abb.as_quat()
        return position_abb, quaternion_abb
    
    return position_abb


def dh_transform(theta, d, a, alpha):
    """
    Matriz de Transformación Homogénea de Denavit-Hartenberg (DH estándar).
    
    Parámetros:
    - theta: Ángulo de rotación alrededor del eje Z (radianes)
    - d: Desplazamiento a lo largo del eje Z
    - a: Longitud del eslabón (desplazamiento a lo largo del eje X)
    - alpha: Ángulo de torsión alrededor del eje X (radianes)
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


def quaternion_to_rotation_matrix(q):
    """
    Convierte un cuaternión [qw, qx, qy, qz] o [qx, qy, qz, qw] a matriz de rotación 3x3.
    
    RAPID usa formato: [q1, q2, q3, q4] donde q1 es la parte real (w).
    """
    if len(q) == 4:
        qw, qx, qy, qz = q
    else:
        raise ValueError("Cuaternión debe tener 4 elementos")
    
    # Normalizar
    norm = np.sqrt(qw**2 + qx**2 + qy**2 + qz**2)
    qw, qx, qy, qz = qw/norm, qx/norm, qy/norm, qz/norm
    
    # Construir matriz de rotación
    R = np.array([
        [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qw*qz), 2*(qx*qz + qw*qy)],
        [2*(qx*qy + qw*qz), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qw*qx)],
        [2*(qx*qz - qw*qy), 2*(qy*qz + qw*qx), 1 - 2*(qx**2 + qy**2)]
    ])
    
    return R


def inverse_kinematics_irb140(position, quaternion):
    """
    Cinemática Inversa Analítica para el ABB IRB 140.
    
    Parámetros:
    - position: [x, y, z] en mm
    - quaternion: [q1, q2, q3, q4] donde q4 es la parte real (formato RAPID)
    
    Retorna:
    - numpy array con [q1, q2, q3, q4, q5, q6] en GRADOS
    - None si la solución está fuera del alcance del robot
    
    Configuración: Codo arriba (Elbow Up)
    
    NOTA: Los ángulos retornados son RELATIVOS (sin offsets de la tabla DH)
    """
    
    # Constantes cinemáticas del IRB 140 (en mm)
    # Construir matriz de transformación objetivo
    d1 = 352.0  # Altura de la base
    a2 = 360.0  # Longitud del brazo superior  
    a3 = 70.0   # Offset del codo
    d4 = 380.0  # Longitud del antebrazo
    d6 = 65.0   # Longitud del efector final
    
    try:
        # 1. Construir matriz de transformación objetivo
        R = quaternion_to_rotation_matrix(quaternion)
        P = np.array(position)
        
        # Construir T_target
        T_target = np.eye(4)
        T_target[:3, :3] = R
        T_target[:3, 3] = P
        
        # 2. Calcular el centro de la muñeca (Pw)
        # Pw = P - d6 * Z_axis_target
        # El vector Z del efector apunta en la dirección de aproximación
        Pw = P - d6 * R[:, 2]
        xc, yc, zc = Pw[0], Pw[1], Pw[2]
        
        # 3. Solución para q1 (rotación de la base)
        q1 = np.arctan2(yc, xc)
        
        # 4. Solución para q2 y q3 en el plano R-Z
        # r: distancia radial en el plano XY
        r = np.sqrt(xc**2 + yc**2)
        # s: altura relativa desde la base
        s = zc - d1
        
        # Distancia al cuadrado desde el origen de la articulación 2 al centro de muñeca
        D2 = r**2 + s**2
        D = np.sqrt(D2)
        
        # Longitud efectiva del eslabón 3 (triángulo entre A3 y D4)
        L3 = np.sqrt(a3**2 + d4**2)
        
        # Verificar si el objetivo está dentro del alcance
        max_reach = a2 + L3
        min_reach = abs(a2 - L3)
        
        if D > max_reach or D < min_reach:
            print(f"⚠️ Objetivo fuera de alcance: D={D:.2f}mm, rango=[{min_reach:.2f}, {max_reach:.2f}]mm")
            return None
        
        # Ley de cosenos para encontrar el ángulo gamma
        cos_gamma = (a2**2 + L3**2 - D2) / (2 * a2 * L3)
        cos_gamma = np.clip(cos_gamma, -1.0, 1.0)  # Asegurar que esté en [-1, 1]
        gamma = np.arccos(cos_gamma)
        
        # Desfase de montaje físico para el eslabón 3
        phi = np.arctan2(d4, a3)
        
        # Solución Codo Arriba (Elbow Up)
        q3 = np.pi - gamma - phi
        
        # Para q2: usar geometría del triángulo
        alpha = np.arctan2(s, r)
        cos_beta = (a2**2 + D2 - L3**2) / (2 * a2 * D)
        cos_beta = np.clip(cos_beta, -1.0, 1.0)
        beta = np.arccos(cos_beta)
        q2 = alpha + beta
        
        # 5. Resolver Orientación (q4, q5, q6)
        # Calcular R0_3 con los ángulos q1, q2, q3 recién calculados
        # IMPORTANTE: Usar la misma convención DH que la cinemática directa
        T0_1 = dh_transform(q1, d1, 0, -np.pi/2)
        T1_2 = dh_transform(q2, 0, a2, 0)
        T2_3 = dh_transform(q3, 0, a3, -np.pi/2)
        R0_3 = (T0_1 @ T1_2 @ T2_3)[:3, :3]
        
        # R3_6 = (R0_3)^T * R_target
        R3_6 = R0_3.T @ R
        
        # Extraer q5 (evitar singularidad de muñeca)
        q5 = np.arccos(np.clip(R3_6[2, 2], -1.0, 1.0))
        
        # Evaluar si estamos en singularidad de muñeca (q5 ≈ 0 o π)
        if np.abs(np.sin(q5)) > 1e-6:
            # Solución normal
            q4 = np.arctan2(R3_6[1, 2], R3_6[0, 2])
            q6 = np.arctan2(R3_6[2, 1], -R3_6[2, 0])
        else:
            # Singularidad de muñeca: q4 y q6 están alineados
            print("⚠️ Singularidad de muñeca detectada")
            q4 = 0.0
            q6 = np.arctan2(-R3_6[0, 1], R3_6[0, 0])
        
        # Convertir de radianes a grados
        q_rad = np.array([q1, q2, q3, q4, q5, q6])
        q_deg = np.degrees(q_rad)
        
        # Mapear los ángulos analíticos (q_geom) a los ángulos físicos (ABB) 
        # que el frontend necesita para dibujar la coordenada cartesiana exacta.
        j2_abb = 90.0 - q_deg[1]
        j3_abb = 90.0 - gamma * 180.0 / np.pi
        
        # Los ángulos finales listos para ser consumidos por el sistema ABB sin filtros:
        return np.array([q_deg[0], j2_abb, j3_abb, q_deg[3], q_deg[4], q_deg[5]])
        
    except Exception as e:
        print(f"❌ Error en cinemática inversa: {e}")
        import traceback
        traceback.print_exc()
        return None


def validate_joint_limits(joints, limits):
    """
    Valida que los ángulos estén dentro de los límites del robot.
    
    Parámetros:
    - joints: array de 6 ángulos en grados
    - limits: lista de 6 pares [min, max] en grados
    
    Retorna:
    - True si todos los ángulos están dentro de los límites
    - False y mensaje de error si alguno está fuera
    """
    for i, (angle, (min_lim, max_lim)) in enumerate(zip(joints, limits)):
        if angle < min_lim or angle > max_lim:
            return False, f"J{i+1}: {angle:.2f}° fuera de límites [{min_lim}°, {max_lim}°]"
    
    return True, "OK"


# Función de conveniencia para usar desde el parser RAPID
def solve_ik_from_robtarget(position, quaternion, robot_limits, from_robotstudio=False):
    """
    Resuelve cinemática inversa desde un robtarget de RAPID.
    
    Parámetros:
    - position: [x, y, z] en mm (coordenadas de RobotStudio si from_robotstudio=True)
    - quaternion: [q1, q2, q3, q4] formato RAPID
    - robot_limits: límites de articulaciones del robot
    - from_robotstudio: Si True, transformar coordenadas antes de IK
    
    Retorna:
    - dict con 'success', 'joints', 'message'
    """
    
    # TRANSFORMAR coordenadas de RobotStudio al sistema del robot rotado en Three.js
    if from_robotstudio:
        position_transformed, quaternion_transformed = transform_robotstudio_to_robot_rotated(
            position, quaternion
        )
        print(f"[IK] Transformación aplicada:")
        print(f"  RobotStudio: {position} → Robot: {position_transformed}")
        if quaternion is not None and quaternion_transformed is not None:
            print(f"  Quat RS: {quaternion[:2]}... → Quat Robot: {quaternion_transformed[:2]}...")
    else:
        position_transformed = position
        quaternion_transformed = quaternion
    
    # Si no hay cuaternión transformado, usar el original
    if quaternion_transformed is None:
        quaternion_transformed = quaternion
    
    # Resolver IK con coordenadas transformadas
    joints = inverse_kinematics_irb140(position_transformed, quaternion_transformed)
    
    if joints is None:
        return {
            "success": False,
            "joints": None,
            "message": "Objetivo fuera del alcance del robot"
        }
    
    # Validar límites
    valid, msg = validate_joint_limits(joints, robot_limits)
    
    if not valid:
        return {
            "success": False,
            "joints": joints.tolist(),
            "message": f"Solución fuera de límites: {msg}"
        }
    
    return {
        "success": True,
        "joints": joints.tolist(),
        "message": "Solución encontrada"
    }
