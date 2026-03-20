"""
Cinemática Inversa para el simulador
Adaptado del digital twin del IRB 140
"""

import numpy as np


def transform_robotstudio_to_robot_rotated(position, quaternion=None):
    # Enviar coordenadas directamente
    position_robot = list(position)
    
    if quaternion is not None and len(quaternion) == 4:
        quaternion_robot = list(quaternion)
        return position_robot, quaternion_robot
    
    return position_robot, None


def transform_robotstudio_to_threejs(position, quaternion=None):
    from scipy.spatial.transform import Rotation
    
    x_abb, y_abb, z_abb = position
    
    # Transformación correcta basada en el sistema DH
    x_dh = -y_abb  # Izquierda ABB → -Lateral DH
    y_dh = x_abb   # Frente ABB → Frente DH (brazo extendido)
    z_dh = z_abb   # Arriba ABB → Arriba DH
    
    position_three = [x_dh, y_dh, z_dh]
    
    if quaternion is not None:
        r_abb = Rotation.from_quat(quaternion)
        r_ajuste = Rotation.from_euler('z', -90, degrees=True)
        r_three = r_ajuste * r_abb
        quaternion_three = r_three.as_quat()
        return position_three, quaternion_three
    
    return position_three


def transform_threejs_to_robotstudio(position, quaternion=None):
    x_three, y_three, z_three = position
    
    x_abb = z_three   # Frente Three.js → Frente ABB
    y_abb = -x_three  # -Derecha Three.js → Izquierda ABB
    z_abb = y_three   # Arriba Three.js → Arriba ABB
    
    position_abb = [x_abb, y_abb, z_abb]
    
    if quaternion is not None:
        from scipy.spatial.transform import Rotation
        r_three = Rotation.from_quat(quaternion)
        r_ajuste_inv = Rotation.from_euler('x', 90, degrees=True)
        r_abb = r_ajuste_inv * r_three
        quaternion_abb = r_abb.as_quat()
        return position_abb, quaternion_abb
    
    return position_abb


def dh_transform(theta, d, a, alpha):
    """
    Matriz de Transformación Homogénea de Denavit-Hartenberg (DH estándar).
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


def apply_axis_angle(vec, axis, angle):

    import math

    v = np.array(vec)

    k = np.array(axis)

    k = k / np.linalg.norm(k)

    return v * math.cos(angle) + np.cross(k, v) * math.sin(angle) + k * np.dot(k, v) * (1 - math.cos(angle))

def get_visual_R0_3(j1_deg, j2_abb, j3_abb):

    import math

    # Posición inicial de ejes

    axes = [

        np.array([0., 0., 1.]),

        np.array([0., 1., 0.]),

        np.array([0., 1., 0.])

    ]

    wrist_axes = [

        np.array([1., 0., 0.]), # X

        np.array([0., 1., 0.]), # Y

        np.array([1., 0., 0.])  # X

    ]

   

    q = [math.radians(j1_deg), math.radians(j2_abb), math.radians(j3_abb)]

    for j in range(3):

        ang = q[j]

        axis = axes[j]

        for i in range(j+1, 3):

            axes[i] = apply_axis_angle(axes[i], axis, ang)

        for w in range(3):

            wrist_axes[w] = apply_axis_angle(wrist_axes[w], axis, ang)

           

    X_local = wrist_axes[0]

    Y_local = wrist_axes[1]

    Z_local = np.cross(X_local, Y_local)

    return np.column_stack((X_local, Y_local, Z_local))

def inverse_kinematics_irb140(position, quaternion):
    """
    Cinemática Inversa Analítica para el ABB IRB 140.
    Mantiene la orientación visual original para Three.js, pero corrige
    las medidas geométricas para eliminar el desfase de 5.4 mm.
    """
    # 1. LAS MEDIDAS REALES (Aquí estaba el origen del desfase)
    d1 = 352.0  
    a1 = 70.0   # CORRECCIÓN 1: Agregamos el offset del hombro
    a2 = 360.0  # CORRECCIÓN 2: Brazo real (antes 280.0)
    a3 = 0.0    # CORRECCIÓN 3: Codo directo (antes 70.0)
    d4 = 380.0  
    d6 = 65.0   
    
    try:
        R_orig = quaternion_to_rotation_matrix(quaternion)
        R_align_T = np.array([
            [0.,  0., -1.],
            [0.,  1.,  0.],
            [1.,  0.,  0.]
        ])
        R = R_orig @ R_align_T
        P = np.array(position)
        
        # Calcular el centro de la muñeca (Pw)
        Pw = P - d6 * R[:, 0]
        xc, yc, zc = Pw[0], Pw[1], Pw[2]
        
        # Solución para q1 
        q1 = np.arctan2(yc, xc)
        
        # CORRECCIÓN 4: Restar el offset del hombro (a1) a la distancia radial
        r = np.sqrt(xc**2 + yc**2) - a1 
        s = zc - d1
        
        D2 = r**2 + s**2
        D = np.sqrt(D2)
        
        # L3 ahora es directamente d4 porque a3 es 0
        L3 = d4 
        
        max_reach = a2 + L3
        min_reach = abs(a2 - L3)
        
        if D > max_reach or D < min_reach:
            print(f"⚠️ Objetivo fuera de alcance: D={D:.2f}mm, rango=[{min_reach:.2f}, {max_reach:.2f}]mm")
            return None
        
        cos_gamma = (a2**2 + L3**2 - D2) / (2 * a2 * L3)
        cos_gamma = np.clip(cos_gamma, -1.0, 1.0) 
        gamma = np.arccos(cos_gamma)
        
        alpha = np.arctan2(s, r)
        cos_beta = (a2**2 + D2 - L3**2) / (2 * a2 * D)
        cos_beta = np.clip(cos_beta, -1.0, 1.0)
        beta = np.arccos(cos_beta)
        q2 = alpha + beta
        
        # Convertir a grados ABB para buscar el R0_3 correcto
        q1_deg = np.degrees(q1)
        q2_geom_deg = np.degrees(q2)
        gamma_deg = np.degrees(gamma)
        
        j1_abb = q1_deg
        j2_abb = 90.0 - q2_geom_deg
        j3_abb = 90.0 - gamma_deg
        
        # USAMOS TU FUNCIÓN VISUAL ORIGINAL (Esto arregla que el robot se vuelva loco)
        R0_3 = get_visual_R0_3(j1_abb, j2_abb, j3_abb)
        R3_6 = R0_3.T @ R
        
        # Extraer muñeca esférica
        q5_rad = np.arccos(np.clip(R3_6[0, 0], -1.0, 1.0))
        
        if np.abs(np.sin(q5_rad)) > 1e-6:
            q4_rad = np.arctan2(R3_6[1, 0], -R3_6[2, 0])
            q6_rad = np.arctan2(R3_6[0, 1], R3_6[0, 2])
        else:
            q4_rad = 0.0
            q6_rad = np.arctan2(R3_6[2, 1], R3_6[1, 1])
            
        j4_abb = np.degrees(q4_rad)
        j5_abb = np.degrees(q5_rad)
        j6_abb = np.degrees(q6_rad)
        
        return np.array([j1_abb, j2_abb, j3_abb, j4_abb, j5_abb, j6_abb])
        
    except Exception as e:
        print(f"❌ Error en cinemática inversa: {e}")
        import traceback
        traceback.print_exc()
        return None

def validate_joint_limits(joints, limits):
    min_len = min(len(joints), len(limits))
    for i in range(min_len):
        angle = joints[i]
        min_lim, max_lim = limits[i]
        if angle < min_lim or angle > max_lim:
            return False, f"J{i+1}: {angle:.2f} fuera de límites [{min_lim}, {max_lim}]"
    
    return True, "OK"


# ============================================================
# IK Analítico para SCARA (ABB IRB 910SC) - 4 GDL
# ============================================================

def inverse_kinematics_scara(position, quaternion):
    """
    Cinemática Inversa Analítica para el ABB IRB 910SC (SCARA).
    
    Parámetros DH del SCARA:
        J1 (R): d1=200mm, a1=250mm, α=0°
        J2 (R): d2=0,     a2=200mm, α=180°
        J3 (P): d3=q3,    a3=0,     α=0°
        J4 (R): d4=0,     a4=0,     α=0°
    
    FK resultado:
        x = a1*cos(q1) + a2*cos(q1+q2)
        y = a1*sin(q1) + a2*sin(q1+q2)
        z = d1 - q3  (α2=180° invierte eje Z para el prismatico)
    
    Args:
        position: [x, y, z] en mm (coordenadas del TCP)
        quaternion: [qw, qx, qy, qz] (orientación, se extrae rotación Z)
    
    Returns:
        np.array([q1_deg, q2_deg, q3_mm, q4_deg]) o None si no hay solución
    """
    # Parámetros geométricos del SCARA
    a1 = 250.0   # Longitud brazo 1 (mm)
    a2 = 200.0   # Longitud brazo 2 (mm)
    d1 = 200.0   # Altura base (mm)
    d3 = 100.0   # Offset base de la columna Z en la tabla DH (mm)
    
    x, y, z = position[0], position[1], position[2]
    
    try:
        # FK es: z = d1 - (d3 + q3)  (porque alpha2 es 180 grados, el eje Z se invierte)
        # Despejando: q3 = d1 - d3 - z
        q3 = d1 - d3 - z
        
        # q3 es el desplazamiento prismático que debe estar teóricamente entre 0 y 150mm
        if q3 < -5.0 or q3 > 155.0:  # Margen de tolerancia  
            max_z = d1 - d3  # El z más alto que puede alcanzar (cuando q3 = 0 -> 200-100 = 100mm)
            min_z = d1 - d3 - 150.0 # El z más bajo (cuando q3 = max alcance 150 -> 200-100-150 = -50mm)
            print(f"⚠️ SCARA IK: Altura Z={z:.2f}mm inalcanzable. Rango válido Z es [{min_z}, {max_z}]mm")
            return None
        
        # ── J2 (Codo): Ley de cosenos ──
        r_sq = x**2 + y**2
        r = np.sqrt(r_sq)
        
        cos_q2 = (r_sq - a1**2 - a2**2) / (2 * a1 * a2)
        
        if abs(cos_q2) > 1.0 + 1e-6:
            print(f"⚠️ SCARA IK: Alcance {r:.2f}mm fuera de rango [{abs(a1-a2):.0f}, {a1+a2:.0f}]mm")
            return None
        
        cos_q2 = np.clip(cos_q2, -1.0, 1.0)
        
        # Elegir configuración "codo arriba" (elbow-up) por defecto
        sin_q2 = -np.sqrt(1 - cos_q2**2)  # Negativo = codo derecho (más natural en SCARA)
        q2 = np.arctan2(sin_q2, cos_q2)
        
        # ── J1 (Hombro): Geometría planar ──
        k1 = a1 + a2 * cos_q2
        k2 = a2 * sin_q2
        q1 = np.arctan2(y, x) - np.arctan2(k2, k1)
        
        # ── J4 (Muñeca): Rotación Z del TCP ──
        # Extraer la rotación total en Z del cuaternión
        # RAPID quaternion: [qw, qx, qy, qz]
        if quaternion is not None and len(quaternion) >= 4:
            qw, qx, qy, qz = quaternion[0], quaternion[1], quaternion[2], quaternion[3]
            # Para SCARA la orientación es puramente en Z
            # φ_total = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy² + qz²))
            phi_total = np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy**2 + qz**2))
        else:
            phi_total = 0.0
        
        # La rotación Z total del TCP = q1 + q2 + q4
        q4 = phi_total - q1 - q2
        
        # Convertir a grados (J3 se queda en mm)
        q1_deg = np.degrees(q1)
        q2_deg = np.degrees(q2)
        q3_mm = q3  # Prismático: ya está en mm
        q4_deg = np.degrees(q4)
        
        return np.array([q1_deg, q2_deg, q3_mm, q4_deg])
        
    except Exception as e:
        print(f"❌ Error en IK SCARA: {e}")
        import traceback
        traceback.print_exc()
        return None


# ============================================================
# Router principal de IK
# ============================================================

def solve_ik_from_robtarget(position, quaternion, robot_limits, from_robotstudio=False, robot_id="ABB_IRB_140"):
    """
    Resuelve IK para cualquier robot soportado.
    Enruta al solver analítico correcto según robot_id.
    """
    
    # ── Transformación de coordenadas ──
    if from_robotstudio:
        position_transformed, quaternion_transformed = transform_robotstudio_to_robot_rotated(
            position, quaternion
        )
    else:
        position_transformed = position
        quaternion_transformed = quaternion
    
    if quaternion_transformed is None:
        quaternion_transformed = quaternion
    
    # ── Enrutar al solver correcto ──
    if robot_id == "ABB_IRB_910SC":
        return _solve_ik_scara(position_transformed, quaternion_transformed, robot_limits)
    else:
        return _solve_ik_irb140(position_transformed, quaternion_transformed, robot_limits)


def _solve_ik_scara(position, quaternion, robot_limits):
    """Wrapper IK para el ABB IRB 910SC (SCARA)."""
    joints = inverse_kinematics_scara(position, quaternion)
    
    if joints is None:
        x, y, z = position
        a1, a2 = 250.0, 200.0
        r = np.sqrt(x**2 + y**2)
        
        return {
            "success": False,
            "joints": None,
            "message": "Objetivo fuera del alcance del SCARA",
            "error_detail": f"⚠️ SCARA: Alcance={r:.1f}mm (máx={a1+a2}mm), Z={z:.1f}mm (rango=[-50, 100]mm)"
        }
    
    valid, msg = validate_joint_limits(joints, robot_limits)
    
    if not valid:
        return {
            "success": False,
            "joints": joints.tolist(),
            "message": f"Solución SCARA fuera de límites: {msg}",
            "error_detail": f"⚠️ Límites excedidos: {msg}"
        }
    
    return {
        "success": True,
        "joints": joints.tolist(),
        "message": "Solución SCARA encontrada",
        "error_detail": None
    }


def _solve_ik_irb140(position, quaternion, robot_limits):
    """Wrapper IK para el ABB IRB 140."""
    joints = inverse_kinematics_irb140(position, quaternion)
    
    if joints is None:
        xc, yc, zc = position
        
        d1 = 352.0
        a1 = 70.0
        a2 = 360.0
        d4 = 380.0
        
        r = np.sqrt(xc**2 + yc**2) - a1
        s = zc - d1
        D = np.sqrt(r**2 + s**2)
        
        max_reach = a2 + d4
        min_reach = abs(a2 - d4)
        
        error_detail = f"⚠️ Objetivo fuera de alcance: D={D:.2f}mm, rango=[{min_reach:.2f}, {max_reach:.2f}]mm"
        
        return {
            "success": False,
            "joints": None,
            "message": "Objetivo fuera del alcance del robot",
            "error_detail": error_detail
        }
    
    valid, msg = validate_joint_limits(joints, robot_limits)
    
    if not valid:
        return {
            "success": False,
            "joints": joints.tolist(),
            "message": f"Solución fuera de límites: {msg}",
            "error_detail": f"⚠️ Límites excedidos: {msg}"
        }
    
    return {
        "success": True,
        "joints": joints.tolist(),
        "message": "Solución encontrada",
        "error_detail": None
    }