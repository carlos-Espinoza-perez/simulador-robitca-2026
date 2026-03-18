"""
Análisis de singularidades para el robot ABB IRB 140
"""
import numpy as np
from typing import Dict, List, Tuple
from cinematica_directa import cinematica_directa


def calcular_jacobiano(joints_rad: List[float], tabla_dh: List[List[float]]) -> np.ndarray:
    """
    Calcula el Jacobiano geométrico del robot
    
    Args:
        joints_rad: Ángulos de las articulaciones en radianes
        tabla_dh: Tabla de parámetros DH
        
    Returns:
        Jacobiano 6x6 (velocidad lineal y angular)
    """
    # Calcular transformaciones
    T_final, transformaciones = cinematica_directa(joints_rad, tabla_dh)
    
    # Posición del efector final
    p_n = T_final[:3, 3]
    
    # Inicializar Jacobiano
    J = np.zeros((6, 6))
    
    # Para cada articulación
    for i in range(6):
        # Transformación hasta la articulación i
        if i == 0:
            T_i = np.eye(4)
        else:
            T_i = transformaciones[i-1]
        
        # Eje de rotación (columna Z de la transformación)
        z_i = T_i[:3, 2]
        
        # Posición de la articulación
        p_i = T_i[:3, 3]
        
        # Jacobiano de velocidad lineal: z_i × (p_n - p_i)
        J[:3, i] = np.cross(z_i, p_n - p_i)
        
        # Jacobiano de velocidad angular: z_i
        J[3:, i] = z_i
    
    return J


def analizar_singularidades(joints_deg: List[float], tabla_dh: List[List[float]]) -> Dict:
    """
    Analiza las singularidades del robot en una configuración dada
    
    Args:
        joints_deg: Ángulos en grados
        tabla_dh: Tabla DH del robot
        
    Returns:
        Diccionario con análisis de singularidades
    """
    joints_rad = [np.deg2rad(j) for j in joints_deg]
    
    # Calcular Jacobiano
    J = calcular_jacobiano(joints_rad, tabla_dh)
    
    # Calcular determinante y número de condición
    det_J = np.linalg.det(J)
    
    # Valores singulares
    U, s, Vt = np.linalg.svd(J)
    
    # Número de condición (ratio entre mayor y menor valor singular)
    cond_number = s[0] / s[-1] if s[-1] > 1e-10 else np.inf
    
    # Clasificar singularidades
    singularidades = []
    
    # Singularidad de muñeca (J4, J5, J6 alineados)
    # Ocurre cuando J5 ≈ 0° o ≈ 180°
    j5 = joints_deg[4]
    if abs(j5) < 5 or abs(abs(j5) - 180) < 5:
        singularidades.append({
            "tipo": "Muñeca",
            "descripcion": "J5 cerca de 0° o 180° - ejes J4 y J6 alineados",
            "severidad": "alta",
            "articulaciones": [4, 5, 6],
            "recomendacion": "Evitar J5=0° o J5=180°"
        })
    
    # Singularidad de codo (brazo completamente extendido o retraído)
    # Ocurre cuando J3 ≈ 0° (extendido) o J3 ≈ -90° (retraído)
    j3 = joints_deg[2]
    if abs(j3) < 5:
        singularidades.append({
            "tipo": "Codo extendido",
            "descripcion": "Brazo completamente extendido - pérdida de movilidad",
            "severidad": "media",
            "articulaciones": [2, 3],
            "recomendacion": "Mantener J3 alejado de 0°"
        })
    elif abs(j3 + 90) < 5:
        singularidades.append({
            "tipo": "Codo retraído",
            "descripcion": "Brazo completamente retraído - configuración singular",
            "severidad": "media",
            "articulaciones": [2, 3],
            "recomendacion": "Mantener J3 alejado de -90°"
        })
    
    # Singularidad de hombro (brazo vertical)
    # Ocurre cuando el brazo está directamente sobre la base
    j2 = joints_deg[1]
    if abs(j2 - 90) < 5:
        singularidades.append({
            "tipo": "Hombro vertical",
            "descripcion": "Brazo vertical sobre la base - pérdida de dirección",
            "severidad": "baja",
            "articulaciones": [1, 2],
            "recomendacion": "Evitar J2=90°"
        })
    
    # Determinar estado general
    if abs(det_J) < 1e-3:
        estado = "singular"
    elif cond_number > 100:
        estado = "cerca_singular"
    else:
        estado = "normal"
    
    return {
        "estado": estado,
        "determinante": float(det_J),
        "numero_condicion": float(cond_number),
        "valores_singulares": s.tolist(),
        "singularidades": singularidades,
        "manipulabilidad": float(abs(det_J)),  # Índice de Yoshikawa
        "jacobiano": J.tolist()
    }


def verificar_limites_articulares(joints_deg: List[float], limites: List[List[float]]) -> Dict:
    """
    Verifica si las articulaciones están cerca de sus límites
    
    Args:
        joints_deg: Ángulos en grados
        limites: Límites [min, max] para cada articulación
        
    Returns:
        Diccionario con advertencias de límites
    """
    advertencias = []
    margen_seguridad = 10  # grados
    
    for i, (angulo, (min_lim, max_lim)) in enumerate(zip(joints_deg, limites)):
        # Verificar si está fuera de límites
        if angulo < min_lim or angulo > max_lim:
            advertencias.append({
                "articulacion": i + 1,
                "tipo": "fuera_limites",
                "angulo": angulo,
                "limite_min": min_lim,
                "limite_max": max_lim,
                "severidad": "critica"
            })
        # Verificar si está cerca del límite
        elif angulo < min_lim + margen_seguridad:
            advertencias.append({
                "articulacion": i + 1,
                "tipo": "cerca_limite_inferior",
                "angulo": angulo,
                "limite_min": min_lim,
                "margen": angulo - min_lim,
                "severidad": "advertencia"
            })
        elif angulo > max_lim - margen_seguridad:
            advertencias.append({
                "articulacion": i + 1,
                "tipo": "cerca_limite_superior",
                "angulo": angulo,
                "limite_max": max_lim,
                "margen": max_lim - angulo,
                "severidad": "advertencia"
            })
    
    return {
        "en_limites": len([a for a in advertencias if a["tipo"] == "fuera_limites"]) == 0,
        "advertencias": advertencias
    }


def analisis_completo(joints_deg: List[float], tabla_dh: List[List[float]], 
                      limites: List[List[float]]) -> Dict:
    """
    Análisis completo de la configuración del robot
    
    Args:
        joints_deg: Ángulos en grados
        tabla_dh: Tabla DH
        limites: Límites articulares
        
    Returns:
        Análisis completo
    """
    singularidades = analizar_singularidades(joints_deg, tabla_dh)
    limites_check = verificar_limites_articulares(joints_deg, limites)
    
    # Determinar estado general
    if not limites_check["en_limites"]:
        estado_general = "error"
    elif singularidades["estado"] == "singular":
        estado_general = "singular"
    elif singularidades["estado"] == "cerca_singular":
        estado_general = "advertencia"
    else:
        estado_general = "normal"
    
    return {
        "estado_general": estado_general,
        "singularidades": singularidades,
        "limites": limites_check,
        "joints": joints_deg
    }
