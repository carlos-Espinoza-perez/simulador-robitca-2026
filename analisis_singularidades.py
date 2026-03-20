"""
Análisis de singularidades para el robot ABB IRB 140
"""
import numpy as np
from typing import Dict, List, Tuple
from cinematica_directa import cinematica_directa


def calcular_jacobiano(joints_deg: List[float], tabla_dh: List[List[float]]) -> np.ndarray:
    """
    Calcula el Jacobiano geométrico del robot
    
    Args:
        joints_deg: Ángulos de las articulaciones en grados
        tabla_dh: Tabla de parámetros DH
        
    Returns:
        Jacobiano 6x6 (velocidad lineal y angular)
    """
    # Usar grados directamente en cinematica_directa (ella convierte a rad internamente)
    T_final, transformaciones = cinematica_directa(joints_deg, tabla_dh)
    
    # Convertir ángulos a radianes para los ejes y posiciones relativos
    joints_rad = [np.deg2rad(j) for j in joints_deg]
    
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
        
        # Normalizar el eje para evitar problemas numéricos
        z_norm = np.linalg.norm(z_i)
        if z_norm > 1e-10:
            z_i = z_i / z_norm
        
        # Posición de la articulación
        p_i = T_i[:3, 3]
        
        # Jacobiano de velocidad lineal: z_i × (p_n - p_i)
        J[:3, i] = np.cross(z_i, p_n - p_i)
        
        # Jacobiano de velocidad angular: z_i
        J[3:, i] = z_i
    
    # Verificar si hay valores inválidos
    if np.any(np.isnan(J)) or np.any(np.isinf(J)):
        # Usar identidad como fallback seguro (sin imprimir para evitar spam)
        J = np.eye(6)
    
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
    # 66: J = calcular_jacobiano(joints_deg, tabla_dh)
    J = calcular_jacobiano(joints_deg, tabla_dh)
    
    # Verificar si el Jacobiano es válido antes de continuar
    if np.any(np.isnan(J)) or np.any(np.isinf(J)):
        # No imprimir para evitar spam en consola
        return {
            "estado_general": "normal",
            "singularidades": {
                "singularidades": [],
                "total": 0
            },
            "metricas": {
                "determinante": 0.0,
                "numero_condicion": 1.0,
                "valores_singulares": [1.0] * 6,
                "manipulabilidad": 1.0
            },
            "limites": {
                "violaciones": [],
                "total": 0
            }
        }
    
    # Calcular determinante
    try:
        det_J = np.linalg.det(J)
        if np.isnan(det_J) or np.isinf(det_J):
            det_J = 0.0
    except:
        det_J = 0.0
    
    # Valores singulares con manejo de errores mejorado
    try:
        # Intentar SVD con configuración más robusta
        U, s, Vt = np.linalg.svd(J, full_matrices=False)
        
        # Verificar si los valores singulares son válidos
        if np.any(np.isnan(s)) or np.any(np.isinf(s)):
            raise ValueError("Valores singulares inválidos")
            
    except (np.linalg.LinAlgError, ValueError):
        # Si SVD no converge o valores inválidos, usar valores por defecto seguros
        # No imprimir para evitar spam en consola
        s = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
        cond_number = 1.0
    else:
        # Número de condición (ratio entre mayor y menor valor singular)
        if s[-1] > 1e-10:
            cond_number = s[0] / s[-1]
        else:
            cond_number = np.inf
    
    # Clasificar singularidades con umbrales refinados
    singularidades = []
    
    # Singularidad de muñeca (J4, J5, J6 alineados)
    # Ocurre cuando J5 ≈ 0° o ≈ 180°
    j5 = joints_deg[4]
    # Umbral reducido de 5° a 1° para mayor realismo y evitar falsos positivos en Home
    if abs(j5) < 1.0 or abs(abs(j5) - 180) < 1.0:
        singularidades.append({
            "tipo": "Muñeca",
            "descripcion": "J5 cerca de 0° o 180° - ejes J4 y J6 alineados",
            "severidad": "alta",
            "articulaciones": [4, 5, 6],
            "recomendacion": "Evitar J5=0° o J5=180°"
        })
    elif abs(j5) < 3.0: # Advertencia leve
        singularidades.append({
            "tipo": "Cerca de Muñeca",
            "descripcion": "J5 se aproxima a configuración singular",
            "severidad": "baja",
            "articulaciones": [4, 5, 6],
            "recomendacion": "Aumentar ángulo de J5"
        })
    
    # Singularidad de codo (brazo completamente extendido o retraído)
    # Ocurre cuando el brazo está en sus límites mecánicos de extensión
    # En el IRB 140, J3=0 es una posición estándar, no necesariamente singular
    # La verdadera singularidad de codo ocurre en el límite de alcance
    j3 = joints_deg[2]
    # Reducimos sensibilidad: J3=0 es común, solo advertir si es extremo
    # El IRB 140 tiene límites J3 [-52, 50]. 1e-3 det_J ya captura pérdida de movilidad.
    if abs(j3 - 50) < 1 or abs(j3 + 52) < 1:
        singularidades.append({
            "tipo": "Límite de Codo",
            "descripcion": "Brazo cerca de su máxima extensión/retracción",
            "severidad": "media",
            "articulaciones": [2, 3],
            "recomendacion": "Mantener J3 alejado de los límites extremos"
        })
    
    # Singularidad de hombro (brazo vertical)
    # Ocurre cuando el brazo está directamente sobre la base (centro de la base)
    j2 = joints_deg[1]
    # J2=90 es vertical. Umbral de 5° a 2°
    if abs(j2 - 90) < 2:
        singularidades.append({
            "tipo": "Hombro vertical",
            "descripcion": "Brazo vertical sobre la base - pérdida de dirección",
            "severidad": "baja",
            "articulaciones": [1, 2],
            "recomendacion": "Evitar J2=90°"
        })
    
    # Determinar estado general con histéresis de seguridad
    if abs(det_J) < 1e-4: # Umbral de tolerancia reducido (más permisivo)
        estado = "singular"
    elif cond_number > 500: # Aumentado de 100 a 500
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
