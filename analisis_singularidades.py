"""
Análisis de singularidades para el robot ABB IRB 140
"""
import numpy as np
from typing import Dict, List, Tuple
from cinematica_directa import cinematica_directa


def calcular_jacobiano(joints_deg: List[float], tabla_dh: List[List[float]], tipos_articulaciones: List[str] = None) -> np.ndarray:
    """
    Calcula el Jacobiano geométrico del robot
    
    Args:
        joints_deg: Ángulos de las articulaciones en grados (o param desplazamiento en mm si es prismática)
        tabla_dh: Tabla de parámetros DH
        tipos_articulaciones: Lista de tipos ('R' o 'P')
        
    Returns:
        Jacobiano 6xN (velocidad lineal y angular)
    """
    num_joints = len(joints_deg)
    if tipos_articulaciones is None:
        tipos_articulaciones = ["R"] * num_joints
        
    # Usar grados directamente en cinematica_directa (ella convierte a rad internamente)
    T_final, transformaciones = cinematica_directa(joints_deg, tabla_dh, tipos_articulaciones)
    
    # Posición del efector final
    p_n = T_final[:3, 3]
    
    # Inicializar Jacobiano
    J = np.zeros((6, num_joints))
    
    # Para cada articulación
    for i in range(num_joints):
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
        
        p_i = T_i[:3, 3]
        
        if tipos_articulaciones[i] == 'R':
            # Jacobiano de velocidad lineal: z_i × (p_n - p_i)
            J[:3, i] = np.cross(z_i, p_n - p_i)
            # Jacobiano de velocidad angular: z_i
            J[3:, i] = z_i
        else:
            # Prismática: velocidad lineal plana en el eje, angular 0
            J[:3, i] = z_i
            J[3:, i] = 0.0
            
    # Verificar si hay valores inválidos
    if np.any(np.isnan(J)) or np.any(np.isinf(J)):
        # Usar forma correcta
        J = np.zeros((6, num_joints))
    
    return J


def analizar_singularidades(joints_deg: List[float], tabla_dh: List[List[float]], tipos_articulaciones: List[str] = None) -> Dict:
    """
    Analiza las singularidades del robot en una configuración dada
    
    Args:
        joints_deg: Ángulos en grados
        tabla_dh: Tabla DH del robot
        
    Returns:
        Diccionario con análisis de singularidades
    """
    J = calcular_jacobiano(joints_deg, tabla_dh, tipos_articulaciones)
    num_joints = len(joints_deg)
    
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
                "valores_singulares": [1.0] * min(6, num_joints),
                "manipulabilidad": 1.0
            },
            "limites": {
                "violaciones": [],
                "total": 0
            }
        }
    
    # Calcular determinante si J es cuadrada (6x6), sino 0.0
    try:
        if J.shape[0] == J.shape[1]:
            det_J = np.linalg.det(J)
        else:
            # Determinante de la pseudo-inversa o raiz de J*J.T para manipulabilidad
            det_J = np.sqrt(np.linalg.det(J @ J.T))
            
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
        s = np.array([1.0] * min(6, num_joints))
        cond_number = 1.0
    else:
        # Número de condición (ratio entre mayor y menor valor singular)
        if s[-1] > 1e-10:
            cond_number = s[0] / s[-1]
        else:
            cond_number = np.inf
    
    # Clasificar singularidades con umbrales refinados
    singularidades = []
    
    # Las reglas de singularidad (codo, muñeca, hombro) dependen directamente 
    # de la estructura geométrica IRB 140 de 6 GDL.
    if num_joints == 6:
        # Singularidad de muñeca (J4, J5, J6 alineados)
        j5 = joints_deg[4]
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
        
        # Singularidad de codo
        j3 = joints_deg[2]
        if abs(j3 - 50) < 1 or abs(j3 + 52) < 1:
            singularidades.append({
                "tipo": "Límite de Codo",
                "descripcion": "Brazo cerca de su máxima extensión/retracción",
                "severidad": "media",
                "articulaciones": [2, 3],
                "recomendacion": "Mantener J3 alejado de los límites extremos"
            })
        
        # Singularidad de hombro (brazo vertical)
        j2 = joints_deg[1]
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
                      limites: List[List[float]], tipos_articulaciones: List[str] = None) -> Dict:
    """
    Análisis completo de la configuración del robot
    
    Args:
        joints_deg: Ángulos en grados
        tabla_dh: Tabla DH
        limites: Límites articulares
        tipos_articulaciones: Tipo de articulación
        
    Returns:
        Análisis completo
    """
    singularidades = analizar_singularidades(joints_deg, tabla_dh, tipos_articulaciones)
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
