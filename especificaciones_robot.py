ROBOTS = {
    "ABB_IRB_140": {
        "nombre": "ABB IRB 140",
        "tipo": "Antropomórfico",
        "grados_libertad": 6,
        
        "tabla_dh": [
            [0, 352, 0, 0],
            [0, 0, 360, 0],
            [0, 0, 70, -90],
            [0, 380, 0, 90],
            [0, 0, 0, -90],
            [0, 65, 0, 0]
        ],
        
        "limites_articulares": [
            [-180, 180],
            [-90, 110],
            [-52, 50],
            [-200, 200],
            [-115, 115],
            [-400, 400]
        ],
        
        "archivos_stl": [
            "IRB140_-_M2004C_BASE.STL",
            "IRB140_-_M2004C_LINK1.STL",
            "IRB140_-_M2004C_LINK2.STL",
            "IRB140_-_M2004C_LINK3.STL",
            "IRB140_-_M2004C_LINK4.STL",
            "IRB140_-_M2004C_LINK5.STL",
            "IRB140_-_M2004C_LINK6.STL"
        ],
        
        "ruta_stl": "robot_configs/ABB_IRB_140",
        
        "visualizacion": {
            "escala": 0.001,
            "unidades": "mm",
            "posicion_inicial": [0, 0, 0],
            "rotacion_inicial": [-90, 0, 0],
            "camara": {
                "posicion": [2, 1.5, 2],
                "fov": 50,
                "min_distancia": 0.5,
                "max_distancia": 10
            }
        },
        
        "eslabones": [
            {
                "id": 0,
                "nombre": "Base",
                "archivo_stl": "IRB140_-_M2004C_BASE.STL",
                "posicion_relativa": [0, 0, 0],
                "rotacion_relativa": [0, 0, 0],
                "pivot_local": [0, 0, 0],
                "eje_rotacion": "y",
                "color": "#E8E8E8",
                "descripcion": "Base fija - Rotación vertical (J1)"
            },
            {
                "id": 1,
                "nombre": "Link 1",
                "archivo_stl": "IRB140_-_M2004C_LINK1.STL",
                "posicion_relativa": [0, 0.352, 0],
                "rotacion_relativa": [0, 0, 0],
                "pivot_local": [0, 0, 0],
                "eje_rotacion": "y",
                "color": "#E8E8E8",
                "descripcion": "Hombro - Rotación vertical (J2), altura 352mm"
            },
            {
                "id": 2,
                "nombre": "Link 2",
                "archivo_stl": "IRB140_-_M2004C_LINK2.STL",
                "posicion_relativa": [0, 0, 0.280],
                "rotacion_relativa": [0, 0, 0],
                "pivot_local": [0, 0, 0],
                "eje_rotacion": "y",
                "color": "#E8E8E8",
                "descripcion": "Brazo superior - Rotación vertical (J3), longitud 360mm"
            },
            {
                "id": 3,
                "nombre": "Link 3",
                "archivo_stl": "IRB140_-_M2004C_LINK3.STL",
                "posicion_relativa": [0.070, 0, 0],
                "rotacion_relativa": [0, 0, -90],
                "pivot_local": [0, 0, 0],
                "eje_rotacion": "y",
                "color": "#E8E8E8",
                "descripcion": "Codo - Rotación vertical (J4), offset 70mm, giro -90°"
            },
            {
                "id": 4,
                "nombre": "Link 4",
                "archivo_stl": "IRB140_-_M2004C_LINK4.STL",
                "posicion_relativa": [0, 0.380, 0],
                "rotacion_relativa": [0, 0, 90],
                "pivot_local": [0, 0, 0],
                "eje_rotacion": "x",
                "color": "#E8E8E8",
                "descripcion": "Antebrazo - Rotación longitudinal (J5), longitud 380mm, giro 90°"
            },
            {
                "id": 5,
                "nombre": "Link 5",
                "archivo_stl": "IRB140_-_M2004C_LINK5.STL",
                "posicion_relativa": [0, 0, 0],
                "rotacion_relativa": [0, 0, -90],
                "pivot_local": [0, 0, 0],
                "eje_rotacion": "y",
                "color": "#C0C0C0",
                "descripcion": "Muñeca - Rotación perpendicular (J6), giro -90°"
            },
            {
                "id": 6,
                "nombre": "Link 6",
                "archivo_stl": "IRB140_-_M2004C_LINK6.STL",
                "posicion_relativa": [0, 0.065, 0],
                "rotacion_relativa": [0, 0, 0],
                "pivot_local": [0, 0, 0],
                "eje_rotacion": "x",
                "color": "#C0C0C0",
                "descripcion": "Efector final - Rotación longitudinal (J7), longitud 65mm"
            }
        ],
        
        "workspace": {
            "alcance_maximo": 1.5,
            "alcance_minimo": 0.2,
            "altura_maxima": 2.0,
            "altura_minima": -0.5,
            "grid": {
                "tamaño": 10,
                "division": 0.1
            }
        },
        
        "materiales": {
            "metalness": 0.7,
            "roughness": 0.3,
            "colores": {
                "base": "#E8E8E8",
                "eslabones": "#E8E8E8",
                "end_effector": "#C0C0C0"
            }
        },
        
        "metadatos": {
            "fabricante": "ABB",
            "modelo": "IRB 140",
            "año": 2020,
            "carga_maxima": 6,
            "repetibilidad": 0.03,
            "velocidad_maxima": [250, 250, 250, 320, 320, 420]
        }
    },
    
    "ABB_IRB_910SC": {
        "nombre": "ABB IRB 910SC (SCARA)",
        "tipo": "SCARA",
        "grados_libertad": 4,
        
        "tabla_dh": [
            [0, 192, 300, 0],
            [0, 0, 250, 180],
            [0, 0, 0, 0],
            [0, 0, 0, 0]
        ],
        
        "limites_articulares": [
            [-140, 140],
            [-150, 150],
            [0, 200],
            [-360, 360]
        ],
        
        "archivos_stl": [],
        "ruta_stl": "robot_configs/ABB_IRB_910SC",
        
        "visualizacion": {
            "escala": 0.001,
            "unidades": "mm",
            "posicion_inicial": [0, 0, 0],
            "rotacion_inicial": [0, 0, 0],
            "camara": {
                "posicion": [1.5, 1.5, 1.5],
                "fov": 50,
                "min_distancia": 0.3,
                "max_distancia": 5
            }
        },
        
        "eslabones": [],
        
        "workspace": {
            "alcance_maximo": 0.8,
            "alcance_minimo": 0.1,
            "altura_maxima": 0.5,
            "altura_minima": -0.2,
            "grid": {
                "tamaño": 5,
                "division": 0.1
            }
        },
        
        "materiales": {
            "metalness": 0.6,
            "roughness": 0.4,
            "colores": {
                "base": "#455a64",
                "eslabones": "#64b5f6",
                "end_effector": "#4caf50"
            }
        },
        
        "metadatos": {
            "fabricante": "ABB",
            "modelo": "IRB 910SC",
            "año": 2021,
            "carga_maxima": 3,
            "repetibilidad": 0.01,
            "velocidad_maxima": [300, 300, 1000, 400]
        }
    }
}
