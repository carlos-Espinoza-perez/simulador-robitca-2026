ROBOTS = {
    "ABB_IRB_140": {
        "nombre": "ABB IRB 140",
        "tipo": "Antropomórfico",
        "grados_libertad": 6,
        "tabla_dh": [
            # [theta, d, a, alpha] -> Valores en mm y grados
            [0, 352, 70, -90],  # Eje 1
            [-90, 0, 360, 0],   # Eje 2
            [0, 0, 0, -90],     # Eje 3
            [0, 380, 0, 90],    # Eje 4
            [0, 0, 0, -90],     # Eje 5
            [0, 65, 0, 0]       # Eje 6
        ],
        "limites_articulares": [
            [-180, 180], # Eje 1
            [-90, 110],  # Eje 2
            [-230, 50],  # Eje 3
            [-200, 200], # Eje 4
            [-115, 115], # Eje 5
            [-400, 400]  # Eje 6
        ],
        "archivos_stl": [
            "IRB140_-_M2004C_BASE.STL", 
            "IRB140_-_M2004C_LINK1.STL", 
            "IRB140_-_M2004C_LINK2.STL", 
            "IRB140_-_M2004C_LINK3.STL", 
            "IRB140_-_M2004C_LINK4.STL", 
            "IRB140_-_M2004C_LINK5.STL", 
            "IRB140_-_M2004C_LINK6.STL"
        ]
    },
    "ABB_IRB_910SC": {
        "nombre": "ABB IRB 910SC (SCARA)",
        "tipo": "SCARA",
        "grados_libertad": 4,
        "parametros_dh": [
            # Aquí la d o la a cambian según sea Revoluta o Prismática
            [0, 192, 300, 0],   # Eje 1 (R)
            [0, 0, 250, 180],   # Eje 2 (R)
            [0, "variable", 0, 0], # Eje 3 (Prismático - d es la variable)
            [0, 0, 0, 0]        # Eje 4 (R)
        ],
        "stls": ["scara_base.stl", "arm1.stl", "arm2.stl", "tool.stl"]
    }
}