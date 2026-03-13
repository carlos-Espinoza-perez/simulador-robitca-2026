import numpy as np

def get_dh_matriz(theta, d, a, alpha):
    # Convertimos los ángulos de grados a radianes para los cálculos
    theta_rad = np.radians(theta)
    alpha_rad = np.radians(alpha)
    
    # La matriz Ai basada en la fórmula estándar de DH
    matriz = np.array([
        [np.cos(theta_rad), -np.sin(theta_rad) * np.cos(alpha_rad),  np.sin(theta_rad) * np.sin(alpha_rad), a * np.cos(theta_rad)],
        [np.sin(theta_rad),  np.cos(theta_rad) * np.cos(alpha_rad), -np.cos(theta_rad) * np.sin(alpha_rad), a * np.sin(theta_rad)],
        [0,                 np.sin(alpha_rad),                  np.cos(alpha_rad),                 d],
        [0,                 0,                                  0,                                 1]
    ])
    
    return matriz


def get_robot_transforms(joint_angles, dh_table):
    transforms = []
    # Matriz Identidad (el origen del mundo [0,0,0])
    T_accumulated = np.identity(4)
    
    for i in range(len(dh_table)):
        # Obtenemos los parametros base del archivo de configuración
        theta_fixed, d, a, alpha = dh_table[i]
        
        # El angulo real es la suma del ángulo fijo de DH + el angulo del motor (q)
        q_current = theta_fixed + joint_angles[i]
        
        # Calculamos la matriz local de este eslabon
        A_i = get_dh_matriz(q_current, d, a, alpha)
        
        # La multiplicamos por la anterior para obtener la posición global
        T_accumulated = np.dot(T_accumulated, A_i)
        
        # Guardamos la matriz resultante de este eslabon
        transforms.append(T_accumulated)
        
    return transforms