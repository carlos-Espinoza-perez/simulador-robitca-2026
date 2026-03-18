import numpy as np

def matriz_dh(theta, d, a, alpha):
    theta_rad = np.radians(theta)
    alpha_rad = np.radians(alpha)
    ct = np.cos(theta_rad)
    st = np.sin(theta_rad)
    ca = np.cos(alpha_rad)
    sa = np.sin(alpha_rad)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0,   sa,       ca,      d],
        [0,   0,        0,       1]
    ])

A1 = matriz_dh(0, 352, 0, -90)
A2 = matriz_dh(-90, 0, 360, 0)
print("A1:\n", np.round(A1, 3))
print("A2:\n", np.round(A2, 3))
print("A1 @ A2:\n", np.round(A1 @ A2, 3))
