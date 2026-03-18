import math
import numpy as np

def apply_axis_angle(vec, axis, angle):
    v = np.array(vec)
    k = np.array(axis)
    k = k / np.linalg.norm(k)
    return v * math.cos(angle) + np.cross(k, v) * math.sin(angle) + k * np.dot(k, v) * (1 - math.cos(angle))

def visual_R0_3(q1, q2, q3):
    # Ejes base
    axes = [
        np.array([0, 0, 1]), # J1
        np.array([0, 1, 0]), # J2
        np.array([0, 1, 0]), # J3
    ]
    
    # Orientaciones base de las articulaciones de la muñeca (J4, J5, J6)
    # J4 is X, J5 is Y, J6 is X
    wrist_axes = [
        np.array([1, 0, 0]), # J4 (X)
        np.array([0, 1, 0]), # J5 (Y)
        np.array([1, 0, 0])  # J6 (X)
    ]
    
    # Aplicar rotaciones Q1, Q2, Q3
    q = [math.radians(q1), math.radians(q2), math.radians(q3)]
    
    for j in range(3):
        ang = q[j]
        # Frontend modifiers:
        if j == 1 or j == 2:
            ang = -ang
            
        axis = axes[j]
        
        # Orientar ejes siguientes
        for i in range(j+1, 3):
            axes[i] = apply_axis_angle(axes[i], axis, ang)
            
        for w in range(3):
            wrist_axes[w] = apply_axis_angle(wrist_axes[w], axis, ang)
            
    # Matrix R0_3 contains the rotated X, Y, Z axes of the wrist
    # Wait, R0_3 transforms from base to wrist frame.
    # The local wrist frame axes are wrist_axes[0]=X_local, wrist_axes[1]=Y_local, Z_local?
    # Since X is J4, Y is J5. Z_local = X_local cross Y_local.
    X_local = wrist_axes[0]
    Y_local = wrist_axes[1]
    Z_local = np.cross(X_local, Y_local)
    
    R0_3 = np.column_stack((X_local, Y_local, Z_local))
    return R0_3

R = visual_R0_3(0, 90, -90)
print(np.round(R, 3))
