import math
import numpy as np

# Copiar la lógica de RobotViewer3D.tsx
DX = 0.247 * 1000
DY = 0.203 * 1000

PIVOTS_HOME = [
    np.array([0, 0, 0]),
    np.array([DX + 65, DY, 0]),
    np.array([DX + 140, DY, 352]),
    np.array([DX + 140, DY, 712]),
    np.array([DX + 70, DY, 712]),
    np.array([DX + 520, DY, 712]),
    np.array([DX + 515, DY, 712])
]

# Ejes de rotación en Three.js
AXES_HOME = [
    np.array([0, 0, 0]),
    np.array([0, 0, 1]), # J1
    np.array([0, 1, 0]), # J2
    np.array([0, 1, 0]), # J3
    np.array([1, 0, 0]), # J4
    np.array([0, 1, 0]), # J5
    np.array([1, 0, 0])  # J6
]

def apply_axis_angle(vec, axis, angle):
    # Rodrigues rotation formula
    v = np.array(vec)
    k = np.array(axis)
    k = k / np.linalg.norm(k)
    return v * math.cos(angle) + np.cross(k, v) * math.sin(angle) + k * np.dot(k, v) * (1 - math.cos(angle))

def robot_viewer_fk(joint_angles_deg):
    q_rad = [math.radians(deg) for deg in joint_angles_deg]
    
    pivots = [p.copy() for p in PIVOTS_HOME]
    axes = [a.copy() for a in AXES_HOME]
    
    for j in range(1, 7):
        ang = q_rad[j - 1]
        if abs(ang) < 1e-5:
            continue
            
        origin = pivots[j]
        axis = axes[j]
        
        # Apply rotation to subsequent pivots
        for p_idx in range(j + 1, 7):
            offset = pivots[p_idx] - origin
            new_offset = apply_axis_angle(offset, axis, ang)
            pivots[p_idx] = origin + new_offset
            axes[p_idx] = apply_axis_angle(axes[p_idx], axis, ang)
            
    # Calculate TCP (last Pivot + offset along last axis)
    tcpOffset = 65
    lastPivot = pivots[6]
    lastAxis = axes[6]
    tcpPosition = lastPivot + lastAxis * tcpOffset
    
    return tcpPosition

print("ZERO joints:", robot_viewer_fk([0, 0, 0, 0, 0, 0]))

# Given IK from the user issue for P10
ik_joints = [0.0, 72.49746640115283, 20.75581562318599, 0.0, 93.25327727089761, 180.0]
print("IK P10 applied to Visual FK:", robot_viewer_fk(ik_joints))

# Let's try negative q2 and adjusted q3
# Let's test the derived mappings!
# Target X=450, Z=450.
# In DH: r = sqrt(450^2 + 0) = 450. s = 450 - 352 = 98.
d1 = 352.0; a2 = 360.0; L3 = 380.0
r = 450.0; s = 98.0
D = math.sqrt(r**2 + s**2)
alpha = math.degrees(math.atan2(s, r))
cos_beta = (a2**2 + D**2 - L3**2) / (2 * a2 * D)
beta = math.degrees(math.acos(cos_beta))
q2_geom = alpha + beta

cos_gamma = (a2**2 + L3**2 - D**2) / (2 * a2 * L3)
gamma_geom = math.degrees(math.acos(cos_gamma))

# Map to ABB
j2_abb = 90.0 - q2_geom
j3_abb = 90.0 - gamma_geom

# With no negations, let's see where a target of 0, 90, 0 sends the arm.
# q2_abb = 90 should point forward horizontally!
print("ABB J2=90:", robot_viewer_fk([0.0, 90.0, 0.0, 0.0, 0.0, 0.0]))

# P10 using standard IK solver mapping?
# IK: q2_geom = 65.79. J2_abb = 90 - 65.79 = 24.21.
# IK: q3_geom = 90 - gamma.
# J2=90 (Horizontal), J3=-90 (Bend UP to counteract so forearm goes Horizontal again)
print("ABB J2=90, J3=-90:", robot_viewer_fk([0.0, 90.0, -90.0, 0.0, 0.0, 0.0]))

# Let's test standard DH against visual!
from cinematica_directa import cinematica_directa, obtener_posicion_efector

standard_dh = [
    [0, 352, 0, -90],
    [-90, 0, 360, 0],
    [0, 0, 0, -90],
    [0, 380, 0, 90],
    [0, 0, 0, -90],
    [0, 65, 0, 0]
]

def dh_tcp(angles):
    T, _ = cinematica_directa(angles, standard_dh)
    pose = obtener_posicion_efector(T)
    # DX=247, DY=203 are visual offsets not in DH
    return [pose['x'] + DX, pose['y'] + DY, pose['z']]

print("DH Zero:", dh_tcp([0,0,0,0,0,0]), " Visual:", robot_viewer_fk([0,0,0,0,0,0]))
print("DH J2=90:", dh_tcp([0,90,0,0,0,0]), " Visual:", robot_viewer_fk([0,90,0,0,0,0]))
print("DH J2=90, J3=-90:", dh_tcp([0,90,-90,0,0,0]), " Visual:", robot_viewer_fk([0,90,-90,0,0,0]))
