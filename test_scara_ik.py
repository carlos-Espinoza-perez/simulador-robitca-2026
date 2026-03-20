"""
Test script para diagnosticar el flujo de ejecución SCARA
"""
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np
from especificaciones_robot import ROBOTS
from cinematica_directa import cinematica_directa, obtener_posicion_efector, matriz_a_euler, matriz_a_cuaternion
from cinematica_inversa import solve_ik_from_robtarget, inverse_kinematics_scara
from interpolacion import interpolar_joints

# Cargar config SCARA
robot = ROBOTS["ABB_IRB_910SC"]
tabla_dh = robot["tabla_dh"]
limites = robot["limites_articulares"]
tipos_art = robot.get("tipos_articulaciones", ["R"] * robot["grados_libertad"])

print("=" * 60)
print("CONFIGURACION DEL SCARA")
print("=" * 60)
print(f"  Nombre: {robot['nombre']}")
print(f"  GDL: {robot['grados_libertad']}")
print(f"  Tipos articulaciones: {tipos_art}")
print(f"  Tabla DH ({len(tabla_dh)} filas):")
for i, row in enumerate(tabla_dh):
    print(f"    J{i+1}: theta={row[0]}, d={row[1]}, a={row[2]}, alpha={row[3]}")
print(f"  Limites ({len(limites)} entries): {limites}")

# Test 1: FK con [0,0,0,0]
print("\n" + "=" * 60)
print("TEST 1: Cinematica Directa con [0,0,0,0]")
print("=" * 60)
try:
    joints_home = [0, 0, 0, 0]
    T_final, transforms = cinematica_directa(joints_home, tabla_dh, tipos_art)
    pose = obtener_posicion_efector(T_final)
    print(f"  FK OK: x={pose['x']:.2f}, y={pose['y']:.2f}, z={pose['z']:.2f}")
except Exception as e:
    print(f"  FK FALLO: {e}")
    import traceback
    traceback.print_exc()

# Test 2: IK para P10=[350, 0, 200]
print("\n" + "=" * 60)
print("TEST 2: IK SCARA para P10=[350, 0, 200]")
print("=" * 60)
try:
    position = [350, 0, 200]
    quaternion = [1, 0, 0, 0]
    joints = inverse_kinematics_scara(position, quaternion)
    if joints is not None:
        print(f"  IK directo: J1={joints[0]:.2f} J2={joints[1]:.2f} J3={joints[2]:.2f} J4={joints[3]:.2f}")
        
        # Verificar con FK
        T_v, _ = cinematica_directa(joints.tolist(), tabla_dh, tipos_art)
        pose_v = obtener_posicion_efector(T_v)
        print(f"  FK verificacion: x={pose_v['x']:.2f}, y={pose_v['y']:.2f}, z={pose_v['z']:.2f}")
        error = np.linalg.norm(np.array([pose_v['x'], pose_v['y'], pose_v['z']]) - np.array(position))
        print(f"  Error: {error:.4f} mm {'OK' if error < 1 else 'FALLO'}")
    else:
        print(f"  IK retorno None")
    
    result = solve_ik_from_robtarget(position, quaternion, limites, from_robotstudio=True, robot_id="ABB_IRB_910SC")
    print(f"  Router: success={result['success']}, msg={result['message']}")
    if result.get('error_detail'):
        print(f"  Error: {result['error_detail']}")
except Exception as e:
    print(f"  FALLO: {e}")
    import traceback
    traceback.print_exc()

# Test 3: IK para P20=[350, 0, 80]
print("\n" + "=" * 60)
print("TEST 3: IK SCARA para P20=[350, 0, 80]")
print("=" * 60)
try:
    position = [350, 0, 80]
    quaternion = [1, 0, 0, 0]
    result = solve_ik_from_robtarget(position, quaternion, limites, from_robotstudio=True, robot_id="ABB_IRB_910SC")
    print(f"  success={result['success']}, msg={result['message']}")
    if result['joints']:
        print(f"  Joints: {[f'{v:.2f}' for v in result['joints']]}")
    if result.get('error_detail'):
        print(f"  Error: {result['error_detail']}")
except Exception as e:
    print(f"  FALLO: {e}")
    import traceback
    traceback.print_exc()

# Test 4: IK para P30=[400, 100, 80]
print("\n" + "=" * 60)
print("TEST 4: IK SCARA para P30=[400, 100, 80]")
print("=" * 60)
try:
    position = [400, 100, 80]
    quaternion = [1, 0, 0, 0]
    result = solve_ik_from_robtarget(position, quaternion, limites, from_robotstudio=True, robot_id="ABB_IRB_910SC")
    print(f"  success={result['success']}, msg={result['message']}")
    if result['joints']:
        print(f"  Joints: {[f'{v:.2f}' for v in result['joints']]}")
    if result.get('error_detail'):
        print(f"  Error: {result['error_detail']}")
except Exception as e:
    print(f"  FALLO: {e}")
    import traceback
    traceback.print_exc()

# Test 5: Interpolacion 
print("\n" + "=" * 60)
print("TEST 5: Interpolacion 4-joints")
print("=" * 60)
try:
    q_start = [0, 0, 0, 0]
    q_end = [30, -45, 120, 10]
    traj = interpolar_joints(q_start, q_end, 3)
    print(f"  Trayectoria ({len(traj)} puntos, {len(traj[0])} elementos cada uno)")
    for i, pt in enumerate(traj):
        print(f"    [{i}] {[f'{v:.1f}' for v in pt]}")
except Exception as e:
    print(f"  FALLO: {e}")
    import traceback
    traceback.print_exc()

# Test 6: Flujo completo simulado
print("\n" + "=" * 60)
print("TEST 6: Flujo completo MoveAbsJ -> MoveJ P10 -> MoveL P20")
print("=" * 60)
try:
    current_joints = [0, 0, 0, 0]
    
    # Step 1: calcular_estado equivalente
    T_final, _ = cinematica_directa(current_joints, tabla_dh, tipos_art)
    pose = obtener_posicion_efector(T_final)
    current_position = [pose['x'], pose['y'], pose['z']]
    print(f"  Home pos: ({pose['x']:.1f}, {pose['y']:.1f}, {pose['z']:.1f})")
    
    # Step 2: MoveJ P10
    target_pos = [350, 0, 200]
    ik = solve_ik_from_robtarget(target_pos, [1,0,0,0], limites, True, "ABB_IRB_910SC")
    print(f"\n  MoveJ P10: IK success={ik['success']}")
    if ik['success']:
        target_joints = ik['joints']
        print(f"  Target joints: {[f'{v:.2f}' for v in target_joints]}")
        traj = interpolar_joints(current_joints, target_joints, 3)
        for i, pt in enumerate(traj):
            T_i, _ = cinematica_directa(pt, tabla_dh, tipos_art)
            p_i = obtener_posicion_efector(T_i)
            print(f"    [{i}] pos=({p_i['x']:.1f}, {p_i['y']:.1f}, {p_i['z']:.1f})")
        current_joints = target_joints
    else:
        print(f"  FALLO: {ik.get('error_detail')}")
    
    # Step 3: MoveL P20
    target_pos = [350, 0, 80]
    ik = solve_ik_from_robtarget(target_pos, [1,0,0,0], limites, True, "ABB_IRB_910SC")
    print(f"\n  MoveL P20: IK success={ik['success']}")
    if ik['success']:
        target_joints = ik['joints']
        print(f"  Target joints: {[f'{v:.2f}' for v in target_joints]}")
        traj = interpolar_joints(current_joints, target_joints, 3)
        for i, pt in enumerate(traj):
            T_i, _ = cinematica_directa(pt, tabla_dh, tipos_art)
            p_i = obtener_posicion_efector(T_i)
            print(f"    [{i}] pos=({p_i['x']:.1f}, {p_i['y']:.1f}, {p_i['z']:.1f})")
        current_joints = target_joints
    else:
        print(f"  FALLO: {ik.get('error_detail')}")
        
    # Step 4: MoveL P30
    target_pos = [400, 100, 80]
    ik = solve_ik_from_robtarget(target_pos, [1,0,0,0], limites, True, "ABB_IRB_910SC")
    print(f"\n  MoveL P30: IK success={ik['success']}")
    if ik['success']:
        target_joints = ik['joints']
        print(f"  Target joints: {[f'{v:.2f}' for v in target_joints]}")
    else:
        print(f"  FALLO: {ik.get('error_detail')}")
        
except Exception as e:
    print(f"  FALLO GENERAL: {e}")
    import traceback
    traceback.print_exc()

print("\n" + "=" * 60)
print("TESTS COMPLETADOS")
print("=" * 60)
