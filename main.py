from flask import Flask, request, jsonify
from flask_cors import CORS
from flask_socketio import SocketIO, emit
import numpy as np
from datetime import datetime

from especificaciones_robot import ROBOTS
from cinematica_directa import (
    cinematica_directa,
    obtener_posicion_efector,
    matriz_a_euler,
    matriz_a_cuaternion
)
from rapid_parser import parse_rapid_code
from cinematica_inversa import solve_ik_from_robtarget
from interpolacion import (
    interpolar_joints,
    interpolar_lineal_cartesiano,
    interpolar_circular,
    calcular_num_pasos_por_velocidad
)
from analisis_singularidades import analisis_completo

app = Flask(__name__)
CORS(app, resources={r"/*": {"origins": "*"}})
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='eventlet')

estado_robot = {
    "robot_actual": "ABB_IRB_140",
    "angulos": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "clientes_conectados": 0
}

def obtener_robot_actual():
    return ROBOTS[estado_robot["robot_actual"]]

robot = obtener_robot_actual()
tabla_dh = robot["tabla_dh"]
limites = robot["limites_articulares"]


def calcular_estado(angulos):
    try:
        T_final, transformaciones = cinematica_directa(angulos, tabla_dh)
        pose = obtener_posicion_efector(T_final)
        
        R = np.array(pose['matriz_rotacion'])
        roll, pitch, yaw = matriz_a_euler(R)
        w, x, y, z = matriz_a_cuaternion(R)
        
        return {
            "success": True,
            "angulos": angulos,
            "posicion": {
                "x": pose['x'],
                "y": pose['y'],
                "z": pose['z']
            },
            "orientacion": {
                "euler": {"roll": roll, "pitch": pitch, "yaw": yaw},
                "cuaternion": {"w": w, "x": x, "y": y, "z": z}
            },
            "transformaciones": [T.tolist() for T in transformaciones]
        }
    except Exception as e:
        return {"success": False, "error": str(e)}


def quat_dict_to_list(quat_dict):
    """Convierte cuaternión de dict a lista [x, y, z, w]"""
    return [quat_dict['x'], quat_dict['y'], quat_dict['z'], quat_dict['w']]


def validar_limites(angulos):
    for i, angulo in enumerate(angulos):
        min_lim, max_lim = limites[i]
        if angulo < min_lim or angulo > max_lim:
            return False, f"Eje {i+1}: {angulo}° fuera de límites [{min_lim}°, {max_lim}°]"
    return True, "OK"


@app.route('/health', methods=['GET'])
def health():
    return jsonify({
        "status": "online",
        "robot": robot["nombre"],
        "clientes": estado_robot["clientes_conectados"]
    })


@app.route('/api/robots/lista', methods=['GET'])
def lista_robots():
    robots_disponibles = []
    for key, robot in ROBOTS.items():
        robots_disponibles.append({
            "id": key,
            "nombre": robot["nombre"],
            "tipo": robot["tipo"],
            "grados_libertad": robot["grados_libertad"]
        })
    return jsonify({
        "success": True,
        "robots": robots_disponibles,
        "robot_actual": estado_robot["robot_actual"]
    })


@app.route('/api/robot/seleccionar', methods=['POST'])
def seleccionar_robot():
    try:
        data = request.get_json()
        robot_id = data.get('robot_id')
        
        if not robot_id or robot_id not in ROBOTS:
            return jsonify({
                "success": False,
                "error": f"Robot '{robot_id}' no encontrado"
            }), 400
        
        estado_robot["robot_actual"] = robot_id
        estado_robot["angulos"] = [0.0] * ROBOTS[robot_id]["grados_libertad"]
        
        global robot, tabla_dh, limites
        robot = obtener_robot_actual()
        tabla_dh = robot["tabla_dh"]
        limites = robot["limites_articulares"]
        
        estado = calcular_estado(estado_robot["angulos"])
        
        socketio.emit('robot_cambiado', {
            "robot_id": robot_id,
            "robot_nombre": robot["nombre"],
            "estado": estado
        })
        
        return jsonify({
            "success": True,
            "robot": robot,
            "estado": estado
        })
    except Exception as e:
        return jsonify({"success": False, "error": str(e)}), 500


@app.route('/api/robot/info', methods=['GET'])
def info_robot():
    return jsonify({
        "success": True,
        "robot": robot
    })


@app.route('/api/robot/estado', methods=['GET'])
def estado():
    estado_actual = calcular_estado(estado_robot["angulos"])
    
    # Agregar análisis de singularidades
    analisis = analisis_completo(
        [np.rad2deg(a) for a in estado_robot["angulos"]], 
        tabla_dh, 
        limites
    )
    estado_actual["analisis_singularidades"] = analisis
    
    return jsonify(estado_actual)


@app.route('/api/robot/analisis', methods=['POST'])
def analizar_configuracion():
    """
    Analiza singularidades para una configuración específica
    """
    try:
        data = request.get_json()
        joints_deg = data.get('joints', estado_robot["angulos"])
        
        # Convertir a grados si vienen en radianes
        if all(abs(j) < 10 for j in joints_deg):  # Probablemente radianes
            joints_deg = [np.rad2deg(j) for j in joints_deg]
        
        analisis = analisis_completo(joints_deg, tabla_dh, limites)
        
        return jsonify({
            "success": True,
            "analisis": analisis
        })
    except Exception as e:
        return jsonify({
            "success": False,
            "error": str(e)
        }), 500


@app.route('/api/robot/mover', methods=['POST'])
def mover():
    try:
        data = request.get_json()
        angulos = data.get('angulos')
        
        n_articulaciones = robot["grados_libertad"]
        
        if not angulos or len(angulos) != n_articulaciones:
            return jsonify({
                "success": False,
                "error": f"Se requieren {n_articulaciones} ángulos"
            }), 400
        
        valido, mensaje = validar_limites(angulos)
        if not valido:
            return jsonify({"success": False, "error": mensaje}), 400
        
        estado_robot["angulos"] = angulos
        estado = calcular_estado(angulos)
        
        socketio.emit('actualizacion_robot', {
            "angulos": angulos,
            "posicion": estado["posicion"],
            "orientacion": estado["orientacion"]
        })
        
        return jsonify(estado)
    except Exception as e:
        return jsonify({"success": False, "error": str(e)}), 500


@app.route('/api/robot/home', methods=['POST'])
def home():
    n_articulaciones = robot["grados_libertad"]
    angulos_home = [0.0] * n_articulaciones
    estado_robot["angulos"] = angulos_home
    estado = calcular_estado(angulos_home)
    
    socketio.emit('actualizacion_robot', {
        "angulos": angulos_home,
        "posicion": estado["posicion"],
        "orientacion": estado["orientacion"]
    })
    
    return jsonify(estado)


@app.route('/api/cinematica/directa', methods=['POST'])
def cinematica():
    try:
        data = request.get_json()
        angulos = data.get('angulos')
        
        n_articulaciones = robot["grados_libertad"]
        
        if not angulos or len(angulos) != n_articulaciones:
            return jsonify({
                "success": False,
                "error": f"Se requieren {n_articulaciones} ángulos"
            }), 400
        
        return jsonify(calcular_estado(angulos))
    except Exception as e:
        return jsonify({"success": False, "error": str(e)}), 500


@app.route('/api/rapid/parse', methods=['POST'])
def parse_rapid():
    """
    Parsea código RAPID y retorna una lista de movimientos
    """
    try:
        data = request.get_json()
        rapid_code = data.get('code')
        
        if not rapid_code:
            return jsonify({
                "success": False,
                "error": "No se proporcionó código RAPID"
            }), 400
        
        # Parsear código RAPID
        result = parse_rapid_code(rapid_code)
        
        if not result.get('success'):
            return jsonify({
                "success": False,
                "error": "Error al parsear código RAPID"
            }), 400
        
        return jsonify(result)
        
    except Exception as e:
        return jsonify({
            "success": False,
            "error": f"Error al procesar código RAPID: {str(e)}"
        }), 500


@app.route('/api/rapid/execute', methods=['POST'])
def execute_rapid():
    """
    Ejecuta un programa RAPID paso a paso
    Retorna la lista completa de movimientos con cinemática calculada
    """
    try:
        data = request.get_json()
        rapid_code = data.get('code')
        
        if not rapid_code:
            return jsonify({
                "success": False,
                "error": "No se proporcionó código RAPID"
            }), 400
        
        # Parsear código RAPID
        parsed = parse_rapid_code(rapid_code)
        
        if not parsed.get('success'):
            return jsonify({
                "success": False,
                "error": "Error al parsear código RAPID"
            }), 400
        
        movements = parsed.get('movements', [])
        
        # Procesar cada movimiento y calcular cinemática
        processed_movements = []
        
        for i, movement in enumerate(movements):
            movement_data = {
                "step": i + 1,
                "type": movement['type'],
                "target": movement.get('target', ''),
                "speed": movement.get('speed', 'v1000'),
                "zone": movement.get('zone', 'fine'),
            }
            
            # Procesar según tipo de movimiento
            if movement['type'] in ['MoveAbsJ', 'MoveJ']:
                # Movimiento de articulaciones
                if 'joints' in movement:
                    joints = movement['joints']
                    movement_data['joints'] = joints
                    
                    # Calcular cinemática directa
                    estado = calcular_estado(joints)
                    if estado.get('success'):
                        movement_data['position'] = estado['posicion']
                        movement_data['orientation'] = estado['orientacion']
                    
                elif movement.get('target_type') == 'cartesian':
                    # Movimiento cartesiano - usar cinemática inversa
                    cart_data = movement['data']
                    position = cart_data['position']
                    quaternion = cart_data['quaternion']
                    
                    movement_data['position'] = {
                        'x': position[0],
                        'y': position[1],
                        'z': position[2]
                    }
                    movement_data['quaternion'] = quaternion
                    
                    # Resolver cinemática inversa
                    ik_result = solve_ik_from_robtarget(position, quaternion, limites)
                    
                    if ik_result['success']:
                        movement_data['joints'] = ik_result['joints']
                        movement_data['ik_message'] = ik_result['message']
                        
                        # Calcular cinemática directa para verificar
                        estado = calcular_estado(ik_result['joints'])
                        if estado.get('success'):
                            movement_data['orientation'] = estado['orientacion']
                    else:
                        movement_data['ik_error'] = ik_result['message']
                        movement_data['warning'] = 'Cinemática inversa falló'
                    
            elif movement['type'] == 'MoveL':
                # Movimiento lineal cartesiano
                if movement.get('target_type') == 'cartesian':
                    cart_data = movement['data']
                    position = cart_data['position']
                    quaternion = cart_data['quaternion']
                    
                    movement_data['position'] = {
                        'x': position[0],
                        'y': position[1],
                        'z': position[2]
                    }
                    movement_data['quaternion'] = quaternion
                    
                    # Resolver cinemática inversa
                    ik_result = solve_ik_from_robtarget(position, quaternion, limites)
                    
                    if ik_result['success']:
                        movement_data['joints'] = ik_result['joints']
                        movement_data['ik_message'] = ik_result['message']
                        
                        # Calcular cinemática directa para verificar
                        estado = calcular_estado(ik_result['joints'])
                        if estado.get('success'):
                            movement_data['orientation'] = estado['orientacion']
                    else:
                        movement_data['ik_error'] = ik_result['message']
                        movement_data['warning'] = 'Cinemática inversa falló'
                    
            elif movement['type'] == 'MoveC':
                # Movimiento circular
                via_data = movement['via_data']
                to_data = movement['to_data']
                
                movement_data['via_point'] = movement['via_point']
                movement_data['to_point'] = movement['to_point']
                movement_data['via_position'] = {
                    'x': via_data['position'][0],
                    'y': via_data['position'][1],
                    'z': via_data['position'][2]
                }
                movement_data['to_position'] = {
                    'x': to_data['position'][0],
                    'y': to_data['position'][1],
                    'z': to_data['position'][2]
                }
                # TODO: Generar puntos intermedios del arco
            
            processed_movements.append(movement_data)
        
        return jsonify({
            "success": True,
            "module": parsed['module'],
            "total_steps": len(processed_movements),
            "movements": processed_movements,
            "message": f"Programa parseado exitosamente: {len(processed_movements)} movimientos"
        })
        
    except Exception as e:
        return jsonify({
            "success": False,
            "error": f"Error al ejecutar código RAPID: {str(e)}"
        }), 500


@app.route('/api/rapid/interpolate', methods=['POST'])
def interpolate_rapid():
    """
    Genera trayectorias interpoladas para movimientos suaves
    Retorna lista expandida con puntos intermedios
    """
    try:
        data = request.get_json()
        rapid_code = data.get('code')
        
        if not rapid_code:
            return jsonify({
                "success": False,
                "error": "No se proporcionó código RAPID"
            }), 400
        
        # Parsear código RAPID
        parsed = parse_rapid_code(rapid_code)
        
        if not parsed.get('success'):
            return jsonify({
                "success": False,
                "error": "Error al parsear código RAPID"
            }), 400
        
        movements = parsed.get('movements', [])
        
        # Lista de trayectorias interpoladas
        interpolated_trajectory = []
        
        # Estado actual (posición inicial)
        current_joints = estado_robot["angulos"].copy()
        current_estado = calcular_estado(current_joints)
        current_position = [
            current_estado['posicion']['x'],
            current_estado['posicion']['y'],
            current_estado['posicion']['z']
        ]
        current_quaternion = quat_dict_to_list(current_estado['orientacion']['cuaternion'])
        
        for i, movement in enumerate(movements):
            num_pasos = calcular_num_pasos_por_velocidad(movement.get('speed', 'v1000'))
            
            # Procesar según tipo de movimiento
            if movement['type'] in ['MoveAbsJ', 'MoveJ']:
                # Movimiento de articulaciones
                if 'joints' in movement:
                    target_joints = movement['joints']
                    
                    # Interpolar en espacio de articulaciones
                    trayectoria_joints = interpolar_joints(current_joints, target_joints, num_pasos)
                    
                    # Convertir cada punto a formato completo
                    for j, joints in enumerate(trayectoria_joints):
                        estado = calcular_estado(joints)
                        if estado.get('success'):
                            interpolated_trajectory.append({
                                "step": len(interpolated_trajectory) + 1,
                                "movement_index": i,
                                "type": movement['type'],
                                "target": movement.get('target', ''),
                                "joints": joints,
                                "position": estado['posicion'],
                                "orientation": estado['orientacion'],
                                "is_intermediate": j < len(trayectoria_joints) - 1
                            })
                    
                    # Actualizar estado actual
                    current_joints = target_joints
                    current_estado = calcular_estado(current_joints)
                    current_position = [
                        current_estado['posicion']['x'],
                        current_estado['posicion']['y'],
                        current_estado['posicion']['z']
                    ]
                    current_quaternion = quat_dict_to_list(current_estado['orientacion']['cuaternion'])
                    
                elif movement.get('target_type') == 'cartesian':
                    # Movimiento cartesiano - resolver IK primero
                    cart_data = movement['data']
                    target_position = cart_data['position']
                    target_quaternion = cart_data['quaternion']
                    
                    # Resolver cinemática inversa
                    ik_result = solve_ik_from_robtarget(target_position, target_quaternion, limites)
                    
                    if ik_result['success']:
                        target_joints = ik_result['joints']
                        
                        # Interpolar en espacio de articulaciones
                        trayectoria_joints = interpolar_joints(current_joints, target_joints, num_pasos)
                        
                        for j, joints in enumerate(trayectoria_joints):
                            estado = calcular_estado(joints)
                            if estado.get('success'):
                                interpolated_trajectory.append({
                                    "step": len(interpolated_trajectory) + 1,
                                    "movement_index": i,
                                    "type": movement['type'],
                                    "target": movement.get('target', ''),
                                    "joints": joints,
                                    "position": estado['posicion'],
                                    "orientation": estado['orientacion'],
                                    "is_intermediate": j < len(trayectoria_joints) - 1
                                })
                        
                        current_joints = target_joints
                        current_position = target_position
                        current_quaternion = target_quaternion
                    else:
                        # IK falló, agregar solo el punto objetivo
                        interpolated_trajectory.append({
                            "step": len(interpolated_trajectory) + 1,
                            "movement_index": i,
                            "type": movement['type'],
                            "target": movement.get('target', ''),
                            "position": {
                                'x': target_position[0],
                                'y': target_position[1],
                                'z': target_position[2]
                            },
                            "ik_error": ik_result['message'],
                            "is_intermediate": False
                        })
                    
            elif movement['type'] == 'MoveL':
                # Movimiento lineal cartesiano
                if movement.get('target_type') == 'cartesian':
                    cart_data = movement['data']
                    target_position = cart_data['position']
                    target_quaternion = cart_data['quaternion']
                    
                    # Interpolar en espacio cartesiano
                    trayectoria_cart = interpolar_lineal_cartesiano(
                        current_position, current_quaternion,
                        target_position, target_quaternion,
                        num_pasos
                    )
                    
                    # Resolver IK para cada punto
                    for j, pose in enumerate(trayectoria_cart):
                        ik_result = solve_ik_from_robtarget(pose['position'], pose['quaternion'], limites)
                        
                        if ik_result['success']:
                            estado = calcular_estado(ik_result['joints'])
                            if estado.get('success'):
                                interpolated_trajectory.append({
                                    "step": len(interpolated_trajectory) + 1,
                                    "movement_index": i,
                                    "type": movement['type'],
                                    "target": movement.get('target', ''),
                                    "joints": ik_result['joints'],
                                    "position": estado['posicion'],
                                    "orientation": estado['orientacion'],
                                    "is_intermediate": j < len(trayectoria_cart) - 1
                                })
                    
                    # Actualizar estado actual
                    if ik_result['success']:
                        current_joints = ik_result['joints']
                        current_position = target_position
                        current_quaternion = target_quaternion
                    
            elif movement['type'] == 'MoveC':
                # Movimiento circular
                via_data = movement['via_data']
                to_data = movement['to_data']
                
                via_position = via_data['position']
                via_quaternion = via_data['quaternion']
                to_position = to_data['position']
                to_quaternion = to_data['quaternion']
                
                # Interpolar arco circular (mismo número de pasos que movimientos lineales)
                trayectoria_cart = interpolar_circular(
                    current_position, current_quaternion,
                    via_position, via_quaternion,
                    to_position, to_quaternion,
                    num_pasos  # Reducido de num_pasos * 2
                )
                
                # Resolver IK para cada punto
                for j, pose in enumerate(trayectoria_cart):
                    ik_result = solve_ik_from_robtarget(pose['position'], pose['quaternion'], limites)
                    
                    if ik_result['success']:
                        estado = calcular_estado(ik_result['joints'])
                        if estado.get('success'):
                            interpolated_trajectory.append({
                                "step": len(interpolated_trajectory) + 1,
                                "movement_index": i,
                                "type": movement['type'],
                                "target": f"{movement.get('via_point', '')} -> {movement.get('to_point', '')}",
                                "joints": ik_result['joints'],
                                "position": estado['posicion'],
                                "orientation": estado['orientacion'],
                                "is_intermediate": j < len(trayectoria_cart) - 1
                            })
                
                # Actualizar estado actual
                if ik_result['success']:
                    current_joints = ik_result['joints']
                    current_position = to_position
                    current_quaternion = to_quaternion
        
        return jsonify({
            "success": True,
            "module": parsed['module'],
            "total_movements": len(movements),
            "total_steps": len(interpolated_trajectory),
            "trajectory": interpolated_trajectory,
            "message": f"Trayectoria interpolada: {len(interpolated_trajectory)} puntos"
        })
        
    except Exception as e:
        import traceback
        traceback.print_exc()
        return jsonify({
            "success": False,
            "error": f"Error al interpolar trayectoria: {str(e)}"
        }), 500


@socketio.on('connect')
def conectar():
    estado_robot["clientes_conectados"] += 1
    print(f"✓ Cliente conectado (Total: {estado_robot['clientes_conectados']})")
    
    estado = calcular_estado(estado_robot["angulos"])
    emit('respuesta_conexion', {
        "status": "conectado",
        "robot": robot["nombre"],
        "estado_actual": estado
    })


@socketio.on('disconnect')
def desconectar():
    estado_robot["clientes_conectados"] -= 1
    print(f"✗ Cliente desconectado (Total: {estado_robot['clientes_conectados']})")


@socketio.on('solicitar_estado')
def solicitar_estado():
    emit('estado_robot', calcular_estado(estado_robot["angulos"]))


@socketio.on('mover_robot')
def mover_robot_ws(data):
    try:
        angulos = data.get('angulos')
        n_articulaciones = robot["grados_libertad"]
        
        if not angulos or len(angulos) != n_articulaciones:
            emit('error', {"mensaje": f"Se requieren {n_articulaciones} ángulos"})
            return
        
        valido, mensaje = validar_limites(angulos)
        if not valido:
            emit('error', {"mensaje": mensaje})
            return
        
        estado_robot["angulos"] = angulos
        estado = calcular_estado(angulos)
        
        socketio.emit('actualizacion_robot', {
            "angulos": angulos,
            "posicion": estado["posicion"],
            "orientacion": estado["orientacion"]
        })
    except Exception as e:
        emit('error', {"mensaje": str(e)})


@socketio.on('execute_rapid_stream')
def execute_rapid_stream(data):
    """
    Ejecuta código RAPID con streaming de puntos
    Envía cada punto conforme se calcula para ejecución inmediata
    """
    try:
        code = data.get('code')
        
        if not code:
            emit('rapid_error', {"message": "No se proporcionó código RAPID"})
            return
        
        # Parsear código RAPID
        parsed = parse_rapid_code(code)
        
        if not parsed.get('success'):
            emit('rapid_error', {"message": "Error al parsear código RAPID"})
            return
        
        movements = parsed.get('movements', [])
        
        # Estado actual
        current_joints = estado_robot["angulos"].copy()
        current_estado = calcular_estado(current_joints)
        current_position = [
            current_estado['posicion']['x'],
            current_estado['posicion']['y'],
            current_estado['posicion']['z']
        ]
        # Convertir cuaternión de dict a lista [x, y, z, w]
        current_quaternion = quat_dict_to_list(current_estado['orientacion']['cuaternion'])
        
        point_count = 0
        
        for i, movement in enumerate(movements):
            num_pasos = calcular_num_pasos_por_velocidad(movement.get('speed', 'v1000'))
            
            # Procesar según tipo de movimiento
            if movement['type'] in ['MoveAbsJ', 'MoveJ']:
                if 'joints' in movement:
                    target_joints = movement['joints']
                    trayectoria_joints = interpolar_joints(current_joints, target_joints, num_pasos)
                    
                    for j, joints in enumerate(trayectoria_joints):
                        estado = calcular_estado(joints)
                        if estado.get('success'):
                            point_count += 1
                            emit('rapid_point', {
                                "step": point_count,
                                "movement_index": i,
                                "type": movement['type'],
                                "target": movement.get('target', ''),
                                "joints": joints,
                                "position": estado['posicion'],
                                "orientation": estado['orientacion'],
                                "is_intermediate": j < len(trayectoria_joints) - 1
                            })
                            # Delay para visualización suave (50ms por paso)
                            socketio.sleep(0.05)
                    
                    current_joints = target_joints
                    current_estado = calcular_estado(current_joints)
                    current_position = [
                        current_estado['posicion']['x'],
                        current_estado['posicion']['y'],
                        current_estado['posicion']['z']
                    ]
                    current_quaternion = quat_dict_to_list(current_estado['orientacion']['cuaternion'])
                    
                elif movement.get('target_type') == 'cartesian':
                    cart_data = movement['data']
                    target_position = cart_data['position']
                    target_quaternion = cart_data['quaternion']
                    
                    ik_result = solve_ik_from_robtarget(target_position, target_quaternion, limites)
                    
                    if ik_result['success']:
                        target_joints = ik_result['joints']
                        trayectoria_joints = interpolar_joints(current_joints, target_joints, num_pasos)
                        
                        for j, joints in enumerate(trayectoria_joints):
                            estado = calcular_estado(joints)
                            if estado.get('success'):
                                point_count += 1
                                emit('rapid_point', {
                                    "step": point_count,
                                    "movement_index": i,
                                    "type": movement['type'],
                                    "target": movement.get('target', ''),
                                    "joints": joints,
                                    "position": estado['posicion'],
                                    "orientation": estado['orientacion'],
                                    "is_intermediate": j < len(trayectoria_joints) - 1
                                })
                                socketio.sleep(0.05)
                        
                        current_joints = target_joints
                        current_position = target_position
                        current_quaternion = target_quaternion
                    
            elif movement['type'] == 'MoveL':
                if movement.get('target_type') == 'cartesian':
                    cart_data = movement['data']
                    target_position = cart_data['position']
                    target_quaternion = cart_data['quaternion']
                    
                    trayectoria_cart = interpolar_lineal_cartesiano(
                        current_position, current_quaternion,
                        target_position, target_quaternion,
                        num_pasos
                    )
                    
                    for j, pose in enumerate(trayectoria_cart):
                        ik_result = solve_ik_from_robtarget(pose['position'], pose['quaternion'], limites)
                        
                        if ik_result['success']:
                            estado = calcular_estado(ik_result['joints'])
                            if estado.get('success'):
                                point_count += 1
                                emit('rapid_point', {
                                    "step": point_count,
                                    "movement_index": i,
                                    "type": movement['type'],
                                    "target": movement.get('target', ''),
                                    "joints": ik_result['joints'],
                                    "position": estado['posicion'],
                                    "orientation": estado['orientacion'],
                                    "is_intermediate": j < len(trayectoria_cart) - 1
                                })
                                socketio.sleep(0.05)
                    
                    if ik_result['success']:
                        current_joints = ik_result['joints']
                        current_position = target_position
                        current_quaternion = target_quaternion
                    
            elif movement['type'] == 'MoveC':
                via_data = movement['via_data']
                to_data = movement['to_data']
                
                via_position = via_data['position']
                via_quaternion = via_data['quaternion']
                to_position = to_data['position']
                to_quaternion = to_data['quaternion']
                
                trayectoria_cart = interpolar_circular(
                    current_position, current_quaternion,
                    via_position, via_quaternion,
                    to_position, to_quaternion,
                    num_pasos
                )
                
                for j, pose in enumerate(trayectoria_cart):
                    ik_result = solve_ik_from_robtarget(pose['position'], pose['quaternion'], limites)
                    
                    if ik_result['success']:
                        estado = calcular_estado(ik_result['joints'])
                        if estado.get('success'):
                            point_count += 1
                            emit('rapid_point', {
                                "step": point_count,
                                "movement_index": i,
                                "type": movement['type'],
                                "target": f"{movement.get('via_point', '')} -> {movement.get('to_point', '')}",
                                "joints": ik_result['joints'],
                                "position": estado['posicion'],
                                "orientation": estado['orientacion'],
                                "is_intermediate": j < len(trayectoria_cart) - 1
                            })
                            socketio.sleep(0.05)
                
                if ik_result['success']:
                    current_joints = ik_result['joints']
                    current_position = to_position
                    current_quaternion = to_quaternion
        
        # Enviar señal de completado
        emit('rapid_complete', {
            "total_points": point_count,
            "total_movements": len(movements)
        })
        
    except Exception as e:
        import traceback
        traceback.print_exc()
        emit('rapid_error', {"message": f"Error al ejecutar código RAPID: {str(e)}"})


def main():
    print("\n" + "="*60)
    print("  🤖 SIMULADOR ROBOT ABB IRB 140")
    print("="*60)
    print(f"\n  Robot: {robot['nombre']}")
    print(f"  Tipo: {robot['tipo']}")
    print(f"  Grados de libertad: {robot['grados_libertad']}")
    print(f"\n  Robots disponibles: {len(ROBOTS)}")
    for key, r in ROBOTS.items():
        print(f"    - {r['nombre']} ({key})")
    print("\n" + "-"*60)
    print("  ENDPOINTS:")
    print("-"*60)
    print("  GET  /health")
    print("  GET  /api/robots/lista")
    print("  POST /api/robot/seleccionar")
    print("  GET  /api/robot/info")
    print("  GET  /api/robot/estado")
    print("  POST /api/robot/mover")
    print("  POST /api/robot/home")
    print("  POST /api/cinematica/directa")
    print("  POST /api/rapid/parse")
    print("  POST /api/rapid/execute")
    print("\n" + "="*60)
    print("  Servidor: http://127.0.0.1:5000")
    print("  WebSocket: ws://127.0.0.1:5000")
    print("  Modo: DESARROLLO (Hot-Reload activado)")
    print("="*60 + "\n")
    
    socketio.run(
        app, 
        host='127.0.0.1', 
        port=5000, 
        debug=True, 
        use_reloader=True,
        allow_unsafe_werkzeug=True
    )


if __name__ == "__main__":
    main()
