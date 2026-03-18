"""
Parser de código RAPID para ABB
Convierte código RAPID a una lista de movimientos ejecutables
"""

import re
from typing import List, Dict, Any, Tuple, Optional
import numpy as np


class RapidParser:
    def __init__(self):
        self.constants = {}
        self.procedures = {}
        self.current_module = None
        
    def parse(self, rapid_code: str) -> Dict[str, Any]:
        """
        Parsea código RAPID y retorna una estructura de datos ejecutable
        """
        # Normalizar el código: agregar saltos de línea en puntos clave
        rapid_code = rapid_code.replace(';', ';\n')  # Después de punto y coma
        rapid_code = rapid_code.replace('MODULE ', '\nMODULE ')  # Antes de MODULE
        rapid_code = rapid_code.replace('CONST ', '\nCONST ')  # Antes de CONST
        rapid_code = rapid_code.replace('PROC ', '\nPROC ')  # Antes de PROC
        rapid_code = rapid_code.replace('ENDPROC', '\nENDPROC\n')  # Alrededor de ENDPROC
        rapid_code = rapid_code.replace('ENDMODULE', '\nENDMODULE\n')  # Alrededor de ENDMODULE
        
        lines = rapid_code.strip().split('\n')
        
        # Limpiar líneas
        lines = [self._clean_line(line) for line in lines]
        lines = [line for line in lines if line and line != ';']  # Remover líneas vacías y solo punto y coma
        
        # Parsear constantes y procedimientos
        i = 0
        while i < len(lines):
            line = lines[i]
            
            # Detectar MODULE
            if line.startswith('MODULE'):
                self.current_module = self._extract_module_name(line)
                
            # Detectar CONST
            elif line.startswith('CONST'):
                const_name, const_value = self._parse_constant(line)
                if const_name:
                    self.constants[const_name] = const_value
                    
            # Detectar PROC
            elif line.startswith('PROC'):
                proc_name = self._extract_proc_name(line)
                proc_body = []
                i += 1
                
                # Leer cuerpo del procedimiento
                while i < len(lines) and not lines[i].startswith('ENDPROC'):
                    if lines[i]:
                        proc_body.append(lines[i])
                    i += 1
                    
                self.procedures[proc_name] = proc_body
                
            i += 1
        
        # Generar lista de movimientos desde el procedimiento main
        movements = []
        if 'main' in self.procedures:
            movements = self._parse_movements(self.procedures['main'])
        
        return {
            "success": True,
            "module": self.current_module,
            "constants": self.constants,
            "procedures": list(self.procedures.keys()),
            "movements": movements,
            "total_steps": len(movements)
        }
    
    def _clean_line(self, line: str) -> str:
        """Limpia una línea de código"""
        # Remover comentarios
        if '!' in line:
            line = line[:line.index('!')]
        # Remover espacios extra
        line = line.strip()
        # Agregar espacio después de punto y coma si no existe (para separar comandos en una línea)
        line = line.replace(';', '; ')
        return line
    
    def _extract_module_name(self, line: str) -> str:
        """Extrae el nombre del módulo"""
        match = re.search(r'MODULE\s+(\w+)', line)
        return match.group(1) if match else "Unknown"
    
    def _extract_proc_name(self, line: str) -> str:
        """Extrae el nombre del procedimiento"""
        match = re.search(r'PROC\s+(\w+)', line)
        return match.group(1) if match else "unknown"
    
    def _parse_constant(self, line: str) -> Tuple[Optional[str], Optional[Dict]]:
        """Parsea una constante RAPID"""
        try:
            # CONST jointtarget NAME:=[[j1,j2,j3,j4,j5,j6],[...]]
            # CONST robtarget NAME:=[[x,y,z],[q1,q2,q3,q4],[...],[...]]
            
            match = re.search(r'CONST\s+(\w+)\s+(\w+):=(.+)', line)
            if not match:
                return None, None
            
            const_type = match.group(1)
            const_name = match.group(2)
            const_value = match.group(3)
            
            # Extraer arrays anidados correctamente
            arrays = self._extract_nested_arrays(const_value)
            
            if const_type == 'jointtarget':
                # Extraer ángulos de articulaciones (primer array)
                if len(arrays) >= 1:
                    joints = [float(x.strip()) for x in arrays[0].split(',')]
                    return const_name, {
                        "type": "joint",
                        "joints": joints
                    }
                    
            elif const_type == 'robtarget':
                # Extraer posición cartesiana y cuaternión
                # [[x,y,z],[q1,q2,q3,q4],[...],[...]]
                if len(arrays) >= 2:
                    position = [float(x.strip()) for x in arrays[0].split(',')]
                    quaternion = [float(x.strip().replace('E-', 'e-').replace('E+', 'e+')) 
                                 for x in arrays[1].split(',')]
                    return const_name, {
                        "type": "cartesian",
                        "position": position,
                        "quaternion": quaternion
                    }
            
            return None, None
            
        except Exception as e:
            print(f"Error parseando constante: {line}, Error: {e}")
            return None, None
    
    def _extract_nested_arrays(self, text: str) -> List[str]:
        """Extrae arrays de un texto que puede contener arrays anidados"""
        arrays = []
        depth = 0
        current_array = ""
        
        for char in text:
            if char == '[':
                depth += 1
                if depth > 1:  # Solo capturar contenido de arrays internos
                    current_array = ""
            elif char == ']':
                depth -= 1
                if depth == 1:  # Fin de un array interno
                    arrays.append(current_array)
                    current_array = ""
            elif depth > 1:  # Dentro de un array interno
                current_array += char
        
        return arrays
    
    def _parse_movements(self, proc_body: List[str]) -> List[Dict[str, Any]]:
        """Parsea los movimientos de un procedimiento"""
        movements = []
        
        for i, line in enumerate(proc_body):
            movement = None
            
            # MoveAbsJ - Movimiento absoluto de articulaciones
            if line.startswith('MoveAbsJ'):
                movement = self._parse_move_absj(line)
                
            # MoveJ - Movimiento de articulaciones
            elif line.startswith('MoveJ'):
                movement = self._parse_move_j(line)
                
            # MoveL - Movimiento lineal
            elif line.startswith('MoveL'):
                movement = self._parse_move_l(line)
                
            # MoveC - Movimiento circular
            elif line.startswith('MoveC'):
                movement = self._parse_move_c(line)
            
            if movement:
                movements.append(movement)
        
        return movements
    
    def _parse_move_absj(self, line: str) -> Optional[Dict[str, Any]]:
        """Parsea MoveAbsJ"""
        try:
            # MoveAbsJ ZERO\NoEOffs, v1000, fine, tool0;
            # Remover modificadores como \NoEOffs
            line = line.replace('\\NoEOffs', '').replace('\\NoEoffs', '')
            
            match = re.search(r'MoveAbsJ\s+(\w+)', line)
            if match:
                target_name = match.group(1)
                
                if target_name in self.constants:
                    target = self.constants[target_name]
                    if target['type'] == 'joint':
                        return {
                            "type": "MoveAbsJ",
                            "target": target_name,
                            "joints": target['joints'],
                            "speed": self._extract_speed(line),
                            "zone": self._extract_zone(line)
                        }
            return None
        except Exception as e:
            print(f"Error parseando MoveAbsJ: {line}, Error: {e}")
            return None
    
    def _parse_move_j(self, line: str) -> Optional[Dict[str, Any]]:
        """Parsea MoveJ"""
        try:
            # MoveJ P10, v1000, fine, tool0;
            match = re.search(r'MoveJ\s+(\w+)', line)
            if match:
                target_name = match.group(1)
                
                if target_name in self.constants:
                    target = self.constants[target_name]
                    
                    return {
                        "type": "MoveJ",
                        "target": target_name,
                        "target_type": target['type'],
                        "data": target,
                        "speed": self._extract_speed(line),
                        "zone": self._extract_zone(line)
                    }
            return None
        except Exception as e:
            print(f"Error parseando MoveJ: {line}, Error: {e}")
            return None
    
    def _parse_move_l(self, line: str) -> Optional[Dict[str, Any]]:
        """Parsea MoveL"""
        try:
            # MoveL P20, v1000, fine, tool0;
            match = re.search(r'MoveL\s+(\w+)', line)
            if match:
                target_name = match.group(1)
                
                if target_name in self.constants:
                    target = self.constants[target_name]
                    
                    return {
                        "type": "MoveL",
                        "target": target_name,
                        "target_type": target['type'],
                        "data": target,
                        "speed": self._extract_speed(line),
                        "zone": self._extract_zone(line)
                    }
            return None
        except Exception as e:
            print(f"Error parseando MoveL: {line}, Error: {e}")
            return None
    
    def _parse_move_c(self, line: str) -> Optional[Dict[str, Any]]:
        """Parsea MoveC (movimiento circular)"""
        try:
            # MoveC P80, P90, v100, fine, tool0;
            match = re.search(r'MoveC\s+(\w+),\s*(\w+)', line)
            if match:
                via_point = match.group(1)
                to_point = match.group(2)
                
                if via_point in self.constants and to_point in self.constants:
                    return {
                        "type": "MoveC",
                        "via_point": via_point,
                        "to_point": to_point,
                        "via_data": self.constants[via_point],
                        "to_data": self.constants[to_point],
                        "speed": self._extract_speed(line),
                        "zone": self._extract_zone(line)
                    }
            return None
        except Exception as e:
            print(f"Error parseando MoveC: {line}, Error: {e}")
            return None
    
    def _extract_speed(self, line: str) -> str:
        """Extrae la velocidad del comando"""
        match = re.search(r'v(\d+)', line)
        return f"v{match.group(1)}" if match else "v1000"
    
    def _extract_zone(self, line: str) -> str:
        """Extrae la zona del comando"""
        if 'fine' in line:
            return "fine"
        match = re.search(r'z(\d+)', line)
        return f"z{match.group(1)}" if match else "fine"


def parse_rapid_code(rapid_code: str) -> Dict[str, Any]:
    """
    Función principal para parsear código RAPID
    """
    parser = RapidParser()
    return parser.parse(rapid_code)
