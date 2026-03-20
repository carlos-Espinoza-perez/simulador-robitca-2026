

import re
from typing import List, Dict, Any, Tuple, Optional

def _strip_comments(text: str) -> str:
    
    result = []
    in_string = False
    for ch in text:
        if ch == '"':
            in_string = not in_string
        if ch == '!' and not in_string:
            break
        result.append(ch)
    return ''.join(result)

def _extract_nested_arrays(text: str) -> List[str]:
    
    arrays: List[str] = []
    depth = 0
    buf: List[str] = []
    for ch in text:
        if ch == '[':
            depth += 1
            if depth == 2:          # inicio de sub-array
                buf = []
            elif depth > 2:         # arrays más profundos (poco frecuente)
                buf.append(ch)
        elif ch == ']':
            if depth == 2:          # fin de sub-array
                arrays.append(''.join(buf))
            elif depth > 2:
                buf.append(ch)
            depth -= 1
        elif depth >= 2:
            buf.append(ch)
    return arrays

def _parse_float_list(csv: str) -> List[float]:
    
    values = []
    for token in csv.split(','):
        token = token.strip()
        if not token:
            continue
        values.append(float(token))
    return values

class RapidParser:
    def __init__(self):
        self.targets: Dict[str, Dict[str, Any]] = {}   # Constantes/variables declaradas
        self.procedures: Dict[str, List[str]] = {}
        self.module_name: str = "Unknown"
        self.warnings: List[str] = []

    def _preprocess(self, raw: str) -> List[str]:

        raw = raw.replace('\r\n', '\n').replace('\r', '\n')

        lines = [_strip_comments(line) for line in raw.split('\n')]

        blob = ' '.join(lines)

        block_keywords = [
            r'\bMODULE\b', r'\bENDMODULE\b', r'\bPROC\b', r'\bENDPROC\b',
            r'\bCONST\b', r'\bVAR\b', r'\bPERS\b', r'\bLOCAL\b',
        ]
        for kw in block_keywords:
            blob = re.sub(
                r'(?<![;\s])(\s+)(' + kw + r')',
                r'; \2',
                blob,
                flags=re.IGNORECASE
            )

        statements = blob.split(';')

        clean: List[str] = []
        for stmt in statements:
            s = ' '.join(stmt.split())     # colapsar whitespace
            s = s.strip()
            if s:
                clean.append(s)
        return clean

    def _classify_statements(self, statements: List[str]):
        
        current_proc: Optional[str] = None
        proc_body: List[str] = []

        for stmt in statements:
            upper = stmt.upper().lstrip()

            m = re.match(r'MODULE\s+(\w+)', stmt, re.IGNORECASE)
            if m:
                self.module_name = m.group(1)
                continue

            if upper.startswith('ENDMODULE'):
                continue

            m = re.match(
                r'(?:CONST|VAR|PERS|LOCAL\s+CONST|LOCAL\s+VAR|LOCAL\s+PERS)'
                r'\s+(jointtarget|robtarget|tooldata|wobjdata|speeddata|zonedata|num|bool|string)\s+'
                r'(\w+)\s*:=\s*(.+)',
                stmt, re.IGNORECASE
            )
            if m:
                dtype = m.group(1).lower()
                name  = m.group(2)
                value = m.group(3)
                self._store_target(dtype, name, value)
                continue

            m = re.match(r'PROC\s+(\w+)\s*\(', stmt, re.IGNORECASE)
            if m:
                current_proc = m.group(1).lower()
                proc_body = []
                continue

            m = re.match(r'PROC\s+(\w+)', stmt, re.IGNORECASE)
            if m and current_proc is None:
                current_proc = m.group(1).lower()
                proc_body = []
                continue

            if upper.startswith('ENDPROC'):
                if current_proc is not None:
                    self.procedures[current_proc] = proc_body
                    current_proc = None
                    proc_body = []
                continue

            if current_proc is not None:
                proc_body.append(stmt)

        if current_proc is not None and proc_body:
            self.procedures[current_proc] = proc_body

    def _store_target(self, dtype: str, name: str, value_str: str):
        
        try:
            arrays = _extract_nested_arrays(value_str)
            if dtype == 'jointtarget' and len(arrays) >= 1:
                joints = _parse_float_list(arrays[0])
                self.targets[name] = {
                    "type": "joint",
                    "joints": joints
                }
            elif dtype == 'robtarget' and len(arrays) >= 2:
                position   = _parse_float_list(arrays[0])
                quaternion = _parse_float_list(arrays[1])
                self.targets[name] = {
                    "type": "cartesian",
                    "position": position,
                    "quaternion": quaternion
                }

        except Exception as e:
            self.warnings.append(f"No se pudo parsear '{name}': {e}")

    def _parse_movements(self, body: List[str]) -> List[Dict[str, Any]]:
        movements: List[Dict[str, Any]] = []

        for stmt in body:
            mv = self._try_parse_movement(stmt)
            if mv is not None:
                movements.append(mv)
        return movements

    def _try_parse_movement(self, stmt: str) -> Optional[Dict[str, Any]]:
        
        upper = stmt.upper().lstrip()

        if upper.startswith('MOVEABSJ'):
            return self._parse_moveabsj(stmt)
        if upper.startswith('MOVEJ'):
            return self._parse_movej(stmt)
        if upper.startswith('MOVEL'):
            return self._parse_movel(stmt)
        if upper.startswith('MOVEC'):
            return self._parse_movec(stmt)

        return None

    def _parse_moveabsj(self, stmt: str) -> Optional[Dict[str, Any]]:
        
        try:

            rest = re.sub(r'^MoveAbsJ\s+', '', stmt, flags=re.IGNORECASE).strip()

            rest = re.sub(r'\\{1,2}[A-Za-z]\w*(?::=[^\s,]*)?', '', rest)

            tokens = [t.strip() for t in rest.split(',') if t.strip()]

            if not tokens:
                return None

            target_name = tokens[0]
            speed = self._find_speed(tokens)
            zone  = self._find_zone(tokens)

            if target_name in self.targets:
                target = self.targets[target_name]
                if target['type'] == 'joint':
                    return {
                        "type": "MoveAbsJ",
                        "target": target_name,
                        "joints": target['joints'],
                        "speed": speed,
                        "zone": zone
                    }
                elif target['type'] == 'cartesian':
                    return {
                        "type": "MoveAbsJ",
                        "target": target_name,
                        "target_type": "cartesian",
                        "data": target,
                        "speed": speed,
                        "zone": zone
                    }
            else:
                self.warnings.append(f"Target '{target_name}' no declarado (MoveAbsJ)")
            return None
        except Exception as e:
            self.warnings.append(f"Error en MoveAbsJ: {e}")
            return None

    def _parse_movej(self, stmt: str) -> Optional[Dict[str, Any]]:
        try:
            rest = re.sub(r'^MoveJ\s+', '', stmt, flags=re.IGNORECASE).strip()
            rest = re.sub(r'\\{1,2}[A-Za-z]\w*(?::=[^\s,]*)?', '', rest)
            tokens = [t.strip() for t in rest.split(',') if t.strip()]

            if not tokens:
                return None

            target_name = tokens[0]
            speed = self._find_speed(tokens)
            zone  = self._find_zone(tokens)

            if target_name in self.targets:
                target = self.targets[target_name]
                result = {
                    "type": "MoveJ",
                    "target": target_name,
                    "target_type": target['type'],
                    "data": target,
                    "speed": speed,
                    "zone": zone
                }
                if target['type'] == 'joint':
                    result['joints'] = target['joints']
                return result
            else:
                self.warnings.append(f"Target '{target_name}' no declarado (MoveJ)")
            return None
        except Exception as e:
            self.warnings.append(f"Error en MoveJ: {e}")
            return None

    def _parse_movel(self, stmt: str) -> Optional[Dict[str, Any]]:
        try:
            rest = re.sub(r'^MoveL\s+', '', stmt, flags=re.IGNORECASE).strip()
            rest = re.sub(r'\\{1,2}[A-Za-z]\w*(?::=[^\s,]*)?', '', rest)
            tokens = [t.strip() for t in rest.split(',') if t.strip()]

            if not tokens:
                return None

            target_name = tokens[0]
            speed = self._find_speed(tokens)
            zone  = self._find_zone(tokens)

            if target_name in self.targets:
                target = self.targets[target_name]
                return {
                    "type": "MoveL",
                    "target": target_name,
                    "target_type": target['type'],
                    "data": target,
                    "speed": speed,
                    "zone": zone
                }
            else:
                self.warnings.append(f"Target '{target_name}' no declarado (MoveL)")
            return None
        except Exception as e:
            self.warnings.append(f"Error en MoveL: {e}")
            return None

    def _parse_movec(self, stmt: str) -> Optional[Dict[str, Any]]:
        try:
            rest = re.sub(r'^MoveC\s+', '', stmt, flags=re.IGNORECASE).strip()
            rest = re.sub(r'\\{1,2}[A-Za-z]\w*(?::=[^\s,]*)?', '', rest)
            tokens = [t.strip() for t in rest.split(',') if t.strip()]

            if len(tokens) < 2:
                return None

            via_name = tokens[0]
            to_name  = tokens[1]
            speed = self._find_speed(tokens)
            zone  = self._find_zone(tokens)

            if via_name in self.targets and to_name in self.targets:
                return {
                    "type": "MoveC",
                    "via_point": via_name,
                    "to_point": to_name,
                    "via_data": self.targets[via_name],
                    "to_data": self.targets[to_name],
                    "speed": speed,
                    "zone": zone
                }
            else:
                missing = []
                if via_name not in self.targets:
                    missing.append(via_name)
                if to_name not in self.targets:
                    missing.append(to_name)
                self.warnings.append(f"Targets no declarados en MoveC: {missing}")
            return None
        except Exception as e:
            self.warnings.append(f"Error en MoveC: {e}")
            return None

    def _find_speed(self, tokens: List[str]) -> str:
        
        for t in tokens:
            m = re.match(r'^(v\d+|vmax)$', t, re.IGNORECASE)
            if m:
                return m.group(0).lower()
        return "v1000"

    def _find_zone(self, tokens: List[str]) -> str:
        
        for t in tokens:
            tl = t.lower()
            if tl == 'fine':
                return 'fine'
            m = re.match(r'^z\d+$', tl)
            if m:
                return tl
        return "fine"

    def parse(self, rapid_code: str) -> Dict[str, Any]:
        
        self.targets = {}
        self.procedures = {}
        self.module_name = "Unknown"
        self.warnings = []

        statements = self._preprocess(rapid_code)

        self._classify_statements(statements)

        movements: List[Dict[str, Any]] = []
        if 'main' in self.procedures:
            movements = self._parse_movements(self.procedures['main'])
        else:

            for proc_name, body in self.procedures.items():
                movements = self._parse_movements(body)
                if movements:
                    self.warnings.append(
                        f"No se encontró PROC main, usando PROC {proc_name}"
                    )
                    break

        return {
            "success": True,
            "module": self.module_name,
            "constants": self.targets,
            "procedures": list(self.procedures.keys()),
            "movements": movements,
            "total_steps": len(movements),
            "warnings": self.warnings
        }

def parse_rapid_code(rapid_code: str) -> Dict[str, Any]:
    
    parser = RapidParser()
    return parser.parse(rapid_code)
