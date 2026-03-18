# Simulador Robot ABB IRB 140

Simulador para análisis y validación de trayectorias de robots industriales con API REST y WebSocket.

## Características

- Soporte para múltiples robots (ABB IRB 140, ABB IRB 910SC)
- Cinemática directa (Denavit-Hartenberg)
- Orientación en Euler y Cuaterniones
- API REST + WebSocket para sincronización en tiempo real
- Validación de límites articulares
- Matrices de transformación para visualización 3D
- Configuración completa para Three.js

## Instalación

```bash
pip install -r requirements.txt
```

## Ejecutar

```bash
python main.py
```

Servidor: `http://127.0.0.1:5000`

## Robots Disponibles

- **ABB IRB 140** - Robot antropomórfico de 6 ejes
  - Carga máxima: 6 kg
  - Alcance: 1.5 m
  - Repetibilidad: ±0.03 mm
  
- **ABB IRB 910SC** - Robot SCARA de 4 ejes
  - Carga máxima: 3 kg
  - Alcance: 0.8 m
  - Repetibilidad: ±0.01 mm

## Documentación

### Para Desarrolladores
- **[INICIO_RAPIDO.md](INICIO_RAPIDO.md)** - Guía para nuevo desarrollador (EMPIEZA AQUÍ)
- **[CONTEXTO_PROYECTO.md](CONTEXTO_PROYECTO.md)** - Contexto rápido del proyecto
- **[README.md](README.md)** - Este archivo (visión general)
- **[API_DOCUMENTACION.md](API_DOCUMENTACION.md)** - Documentación completa de la API
- **[GUIA_DESARROLLO.md](GUIA_DESARROLLO.md)** - Guía para continuar el desarrollo
- **Skill de Kiro**: Activa `@simulador-robot` para contexto completo del proyecto

### Para Presentación/Documentación del Proyecto
- **[DESCRIPCION_PROYECTO.md](DESCRIPCION_PROYECTO.md)** - Descripción técnica completa del proyecto
- **[PRESENTACION_PROYECTO.md](PRESENTACION_PROYECTO.md)** - Versión resumida para presentaciones

## Endpoints Principales

| Método | Endpoint | Descripción |
|--------|----------|-------------|
| GET | `/health` | Estado del servidor |
| GET | `/api/robots/lista` | Lista de robots disponibles |
| POST | `/api/robot/seleccionar` | Seleccionar robot activo |
| GET | `/api/robot/info` | Información completa del robot |
| GET | `/api/robot/estado` | Estado actual + matrices de transformación |
| POST | `/api/robot/mover` | Mover robot a ángulos específicos |
| POST | `/api/robot/home` | Mover a posición home (0°) |
| POST | `/api/cinematica/directa` | Calcular cinemática directa |

## WebSocket

```
ws://127.0.0.1:5000
```

Eventos:
- `actualizacion_robot` - Broadcast cuando el robot se mueve
- `robot_cambiado` - Cuando se selecciona otro robot
- `respuesta_conexion` - Confirmación de conexión

## Integración con Three.js

El backend calcula las matrices de transformación de cada eslabón. Usa estas matrices para posicionar correctamente el robot:

```javascript
// 1. Cargar configuración y modelos STL
const response = await fetch('http://127.0.0.1:5000/api/robot/info');
const { robot } = await response.json();

const escala = robot.visualizacion.escala; // 0.001 (mm a metros)

// 2. Crear grupo contenedor con rotación inicial (corrige orientación de STL)
const robotGroup = new THREE.Group();
const [rx, ry, rz] = robot.visualizacion.rotacion_inicial;
robotGroup.rotation.set(
  rx * Math.PI / 180,
  ry * Math.PI / 180,
  rz * Math.PI / 180
);
scene.add(robotGroup);

// 3. Cargar cada eslabón
const eslabones = [];
for (const eslabonConfig of robot.eslabones) {
  const geometry = await loadSTL(`${robot.ruta_stl}/${eslabonConfig.archivo_stl}`);
  const material = new THREE.MeshStandardMaterial({
    color: eslabonConfig.color,
    metalness: robot.materiales.metalness,
    roughness: robot.materiales.roughness
  });
  const mesh = new THREE.Mesh(geometry, material);
  mesh.scale.set(escala, escala, escala);
  mesh.matrixAutoUpdate = false; // Usaremos matrices manuales
  robotGroup.add(mesh); // Agregar al grupo, no a la escena
  eslabones.push(mesh);
}

// 4. Obtener estado con matrices de transformación
const estadoResponse = await fetch('http://127.0.0.1:5000/api/robot/estado');
const estado = await estadoResponse.json();

// 5. Aplicar matrices de transformación
estado.transformaciones.forEach((matriz, index) => {
  if (eslabones[index]) {
    const matrix = new THREE.Matrix4();
    matrix.set(
      matriz[0][0], matriz[0][1], matriz[0][2], matriz[0][3] * escala,
      matriz[1][0], matriz[1][1], matriz[1][2], matriz[1][3] * escala,
      matriz[2][0], matriz[2][1], matriz[2][2], matriz[2][3] * escala,
      matriz[3][0], matriz[3][1], matriz[3][2], matriz[3][3]
    );
    eslabones[index].matrix.copy(matrix);
  }
});

// 6. Actualizar cuando el robot se mueve
socket.on('actualizacion_robot', async () => {
  const response = await fetch('http://127.0.0.1:5000/api/robot/estado');
  const estado = await response.json();
  
  estado.transformaciones.forEach((matriz, index) => {
    if (eslabones[index]) {
      const matrix = new THREE.Matrix4();
      matrix.set(
        matriz[0][0], matriz[0][1], matriz[0][2], matriz[0][3] * escala,
        matriz[1][0], matriz[1][1], matriz[1][2], matriz[1][3] * escala,
        matriz[2][0], matriz[2][1], matriz[2][2], matriz[2][3] * escala,
        matriz[3][0], matriz[3][1], matriz[3][2], matriz[3][3]
      );
      eslabones[index].matrix.copy(matrix);
    }
  });
});
```

Ver **[API_DOCUMENTACION.md](API_DOCUMENTACION.md)** para más detalles.

## Estructura del Proyecto

```
.
├── main.py                      # Servidor Flask
├── cinematica_directa.py        # Cinemática directa (DH)
├── especificaciones_robot.py    # Configuración de robots
├── requirements.txt             # Dependencias
├── API_DOCUMENTACION.md         # Documentación completa
├── README.md                    # Este archivo
├── test_api.py                  # Script de prueba
└── robot_configs/               # Archivos STL
    ├── ABB_IRB_140/
    └── ABB_IRB_910SC/
```

## Ejemplo de Uso

```bash
# Obtener información del robot
curl http://127.0.0.1:5000/api/robot/info

# Obtener estado con matrices de transformación
curl http://127.0.0.1:5000/api/robot/estado

# Mover robot
curl -X POST http://127.0.0.1:5000/api/robot/mover \
  -H "Content-Type: application/json" \
  -d '{"angulos": [0, -30, 30, 0, 60, 0]}'
```

## Notas Importantes

1. **Matrices de Transformación**: El backend calcula las matrices 4x4 de cada eslabón usando Denavit-Hartenberg. Usa estas matrices en Three.js para posicionar correctamente el robot.

2. **Escala**: Las matrices están en milímetros. Multiplica las componentes de traslación (columna 4) por `escala` (0.001) para convertir a metros.

3. **WebSocket**: Usa WebSocket para sincronización en tiempo real entre múltiples clientes.

4. **Archivos STL**: Los archivos STL ya tienen sus offsets internos correctos. No agregues posiciones adicionales, usa las matrices del backend.

5. **Robot Desarmado**: Si el robot aparece desarmado, verifica que estés aplicando las matrices de transformación del backend correctamente. Ver **[SOLUCION_FRONTEND.md](SOLUCION_FRONTEND.md)** para el código correcto.
