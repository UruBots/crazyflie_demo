# Crazyflie Dual Control

Paquete ROS2 para controlar 2 Crazyflies simultáneamente usando [Crazyswarm2](https://github.com/IMRCLab/crazyswarm2). Incluye control manual con teclado, simulación con Gazebo, multiranger y capacidades de mapeo.

## Características

- ✅ Control simultáneo de 2 Crazyflies
- ✅ Control manual con teclado
- ✅ Simulación con Gazebo
- ✅ Soporte para multiranger
- ✅ Mapeo simultáneo
- ✅ Navegación autónoma
- ✅ Monitoreo de batería
- ✅ Formaciones de vuelo
- ✅ Evasión de obstáculos

## Requisitos

### Hardware
- 2 Crazyflies (CF2.0 o CF2.1)
- 2 Crazyradios PA
- Computadora con Ubuntu 22.04+ y ROS2 Jazzy

### Software
- ROS2 Jazzy
- Crazyswarm2
- Gazebo (para simulación)
- Python 3.10+

## Instalación

### Instalación Automática (Recomendada)

```bash
# Descargar y ejecutar script de instalación
wget https://raw.githubusercontent.com/tu-usuario/crazyflie_dual_control/main/install_crazyflie_dual_control.sh
chmod +x install_crazyflie_dual_control.sh
./install_crazyflie_dual_control.sh
```

### Instalación Manual

#### 1. Instalar Crazyswarm2

```bash
# Crear workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clonar Crazyswarm2
git clone https://github.com/IMRCLab/crazyswarm2 --recursive
cd crazyswarm2

# Instalar dependencias
rosdep install --from-paths . --ignore-src -r -y

# Compilar
cd ~/ros2_ws
colcon build
source install/setup.bash
```

#### 2. Instalar paquetes adicionales (opcional)

Para simulación y multiranger:

```bash
cd ~/ros2_ws/src

# Clonar repositorios adicionales
git clone https://github.com/bitcraze/crazyflie-simulation.git
git clone https://github.com/knmcguire/crazyflie_ros2_multiranger.git

# Compilar
cd ~/ros2_ws
colcon build
source install/setup.bash
```

#### 3. Configurar permisos USB

```bash
# Ejecutar script de permisos
sudo ./setup_usb_permissions.sh

# Reiniciar contenedor Docker (si usas Docker)
docker-compose down
docker-compose up -d
```

### Verificar Instalación

```bash
# Ejecutar script de prueba
./test_crazyflie_setup.sh
```

## Configuración

### 1. Configurar drones

Edita el archivo `config/crazyflies.yaml` para configurar tus drones:

```yaml
crazyflies:
  cf1:
    id: 1
    channel: 80
    initialPosition: [0.0, 0.0, 0.0]
    type: default
    
  cf2:
    id: 2
    channel: 80
    initialPosition: [1.0, 0.0, 0.0]
    type: default
```

### 2. Verificar conexión

```bash
# Verificar dispositivos USB
lsusb | grep 1915

# Probar conexión
python3 test_crazyflie_connection.py
```

## Uso

### Control Manual con Teclado

```bash
# Terminal 1: Lanzar servidor y controlador
ros2 launch crazyflie_dual_control dual_crazyflies.launch.py

# Terminal 2: Controlador de teclado
ros2 run crazyflie_dual_control keyboard_controller
```

**Controles del teclado:**
- `WASD` - Mover drone actual (adelante/izq/atrás/der)
- `QE` - Rotar drone actual (izq/der)
- `RF` - Subir/Bajar drone actual
- `T` - Despegar drone actual
- `G` - Aterrizar drone actual
- `TAB` - Cambiar drone (cf1 ↔ cf2)
- `SPACE` - Parada de emergencia
- `H` - Mostrar ayuda
- `ESC/q` - Salir

### Vuelo Automático

```bash
# Vuelo simple con patrón predefinido
ros2 run crazyflie_dual_control simple_flight

# Controlador avanzado con exploración
ros2 run crazyflie_dual_control advanced_controller
```

### Simulación con Gazebo

```bash
# Lanzar simulación completa
ros2 launch crazyflie_dual_control simulation.launch.py

# Con mapeo habilitado
ros2 launch crazyflie_dual_control simulation.launch.py enable_mapping:=true

# Con RViz para visualización
ros2 launch crazyflie_dual_control simulation.launch.py enable_rviz:=true
```

### Monitoreo de Batería

```bash
# Monitorear batería de ambos drones
ros2 run crazyflie_dual_control battery_monitor
```

## Archivos de Lanzamiento

### `dual_crazyflies.launch.py`
Lanza el sistema completo para 2 drones reales:
- Servidor Crazyflie
- Controlador dual
- Monitor de batería
- RViz (opcional)

### `simulation.launch.py`
Lanza simulación con Gazebo:
- Gazebo con mundo vacío
- 2 drones simulados
- Multiranger habilitado
- Mapeo simultáneo
- RViz para visualización

## Scripts Disponibles

### `dual_controller.py`
Controlador principal con servicios ROS2:
- Despegue/aterrizaje coordinado
- Comandos de posición
- Patrones de vuelo
- Parada de emergencia

### `keyboard_controller.py`
Control manual con teclado:
- Control individual de cada drone
- Cambio dinámico entre drones
- Controles intuitivos
- Feedback visual

### `simple_flight.py`
Vuelo automático simple:
- Patrón de despegue
- Formación horizontal
- Movimiento circular
- Aterrizaje automático

### `advanced_controller.py`
Controlador avanzado con:
- Exploración autónoma
- Evasión de obstáculos
- Fusión de mapas
- Múltiples modos de operación

### `battery_monitor.py`
Monitor de batería:
- Alertas de batería baja
- Monitoreo continuo
- Notificaciones ROS2

## Configuración Avanzada

### Modos de Operación

El controlador avanzado soporta varios modos:

```python
# Cambiar modo de operación
controller.set_operation_mode('exploration')  # Exploración autónoma
controller.set_operation_mode('formation')    # Vuelo en formación
controller.set_operation_mode('mapping')      # Mapeo coordinado
controller.set_operation_mode('manual')       # Control manual
```

### Parámetros de Vuelo

Edita `config/crazyflies.yaml` para ajustar:

```yaml
all:
  maxVel: 2.0        # Velocidad máxima (m/s)
  maxAcc: 1.0        # Aceleración máxima (m/s²)
  maxYawRate: 3.14   # Velocidad de giro máxima (rad/s)
  batteryThreshold: 3.0  # Umbral de batería (V)
```

## Solución de Problemas

### Error de Permisos USB
```bash
# Verificar grupos del usuario
groups $USER

# Verificar dispositivos USB
ls -la /dev/bus/usb/*/

# Reiniciar servicios udev
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### Drones No Detectados
1. Verificar que las antenas estén conectadas
2. Ejecutar `lsusb` para ver dispositivos USB
3. Verificar que los drivers estén instalados
4. Reiniciar el contenedor después de conectar dispositivos

### Problemas de Conexión ROS2
```bash
# Verificar que ROS2 esté funcionando
ros2 node list

# Verificar topics
ros2 topic list

# Verificar servicios
ros2 service list
```

### Error de Compilación
```bash
# Limpiar y recompilar
cd ~/ros2_ws
rm -rf build/ install/ log/
colcon build
source install/setup.bash
```

## Estructura del Proyecto

```
crazyflie_dual_control/
├── config/
│   ├── crazyflies.yaml              # Configuración drones reales
│   └── crazyflies_simulation.yaml   # Configuración simulación
├── launch/
│   ├── dual_crazyflies.launch.py    # Lanzamiento drones reales
│   └── simulation.launch.py         # Lanzamiento simulación
├── rviz/
│   └── dual_crazyflies.rviz         # Configuración RViz
├── crazyflie_dual_control/
│   ├── dual_controller.py           # Controlador principal
│   ├── keyboard_controller.py       # Control teclado
│   ├── simple_flight.py             # Vuelo automático
│   ├── advanced_controller.py      # Controlador avanzado
│   └── battery_monitor.py           # Monitor batería
├── package.xml                      # Dependencias ROS2
├── setup.py                         # Configuración paquete
└── README.md                        # Este archivo
```

## Contribuir

1. Fork el repositorio
2. Crea una rama para tu feature (`git checkout -b feature/nueva-funcionalidad`)
3. Commit tus cambios (`git commit -am 'Agregar nueva funcionalidad'`)
4. Push a la rama (`git push origin feature/nueva-funcionalidad`)
5. Crea un Pull Request

## Licencia

Este proyecto está bajo la licencia MIT. Ver `LICENSE` para más detalles.

## Referencias

- [Crazyswarm2](https://github.com/IMRCLab/crazyswarm2) - Framework principal
- [Crazyflie Simulation](https://github.com/bitcraze/crazyflie-simulation) - Simulación
- [Crazyflie ROS2 Multiranger](https://github.com/knmcguire/crazyflie_ros2_multiranger) - Multiranger
- [Documentación Crazyswarm2](https://imrclab.github.io/crazyswarm2/)

## Soporte

Para problemas y preguntas:
- Abre un Issue en GitHub
- Consulta la documentación de Crazyswarm2
- Únete a la comunidad de Bitcraze
