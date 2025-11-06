# Crazyflie Dual Control - Proyecto de Simulación

Sistema ROS 2 para controlar 2 drones Crazyflie simultáneamente en simulación usando Gazebo Sim y RViz. Incluye capacidades de mapeo, navegación autónoma y monitoreo de batería.

## 📋 Tabla de Contenidos

- [Características](#-características)
- [Requisitos](#-requisitos)
- [Instalación](#-instalación)
- [Configuración](#-configuración)
- [Uso](#-uso)
- [Estructura del Proyecto](#-estructura-del-proyecto)
- [Solución de Problemas](#-solución-de-problemas)

## ✨ Características

- ✅ **Control simultáneo de 2 Crazyflies** (cf1 y cf2)
- ✅ **Simulación completa** con Gazebo Sim y RViz
- ✅ **Sincronización TF ↔ Gazebo** para visualización precisa
- ✅ **Multiranger** para detección de obstáculos
- ✅ **Mapeo simultáneo** con múltiples drones
- ✅ **Navegación autónoma** con seguimiento de paredes
- ✅ **Monitoreo de batería** en tiempo real
- ✅ **Control manual** con teclado
- ✅ **Controlador dual** con servicios ROS 2

## 🔧 Requisitos

### Software

- **ROS 2 Jazzy** (Ubuntu 22.04+)
- **Gazebo Sim** (incluido con ROS 2 Jazzy)
- **RViz2** (incluido con ROS 2 Jazzy)
- **Python 3.10+**
- **Crazyswarm2** (framework para control de Crazyflies)
- **cffirmware** (firmware de Crazyflie para simulación)

### Dependencias ROS 2

```bash
sudo apt update
sudo apt install -y \
    ros-jazzy-rviz2 \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge \
    python3-pip \
    python3-colcon-common-extensions
```

## 📦 Instalación

### 1. Clonar el Workspace

```bash
cd /workspace/ros2_ws/src
```

### 2. Instalar Crazyswarm2

```bash
# Clonar Crazyswarm2
git clone https://github.com/IMRCLab/crazyswarm2 --recursive
cd crazyswarm2

# Instalar dependencias del sistema
rosdep update
rosdep install --from-paths . --ignore-src -r -y

# Compilar
cd /workspace/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 3. Instalar Firmware de Crazyflie

El simulador requiere el firmware compilado de Crazyflie:

```bash
# Clonar firmware
cd ~
git clone https://github.com/bitcraze/crazyflie-firmware.git
cd crazyflie-firmware

# Compilar
make

# Verificar que existe el build
ls -la ~/crazyflie-firmware/build
```

### 4. Instalar Modelos de Gazebo (Opcional)

Si usas modelos personalizados de Gazebo:

```bash
cd /workspace/ros2_ws/src
# Clonar repositorio de modelos (si aplica)
git clone <tu-repositorio-de-modelos>
```

### 5. Compilar el Proyecto

```bash
cd /workspace/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## ⚙️ Configuración

### Archivo de Configuración Principal

El archivo `config/crazyflies_simulation.yaml` contiene la configuración de los drones:

```yaml
robots:
  cf1:
    enabled: true
    uri: sim://cf1
    initial_position: [0.0, 0.0, 0.05]
    type: cf21
    
  cf2:
    enabled: true
    uri: sim://cf2
    initial_position: [1.0, 0.0, 0.05]
    type: cf21

all:
  reference_frame: "world"
  
sim:
  backend: np
  controller: mellinger
  visualizations:
    rviz:
      enabled: true
```

### Variables de Entorno DDS

El proyecto está configurado para evitar problemas de comunicación DDS en contenedores:

- `RMW_FASTRTPS_USE_SHM=0`: Deshabilita Shared Memory de FastDDS
- `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`: Usa CycloneDDS como middleware

Estas variables se configuran automáticamente en el launch file.

## 🚀 Uso

### Ejecutar Simulación Completa

```bash
cd /workspace/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Lanzar simulación con todos los componentes
ros2 launch crazyflie_dual_control simulation.launch.py
```

Esto iniciará:
- **Gazebo Sim** con mundo vacío
- **Servidor Crazyflie** (simulación con backend `np`)
- **2 drones** spawneados en Gazebo (cf1 y cf2)
- **Sincronizador de poses** (TF → Gazebo)
- **Controlador dual** para comandos de vuelo
- **Mappers** para cada drone
- **Wall followers** para navegación autónoma
- **Monitor de batería**
- **RViz2** para visualización

### Opciones de Lanzamiento

```bash
# Deshabilitar RViz
ros2 launch crazyflie_dual_control simulation.launch.py enable_rviz:=false

# Deshabilitar mapeo
ros2 launch crazyflie_dual_control simulation.launch.py enable_mapping:=false

# Deshabilitar multiranger
ros2 launch crazyflie_dual_control simulation.launch.py enable_multiranger:=false

# Combinar opciones
ros2 launch crazyflie_dual_control simulation.launch.py \
    enable_rviz:=true \
    enable_mapping:=true \
    enable_multiranger:=true
```

### Verificar que Todo Funciona

#### 1. Verificar Tópicos ROS 2

```bash
# Listar tópicos
ros2 topic list

# Verificar transforms
ros2 run tf2_ros tf2_echo world cf1
ros2 run tf2_ros tf2_echo world cf2

# Verificar robot_description
ros2 topic echo /cf1/robot_description --once
ros2 topic echo /cf2/robot_description --once
```

#### 2. Verificar en Gazebo

```bash
# Listar modelos en Gazebo
gz model --list

# Ver pose de un modelo
gz model --pose -m cf1
gz model --pose -m cf2
```

#### 3. Verificar en RViz

- Abre RViz (se abre automáticamente con el launch)
- Verifica que los frames `cf1` y `cf2` aparezcan en el árbol TF
- Verifica que los modelos de los drones sean visibles

### Comandos de Control

#### Usar Servicios ROS 2

```bash
# Despegar ambos drones
ros2 service call /cf1/takeoff crazyflie_interfaces/srv/Takeoff "{height: 0.5, duration: {sec: 0, nanosec: 0}}"
ros2 service call /cf2/takeoff crazyflie_interfaces/srv/Takeoff "{height: 0.5, duration: {sec: 0, nanosec: 0}}"

# Ir a una posición
ros2 service call /cf1/go_to crazyflie_interfaces/srv/GoTo "{goal: {x: 1.0, y: 0.0, z: 0.5, yaw: 0.0}, relative: false, duration: {sec: 2, nanosec: 0}}"

# Aterrizar
ros2 service call /cf1/land crazyflie_interfaces/srv/Land "{height: 0.0, duration: {sec: 0, nanosec: 0}}"
```

#### Publicar Comandos de Posición

```bash
# Comando de posición para cf1
ros2 topic pub /cf1/cmd_position crazyflie_interfaces/msg/Position \
  "{x: 1.0, y: 0.5, z: 0.5, yaw: 1.57}" --once
```

## 📁 Estructura del Proyecto

```
ros2_ws/
├── src/
│   ├── crazyflie_dual_control/
│   │   ├── config/
│   │   │   ├── crazyflies_simulation.yaml  # Configuración de simulación
│   │   │   └── crazyflies.yaml              # Configuración para hardware real
│   │   ├── launch/
│   │   │   ├── simulation.launch.py         # Launch file principal de simulación
│   │   │   └── dual_crazyflies.launch.py     # Launch file para hardware real
│   │   ├── rviz/
│   │   │   └── dual_crazyflies_simulation.rviz  # Configuración de RViz
│   │   ├── crazyflie_dual_control/
│   │   │   ├── dual_controller.py           # Controlador principal
│   │   │   ├── gazebo_pose_sync.py          # Sincronizador TF → Gazebo
│   │   │   ├── battery_monitor.py           # Monitor de batería
│   │   │   ├── keyboard_controller.py       # Control con teclado
│   │   │   ├── simple_flight.py             # Vuelo automático simple
│   │   │   └── advanced_controller.py       # Controlador avanzado
│   │   └── package.xml
│   ├── crazyswarm2/                         # Framework Crazyswarm2
│   ├── crazyflie/                            # Paquete base de Crazyflie
│   ├── crazyflie_sim/                        # Simulador de Crazyflie
│   └── ...
└── install/
```

## 🔍 Solución de Problemas

### RViz no se abre

**Error**: `libOgreMain.so.1.12.10: cannot open shared object file`

**Solución**:
```bash
sudo apt install -y ros-jazzy-rviz2 libogre-1.12-dev
```

### Drones no aparecen en Gazebo

**Verificar**:
1. Que el modelo SDF existe:
   ```bash
   ls -la /workspace/ros2_ws/src/ros_gz_crazyflie/ros_gz_crazyflie_gazebo/models/crazyflie/model.sdf
   ```

2. Que Gazebo encontró los recursos:
   ```bash
   echo $GZ_SIM_RESOURCE_PATH
   ```

3. Revisar logs del launch:
   ```bash
   ros2 launch crazyflie_dual_control simulation.launch.py 2>&1 | grep -i "spawn\|model\|error"
   ```

### Drones no se mueven en Gazebo

**Causa**: Los plugins de física pueden interferir con `set_pose`.

**Solución**: El launch file ya remueve automáticamente los plugins problemáticos. Si persiste:
1. Verificar que `gazebo_pose_sync` está corriendo:
   ```bash
   ros2 node list | grep gazebo_pose_sync
   ```

2. Verificar que hay transforms:
   ```bash
   ros2 run tf2_ros tf2_echo world cf1
   ```

### Errores de DDS (Shared Memory)

**Error**: `RTPS_TRANSPORT_SHM Error: Failed init_port`

**Solución**: Ya está configurado en el launch file. Si persiste:
```bash
export RMW_FASTRTPS_USE_SHM=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

### Servicio `arm` no disponible

**Nota**: En simulación, el servicio `arm` no está disponible. Esto es normal. El controlador maneja esto automáticamente y usa `takeoff` directamente.

### Transformaciones TF no aparecen

**Verificar**:
1. Que el servidor está corriendo:
   ```bash
   ros2 node list | grep crazyflie_server
   ```

2. Que `reference_frame` está configurado correctamente en `crazyflies_simulation.yaml`:
   ```yaml
   all:
     reference_frame: "world"
   ```

3. Verificar el árbol TF:
   ```bash
   ros2 run tf2_tools view_frames
   evince frames.pdf
   ```

### Gazebo no encuentra meshes

**Error**: `Unable to find uri[meshes/cf2_assembly.dae]`

**Solución**: El launch file convierte automáticamente rutas relativas a absolutas. Si persiste:
1. Verificar que `GZ_SIM_RESOURCE_PATH` incluye la ruta correcta
2. Verificar que los archivos de mesh existen:
   ```bash
   find /workspace/ros2_ws -name "*.dae" -type f
   ```

### El servidor no inicia

**Error**: `No module named 'cffirmware'`

**Solución**: Asegúrate de que el firmware está compilado y en el PYTHONPATH:
```bash
# Verificar que existe
ls -la ~/crazyflie-firmware/build

# El launch file configura PYTHONPATH automáticamente
```

## 📝 Notas Adicionales

### Sincronización TF ↔ Gazebo

El nodo `gazebo_pose_sync` lee las transformaciones TF (`world → cfX`) y actualiza las poses de los modelos en Gazebo usando `ros2 run ros_gz_sim set_entity_pose`. Esto permite que:

- Los drones se muevan según los comandos del controlador
- La visualización en Gazebo coincida con RViz
- El rate de actualización es configurable (por defecto: 15 Hz)

### Backend de Simulación

El proyecto usa el backend `np` (numpy) de `crazyflie_sim`, que:
- No integra directamente con Gazebo para física
- Publica TF transforms para RViz
- Permite control manual de poses en Gazebo

### Modificaciones del SDF

El launch file modifica dinámicamente el SDF del modelo:
- Actualiza nombres de modelos y namespaces
- Convierte rutas de meshes a absolutas
- Remueve plugins de física que interfieren
- Actualiza referencias de joints y links

## 🤝 Contribuir

Para contribuir al proyecto:
1. Fork el repositorio
2. Crea una rama para tu feature
3. Realiza tus cambios
4. Envía un pull request

## 📄 Licencia

MIT License - Ver archivo `LICENSE` para más detalles.

## 📧 Contacto

Para preguntas o soporte, abre un issue en el repositorio.

---

**Última actualización**: 2024

