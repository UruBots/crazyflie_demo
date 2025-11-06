#!/bin/bash

# Script de instalación para Crazyflie Dual Control
# Instala todas las dependencias necesarias para controlar 2 Crazyflies

set -e

echo "=========================================="
echo "Instalación Crazyflie Dual Control"
echo "=========================================="

# Colores para output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Función para imprimir mensajes
print_status() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Verificar que estamos en Ubuntu
if ! command -v apt-get &> /dev/null; then
    print_error "Este script está diseñado para Ubuntu/Debian"
    exit 1
fi

# Verificar ROS2
if ! command -v ros2 &> /dev/null; then
    print_error "ROS2 no está instalado. Por favor instala ROS2 Jazzy primero:"
    echo "https://docs.ros.org/en/jazzy/Installation.html"
    exit 1
fi

# Verificar versión de ROS2
ROS_VERSION=$(ros2 --version 2>/dev/null | head -n1 || echo "ROS2 detectado")
print_status "ROS2 detectado: $ROS_VERSION"

# Detectar workspace (usar el que esté disponible)
if [ -d "/workspace/ros2_ws" ]; then
    WORKSPACE_DIR="/workspace/ros2_ws"
    print_status "Usando workspace: $WORKSPACE_DIR"
elif [ -d "$HOME/ros2_ws" ]; then
    WORKSPACE_DIR="$HOME/ros2_ws"
    print_status "Usando workspace: $WORKSPACE_DIR"
else
    # Si no existe ninguno, crear en el directorio actual o en home
    if [ -d "/workspace" ]; then
        WORKSPACE_DIR="/workspace/ros2_ws"
    else
        WORKSPACE_DIR="$HOME/ros2_ws"
    fi
    print_status "Creando workspace ROS2 en: $WORKSPACE_DIR"
    mkdir -p "$WORKSPACE_DIR/src"
fi

if [ ! -d "$WORKSPACE_DIR/src" ]; then
    print_status "Creando directorio src..."
    mkdir -p "$WORKSPACE_DIR/src"
fi

cd "$WORKSPACE_DIR/src"
print_status "Directorio de trabajo: $(pwd)"

# Instalar Crazyswarm2
print_status "Instalando Crazyswarm2..."
if [ ! -d "crazyswarm2" ]; then
    git clone https://github.com/IMRCLab/crazyswarm2 --recursive
else
    print_warning "Crazyswarm2 ya existe, actualizando..."
    cd crazyswarm2
    git pull
    git submodule update --recursive
    cd ..
fi

# Instalar paquetes adicionales
print_status "Instalando paquetes adicionales..."

# Asegurarse de estar en el directorio correcto
cd "$WORKSPACE_DIR/src"

# Crazyflie simulation
if [ ! -d "crazyflie-simulation" ]; then
    print_status "Clonando crazyflie-simulation..."
    git clone https://github.com/bitcraze/crazyflie-simulation.git
    print_status "✅ crazyflie-simulation clonado"
else
    print_warning "crazyflie-simulation ya existe"
fi

# Crazyflie multiranger
if [ ! -d "crazyflie_ros2_multiranger" ]; then
    print_status "Clonando crazyflie_ros2_multiranger..."
    git clone https://github.com/knmcguire/crazyflie_ros2_multiranger.git
    print_status "✅ crazyflie_ros2_multiranger clonado"
else
    print_warning "crazyflie_ros2_multiranger ya existe"
fi

# Crazyflie ros_gz_crazyflie
if [ ! -d "ros_gz_crazyflie" ]; then
    print_status "Clonando ros_gz_crazyflie..."
    git clone https://github.com/knmcguire/ros_gz_crazyflie.git
    print_status "✅ ros_gz_crazyflie clonado"
else
    print_warning "ros_gz_crazyflie ya existe"
fi

# Verificar que todos los repositorios necesarios estén presentes
print_status "Verificando repositorios clonados..."
required_repos=("crazyswarm2" "crazyflie-simulation" "crazyflie_ros2_multiranger" "ros_gz_crazyflie")
all_repos_ok=true
for repo in "${required_repos[@]}"; do
    if [ -d "$repo" ]; then
        print_status "✅ $repo encontrado"
    else
        print_error "❌ $repo NO encontrado"
        all_repos_ok=false
    fi
done

if [ "$all_repos_ok" = false ]; then
    print_error "Algunos repositorios requeridos no se encontraron"
    print_warning "El script continuará, pero la compilación puede fallar"
fi

# Verificar carpetas clonadas
print_status "Listando carpetas clonadas..."
cd "$WORKSPACE_DIR/src"
ls -la

# Instalar dependencias del sistema
print_status "Instalando dependencias del sistema..."
sudo apt-get update
sudo apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-argcomplete \
    build-essential \
    git \
    curl \
    wget \
    nano \
    vim \
    python3-usb \
    usbutils \
    udev \
    python3-numpy \
    python3-scipy \
    python3-matplotlib \
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-motion-capture-tracking


# Instalar dependencias Python
print_status "Instalando dependencias Python..."
print_status "Usando paquetes del sistema (apt-get) para evitar conflictos..."

# Instalar paquetes Python adicionales disponibles
sudo apt-get install -y \
    python3-serial \
    python3-yaml \
    python3-tk \
    python3-setuptools \
    python3-wheel \
    python3-pip

# Instalar dependencias Python adicionales (nicegui, rowan)
# Estas son necesarias para el GUI de Crazyswarm2 (opcional, solo si usas gui: 'True')
print_status "Instalando dependencias Python adicionales (nicegui, rowan)..."
print_status "Nota: Estas son opcionales, solo necesarias si usas el GUI de Crazyswarm2"

# Verificar NumPy inicial (será corregido después de instalar cflib si es necesario)
print_status "Verificando versión de NumPy..."

# Verificar versión compatible de nicegui (necesita Tailwind, disponible en versiones < 2.0)
install_nicegui=false
if ! python3 -c "from nicegui import Tailwind" 2>/dev/null; then
    print_status "Instalando nicegui (versión compatible con Tailwind)..."
    # Crazyswarm2 requiere nicegui < 2.0 para tener Tailwind
    if pip3 install --user --break-system-packages "nicegui<2.0" 2>&1 | grep -q "externally-managed-environment"; then
        pip3 install --break-system-packages "nicegui<2.0" || true
    else
        pip3 install --user "nicegui<2.0" || true
    fi
    install_nicegui=true
fi

if ! python3 -c "import rowan" 2>/dev/null; then
    print_status "Instalando rowan..."
    if pip3 install --user --break-system-packages rowan 2>&1 | grep -q "externally-managed-environment"; then
        pip3 install --break-system-packages rowan || true
    else
        pip3 install --user rowan || true
    fi
fi

# Instalar cflib y transforms3d (requeridos para el backend cflib de Crazyswarm2)
print_status "Instalando cflib y transforms3d (backend cflib)..."
if ! python3 -c "import cflib" 2>/dev/null; then
    print_status "Instalando cflib..."
    # Intentar primero con --ignore-installed para evitar conflictos con paquetes del sistema
    if ! pip3 install --break-system-packages --ignore-installed packaging cflib 2>/dev/null; then
        # Si falla, intentar sin --ignore-installed
        if pip3 install --user --break-system-packages cflib 2>&1 | grep -q "externally-managed-environment"; then
            pip3 install --break-system-packages cflib || true
        else
            pip3 install --user cflib || true
        fi
    fi
fi

if ! python3 -c "import transforms3d" 2>/dev/null; then
    print_status "Instalando transforms3d..."
    if pip3 install --user --break-system-packages transforms3d 2>&1 | grep -q "externally-managed-environment"; then
        pip3 install --break-system-packages transforms3d || true
    else
        pip3 install --user transforms3d || true
    fi
fi

# Instalar cffirmware (requerido para simulación con crazyflie_sim)
print_status "Instalando cffirmware (bindings Python del firmware para simulación)..."
FIRMWARE_DIR="$HOME/crazyflie-firmware"
if [ ! -d "$FIRMWARE_DIR" ]; then
    print_status "Clonando crazyflie-firmware..."
    cd "$HOME"
    git clone --branch 2025.02 --single-branch --recursive https://github.com/bitcraze/crazyflie-firmware.git || \
    git clone --recursive https://github.com/bitcraze/crazyflie-firmware.git
    FIRMWARE_DIR="$HOME/crazyflie-firmware"
fi

if [ -d "$FIRMWARE_DIR" ]; then
    cd "$FIRMWARE_DIR"
    if ! python3 -c "import cffirmware" 2>/dev/null; then
        print_status "Compilando bindings Python del firmware..."
        # Asegurar que tenemos las dependencias de compilación
        if ! command -v make &> /dev/null; then
            sudo apt-get update && sudo apt-get install -y build-essential || true
        fi
        
        # Configurar y compilar los bindings
        make cf2_defconfig 2>/dev/null || make defconfig 2>/dev/null || true
        make bindings_python 2>/dev/null || {
            print_warning "No se pudo compilar bindings con 'make bindings_python', intentando método alternativo..."
            cd build
            if [ -f "setup.py" ]; then
                pip3 install --user --break-system-packages . || pip3 install --break-system-packages . || true
            fi
        }
        
        # Agregar al PYTHONPATH si se instaló localmente
        if [ -d "$FIRMWARE_DIR/build" ]; then
            export PYTHONPATH="$FIRMWARE_DIR/build:$PYTHONPATH"
            # Agregar al .bashrc para que persista
            if ! grep -q "crazyflie-firmware/build" ~/.bashrc 2>/dev/null; then
                echo "export PYTHONPATH=\"\$HOME/crazyflie-firmware/build:\$PYTHONPATH\"" >> ~/.bashrc
            fi
            print_status "✅ cffirmware instalado, PYTHONPATH actualizado"
        fi
    else
        print_status "✅ cffirmware ya está instalado"
    fi
else
    print_warning "⚠️  No se pudo instalar cffirmware. La simulación puede no funcionar."
fi

# Verificar instalación
if python3 -c "from nicegui import Tailwind" 2>/dev/null; then
    print_status "✅ nicegui con Tailwind instalado"
elif python3 -c "import nicegui" 2>/dev/null; then
    print_warning "⚠️  nicegui instalado pero Tailwind no disponible"
    print_warning "   Esto puede causar problemas si usas el GUI"
else
    print_warning "⚠️  nicegui no se pudo instalar (puedes ignorarlo si no usas el GUI)"
fi

if python3 -c "import rowan" 2>/dev/null; then
    print_status "✅ rowan instalado"
else
    print_warning "⚠️  rowan no se pudo instalar"
fi

if python3 -c "import cflib" 2>/dev/null; then
    print_status "✅ cflib instalado"
else
    print_warning "⚠️  cflib no se pudo instalar (necesario para backend cflib)"
fi

if python3 -c "import transforms3d" 2>/dev/null; then
    print_status "✅ transforms3d instalado"
else
    print_warning "⚠️  transforms3d no se pudo instalar (necesario para backend cflib)"
fi

# Forzar NumPy < 2.0 después de instalar cflib (cflib intenta instalar NumPy 2.2+)
# matplotlib necesita NumPy < 2.0 para funcionar correctamente
print_status "Asegurando compatibilidad de NumPy con matplotlib (NumPy < 2.0)..."
CURRENT_NUMPY_VERSION=$(python3 -c "import numpy; print(numpy.__version__)" 2>/dev/null || echo "0.0.0")
NUMPY_MAJOR=$(echo "$CURRENT_NUMPY_VERSION" | cut -d. -f1)
if [ "$NUMPY_MAJOR" -ge 2 ] 2>/dev/null; then
    print_status "NumPy $CURRENT_NUMPY_VERSION detectado, downgrade a < 2.0 requerido para matplotlib..."
    if pip3 install --break-system-packages --force-reinstall "numpy<2.0" 2>&1 | grep -q "externally-managed-environment"; then
        pip3 install --break-system-packages --force-reinstall "numpy<2.0" || true
    else
        pip3 install --user --force-reinstall "numpy<2.0" || true
    fi
    print_status "✅ NumPy downgradeado para compatibilidad con matplotlib"
else
    print_status "✅ NumPy $CURRENT_NUMPY_VERSION está en versión compatible (< 2.0)"
fi

# Inicializar rosdep
print_status "Inicializando rosdep..."
if ! sudo rosdep init 2>/dev/null; then
    print_warning "rosdep ya está inicializado (esto es normal)"
else
    print_status "✅ rosdep inicializado"
fi

if rosdep update; then
    print_status "✅ rosdep actualizado"
else
    print_warning "Error actualizando rosdep (puede ser límite de tasa de GitHub)"
    print_warning "Puedes continuar, pero algunas dependencias podrían no instalarse automáticamente"
fi

# Instalar dependencias ROS2
print_status "Instalando dependencias ROS2..."
rosdep install --from-paths . --ignore-src -r -y

# Compilar workspace
print_status "Compilando workspace..."
cd "$WORKSPACE_DIR"

# Función para limpiar y recompilar un paquete problemático
fix_symlink_issue() {
    local pkg=$1
    print_status "Corrigiendo problema de symlink en $pkg..."
    rm -rf "$WORKSPACE_DIR/build/$pkg" "$WORKSPACE_DIR/install/$pkg"
    colcon build --packages-select "$pkg" --symlink-install
}

# Intentar compilar todo el workspace
if colcon build --symlink-install 2>&1 | tee /tmp/colcon_build.log; then
    print_status "✅ Workspace compilado correctamente"
else
    print_warning "Algunos paquetes fallaron en la primera compilación"
    
    # Intentar corregir problemas comunes de symlinks
    if grep -q "failed to create symbolic link.*Is a directory" /tmp/colcon_build.log; then
        print_status "Corrigiendo problemas de symlinks..."
        
        # Extraer nombres de paquetes con problemas de symlink
        problematic_packages=$(grep -o "Failed   <<< [a-zA-Z0-9_]*" /tmp/colcon_build.log | awk '{print $3}' | sort -u)
        
        for pkg in $problematic_packages; do
            if [ ! -z "$pkg" ]; then
                fix_symlink_issue "$pkg"
            fi
        done
        
        # Recompilar todo de nuevo
        print_status "Recompilando workspace completo..."
        if colcon build --symlink-install; then
            print_status "✅ Workspace compilado correctamente después de correcciones"
        else
            colcon build --cmake-args -DBUILD_TESTING=ON --symlink-install
            print_status "✅ Workspace compilado correctamente después de correcciones"
        fi
    else
        print_error "Error compilando workspace"
        print_warning "Revisa los logs en $WORKSPACE_DIR/log/"
    fi
    
    print_warning "Puedes intentar compilar paquetes individuales más tarde"
    print_warning "Ejemplo: colcon build --packages-select crazyflie_dual_control"
fi

# Configurar permisos USB
print_status "Configurando permisos USB..."
# Obtener el usuario actual de manera segura
CURRENT_USER=${USER:-$(whoami)}
# Verificar si el usuario está en el grupo antes de agregarlo
if ! groups $CURRENT_USER 2>/dev/null | grep -q "plugdev"; then
    sudo usermod -a -G plugdev $CURRENT_USER
    print_status "Usuario agregado al grupo plugdev (reinicia la sesión para que tome efecto)"
else
  
    print_warning "Usuario ya está en el grupo plugdev"
fi

# Crear reglas udev para Crazyflie
print_status "Creando reglas udev para Crazyflie..."
# Asegurarse de que el directorio existe
sudo mkdir -p /etc/udev/rules.d/
# Crear archivo de reglas
sudo tee /etc/udev/rules.d/99-crazyflie.rules > /dev/null <<EOF
# Crazyflie
SUBSYSTEM=="usb", ATTRS{idVendor}=="1915", ATTRS{idProduct}=="7777", MODE="0664", GROUP="plugdev"
SUBSYSTEM=="usb", ATTRS{idVendor}=="1915", ATTRS{idProduct}=="0101", MODE="0664", GROUP="plugdev"
EOF

if [ -f /etc/udev/rules.d/99-crazyflie.rules ]; then
    print_status "✅ Reglas udev creadas correctamente"
else
    print_error "❌ Error creando reglas udev"
fi

# Recargar reglas udev
print_status "Recargando reglas udev..."
# Intentar instalar udev si no está disponible (en algunos contenedores)
if ! command -v udevadm &> /dev/null; then
    print_warning "udevadm no encontrado, intentando instalar udev..."
    if sudo apt-get install -y udev 2>/dev/null; then
        print_status "✅ udev instalado"
    else
        print_warning "No se pudo instalar udev (puede ser un contenedor Docker)"
        print_warning "Las reglas udev se aplicarán después de reiniciar o reconectar dispositivos"
    fi
fi

# Intentar recargar reglas si udevadm está disponible
if command -v udevadm &> /dev/null; then
    if sudo udevadm control --reload-rules 2>/dev/null; then
        print_status "✅ Reglas udev recargadas"
    else
        print_warning "Error recargando reglas udev (puede requerir reinicio)"
    fi
    
    if sudo udevadm trigger 2>/dev/null; then
        print_status "✅ Dispositivos udev actualizados"
    else
        print_warning "Error actualizando dispositivos udev"
    fi
else
    print_warning "udevadm no está disponible"
    print_warning "En contenedores Docker, las reglas udev pueden no aplicarse hasta reiniciar"
    print_warning "O ejecuta manualmente en el host:"
    print_warning "  sudo udevadm control --reload-rules"
    print_warning "  sudo udevadm trigger"
fi

# Crear script de setup
print_status "Creando script de setup..."
cat > "$WORKSPACE_DIR/setup_crazyflie.sh" <<EOF
#!/bin/bash
# Script para configurar el entorno Crazyflie

# Source ROS2
source /opt/ros/jazzy/setup.bash

# Source workspace
source $WORKSPACE_DIR/install/setup.bash

# Configurar variables de entorno
export GZ_SIM_RESOURCE_PATH="$WORKSPACE_DIR/src/crazyflie-simulation/simulator_files/gazebo/"

echo "Entorno Crazyflie configurado!"
echo "Workspace: $WORKSPACE_DIR"
echo "Para usar: source $WORKSPACE_DIR/setup_crazyflie.sh"
EOF

chmod +x "$WORKSPACE_DIR/setup_crazyflie.sh"

# Crear alias en bashrc
print_status "Configurando alias..."
if ! grep -q "crazyflie" "$HOME/.bashrc"; then
    echo "" >> "$HOME/.bashrc"
    echo "# Crazyflie Dual Control" >> "$HOME/.bashrc"
    echo "alias crazyflie_setup='source $WORKSPACE_DIR/setup_crazyflie.sh'" >> "$HOME/.bashrc"
    echo "alias crazyflie_ws='cd $WORKSPACE_DIR'" >> "$HOME/.bashrc"
fi

# Crear directorio de configuración
print_status "Creando directorio de configuración..."
mkdir -p "$HOME/.crazyflie_dual_control"

# Crear archivo de configuración por defecto
cat > "$HOME/.crazyflie_dual_control/config.yaml" <<EOF
# Configuración por defecto para Crazyflie Dual Control

drones:
  cf1:
    uri: "radio://0/80/2M/E7E7E7E701"
    channel: 80
    initial_position: [0.0, 0.0, 0.0]
    
  cf2:
    uri: "radio://0/80/2M/E7E7E7E702"
    channel: 80
    initial_position: [1.0, 0.0, 0.0]

flight_params:
  max_velocity: 2.0
  max_acceleration: 1.0
  max_yaw_rate: 3.14159
  battery_threshold: 3.0

simulation:
  enabled: false
  world_file: "empty_world.sdf"
  enable_multiranger: true
  enable_mapping: true
EOF

# Verificar instalación
print_status "Verificando instalación..."
if [ -f "$WORKSPACE_DIR/install/setup.bash" ]; then
    print_status "Workspace compilado correctamente"
else
    print_error "Error compilando workspace"
    exit 1
fi

if [ -f "$WORKSPACE_DIR/setup_crazyflie.sh" ]; then
    print_status "Script de setup creado"
else
    print_error "Error creando script de setup"
fi

print_status "Instalación completada!"
echo ""
echo "=========================================="
echo "Próximos pasos:"
echo "=========================================="
echo "1. Reinicia tu sesión o ejecuta: source ~/.bashrc"
echo "2. Configura el entorno: crazyflie_setup"
echo "3. Conecta tus Crazyflies por USB"
echo "4. Verifica conexión: python3 test_crazyflie_connection.py"
echo "5. Lanza el sistema: ros2 launch crazyflie_dual_control dual_crazyflies.launch.py"
echo ""
echo "Para control manual:"
echo "ros2 run crazyflie_dual_control keyboard_controller"
echo ""
echo "Para simulación:"
echo "ros2 launch crazyflie_dual_control simulation.launch.py"
echo ""
echo "Para verificar la instalación:"
echo "./test_crazyflie_setup.sh"
echo ""
print_status "¡Listo para volar!"
