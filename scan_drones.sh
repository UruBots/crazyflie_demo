#!/bin/bash

# Script simple para escanear drones usando el ejecutable scan de Crazyswarm2

echo "=========================================="
echo "ESCANEO DE DRONES CRAZYFLIES"
echo "=========================================="
echo "Asegúrate de que:"
echo "1. La antena Crazyradio PA esté conectada"
echo "2. Los drones estén encendidos"
echo "3. Los drones tengan batería suficiente"
echo "=========================================="
echo ""

# Verificar que el entorno ROS2 esté configurado
if ! command -v ros2 &> /dev/null; then
    echo "❌ ROS2 no está disponible"
    echo "Ejecuta: source /opt/ros/jazzy/setup.bash"
    exit 1
fi

# Source el workspace si existe
if [ -f "/workspace/ros2_ws/install/setup.bash" ]; then
    source /workspace/ros2_ws/install/setup.bash
fi

echo "Escaneando drones (esto puede tardar unos segundos)..."
echo ""

# Ejecutar scan con dirección por defecto
if ros2 run crazyflie scan; then
    echo ""
    echo "=========================================="
    echo "✓ Escaneo completado"
    echo "=========================================="
    echo ""
    echo "Si encontraste URIs, actualiza el archivo:"
    echo "/workspace/ros2_ws/src/crazyflie_dual_control/config/crazyflies.yaml"
else
    echo ""
    echo "❌ Error durante el escaneo"
    echo ""
    echo "Solución de problemas:"
    echo "1. Verifica conexión USB: lsusb | grep 1915"
    echo "2. Verifica permisos: groups \$USER | grep plugdev"
    echo "3. Asegúrate de que los drones estén encendidos"
    exit 1
fi
