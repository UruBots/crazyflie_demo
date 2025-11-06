#!/bin/bash

# Script de diagnóstico para problemas de conexión con drones Crazyflies

echo "=========================================="
echo "DIAGNÓSTICO DE CONEXIÓN CON DRONES"
echo "=========================================="
echo ""

# 1. Verificar USB
echo "1. Verificando conexión USB..."
if lsusb | grep -q "1915"; then
    echo "   ✅ Antena Crazyradio PA detectada"
    lsusb | grep "1915"
else
    echo "   ❌ Antena NO detectada"
    echo "   Conecta la antena por USB"
    exit 1
fi

echo ""
echo "2. Verificando permisos USB..."
if groups | grep -q "plugdev"; then
    echo "   ✅ Usuario en grupo plugdev"
else
    echo "   ⚠️  Usuario NO está en grupo plugdev"
    echo "   Ejecuta: sudo usermod -a -G plugdev $USER"
    echo "   Luego reinicia la sesión"
fi

# Verificar permisos de dispositivos USB
echo ""
echo "3. Verificando permisos de dispositivos USB..."
USB_DEVICES=$(find /dev/bus/usb -type c 2>/dev/null | head -5)
if [ -n "$USB_DEVICES" ]; then
    for dev in $USB_DEVICES; do
        perms=$(stat -c "%a" "$dev" 2>/dev/null)
        echo "   $dev: permisos $perms"
    done
else
    echo "   ⚠️  No se pueden verificar dispositivos USB"
fi

echo ""
echo "4. Verificando si los drones están encendidos..."
echo "   (Esta verificación requiere que ejecutes el escaneo)"
echo ""

echo "=========================================="
echo "PRUEBAS DE ESCANEO"
echo "=========================================="
echo ""

# Intentar escaneo con timeout
echo "Intentando escaneo rápido (10 segundos)..."
source /opt/ros/jazzy/setup.bash 2>/dev/null
if [ -f "/workspace/ros2_ws/install/setup.bash" ]; then
    source /workspace/ros2_ws/install/setup.bash
fi

timeout 10 ros2 run crazyflie scan 2>&1 || {
    echo ""
    echo "⚠️  El escaneo automático no encontró drones"
    echo ""
    echo "POSIBLES CAUSAS:"
    echo "1. Los drones NO están encendidos"
    echo "2. Los drones NO tienen batería suficiente"
    echo "3. Los drones están demasiado lejos de la antena (>5m)"
    echo "4. Los drones tienen direcciones/canales personalizados"
    echo "5. Problema con el firmware de la antena"
    echo ""
    echo "SOLUCIONES:"
    echo "- Asegúrate de que los drones estén ENCENDIDOS (LEDs activos)"
    echo "- Verifica que tengan batería (mínimo 3.7V)"
    echo "- Coloca los drones cerca de la antena (1-2 metros)"
    echo "- Si conoces las direcciones, edítalas manualmente en crazyflies.yaml"
}

echo ""
echo "=========================================="
echo "INFORMACIÓN DE CONFIGURACIÓN"
echo "=========================================="
echo ""
echo "Las URIs siguen este formato:"
echo "  radio://<radio_id>/<channel>/<datarate>/<address>"
echo ""
echo "Valores comunes:"
echo "  radio_id: 0 (si solo tienes una antena)"
echo "  channel: 80, 90, o 100"
echo "  datarate: 2M, 1M, o 250K"
echo "  address: E7E7E7E701, E7E7E7E702, etc."
echo ""
echo "Ejemplos:"
echo "  radio://0/80/2M/E7E7E7E701"
echo "  radio://0/80/2M/E7E7E7E702"
echo ""
echo "Si conoces las direcciones de tus drones, puedes editarlas en:"
echo "  /workspace/ros2_ws/src/crazyflie_dual_control/config/crazyflies.yaml"
