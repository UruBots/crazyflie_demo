#!/bin/bash

# Script para diagnosticar y corregir problemas de acceso USB con Crazyradio

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

print_status() {
    echo -e "${GREEN}$1${NC}"
}

print_warning() {
    echo -e "${YELLOW}$1${NC}"
}

print_error() {
    echo -e "${RED}$1${NC}"
}

echo "=========================================="
echo "DIAGNÓSTICO Y FIX DE ACCESO USB"
echo "=========================================="
echo ""

# 1. Verificar dispositivo
echo "1. Verificando dispositivo USB..."
if lsusb | grep -q "1915:7777"; then
    print_status "   ✅ Dispositivo Crazyradio detectado"
    lsusb | grep "1915:7777"
else
    print_error "   ❌ Dispositivo Crazyradio NO detectado"
    echo "   Asegúrate de que esté conectado físicamente"
    exit 1
fi

# 2. Detener procesos que puedan estar usando el USB
echo ""
echo "2. Deteniendo procesos que puedan estar usando el USB..."
pkill -f "crazyflie_server" 2>/dev/null && print_status "   ✅ Procesos detenidos" || print_warning "   ⚠️  No hay procesos corriendo"
pkill -f "simple_crazyflie_server" 2>/dev/null
pkill -f "crazyswarm_server" 2>/dev/null
sleep 2

# 3. Verificar permisos USB
echo ""
echo "3. Verificando permisos USB..."
if [ -w /dev/bus/usb ]; then
    print_status "   ✅ Permisos de escritura en /dev/bus/usb"
else
    print_warning "   ⚠️  Sin permisos de escritura en /dev/bus/usb"
    print_warning "   En Docker privileged mode, esto debería estar OK"
fi

# 4. Verificar si estamos en Docker
echo ""
echo "4. Verificando entorno..."
if [ -f /.dockerenv ] || grep -qa docker /proc/1/cgroup 2>/dev/null; then
    print_status "   ✅ Ejecutándose en contenedor Docker"
    if [ -w /dev/bus/usb ] || [ "$(id -u)" = "0" ]; then
        print_status "   ✅ Permisos adecuados (root o acceso USB)"
    else
        print_warning "   ⚠️  Puede haber problemas de permisos"
    fi
else
    print_warning "   ⚠️  No es un contenedor Docker"
    print_warning "   Verifica que el usuario esté en grupo plugdev:"
    print_warning "   sudo usermod -a -G plugdev \$USER"
fi

# 5. Intentar resetear el dispositivo USB (si es posible)
echo ""
echo "5. Intentando resetear dispositivo USB..."
DEVICE_BUS=$(lsusb | grep "1915:7777" | sed 's/.*Bus \([0-9]*\).*/\1/')
DEVICE_DEV=$(lsusb | grep "1915:7777" | sed 's/.*Device \([0-9]*\).*/\1/')

if [ ! -z "$DEVICE_BUS" ] && [ ! -z "$DEVICE_DEV" ]; then
    DEVICE_PATH="/dev/bus/usb/${DEVICE_BUS}/${DEVICE_DEV}"
    if [ -e "$DEVICE_PATH" ]; then
        print_status "   Dispositivo encontrado en: $DEVICE_PATH"
        # Intentar resetear usando usb_modeswitch o similar si está disponible
        if command -v usb_modeswitch &> /dev/null; then
            print_warning "   (usb_modeswitch no aplicable para Crazyradio)"
        fi
    else
        print_warning "   No se puede acceder directamente al dispositivo"
    fi
fi

# 6. Verificar libusb
echo ""
echo "6. Verificando libusb..."
if python3 -c "import usb.core; dev = usb.core.find(idVendor=0x1915, idProduct=0x7777); print('Dispositivo encontrado' if dev else 'NO encontrado')" 2>&1 | grep -q "encontrado"; then
    print_status "   ✅ libusb puede encontrar el dispositivo"
    
    # Intentar abrir
    if python3 -c "import usb.core; dev = usb.core.find(idVendor=0x1915, idProduct=0x7777); dev.set_configuration(1) if dev else None; print('OK')" 2>&1 | grep -q "OK"; then
        print_status "   ✅ libusb puede abrir el dispositivo"
    else
        print_error "   ❌ libusb NO puede abrir el dispositivo"
        print_error "   Esto puede ser porque:"
        print_error "   - Otro proceso lo está usando"
        print_error "   - Permisos insuficientes"
        print_error "   - El dispositivo necesita ser reconectado"
    fi
else
    print_error "   ❌ libusb NO puede encontrar el dispositivo"
fi

# 7. Verificar backend del servidor
echo ""
echo "7. Verificando configuración del servidor..."
if [ -f /workspace/ros2_ws/src/crazyswarm2/crazyflie/launch/launch.py ]; then
    BACKEND_DEFAULT=$(grep -A 1 "DeclareLaunchArgument('backend'" /workspace/ros2_ws/src/crazyswarm2/crazyflie/launch/launch.py | grep "default_value" | sed "s/.*default_value='\([^']*\)'.*/\1/")
    if [ "$BACKEND_DEFAULT" = "cpp" ]; then
        print_status "   ✅ Backend por defecto: cpp (correcto)"
    else
        print_warning "   ⚠️  Backend por defecto: $BACKEND_DEFAULT"
        print_warning "   Recomendado usar backend='cpp'"
    fi
fi

echo ""
echo "=========================================="
echo "RECOMENDACIONES"
echo "=========================================="
echo ""
echo "1. Si el problema persiste, intenta:"
echo "   - Desconectar y reconectar físicamente el USB"
echo "   - Reiniciar el contenedor Docker"
echo "   - Verificar que no haya otros procesos usando el USB"
echo ""
echo "2. Para probar el servidor:"
echo "   ros2 launch crazyflie_dual_control dual_crazyflies.launch.py backend:=cpp"
echo ""
echo "3. Si usas Docker, verifica en docker-compose.yml:"
echo "   - privileged: true"
echo "   - devices: - /dev/bus/usb:/dev/bus/usb"
echo ""

