#!/bin/bash

# Script para configurar permisos USB para Crazyflights
# Ejecutar con sudo

echo "Configurando permisos USB para Crazyflights..."

# Crear regla udev para Crazyflights
cat > /etc/udev/rules.d/99-crazyflie.rules << 'EOF'
# Crazyflie USB rules
SUBSYSTEM=="usb", ATTRS{idVendor}=="1915", ATTRS{idProduct}=="7777", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTRS{idVendor}=="1915", ATTRS{idProduct}=="0101", MODE="0666", GROUP="plugdev"
EOF

# Recargar reglas udev
udevadm control --reload-rules
udevadm trigger

# Agregar usuario al grupo plugdev (si existe)
if getent group plugdev > /dev/null 2>&1; then
    echo "Agregando usuario $USER al grupo plugdev..."
    usermod -a -G plugdev $USER
fi

echo "Configuración USB completada."
echo "Reinicia el contenedor Docker para aplicar los cambios."
echo ""
echo "Para verificar dispositivos USB conectados:"
echo "  lsusb | grep 1915"
echo ""
echo "Para verificar permisos:"
echo "  ls -la /dev/bus/usb/*/"
