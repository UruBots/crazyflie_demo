#!/bin/bash

# Script para instalar nicegui y rowan necesarios para Crazyswarm2 GUI

echo "=========================================="
echo "Instalando nicegui y rowan"
echo "=========================================="

# Primero arreglar NumPy (matplotlib necesita NumPy < 2.0)
echo "Instalando NumPy compatible (versión < 2.0)..."
pip3 install --user --break-system-packages "numpy<2" 2>&1 | grep -v "NumPy\|Warning" || \
pip3 install --break-system-packages "numpy<2" 2>&1 | grep -v "NumPy\|Warning" || true

# Crazyswarm2 requiere nicegui < 2.0 para tener Tailwind
# Verificar si Tailwind está disponible
if python3 -c "from nicegui import Tailwind" 2>/dev/null; then
    echo "✅ nicegui con Tailwind ya está instalado"
else
    echo "Instalando nicegui (versión < 2.0 para compatibilidad con Tailwind)..."
    pip3 install --user --break-system-packages "nicegui<2.0" 2>&1 | grep -v "NumPy\|Warning" || \
    pip3 install --break-system-packages "nicegui<2.0" 2>&1 | grep -v "NumPy\|Warning" || \
    pip3 install "nicegui<2.0" 2>&1 | grep -v "NumPy\|Warning"
fi

# Verificar si rowan ya está instalado
if python3 -c "import rowan" 2>/dev/null; then
    echo "✅ rowan ya está instalado"
else
    echo "Instalando rowan..."
    pip3 install --user --break-system-packages rowan 2>&1 | grep -v "NumPy\|Warning" || \
    pip3 install --break-system-packages rowan 2>&1 | grep -v "NumPy\|Warning" || \
    pip3 install rowan 2>&1 | grep -v "NumPy\|Warning"
fi

echo ""
echo "=========================================="
echo "Verificación"
echo "=========================================="

if python3 -c "from nicegui import Tailwind" 2>/dev/null; then
    echo "✅ nicegui con Tailwind: OK"
elif python3 -c "import nicegui" 2>/dev/null; then
    echo "⚠️  nicegui instalado pero Tailwind no disponible"
    echo "   Reinstalando nicegui < 2.0..."
    pip3 install --user --break-system-packages --force-reinstall "nicegui<2.0" 2>&1 | grep -v "NumPy\|Warning"
else
    echo "❌ nicegui: Error"
fi

if python3 -c "import rowan" 2>/dev/null; then
    echo "✅ rowan: OK"
else
    echo "❌ rowan: Error"
fi

echo ""
echo "Nota: nicegui es solo necesario si usas el GUI de Crazyswarm2"
echo "Si solo usas el controlador (gui: 'False'), no es necesario"
