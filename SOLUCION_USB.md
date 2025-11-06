# Solución para el problema de acceso USB con Crazyradio

## Problema identificado

El servidor C++ de Crazyswarm2 reporta "No Crazyradio dongle found!" aunque:
- El dispositivo se detecta con `lsusb` ✅
- libusb puede encontrar el dispositivo ✅
- **PERO libusb NO puede abrir el dispositivo** ❌

## Causas posibles

1. **Dispositivo ocupado por otro proceso**: Aunque no se detectan procesos activos, puede haber un proceso zombie o un descriptor abierto
2. **Problema de permisos en Docker**: Aunque estamos en modo privilegiado, puede haber un problema de acceso
3. **El dispositivo necesita ser reconectado**: El USB puede estar en un estado inconsistente
4. **Problema con el backend**: El launch file puede estar usando el backend incorrecto

## Soluciones

### Solución 1: Reiniciar el contenedor Docker (RECOMENDADO)

```bash
# Detener el contenedor
docker-compose down

# Desconectar y reconectar físicamente el USB

# Reiniciar el contenedor
docker-compose up -d
```

### Solución 2: Verificar y forzar el backend C++

Asegúrate de que el launch file use explícitamente el backend C++:

```bash
ros2 launch crazyflie_dual_control dual_crazyflies.launch.py backend:=cpp
```

Verifica en el log que se ejecute `crazyflie_server` (no `crazyflie_server.py`).

### Solución 3: Verificar permisos USB en el host

En el **host** (no en el contenedor), verifica permisos:

```bash
# En el host
ls -la /dev/bus/usb/003/010
sudo chmod 666 /dev/bus/usb/003/010  # Si es necesario
```

### Solución 4: Resetear el dispositivo USB desde el host

```bash
# En el host, encontrar el dispositivo
lsusb | grep 1915:7777

# Usar usbreset si está disponible
sudo apt-get install usbutils
sudo usbreset /dev/bus/usb/003/010
```

### Solución 5: Verificar que no haya procesos Python bloqueando

```bash
# En el contenedor
ps aux | grep -E "python.*crazyflie|cflib"
pkill -9 -f "crazyflie.*python"
```

## Verificación del backend

El launch file de Crazyswarm2 selecciona el backend así:

```python
# Backend Python (cflib)
Node(
    executable='crazyflie_server.py',
    condition=LaunchConfigurationEquals('backend','cflib'),
)

# Backend C++ (cpp) - RECOMENDADO
Node(
    executable='crazyflie_server',
    condition=LaunchConfigurationEquals('backend','cpp'),
)
```

## Estado actual

- ✅ Dispositivo detectado físicamente
- ✅ Permisos Docker OK (privileged mode)
- ❌ **libusb no puede abrir el dispositivo** ← PROBLEMA PRINCIPAL

## Próximos pasos

1. **Reinicia el contenedor Docker** (esto suele resolver el problema)
2. **Reconecta físicamente el USB** si es posible
3. **Verifica que uses `backend:=cpp`** explícitamente
4. **Revisa los logs del servidor** para confirmar qué backend se está usando

## Comandos útiles

```bash
# Diagnosticar USB
/workspace/fix_usb_access.sh

# Verificar procesos
ps aux | grep crazyflie

# Lanzar con backend explícito
ros2 launch crazyflie_dual_control dual_crazyflies.launch.py backend:=cpp

# Verificar qué ejecutable se usa
ps aux | grep crazyflie_server
```

