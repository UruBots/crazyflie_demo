#!/usr/bin/env python3
"""
Script mejorado para escanear Crazyflies con múltiples métodos y configuraciones
"""

import sys
import os
import time
import subprocess

print("="*70)
print("ESCANEO AVANZADO DE CRAZYFLIES")
print("="*70)
print()

# 1. Verificar USB
print("1. Verificando hardware...")
try:
    result = subprocess.run(['lsusb'], capture_output=True, text=True, timeout=5)
    if '1915:7777' in result.stdout:
        print("   ✅ Crazyradio PA detectada")
        for line in result.stdout.split('\n'):
            if '1915' in line:
                print(f"   {line.strip()}")
    else:
        print("   ❌ Crazyradio PA NO detectada")
        sys.exit(1)
except Exception as e:
    print(f"   ❌ Error: {e}")
    sys.exit(1)

print()

# 2. Importar cflib
print("2. Inicializando cflib...")
try:
    import cflib.crtp
    from cflib.crazyflie import Crazyflie
    from cflib.crazyflie.swarm import CachedCfFactory
    from cflib.crazyflie.swarm import Swarm
    print("   ✅ cflib importado")
except ImportError as e:
    print(f"   ❌ Error importando cflib: {e}")
    sys.exit(1)

# Inicializar drivers
try:
    cflib.crtp.init_drivers()
    print("   ✅ Drivers inicializados")
except Exception as e:
    print(f"   ❌ Error inicializando drivers: {e}")
    sys.exit(1)

print()

# 3. Método 1: Escaneo usando cflib.scan
print("3. Método 1: Escaneo con cflib.scan()...")
try:
    from cflib.crtp import scan_interfaces
    
    print("   Escaneando interfaces...")
    interfaces = scan_interfaces()
    
    if interfaces:
        print(f"   ✅ Encontradas {len(interfaces)} interfaces:")
        for interface in interfaces:
            print(f"      {interface}")
    else:
        print("   ⚠️  No se encontraron interfaces")
except Exception as e:
    print(f"   ⚠️  Error en scan_interfaces: {e}")

print()

# 4. Método 2: Escaneo manual con múltiples configuraciones
print("4. Método 2: Escaneo manual con diferentes configuraciones...")
print("   (Esto puede tardar varios minutos...)")
print("   Asegúrate de que el drone esté ENCENDIDO y cerca (< 5m)")
print()

# Configuraciones comunes a probar
channels = [80, 90, 100]  # Canales comunes
addresses = [
    'E7E7E7E7E7',  # Dirección por defecto
    'E7E7E7E7E8',  # Dirección por defecto
    'E7E7E7E701',  # Drone 1
    'E7E7E7E702',  # Drone 2
    'E7E7E7E703',  # Drone 3
    'E7E7E7E704',  # Drone 4
    'E7E7E7E705',  # Drone 5
]
datarates = ['2M', '1M', '250K']  # Velocidades de datos

found_drones = []

print("   Probando combinaciones...")
print("   (Presiona Ctrl+C para cancelar)")
print()

total_tests = len(channels) * len(addresses) * len(datarates)
test_count = 0

try:
    for channel in channels:
        for address in addresses:
            for datarate in datarates:
                uri = f"radio://0/{channel}/{datarate}/{address}"
                test_count += 1
                
                if test_count % 10 == 0:
                    print(f"   Progreso: {test_count}/{total_tests} ({test_count*100//total_tests}%)")
                
                cf = Crazyflie()
                connected = [False]
                error_msg = [None]
                
                def connected_cb(uri_cb):
                    connected[0] = True
                    print(f"   ✅ ¡ENCONTRADO! URI: {uri_cb}")
                    found_drones.append(uri_cb)
                
                def failed_cb(uri_cb, msg):
                    error_msg[0] = msg
                    # No imprimir todos los errores para no saturar la salida
                
                def disconnected_cb(uri_cb):
                    pass
                
                cf.connected.add_callback(connected_cb)
                cf.connection_failed.add_callback(failed_cb)
                cf.disconnected.add_callback(disconnected_cb)
                
                # Intentar conexión rápida (1 segundo máximo por intento)
                cf.open_link(uri)
                
                # Esperar hasta 1 segundo
                for _ in range(10):
                    time.sleep(0.1)
                    if connected[0]:
                        break
                
                cf.close_link()
                time.sleep(0.05)  # Pequeña pausa entre intentos
                
except KeyboardInterrupt:
    print()
    print("   ⚠️  Escaneo cancelado por el usuario")
except Exception as e:
    print(f"   ⚠️  Error durante el escaneo: {e}")

print()

# 5. Método 3: Intentar con ejecutable de Crazyswarm2
print("5. Método 3: Intentando con ejecutable scan de Crazyswarm2...")
try:
    scan_paths = [
        '/workspace/ros2_ws/install/crazyflie/lib/crazyflie/scan',
        '/workspace/ros2_ws/build/crazyflie/deps/crazyflie_tools/src/scan',
    ]
    
    for scan_path in scan_paths:
        if os.path.exists(scan_path):
            print(f"   Probando: {scan_path}")
            try:
                result = subprocess.run(
                    [scan_path],
                    capture_output=True,
                    text=True,
                    timeout=30
                )
                if result.stdout.strip():
                    print(f"   ✅ Salida: {result.stdout.strip()}")
            except subprocess.TimeoutExpired:
                print("   ⚠️  Timeout")
            except Exception as e:
                print(f"   ⚠️  Error: {e}")
except Exception as e:
    print(f"   ⚠️  Error: {e}")

print()

# Resumen
print("="*70)
if found_drones:
    print("✅ DRONES ENCONTRADOS:")
    print()
    for i, uri in enumerate(found_drones, 1):
        print(f"   Drone {i}: {uri}")
    print()
    print("Para usar estos drones, actualiza el archivo:")
    print("  /workspace/ros2_ws/src/crazyflie_ros2_multiranger/crazyflie_ros2_multiranger_bringup/config/crazyflie_real_crazyswarm2.yaml")
    print()
    print("Cambia la URI en la línea 5:")
    print(f"    uri: {found_drones[0]}")
else:
    print("❌ NO SE ENCONTRARON DRONES")
    print()
    print("POSIBLES CAUSAS:")
    print()
    print("1. El drone NO está encendido")
    print("   - Verifica que los LEDs estén encendidos")
    print("   - Verifica la batería (LEDs de batería)")
    print()
    print("2. El drone está fuera de rango")
    print("   - El drone debe estar a menos de 5 metros de la antena")
    print("   - Verifica que no haya obstáculos")
    print()
    print("3. El drone tiene una configuración diferente")
    print("   - Canal diferente (80, 90, 100, etc.)")
    print("   - Dirección diferente (no E7E7E7E7XX)")
    print("   - Velocidad de datos diferente (2M, 1M, 250K)")
    print()
    print("4. Problema con el firmware del drone")
    print("   - Intenta actualizar el firmware del drone")
    print("   - Usa el Crazyflie Client oficial para configurar el drone")
    print()
    print("5. Problema con la antena")
    print("   - Verifica que la antena esté bien conectada")
    print("   - Prueba reconectar la antena USB")
    print()
    print("SOLUCIÓN RECOMENDADA:")
    print("1. Usa el Crazyflie Client oficial (descarga de bitcraze.io)")
    print("2. Conecta el drone por USB y configura:")
    print("   - Dirección: E7E7E7E7XX (donde XX es el número del drone)")
    print("   - Canal: 80 (u otro que prefieras)")
    print("   - Velocidad: 2M")
    print("3. Luego actualiza la URI en el archivo YAML con esos valores")
    print()

print("="*70)

