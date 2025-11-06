#!/usr/bin/env python3
"""
Script rápido de diagnóstico para problemas de conexión con Crazyflie
"""

import sys
import os

# Agregar paths de ROS2
sys.path.insert(0, '/opt/ros/jazzy/lib/python3.12/site-packages')
if os.path.exists('/workspace/ros2_ws/install'):
    sys.path.insert(0, '/workspace/ros2_ws/install')

print("="*70)
print("DIAGNÓSTICO DE CONEXIÓN CON CRAZYFLIE")
print("="*70)
print()

# 1. Verificar USB
print("1. Verificando conexión USB...")
try:
    import subprocess
    result = subprocess.run(['lsusb'], capture_output=True, text=True, timeout=5)
    if '1915:7777' in result.stdout:
        print("   ✅ Crazyradio PA detectada")
        for line in result.stdout.split('\n'):
            if '1915' in line:
                print(f"   {line.strip()}")
    else:
        print("   ❌ Crazyradio PA NO detectada")
        print("   Conecta la antena por USB")
        sys.exit(1)
except Exception as e:
    print(f"   ⚠️  Error: {e}")

print()

# 2. Verificar cflib
print("2. Verificando cflib...")
try:
    import cflib.crtp
    from cflib.crazyflie import Crazyflie
    from cflib.utils import uri_helper
    print("   ✅ cflib disponible")
except ImportError as e:
    print(f"   ❌ cflib no disponible: {e}")
    print("   Ejecuta: pip3 install --break-system-packages cflib")
    sys.exit(1)

print()

# 3. Inicializar drivers
print("3. Inicializando drivers USB...")
try:
    cflib.crtp.init_drivers()
    print("   ✅ Drivers inicializados")
except Exception as e:
    print(f"   ❌ Error inicializando drivers: {e}")
    sys.exit(1)

print()

# 4. Verificar URI configurada
print("4. Verificando URI configurada...")
try:
    import yaml
    config_path = '/workspace/ros2_ws/src/crazyflie_ros2_multiranger/crazyflie_ros2_multiranger_bringup/config/crazyflie_real_crazyswarm2.yaml'
    if os.path.exists(config_path):
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
            if 'robots' in config and 'crazyflie_real' in config['robots']:
                uri = config['robots']['crazyflie_real'].get('uri', 'No definida')
                print(f"   URI configurada: {uri}")
                
                # Intentar conexión
                print()
                print("5. Intentando conexión con el drone...")
                print(f"   Probando URI: {uri}")
                print("   (Esto puede tardar 10-15 segundos...)")
                
                cf = Crazyflie()
                connection_state = {'connected': False, 'failed': False, 'error_msg': None}
                
                def connected_callback(uri_cb):
                    connection_state['connected'] = True
                    print(f"   ✅ Conectado exitosamente a {uri_cb}")
                
                def connection_failed_callback(uri_cb, msg):
                    connection_state['failed'] = True
                    connection_state['error_msg'] = msg
                    print(f"   ❌ Conexión fallida: {msg}")
                
                def disconnected_callback(uri_cb):
                    print(f"   ⚠️  Desconectado de {uri_cb}")
                
                cf.connected.add_callback(connected_callback)
                cf.connection_failed.add_callback(connection_failed_callback)
                cf.disconnected.add_callback(disconnected_callback)
                
                cf.open_link(uri)
                
                # Esperar hasta 15 segundos
                import time
                for i in range(30):
                    time.sleep(0.5)
                    if connection_state['connected']:
                        print()
                        print("   ✅ CONEXIÓN EXITOSA")
                        print("   El problema puede ser temporal. Intenta:")
                        print("   1. Reiniciar el launch file")
                        print("   2. Verificar que el drone esté encendido")
                        cf.close_link()
                        sys.exit(0)
                    if connection_state['failed']:
                        break
                    if i % 4 == 0 and i > 0:
                        print(f"   Esperando... ({i*0.5:.1f}s)")
                
                cf.close_link()
                print()
                print("   ❌ NO SE PUDO CONECTAR")
                print()
                print("   SOLUCIONES:")
                print("   1. Verifica que el drone esté ENCENDIDO")
                print("   2. Verifica que la URI sea correcta")
                print("   3. Prueba escanear drones: python3 find_drone_uris.py")
                print("   4. Verifica que el drone esté cerca (menos de 5 metros)")
                print("   5. Reinicia el drone (apagar y encender)")
                print("   6. Verifica batería (LEDs deben estar encendidos)")
                
            else:
                print("   ⚠️  Configuración no encontrada")
    else:
        print(f"   ⚠️  Archivo de configuración no encontrado: {config_path}")
except Exception as e:
    print(f"   ⚠️  Error leyendo configuración: {e}")

print()
print("="*70)

