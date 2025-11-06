#!/usr/bin/env python3

"""
Script avanzado para encontrar las URIs de los drones Crazyflies
Prueba múltiples métodos y proporciona diagnósticos detallados
"""

import sys
import os
import subprocess
import time

def check_usb_connection():
    """Verificar que la antena esté conectada"""
    print("1. Verificando conexión USB...")
    try:
        result = subprocess.run(['lsusb'], capture_output=True, text=True, timeout=5)
        if '1915' in result.stdout or 'Crazyradio' in result.stdout:
            print("   ✅ Antena Crazyradio PA detectada")
            print(f"   {[line for line in result.stdout.split('\n') if '1915' in line or 'Crazyradio' in line][0]}")
            return True
        else:
            print("   ❌ No se encontró la antena Crazyradio PA")
            print("   Conecta la antena por USB y vuelve a intentar")
            return False
    except Exception as e:
        print(f"   ⚠️  Error verificando USB: {e}")
        return False

def check_usb_permissions():
    """Verificar permisos USB"""
    print("\n2. Verificando permisos USB...")
    try:
        # Verificar grupo plugdev
        result = subprocess.run(['groups'], capture_output=True, text=True, timeout=5)
        if 'plugdev' in result.stdout:
            print("   ✅ Usuario en grupo plugdev")
            return True
        else:
            print("   ⚠️  Usuario NO está en grupo plugdev")
            print("   Ejecuta: sudo usermod -a -G plugdev $USER")
            print("   Y luego reinicia la sesión")
            # Intentar cambiar permisos temporalmente
            print("   Intentando permisos temporales...")
            return True  # Continuar de todos modos
    except Exception as e:
        print(f"   ⚠️  Error verificando permisos: {e}")
        return True  # Continuar

def method_1_crazyswarm2():
    """Método 1: Usar Crazyswarm2"""
    print("\n3. Método 1: Escaneo con Crazyswarm2...")
    try:
        from crazyflie_py import Crazyswarm
        print("   Inicializando Crazyswarm2...")
        print("   (Esto puede tardar 30-60 segundos...)")
        
        crazyswarm = Crazyswarm()
        allcfs = crazyswarm.allcfs
        
        uris = []
        for cf in allcfs.crazyflies:
            uri = cf.uri()
            uris.append(uri)
        
        if uris:
            print(f"   ✅ Encontrados {len(uris)} drones:")
            for i, uri in enumerate(uris, 1):
                print(f"      Drone {i}: {uri}")
            return uris
        else:
            print("   ⚠️  No se encontraron drones")
            return None
    except Exception as e:
        print(f"   ❌ Error: {e}")
        return None

def method_2_scan_executable():
    """Método 2: Usar ejecutable scan"""
    print("\n4. Método 2: Usar ejecutable scan...")
    try:
        # Buscar el ejecutable scan
        scan_paths = [
            '/workspace/ros2_ws/install/crazyflie/lib/crazyflie/scan',
            '/workspace/ros2_ws/build/crazyflie/deps/crazyflie_tools/src/scan',
        ]
        
        scan_exe = None
        for path in scan_paths:
            if os.path.exists(path):
                scan_exe = path
                break
        
        if not scan_exe:
            print("   ⚠️  Ejecutable scan no encontrado")
            return None
        
        print(f"   Ejecutando: {scan_exe}")
        result = subprocess.run(
            [scan_exe, '--address', '0xE7E7E7E7E7'],
            capture_output=True,
            text=True,
            timeout=60
        )
        
        if result.returncode == 0 and result.stdout.strip():
            uris = [line.strip() for line in result.stdout.strip().split('\n') if line.strip()]
            if uris:
                print(f"   ✅ Encontrados {len(uris)} drones:")
                for i, uri in enumerate(uris, 1):
                    print(f"      Drone {i}: {uri}")
                return uris
            else:
                print("   ⚠️  No se encontraron URIs en la salida")
                return None
        else:
            print(f"   ⚠️  El escaneo no encontró drones")
            if result.stderr:
                print(f"   Error: {result.stderr[:200]}")
            return None
    except subprocess.TimeoutExpired:
        print("   ⚠️  El escaneo tardó demasiado (timeout)")
        return None
    except Exception as e:
        print(f"   ❌ Error: {e}")
        return None

def method_3_manual_scan():
    """Método 3: Escaneo manual de canales comunes"""
    print("\n5. Método 3: Escaneo manual de canales comunes...")
    print("   Probando canales comunes (80, 90, 100)...")
    
    common_channels = [80, 90, 100]
    common_addresses = ['E7E7E7E701', 'E7E7E7E702', 'E7E7E7E703']
    datarates = ['2M', '1M', '250K']
    
    uris_found = []
    
    # Intentar con cflib directamente
    try:
        import cflib.crtp
        from cflib.crazyflie import Crazyflie
        from cflib.utils import uri_helper
        
        print("   Usando cflib para escaneo...")
        cflib.crtp.init_drivers()
        
        # Buscar en canales y direcciones comunes
        for channel in common_channels:
            for address in common_addresses:
                for datarate in datarates:
                    uri = f"radio://0/{channel}/{datarate}/{address}"
                    print(f"   Probando: {uri}")
                    try:
                        cf = Crazyflie()
                        # Intentar conexión rápida
                        cf.open_link(uri)
                        time.sleep(0.5)
                        if cf.is_connected():
                            uris_found.append(uri)
                            print(f"      ✅ Conectado!")
                        cf.close_link()
                    except:
                        pass
                    time.sleep(0.1)
        
        if uris_found:
            print(f"   ✅ Encontrados {len(uris_found)} drones:")
            for i, uri in enumerate(uris_found, 1):
                print(f"      Drone {i}: {uri}")
            return uris_found
        else:
            print("   ⚠️  No se encontraron drones en canales comunes")
            return None
            
    except ImportError:
        print("   ⚠️  cflib no está disponible")
        return None
    except Exception as e:
        print(f"   ❌ Error: {e}")
        return None

def main():
    print("="*70)
    print("ESCANEO AVANZADO DE DRONES CRAZYFLIES")
    print("="*70)
    print()
    print("Este script probará múltiples métodos para encontrar tus drones.")
    print()
    
    # Verificaciones iniciales
    usb_ok = check_usb_connection()
    if not usb_ok:
        print("\n❌ PROBLEMA: No se detectó la antena USB")
        print("   Solución: Conecta la antena Crazyradio PA por USB")
        return
    
    check_usb_permissions()
    
    print("\n" + "="*70)
    print("INICIANDO ESCANEO...")
    print("="*70)
    print("\nAsegúrate de que:")
    print("- Los drones estén ENCENDIDOS")
    print("- Los drones tengan BATERÍA suficiente")
    print("- Estén a menos de 5 metros de la antena")
    print()
    
    uris_found = None
    
    # Intentar método 1
    uris_found = method_1_crazyswarm2()
    if uris_found:
        print_uris(uris_found)
        return
    
    # Intentar método 2
    uris_found = method_2_scan_executable()
    if uris_found:
        print_uris(uris_found)
        return
    
    # Intentar método 3
    uris_found = method_3_manual_scan()
    if uris_found:
        print_uris(uris_found)
        return
    
    # Si llegamos aquí, no se encontró nada
    print("\n" + "="*70)
    print("❌ NO SE ENCONTRARON DRONES")
    print("="*70)
    print("\nSolución de problemas:")
    print("1. Verifica que los drones estén ENCENDIDOS")
    print("2. Verifica que tengan BATERÍA suficiente (LEDs encendidos)")
    print("3. Verifica que estén cerca de la antena (menos de 5 metros)")
    print("4. Prueba reiniciar los drones (apagar y encender)")
    print("5. Verifica que la antena esté funcionando: lsusb | grep 1915")
    print("6. Si los drones tienen direcciones personalizadas, necesitas conocerlas")
    print("\nSi conoces las URIs manualmente, puedes editarlas directamente en:")
    print("  /workspace/ros2_ws/src/crazyflie_dual_control/config/crazyflies.yaml")

def print_uris(uris):
    """Mostrar las URIs encontradas con instrucciones"""
    print("\n" + "="*70)
    print("✅ URIs ENCONTRADAS")
    print("="*70)
    for i, uri in enumerate(uris, 1):
        print(f"  Drone {i}: {uri}")
    
    print("\n" + "="*70)
    print("PRÓXIMOS PASOS")
    print("="*70)
    print("\nActualiza el archivo de configuración:")
    print("  /workspace/ros2_ws/src/crazyflie_dual_control/config/crazyflies.yaml")
    print("\nReemplaza las URIs con las encontradas:")
    print("\n  robots:")
    for i, uri in enumerate(uris, 1):
        print(f"    cf{i}:")
        print(f"      enabled: true")
        print(f"      uri: {uri}")
        print(f"      initial_position: [{i-1}.0, 0.0, 0.0]")
        print(f"      type: cf21")
    print("="*70)

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nEscaneo cancelado por el usuario")
    except Exception as e:
        print(f"\n❌ Error inesperado: {e}")
        import traceback
        traceback.print_exc()