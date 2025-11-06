#!/usr/bin/env python3
"""
Script de diagnóstico para entender por qué los drones no despegan
"""

import rclpy
from rclpy.node import Node
from crazyflie_interfaces.msg import Status
from crazyflie_interfaces.srv import Arm, Takeoff
import time
import sys

# Constantes del mensaje Status
SUPERVISOR_INFO_CAN_BE_ARMED = 1
SUPERVISOR_INFO_IS_ARMED = 2
SUPERVISOR_INFO_AUTO_ARM = 4
SUPERVISOR_INFO_CAN_FLY = 8
SUPERVISOR_INFO_IS_FLYING = 16
SUPERVISOR_INFO_IS_TUMBLED = 32
SUPERVISOR_INFO_IS_LOCKED = 64


class TakeoffDiagnostic(Node):
    def __init__(self):
        super().__init__('takeoff_diagnostic')
        self.drone_ids = ['cf1', 'cf2']
        self.status_data = {}
        self.status_subscribers = {}
        self.arm_clients = {}
        self.takeoff_clients = {}
        
        # Suscribirse a los tópicos de estado
        for drone_id in self.drone_ids:
            self.status_subscribers[drone_id] = self.create_subscription(
                Status,
                f'/{drone_id}/status',
                lambda msg, d=drone_id: self.status_callback(msg, d),
                10
            )
            self.arm_clients[drone_id] = self.create_client(Arm, f'/{drone_id}/arm')
            self.takeoff_clients[drone_id] = self.create_client(Takeoff, f'/{drone_id}/takeoff')
        
        self.get_logger().info('🔍 Diagnóstico de Takeoff iniciado')
        self.get_logger().info('   Esperando datos de estado de los drones...')
        
    def status_callback(self, msg, drone_id):
        """Callback para recibir mensajes de estado"""
        self.status_data[drone_id] = {
            'supervisor_info': msg.supervisor_info,
            'battery_voltage': msg.battery_voltage,
            'pm_state': msg.pm_state,
            'rssi': msg.rssi,
            'timestamp': msg.header.stamp
        }
    
    def check_status(self, drone_id):
        """Verifica el estado de un drone"""
        if drone_id not in self.status_data:
            return None
        
        status = self.status_data[drone_id]
        supervisor_info = status['supervisor_info']
        
        return {
            'can_be_armed': bool(supervisor_info & SUPERVISOR_INFO_CAN_BE_ARMED),
            'is_armed': bool(supervisor_info & SUPERVISOR_INFO_IS_ARMED),
            'auto_arm': bool(supervisor_info & SUPERVISOR_INFO_AUTO_ARM),
            'can_fly': bool(supervisor_info & SUPERVISOR_INFO_CAN_FLY),
            'is_flying': bool(supervisor_info & SUPERVISOR_INFO_IS_FLYING),
            'is_tumbled': bool(supervisor_info & SUPERVISOR_INFO_IS_TUMBLED),
            'is_locked': bool(supervisor_info & SUPERVISOR_INFO_IS_LOCKED),
            'battery_voltage': status['battery_voltage'],
            'pm_state': status['pm_state'],
            'rssi': status['rssi']
        }
    
    def print_status(self, drone_id):
        """Imprime el estado de un drone"""
        status = self.check_status(drone_id)
        if status is None:
            print(f'\n❌ {drone_id}: NO HAY DATOS DE ESTADO')
            print(f'   El tópico /{drone_id}/status no está publicando datos')
            print(f'   Posibles causas:')
            print(f'   1. El servidor no está corriendo')
            print(f'   2. El drone no está conectado')
            print(f'   3. El drone no está configurado correctamente')
            return
        
        print(f'\n📊 ESTADO DE {drone_id.upper()}:')
        print(f'   🔋 Batería: {status["battery_voltage"]:.2f}V', end='')
        if status['battery_voltage'] < 3.7:
            print(' ⚠️  CRÍTICA (muy baja)')
        elif status['battery_voltage'] < 3.8:
            print(' ⚠️  BAJA')
        else:
            print(' ✅ OK')
        
        print(f'   📡 RSSI: {status["rssi"]} dBm', end='')
        if status['rssi'] < -80:
            print(' ⚠️  SEÑAL DÉBIL')
        else:
            print(' ✅ OK')
        
        print(f'   🔓 Puede ser armado: {"✅ SÍ" if status["can_be_armed"] else "❌ NO"}')
        print(f'   🔧 Está armado: {"✅ SÍ" if status["is_armed"] else "❌ NO"}')
        print(f'   ✈️  Puede volar: {"✅ SÍ" if status["can_fly"] else "❌ NO"}')
        print(f'   🚁 Está volando: {"✅ SÍ" if status["is_flying"] else "❌ NO"}')
        print(f'   🔄 Auto-arm: {"✅ ACTIVO" if status["auto_arm"] else "❌ INACTIVO"}')
        print(f'   💥 Está tumbado: {"⚠️  SÍ" if status["is_tumbled"] else "✅ NO"}')
        print(f'   🔒 Está bloqueado: {"⚠️  SÍ" if status["is_locked"] else "✅ NO"}')
        
        # Diagnóstico
        issues = []
        if not status['can_be_armed']:
            issues.append('No puede ser armado (verifica batería, calibración, etc.)')
        if not status['is_armed']:
            issues.append('No está armado (necesita llamar al servicio arm)')
        if not status['can_fly']:
            issues.append('No puede volar (verifica calibración, sensores, etc.)')
        if status['is_tumbled']:
            issues.append('Está tumbado (colócalo en posición correcta)')
        if status['is_locked']:
            issues.append('Está bloqueado (verifica el motivo del bloqueo)')
        if status['battery_voltage'] < 3.7:
            issues.append('Batería crítica (carga la batería)')
        if status['rssi'] < -80:
            issues.append('Señal de radio débil (acerca el drone a la antena)')
        
        if issues:
            print(f'\n   ⚠️  PROBLEMAS DETECTADOS:')
            for i, issue in enumerate(issues, 1):
                print(f'      {i}. {issue}')
        else:
            print(f'\n   ✅ TODO PARECE ESTAR BIEN')
    
    def test_arm_service(self, drone_id):
        """Prueba el servicio arm"""
        print(f'\n🔧 PROBANDO SERVICIO ARM PARA {drone_id.upper()}...')
        
        if not self.arm_clients[drone_id].service_is_ready():
            print(f'   ❌ Servicio arm no está disponible')
            return False
        
        print(f'   ✅ Servicio arm está disponible')
        print(f'   → Enviando comando arm...')
        
        req = Arm.Request()
        req.arm = True
        future = self.arm_clients[drone_id].call_async(req)
        
        # Esperar respuesta
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        if future.done():
            try:
                response = future.result()
                print(f'   ✅ Comando arm enviado exitosamente')
                return True
            except Exception as e:
                print(f'   ❌ Error en respuesta: {e}')
                return False
        else:
            print(f'   ❌ TIMEOUT - El servicio no respondió')
            return False
    
    def test_takeoff_service(self, drone_id):
        """Prueba el servicio takeoff"""
        print(f'\n✈️  PROBANDO SERVICIO TAKEOFF PARA {drone_id.upper()}...')
        
        if not self.takeoff_clients[drone_id].service_is_ready():
            print(f'   ❌ Servicio takeoff no está disponible')
            return False
        
        print(f'   ✅ Servicio takeoff está disponible')
        print(f'   → Enviando comando takeoff (altura: 1.0m, duración: 5.0s)...')
        
        from crazyflie_interfaces.srv import Takeoff
        req = Takeoff.Request()
        req.height = 1.0
        req.duration.sec = 5
        req.duration.nanosec = 0
        req.group_mask = 0
        
        future = self.takeoff_clients[drone_id].call_async(req)
        
        # Esperar respuesta
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        if future.done():
            try:
                response = future.result()
                print(f'   ✅ Comando takeoff enviado exitosamente')
                return True
            except Exception as e:
                print(f'   ❌ Error en respuesta: {e}')
                return False
        else:
            print(f'   ❌ TIMEOUT - El servicio no respondió')
            return False
    
    def run_diagnosis(self):
        """Ejecuta el diagnóstico completo"""
        print('=' * 70)
        print('🔍 DIAGNÓSTICO DE TAKEOFF PARA CRAZYFLIES')
        print('=' * 70)
        
        # Esperar un poco para recibir datos de estado
        print('\n⏳ Esperando 3 segundos para recibir datos de estado...')
        time.sleep(3.0)
        
        # Verificar estado de cada drone
        for drone_id in self.drone_ids:
            self.print_status(drone_id)
        
        # Probar servicios
        print('\n' + '=' * 70)
        print('🧪 PROBANDO SERVICIOS')
        print('=' * 70)
        
        for drone_id in self.drone_ids:
            print(f'\n--- {drone_id.upper()} ---')
            arm_ok = self.test_arm_service(drone_id)
            if arm_ok:
                time.sleep(2.0)  # Esperar después del arm
                takeoff_ok = self.test_takeoff_service(drone_id)
            else:
                print(f'   ⚠️  No se puede probar takeoff sin arm exitoso')
        
        # Resumen final
        print('\n' + '=' * 70)
        print('📋 RESUMEN Y RECOMENDACIONES')
        print('=' * 70)
        
        for drone_id in self.drone_ids:
            status = self.check_status(drone_id)
            if status is None:
                print(f'\n❌ {drone_id}: No hay datos disponibles')
                print(f'   → Verifica que el servidor esté corriendo')
                print(f'   → Verifica que el drone esté conectado')
                continue
            
            print(f'\n{drone_id.upper()}:')
            if not status['is_armed']:
                print(f'   ⚠️  El drone NO está armado')
                print(f'   → Ejecuta: ros2 service call /{drone_id}/arm crazyflie_interfaces/srv/Arm "{{arm: true}}"')
            if not status['can_fly']:
                print(f'   ⚠️  El drone NO puede volar')
                print(f'   → Verifica calibración, sensores, batería')
            if status['is_tumbled']:
                print(f'   ⚠️  El drone está tumbado')
                print(f'   → Colócalo en posición correcta')
            if status['battery_voltage'] < 3.7:
                print(f'   ⚠️  Batería crítica: {status["battery_voltage"]:.2f}V')
                print(f'   → Carga la batería antes de volar')
            if status['rssi'] < -80:
                print(f'   ⚠️  Señal débil: {status["rssi"]} dBm')
                print(f'   → Acerca el drone a la antena')


def main():
    rclpy.init()
    node = TakeoffDiagnostic()
    
    try:
        node.run_diagnosis()
        # Mantener el nodo vivo un poco más para recibir actualizaciones
        print('\n⏳ Esperando 5 segundos más para recibir actualizaciones de estado...')
        time.sleep(5.0)
        node.run_diagnosis()  # Ejecutar diagnóstico de nuevo
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

