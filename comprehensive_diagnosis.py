#!/usr/bin/env python3
"""
Diagnóstico completo para entender por qué los drones no despegan
Verifica: parámetros del firmware, estado de los drones, servicios, y más
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters, SetParameters, ListParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
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


class ComprehensiveDiagnosis(Node):
    def __init__(self):
        super().__init__('comprehensive_diagnosis')
        self.drone_ids = ['cf1', 'cf2']
        self.status_data = {}
        
        # Suscribirse a status
        for drone_id in self.drone_ids:
            self.create_subscription(
                Status,
                f'/{drone_id}/status',
                lambda msg, d=drone_id: self.status_callback(msg, d),
                10
            )
        
        # Clientes de servicios
        self.get_params_client = self.create_client(GetParameters, '/crazyflie_server/get_parameters')
        self.set_params_client = self.create_client(SetParameters, '/crazyflie_server/set_parameters')
        self.list_params_client = self.create_client(ListParameters, '/crazyflie_server/list_parameters')
        self.arm_clients = {}
        self.takeoff_clients = {}
        
        for drone_id in self.drone_ids:
            self.arm_clients[drone_id] = self.create_client(Arm, f'/{drone_id}/arm')
            self.takeoff_clients[drone_id] = self.create_client(Takeoff, f'/{drone_id}/takeoff')
        
        self.get_logger().info('🔍 Diagnóstico completo iniciado')
    
    def status_callback(self, msg, drone_id):
        """Callback para recibir mensajes de estado"""
        self.status_data[drone_id] = {
            'supervisor_info': msg.supervisor_info,
            'battery_voltage': msg.battery_voltage,
            'pm_state': msg.pm_state,
            'rssi': msg.rssi,
            'timestamp': msg.header.stamp
        }
    
    def list_all_params(self, prefix=""):
        """Lista todos los parámetros con un prefijo"""
        if not self.list_params_client.service_is_ready():
            return []
        
        req = ListParameters.Request()
        req.prefixes = [prefix] if prefix else []
        req.depth = 10
        future = self.list_params_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.done():
            try:
                response = future.result()
                return response.names
            except Exception as e:
                self.get_logger().error(f'Error listando parámetros: {e}')
        return []
    
    def get_param(self, param_name):
        """Obtiene un parámetro"""
        if not self.get_params_client.service_is_ready():
            return None
        
        req = GetParameters.Request()
        req.names = [param_name]
        future = self.get_params_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.done():
            try:
                response = future.result()
                if response.values and len(response.values) > 0:
                    return response.values[0]
            except Exception as e:
                pass
        return None
    
    def set_param(self, param_name, value, param_type=ParameterType.PARAMETER_INTEGER):
        """Establece un parámetro"""
        if not self.set_params_client.service_is_ready():
            return False
        
        req = SetParameters.Request()
        param = Parameter()
        param.name = param_name
        param.value = ParameterValue()
        param.value.type = param_type
        
        if param_type == ParameterType.PARAMETER_INTEGER:
            param.value.integer_value = int(value)
        elif param_type == ParameterType.PARAMETER_DOUBLE:
            param.value.double_value = float(value)
        elif param_type == ParameterType.PARAMETER_STRING:
            param.value.string_value = str(value)
        elif param_type == ParameterType.PARAMETER_BOOL:
            param.value.bool_value = bool(value)
        
        req.parameters = [param]
        future = self.set_params_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        if future.done():
            try:
                response = future.result()
                if response.results and len(response.results) > 0:
                    return response.results[0].successful
            except Exception as e:
                pass
        return False
    
    def check_param(self, drone_id, param_name, expected_value, param_type=ParameterType.PARAMETER_INTEGER):
        """Verifica y corrige un parámetro"""
        full_name = f'{drone_id}.params.{param_name}'
        param = self.get_param(full_name)
        
        if param is None:
            print(f'   ⚠️  {param_name}: NO ENCONTRADO')
            print(f'      → Estableciendo a {expected_value}...')
            if self.set_param(full_name, expected_value, param_type):
                print(f'      ✅ Establecido correctamente')
                return True
            else:
                print(f'      ❌ No se pudo establecer')
                return False
        
        current = None
        if param.type == ParameterType.PARAMETER_INTEGER:
            current = param.integer_value
        elif param.type == ParameterType.PARAMETER_DOUBLE:
            current = param.double_value
        
        if current != expected_value:
            print(f'   ⚠️  {param_name}: {current} (esperado: {expected_value})')
            print(f'      → Corrigiendo...')
            if self.set_param(full_name, expected_value, param_type):
                print(f'      ✅ Corregido')
                return True
            else:
                print(f'      ❌ No se pudo corregir')
                return False
        else:
            print(f'   ✅ {param_name}: {current} (correcto)')
            return True
    
    def check_status(self, drone_id):
        """Verifica el estado del drone"""
        if drone_id not in self.status_data:
            return None
        
        s = self.status_data[drone_id]
        info = s['supervisor_info']
        
        return {
            'can_be_armed': bool(info & SUPERVISOR_INFO_CAN_BE_ARMED),
            'is_armed': bool(info & SUPERVISOR_INFO_IS_ARMED),
            'auto_arm': bool(info & SUPERVISOR_INFO_AUTO_ARM),
            'can_fly': bool(info & SUPERVISOR_INFO_CAN_FLY),
            'is_flying': bool(info & SUPERVISOR_INFO_IS_FLYING),
            'is_tumbled': bool(info & SUPERVISOR_INFO_IS_TUMBLED),
            'is_locked': bool(info & SUPERVISOR_INFO_IS_LOCKED),
            'battery': s['battery_voltage'],
            'rssi': s['rssi'],
            'pm_state': s['pm_state']
        }
    
    def run_full_diagnosis(self):
        """Ejecuta diagnóstico completo"""
        print('=' * 80)
        print('🔍 DIAGNÓSTICO COMPLETO - POR QUÉ LOS DRONES NO DESPEGAN')
        print('=' * 80)
        
        print('\n⏳ Esperando 3 segundos para recibir datos...')
        time.sleep(3.0)
        
        # 1. Verificar parámetros críticos
        print('\n' + '=' * 80)
        print('1️⃣  VERIFICACIÓN DE PARÁMETROS CRÍTICOS DEL FIRMWARE')
        print('=' * 80)
        
        critical_params = {
            'commander.enHighLevel': (1, ParameterType.PARAMETER_INTEGER),
            'commander.enAutoArm': (0, ParameterType.PARAMETER_INTEGER),  # 0 = deshabilitado
            'stabilizer.estimator': (2, ParameterType.PARAMETER_INTEGER),
            'stabilizer.controller': (2, ParameterType.PARAMETER_INTEGER),
        }
        
        params_ok = True
        for drone_id in self.drone_ids:
            print(f'\n🚁 {drone_id.upper()}:')
            for param, (val, typ) in critical_params.items():
                if not self.check_param(drone_id, param, val, typ):
                    params_ok = False
                time.sleep(0.3)
        
        # 2. Verificar estado de los drones
        print('\n' + '=' * 80)
        print('2️⃣  ESTADO DE LOS DRONES')
        print('=' * 80)
        
        for drone_id in self.drone_ids:
            print(f'\n🚁 {drone_id.upper()}:')
            status = self.check_status(drone_id)
            if status:
                print(f'   🔋 Batería: {status["battery"]:.2f}V', end='')
                if status["battery"] < 3.7:
                    print(' ⚠️  CRÍTICA')
                elif status["battery"] < 3.8:
                    print(' ⚠️  BAJA')
                else:
                    print(' ✅')
                
                print(f'   📡 RSSI: {status["rssi"]} dBm', end='')
                if status["rssi"] < -80:
                    print(' ⚠️  DÉBIL')
                else:
                    print(' ✅')
                
                print(f'   🔓 Puede armar: {"✅" if status["can_be_armed"] else "❌"}')
                print(f'   🔧 Armado: {"✅" if status["is_armed"] else "❌"}')
                print(f'   🔄 Auto-arm: {"⚠️  ACTIVO" if status["auto_arm"] else "✅ Desactivado"}')
                if status["auto_arm"]:
                    print(f'      ⚠️  El auto-arm está activo, esto puede interferir con el takeoff')
                    print(f'      → Configura commander.enAutoArm = 0 para deshabilitarlo')
                print(f'   ✈️  Puede volar: {"✅" if status["can_fly"] else "❌"}')
                print(f'   🚁 Volando: {"✅" if status["is_flying"] else "❌"}')
                print(f'   💥 Tumbado: {"⚠️  SÍ" if status["is_tumbled"] else "✅ NO"}')
                print(f'   🔒 Bloqueado: {"⚠️  SÍ" if status["is_locked"] else "✅ NO"}')
            else:
                print('   ❌ NO HAY DATOS DE ESTADO')
                print('   → Verifica que el servidor esté corriendo y el drone conectado')
        
        # 3. Verificar servicios
        print('\n' + '=' * 80)
        print('3️⃣  VERIFICACIÓN DE SERVICIOS')
        print('=' * 80)
        
        for drone_id in self.drone_ids:
            print(f'\n🚁 {drone_id.upper()}:')
            
            # Servicio arm
            if self.arm_clients[drone_id].service_is_ready():
                print('   ✅ Servicio arm: Disponible')
            else:
                print('   ❌ Servicio arm: NO disponible')
            
            # Servicio takeoff
            if self.takeoff_clients[drone_id].service_is_ready():
                print('   ✅ Servicio takeoff: Disponible')
            else:
                print('   ❌ Servicio takeoff: NO disponible')
        
        # 4. Resumen y recomendaciones
        print('\n' + '=' * 80)
        print('4️⃣  RESUMEN Y RECOMENDACIONES')
        print('=' * 80)
        
        issues_found = []
        
        for drone_id in self.drone_ids:
            status = self.check_status(drone_id)
            if not status:
                issues_found.append(f'{drone_id}: No hay datos de estado')
                continue
            
            if not params_ok:
                issues_found.append(f'{drone_id}: Parámetros del firmware incorrectos')
            
            if status['auto_arm']:
                issues_found.append(f'{drone_id}: Auto-arm está ACTIVO (puede interferir con takeoff)')
                print(f'\n💡 {drone_id} tiene auto-arm activo:')
                print(f'   → Deshabilítalo con: ros2 param set /crazyflie_server {drone_id}.params.commander.enAutoArm 0')
                print(f'   → O reinicia el servidor con la configuración actualizada')
            
            if not status['is_armed']:
                issues_found.append(f'{drone_id}: NO está armado')
                print(f'\n💡 Para armar {drone_id}:')
                print(f'   ros2 service call /{drone_id}/arm crazyflie_interfaces/srv/Arm "{{arm: true}}"')
            
            if not status['can_fly']:
                issues_found.append(f'{drone_id}: NO puede volar')
                print(f'\n💡 {drone_id} no puede volar. Verifica:')
                print(f'   - Calibración de sensores')
                print(f'   - Batería suficiente (>3.7V)')
                print(f'   - Drone en posición correcta (no tumbado)')
            
            if status['battery'] < 3.7:
                issues_found.append(f'{drone_id}: Batería crítica ({status["battery"]:.2f}V)')
            
            if status['rssi'] < -80:
                issues_found.append(f'{drone_id}: Señal débil ({status["rssi"]} dBm)')
            
            if status['is_tumbled']:
                issues_found.append(f'{drone_id}: Está tumbado')
            
            if status['is_locked']:
                issues_found.append(f'{drone_id}: Está bloqueado')
        
        if issues_found:
            print(f'\n⚠️  PROBLEMAS ENCONTRADOS ({len(issues_found)}):')
            for i, issue in enumerate(issues_found, 1):
                print(f'   {i}. {issue}')
        else:
            print('\n✅ NO SE ENCONTRARON PROBLEMAS OBVIOS')
            print('   Si los drones aún no despegan, puede ser:')
            print('   - Problema de timing (espera más tiempo después del arm)')
            print('   - Problema de calibración de sensores')
            print('   - Problema con el sistema de tracking/motion capture')
            print('   - Problema con la estimación de posición')
        
        print('\n' + '=' * 80)
        print('💡 COMANDOS ÚTILES:')
        print('=' * 80)
        print('Ver parámetros:')
        print('  ros2 param list | grep commander')
        print('  ros2 param get /crazyflie_server cf1.params.commander.enHighLevel')
        print('\nArmar manualmente:')
        print('  ros2 service call /cf1/arm crazyflie_interfaces/srv/Arm "{arm: true}"')
        print('\nTakeoff manual:')
        print('  ros2 service call /cf1/takeoff crazyflie_interfaces/srv/Takeoff "{height: 1.0, duration: {sec: 5, nanosec: 0}, group_mask: 0}"')


def main():
    rclpy.init()
    node = ComprehensiveDiagnosis()
    
    try:
        node.run_full_diagnosis()
        print('\n⏳ Esperando 5 segundos más para actualizaciones...')
        time.sleep(5.0)
        node.run_full_diagnosis()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

