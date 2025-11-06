#!/usr/bin/env python3
"""
Script para verificar y configurar parámetros críticos del firmware
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters, SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from crazyflie_interfaces.msg import Status
import time
import sys

class FirmwareParamChecker(Node):
    def __init__(self):
        super().__init__('firmware_param_checker')
        self.drone_ids = ['cf1', 'cf2']
        self.status_data = {}
        
        # Suscribirse a status para verificar estado
        for drone_id in self.drone_ids:
            self.create_subscription(
                Status,
                f'/{drone_id}/status',
                lambda msg, d=drone_id: self.status_callback(msg, d),
                10
            )
        
        # Cliente para obtener parámetros
        self.get_params_client = self.create_client(
            GetParameters,
            '/crazyflie_server/get_parameters'
        )
        
        # Cliente para establecer parámetros
        self.set_params_client = self.create_client(
            SetParameters,
            '/crazyflie_server/set_parameters'
        )
        
        self.get_logger().info('🔍 Verificador de parámetros del firmware iniciado')
    
    def status_callback(self, msg, drone_id):
        """Callback para recibir mensajes de estado"""
        self.status_data[drone_id] = {
            'supervisor_info': msg.supervisor_info,
            'battery_voltage': msg.battery_voltage,
            'rssi': msg.rssi
        }
    
    def get_param(self, param_name):
        """Obtiene un parámetro del servidor"""
        if not self.get_params_client.service_is_ready():
            self.get_logger().warn(f'Servicio get_parameters no está disponible')
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
                self.get_logger().error(f'Error obteniendo parámetro {param_name}: {e}')
        return None
    
    def set_param(self, param_name, value, param_type=ParameterType.PARAMETER_INTEGER):
        """Establece un parámetro en el servidor"""
        if not self.set_params_client.service_is_ready():
            self.get_logger().warn(f'Servicio set_parameters no está disponible')
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
                self.get_logger().error(f'Error estableciendo parámetro {param_name}: {e}')
        return False
    
    def check_and_set_param(self, drone_id, param_name, expected_value, param_type=ParameterType.PARAMETER_INTEGER):
        """Verifica y establece un parámetro si es necesario"""
        full_param_name = f'{drone_id}.params.{param_name}'
        
        print(f'\n📋 Verificando: {full_param_name}')
        param_value = self.get_param(full_param_name)
        
        if param_value is None:
            print(f'   ⚠️  Parámetro no encontrado o no disponible')
            print(f'   → Intentando establecer a {expected_value}...')
            success = self.set_param(full_param_name, expected_value, param_type)
            if success:
                print(f'   ✅ Parámetro establecido correctamente')
                return True
            else:
                print(f'   ❌ No se pudo establecer el parámetro')
                return False
        else:
            current_value = None
            if param_value.type == ParameterType.PARAMETER_INTEGER:
                current_value = param_value.integer_value
            elif param_value.type == ParameterType.PARAMETER_DOUBLE:
                current_value = param_value.double_value
            elif param_value.type == ParameterType.PARAMETER_STRING:
                current_value = param_value.string_value
            elif param_value.type == ParameterType.PARAMETER_BOOL:
                current_value = param_value.bool_value
            
            print(f'   Valor actual: {current_value}')
            
            if current_value != expected_value:
                print(f'   ⚠️  Valor incorrecto (esperado: {expected_value})')
                print(f'   → Estableciendo a {expected_value}...')
                success = self.set_param(full_param_name, expected_value, param_type)
                if success:
                    print(f'   ✅ Parámetro actualizado correctamente')
                    return True
                else:
                    print(f'   ❌ No se pudo actualizar el parámetro')
                    return False
            else:
                print(f'   ✅ Valor correcto')
                return True
    
    def check_status(self, drone_id):
        """Verifica el estado del drone"""
        if drone_id not in self.status_data:
            return None
        
        status = self.status_data[drone_id]
        supervisor_info = status['supervisor_info']
        
        return {
            'is_armed': bool(supervisor_info & 2),  # SUPERVISOR_INFO_IS_ARMED
            'can_fly': bool(supervisor_info & 8),   # SUPERVISOR_INFO_CAN_FLY
            'is_flying': bool(supervisor_info & 16), # SUPERVISOR_INFO_IS_FLYING
            'battery_voltage': status['battery_voltage'],
            'rssi': status['rssi']
        }
    
    def run_check(self):
        """Ejecuta la verificación completa"""
        print('=' * 70)
        print('🔍 VERIFICACIÓN DE PARÁMETROS CRÍTICOS DEL FIRMWARE')
        print('=' * 70)
        
        # Esperar un poco para recibir datos
        print('\n⏳ Esperando 2 segundos para recibir datos de estado...')
        time.sleep(2.0)
        
        # Parámetros críticos para cada drone
        critical_params = {
            'commander.enHighLevel': (1, ParameterType.PARAMETER_INTEGER),
            'commander.enAutoArm': (0, ParameterType.PARAMETER_INTEGER),  # 0 = deshabilitado
            'stabilizer.estimator': (2, ParameterType.PARAMETER_INTEGER),  # 2 = kalman
            'stabilizer.controller': (2, ParameterType.PARAMETER_INTEGER),  # 2 = mellinger
        }
        
        all_ok = True
        
        for drone_id in self.drone_ids:
            print(f'\n{"=" * 70}')
            print(f'🚁 DRONE: {drone_id.upper()}')
            print(f'{"=" * 70}')
            
            # Verificar parámetros
            for param_name, (expected_value, param_type) in critical_params.items():
                ok = self.check_and_set_param(drone_id, param_name, expected_value, param_type)
                if not ok:
                    all_ok = False
                time.sleep(0.5)  # Pequeña pausa entre parámetros
            
            # Verificar estado
            print(f'\n📊 ESTADO DEL DRONE:')
            status = self.check_status(drone_id)
            if status:
                print(f'   🔋 Batería: {status["battery_voltage"]:.2f}V')
                print(f'   📡 RSSI: {status["rssi"]} dBm')
                print(f'   🔧 Armado: {"✅ SÍ" if status["is_armed"] else "❌ NO"}')
                print(f'   ✈️  Puede volar: {"✅ SÍ" if status["can_fly"] else "❌ NO"}')
                print(f'   🚁 Volando: {"✅ SÍ" if status["is_flying"] else "❌ NO"}')
                
                if not status['is_armed']:
                    print(f'\n   ⚠️  El drone NO está armado')
                    print(f'   → Ejecuta: ros2 service call /{drone_id}/arm crazyflie_interfaces/srv/Arm "{{arm: true}}"')
                if not status['can_fly']:
                    print(f'\n   ⚠️  El drone NO puede volar')
                    print(f'   → Verifica calibración, sensores, batería')
            else:
                print(f'   ⚠️  No hay datos de estado disponibles')
                print(f'   → Verifica que el servidor esté corriendo y el drone conectado')
        
        # Resumen
        print(f'\n{"=" * 70}')
        print('📋 RESUMEN')
        print(f'{"=" * 70}')
        
        if all_ok:
            print('✅ Todos los parámetros críticos están configurados correctamente')
            print('\n💡 Si los drones aún no despegan, verifica:')
            print('   1. Que los drones estén armados (servicio arm)')
            print('   2. Que los drones tengan batería suficiente (>3.7V)')
            print('   3. Que los drones estén en una superficie plana')
            print('   4. Que no haya objetos cerca que interfieran')
            print('   5. Que la señal de radio sea buena (RSSI > -80 dBm)')
        else:
            print('❌ Algunos parámetros no se pudieron configurar correctamente')
            print('   → Verifica los logs del servidor para más detalles')
        
        print('\n💡 Para verificar manualmente los parámetros:')
        print('   ros2 param list | grep commander')
        print('   ros2 param get /crazyflie_server cf1.params.commander.enHighLevel')


def main():
    rclpy.init()
    node = FirmwareParamChecker()
    
    try:
        node.run_check()
        # Esperar un poco más para recibir actualizaciones
        print('\n⏳ Esperando 3 segundos más para recibir actualizaciones...')
        time.sleep(3.0)
        node.run_check()  # Verificar de nuevo
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

