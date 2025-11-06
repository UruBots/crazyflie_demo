#!/usr/bin/env python3

"""
Monitor de batería para múltiples Crazyflies
Monitorea el nivel de batería y alerta cuando está bajo
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from crazyflie_interfaces.msg import Status
import time

class BatteryMonitor(Node):
    """Monitor de batería para múltiples drones"""
    
    def __init__(self):
        super().__init__('battery_monitor')
        
        # IDs de los drones
        self.drone_ids = ['cf1', 'cf2']
        
        # Suscriptores para batería de cada drone
        self.battery_subscribers = {}
        
        # Estados de batería
        self.battery_levels = {}
        self.battery_warnings = {}
        
        # Umbrales de batería
        self.low_battery_threshold = 3.0  # Voltios
        self.critical_battery_threshold = 2.8  # Voltios
        
        # Inicializar suscriptores
        for drone_id in self.drone_ids:
            # Suscribirse al topic de status que incluye información de batería
            self.battery_subscribers[drone_id] = self.create_subscription(
                Status,
                f'/{drone_id}/status',
                lambda msg, drone=drone_id: self.battery_callback(msg, drone),
                10
            )
            
            # Inicializar estados
            self.battery_levels[drone_id] = 0.0
            self.battery_warnings[drone_id] = False
        
        # Timer para verificación periódica
        self.timer = self.create_timer(1.0, self.check_battery_levels)
        
        # Publisher para alertas
        self.alert_publisher = self.create_publisher(
            String, '/battery_alerts', 10
        )
        
        self.get_logger().info('Monitor de batería iniciado')
    
    def battery_callback(self, msg, drone_id):
        """Callback para actualizar nivel de batería"""
        # El mensaje Status tiene battery_voltage como campo
        voltage = msg.battery_voltage
        self.battery_levels[drone_id] = voltage
        
        # Verificar si está bajo
        if voltage < self.low_battery_threshold:
            if not self.battery_warnings[drone_id]:
                self.send_battery_warning(drone_id, voltage, 'low')
                self.battery_warnings[drone_id] = True
        
        # Verificar si está crítico
        if voltage < self.critical_battery_threshold:
            self.send_battery_warning(drone_id, voltage, 'critical')
    
    def check_battery_levels(self):
        """Verificación periódica de niveles de batería"""
        for drone_id in self.drone_ids:
            voltage = self.battery_levels[drone_id]
            
            if voltage > 0:  # Solo si tenemos datos
                # Log periódico
                if voltage < self.low_battery_threshold:
                    self.get_logger().warn(
                        f'{drone_id}: Batería baja - {voltage:.2f}V'
                    )
                else:
                    self.get_logger().info(
                        f'{drone_id}: Batería OK - {voltage:.2f}V'
                    )
    
    def send_battery_warning(self, drone_id, voltage, level):
        """Envía alerta de batería"""
        alert_msg = String()
        
        if level == 'low':
            alert_msg.data = f'WARNING: {drone_id} batería baja ({voltage:.2f}V)'
            self.get_logger().warn(alert_msg.data)
        elif level == 'critical':
            alert_msg.data = f'CRITICAL: {drone_id} batería crítica ({voltage:.2f}V) - Aterrizar inmediatamente!'
            self.get_logger().error(alert_msg.data)
        
        self.alert_publisher.publish(alert_msg)
    
    def get_battery_status(self):
        """Retorna el estado actual de todas las baterías"""
        status = {}
        for drone_id in self.drone_ids:
            voltage = self.battery_levels[drone_id]
            if voltage > 0:
                if voltage < self.critical_battery_threshold:
                    status[drone_id] = 'critical'
                elif voltage < self.low_battery_threshold:
                    status[drone_id] = 'low'
                else:
                    status[drone_id] = 'ok'
            else:
                status[drone_id] = 'unknown'
        
        return status

def main(args=None):
    rclpy.init(args=args)
    
    monitor = BatteryMonitor()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        monitor.get_logger().info('Deteniendo monitor de batería...')
    finally:
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
