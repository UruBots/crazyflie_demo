#!/usr/bin/env python3

"""
Script de ejemplo simple para volar 2 Crazyflies
Demuestra comandos básicos de vuelo
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty
import time
import math

class SimpleDualFlight(Node):
    """Vuelo simple con 2 drones"""
    
    def __init__(self):
        super().__init__('simple_dual_flight')
        
        # Publishers para comandos de posición
        self.cf1_position_pub = self.create_publisher(
            PoseStamped, '/cf1/cmd_position', 10
        )
        self.cf2_position_pub = self.create_publisher(
            PoseStamped, '/cf2/cmd_position', 10
        )
        
        # Publisher para parada de emergencia
        self.emergency_pub = self.create_publisher(
            Empty, '/emergency_stop', 10
        )
        
        # Timer para comandos de vuelo
        self.timer = self.create_timer(0.1, self.flight_loop)
        
        # Estado del vuelo
        self.start_time = time.time()
        self.flight_phase = 0
        
        self.get_logger().info('Vuelo simple iniciado')
    
    def send_position(self, drone_pub, x, y, z, yaw=0.0):
        """Envía comando de posición a un drone"""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        
        # Convertir yaw a cuaternión
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        
        drone_pub.publish(msg)
    
    def flight_loop(self):
        """Loop principal de vuelo"""
        current_time = time.time() - self.start_time
        
        # Fase 1: Despegue (0-3 segundos)
        if current_time < 3.0:
            if self.flight_phase == 0:
                self.get_logger().info('Fase 1: Despegue')
                self.flight_phase = 1
            
            # Despegar ambos drones
            self.send_position(self.cf1_position_pub, 0.0, 0.0, 0.5)
            self.send_position(self.cf2_position_pub, 1.0, 0.0, 0.5)
        
        # Fase 2: Formación horizontal (3-8 segundos)
        elif current_time < 8.0:
            if self.flight_phase == 1:
                self.get_logger().info('Fase 2: Formación horizontal')
                self.flight_phase = 2
            
            # Mantener formación horizontal
            self.send_position(self.cf1_position_pub, 0.0, 0.0, 0.5)
            self.send_position(self.cf2_position_pub, 1.0, 0.0, 0.5)
        
        # Fase 3: Movimiento circular (8-15 segundos)
        elif current_time < 15.0:
            if self.flight_phase == 2:
                self.get_logger().info('Fase 3: Movimiento circular')
                self.flight_phase = 3
            
            # Movimiento circular sincronizado
            angle = (current_time - 8.0) * 0.5  # Velocidad angular
            
            # Drone 1: círculo pequeño
            x1 = 0.3 * math.cos(angle)
            y1 = 0.3 * math.sin(angle)
            
            # Drone 2: círculo más grande
            x2 = 1.0 + 0.5 * math.cos(angle + math.pi)
            y2 = 0.5 * math.sin(angle + math.pi)
            
            self.send_position(self.cf1_position_pub, x1, y1, 0.5)
            self.send_position(self.cf2_position_pub, x2, y2, 0.5)
        
        # Fase 4: Aterrizaje (15+ segundos)
        else:
            if self.flight_phase == 3:
                self.get_logger().info('Fase 4: Aterrizaje')
                self.flight_phase = 4
            
            # Aterrizar ambos drones
            self.send_position(self.cf1_position_pub, 0.0, 0.0, 0.0)
            self.send_position(self.cf2_position_pub, 1.0, 0.0, 0.0)
            
            # Parar después de 20 segundos
            if current_time > 20.0:
                self.get_logger().info('Vuelo completado')
                self.timer.cancel()
    
    def emergency_stop(self):
        """Parada de emergencia"""
        self.get_logger().warn('¡PARADA DE EMERGENCIA!')
        msg = Empty()
        self.emergency_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    
    flight = SimpleDualFlight()
    
    try:
        rclpy.spin(flight)
    except KeyboardInterrupt:
        flight.get_logger().info('Deteniendo vuelo...')
        flight.emergency_stop()
    finally:
        flight.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
