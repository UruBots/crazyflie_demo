#!/usr/bin/env python3

"""
Controlador avanzado para 2 Crazyflies con multiranger y mapeo
Integra capacidades de navegación autónoma y mapeo simultáneo
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist, Point
from std_msgs.msg import Empty, String, Float32MultiArray
from sensor_msgs.msg import LaserScan, PointCloud2
from nav_msgs.msg import OccupancyGrid, MapMetaData
from crazyflie_interfaces.srv import Takeoff, Land, GoTo
from crazyflie_interfaces.msg import Position, VelocityWorld
import numpy as np
import time
import math
import threading

class AdvancedDualController(Node):
    """Controlador avanzado con multiranger y mapeo"""
    
    def __init__(self):
        super().__init__('advanced_dual_controller')
        
        # IDs de los drones
        self.drone_ids = ['cf1', 'cf2']
        
        # Estados de los drones
        self.drone_positions = {
            'cf1': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0},
            'cf2': {'x': 1.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0}
        }
        
        self.drone_active = {
            'cf1': False,
            'cf2': False
        }
        
        # Datos de multiranger
        self.multiranger_data = {
            'cf1': {'front': 0.0, 'back': 0.0, 'left': 0.0, 'right': 0.0, 'up': 0.0, 'down': 0.0},
            'cf2': {'front': 0.0, 'back': 0.0, 'left': 0.0, 'right': 0.0, 'up': 0.0, 'down': 0.0}
        }
        
        # Mapas individuales
        self.drone_maps = {
            'cf1': None,
            'cf2': None
        }
        
        # Mapa fusionado
        self.fused_map = None
        
        # Clientes de servicios
        self.takeoff_clients = {}
        self.land_clients = {}
        self.goto_clients = {}
        
        # Publishers
        self.position_publishers = {}
        self.velocity_publishers = {}
        self.emergency_publisher = self.create_publisher(Empty, '/emergency_stop', 10)
        self.fused_map_publisher = self.create_publisher(OccupancyGrid, '/fused_map', 10)
        
        # Suscriptores para multiranger
        self.multiranger_subscribers = {}
        
        # Suscriptores para mapas
        self.map_subscribers = {}
        
        # Inicializar clientes y publishers
        for drone_id in self.drone_ids:
            # Servicios
            self.takeoff_clients[drone_id] = self.create_client(
                Takeoff, f'/{drone_id}/takeoff'
            )
            self.land_clients[drone_id] = self.create_client(
                Land, f'/{drone_id}/land'
            )
            self.goto_clients[drone_id] = self.create_client(
                GoTo, f'/{drone_id}/go_to'
            )
            
            # Publishers
            self.position_publishers[drone_id] = self.create_publisher(
                Position, f'/{drone_id}/cmd_position', 10
            )
            self.velocity_publishers[drone_id] = self.create_publisher(
                VelocityWorld, f'/{drone_id}/cmd_velocity_world', 10
            )
            
            # Suscriptores multiranger
            self.multiranger_subscribers[drone_id] = self.create_subscription(
                LaserScan,
                f'/{drone_id}/scan',
                lambda msg, drone=drone_id: self.multiranger_callback(msg, drone),
                10
            )
            
            # Suscriptores mapas
            self.map_subscribers[drone_id] = self.create_subscription(
                OccupancyGrid,
                f'/{drone_id}/map',
                lambda msg, drone=drone_id: self.map_callback(msg, drone),
                10
            )
        
        # Timer para control principal
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # Timer para fusión de mapas
        self.map_fusion_timer = self.create_timer(1.0, self.fuse_maps)
        
        # Modos de operación
        self.operation_mode = 'manual'  # manual, exploration, formation, mapping
        
        self.get_logger().info('Controlador avanzado iniciado')
    
    def multiranger_callback(self, msg, drone_id):
        """Callback para datos de multiranger"""
        if len(msg.ranges) >= 6:
            self.multiranger_data[drone_id] = {
                'front': msg.ranges[0] if not np.isnan(msg.ranges[0]) else 4.0,
                'back': msg.ranges[1] if not np.isnan(msg.ranges[1]) else 4.0,
                'left': msg.ranges[2] if not np.isnan(msg.ranges[2]) else 4.0,
                'right': msg.ranges[3] if not np.isnan(msg.ranges[3]) else 4.0,
                'up': msg.ranges[4] if not np.isnan(msg.ranges[4]) else 4.0,
                'down': msg.ranges[5] if not np.isnan(msg.ranges[5]) else 4.0
            }
    
    def map_callback(self, msg, drone_id):
        """Callback para mapas individuales"""
        self.drone_maps[drone_id] = msg
        self.get_logger().debug(f'Mapa recibido de {drone_id}: {msg.info.width}x{msg.info.height}')
    
    def fuse_maps(self):
        """Fusiona mapas de ambos drones"""
        if self.drone_maps['cf1'] is not None and self.drone_maps['cf2'] is not None:
            # Implementar fusión de mapas aquí
            # Por simplicidad, usar el mapa del cf1 como base
            self.fused_map = self.drone_maps['cf1']
            self.fused_map_publisher.publish(self.fused_map)
    
    def check_obstacles(self, drone_id, direction):
        """Verifica obstáculos en una dirección específica"""
        ranges = self.multiranger_data[drone_id]
        min_distance = 0.5  # Distancia mínima segura
        
        if direction == 'forward' and ranges['front'] < min_distance:
            return True
        elif direction == 'backward' and ranges['back'] < min_distance:
            return True
        elif direction == 'left' and ranges['left'] < min_distance:
            return True
        elif direction == 'right' and ranges['right'] < min_distance:
            return True
        elif direction == 'up' and ranges['up'] < min_distance:
            return True
        elif direction == 'down' and ranges['down'] < min_distance:
            return True
        
        return False
    
    def avoid_obstacles(self, drone_id, target_x, target_y, target_z):
        """Evita obstáculos mientras se mueve hacia el objetivo"""
        current_pos = self.drone_positions[drone_id]
        ranges = self.multiranger_data[drone_id]
        
        # Calcular dirección hacia el objetivo
        dx = target_x - current_pos['x']
        dy = target_y - current_pos['y']
        dz = target_z - current_pos['z']
        
        # Normalizar
        distance = math.sqrt(dx*dx + dy*dy + dz*dz)
        if distance > 0:
            dx /= distance
            dy /= distance
            dz /= distance
        
        # Verificar obstáculos y ajustar
        if ranges['front'] < 0.5 and dx > 0:
            dx = 0
        if ranges['back'] < 0.5 and dx < 0:
            dx = 0
        if ranges['left'] < 0.5 and dy > 0:
            dy = 0
        if ranges['right'] < 0.5 and dy < 0:
            dy = 0
        if ranges['up'] < 0.5 and dz > 0:
            dz = 0
        if ranges['down'] < 0.5 and dz < 0:
            dz = 0
        
        # Aplicar movimiento
        new_x = current_pos['x'] + dx * 0.1
        new_y = current_pos['y'] + dy * 0.1
        new_z = current_pos['z'] + dz * 0.1
        
        return new_x, new_y, new_z
    
    def exploration_mode(self):
        """Modo de exploración autónoma"""
        for drone_id in self.drone_ids:
            if self.drone_active[drone_id]:
                current_pos = self.drone_positions[drone_id]
                
                # Buscar dirección con menos obstáculos
                ranges = self.multiranger_data[drone_id]
                best_direction = 'forward'
                max_distance = ranges['front']
                
                if ranges['left'] > max_distance:
                    best_direction = 'left'
                    max_distance = ranges['left']
                if ranges['right'] > max_distance:
                    best_direction = 'right'
                    max_distance = ranges['right']
                if ranges['back'] > max_distance:
                    best_direction = 'backward'
                    max_distance = ranges['back']
                
                # Mover en la mejor dirección
                step_size = 0.2
                if best_direction == 'forward':
                    target_x = current_pos['x'] + step_size
                    target_y = current_pos['y']
                elif best_direction == 'backward':
                    target_x = current_pos['x'] - step_size
                    target_y = current_pos['y']
                elif best_direction == 'left':
                    target_x = current_pos['x']
                    target_y = current_pos['y'] + step_size
                elif best_direction == 'right':
                    target_x = current_pos['x']
                    target_y = current_pos['y'] - step_size
                
                target_z = current_pos['z']
                
                # Evitar obstáculos
                safe_x, safe_y, safe_z = self.avoid_obstacles(drone_id, target_x, target_y, target_z)
                
                # Enviar comando
                self.send_position_command(drone_id, safe_x, safe_y, safe_z)
    
    def formation_mode(self):
        """Modo de vuelo en formación"""
        # Drone 1: líder
        # Drone 2: seguidor a distancia fija
        
        if self.drone_active['cf1'] and self.drone_active['cf2']:
            # Mover drone 1 (líder)
            leader_pos = self.drone_positions['cf1']
            leader_pos['x'] += 0.1  # Movimiento hacia adelante
            
            # Calcular posición del seguidor
            follower_target_x = leader_pos['x'] - 1.0  # 1 metro detrás
            follower_target_y = leader_pos['y']
            follower_target_z = leader_pos['z']
            
            # Enviar comandos
            self.send_position_command('cf1', leader_pos['x'], leader_pos['y'], leader_pos['z'])
            self.send_position_command('cf2', follower_target_x, follower_target_y, follower_target_z)
    
    def mapping_mode(self):
        """Modo de mapeo coordinado"""
        # Los drones exploran áreas diferentes para mapear eficientemente
        for drone_id in self.drone_ids:
            if self.drone_active[drone_id]:
                # Implementar patrón de exploración sistemática
                self.exploration_mode()
    
    def control_loop(self):
        """Loop principal de control"""
        if self.operation_mode == 'exploration':
            self.exploration_mode()
        elif self.operation_mode == 'formation':
            self.formation_mode()
        elif self.operation_mode == 'mapping':
            self.mapping_mode()
    
    def send_position_command(self, drone_id, x, y, z, yaw=0.0):
        """Envía comando de posición"""
        if drone_id not in self.position_publishers:
            return False
        
        # Actualizar posición interna
        self.drone_positions[drone_id]['x'] = x
        self.drone_positions[drone_id]['y'] = y
        self.drone_positions[drone_id]['z'] = z
        self.drone_positions[drone_id]['yaw'] = yaw
        
        # Enviar comando
        msg = Position()
        msg.x = float(x)
        msg.y = float(y)
        msg.z = float(z)
        msg.yaw = float(yaw)
        
        self.position_publishers[drone_id].publish(msg)
        return True
    
    def takeoff_all(self, height=0.5):
        """Despega todos los drones"""
        for drone_id in self.drone_ids:
            req = Takeoff.Request()
            req.height = height
            req.duration = rclpy.duration.Duration(seconds=2.0).to_msg()
            
            future = self.takeoff_clients[drone_id].call_async(req)
            self.drone_active[drone_id] = True
        
        self.get_logger().info('Comando de despegue enviado a todos los drones')
    
    def land_all(self):
        """Aterriza todos los drones"""
        for drone_id in self.drone_ids:
            req = Land.Request()
            req.height = 0.0
            req.duration = rclpy.duration.Duration(seconds=2.0).to_msg()
            
            future = self.land_clients[drone_id].call_async(req)
            self.drone_active[drone_id] = False
        
        self.get_logger().info('Comando de aterrizaje enviado a todos los drones')
    
    def emergency_stop(self):
        """Parada de emergencia"""
        self.get_logger().warn('¡PARADA DE EMERGENCIA!')
        msg = Empty()
        self.emergency_publisher.publish(msg)
        self.land_all()
    
    def set_operation_mode(self, mode):
        """Cambia el modo de operación"""
        valid_modes = ['manual', 'exploration', 'formation', 'mapping']
        if mode in valid_modes:
            self.operation_mode = mode
            self.get_logger().info(f'Modo cambiado a: {mode}')
        else:
            self.get_logger().error(f'Modo inválido: {mode}. Modos válidos: {valid_modes}')

def main(args=None):
    rclpy.init(args=args)
    
    controller = AdvancedDualController()
    
    try:
        # Ejemplo de uso
        controller.takeoff_all(0.5)
        time.sleep(3)
        
        controller.set_operation_mode('exploration')
        
        rclpy.spin(controller)
        
    except KeyboardInterrupt:
        controller.get_logger().info('Deteniendo controlador...')
        controller.emergency_stop()
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
