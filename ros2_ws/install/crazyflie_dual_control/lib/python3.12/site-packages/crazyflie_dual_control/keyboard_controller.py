#!/usr/bin/env python3

"""
Controlador de teclado para 2 Crazyflies
Permite controlar ambos drones usando el teclado
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Empty, String
from crazyflie_interfaces.srv import Takeoff, Land
from crazyflie_interfaces.msg import Position, VelocityWorld
import sys
import select
import tty
import termios
import threading
import time

class KeyboardController(Node):
    """Controlador de teclado para múltiples drones"""
    
    def __init__(self):
        super().__init__('keyboard_controller')
        
        # IDs de los drones
        self.drone_ids = ['cf1', 'cf2']
        self.current_drone = 0  # Índice del drone actualmente controlado
        
        # Estados de los drones
        self.drone_positions = {
            'cf1': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0},
            'cf2': {'x': 1.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0}
        }
        
        self.drone_active = {
            'cf1': False,
            'cf2': False
        }
        
        # Parámetros de movimiento
        self.step_size = 0.1  # metros
        self.yaw_step = 0.1   # radianes
        self.max_height = 2.0
        self.min_height = 0.1
        
        # Clientes de servicios
        self.takeoff_clients = {}
        self.land_clients = {}
        
        # Publishers
        self.position_publishers = {}
        self.velocity_publishers = {}
        self.emergency_publisher = self.create_publisher(Empty, '/emergency_stop', 10)
        
        # Inicializar clientes y publishers
        for drone_id in self.drone_ids:
            # Servicios
            self.takeoff_clients[drone_id] = self.create_client(
                Takeoff, f'/{drone_id}/takeoff'
            )
            self.land_clients[drone_id] = self.create_client(
                Land, f'/{drone_id}/land'
            )
            
            # Publishers
            self.position_publishers[drone_id] = self.create_publisher(
                Position, f'/{drone_id}/cmd_position', 10
            )
            self.velocity_publishers[drone_id] = self.create_publisher(
                VelocityWorld, f'/{drone_id}/cmd_velocity_world', 10
            )
        
        # Configurar terminal para entrada de teclado
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
        
        # Timer para comandos periódicos
        self.timer = self.create_timer(0.1, self.publish_position_commands)
        
        # Thread para lectura de teclado
        self.keyboard_thread = threading.Thread(target=self.keyboard_listener)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()
        
        self.print_instructions()
        self.get_logger().info('Controlador de teclado iniciado')
    
    def print_instructions(self):
        """Imprime las instrucciones de control"""
        print("\n" + "="*60)
        print("CONTROLADOR DE TECLADO PARA 2 CRAZYFLIES")
        print("="*60)
        print("CONTROLES:")
        print("  WASD     - Mover drone actual (adelante/izq/atrás/der)")
        print("  QE       - Rotar drone actual (izq/der)")
        print("  RF       - Subir/Bajar drone actual")
        print("  T        - Despegar drone actual")
        print("  G        - Aterrizar drone actual")
        print("  TAB      - Cambiar drone (cf1 <-> cf2)")
        print("  SPACE    - Parada de emergencia (todos los drones)")
        print("  H        - Mostrar esta ayuda")
        print("  ESC/q    - Salir")
        print("="*60)
        print(f"Drone actual: {self.drone_ids[self.current_drone]}")
        print("="*60)
    
    def keyboard_listener(self):
        """Escucha las teclas presionadas"""
        while rclpy.ok():
            if select.select([sys.stdin], [], [], 0.1)[0]:
                key = sys.stdin.read(1)
                self.handle_key(key)
    
    def handle_key(self, key):
        """Maneja las teclas presionadas"""
        current_drone_id = self.drone_ids[self.current_drone]
        
        if key == '\x1b':  # ESC
            self.cleanup_and_exit()
        elif key == 'q':
            self.cleanup_and_exit()
        elif key == 'h':
            self.print_instructions()
        elif key == '\t':  # TAB
            self.switch_drone()
        elif key == ' ':  # SPACE
            self.emergency_stop()
        elif key == 't':
            self.takeoff_drone(current_drone_id)
        elif key == 'g':
            self.land_drone(current_drone_id)
        elif key == 'w':  # Adelante
            self.move_drone(current_drone_id, 'forward')
        elif key == 's':  # Atrás
            self.move_drone(current_drone_id, 'backward')
        elif key == 'a':  # Izquierda
            self.move_drone(current_drone_id, 'left')
        elif key == 'd':  # Derecha
            self.move_drone(current_drone_id, 'right')
        elif key == 'q':  # Rotar izquierda
            self.rotate_drone(current_drone_id, 'left')
        elif key == 'e':  # Rotar derecha
            self.rotate_drone(current_drone_id, 'right')
        elif key == 'r':  # Subir
            self.move_drone(current_drone_id, 'up')
        elif key == 'f':  # Bajar
            self.move_drone(current_drone_id, 'down')
    
    def switch_drone(self):
        """Cambia el drone actualmente controlado"""
        self.current_drone = (self.current_drone + 1) % len(self.drone_ids)
        current_drone_id = self.drone_ids[self.current_drone]
        print(f"\nCambiado a drone: {current_drone_id}")
        print(f"Posición actual: ({self.drone_positions[current_drone_id]['x']:.2f}, "
              f"{self.drone_positions[current_drone_id]['y']:.2f}, "
              f"{self.drone_positions[current_drone_id]['z']:.2f})")
    
    def move_drone(self, drone_id, direction):
        """Mueve el drone en la dirección especificada"""
        pos = self.drone_positions[drone_id]
        
        if direction == 'forward':
            pos['y'] += self.step_size
        elif direction == 'backward':
            pos['y'] -= self.step_size
        elif direction == 'left':
            pos['x'] -= self.step_size
        elif direction == 'right':
            pos['x'] += self.step_size
        elif direction == 'up':
            pos['z'] = min(pos['z'] + self.step_size, self.max_height)
        elif direction == 'down':
            pos['z'] = max(pos['z'] - self.step_size, self.min_height)
        
        print(f"{drone_id}: Posición ({pos['x']:.2f}, {pos['y']:.2f}, {pos['z']:.2f})")
    
    def rotate_drone(self, drone_id, direction):
        """Rota el drone"""
        pos = self.drone_positions[drone_id]
        
        if direction == 'left':
            pos['yaw'] -= self.yaw_step
        elif direction == 'right':
            pos['yaw'] += self.yaw_step
        
        # Normalizar ángulo
        while pos['yaw'] > 3.14159:
            pos['yaw'] -= 2 * 3.14159
        while pos['yaw'] < -3.14159:
            pos['yaw'] += 2 * 3.14159
        
        print(f"{drone_id}: Yaw {pos['yaw']:.2f} rad ({pos['yaw']*180/3.14159:.1f}°)")
    
    def takeoff_drone(self, drone_id):
        """Despega el drone especificado"""
        if not self.drone_active[drone_id]:
            req = Takeoff.Request()
            req.height = 0.5
            req.duration = rclpy.duration.Duration(seconds=2.0).to_msg()
            
            future = self.takeoff_clients[drone_id].call_async(req)
            self.drone_active[drone_id] = True
            print(f"{drone_id}: Despegando...")
        else:
            print(f"{drone_id}: Ya está en vuelo")
    
    def land_drone(self, drone_id):
        """Aterriza el drone especificado"""
        if self.drone_active[drone_id]:
            req = Land.Request()
            req.height = 0.0
            req.duration = rclpy.duration.Duration(seconds=2.0).to_msg()
            
            future = self.land_clients[drone_id].call_async(req)
            self.drone_active[drone_id] = False
            print(f"{drone_id}: Aterrizando...")
        else:
            print(f"{drone_id}: No está en vuelo")
    
    def emergency_stop(self):
        """Parada de emergencia para todos los drones"""
        print("\n¡PARADA DE EMERGENCIA!")
        msg = Empty()
        self.emergency_publisher.publish(msg)
        
        # Aterrizar todos los drones
        for drone_id in self.drone_ids:
            if self.drone_active[drone_id]:
                self.land_drone(drone_id)
    
    def publish_position_commands(self):
        """Publica comandos de posición para todos los drones activos"""
        for drone_id in self.drone_ids:
            if self.drone_active[drone_id]:
                pos = self.drone_positions[drone_id]
                
                msg = Position()
                msg.x = pos['x']
                msg.y = pos['y']
                msg.z = pos['z']
                msg.yaw = pos['yaw']
                
                self.position_publishers[drone_id].publish(msg)
    
    def cleanup_and_exit(self):
        """Limpia recursos y sale"""
        print("\nDeteniendo controlador...")
        
        # Restaurar configuración del terminal
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        
        # Parada de emergencia
        self.emergency_stop()
        
        # Destruir nodo
        self.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

def main(args=None):
    rclpy.init(args=args)
    
    controller = KeyboardController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.cleanup_and_exit()
    finally:
        if rclpy.ok():
            controller.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
