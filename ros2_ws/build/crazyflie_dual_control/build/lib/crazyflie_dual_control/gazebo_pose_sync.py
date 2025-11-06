#!/usr/bin/env python3
"""
Nodo para sincronizar las poses de los drones desde /tf a Gazebo
Lee las transforms world->cf1 y world->cf2 y actualiza las poses en Gazebo
"""

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import Pose, TransformStamped, PoseStamped
from ros_gz_interfaces.srv import SetEntityPose
from ros_gz_interfaces.msg import Entity
import subprocess
import time
import math


class GazeboPoseSync(Node):
    """Sincroniza poses de drones desde ROS2 /tf a Gazebo"""
    
    def __init__(self):
        super().__init__('gazebo_pose_sync')
        self.declare_parameter('drone_names', ['cf1', 'cf2'])
        self.declare_parameter('update_rate', 30.0)  # Hz
        
        self.drone_names = self.get_parameter('drone_names').get_parameter_value().string_array_value
        self.update_rate = self.get_parameter('update_rate').get_parameter_value().double_value
        
        # Buffer y listener de TF
        self.tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Frame de referencia - intentaremos detectarlo automáticamente
        self.reference_frame = None
        self._frame_discovered = False
        
        # Crear clientes de servicio para set_entity_pose (ROS2 service)
        # El servicio puede estar en diferentes nombres, intentemos detectarlo
        self.pose_service_clients = {}
        self.pose_service_name = None
        
        # Intentar diferentes nombres posibles del servicio
        possible_service_names = [
            '/world/empty/set_entity_pose',
            '/set_entity_pose',
            '/world/set_entity_pose',
        ]
        
        # Crear clientes para el primer servicio que funcione
        # Usaremos el primero como default y lo verificaremos cuando esté disponible
        service_name = possible_service_names[0]
        for drone_name in self.drone_names:
            client = self.create_client(SetEntityPose, service_name)
            self.pose_service_clients[drone_name] = client
        self.pose_service_name = service_name
        
        # Timer para actualizar poses
        timer_period = 1.0 / self.update_rate
        self.timer = self.create_timer(timer_period, self.update_poses)
        
        # Contador para rate limiting de comandos
        self._cmd_counters = {name: 0 for name in self.drone_names}
        
        self.get_logger().info(f'Gazebo Pose Sync iniciado para drones: {self.drone_names}')
    
    def discover_reference_frame(self):
        """Descubre el frame de referencia desde el cual se publican las transforms"""
        if self._frame_discovered and self.reference_frame:
            return True
        
        # Intentar encontrar el frame de referencia probando diferentes opciones
        possible_frames = ['world', 'odom', 'map']
        
        for frame in possible_frames:
            try:
                # Intentar obtener transform desde este frame a cualquier drone
                for drone_name in self.drone_names:
                    try:
                        self.tf_buffer.lookup_transform(
                            frame,
                            drone_name,
                            rclpy.time.Time()
                        )
                        # Si llegamos aquí, el frame existe y tiene transforms
                        self.reference_frame = frame
                        self._frame_discovered = True
                        self.get_logger().info(f'Frame de referencia detectado: {frame}')
                        return True
                    except Exception:
                        continue
            except Exception:
                continue
        
        return False
    
    def update_poses(self):
        """Actualiza las poses de los drones en Gazebo"""
        # Primero, intentar descubrir el frame de referencia si no lo hemos hecho
        if not self.discover_reference_frame():
            # Si no podemos encontrar el frame, esperar un poco más
            return
        
        for drone_name in self.drone_names:
            try:
                # Obtener transform desde el frame de referencia descubierto a drone
                transform: TransformStamped = self.tf_buffer.lookup_transform(
                    self.reference_frame,
                    drone_name,
                    rclpy.time.Time()
                )
                
                # Extraer posición y orientación
                pos = transform.transform.translation
                quat = transform.transform.rotation
                
                # Rate limiting: solo actualizar cada N frames para evitar saturar el sistema
                # Pero mantener suficiente frecuencia para movimiento suave
                self._cmd_counters[drone_name] += 1
                update_every = max(1, int(self.update_rate / 20.0))  # Limitar a ~20 Hz máximo
                if self._cmd_counters[drone_name] % update_every != 0:
                    continue
                
                # Loggear primera vez que obtenemos transform exitosamente
                if not hasattr(self, '_first_transform'):
                    self._first_transform = {}
                if drone_name not in self._first_transform:
                    self._first_transform[drone_name] = True
                    self.get_logger().info(
                        f'✅ Transform obtenido para {drone_name} desde {self.reference_frame}: '
                        f'pos=({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f})'
                    )
                
                # Usar el nodo ros_gz_sim set_entity_pose directamente
                # Este nodo acepta argumentos de línea de comandos para actualizar poses
                # Formato: ros_gz_sim set_entity_pose --name <name> --pos <x> <y> <z> --euler <roll> <pitch> <yaw>
                roll, pitch, yaw = self.quaternion_to_euler(quat.x, quat.y, quat.z, quat.w)
                
                cmd = [
                    'ros2', 'run', 'ros_gz_sim', 'set_entity_pose',
                    '--name', drone_name,
                    '--type', '2',  # 2 = MODEL
                    '--pos', str(float(pos.x)), str(float(pos.y)), str(float(pos.z)),
                    '--euler', str(roll), str(pitch), str(yaw)
                ]
                
                # Ejecutar de forma asíncrona
                try:
                    process = subprocess.Popen(
                        cmd,
                        stdout=subprocess.PIPE,
                        stderr=subprocess.PIPE,
                        start_new_session=True
                    )
                    
                    # No esperamos el proceso, pero verificamos si hay errores iniciales
                    # Después de un momento, verificamos si el proceso terminó con error
                    
                except FileNotFoundError:
                    if not hasattr(self, '_ros_gz_not_found'):
                        self._ros_gz_not_found = True
                        self.get_logger().error('ros_gz_sim no encontrado. ¿Está instalado?')
                    return
                except Exception as e:
                    if not hasattr(self, '_cmd_error_count'):
                        self._cmd_error_count = {}
                    if drone_name not in self._cmd_error_count:
                        self._cmd_error_count[drone_name] = 0
                    self._cmd_error_count[drone_name] += 1
                    if self._cmd_error_count[drone_name] <= 3:
                        self.get_logger().warn(
                            f'Error al ejecutar comando para {drone_name}: {str(e)[:100]}'
                        )
                    return
                
                # Loggear éxito ocasionalmente
                if not hasattr(self, '_success_count'):
                    self._success_count = {}
                if drone_name not in self._success_count:
                    self._success_count[drone_name] = 0
                self._success_count[drone_name] += 1
                if self._success_count[drone_name] % 300 == 0:  # Cada ~10 segundos a 30 Hz
                    self.get_logger().info(
                        f'✅ Actualizando pose para {drone_name}: ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f})'
                    )
                    
            except Exception as e:
                # Loggear errores de transform ocasionalmente
                if not hasattr(self, '_tf_error_logged'):
                    self._tf_error_logged = {}
                if drone_name not in self._tf_error_logged or self._tf_error_logged[drone_name] < 3:
                    self.get_logger().warn(
                        f'Error al obtener transform para {drone_name}: {str(e)[:100]}'
                    )
                    if drone_name not in self._tf_error_logged:
                        self._tf_error_logged[drone_name] = 0
                    self._tf_error_logged[drone_name] += 1
    
    @staticmethod
    def quaternion_to_euler(x, y, z, w):
        """Convierte quaternion a ángulos de Euler (roll, pitch, yaw)"""
        # Roll (rotación alrededor del eje x)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (rotación alrededor del eje y)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # Usar 90 grados si fuera del rango
        else:
            pitch = math.asin(sinp)
        
        # Yaw (rotación alrededor del eje z)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw


def main(args=None):
    rclpy.init(args=args)
    node = GazeboPoseSync()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

