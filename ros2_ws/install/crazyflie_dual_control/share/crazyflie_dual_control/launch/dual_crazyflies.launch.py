#!/usr/bin/env python3

"""
Archivo de lanzamiento simplificado para 2 Crazyflies
Usa directamente el launch file de Crazyswarm2 y agrega nuestros nodos
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """Genera la descripción de lanzamiento para 2 Crazyflies"""
    
    # Argumentos de lanzamiento
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('crazyflie_dual_control'),
            'config',
            'crazyflies.yaml'
        ]),
        description='Archivo de configuración de los drones'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Nivel de logging'
    )
    
    # Configuración de parámetros
    config_file = LaunchConfiguration('config_file')
    log_level = LaunchConfiguration('log_level')
    
    # Incluir el launch file completo de Crazyswarm2
    # Esto maneja el servidor y toda la configuración correctamente
    crazyswarm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('crazyflie'),
                'launch',
                'launch.py'
            ])
        ]),
        launch_arguments={
            'crazyflies_yaml_file': config_file,
            'backend': 'cpp',  # Usar backend C++ (más estable, evita problemas con NumPy)
            'teleop': 'False',  # No necesitamos teleop
            'gui': 'False',     # No necesitamos GUI
            'rviz': 'False',    # RViz opcional (lo controlamos nosotros)
            'mocap': 'False',   # Sin motion capture
        }.items()
    )
    
    # Nodo del controlador dual
    dual_controller_node = Node(
        package='crazyflie_dual_control',
        executable='dual_controller',
        name='dual_crazyflie_controller',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level]
    )
    
    # Nodo de monitoreo de batería
    battery_monitor_node = Node(
        package='crazyflie_dual_control',
        executable='battery_monitor',
        name='battery_monitor',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level]
    )
    
    return LaunchDescription([
        config_file_arg,
        log_level_arg,
        crazyswarm_launch,  # Incluye el servidor de Crazyswarm2
        dual_controller_node,
        battery_monitor_node,
    ])