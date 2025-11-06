#!/usr/bin/env python3

"""
Archivo de lanzamiento para simulación de 2 Crazyflies en Gazebo
Incluye multiranger y capacidades de mapeo
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os
import yaml
import xml.etree.ElementTree as ET

def generate_launch_description():
    """Genera la descripción de lanzamiento para simulación de 2 Crazyflies"""
    
    # Argumentos de lanzamiento
    world_file_arg = DeclareLaunchArgument(
        'world_file',
        default_value='empty_world.sdf',
        description='Archivo de mundo de Gazebo'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Usar tiempo simulado'
    )
    
    enable_multiranger_arg = DeclareLaunchArgument(
        'enable_multiranger',
        default_value='true',
        description='Habilitar multiranger'
    )
    
    enable_mapping_arg = DeclareLaunchArgument(
        'enable_mapping',
        default_value='true',
        description='Habilitar mapeo'
    )
    
    enable_rviz_arg = DeclareLaunchArgument(
        'enable_rviz',
        default_value='true',
        description='Habilitar RViz'
    )
    
    # Configuración de parámetros
    world_file = LaunchConfiguration('world_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_multiranger = LaunchConfiguration('enable_multiranger')
    enable_mapping = LaunchConfiguration('enable_mapping')
    enable_rviz = LaunchConfiguration('enable_rviz')
    
    # Calcular rutas del workspace necesarias para modelos de Gazebo
    pkg_share = get_package_share_directory('crazyflie_dual_control')
    workspace_path = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(pkg_share))))
    workspace_src_path = os.path.join(workspace_path, 'src')
    model_base_path = os.path.join(workspace_src_path, 'ros_gz_crazyflie', 'ros_gz_crazyflie_gazebo')
    
    # Configurar GZ_SIM_RESOURCE_PATH para que Gazebo encuentre los modelos y meshes
    gazebo_env = dict(os.environ)
    current_gz_resource = gazebo_env.get('GZ_SIM_RESOURCE_PATH', '')
    if current_gz_resource:
        gazebo_env['GZ_SIM_RESOURCE_PATH'] = f"{model_base_path}:{current_gz_resource}"
    else:
        gazebo_env['GZ_SIM_RESOURCE_PATH'] = f"{model_base_path}:/opt/ros/jazzy/share"
    
    # Deshabilitar SHM en DDS para evitar errores de FastDDS en contenedores
    ddsenv = {
        'RMW_FASTRTPS_USE_SHM': '0',
        'RMW_IMPLEMENTATION': os.environ.get('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
    }

    # Lanzar Gazebo (usando world completo de Gazebo)
    gazebo_env.update(ddsenv)
    gazebo_launch = ExecuteProcess(
        cmd=['gz', 'sim', '-v', '4', '-r', 'empty.sdf'],
        output='screen',
        env=gazebo_env
    )
    
    # Nodo del servidor Crazyflie para simulación
    # Usa el paquete 'crazyflie_sim' para simulación (no 'crazyflie' que es para hardware real)
    # Necesita PYTHONPATH con cffirmware para funcionar
    # El servidor de simulación espera parámetros directamente (no desde archivo ROS2)
    firmware_path = os.path.expanduser('~/crazyflie-firmware/build')
    # Copiar todo el entorno actual y agregar al PYTHONPATH
    server_env = dict(os.environ)
    current_pythonpath = server_env.get('PYTHONPATH', '')
    if current_pythonpath:
        server_env['PYTHONPATH'] = f"{firmware_path}:{current_pythonpath}"
    else:
        server_env['PYTHONPATH'] = firmware_path
    
    # Cargar el archivo YAML y convertirlo en parámetros
    crazyflies_yaml_path = PathJoinSubstitution([
        FindPackageShare('crazyflie_dual_control'),
        'config',
        'crazyflies_simulation.yaml'
    ])
    # Nota: PathJoinSubstitution no se puede evaluar directamente aquí, así que usamos
    # el método estándar de ROS2 para pasar parámetros
    crazyflies_config_path = os.path.join(
        get_package_share_directory('crazyflie_dual_control'),
        'config',
        'crazyflies_simulation.yaml'
    )
    
    # Cargar YAML y pasarlo como parámetros (el servidor usa automatically_declare_parameters_from_overrides)
    with open(crazyflies_config_path, 'r') as file:
        crazyflies_config = yaml.safe_load(file)
    
    # Cargar el URDF del Crazyflie (requerido por el servidor de simulación)
    urdf_path = os.path.join(
        get_package_share_directory('crazyflie'),
        'urdf',
        'crazyflie_description.urdf'
    )
    with open(urdf_path, 'r') as f:
        robot_description = f.read()
    
    # Agregar robot_description a la configuración
    crazyflies_config['robot_description'] = robot_description
    
    # Convertir el diccionario YAML en parámetros ROS2
    # El servidor espera los parámetros directamente en el diccionario
    server_params = [crazyflies_config, {'use_sim_time': use_sim_time}]
    
    # Propagar variables DDS a los nodos ROS2
    server_env.update(ddsenv)
    crazyflie_server_node = Node(
        package='crazyflie_sim',
        executable='crazyflie_server',
        name='crazyflie_server',
        parameters=server_params,
        output='screen',
        emulate_tty=True,
        env=server_env
    )
    
    # Nota: El servidor de simulación ya publica:
    # - Transforms dinámicas: 'world' -> 'cf1' y 'world' -> 'cf2'
    # - robot_description en tópicos: '/cf1/robot_description' y '/cf2/robot_description'
    # RViz se suscribe directamente a estos tópicos para visualizar los drones
    
    # Spawnear modelos de Crazyflie en Gazebo
    # Cargar modelo SDF y modificarlo para cada drone
    # Las rutas del workspace ya fueron calculadas arriba
    # Lista de rutas posibles para el modelo SDF
    possible_paths = [
        os.path.join(workspace_src_path, 'ros_gz_crazyflie', 'ros_gz_crazyflie_gazebo', 'models', 'crazyflie', 'model.sdf'),
        os.path.join(workspace_src_path, 'crazyflie-simulation', 'simulator_files', 'gazebo', 'crazyflie', 'model.sdf'),
    ]
    
    # Buscar la primera ruta que exista
    sdf_model_path = None
    for path in possible_paths:
        if os.path.exists(path):
            sdf_model_path = path
            break
    
    if sdf_model_path is None:
        # Si no se encuentra, intentar construir desde ros_gz_crazyflie_gazebo si está disponible
        try:
            pkg_path = get_package_share_directory('ros_gz_crazyflie_gazebo')
            sdf_model_path = os.path.join(pkg_path, 'models', 'crazyflie', 'model.sdf')
            if not os.path.exists(sdf_model_path):
                raise FileNotFoundError(f"Modelo SDF no encontrado en ninguna ubicación esperada")
        except Exception:
            raise FileNotFoundError(
                f"No se encontró el modelo SDF. Buscado en:\n" +
                "\n".join([f"  - {p}" for p in possible_paths])
            )
    
    # Leer posiciones iniciales desde la configuración
    initial_positions = {}
    for cf_name, cf_data in crazyflies_config.get('robots', {}).items():
        if cf_data.get('enabled', False):
            initial_pos = cf_data.get('initial_position', [0.0, 0.0, 0.05])
            initial_positions[cf_name] = initial_pos
    
    # Crear nodos para spawnear cada drone
    spawn_nodes = []
    for cf_name, initial_pos in initial_positions.items():
        # Leer y modificar el SDF para este drone
        tree = ET.parse(sdf_model_path)
        root = tree.getroot()
        model_elem = root.find('.//model')
        if model_elem is not None:
            # Cambiar el nombre del modelo
            model_elem.set('name', cf_name)
            
            # NO hacer el modelo estático porque los modelos estáticos NO pueden ser movidos con set_pose
            # En su lugar, removemos los plugins de física para evitar interferencias,
            # pero permitimos que set_pose funcione
            # Si el modelo tiene <static>, removerlo para permitir movimiento con set_pose
            static_elem = model_elem.find('static')
            if static_elem is not None:
                model_elem.remove(static_elem)
            
            # Desactivar/eliminar TODOS los plugins que interfieren con el control manual de poses
            # Esto incluye plugins de física, motores, control y odometría
            # Ya que solo queremos visualización, no necesitamos estos plugins
            plugins_to_remove = []
            for plugin in model_elem.findall('plugin'):
                filename = plugin.get('filename', '')
                name_attr = plugin.get('name', '')
                # Remover todos los plugins que podrían interferir
                if any(phys_plugin in filename for phys_plugin in [
                    'multicopter-motor-model',
                    'multicopter-control',
                    'odometry-publisher',  # También remover odometría que podría interferir
                ]) or any(phys_name in name_attr for phys_name in [
                    'MulticopterMotorModel',
                    'MulticopterVelocityControl',
                    'OdometryPublisher',
                ]):
                    plugins_to_remove.append(plugin)
            
            # Eliminar los plugins problemáticos
            for plugin in plugins_to_remove:
                model_elem.remove(plugin)
            
            # Para modelos estáticos, también podemos desactivar los joints para evitar movimiento no deseado
            # Esto asegura que las hélices no giren y no haya fuerzas aplicadas
            for joint in model_elem.findall('joint'):
                # Agregar atributo stop_publish para joints en modelos estáticos (opcional)
                # O simplemente dejar que el modelo estático los maneje
                pass  # Los joints en modelos estáticos ya no aplican fuerzas
            
            # Actualizar los namespaces en los plugins restantes (si los hay)
            for plugin in root.findall('.//plugin'):
                robot_ns = plugin.find('robotNamespace')
                if robot_ns is not None:
                    robot_ns.text = cf_name
            
            # Actualizar nombres de joints ANTES de actualizar referencias child/parent
            for joint in root.findall('.//joint'):
                joint_name = joint.get('name', '')
                if joint_name and 'crazyflie' in joint_name:
                    joint.set('name', joint_name.replace('crazyflie', cf_name))
            
            # Actualizar referencias a crazyflie/* a cf_name/* en todos los elementos
            for elem in root.iter():
                # Actualizar texto que contiene crazyflie/
                if elem.text and 'crazyflie/' in elem.text:
                    elem.text = elem.text.replace('crazyflie/', f'{cf_name}/')
                # Actualizar atributos que contienen crazyflie
                for attr_name, attr_value in list(elem.attrib.items()):
                    if 'crazyflie' in attr_value:
                        elem.attrib[attr_name] = attr_value.replace('crazyflie', cf_name)
                # Actualizar tags child y parent
                if elem.tag in ['child', 'parent'] and elem.text and 'crazyflie' in elem.text:
                    elem.text = elem.text.replace('crazyflie', cf_name)
            
            # Actualizar rutas de meshes a rutas absolutas para que Gazebo las encuentre
            # cuando el SDF se pasa como string
            model_mesh_dir = os.path.join(model_base_path, 'models', 'crazyflie', 'meshes')
            for mesh_elem in root.findall('.//mesh'):
                uri_elem = mesh_elem.find('uri')
                if uri_elem is not None and uri_elem.text:
                    # Si es una ruta relativa, convertirla a absoluta
                    if uri_elem.text.startswith('meshes/'):
                        mesh_filename = uri_elem.text.replace('meshes/', '')
                        mesh_abs_path = os.path.join(model_mesh_dir, mesh_filename)
                        # Usar file:// para rutas absolutas en Gazebo
                        uri_elem.text = f"file://{mesh_abs_path}"
        
        # Convertir a string
        sdf_string = ET.tostring(root, encoding='unicode')
        
        # Debug: guardar SDF modificado para inspección
        # import tempfile
        # with tempfile.NamedTemporaryFile(mode='w', suffix='.sdf', delete=False) as f:
        #     f.write(sdf_string)
        #     self.get_logger().info(f'SDF guardado en {f.name}')
        
        # Crear nodo de spawn con delay para asegurar que Gazebo esté listo
        # El mundo se llama 'empty' según el comando 'gz sim -r empty.sdf'
        spawn_node = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-world', 'empty',  # Nombre del mundo (no 'default')
                '-name', cf_name,
                '-string', sdf_string,
                '-x', str(initial_pos[0]),
                '-y', str(initial_pos[1]),
                '-z', str(initial_pos[2]),
            ],
            output='screen',
            env=ddsenv
        )
        
        # Envolver el nodo de spawn en un TimerAction para dar tiempo a que Gazebo se inicialice
        spawn_timer = TimerAction(
            period=3.0,  # Esperar 3 segundos antes de spawnear
            actions=[spawn_node]
        )
        spawn_nodes.append(spawn_timer)
    
    # Nodo para sincronizar poses desde /tf a Gazebo
    gazebo_pose_sync_node = Node(
        package='crazyflie_dual_control',
        executable='gazebo_pose_sync',
        name='gazebo_pose_sync',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'drone_names': list(initial_positions.keys())},
            {'update_rate': 15.0}
        ],
        output='screen',
        env=ddsenv
    )
    
    # Nodo del controlador dual
    dual_controller_node = Node(
        package='crazyflie_dual_control',
        executable='dual_controller',
        name='dual_crazyflie_controller',
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        output='screen',
        env=ddsenv
    )
    
    # Nodo de mapeo simple para cf1
    mapper_cf1_node = Node(
        package='crazyflie_ros2_multiranger_simple_mapper',
        executable='simple_mapper_multiranger',
        name='simple_mapper_cf1',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'drone_id': 'cf1'}
        ],
        output='screen',
        env=ddsenv,
        condition=IfCondition(enable_mapping)
    )
    
    # Nodo de mapeo simple para cf2
    mapper_cf2_node = Node(
        package='crazyflie_ros2_multiranger_simple_mapper',
        executable='simple_mapper_multiranger',
        name='simple_mapper_cf2',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'drone_id': 'cf2'}
        ],
        output='screen',
        env=ddsenv,
        condition=IfCondition(enable_mapping)
    )
    
    # Nodo de seguimiento de pared para cf1
    # Solo se ejecuta si tanto multiranger como mapping están habilitados
    wall_follower_cf1_node = Node(
        package='crazyflie_ros2_multiranger_wall_following',
        executable='wall_following_multiranger',
        name='wall_follower_cf1',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'drone_id': 'cf1'}
        ],
        output='screen',
        env=ddsenv,
        condition=IfCondition(PythonExpression([
            "'", enable_multiranger, "' == 'true' and '", enable_mapping, "' == 'true'"
        ]))
    )
    
    # Nodo de seguimiento de pared para cf2
    # Solo se ejecuta si tanto multiranger como mapping están habilitados
    wall_follower_cf2_node = Node(
        package='crazyflie_ros2_multiranger_wall_following',
        executable='wall_following_multiranger',
        name='wall_follower_cf2',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'drone_id': 'cf2'}
        ],
        output='screen',
        env=ddsenv,
        condition=IfCondition(PythonExpression([
            "'", enable_multiranger, "' == 'true' and '", enable_mapping, "' == 'true'"
        ]))
    )
    
    # RViz para visualización
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('crazyflie_dual_control'),
            'rviz',
            'dual_crazyflies_simulation.rviz'
        ])],
        condition=IfCondition(enable_rviz),
        output='screen',
        env=ddsenv
    )
    
    # Nodo de monitoreo de batería
    battery_monitor_node = Node(
        package='crazyflie_dual_control',
        executable='battery_monitor',
        name='battery_monitor',
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        output='screen',
        env=ddsenv
    )
    
    return LaunchDescription([
        world_file_arg,
        use_sim_time_arg,
        enable_multiranger_arg,
        enable_mapping_arg,
        enable_rviz_arg,
        gazebo_launch,
        crazyflie_server_node,
        *spawn_nodes,  # Spawnear todos los drones
        gazebo_pose_sync_node,  # Reactivado con rate moderado
        dual_controller_node,
        mapper_cf1_node,
        mapper_cf2_node,
        wall_follower_cf1_node,
        wall_follower_cf2_node,
        battery_monitor_node,
        rviz_node
    ])
