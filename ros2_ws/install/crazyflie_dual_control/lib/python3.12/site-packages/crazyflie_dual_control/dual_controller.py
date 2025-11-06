#!/usr/bin/env python3

"""
Script principal para controlar 2 Crazyflies simultáneamente
Usa Crazyswarm2 para comunicación y control de vuelo
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Empty, String
from crazyflie_interfaces.srv import Takeoff, Land, GoTo, Arm
from crazyflie_interfaces.msg import Position, VelocityWorld, Status
import time
import math

class DualCrazyflieController(Node):
    """Controlador para 2 Crazyflies"""
    
    def __init__(self):
        super().__init__('dual_crazyflie_controller')
        
        # IDs de los drones (se descubrirán automáticamente)
        self.drone_ids = ['cf1', 'cf2']  # Valores por defecto
        
        # Clientes de servicios para cada drone (se crearán después)
        self.takeoff_clients = {}
        self.land_clients = {}
        self.goto_clients = {}
        self.arm_clients = {}
        
        # Publishers para comandos de posición y velocidad (se crearán después)
        self.position_publishers = {}
        self.velocity_publishers = {}
        
        # Publisher para comandos de emergencia
        self.emergency_publisher = self.create_publisher(
            Empty, '/emergency_stop', 10
        )
        
        # Timer para comandos periódicos
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # Estado de los drones
        self.drone_states = {
            'cf1': {'position': [0.0, 0.0, 0.0], 'active': False},
            'cf2': {'position': [0.0, 0.0, 0.0], 'active': False}
        }
        
        # Suscriptores de estado (se crearán después de descubrir los nombres)
        self.status_subscribers = {}
        self.drone_status = {}  # Almacenar estado de cada drone
        
        self.get_logger().info('Controlador Dual Crazyflie iniciado')
    
    def status_callback(self, msg, drone_id):
        """Callback para recibir mensajes de estado de los drones"""
        self.drone_status[drone_id] = msg
    
    def check_auto_arm(self, drone_id):
        """Verifica si el auto-arm está activo"""
        if drone_id not in self.drone_status or self.drone_status[drone_id] is None:
            return None
        
        status = self.drone_status[drone_id]
        # SUPERVISOR_INFO_AUTO_ARM = 4
        auto_arm_active = bool(status.supervisor_info & 4)
        return auto_arm_active
        
    def discover_robot_names(self):
        """Descubre los nombres reales de los robots desde los servicios disponibles"""
        self.get_logger().info('Descubriendo nombres de robots...')
        
        try:
            # Buscar servicios de takeoff para encontrar nombres de robots
            # Esperar un poco para que los servicios estén disponibles
            time.sleep(5.0)  # Dar tiempo al servidor para iniciar (aumentado de 2.0 a 5.0)
            
            # get_service_names_and_types() retorna lista de tuplas (nombre, [tipos])
            service_list = self.get_service_names_and_types()
            
            robot_names = []
            for srv_name, srv_types in service_list:
                if '/takeoff' in srv_name and srv_name != '/all/takeoff':
                    # Verificar que sea un servicio de takeoff de crazyflie
                    if any('crazyflie_interfaces/srv/Takeoff' in str(t) for t in srv_types):
                        # Extraer nombre del robot (ej: /cf1/takeoff -> cf1)
                        robot_name = srv_name.split('/takeoff')[0].strip('/')
                        if robot_name and robot_name != 'all' and robot_name not in robot_names:
                            robot_names.append(robot_name)
                            self.get_logger().info(f'✅ Robot descubierto: {robot_name}')
            
            if robot_names:
                self.drone_ids = robot_names[:2]  # Tomar los primeros 2
                self.get_logger().info(f'✅ Usando robots: {self.drone_ids}')
                return True
            else:
                self.get_logger().warn('⚠️  No se encontraron robots en los servicios disponibles')
                self.get_logger().warn('   Posibles causas:')
                self.get_logger().warn('   1. El servidor no está corriendo o falló al iniciar')
                self.get_logger().warn('   2. Los drones no están conectados/configurados')
                self.get_logger().warn('   3. El servidor no detectó la antena USB ("No Crazyradio dongle found!")')
                self.get_logger().warn('   Verifica los logs del servidor para más detalles')
                self.get_logger().warn('   Usando nombres por defecto: cf1, cf2')
                return False
        except Exception as e:
            self.get_logger().error(f'Error descubriendo robots: {e}')
            import traceback
            self.get_logger().debug(f'Traceback: {traceback.format_exc()}')
            self.get_logger().warn('Usando nombres por defecto: cf1, cf2')
            return False
    
    def wait_for_services(self):
        """Espera a que todos los servicios estén disponibles"""
        # Primero descubrir los nombres reales de los robots
        self.discover_robot_names()
        
        # Recrear clientes con los nombres correctos
        self.takeoff_clients = {}
        self.land_clients = {}
        self.goto_clients = {}
        self.arm_clients = {}
        self.position_publishers = {}
        self.velocity_publishers = {}
        
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
            self.arm_clients[drone_id] = self.create_client(
                Arm, f'/{drone_id}/arm'
            )
            
            # Publishers
            self.position_publishers[drone_id] = self.create_publisher(
                Position, f'/{drone_id}/cmd_position', 10
            )
            self.velocity_publishers[drone_id] = self.create_publisher(
                VelocityWorld, f'/{drone_id}/cmd_velocity_world', 10
            )
            
            # Suscriptores de estado para verificar auto-arm
            self.status_subscribers[drone_id] = self.create_subscription(
                Status,
                f'/{drone_id}/status',
                lambda msg, d=drone_id: self.status_callback(msg, d),
                10
            )
            self.drone_status[drone_id] = None
        
        # Esperar servicios con timeout más largo (aumentado de 10.0 a 20.0 segundos)
        all_available = True
        for drone_id in self.drone_ids:
            self.get_logger().info(f'Esperando servicios para {drone_id}...')
            
            # Esperar servicios de takeoff
            if not self.takeoff_clients[drone_id].wait_for_service(timeout_sec=20.0):
                self.get_logger().error(f'Servicio takeoff no disponible para {drone_id}')
                self.get_logger().error(f'Verifica que el servidor esté corriendo y que {drone_id} esté en el archivo de configuración')
                all_available = False
                continue
                
            # Esperar servicios de land
            if not self.land_clients[drone_id].wait_for_service(timeout_sec=20.0):
                self.get_logger().error(f'Servicio land no disponible para {drone_id}')
                all_available = False
                continue
                
            # Esperar servicios de goto
            if not self.goto_clients[drone_id].wait_for_service(timeout_sec=20.0):
                self.get_logger().error(f'Servicio goto no disponible para {drone_id}')
                all_available = False
                continue
            
            # Esperar servicios de arm (opcional, puede no estar disponible)
            if not self.arm_clients[drone_id].wait_for_service(timeout_sec=5.0):
                self.get_logger().warn(f'Servicio arm no disponible para {drone_id} (puede ser normal)')
            else:
                self.get_logger().info(f'Servicio arm disponible para {drone_id}')
            
            self.get_logger().info(f'✅ Servicios disponibles para {drone_id}')
                
        if all_available:
            self.get_logger().info('✅ Todos los servicios están disponibles')
        else:
            self.get_logger().warn('⚠️  Algunos servicios no están disponibles')
            
        return all_available
    
    def arm_all(self):
        """Arma todos los drones (habilita los motores)"""
        self.get_logger().info('=' * 60)
        self.get_logger().info('ARMANDO TODOS LOS DRONES...')
        self.get_logger().info('=' * 60)
        
        futures = []
        arms_sent = 0
        
        for drone_id in self.drone_ids:
            if drone_id not in self.arm_clients:
                self.get_logger().error(f'❌ Cliente arm no existe para {drone_id}')
                continue
                
            if not self.arm_clients[drone_id].service_is_ready():
                self.get_logger().error(f'❌ Servicio arm no está listo para {drone_id}')
                self.get_logger().error(f'   Verifica que el servidor esté corriendo y que {drone_id} esté conectado')
                continue
            
            try:
                req = Arm.Request()
                req.arm = True
                future = self.arm_clients[drone_id].call_async(req)
                futures.append((drone_id, future))
                arms_sent += 1
                self.get_logger().info(f'  → Comando de armado enviado a {drone_id}')
            except Exception as e:
                self.get_logger().error(f'❌ Error enviando comando arm a {drone_id}: {e}')
        
        if arms_sent == 0:
            self.get_logger().error('❌ NO SE ENVIÓ NINGÚN COMANDO DE ARMADO')
            self.get_logger().error('   Verifica que los servicios arm estén disponibles')
            return False
        
        # Esperar respuestas
        all_armed = True
        for drone_id, future in futures:
            self.get_logger().info(f'  Esperando respuesta de armado de {drone_id}...')
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)  # Aumentado a 10 segundos
            if future.done():
                try:
                    response = future.result()
                    self.get_logger().info(f'  ✅ {drone_id} ARMADO EXITOSAMENTE')
                    self.get_logger().info(f'     Respuesta del servicio recibida correctamente')
                    # El servicio arm no retorna información útil, solo confirma que se envió el comando
                    # El drone debería estar armado ahora, pero no hay forma de verificar el estado directamente
                except Exception as e:
                    self.get_logger().error(f'  ❌ Error en respuesta de armado de {drone_id}: {e}')
                    self.get_logger().error(f'     Tipo de error: {type(e).__name__}')
                    import traceback
                    self.get_logger().debug(f'     Traceback: {traceback.format_exc()}')
                    all_armed = False
            else:
                self.get_logger().error(f'  ❌ TIMEOUT esperando respuesta de armado de {drone_id}')
                self.get_logger().error(f'     El servicio no respondió en 10 segundos')
                self.get_logger().error(f'     Posibles causas:')
                self.get_logger().error(f'     1. El servidor no está procesando el comando')
                self.get_logger().error(f'     2. El drone no está conectado o no responde')
                self.get_logger().error(f'     3. Problema de comunicación con el drone')
                all_armed = False
        
        if all_armed:
            self.get_logger().info('✅ TODOS LOS DRONES ARMADOS CORRECTAMENTE')
        else:
            self.get_logger().error('❌ ALGUNOS DRONES NO SE ARMARON')
        
        # Esperar un poco para que los motores se estabilicen
        # Nota: Este tiempo puede ser aumentado en takeoff_all() si es hardware real
        self.get_logger().info('⏳ Esperando 2 segundos para estabilización de motores...')
        time.sleep(2.0)
        
        return all_armed
    
    def takeoff_all(self, height=1.0):
        """Despega todos los drones a la altura especificada"""
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'INICIANDO DESPEGUE DE TODOS LOS DRONES A {height}m')
        self.get_logger().info('=' * 60)
        
        # Primero verificar que los servicios de takeoff estén disponibles
        self.get_logger().info('\n📋 VERIFICANDO SERVICIOS...')
        for drone_id in self.drone_ids:
            if drone_id not in self.takeoff_clients:
                self.get_logger().error(f'❌ Cliente takeoff no existe para {drone_id}')
                return False
            # Verificar múltiples veces con espera si es necesario
            max_retries = 3
            service_ready = False
            for retry in range(max_retries):
                if self.takeoff_clients[drone_id].service_is_ready():
                    service_ready = True
                    break
                else:
                    if retry < max_retries - 1:
                        self.get_logger().warn(f'  ⚠️  Servicio takeoff no listo para {drone_id}, reintentando en 1 segundo... (intento {retry + 1}/{max_retries})')
                        time.sleep(1.0)
            
            if not service_ready:
                self.get_logger().error(f'❌ Servicio takeoff no está listo para {drone_id} después de {max_retries} intentos')
                self.get_logger().error(f'   Verifica que el servidor esté corriendo y que {drone_id} esté conectado')
                return False
            self.get_logger().info(f'  ✅ Servicio takeoff disponible para {drone_id}')
        
        # Paso 0: Verificar que los drones estén listos
        self.get_logger().info('\n📋 PASO 0: VERIFICACIONES PRE-VUELO...')
        self.get_logger().info('   ⚠️  IMPORTANTE: Verifica que:')
        self.get_logger().info('      1. Los drones estén encendidos y con batería cargada')
        self.get_logger().info('      2. Los drones estén en una superficie plana')
        self.get_logger().info('      3. No haya objetos cerca que interfieran')
        self.get_logger().info('      4. Las URIs en crazyflies.yaml sean correctas')
        time.sleep(1.0)  # Dar tiempo para leer
        
        # Paso 1: Armar los drones
        # Detectar si el servicio arm está disponible para determinar si es hardware real o simulación
        arm_service_available = False
        for drone_id in self.drone_ids:
            if drone_id in self.arm_clients:
                if self.arm_clients[drone_id].service_is_ready():
                    arm_service_available = True
                    break
        
        if arm_service_available:
            self.get_logger().info('\n📋 PASO 1: ARMANDO DRONES (OBLIGATORIO para hardware real)...')
            self.get_logger().info('   Detectado: Servicio arm disponible → Hardware real')
            
            # Verificar si el auto-arm está activo (solo informativo, no bloquea)
            time.sleep(1.0)  # Esperar un poco para recibir mensajes de estado
            auto_arm_active = False
            for drone_id in self.drone_ids:
                auto_arm = self.check_auto_arm(drone_id)
                if auto_arm is True:
                    self.get_logger().info(f'   ℹ️  {drone_id}: AUTO-ARM está ACTIVO')
                    self.get_logger().info(f'      El drone se armará automáticamente cuando esté listo')
                    self.get_logger().info(f'      Intentando takeoff con auto-arm activo...')
                    auto_arm_active = True
            
            # Si el auto-arm está activo, NO hacer arm manual
            # El drone se armará automáticamente cuando esté listo para volar
            if auto_arm_active:
                self.get_logger().info('   ✅ AUTO-ARM ACTIVO: Los drones se armarán automáticamente')
                self.get_logger().info('   ⏳ Esperando que el auto-arm arme los drones...')
                self.get_logger().info('   → No se llamará al servicio arm manualmente')
                time.sleep(5.0)  # Dar tiempo suficiente para que el auto-arm arme los drones
                # No llamamos arm_all() porque el auto-arm lo hará automáticamente
                arm_success = True  # Asumimos éxito si auto-arm está activo
                self.get_logger().info('   ✅ Auto-arm debería haber armado los drones')
                # Esperar un poco más para estabilización de motores (si el auto-arm los armó)
                self.get_logger().info('⏳ Esperando 3 segundos adicionales para estabilización de motores...')
                self.get_logger().info('   IMPORTANTE: Los motores deberían estar girando ahora (gracias al auto-arm)')
                time.sleep(3.0)
            else:
                # Si no hay auto-arm, armar manualmente
                self.get_logger().info('   ℹ️  AUTO-ARM NO ACTIVO: Armar manualmente')
                arm_success = self.arm_all()
                if not arm_success:
                    # Para hardware real, el arm es OBLIGATORIO
                    self.get_logger().error('❌ ERROR CRÍTICO: No se pudo armar los drones')
                    self.get_logger().error('   Para drones reales, el arm es OBLIGATORIO antes del takeoff')
                    self.get_logger().error('   Sin arm, los motores no se habilitan y el drone NO puede despegar')
                    self.get_logger().error('')
                    self.get_logger().error('   🔍 DIAGNÓSTICO:')
                    self.get_logger().error('   1. Verifica que los drones estén CONECTADOS y ENCENDIDOS')
                    self.get_logger().error('   2. Verifica que las URIs en crazyflies.yaml sean CORRECTAS')
                    self.get_logger().error('   3. Verifica que el servidor detecte los drones (revisa logs del servidor)')
                    self.get_logger().error('   4. Verifica que los drones tengan BATERÍA SUFICIENTE')
                    self.get_logger().error('   5. Verifica que los drones estén a menos de 5 metros de la antena')
                    self.get_logger().error('   6. Intenta armar manualmente: ros2 service call /cf1/arm crazyflie_interfaces/srv/Arm "{arm: true}"')
                    self.get_logger().error('')
                    self.get_logger().error('   ❌ ABORTANDO takeoff - No se puede continuar sin arm exitoso')
                    return False
                else:
                    self.get_logger().info('✅ Drones armados correctamente (manual)')
                    # Esperar más tiempo para que los motores se estabilicen (especialmente importante para drones reales)
                    self.get_logger().info('⏳ Esperando 5 segundos adicionales para estabilización de motores...')
                    self.get_logger().info('   IMPORTANTE: Los motores deberían estar girando ahora')
                    self.get_logger().info('   Si no escuchas los motores girando, el arm puede no haber funcionado')
                    time.sleep(5.0)  # Aumentado a 5 segundos para dar más tiempo a los motores
        else:
            # En simulación, el servicio arm puede no estar disponible
            self.get_logger().info('\n📋 PASO 1: ARMANDO DRONES (opcional en simulación)...')
            self.get_logger().info('   Detectado: Servicio arm NO disponible → Modo simulación')
            arm_success = self.arm_all()
            if not arm_success:
                self.get_logger().warn('⚠️  No se pudo armar los drones')
                self.get_logger().warn('   En simulación, el servicio arm puede no estar disponible')
                self.get_logger().warn('   Continuando con takeoff de todas formas...')
            else:
                self.get_logger().info('✅ Drones armados correctamente')
                time.sleep(2.0)  # Tiempo estándar para simulación
        
        # Paso 2: Enviar comandos de despegue
        self.get_logger().info('\n📋 PASO 2: ENVIANDO COMANDOS DE DESPEGUE...')
        futures = []
        takeoffs_sent = 0
        
        # Duración del takeoff: tiempo suficiente para despegar del piso
        # NOTA: Una duración más larga da más tiempo para que los motores aceleren
        # Para drones reales, puede necesitar más tiempo para acelerar los motores
        takeoff_duration = 7.0  # Aumentado de 5.0 a 7.0 segundos para dar más tiempo a los motores
        
        for drone_id in self.drone_ids:
            try:
                req = Takeoff.Request()
                req.height = float(height)
                req.duration = rclpy.duration.Duration(seconds=takeoff_duration).to_msg()
                req.group_mask = 0
                
                # Verificar que el servicio esté listo antes de enviar
                if not self.takeoff_clients[drone_id].service_is_ready():
                    self.get_logger().error(f'❌ Servicio takeoff no está listo para {drone_id} en el momento del envío')
                    continue
                
                # Enviar comando de despegue y guardar el future
                future = self.takeoff_clients[drone_id].call_async(req)
                futures.append((drone_id, future))
                takeoffs_sent += 1
                self.get_logger().info(f'  → Comando de despegue enviado a {drone_id} (altura: {height}m, duración: {takeoff_duration}s)')
            except Exception as e:
                self.get_logger().error(f'  ❌ Error enviando comando takeoff a {drone_id}: {e}')
        
        if takeoffs_sent == 0:
            self.get_logger().error('❌ NO SE ENVIÓ NINGÚN COMANDO DE DESPEGUE')
            return False
        
        # Paso 3: Esperar respuestas de los servicios
        self.get_logger().info('\n📋 PASO 3: ESPERANDO CONFIRMACIÓN DE DESPEGUE...')
        all_success = True
        for drone_id, future in futures:
            self.get_logger().info(f'  Esperando respuesta de despegue de {drone_id}...')
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)  # Aumentado a 10 segundos
            if future.done():
                try:
                    response = future.result()
                    self.get_logger().info(f'  ✅ {drone_id} CONFIRMADO DESPEGANDO a {height}m')
                    self.get_logger().info(f'     El comando takeoff fue aceptado por el servidor')
                    self.drone_states[drone_id]['active'] = True
                    self.drone_states[drone_id]['position'][2] = height  # Actualizar altura esperada
                except Exception as e:
                    self.get_logger().error(f'  ❌ Error en respuesta de despegue de {drone_id}: {e}')
                    self.get_logger().error(f'     Tipo de error: {type(e).__name__}')
                    import traceback
                    self.get_logger().debug(f'     Traceback: {traceback.format_exc()}')
                    all_success = False
            else:
                self.get_logger().error(f'  ❌ TIMEOUT esperando respuesta de despegue de {drone_id}')
                self.get_logger().error(f'     El servicio no respondió en 10 segundos')
                self.get_logger().error(f'     Posibles causas:')
                self.get_logger().error(f'     1. El servidor no está procesando el comando')
                self.get_logger().error(f'     2. El drone no está conectado o no responde')
                self.get_logger().error(f'     3. El drone no está armado correctamente')
                self.get_logger().error(f'     4. Problema de comunicación con el drone')
                all_success = False
        
        # Resumen final
        self.get_logger().info('\n' + '=' * 60)
        if all_success:
            self.get_logger().info('✅ TODOS LOS DRONES ESTÁN DESPEGANDO/VOLANDO')
            self.get_logger().info(f'   Altura objetivo: {height}m')
            self.get_logger().info(f'   Drones activos: {", ".join([d for d in self.drone_ids if self.drone_states[d]["active"]])}')
            
            # Paso 4: Esperar que los drones terminen de despegar del piso
            self.get_logger().info(f'\n📋 PASO 4: ESPERANDO QUE LOS DRONES TERMINEN DE DESPEGAR...')
            self.get_logger().info(f'   Esperando {takeoff_duration + 1.0} segundos (duración del takeoff + margen de seguridad)...')
            time.sleep(takeoff_duration + 1.0)  # Esperar duración completa + margen
            self.get_logger().info('   ✅ Los drones deberían estar en el aire ahora')
            
            # Paso 5: Enviar comando go_to explícito para asegurar que los drones mantengan altura
            # Esto fuerza al high_level_commander a mantener la posición
            self.get_logger().info(f'\n📋 PASO 5: CONFIRMANDO POSICIÓN DE VUELO...')
            for drone_id in self.drone_ids:
                if self.drone_states[drone_id]['active']:
                    # Enviar comando go_to a la misma altura para mantener vuelo
                    success = self.goto_position(drone_id, 
                                                 self.drone_states[drone_id]['position'][0],
                                                 self.drone_states[drone_id]['position'][1],
                                                 height,  # Altura objetivo
                                                 yaw=0.0,
                                                 relative=False)
                    if success:
                        self.get_logger().info(f'  ✅ Comando go_to enviado a {drone_id} para mantener altura {height}m')
                    else:
                        self.get_logger().warn(f'  ⚠️  No se pudo enviar go_to a {drone_id}')
            time.sleep(0.5)  # Pequeña pausa después de los comandos
        else:
            self.get_logger().error('❌ ALGUNOS DRONES NO DESPEGARON CORRECTAMENTE')
            for drone_id in self.drone_ids:
                if not self.drone_states[drone_id]['active']:
                    self.get_logger().error(f'   ❌ {drone_id} NO está volando')
                else:
                    self.get_logger().info(f'   ✅ {drone_id} está volando')
        self.get_logger().info('=' * 60 + '\n')
            
        return all_success
    
    def disarm_all(self):
        """Desarma todos los drones (deshabilita los motores)"""
        self.get_logger().info('Desarmando todos los drones...')
        
        futures = []
        for drone_id in self.drone_ids:
            if drone_id in self.arm_clients and self.arm_clients[drone_id].service_is_ready():
                req = Arm.Request()
                req.arm = False  # False = desarmar
                future = self.arm_clients[drone_id].call_async(req)
                futures.append((drone_id, future))
                self.get_logger().info(f'Comando de desarme enviado a {drone_id}')
            else:
                self.get_logger().warn(f'Servicio arm no disponible para {drone_id}, omitiendo desarme')
        
        # Esperar respuestas
        all_success = True
        for drone_id, future in futures:
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            if future.done():
                try:
                    response = future.result()
                    self.get_logger().info(f'✅ {drone_id} desarmado exitosamente')
                except Exception as e:
                    self.get_logger().error(f'❌ Error desarmando {drone_id}: {e}')
                    all_success = False
            else:
                self.get_logger().warn(f'⚠️  Timeout desarmando {drone_id}')
                all_success = False
        
        return all_success
    
    def land_all(self):
        """Aterriza todos los drones y luego los desarma"""
        self.get_logger().info('Iniciando aterrizaje de todos los drones')
        
        futures = []
        for drone_id in self.drone_ids:
            req = Land.Request()
            req.height = 0.0
            req.duration = rclpy.duration.Duration(seconds=2.0).to_msg()
            req.group_mask = 0
            
            future = self.land_clients[drone_id].call_async(req)
            futures.append((drone_id, future))
            self.get_logger().info(f'Comando de aterrizaje enviado a {drone_id}')
        
        # Esperar respuestas de aterrizaje
        all_success = True
        for drone_id, future in futures:
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            if future.done():
                try:
                    response = future.result()
                    self.get_logger().info(f'✅ {drone_id} aterrizando')
                    self.drone_states[drone_id]['active'] = False
                except Exception as e:
                    self.get_logger().error(f'❌ Error en aterrizaje de {drone_id}: {e}')
                    all_success = False
            else:
                self.get_logger().warn(f'⚠️  Timeout esperando respuesta de aterrizaje de {drone_id}')
                all_success = False
        
        # Esperar a que los drones aterricen completamente antes de desarmar
        self.get_logger().info('Esperando a que los drones aterricen completamente...')
        time.sleep(3.0)  # Esperar suficiente tiempo para que aterricen
        
        # Desarmar todos los drones después del aterrizaje
        self.get_logger().info('Desarmando drones después del aterrizaje...')
        disarm_success = self.disarm_all()
        
        if all_success and disarm_success:
            self.get_logger().info('✅ Todos los drones aterrizaron y fueron desarmados correctamente')
        elif all_success:
            self.get_logger().warn('⚠️  Los drones aterrizaron pero hubo problemas al desarmar')
        else:
            self.get_logger().warn('⚠️  Algunos drones tuvieron problemas al aterrizar')
        
        return all_success and disarm_success
    
    def emergency_stop(self):
        """Detiene todos los drones inmediatamente"""
        self.get_logger().warn('🛑 ¡PARADA DE EMERGENCIA!')
        
        # Enviar señal de emergencia
        msg = Empty()
        self.emergency_publisher.publish(msg)
        
        # También enviar comando de aterrizaje inmediato (que incluye desarme)
        self.land_all()
    
    def goto_position(self, drone_id, x, y, z, yaw=0.0, relative=False):
        """Envía un drone a una posición específica"""
        if drone_id not in self.drone_ids:
            self.get_logger().error(f'Drone {drone_id} no válido')
            return False
            
        req = GoTo.Request()
        req.group_mask = 0
        req.relative = relative
        req.goal.x = float(x)
        req.goal.y = float(y)
        req.goal.z = float(z)
        req.yaw = float(yaw)
        req.duration = rclpy.duration.Duration(seconds=2.0).to_msg()
        
        future = self.goto_clients[drone_id].call_async(req)
        self.get_logger().info(f'{drone_id} -> Posición ({x}, {y}, {z}), relative={relative}')
        
        # Esperar respuesta
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.done():
            try:
                response = future.result()
                self.get_logger().info(f'✅ {drone_id} comandado a posición ({x}, {y}, {z})')
                return True
            except Exception as e:
                self.get_logger().error(f'❌ Error en goto de {drone_id}: {e}')
                return False
        else:
            self.get_logger().warn(f'⚠️  Timeout esperando respuesta de goto de {drone_id}')
            return False
    
    def send_position_command(self, drone_id, x, y, z, yaw=0.0):
        """Envía comando de posición directo"""
        if drone_id not in self.position_publishers:
            return False
            
        msg = Position()
        msg.x = float(x)
        msg.y = float(y)
        msg.z = float(z)
        msg.yaw = float(yaw)
        
        self.position_publishers[drone_id].publish(msg)
        return True
    
    def send_velocity_command(self, drone_id, vx, vy, vz, yaw_rate=0.0):
        """Envía comando de velocidad directo"""
        if drone_id not in self.velocity_publishers:
            return False
            
        msg = VelocityWorld()
        msg.vel.x = float(vx)
        msg.vel.y = float(vy)
        msg.vel.z = float(vz)
        msg.yaw_rate = float(yaw_rate)
        
        self.velocity_publishers[drone_id].publish(msg)
        return True
    
    def control_loop(self):
        """Loop principal de control"""
        # Aquí puedes implementar la lógica de control específica
        # Por ejemplo, formación de vuelo, seguimiento de trayectorias, etc.
        pass
    
    def demo_flight_pattern(self):
        """Demuestra un patrón de vuelo con los 2 drones"""
        self.get_logger().info('=' * 60)
        self.get_logger().info('INICIANDO PATRÓN DE VUELO DEMO')
        self.get_logger().info('=' * 60)
        
        # Paso 1: Despegar ambos drones
        self.get_logger().info('\n📌 FASE 1: DESPEGUE')
        if not self.takeoff_all(1.0):
            self.get_logger().error('❌ Falló el despegue, abortando patrón de vuelo')
            return
        
        # Verificar que ambos drones están volando
        flying_drones = [d for d in self.drone_ids if self.drone_states[d]['active']]
        if len(flying_drones) != len(self.drone_ids):
            self.get_logger().error(f'❌ No todos los drones están volando. Volando: {flying_drones}')
            self.land_all()  # Aterrizar los que estén volando
            return
        
        self.get_logger().info(f'✅ Ambos drones están volando: {", ".join(flying_drones)}')
        
        # Ya esperamos en takeoff_all, pero esperamos un poco más para estabilización
        self.get_logger().info('⏳ Esperando estabilización adicional (2 segundos)...')
        time.sleep(2.0)
        
        # Paso 2: Movimiento de formación
        self.get_logger().info('\n📌 FASE 2: MOVIMIENTO DE FORMACIÓN')
        
        # Mover drone 1 a posición específica
        self.get_logger().info('Moviendo cf1 a posición (1.0, 0.0, 1.0)...')
        self.goto_position('cf1', 1.0, 0.0, 1.0)
        time.sleep(2)
        
        # Mover drone 2 a posición específica
        self.get_logger().info('Moviendo cf2 a posición (-1.0, 0.0, 1.0)...')
        self.goto_position('cf2', -1.0, 0.0, 1.0)
        time.sleep(2)
        
        # Formación circular
        self.get_logger().info('Iniciando formación circular (10 pasos)...')
        for i in range(10):
            angle = i * 0.2
            x1 = 0.5 * math.cos(angle)
            y1 = 0.5 * math.sin(angle)
            x2 = 0.5 * math.cos(angle + math.pi)
            y2 = 0.5 * math.sin(angle + math.pi)
            
            self.goto_position('cf1', x1, y1, 1.0)
            self.goto_position('cf2', x2, y2, 1.0)
            time.sleep(0.5)
        
        self.get_logger().info('✅ Formación circular completada')
        
        # Paso 3: Aterrizaje y desarme
        self.get_logger().info('\n📌 FASE 3: ATERRIZAJE Y DESARME')
        time.sleep(2)
        
        if self.land_all():
            self.get_logger().info('✅ Aterrizaje y desarme completados exitosamente')
        else:
            self.get_logger().warn('⚠️  Hubo problemas durante el aterrizaje o desarme')
        
        # Verificación final
        active_drones = [d for d in self.drone_ids if self.drone_states[d]['active']]
        if len(active_drones) == 0:
            self.get_logger().info('✅ Verificación: Todos los drones están en tierra y desarmados')
        else:
            self.get_logger().warn(f'⚠️  Verificación: Algunos drones aún están marcados como activos: {active_drones}')
        
        self.get_logger().info('\n' + '=' * 60)
        self.get_logger().info('PATRÓN DE VUELO DEMO COMPLETADO')
        self.get_logger().info('=' * 60)

def main(args=None):
    rclpy.init(args=args)
    
    controller = DualCrazyflieController()
    
    # Esperar servicios
    if not controller.wait_for_services():
        controller.get_logger().error('No se pudieron conectar todos los servicios')
        return
    
    # Espera adicional para que el sistema se estabilice completamente
    controller.get_logger().info('⏳ Esperando 5 segundos adicionales para que el sistema se estabilice...')
    time.sleep(5.0)
    controller.get_logger().info('✅ Sistema estabilizado, iniciando patrón de vuelo')
    
    try:
        # Ejecutar patrón de vuelo demo
        controller.demo_flight_pattern()
        
        # Mantener el nodo activo
        rclpy.spin(controller)
        
    except KeyboardInterrupt:
        controller.get_logger().info('Deteniendo controlador...')
        controller.emergency_stop()
        
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
