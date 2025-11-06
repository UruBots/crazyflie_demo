# Solución para Problemas de Conexión con Crazyflie

## Problema Actual

- **Error**: "Too many packets lost" y "[crazyflie_real] is disconnected!"
- **URI configurada**: `radio://0/80/2M/E7E7E7E7E7`
- **Estado**: No se puede establecer comunicación con el drone

## Diagnóstico Realizado

✅ **Hardware OK**:
- Crazyradio PA detectada en USB (`1915:7777`)
- Drivers USB inicializados correctamente
- cflib funcionando

❌ **Comunicación FALLO**:
- No se encontraron drones con el escaneo
- Todas las URIs probadas fallan
- Error constante: "Too many packets lost"

## Causas Más Probables

### 1. El drone NO está encendido o visible
**Síntomas**:
- No se encuentra ningún drone con el escaneo
- "Too many packets lost" constantemente

**Soluciones**:
- ✅ Verifica que el drone esté **ENCENDIDO** (presiona el botón de encendido)
- ✅ Verifica que los **LEDs estén encendidos**
- ✅ Verifica que la **batería tenga carga** (LEDs de batería)
- ✅ **Reinicia el drone**: apágalo completamente, espera 5 segundos, y enciéndelo de nuevo

### 2. El drone está fuera de rango
**Síntomas**:
- El drone está encendido pero no responde

**Soluciones**:
- ✅ Coloca el drone **a menos de 5 metros** de la antena
- ✅ Verifica que **no haya obstáculos** entre el drone y la antena
- ✅ Evita **interferencias** (WiFi, Bluetooth, otros dispositivos 2.4GHz)

### 3. Configuración incorrecta del drone
**Síntomas**:
- El drone responde pero con una URI diferente
- El escaneo no encuentra el drone

**Solución**: Configurar el drone usando el **Crazyflie Client oficial**

#### Pasos:

1. **Descarga e instala el Crazyflie Client**:
   - Visita: https://www.bitcraze.io/products/crazyflie-2-1/
   - Descarga el cliente para tu sistema operativo

2. **Conecta el drone por USB** (no por radio):
   - Conecta el drone directamente a tu computadora con un cable USB
   - Abre el Crazyflie Client

3. **Configura la dirección del drone**:
   - En el cliente, ve a la configuración del drone
   - Establece la dirección: `E7E7E7E7E7` (o `E7E7E7E701` para el primer drone, `E7E7E7E702` para el segundo, etc.)
   - Guarda la configuración

4. **Configura el canal y velocidad**:
   - Canal: `80` (puedes usar otros como 90, 100 si tienes interferencias)
   - Velocidad: `2M` (2 Mbps)
   - Guarda la configuración

5. **Actualiza el archivo YAML**:
   ```bash
   nano /workspace/ros2_ws/src/crazyflie_ros2_multiranger/crazyflie_ros2_multiranger_bringup/config/crazyflie_real_crazyswarm2.yaml
   ```
   
   Cambia la línea 5 para usar la URI correcta:
   ```yaml
   uri: radio://0/80/2M/E7E7E7E7E7  # O la URI que configuraste
   ```

6. **Reinicia el drone** después de cambiar la configuración

### 4. Problema con el firmware del drone
**Síntomas**:
- El drone no responde correctamente
- Comportamiento errático

**Soluciones**:
- ✅ Actualiza el firmware del drone usando el Crazyflie Client
- ✅ Asegúrate de usar firmware compatible con Crazyswarm2
- ✅ Verifica que el firmware sea de una versión reciente

### 5. Problema con la antena
**Síntomas**:
- La antena se detecta pero no hay comunicación

**Soluciones**:
- ✅ **Desconecta y reconecta** la antena USB
- ✅ Verifica que la antena esté bien conectada
- ✅ Prueba la antena en otra computadora si es posible
- ✅ Verifica que no haya problemas físicos con la antena

## Scripts de Diagnóstico Disponibles

### 1. Diagnóstico rápido
```bash
python3 /workspace/diagnose_connection.py
```
Prueba la conexión con la URI configurada actualmente.

### 2. Escaneo completo
```bash
python3 /workspace/scan_crazyflies.py
```
Escanea múltiples configuraciones para encontrar el drone.

### 3. Escaneo original
```bash
python3 /workspace/find_drone_uris.py
```
Múltiples métodos de escaneo.

## Solución Recomendada (Paso a Paso)

1. **Usa el Crazyflie Client oficial**:
   - Es la forma más confiable de configurar y verificar el drone
   - Conecta el drone por USB (no por radio)
   - Configura: dirección, canal, velocidad

2. **Verifica la conexión**:
   - Una vez configurado, prueba la conexión desde el cliente
   - Si funciona en el cliente, debería funcionar en ROS2

3. **Actualiza la configuración en ROS2**:
   - Edita el archivo YAML con la URI correcta
   - Reinicia el sistema ROS2

4. **Prueba la conexión**:
   ```bash
   python3 /workspace/diagnose_connection.py
   ```

5. **Si aún no funciona**:
   - Verifica distancia (< 5m)
   - Verifica que el drone esté encendido
   - Verifica batería
   - Intenta reiniciar tanto el drone como la antena

## Verificación Final

Cuando todo funcione correctamente, deberías ver:
- ✅ Conexión exitosa en `diagnose_connection.py`
- ✅ El drone aparece en el escaneo
- ✅ No hay errores "Too many packets lost"
- ✅ El drone responde a comandos

## Contacto y Recursos

- **Documentación oficial**: https://www.bitcraze.io/documentation/
- **Crazyflie Client**: https://www.bitcraze.io/products/crazyflie-2-1/
- **Crazyswarm2**: https://github.com/IMRCLab/crazyswarm2

