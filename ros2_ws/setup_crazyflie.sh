#!/bin/bash
# Script para configurar el entorno Crazyflie

# Source ROS2
source /opt/ros/jazzy/setup.bash

# Source workspace
source /workspace/ros2_ws/install/setup.bash

# Configurar variables de entorno
export GZ_SIM_RESOURCE_PATH="/workspace/ros2_ws/src/crazyflie-simulation/simulator_files/gazebo/"

echo "Entorno Crazyflie configurado!"
echo "Workspace: /workspace/ros2_ws"
echo "Para usar: source /workspace/ros2_ws/setup_crazyflie.sh"
