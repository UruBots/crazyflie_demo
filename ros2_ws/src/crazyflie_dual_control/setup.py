from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'crazyflie_dual_control'

setup(
    name='crazyflie-dual-control',  # Nombre para metadatos (usar guiones)
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='sbarcelona@gmail.com',
    description='Paquete ROS2 para controlar 2 Crazyflies usando Crazyswarm2',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'dual_controller = crazyflie_dual_control.dual_controller:main',
            'simple_flight = crazyflie_dual_control.simple_flight:main',
            'keyboard_controller = crazyflie_dual_control.keyboard_controller:main',
            'advanced_controller = crazyflie_dual_control.advanced_controller:main',
            'battery_monitor = crazyflie_dual_control.battery_monitor:main',
            'gazebo_pose_sync = crazyflie_dual_control.gazebo_pose_sync:main',
        ],
    },
)
