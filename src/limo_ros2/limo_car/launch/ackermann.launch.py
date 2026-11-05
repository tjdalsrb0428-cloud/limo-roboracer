# Autor: Zhui Li
# E-Mail: lz554113510@gmail.com
# Company: Institut für Intermodale Transport- und Logistiksysteme in Technische Universität Braunschweig
# Description: Diese py-Datei basiert auf Regeln und definiert Funktionen durch python,
# um den Launch der Simulation in Gazebo zu ermöglichen. Danach mit der Simulation in Gazebo kann man die weitere
# Forschung arbeiten. Diese Launch-Datei dient zu Ackermann-Type.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node

import xacro


def robot_nodes(context):

    # um die Zeit zu simulieren
    use_sim_time = LaunchConfiguration('use_sim_time')
    # real:=true → 실차 치수·라이다·바퀴 오도메트리 흉내 (xacro 인자로 전달)
    real = context.launch_configurations.get('real', 'false').lower() == 'true'
    robot = context.launch_configurations.get('robot', 'limo1')

    # finden path für Modell
    pkg_path = os.path.join(get_package_share_directory('limo_car'))
    xacro_file = os.path.join(pkg_path, 'gazebo', 'ackermann_with_sensor.xacro')
    robot_description_config = xacro.process_file(
        xacro_file, mappings={'real': 'true' if real else 'false', 'robot': robot})

    # Erstellt ein robot_state_publisher Node
    params = {'robot_description': robot_description_config.toxml(),
            'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        arguments=['--ros-args', '--log-level', 'error'],
        output='screen',
        parameters=[params]
    )
    adapter_params = {'use_sim_time': use_sim_time}
    if real:
        adapter_params.update(wheelbase=0.20, max_steer=0.41)

    if real and robot == 'limo_pro':
        # 실차 Pro: /cmd_vel 없음, /limo/ack_cmd(AckermannDrive)만 받는다
        cmd_node = Node(package='limo_car', executable='sim_pro_ack.py',
                        parameters=[adapter_params], output='screen')
    else:
        cmd_node = Node(package='limo_car', executable='ackermann_twist_adapter.py',
                        parameters=[adapter_params], output='screen')
    return [node_robot_state_publisher, cmd_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        DeclareLaunchArgument('real', default_value='false'),
        DeclareLaunchArgument('robot', default_value='limo1'),
        OpaqueFunction(function=robot_nodes),
    ])
