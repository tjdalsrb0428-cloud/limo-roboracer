import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    map_yaml_path = '/home/mksung0428/Downloads/LIMO_GAZEBO/ws_limo_humble/maps/my_map.yaml'
    amcl_params_path = '/home/mksung0428/Downloads/LIMO_GAZEBO/ws_limo_humble/nav2_config/amcl_params.yaml'

    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'yaml_filename': map_yaml_path,
                'use_sim_time': True
            }]
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[
                amcl_params_path,
                {'use_sim_time': True}
            ]
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': ['map_server', 'amcl'],
                'bond_timeout': 0.0,
                'attempt_respawn': False
            }]
        )
    ])
