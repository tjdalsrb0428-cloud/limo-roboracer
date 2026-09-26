import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    map_arg = DeclareLaunchArgument(
        'map',
        default_value='/home/mksung0428/Downloads/LIMO_GAZEBO/ws_limo_humble/maps/my_map.yaml',
        description='Full path to map yaml file to load'
    )
    map_yaml_file = LaunchConfiguration('map')
    amcl_params_file = '/home/mksung0428/Downloads/LIMO_GAZEBO/ws_limo_humble/nav2_config/amcl_params.yaml'

    return LaunchDescription([
        map_arg,
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'yaml_filename': map_yaml_file}]
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[amcl_params_file,
                        {'use_sim_time': use_sim_time}]
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': True},
                        {'bond_timeout': 0.0},
                        {'node_names': ['map_server', 'amcl']}]
        )
    ])
