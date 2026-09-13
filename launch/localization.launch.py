import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_yaml_file = '/home/mksung0428/Downloads/LIMO_GAZEBO/ws_limo_humble/maps/my_map.yaml'
    amcl_params_file = '/home/mksung0428/Downloads/LIMO_GAZEBO/ws_limo_humble/nav2_config/amcl_params.yaml'

    return LaunchDescription([
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
                        {'use_sim_time': use_sim_time},
                        {'set_initial_pose': True},
                        {'initial_pose': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0}}]
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
