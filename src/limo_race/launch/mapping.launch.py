"""매핑: mapper(지도 그리기) + map_saver(저장 서비스) [+ odom_tf] [+ viz_relay]

  ros2 launch limo_race mapping.launch.py robot:=limo_pro map:=NAME
  → 한 바퀴 돌고 출발점 근처에 세운 뒤, 다른 터미널에서
     ros2 service call /map_saver/save std_srvs/srv/Trigger
  → maps/real/NAME.pgm/.yaml/_traj.csv 저장. 확인 후 Ctrl+C (Ctrl+C 는 저장하지 않음)
인자: calib:=auto|none|파일  sim:=false  own_odom:=false(로봇 EKF TF 없을 때 true)  viz:=true(노트북 RViz 중계)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration as C
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue as V


def generate_launch_description():
    args = [DeclareLaunchArgument('robot', default_value='limo_pro'),
            DeclareLaunchArgument('calib', default_value='auto'),
            DeclareLaunchArgument('map', default_value='map'),
            DeclareLaunchArgument('sim', default_value='false'),
            DeclareLaunchArgument('own_odom', default_value='false'),
            DeclareLaunchArgument('viz', default_value='true')]
    robot = {'robot': V(C('robot'), value_type=str), 'calib': V(C('calib'), value_type=str),
             'sim': V(C('sim'), value_type=bool)}
    return LaunchDescription(args + [
        Node(package='limo_race', executable='mapper', name='mapper', output='screen', emulate_tty=True,
             parameters=[robot]),
        Node(package='limo_race', executable='map_saver', name='map_saver', output='screen', emulate_tty=True,
             parameters=[{'map': V(C('map'), value_type=str)}]),
        Node(package='limo_race', executable='odom_tf', name='odom_tf', output='screen', emulate_tty=True,
             arguments=[['robot:=', C('robot')], ['calib:=', C('calib')], ['sim:=', C('sim')]],
             condition=IfCondition(C('own_odom'))),
        Node(package='limo_race', executable='viz_relay', name='viz_relay', output='screen', emulate_tty=True,
             condition=IfCondition(C('viz'))),
    ])
