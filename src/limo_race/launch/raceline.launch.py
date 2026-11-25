"""global path 생성: map_server(지도 불러오기) + raceline_generator(레이스라인 계산·저장·발행)

  ros2 launch limo_race raceline.launch.py map:=NAME
  → maps/real/NAME_raceline.csv(.png) 저장, RViz 에 지도 + 파란 선. 확인 후 Ctrl+C
인자: clear:=0.30(벽 여유)  kmax:=1.25(곡률 한도)  stay:=true(false 면 저장 후 생성 노드 종료)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration as C
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue as V


def generate_launch_description():
    args = [DeclareLaunchArgument('map', default_value='map'),
            DeclareLaunchArgument('clear', default_value='0.30'),
            DeclareLaunchArgument('kmax', default_value='1.25'),
            DeclareLaunchArgument('stay', default_value='true')]
    m = {'map': V(C('map'), value_type=str)}
    return LaunchDescription(args + [
        Node(package='limo_race', executable='map_server', name='map_server', output='screen', emulate_tty=True,
             parameters=[m]),
        Node(package='limo_race', executable='raceline_generator', name='raceline_generator', output='screen',
             emulate_tty=True,
             parameters=[dict(m, clear=V(C('clear'), value_type=float), kmax=V(C('kmax'), value_type=float),
                              stay=V(C('stay'), value_type=bool))]),
    ])
