"""주행: map_server + global_path + localizer + local_planner + controller [+ odom_tf] [+ viz_relay]

  ros2 launch limo_race race.launch.py robot:=limo_pro map:=NAME vmax:=0.5
  → READY 뜨면 다른 터미널에서  ros2 run limo_race go   (또는 ros2 topic pub --once /race/go std_msgs/msg/Empty)
  → 정지: Ctrl+C (controller 가 속도 0 보내고 종료)
인자: calib:=auto  sim:=false  own_odom:=false  viz:=true  lite:=false(계산 줄이기)  slow:=false(좁은 곳 감속)
      global_only:=false(true = 회피 없이 레이스라인만)  go:=false(true = 바로 출발, 시험용)
      map_server:=true(false = 지도를 이미 다른 map_server 가 발행 중일 때)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration as C
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue as V


def generate_launch_description():
    d = dict(robot='limo_pro', calib='auto', map='map', vmax='1.0', sim='false', own_odom='false', viz='true',
             lite='false', slow='false', global_only='false', go='false', map_server='true')
    args = [DeclareLaunchArgument(k, default_value=v) for k, v in d.items()]
    S = lambda k: V(C(k), value_type=str)    # noqa: E731
    B = lambda k: V(C(k), value_type=bool)   # noqa: E731
    F = lambda k: V(C(k), value_type=float)  # noqa: E731
    robot = {'robot': S('robot'), 'calib': S('calib'), 'sim': B('sim')}
    o = dict(output='screen', emulate_tty=True, package='limo_race')
    return LaunchDescription(args + [
        Node(executable='map_server', name='map_server', parameters=[{'map': S('map')}],
             condition=IfCondition(C('map_server')), **o),
        Node(executable='global_path', name='global_path', parameters=[{'map': S('map')}], **o),
        Node(executable='localizer', name='localizer', parameters=[robot], **o),
        Node(executable='local_planner', name='local_planner',
             parameters=[dict(robot, vmax=F('vmax'), lite=B('lite'), slow=B('slow'), global_only=B('global_only'))],
             **o),
        Node(executable='controller', name='controller', parameters=[dict(robot, vmax=F('vmax'), go=B('go'))], **o),
        Node(executable='odom_tf', name='odom_tf',
             arguments=[['robot:=', C('robot')], ['calib:=', C('calib')], ['sim:=', C('sim')]],
             condition=IfCondition(C('own_odom')), **o),
        Node(executable='viz_relay', name='viz_relay', condition=IfCondition(C('viz')), **o),
    ])
