#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    limo_share = get_package_share_directory("limo_car")
    gazebo_share = get_package_share_directory("gazebo_ros")

    default_world = os.path.join(
        limo_share,
        "worlds",
        "roboracer",
        "ifac_roboracer.world",
    )
    default_rviz = os.path.join(limo_share, "rviz", "gazebo.rviz")

    world = LaunchConfiguration("world")
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_z = LaunchConfiguration("spawn_z")
    spawn_yaw = LaunchConfiguration("spawn_yaw")
    use_rviz = LaunchConfiguration("rviz")
    real = LaunchConfiguration("real")

    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(limo_share, "launch", "ackermann.launch.py")
        ),
        launch_arguments={"use_sim_time": "true", "real": real,
                          "robot": LaunchConfiguration("robot")}.items(),
    )

    # real:=true: 플러그인 정답 위치(/odom_gt)에 오차를 넣어 실차 바퀴 오도메트리(/odom, TF odom→base_link) 흉내
    # robot:=limo_pro → Pro 드라이버 odom 계산 버그 재현, 라이다 ±110° 10 Hz, /limo/ack_cmd 입력
    # ekf:=true → 실차처럼 robot_localization EKF(로봇 설정 원본)가 TF odom→base_link 를 낸다.
    #   (2026-11-05: 시뮬에서는 EKF 출력이 NaN — 원인 미확정 → 기본 false, 바퀴 odom 흉내가 TF 발행)
    def real_nodes(context):
        rb = context.launch_configurations.get("robot", "limo1")
        use_ekf = context.launch_configurations.get("ekf", "false").lower() == "true"
        # odom_tf:=false → 아무도 odom→base_link TF를 안 냄(실차에 EKF TF가 없을 때 흉내, scripts/real/odom_tf.py로 채움)
        wheel_tf = context.launch_configurations.get("odom_tf", "true").lower() == "true"
        nodes = [Node(package="limo_car", executable="sim_wheel_odom.py", output="screen",
                      parameters=[{"use_sim_time": True, "publish_tf": wheel_tf and not use_ekf,
                                   "model": "limo_pro" if rb == "limo_pro" else "ideal"}])]
        if use_ekf:
            nodes.append(Node(package="robot_localization", executable="ekf_node", name="ekf_filter_node_odom",
                              output="screen", arguments=["--ros-args", "--log-level", "warn"],
                              parameters=[os.path.join(limo_share, "config", f"ekf_{rb}.yaml"),
                                          {"use_sim_time": True}]))
        return nodes
    wheel_odom = GroupAction([OpaqueFunction(function=real_nodes)], condition=IfCondition(real))

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, "launch", "gazebo.launch.py")
        ),
        launch_arguments={"world": world, "verbose": "false",
                          "gui": LaunchConfiguration("gui")}.items(),
    )

    spawn = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "limo",
            "-x", spawn_x,
            "-y", spawn_y,
            "-z", spawn_z,
            "-Y", spawn_yaw,
            "-timeout", "60",
            "--ros-args", "--log-level", "error",
        ],
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", default_rviz, "--ros-args", "--log-level", "error"],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(use_rviz),
        output="screen",
    )

    return LaunchDescription([
        # 시험장 오프라인(Wi-Fi 없음/학교망 혼선) 대비: ROS2·Gazebo 통신을 이 PC 내부로 한정
        SetEnvironmentVariable("ROS_LOCALHOST_ONLY", "1"),
        SetEnvironmentVariable("GAZEBO_IP", "127.0.0.1"),
        DeclareLaunchArgument("world", default_value=default_world),
        DeclareLaunchArgument("spawn_x", default_value="0.0"),
        # One 1 m grid cell above the green +Y start line, facing +X.
        DeclareLaunchArgument("spawn_y", default_value="1.0"),
        DeclareLaunchArgument("spawn_z", default_value="0.15"),
        DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("real", default_value="false"),
        DeclareLaunchArgument("robot", default_value="limo1"),
        DeclareLaunchArgument("ekf", default_value="false"),
        DeclareLaunchArgument("odom_tf", default_value="true"),
        robot_description,
        wheel_odom,
        gazebo,
        TimerAction(period=2.0, actions=[spawn]),
        TimerAction(period=4.0, actions=[rviz]),
    ])
