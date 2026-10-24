#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
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
        launch_arguments={"use_sim_time": "true", "real": real}.items(),
    )

    # real:=true: 플러그인 정답 위치(/odom_gt)에 오차를 넣어 실차 바퀴 오도메트리(/odom, TF odom→base_link) 흉내
    wheel_odom = Node(
        package="limo_car",
        executable="sim_wheel_odom.py",
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(real),
        output="screen",
    )

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
        robot_description,
        wheel_odom,
        gazebo,
        TimerAction(period=2.0, actions=[spawn]),
        TimerAction(period=4.0, actions=[rviz]),
    ])
