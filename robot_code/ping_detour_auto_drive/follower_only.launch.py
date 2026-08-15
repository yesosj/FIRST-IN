#!/usr/bin/env python3
"""
follower_only.launch.py
----------------------------------------------------------
map_nav_test.launch.py 를 이미 다른 터미널에서 띄워 둔 상태에서,
"자율주행(경로추종)만" 얹을 때 쓰는 런치.

  터미널 1 :  ros2 launch ~/map_path/map_nav_test.launch.py
  터미널 2 :  ros2 launch ~/auto_drive/follower_only.launch.py

띄우는 것
  1) ~/auto_drive/path_follower_node.py  /plan 추종 -> /cmd_vel
  2) ~/UART/node.py                      /cmd_vel -> 모터 (motor:=true 일 때)
  3) robot_state_publisher                        RViz RobotModel 용 /robot_description

RViz 로봇모델 보기
  map_nav_test.launch.py 의 RViz 창에서 좌하단 [Add] -> RobotModel 을 추가하고
  Description Source = Topic, Description Topic = /robot_description 으로 두면 된다.
  (이 런치가 /robot_description 을 latched 로 발행한다)
  귀찮으면 use_rviz:=true 로 로봇모델이 들어간 RViz 창을 하나 더 띄워도 된다.

인자
  motor:=false          모터 노드를 여기서 안 띄움 (이미 실행 중일 때)
  robot_model:=false    robot_state_publisher 생략
  use_rviz:=true        로봇모델 포함 RViz 창을 추가로 띄움 (기본 false)
  goal_tolerance:=0.12  도착 판정 오차[m]
  use_scan_guard:=false 라이다 앞쪽 비상정지 끄기
----------------------------------------------------------
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HOME = os.path.expanduser("~")
AUTO_DIR = os.path.join(HOME, "auto_drive")
FOLLOWER_NODE = os.path.join(AUTO_DIR, "path_follower_node.py")
MOTOR_NODE = os.path.join(HOME, "UART", "node.py")
URDF_FILE = os.path.join(AUTO_DIR, "robot_model.urdf")
RVIZ_CFG = os.path.join(AUTO_DIR, "auto_drive.rviz")

SYS_PY = "/usr/bin/python3"


def generate_launch_description():
    ld = [
        DeclareLaunchArgument("motor", default_value="false",
                              description="UART 모터 노드도 함께 실행"),
        DeclareLaunchArgument("robot_model", default_value="true",
                              description="RViz RobotModel 용 robot_state_publisher 실행"),
        DeclareLaunchArgument("use_rviz", default_value="false",
                              description="로봇모델 포함 RViz 를 추가로 실행"),

        DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
        DeclareLaunchArgument("path_topic", default_value="/plan"),
        DeclareLaunchArgument("allow_reverse", default_value="false"),
        DeclareLaunchArgument("turn_forward_pulse", default_value="0"),
        DeclareLaunchArgument("heading_offset", default_value="0.0"),
        DeclareLaunchArgument("first_move_forward", default_value="true"),
        DeclareLaunchArgument("first_forward_dist", default_value="0.10"),
        DeclareLaunchArgument("invert_angular", default_value="false"),
        DeclareLaunchArgument("invert_linear", default_value="false"),
        DeclareLaunchArgument("control_mode", default_value="keys"),
        DeclareLaunchArgument("turn_pulse_on", default_value="4"),
        DeclareLaunchArgument("turn_pulse_off", default_value="2"),
        DeclareLaunchArgument("turn_mode", default_value="arc"),
        DeclareLaunchArgument("lookahead", default_value="0.12"),
        DeclareLaunchArgument("goal_tolerance", default_value="0.12"),
        DeclareLaunchArgument("robot_length", default_value="0.23"),
        DeclareLaunchArgument("robot_width", default_value="0.19"),
        DeclareLaunchArgument("use_scan_guard", default_value="false"),
        DeclareLaunchArgument("obstacle_stop_distance", default_value="0.0"),
    ]

    ld.append(ExecuteProcess(
        cmd=[SYS_PY, FOLLOWER_NODE, "--ros-args",
             "-p", ["cmd_vel_topic:=", LaunchConfiguration("cmd_vel_topic")],
             "-p", ["path_topic:=", LaunchConfiguration("path_topic")],
             "-p", "map_frame:=map", "-p", "base_frame:=base_link",
             "-p", ["allow_reverse:=", LaunchConfiguration("allow_reverse")],
             "-p", ["turn_forward_pulse:=", LaunchConfiguration("turn_forward_pulse")],
             "-p", ["heading_offset:=", LaunchConfiguration("heading_offset")],
             "-p", ["first_move_forward:=", LaunchConfiguration("first_move_forward")],
             "-p", ["first_forward_dist:=", LaunchConfiguration("first_forward_dist")],
             "-p", ["invert_angular:=", LaunchConfiguration("invert_angular")],
             "-p", ["invert_linear:=", LaunchConfiguration("invert_linear")],
             "-p", ["control_mode:=", LaunchConfiguration("control_mode")],
             "-p", ["turn_pulse_on:=", LaunchConfiguration("turn_pulse_on")],
             "-p", ["turn_pulse_off:=", LaunchConfiguration("turn_pulse_off")],
             "-p", ["turn_mode:=", LaunchConfiguration("turn_mode")],
             "-p", ["lookahead:=", LaunchConfiguration("lookahead")],
             "-p", ["goal_tolerance:=", LaunchConfiguration("goal_tolerance")],
             "-p", ["robot_length:=", LaunchConfiguration("robot_length")],
             "-p", ["robot_width:=", LaunchConfiguration("robot_width")],
             "-p", ["use_scan_guard:=", LaunchConfiguration("use_scan_guard")],
             "-p", ["obstacle_stop_distance:=",
                    LaunchConfiguration("obstacle_stop_distance")]],
        output="screen"))

    ld.append(ExecuteProcess(
        condition=IfCondition(LaunchConfiguration("motor")),
        cmd=[SYS_PY, MOTOR_NODE],
        output="screen"))

    with open(URDF_FILE, "r") as f:
        robot_description = f.read()
    ld.append(Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        name="robot_state_publisher", output="screen",
        condition=IfCondition(LaunchConfiguration("robot_model")),
        parameters=[{"robot_description": robot_description,
                     "publish_frequency": 10.0}]))

    ld.append(Node(
        package="rviz2", executable="rviz2", name="rviz2_auto_drive",
        output="screen", arguments=["-d", RVIZ_CFG],
        condition=IfCondition(LaunchConfiguration("use_rviz"))))

    return LaunchDescription(ld)
