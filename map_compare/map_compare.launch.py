#!/usr/bin/env python3
"""
map_compare.launch.py
----------------------------------------------------------
터미널 하나로 "기준 도면 + 실시간 SLAM 맵 + 차이 표시"를 띄운다.
(자율주행 없음 — 지도 비교 전용)

  source ~/ros2_ws/install/setup.bash
  ros2 launch ~/map_compare.launch.py

구성
  1) slam_bringup/slam.launch.py (use_rviz:=false)
       라이다(/scan)+IMU+TF+카토그래퍼 SLAM → 실시간 /map, map→odom→base_link
  2) map_diff_node.py
       기준 도면(room_shelf5.yaml) 로드 → /reference_map 발행,
       실시간 /map 과 비교 → /map_diff 발행 + 콘솔 통계
  3) rviz2 (map_compare.rviz)
       회색 기준도면 위에 반투명 실시간맵, 그 위에 차이를 색으로 강조

RViz 표시
  1_기준도면(room_shelf5)   : /reference_map  (회색)
  2_실시간SLAM(/map)   : /map            (반투명 회색, SLAM 진행되며 채워짐)
  3_차이(/map_diff)    : /map_diff       (costmap 색 → 다른 셀이 분홍/청록으로 강조)

인자
  reference:=<yaml>   기준 도면 (기본 ~/map_compare/custom_maps/room_shelf5.yaml)
  map_resolution:=0.01  실시간 /map 격자(기준 도면과 맞춤). 무선이 느리면 0.05.

주의: 실시간 SLAM 맵의 map 프레임 원점은 로봇 시작 위치다. 도면과 정확히
겹쳐 비교하려면 로봇을 도면 기준 위치·방향에서 출발시켜야 한다.
----------------------------------------------------------
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HOME = os.path.expanduser("~")
PKG_DIR = os.path.join(HOME, "map_compare")     # 이 런치가 있는 폴더
DEFAULT_REF = os.path.join(PKG_DIR, "custom_maps", "room_shelf5.yaml")
DIFF_NODE = os.path.join(PKG_DIR, "map_diff_node.py")
RVIZ_CFG = os.path.join(PKG_DIR, "map_compare.rviz")


def generate_launch_description():
    ld = [
        DeclareLaunchArgument("reference", default_value=DEFAULT_REF,
                              description="기준 도면 yaml 경로"),
        DeclareLaunchArgument("map_resolution", default_value="0.01",
                              description="실시간 /map 격자 해상도[m]"),
        # 정렬: 로봇이 도면상 어디서·어느 방향으로 출발했는지.
        # 기본 (0.15, 0.15, 180): 도면 왼쪽아래 근처에서 180도 방향으로 출발.
        DeclareLaunchArgument("start_x", default_value="0.15",
                              description="도면상 로봇 X 위치[m] (왼쪽아래=작은값)"),
        DeclareLaunchArgument("start_y", default_value="0.15",
                              description="도면상 로봇 Y 위치[m] (아래쪽=작은값)"),
        DeclareLaunchArgument("start_yaw", default_value="180.0",
                              description="도면상 로봇 방향[도]"),
    ]

    # 1) 로봇 하드웨어 + 카토그래퍼 SLAM (RViz 는 아래에서 우리 설정으로 따로 띄움)
    slam_launch = os.path.join(
        get_package_share_directory("slam_bringup"), "launch", "slam.launch.py")
    ld.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch),
        launch_arguments={
            "use_rviz": "false",
            "map_resolution": LaunchConfiguration("map_resolution"),
        }.items()))

    # 2) 기준 도면 발행 + 실시간 비교 (rclpy/numpy → 시스템 파이썬)
    ld.append(ExecuteProcess(
        cmd=["/usr/bin/python3", DIFF_NODE, "--ros-args",
             "-p", ["reference:=", LaunchConfiguration("reference")],
             "-p", "live_topic:=/map", "-p", "frame:=map",
             "-p", ["start_x:=", LaunchConfiguration("start_x")],
             "-p", ["start_y:=", LaunchConfiguration("start_y")],
             "-p", ["start_yaw:=", LaunchConfiguration("start_yaw")]],
        output="screen"))

    # 3) RViz — 기준도면 + 실시간맵 + 차이 오버레이 설정
    ld.append(Node(
        package="rviz2", executable="rviz2", name="rviz2",
        output="screen", arguments=["-d", RVIZ_CFG]))

    return LaunchDescription(ld)
