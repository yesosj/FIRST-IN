"""rviz2 만 이 패키지의 설정으로 띄운다.

SLAM 은 젯슨에서 돌리고 화면만 따로 볼 때, 또는 rviz 만 다시 켤 때 사용:
    ros2 launch slam_bringup rviz.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    rviz_config = os.path.join(
        get_package_share_directory('slam_bringup'), 'rviz', 'slam.rviz')

    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
        ),
    ])
