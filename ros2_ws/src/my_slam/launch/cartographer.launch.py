from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('my_slam'), 'config')
    return LaunchDescription([
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            arguments=[
                '-configuration_directory', config_dir,
                '-configuration_basename', 'my_robot.lua'
            ]
        ),
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='occupancy_grid_node',
            output='screen',
            # 서브맵 격자(0.01)와 맞춘다. 값이 다르면 재샘플링이 들어가 맵이 뭉개진다.
            arguments=['-resolution', '0.01']
        ),
    ])
