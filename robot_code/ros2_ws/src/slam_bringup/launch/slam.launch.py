"""YDLidar + MPU6050 IMU + Cartographer 2D SLAM 통합 실행.

터미널 하나로 전부 띄운다:
    ros2 launch slam_bringup slam.launch.py

노드 구성
    ydlidar_ros2_driver_node   /scan  (sensor_msgs/LaserScan)
    imu_filter_node.py         /imu   (sensor_msgs/Imu, 바이어스 보정 + 정지 게이팅)
    static_transform_publisher base_link -> laser_frame, base_link -> imu_link
    cartographer_node          map -> odom -> base_link TF
    occupancy_grid_node        /map   (nav_msgs/OccupancyGrid)
    rviz2                      rviz/slam.rviz

TF 는 여기 한 곳에서만 발행한다. ydlidar_ros2_driver 의 자체 launch 파일도
base_link -> laser_frame 을 다른 값으로 쏘기 때문에 그 launch 는 쓰지 않고
드라이버 노드만 직접 띄운다. (두 개가 동시에 뜨면 TF 가 충돌해서 맵이 뭉개짐)

rviz 를 원격(노트북)에서 볼 때의 지연은 대부분 /map 대역폭 때문이다.
map_resolution 을 0.01 로 두면 7m x 5m 맵이 이미 400KB/장이고, 면적에 비례해
계속 커지므로 무선에서는 갈수록 밀린다. 기본값 0.05 로 25배 줄여둔다.
맵을 최종 저장할 때만 map_resolution:=0.01 로 다시 띄우면 된다.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('slam_bringup')
    config_dir = os.path.join(pkg_share, 'config')
    rviz_config = os.path.join(pkg_share, 'rviz', 'slam.rviz')

    use_rviz = LaunchConfiguration('use_rviz')
    use_lidar = LaunchConfiguration('use_lidar')
    use_imu = LaunchConfiguration('use_imu')
    use_filtered_imu = LaunchConfiguration('use_filtered_imu')
    lidar_port = LaunchConfiguration('lidar_port')
    map_resolution = LaunchConfiguration('map_resolution')
    map_publish_period = LaunchConfiguration('map_publish_period')
    imu_rate = LaunchConfiguration('imu_rate')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    zero_when_stationary = LaunchConfiguration('zero_when_stationary')
    require_cmd_vel_publisher = LaunchConfiguration('require_cmd_vel_publisher')
    gyro_motion_threshold = LaunchConfiguration('gyro_motion_threshold')
    median_window = LaunchConfiguration('median_window')

    args = [
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='rviz2 를 같이 띄울지 여부'),
        DeclareLaunchArgument(
            'use_lidar', default_value='true',
            description='라이다 드라이버를 띄울지 여부 (bag 재생 시 false)'),
        DeclareLaunchArgument(
            'use_imu', default_value='true',
            description='IMU 노드를 띄울지 여부 (bag 재생 시 false)'),
        DeclareLaunchArgument(
            'use_filtered_imu', default_value='true',
            description='true=slam_bringup 의 필터 IMU 노드, '
                        'false=기존 imu_publisher 노드'),
        DeclareLaunchArgument(
            'lidar_port', default_value='/dev/ttyUSB0',
            description='라이다 시리얼 포트'),
        DeclareLaunchArgument(
            'map_resolution', default_value='0.01',
            description='/map 격자 해상도[m]. 서브맵 격자(0.01)와 맞춘 값. '
                        'rviz 가 밀리면 map_resolution:=0.05 로 띄울 것. '
                        'SLAM 내부 정확도(서브맵 1cm)와는 무관하다.'),
        DeclareLaunchArgument(
            'map_publish_period', default_value='2.0',
            description='/map 발행 주기[s]. 무선이 느리면 늘릴 것.'),
        DeclareLaunchArgument(
            'imu_rate', default_value='100.0',
            description='IMU 발행 주파수[Hz]'),
        DeclareLaunchArgument(
            'median_window', default_value='3',
            description='IMU 중위값 필터 창 크기(홀수). 클수록 노이즈는 줄지만 지연 증가'),
        DeclareLaunchArgument(
            'cmd_vel_topic', default_value='/cmd_vel',
            description='모터 명령 토픽. 이게 없거나 0 이면 IMU 적분을 멈춘다.'),
        DeclareLaunchArgument(
            'zero_when_stationary', default_value='true',
            description='정지 판정 시 각속도를 0 으로 발행(드리프트 누적 차단)'),
        DeclareLaunchArgument(
            'require_cmd_vel_publisher', default_value='true',
            description='cmd_vel 발행 노드가 아예 없으면 항상 정지로 간주'),
        DeclareLaunchArgument(
            'gyro_motion_threshold', default_value='0.05',
            description='명령이 없어도 이 값[rad/s] 이상 돌면 이동으로 판정. '
                        '손으로 밀 때 실제 회전이 지워지는 것을 막는다.'),
    ]

    # --- 센서 -------------------------------------------------------------
    lidar_node = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(use_lidar),
        parameters=[
            os.path.join(config_dir, 'ydlidar.yaml'),
            {'port': lidar_port},
        ],
    )

    # use_imu 와 use_filtered_imu 조합으로 둘 중 하나만 뜬다.
    filtered_imu_cond = IfCondition(PythonExpression([
        "'", use_imu, "' == 'true' and '", use_filtered_imu, "' == 'true'"]))
    original_imu_cond = IfCondition(PythonExpression([
        "'", use_imu, "' == 'true' and '", use_filtered_imu, "' != 'true'"]))

    # slam_bringup 자체 IMU 노드 (바이어스 보정 + 중위값 필터 + 정지 게이팅)
    imu_filtered = Node(
        package='slam_bringup',
        executable='imu_filter_node.py',
        name='imu_filter',
        output='screen',
        emulate_tty=True,
        condition=filtered_imu_cond,
        parameters=[{
            'rate_hz': ParameterValue(imu_rate, value_type=float),
            'median_window': ParameterValue(median_window, value_type=int),
            'cmd_vel_topic': ParameterValue(cmd_vel_topic, value_type=str),
            'zero_rate_when_stationary': ParameterValue(
                zero_when_stationary, value_type=bool),
            'require_cmd_vel_publisher': ParameterValue(
                require_cmd_vel_publisher, value_type=bool),
            'gyro_motion_threshold': ParameterValue(
                gyro_motion_threshold, value_type=float),
        }],
    )

    # 기존 imu_publisher 패키지 노드 (비교용 폴백, 원본 그대로)
    imu_original = Node(
        package='imu_publisher',
        executable='imu_node',
        name='imu_publisher',
        output='screen',
        emulate_tty=True,
        condition=original_imu_cond,
    )

    # --- 좌표계 -----------------------------------------------------------
    # 라이다가 뒤집혀 장착되어 있어 roll 180도.
    tf_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_laser',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0.1',
            '--yaw', '0', '--pitch', '0', '--roll', '3.14159',
            '--frame-id', 'base_link', '--child-frame-id', 'laser_frame',
        ],
    )

    tf_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_imu',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0.05',
            '--yaw', '0', '--pitch', '0', '--roll', '0',
            '--frame-id', 'base_link', '--child-frame-id', 'imu_link',
        ],
    )

    # --- SLAM -------------------------------------------------------------
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        arguments=[
            '-configuration_directory', config_dir,
            '-configuration_basename', 'my_robot.lua',
        ],
        remappings=[
            ('scan', '/scan'),
            ('imu', '/imu'),
        ],
    )

    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='occupancy_grid_node',
        output='screen',
        arguments=[
            '-resolution', map_resolution,
            '-publish_period_sec', map_publish_period,
        ],
    )

    # --- 시각화 -----------------------------------------------------------
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(args + [
        lidar_node,
        imu_filtered,
        imu_original,
        tf_laser,
        tf_imu,
        cartographer_node,
        occupancy_grid_node,
        rviz_node,
    ])
