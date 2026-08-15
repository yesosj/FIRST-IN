-- my_robot_no_imu.lua : IMU 없이 스캔매칭만으로 도는 cartographer 설정.
--
-- slam_bringup/config/my_robot.lua 사본에서 딱 세 가지만 다르다:
--   1) tracking_frame = "base_link"  (imu_link 추적 불필요)
--   2) TRAJECTORY_BUILDER_2D.use_imu_data = false
--   3) angular_search_window 20 -> 30도 (IMU yaw 힌트가 없어진 만큼 넓힘)
--
-- 왜 필요한가: 이 로봇의 MPU6050 은 자이로 바이어스가 정상의 3~15배인
-- 불량 개체인 데다, 2026-08-04 배선 수리 후에는 가속도 0 / 자이로 고정값을
-- 뱉는 상태까지 갔다. 그 데이터가 cartographer 에 들어가면 로봇이 정지해
-- 있어도 pose 가 지도 밖까지 흘러간다(실측: 정지 22초에 26cm + yaw 한 바퀴).
-- 10Hz 라이다 스캔매칭만으로도 1m 미로 추적에는 충분하다.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = true,
  use_pose_extrapolator = true,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 1.0,
  pose_publish_period_sec = 20e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.max_range = 8.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 8.0
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1

TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true

TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.01
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.01
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 120

TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
-- IMU yaw 힌트가 없으므로 회전 탐색창을 넓힌다. 10Hz 스캔 기준 30도/장 =
-- 300도/초까지 커버 — 이 차체 제자리 회전(최대 ~200도/초)이면 충분하다.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(30.)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 1.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1.

TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 10

TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 5.
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.15
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(2.)

POSE_GRAPH.optimize_every_n_nodes = 20
POSE_GRAPH.optimization_problem.huber_scale = 1e1
POSE_GRAPH.constraint_builder.min_score = 0.55

return options
