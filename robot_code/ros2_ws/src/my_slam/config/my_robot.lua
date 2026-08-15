include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu_link",
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
  -- 화면 갱신 주기는 늦춰도 정확도엔 영향 없음 -> 유지 (CPU 부담 완화용)
  submap_publish_period_sec = 1.0,
  pose_publish_period_sec = 10e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.max_range = 8.0  -- 방이 크면 원거리 벽 정보를 스캔매칭 힌트로 더 활용
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 8.0

-- odom(엔코더)이 없으므로 위치 추정은 전적으로 스캔매칭에 달려 있다.
-- 2개를 묶으면 묶는 200ms 동안 로봇이 회전한 만큼 점군이 밀린 채로 삽입되어
-- 벽이 호(arc) 모양으로 휘고, 그 틀어진 위치에 새 맵이 겹쳐 그려진다. 1개씩 처리.
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1

TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true

-- 해상도는 1cm 유지
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.01
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.01

-- 서브맵 하나에 담는 스캔 수를 늘린다(기본 90).
-- 서브맵이 클수록 맵이 "적은 수의 큰 강체 조각"으로 만들어져
-- 조각 경계에서 어긋나 찢어지는 현상이 줄고, 넓혀갈 때 이어붙임이 매끄럽다.
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 120

-- 해상도 1cm 를 유지하는 대신 여기서 CPU 를 되찾는다.
-- correlative 매처 비용은 (탐색범위/해상도)^2 에 비례하므로
-- 0.2->0.1 만으로 4배 싸진다. 10Hz 기준 0.1m 는 초속 1m 까지 커버하므로
-- 손으로 미는 속도에서는 충분하다. IMU 가 yaw 를 주므로 각도창도 줄일 수 있다.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(20.)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 1.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1.

-- ceres 최적화 반복 횟수는 5 -> 10으로 원복 (정밀도 확보, CPU 여유 있으면 유지)
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 10

-- 모션 필터: 너무 촘촘하지도, 너무 느슨하지도 않게 절충
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 5.
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.15
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(2.)

-- ↓↓↓ 가장 중요한 원복: loop closure(백엔드 최적화) 반드시 켜야 함 ↓↓↓
-- 0으로 두면 오차가 전혀 보정 안 되고 그대로 누적되어 스미어링이 악화됨
POSE_GRAPH.optimize_every_n_nodes = 20
-- 1e2 는 기본값(1e1)의 10배. huber_scale 이 클수록 손실함수가 넓은 구간에서
-- 2차식이라 "이상치 제거" 기능이 죽는다. 잘못된 loop closure 구속 하나가
-- 맵 전체를 끌고 다니며 찌그러뜨린다 -> 기본값으로 되돌려 이상치를 눌러준다.
POSE_GRAPH.optimization_problem.huber_scale = 1e1
-- 0.65 는 cartographer 기본값(0.55)보다 엄격해서 loop closure 가 거의 안 걸린다.
-- 같은 자리로 돌아와도 "예전 맵과 같은 곳"이라고 인정하지 않으니
-- 옆에 어긋난 새 맵을 또 그린다 -> 맵이 누적되지 않고 바뀌어 보이는 직접 원인.
POSE_GRAPH.constraint_builder.min_score = 0.55

return options
