#!/usr/bin/env bash
# ==========================================================
# auto_drive 실행 래퍼
#   - 실행 후(Ctrl+C 등으로 꺼지면): 이 런치가 띄운 것만 골라 정리
#   → /cmd_vel 구독자 중복 에러가 다시는 안 뜬다.
#
# 실행 '전' 청소는 auto_drive.launch.py 가 알아서 한다
# (_kill_stale_launches + _OWNED). 그래서 여기서는 안 한다.
#
# 사용:  ~/ping_detour_auto_drive/run.sh
#        (인자를 주면 그대로 launch 에 전달됨:  run.sh robot_radius:=0.12 등)
# ==========================================================
set -u

# 이름만 보고 지우면(예전 구현의 pkill -f "rviz2" / "cartographer") 팀원이 띄운
# 노드까지 죽는다 — 이 장비는 계정 하나를 셋이 같이 쓴다. 그래서 경로까지 붙여
# 우리 것만 고른다. rviz2 는 설정 파일 경로로 구분한다.
#
# 참고: ros2 launch 부모는 SIGINT(-2) 로 끝내야 자식까지 정리된다. SIGTERM/-9 로
# 죽이면 부모만 사라지고 rviz2·robot_state_publisher 가 고아로 남는다(실측 확인).
cleanup() {
    echo
    echo "[autodrive] 이 런치가 띄운 프로세스만 정리 중..."
    pkill -2 -f -- "ping_detour_auto_drive/auto_drive.launch.py" 2>/dev/null   # 먼저 정상 종료
    sleep 3
    for pat in \
        "ping_detour_auto_drive/auto_drive.launch.py" \
        "ping_detour_auto_drive/auto_drive.rviz" \
        "ping_detour_auto_drive/path_follower_node.py" \
        "ping_detour_auto_drive/goal_path_planner_node.py" \
        "ping_detour_auto_drive/auto_align_node.py" \
        "map_compare/map_diff_node.py" \
        "map_path/path_planner_node.py" \
        "UART/node.py" \
        "slam_bringup/lib/slam_bringup/imu_filter_node.py" \
        "ydlidar_ros2_driver_node" \
        "cartographer_ros/cartographer_node" \
        "cartographer_ros/cartographer_occupancy_grid_node" \
        "static_tf_base_to_laser" \
        "static_tf_base_to_imu"
    do
        pkill -9 -f -- "$pat" 2>/dev/null
    done
    sleep 1
    echo "[autodrive] 청소 완료."
}

# 종료(정상 종료/Ctrl+C/kill) 시 자동 청소
trap cleanup EXIT

source /opt/ros/humble/setup.bash      2>/dev/null
source "$HOME/ros2_ws/install/setup.bash" 2>/dev/null

echo "[autodrive] 실행 시작 — 종료하려면 Ctrl+C (종료 시 자동 청소됨)"
echo "----------------------------------------------------------"
ros2 launch "$HOME/auto_drive/auto_drive.launch.py" \
    motor:=true heading_offset:=180.0 auto_heading_offset:=false \
    robot_length:=0.25 robot_width:=0.17 robot_radius:=0.13 safety_margin:=0.015 \
    "$@"
