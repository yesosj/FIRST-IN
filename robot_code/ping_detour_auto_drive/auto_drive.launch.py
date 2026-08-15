#!/usr/bin/env python3
"""
auto_drive.launch.py
----------------------------------------------------------
터미널 하나로 "SLAM + 도면비교 + 경로계획 + 자율주행(경로추종) + 모터 + RViz(로봇모델)"
를 전부 띄운다.

  source ~/ros2_ws/install/setup.bash
  ros2 launch ~/auto_drive/auto_drive.launch.py

구성 (기존 파일은 그대로 실행만 한다 — 수정/복사 없음)
  1) slam_bringup/slam.launch.py (use_rviz:=false)
       라이다(/scan) + IMU + static TF + cartographer SLAM
       -> 실시간 /map, TF map->odom->base_link (= 현재 위치)
  2) ~/map_compare/map_diff_node.py     기준도면 /reference_map + 차이
  3) ~/map_path/path_planner_node.py    /goal_pose -> 차체 고려 A* 경로 /plan
  4) ~/auto_drive/path_follower_node.py /plan 추종 -> /cmd_vel        <-- 신규
  5) ~/UART/node.py                     /cmd_vel -> UART 모터 PWM
  6) robot_state_publisher (auto_drive/robot_model.urdf)  -> RViz RobotModel
  7) rviz2 (auto_drive/auto_drive.rviz — 로봇모델 포함)

사용법
  1) 위 명령으로 실행 (로봇은 도면상 start_x/start_y/start_yaw 위치에서 출발)
  2) RViz 상단 "2D Goal Pose" 로 목적지 클릭
  3) 경로(초록선)가 생기면 로봇이 바로 그 경로를 따라 주행
  4) 목적지 도착 시 모터 정지 + 터미널에 "목적지에 도착했습니다." 출력
     + /auto_drive/goal_reached (std_msgs/Bool) 발행

주요 인자
  motor:=false          UART 모터 노드를 여기서 안 띄움 (다른 터미널에서 이미 실행 중일 때)
  drive:=false          경로추종 끄기 (경로만 보고 싶을 때)
  diff:=false           도면 비교 노드 끄기
  use_rviz:=false       RViz 끄기
  goal_tolerance:=0.12  도착 판정 오차[m]
  lookahead:=0.12       경로 추종 전방주시 거리[m]
  use_scan_guard:=false 라이다 앞쪽 비상정지 끄기
  reference:=<yaml>     기준 도면
  start_x/start_y/start_yaw  도면상 로봇 출발 위치/방향 (map_diff_node 와 동일 규약)

정지시키려면
  ros2 topic pub --once /auto_drive/enable std_msgs/Bool "{data: false}"
  ros2 topic pub --once /auto_drive/cancel std_msgs/Empty "{}"
----------------------------------------------------------
"""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction,
                            IncludeLaunchDescription, TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

HOME = os.path.expanduser("~")
# 이 런치 파일이 있는 폴더를 그대로 쓴다. 경로를 박아 두면 폴더를 옮겨도
# 여전히 ~/auto_drive 를 보게 되어 엉뚱한 파일이 실행된다.
AUTO_DIR = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.basename(AUTO_DIR)

# 기준 도면. map_compare 원본이 아니라 auto_drive 안의 수정판을 쓴다.
#   원본 maze_195x162 는 오른쪽 세로 바가 위/아래 구석 바깥벽보다
#   벽 두께 하나(0.05 m)만큼 왼쪽으로 밀려 있어서, 바 바깥에 폭 0.05 m 짜리
#   막다른 슬롯이 남고 오른쪽 벽선이 끊겨 보였다. 수정판은 그 바를 붙여 뒀다.
#   원본으로 돌리려면 reference:=~/map_compare/custom_maps/maze_195x162.yaml
DEFAULT_REF = os.path.join(AUTO_DIR, "maps", "final_map.yaml")
DIFF_NODE = os.path.join(HOME, "map_compare", "map_diff_node.py")
PLANNER_NODE = os.path.join(HOME, "map_path", "path_planner_node.py")
FOLLOWER_NODE = os.path.join(AUTO_DIR, "path_follower_node.py")
GOAL_PLANNER = os.path.join(AUTO_DIR, "goal_path_planner_node.py")
ALIGN_NODE   = os.path.join(AUTO_DIR, "auto_align_node.py")
IMU_NODE     = os.path.join(AUTO_DIR, "imu_fix_node.py")
MOTOR_NODE = os.path.join(HOME, "UART", "node.py")
URDF_FILE = os.path.join(AUTO_DIR, "robot_model.urdf")
RVIZ_CFG = os.path.join(AUTO_DIR, "auto_drive.rviz")

SYS_PY = "/usr/bin/python3"

# --- 재실행 안전장치 --------------------------------------------------------
# ros2 launch 를 Ctrl-C 로 끄면 자식 프로세스가 남는 일이 잦다. 그 상태에서 다시
# 띄우면 라이다 드라이버가 둘이 되어 같은 시리얼 포트를 물고 스캔이 깨지거나,
# 이전 추종기가 /cmd_vel 에 0 을 계속 쏴서 로봇이 안 움직인다.
# generate_launch_description() 안에서 정리하므로 다른 노드보다 먼저 실행된다.
#
# 이 런치가 직접 띄우는 것만 지운다. 사용자가 따로 띄운 UART/node.py 나
# 다른 사람의 노드(camera_node 등)는 건드리지 않는다.
#
# rviz2 는 설정 파일 경로로 우리 것만 골라낸다. 이름만으로 지우면 팀원이 띄운
# rviz2 까지 죽는다(이 장비는 계정 하나를 셋이 같이 쓴다).
_OWNED = (
    PKG + "/auto_drive.rviz",       # 부모가 이미 죽어 고아가 된 우리 rviz2
    "ydlidar_ros2_driver_node",
    "cartographer_node",
    "cartographer_occupancy_grid_node",
    "imu_filter_node.py",
    "map_path/path_planner_node.py",
    PKG + "/goal_path_planner_node.py",
    "map_compare/map_diff_node.py",
    PKG + "/path_follower_node.py",
    PKG + "/auto_align_node.py",
    PKG + "/imu_fix_node.py",
    "static_tf_base_to_laser",
    "static_tf_base_to_imu",
)


_SHELLS = ("bash", "sh", "dash", "zsh", "ksh", "fish")


def _procs():
    """(pid, argv, cmdline) 목록. 좀비/읽기실패/셸은 뺀다.

    ★ 셸을 빼는 게 중요하다. 셸은 실행할 명령 전체를 인자 문자열로 들고 있어서
      'auto_drive.launch.py' 같은 단어가 그 안에 그대로 들어 있다. 문자열 포함만
      보고 지우면 그 명령을 띄운 터미널까지 같이 죽는다(실제로 그랬다).
    """
    out = []
    for entry in os.listdir("/proc"):
        if not entry.isdigit():
            continue
        try:
            with open("/proc/%s/cmdline" % entry, "rb") as f:
                raw = f.read()
        except OSError:
            continue
        if not raw:
            continue
        argv = [a for a in raw.decode("utf-8", "replace").split("\x00") if a]
        if not argv:
            continue
        if os.path.basename(argv[0]) in _SHELLS:
            continue
        out.append((int(entry), argv, " ".join(argv)))
    return out


def _kill_stale_launches():
    """이전에 띄워 둔 이 런치의 부모 프로세스를 먼저 끝낸다.

    부모를 정상 종료시키면 자기가 띄운 것들(rviz2, robot_state_publisher,
    UART/node.py 등)을 스스로 정리한다. 그래서 자식을 이름으로 하나하나
    지우는 것보다 안전하고 깨끗하다 — 남의 rviz2 를 죽일 일이 없다.

    ★ 반드시 SIGINT 여야 한다. 실측으로 확인했다:
        SIGTERM -> 부모만 죽고 자식(sleep 600 x2)이 그대로 살아남음
        SIGINT  -> 부모와 자식이 함께 정리됨
      ros2 launch 는 Ctrl+C(SIGINT)에서만 종료 절차를 밟는다. 예전에 SIGTERM 을
      보냈다가 rviz2 와 robot_state_publisher 가 매번 고아로 쌓였다.

    generate_launch_description() 은 'ros2 launch' 프로세스 안에서 돌기 때문에
    os.getpid() 가 곧 지금 이 런치다. 그것만 빼면 자기 자신은 안 죽인다.
    """
    import signal
    import time
    me = os.getpid()
    keep = {me, os.getppid(), 1}
    # 'ros2 launch ... auto_drive.launch.py' 인 프로세스만 고른다.
    # argv 원소 단위로 확인해야 남의 명령 문자열에 우연히 걸리지 않는다.
    stale = []
    for pid, argv, cmd in _procs():
        if pid in keep:
            continue
        if not any(os.path.basename(a) == "ros2" for a in argv):
            continue
        if not any(a.endswith("auto_drive.launch.py") for a in argv):
            continue
        stale.append((pid, cmd))
    if not stale:
        return
    print("[auto_drive] 이전 실행(launch) %d개를 먼저 종료합니다 — "
          "자기가 띄운 rviz2/모터노드까지 같이 정리됩니다:" % len(stale))
    for pid, cmd in stale:
        print("             %6d  %s" % (pid, cmd.strip()[:70]))
        try:
            os.kill(pid, signal.SIGINT)
        except OSError:
            pass
    # 부모가 자식을 정리할 시간을 준다 (최대 8초)
    alive = {pid for pid, _ in stale}
    for _ in range(16):
        time.sleep(0.5)
        running = {p for p, _, _ in _procs()}
        alive &= running
        if not alive:
            break
    for pid in alive:                    # 그래도 안 죽으면 강제로
        try:
            os.kill(pid, signal.SIGKILL)
        except OSError:
            pass


def _cleanup_leftovers():
    import signal
    import time
    _kill_stale_launches()
    me = os.getpid()
    keep = {me, os.getppid(), 1}
    # 위에서 부모를 끝냈어도 살아남은 고아들을 마저 지운다
    victims = [(pid, cmd.strip()[:70]) for pid, argv, cmd in _procs()
               if pid not in keep
               and any(any(pat in a for pat in _OWNED) for a in argv)]
    if not victims:
        return
    print("[auto_drive] 이전 실행에서 남은 프로세스 %d개를 정리합니다:" % len(victims))
    for pid, cmd in victims:
        print("             %6d  %s" % (pid, cmd))
        try:
            os.kill(pid, signal.SIGTERM)
        except OSError:
            pass
    time.sleep(2.0)
    for pid, _ in victims:
        try:
            os.kill(pid, signal.SIGKILL)
        except OSError:
            pass
    time.sleep(0.5)
    print("[auto_drive] 정리 완료 — 처음 실행과 같은 상태로 시작합니다.")




# reference 인자의 '~' 를 런타임에 펼쳐 준다.
#   셸은 "reference:=~/path" 의 ~ 를 확장하지 않는다(유효한 대입어가 아님).
#   map_diff_node / path_planner_node 는 expanduser 를 기본값에만 적용하므로
#   ~ 가 그대로 넘어가면 도면을 못 열고 /reference_map 이 발행되지 않는다.
def _expanded(cfg):
    return PythonExpression(
        ["__import__('os').path.expanduser('", cfg, "')"])

def generate_launch_description():
    # --show-args/static validation must never stop a robot that is already
    # running. Real launches leave this variable unset and retain the exact
    # original cleanup behaviour.
    if (
        os.environ.get("FINAL_DEMO_TEST_VALIDATE") != "1"
        and "--show-args" not in sys.argv
    ):
        _cleanup_leftovers()

    ld = [
        DeclareLaunchArgument("reference", default_value=DEFAULT_REF,
                              description="기준 도면 yaml 경로"),
        DeclareLaunchArgument("map_resolution", default_value="0.01",
                              description="실시간 /map 격자 해상도[m]. 기준 도면(0.01)과 "
                                          "맞춰야 map_diff 의 빨강/파랑 차이 표시가 "
                                          "곱게 나온다(0.05 면 5칸, 0.01 이면 286칸). "
                                          "이 방은 1x1m 라 0.01 이어도 격자가 130x130 "
                                          "≈17k셀 뿐이라 가볍다. 넓은 공간에서 CPU가 "
                                          "밀리면 0.05 로 낮출 것"),
        DeclareLaunchArgument(
            "slam_imu", default_value="true",
            description="cartographer 에 IMU 사용(기본). imu_fix 가 시작/런타임 "
                        "에 걸린 칩을 자동 리셋해 살린다. IMU 가 완전히 죽어 "
                        "복구가 안 되면 slam_imu:=false 로 스캔매칭 전용 폴백."),
        DeclareLaunchArgument("own_imu", default_value="true",
                              description="slam_bringup 의 IMU 노드 대신 "
                                          "imu_fix_node.py 를 쓴다. 이 개체는 "
                                          "자이로 바이어스가 정상의 3~15배, "
                                          "가속도 스케일 9.5%% 부족, 장착 9.3도 "
                                          "기울어짐이 실측됐다. 게다가 기존 노드는 "
                                          "정지 판정을 /cmd_vel 로 해서, 드리프트로 "
                                          "회전 명령이 나가면 드리프트 차단이 스스로 "
                                          "꺼지는 자기강화 고리가 생긴다(로봇모델이 "
                                          "계속 360도 도는 현상). false 면 기존 노드"),
        DeclareLaunchArgument("imu_calib_seconds", default_value="3.0",
                              description="기동 시 바이어스/중력벡터를 재는 시간[s]. "
                                          "이 동안 로봇을 완전히 정지시켜야 한다"),
        DeclareLaunchArgument("imu_bias_alpha", default_value="0.02",
                              description="정지 판정될 때마다 바이어스를 따라가는 "
                                          "속도. 기존 노드는 0.001 이라 온도 변화로 "
                                          "바이어스가 변해도 못 따라갔다"),
        DeclareLaunchArgument("fix_lidar", default_value="true",
                              description="라이다를 여기서 직접 띄우고 "
                                          "fixed_resolution=false 로 고쳐 쓴다. "
                                          "false 면 slam.launch.py 의 설정 그대로"),
        DeclareLaunchArgument("lidar_port", default_value="/dev/ttyUSB0",
                              description="라이다 시리얼 포트. 라이다가 다른 포트에 "
                                          "잡히면 여기서 바꿀 것 (ls /dev/ttyUSB*)"),
        DeclareLaunchArgument("start_x", default_value="1.72",
                              description="TF 원점(=로봇 출발 자리)의 도면 좌표 X[m]. "
                                          "maze_195x162 의 오른쪽 위 구석. 이 값이 "
                                          "map 프레임 (0,0) 이 되고, 그래야 RViz 에서 "
                                          "TF 원점이 도면 왼쪽 아래에 온다"),
        DeclareLaunchArgument("start_y", default_value="1.39",
                              description="TF 원점(=로봇 출발 자리)의 도면 좌표 Y[m]. "
                                          "위 start_x 와 같은 구석"),
        DeclareLaunchArgument("start_yaw", default_value="356.0",
                              description="TF 원점에서 로봇이 바라본 방향[도]. "
                                          "auto_align 실측값. 180 으로 두면 176도 "
                                          "틀려서 기동 직후 몇 초간 도면이 어긋나 "
                                          "보인다(오차 0.089m, 도면내 점 39%)"),

        DeclareLaunchArgument("robot_length", default_value="0.26",
                              description="차체 길이(전진방향)[m]"),
        DeclareLaunchArgument("robot_width", default_value="0.18",
                              description="차체 폭[m]"),
        DeclareLaunchArgument("lidar_yaw", default_value="0.0",
                              description="라이다가 차체에 대해 돌아 장착된 각도[도]. "
                                          "이 값을 주면 base_link->laser_frame static TF "
                                          "를 그 yaw 로 다시 발행해 '스캔 자체를 돌려' "
                                          "도면과 맞춘다(도면을 돌리지 않는다). "
                                          "auto_align 이 측정해서 권장값을 알려준다"),
        DeclareLaunchArgument("yaw_range", default_value="180.0",
                              description="자동정렬의 각도 탐색 범위[도]. 180=전체 "
                                          "탐색(기본). 작게 주면 start_yaw 주변만 "
                                          "보므로 지정한 방향이 반대편 해로 뒤집히지 "
                                          "않는다 (예: yaw_range:=45)"),
        DeclareLaunchArgument("xy_range", default_value="0.30",
                              description="자동정렬의 위치 탐색 반경[m]. start_x/y "
                                          "주변 이 범위만 본다. 좁힐수록 지정한 "
                                          "자리에서 크게 벗어난 해가 안 나온다"),
        # 정렬을 한 번 재고 바로 믿으면 안 된다. 이 미로처럼 같은 모양이 반복되는
        # 도면은 스캔을 옆 칸으로 밀어도 점수가 거의 같아서, 잴 때마다 답이 수십 cm
        # 씩 튄다(실측: y 가 1.10~1.45 로 35cm 벌어졌는데 오차는 전부 비슷했다).
        # 그래서 여러 번 재서 서로 일치할 때만 적용하고, 아니면 지정값을 지킨다.
        DeclareLaunchArgument("align_repeats", default_value="3",
                              description="자동정렬을 이만큼 독립적으로 재서 결과가 "
                                          "일치할 때만 적용한다. 1=예전처럼 한 번만"),
        DeclareLaunchArgument("agree_xy", default_value="0.05",
                              description="측정끼리 허용하는 위치 편차[m]. 이보다 "
                                          "벌어지면 자동정렬을 포기하고 지정값 유지"),
        DeclareLaunchArgument("agree_yaw", default_value="5.0",
                              description="측정끼리 허용하는 각도 편차[도]"),
        DeclareLaunchArgument("refine_sec", default_value="0.0",
                              description="이 주기[s]로 스캔↔도면 정렬을 다시 "
                                          "재서 더 나으면 갱신한다(추적 모드). "
                                          "0 이면 시작할 때 1회만 — 그러면 "
                                          "cartographer 재최적화로 map 프레임이 "
                                          "조금씩 움직일 때 도면만 제자리에 남아 "
                                          "결국 벽이 장애물로 잡힌다 "
                                          "(실측: 1회만 -> 헛 장애물 1037칸, "
                                          "2초 추적 -> 0~14칸). 추적은 아는 자리 "
                                          "주변 ±0.06 m/±2.5도만 보고 점수가 나을 "
                                          "때만 갱신하므로 튀지 않는다"),
        DeclareLaunchArgument("align_only_when_still", default_value="true",
                              description="정렬 갱신을 로봇이 정지해 있을 때만 한다. "
                                          "주행 중에 갱신하면 도면이 로봇 발밑에서 "
                                          "움직여, 목적지와 이미 낸 경로는 그대로인데 "
                                          "벽만 옮겨져 멀쩡한 경로가 '막힘'으로 "
                                          "판정된다(재계획 폭주의 원인). "
                                          "첫 정렬은 이 설정과 무관하게 한다"),
        DeclareLaunchArgument("align_min_improve", default_value="0.003",
                              description="오차가 이만큼[m]은 좋아져야 실제로 "
                                          "갱신한다. 미세 변동으로 도면이 계속 "
                                          "떨리는 것을 막는다"),
        DeclareLaunchArgument("align_still_wait", default_value="0.8",
                              description="멈춘 뒤 이 시간[s]이 지나야 '정지'로 "
                                          "본다. 직후에는 관성/진동이 남아 있다"),
        DeclareLaunchArgument("realign_error", default_value="0.05",
                              description="정렬 오차가 이 값[m]을 realign_hits 번 "
                                          "연속 넘으면 추적을 포기하고 전역 재정렬을 "
                                          "다시 한다. 추적은 ±0.06 m/±2.5도만 보므로 "
                                          "크게 어긋나면 스스로는 못 돌아온다. "
                                          "정상 정렬 오차는 0.015~0.022 m"),
        DeclareLaunchArgument("realign_hits", default_value="3",
                              description="전역 재정렬로 되돌리기까지 필요한 "
                                          "연속 초과 횟수"),
        DeclareLaunchArgument("min_coverage", default_value="0.85",
                              description="스캔점이 도면 안에 이만큼은 들어와야 후보로 "
                                          "인정. 통과하는 후보가 없으면 실제 최대치 "
                                          "기준으로 한 번 자동 완화한다"),
        DeclareLaunchArgument("auto_align", default_value="true",
                              description="라이다 스캔을 도면에 자동으로 맞춰 "
                                          "start_x/y/yaw 를 실행 중에 직접 넣는다. "
                                          "수동으로 준 값을 그대로 쓰려면 false"),
        DeclareLaunchArgument("planner", default_value="drawing",
                              description="drawing=기준 도면 A* + 출발점은 '지금 "
                                          "로봇이 있는 자리'(기본, 권장) / "
                                          "reference=기존 path_planner_node.py "
                                          "(항상 map 원점에서 시작) / "
                                          "live=실시간 /map 에 A*"),
        DeclareLaunchArgument("plan_resolution", default_value="0.05",
                              description="live 플래너의 계획 격자[m]"),
        DeclareLaunchArgument("unknown_is_obstacle", default_value="false",
                              description="true 면 아직 안 가본 곳을 벽으로 취급해 "
                                          "탐사된 영역 안에서만 경로를 낸다. "
                                          "기본 false = 지도 전체에서 경로를 낸다 "
                                          "(안 가본 곳은 라이다 비상정지에 의존)"),
        DeclareLaunchArgument("dynamic_obstacles", default_value="true",
                              description="실시간 장애물 감지→우회/정지 기능. "
                                          "false 로 주면 도면(정적 지도)만 보고 주행하고 "
                                          "'전방 장애물로 정지'가 뜨지 않는다."),
        DeclareLaunchArgument("lidar_to_front", default_value="0.20",
                              description="base_link(라이다)에서 차체 앞면까지 거리[m]. "
                                          "실측 도면: 전체 길이 0.25 중 라이다가 "
                                          "뒤에서 0.05 지점 -> 앞면까지 0.20. "
                                          "차체 한가운데가 아니므로 길이/2 로 쓰면 안 된다"),
        DeclareLaunchArgument("spin_grace_sec", default_value="0.5",
                              description="제자리 회전이 끝난 뒤에도 이 시간[s] 동안은 "
                                          "장애물 인식을 멈춘 채로 둔다. TF/스캔 지연 "
                                          "때문에 회전 마지막 순간의 스캔이 뒤늦게 "
                                          "들어와 벽을 장애물로 잡는 것을 막는다"),
        DeclareLaunchArgument("escape_radius", default_value="0.25",
                              description="로봇이 부풀림(분홍) 안에 들어가 있으면 "
                                          "출발점 주변 이 반경만 여유 제한을 풀어 "
                                          "빠져나갈 경로를 찾는다[m]. 실제 벽과 "
                                          "장애물 셀은 그대로 막아 둔다"),
        DeclareLaunchArgument("obstacle_range", default_value="0.30",
                              description="라이다에서 이 거리 안의 스캔점만 장애물로 "
                                          "본다[m]. 차체 앞면이 0.20 이므로 0.35 면 "
                                          "앞면 기준 0.15 m 앞까지. 줄이면 더 코앞만 본다"),
        DeclareLaunchArgument("block_check_ahead", default_value="0.30",
                              description="경로가 막혔는지 검사할 구간 길이[m]. "
                                          "목적지까지 다 보면 한참 앞의 장애물 때문에 "
                                          "지금 멀쩡한데도 멈춘다"),
        DeclareLaunchArgument("front_gap", default_value="0.05",
                              description="차체 앞면에서 이 거리 안에 스캔점이 들어오면 "
                                          "도면과 무관하게 '못 간다'로 보고 장애물로 "
                                          "등록한다[m]. 판정 상자 = 진행방향으로 "
                                          "(차체길이/2 + 이 값), 좌우로 "
                                          "(차체폭/2 + safety_margin)"),
        DeclareLaunchArgument("front_min_clear", default_value="0.04",
                              description="차체 앞 상자 안이라도 도면 벽에서 이 거리 "
                                          "안에 있는 점은 장애물로 안 본다[m]. "
                                          "정렬이 통째로 어긋나면 벽이 매 스캔 "
                                          "장애물로 잡혀 주행 중 계속 멈춘다 — "
                                          "0.02 면 약 19%, 0.04 면 약 10%가 샌다. "
                                          "통로를 막은 진짜 장애물은 한가운데 여유가 "
                                          "0.14 m 라 0.04 로 올려도 그대로 잡힌다. "
                                          "0.0 을 주면 필터가 꺼진다"),
        DeclareLaunchArgument("adaptive_wall_clear", default_value="true",
                              description="벽 제외 기준을 현재 정렬 오차에 맞춰 "
                                          "자동으로 넓힌다. 고정 4 cm 로는 정렬이 "
                                          "그보다 어긋나는 순간 벽이 통째로 "
                                          "장애물이 된다(실측: 6 cm 어긋남 -> "
                                          "벽의 95%% 가 장애물). 스캔점 대부분이 "
                                          "벽이라는 성질로 오차를 재서 더한다"),
        DeclareLaunchArgument("wall_clear_max", default_value="0.10",
                              description="자동으로 넓히더라도 이 값[m]을 넘지 "
                                          "않는다. 통로를 막은 진짜 장애물은 "
                                          "한가운데 여유가 0.14 m 라 안 걸린다"),
        DeclareLaunchArgument("wall_clear_margin", default_value="0.02",
                              description="측정된 정렬 잔차에 이만큼 더해 "
                                          "기준으로 삼는다[m]"),
        DeclareLaunchArgument("wall_resid_percentile", default_value="70.0",
                              description="정렬 잔차를 재는 백분위수. 높이면 더 "
                                          "보수적(벽을 더 많이 제외)"),
        DeclareLaunchArgument("obstacle_min_cluster", default_value="4",
                              description="이만큼 붙어 있는 덩어리만 장애물로 본다. "
                                          "벽에서 새는 점은 정렬 잡음이라 한두 개씩 "
                                          "흩어지고, 진짜 물체는 라이다에 한 덩어리로 "
                                          "잡힌다. 1 이면 이 필터가 꺼진다"),
        DeclareLaunchArgument("front_confirm_scans", default_value="2",
                              description="차체 앞 장애물이 연속 이만큼의 스캔에서 "
                                          "보여야 진짜로 인정하고 정지한다. "
                                          "한 장만 보고 세우면 튀는 잡음마다 "
                                          "가다 서다를 반복한다. 스캔 11.7Hz 기준 "
                                          "2장 = 0.17초 = 0.2m/s 에서 3.4 cm 더 진행 "
                                          "(앞 여유 4 cm 안쪽). 1 = 즉시 정지(예전)"),
        DeclareLaunchArgument("blocked_memory_radius", default_value="0.10",
                              description="막힌 지점을 기억할 때 '경로를 막은 자리' "
                                          "주변 이 반경[m] 안의 장애물 셀만 남긴다. "
                                          "보이는 장애물을 전부 기억하면 정렬 잔차로 "
                                          "생긴 칸까지 쌓여 지도가 봉인된다 "
                                          "(실측: 재계획 93회에 398칸, 실시간은 7칸)"),
        DeclareLaunchArgument("blocked_memory_max", default_value="60",
                              description="기억할 셀 수 상한(CPU 보호용). 넘으면 오래된 "
                                          "것부터 버린다. 막힌 자리 주변만 "
                                          "기억하므로 한 번 막힐 때 10여 칸씩 "
                                          "늘어난다 — 500이면 40회분. "
                                          "기억이 길을 다 막으면 플래너가 "
                                          "스스로 오래된 절반을 버리므로 "
                                          "갇히지 않는다"),
        DeclareLaunchArgument("blocked_memory_until_goal", default_value="true",
                              description="true 면 막혔던 지점을 blocked_memory_ttl "
                                          "대신 '목적지 도착까지' 기억한다. "
                                          "같은 막힌 길을 다시 고르는 왕복을 막는다. "
                                          "도착/새 목적지/clear_obstacles 에서 초기화"),
        DeclareLaunchArgument("front_min_points", default_value="3",
                              description="차체 앞 상자 안에 이만큼 이상 스캔점이 "
                                          "들어와야 경고 로그를 낸다(잡음 무시용)"),
        DeclareLaunchArgument("obstacle_min_clear", default_value="0.04",
                              description="스캔점이 도면 벽에서 이만큼 이상 떨어져야 "
                                          "'도면에 없는 장애물'로 인정한다[m]. "
                                          "크면 통로를 꽉 막은 장애물이 벽으로 "
                                          "오인돼 안 보이고, 작으면 정렬오차가 "
                                          "헛 장애물로 잡힌다. "
                                          "dynamic_obstacles:=true 일 때만 의미 있음"),
        DeclareLaunchArgument("obstacle_ttl", default_value="3.0",
                              description="동적 장애물 셀 유지 시간[s]. 이후 자동 삭제"),
        DeclareLaunchArgument("blocked_memory_ttl", default_value="3.0",
                              description="재계획용 막힌 셀 임시 기억 시간[s]"),
        DeclareLaunchArgument("clearance_weight", default_value="3.0",
                              description="벽에서 멀리 가려는 정도. 0=예전처럼 순수 "
                                          "최단경로(부풀림 경계에 딱 붙음). 크면 "
                                          "돌아가더라도 통로 한가운데로 간다"),
        DeclareLaunchArgument("clearance_prefer", default_value="0.06",
                              description="벽 여유가 robot_radius + 이 값 이상이면 "
                                          "더 이상 이득 없음(벌점 0)"),
        DeclareLaunchArgument("robot_radius", default_value="0.13",
                              description="장애물 부풀림 반경[m]. 여기에 "
                                          "safety_margin 이 더해진 값이 실제 부풀림이다. "
                                          "기본은 0.13 + 0.00 = 0.13 m. "
                                          "미로 통로가 0.28 m 라 0.11 이면 양옆 여유가 "
                                          "0.03 m 뿐이라 장애물이 하나만 있어도 "
                                          "우회로가 안 나온다. 외접원 0.142 를 쓰면 "
                                          "갈 곳이 0.009 m^2 밖에 안 남아 경로가 "
                                          "아예 안 나온다"),
        DeclareLaunchArgument("safety_margin", default_value="0.0",
                              description="경로계획 시 벽에서 더 띄울 여유[m]. "
                                          "robot_radius 에 더해진다. 0 이면 실제 "
                                          "부풀림 = robot_radius 그대로. "
                                          "차체 앞 판정 상자의 좌우 여유에도 쓰이므로 "
                                          "0 이면 상자 폭 = 차체 폭(0.17 m)이 된다"),
        DeclareLaunchArgument(
            "manual_pose_min_clearance", default_value="0.0",
            description=(
                "웹 수동 로봇 배치의 최소 벽 여유[m]. "
                "0이면 도면 내부 모든 위치 허용"
            )),

        DeclareLaunchArgument("drive", default_value="true",
                              description="경로추종(자율주행) 노드 실행"),
        DeclareLaunchArgument("motor", default_value="true",
                              description="UART 모터 노드 실행. 다른 터미널에서 "
                                          "이미 돌고 있으면 false 로 줄 것"),
        DeclareLaunchArgument("diff", default_value="true",
                              description="도면 비교 노드 실행"),
        DeclareLaunchArgument("use_rviz", default_value="true",
                              description="RViz2 실행 (로봇모델 포함 설정)"),

        DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel",
                              description="모터 명령 토픽. 실제로 안 움직이고 "
                                          "시험만 하려면 /cmd_vel_test 등으로"),
        # final_demo_test 통합 배선 전용 인자. 주행 코어 코드는 그대로 두고
        # 사용자 목표와 실제 모터 명령을 중재 노드/mux에 연결할 때만 사용한다.
        DeclareLaunchArgument(
            "goal_topic", default_value="/goal_pose",
            description="goal_path_planner_node가 받을 목표 토픽"),
        DeclareLaunchArgument(
            "planner_cmd_vel_topic", default_value="/cmd_vel",
            description="동적 장애물 플래너가 관찰할 실제 모터 명령 토픽"),
        DeclareLaunchArgument("allow_reverse", default_value="false",
                              description="true 면 뒤쪽 목적지로 후진한다. "
                                          "기본 false = 항상 앞을 보고 주행"),
        DeclareLaunchArgument("turn_forward_pulse", default_value="0",
                              description="큰 방향전환 때 섞을 전진 펄스 주기수. "
                                          "0=제자리 회전, 2 정도면 전진하며 호를 그린다"),
        DeclareLaunchArgument("heading_offset", default_value="0.0",
                              description="라이다 장착 yaw 오프셋 보정[도]. "
                                          "heading_check.py 로 측정. 180 이면 "
                                          "라이다가 뒤를 보고 장착된 것"),
        DeclareLaunchArgument("first_move_forward", default_value="false",
                              description="새 경로를 받으면 경로 방향과 무관하게 "
                                          "먼저 조금 전진한다. 출발부터 경로를 "
                                          "벗어나므로 기본은 꺼 둔다. 켜면 그 "
                                          "직진으로 '앞이 어디인가'도 자동 측정한다"),
        DeclareLaunchArgument("auto_heading_offset", default_value="true",
                              description="첫 전진에서 잰 실제 이동 방향으로 "
                                          "heading_offset 을 자동 보정"),
        DeclareLaunchArgument("first_forward_dist", default_value="0.10",
                              description="첫 강제 전진 거리[m]"),
        DeclareLaunchArgument("invert_angular", default_value="false",
                              description="좌회전 명령에 로봇이 오른쪽으로 돌면 true. "
                                          "(모터 좌/우 배선이 뒤바뀐 경우)"),
        DeclareLaunchArgument("invert_linear", default_value="false",
                              description="전진 명령에 로봇이 뒤로 가면 true"),
        # ── ping_detour 검증본(연속 출력) 추종기 튜닝 — 통합 시 프리미티브
        #    추종기로 바뀌며 삭제됐던 인자들을 원본 기본값 그대로 복원 ──
        DeclareLaunchArgument("speed_scale", default_value="1.0",
                              description="전체 속도 배율"),
        DeclareLaunchArgument("max_linear", default_value="1.00",
                              description="직진 출력 상한 (정규화)"),
        DeclareLaunchArgument("min_linear", default_value="0.70",
                              description="직진 출력 하한 — 코너 감속의 바닥"),
        DeclareLaunchArgument("max_angular", default_value="1.00",
                              description="회전 출력 상한"),
        DeclareLaunchArgument("min_angular", default_value="0.80",
                              description="회전 출력 하한"),
        DeclareLaunchArgument("min_wheel_cmd", default_value="0.80",
                              description="바퀴별 최소 명령 (정지마찰 보상, PWM 204)"),
        DeclareLaunchArgument("key_linear", default_value="2.0",
                              description="keys 모드 직진 램프"),
        DeclareLaunchArgument("key_angular", default_value="2.0",
                              description="keys 모드 회전 램프"),
        DeclareLaunchArgument("spin_power", default_value="1.0",
                              description="제자리 회전 출력"),
        DeclareLaunchArgument("spin_nudge_lin", default_value="-0.75",
                              description="회전 고착 시 흔들기 전진량 (음수=뒤부터)"),
        DeclareLaunchArgument("control_mode", default_value="keys",
                              description="keys=keyboard_test.py 의 w/a/d 와 같은 "
                                          "명령만 사용(섞지 않음) / smooth=비례제어"),
        DeclareLaunchArgument("stuck_min_turn_deg", default_value="8.0",
                              description="교착 판정에서 '움직였다'고 볼 최소 회전각. "
                                          "제자리 회전은 선형 이동이 0 이므로 각도로 봐야 "
                                          "한다"),
        DeclareLaunchArgument("turn_lead_sec", default_value="0.4",
                              description="회전 명령을 끊고도 관성/지연으로 더 도는 "
                                          "시간[s]. 이 값 x 실측 각속도 만큼 미리 "
                                          "끊는다. 더 지나치면 키우고, 목표에 못 "
                                          "미치면 줄일 것"),
        DeclareLaunchArgument("turn_pulse_on", default_value="4",
                              description="keys 회전 ON 주기수(20Hz 기준). 오버슛하면 줄일 것"),
        DeclareLaunchArgument("turn_pulse_off", default_value="0",
                              description="keys 회전 OFF 주기수. 0=끊지 않고 연속 "
                                          "회전(기본). 예전 값 2 는 200ms 돌고 "
                                          "100ms 정지라 상판이 무거우면 정지마찰을 "
                                          "매번 다시 깨야 해서 제자리 회전이 안 됐다. "
                                          "오버슛이 심하면 1~2 로 올릴 것"),
        DeclareLaunchArgument("spin_on_new_path", default_value="true",
                              description="새 경로(최초 목적지 / 우회로)를 받은 "
                                          "첫 시작에서 방향이 많이 틀어져 있으면 "
                                          "맞을 때까지 제자리 회전만 한다. "
                                          "경로 중간의 좌/우회전은 turn_mode 대로 "
                                          "(arc) 돈다. arc 는 최소 회전반경 때문에 "
                                          "우회로의 반전을 못 하므로 필요하다"),
        DeclareLaunchArgument("spin_enter_angle", default_value="0.0",
                              description="새 경로 시작 시 이 각도[rad]를 넘으면 "
                                          "제자리 회전으로 방향을 맞춘다. "
                                          "0 = turn_in_place_angle(0.60=34도)"),
        DeclareLaunchArgument("spin_exit_angle", default_value="0.0",
                              description="그 제자리 회전을 끝내는 각도[rad]. "
                                          "들어가는 각도와 달라야(히스테리시스) "
                                          "도는 중에 arc 로 새지 않는다. "
                                          "0 = key_turn_on(10도)"),
        DeclareLaunchArgument("spin_stall_sec", default_value="2.0",
                              description="제자리 회전 명령을 내는데 이 시간[s] 동안 "
                                          "spin_stall_deg 만큼도 못 돌면 "
                                          "정지마찰로 보고 흔들기를 시작한다"),
        DeclareLaunchArgument("spin_stall_deg", default_value="3.0",
                              description="정지마찰 판정 각도[도]"),
        DeclareLaunchArgument("spin_nudge_sec", default_value="0.35",
                              description="흔들기 1회 지속 시간[s]. 0 이면 흔들기 끔"),
        DeclareLaunchArgument("spin_latch_timeout", default_value="15.0",
                              description="제자리 회전으로 이 시간[s] 안에 방향을 "
                                          "못 맞추면 경고하고 풀어 준다. "
                                          "회전이 물리적으로 안 될 때 영원히 "
                                          "갇히는 것을 막는 안전장치"),
        DeclareLaunchArgument("turn_in_place_angle", default_value="0.60",
                              description="방향오차가 이 값[rad]을 넘으면 큰 방향전환으로 "
                                          "본다. turn_mode:=spin 이면 이때부터 전진을 "
                                          "끊고 제자리 회전만 한다. 0.60 rad = 34도. "
                                          "줄이면 더 작은 각도부터 제자리 회전"),
        DeclareLaunchArgument("turn_mode", default_value="arc",
                              description="큰 방향전환 방식. arc=전진하며 호를 그림"
                                          "(안쪽 바퀴 역회전 없음, 제자리회전 못 하는 "
                                          "차체용) / spin=제자리 회전"),
        DeclareLaunchArgument("lookahead", default_value="0.12",
                              description="전방주시 거리[m]. 클수록 코너를 크게 자른다. "
                                          "90도 코너 최대이탈 실측: 0.30->0.075m, "
                                          "0.18->0.043m, 0.12->0.029m. 경로가 벽에서 "
                                          "0.13m 떨어져 있고 차체 반폭이 0.09m 이므로 "
                                          "허용 이탈은 0.04m -> 0.12 를 쓴다"),
        DeclareLaunchArgument("goal_tolerance", default_value="0.12",
                              description="도착 판정 오차[m]"),
        DeclareLaunchArgument("use_scan_guard", default_value="false",
                              description="라이다 앞쪽 비상정지. 기본 false — 경로가 "
                                          "이미 차체 여유를 반영하고, 이 방(통로 0.36m)"
                                          "에서는 코너마다 걸려 교착을 만든다. "
                                          "대신 교착 감지(stuck_timeout)가 알려준다"),
        DeclareLaunchArgument("obstacle_stop_distance", default_value="0.0",
                              description="앞쪽 정지거리[m]. 0 이면 차체에서 자동계산"),
    ]

    # 1) 하드웨어 + SLAM (RViz 는 아래에서 우리 설정으로 하나만 띄운다)
    #
    # 반드시 GroupAction(scoped=True) 로 감쌀 것.
    # IncludeLaunchDescription 은 스코프를 push/pop 하지 않아서
    # (launch/actions/include_launch_description.py: return [*set_..., launch_description])
    # 여기서 넘긴 use_rviz:=false 가 이 파일의 use_rviz 까지 덮어써 버린다.
    # 그러면 아래 7) 의 우리 rviz2 가 조건 false 가 되어 아예 안 뜬다.
    slam_share = get_package_share_directory("slam_bringup")
    slam_launch = os.path.join(slam_share, "launch", "slam.launch.py")
    ydlidar_yaml = os.path.join(slam_share, "config", "ydlidar.yaml")

    # 라이다는 slam.launch.py 대신 여기서 직접 띄운다 (use_lidar:=false).
    # 이유: ydlidar.yaml 의 fixed_resolution=true 가 이 장치와 맞지 않는다.
    #   설정은 sample_rate 5 x frequency 10 => 회전당 430점을 기대하는데
    #   실제 장치는 1041점을 낸다. CPU 부하가 있으면 시리얼 읽기가 밀려
    #   회전 경계를 놓치고, 드라이버가 그 점들을 430 슬롯에 억지로 담는다
    #   ("Real points 1041 > fixed points 430"). 그러면 각도와 거리가 어긋나
    #   cartographer 가 스캔을 버리고(Ignored subdivision) 맵이 1m 크기로
    #   쪼그라든다 -> 경로계획 불가, 주행 엉망.
    #   같은 부하에서 실측 비교:
    #     fixed_resolution=true  : 유효점 53%, 빈 방위구간 8/36, 경고 27회
    #     fixed_resolution=false : 유효점 70%, 빈 방위구간 2/36, 경고 0회
    #   (range_max 는 64.0 그대로 둘 것. 12.0 으로 줄이면 먼 벽을 버려 13% 로 떨어진다)
    ld.append(Node(
        package="ydlidar_ros2_driver", executable="ydlidar_ros2_driver_node",
        name="ydlidar_ros2_driver_node", output="screen", emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("fix_lidar")),
        parameters=[ydlidar_yaml,
                    {"port": LaunchConfiguration("lidar_port"),
                     "fixed_resolution": False}]))

    # slam_imu:=true 일 때만 기존 slam.launch.py(IMU 사용 my_robot.lua)를 쓴다.
    ld.append(GroupAction(
        scoped=True,
        condition=IfCondition(LaunchConfiguration("slam_imu")),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(slam_launch),
                launch_arguments={
                    "use_rviz": "false",
                    "use_lidar": PythonExpression(
                        ["'false' if '", LaunchConfiguration("fix_lidar"),
                         "' == 'true' else 'true'"]),
                    "map_resolution": LaunchConfiguration("map_resolution"),
                    "lidar_port": LaunchConfiguration("lidar_port"),
                    # own_imu:=true 면 slam_bringup 의 IMU 노드를 끄고
                    # 우리 imu_fix_node.py 가 /imu 를 낸다.
                    "use_imu": PythonExpression(
                        ["'false' if '", LaunchConfiguration("own_imu"),
                         "' == 'true' else 'true'"]),
                }.items()),
        ]))

    # slam_imu:=false(기본): IMU 없이 스캔매칭만 쓰는 cartographer 를 직접
    # 띄운다. slam.launch.py 는 lua 파일을 바꿀 인자가 없어서(my_robot.lua
    # 고정) 그 launch 가 띄우던 정적 TF 2개 + cartographer + occupancy_grid 를
    # 같은 값으로 여기서 띄운다. slam_bringup 패키지는 수정하지 않는다.
    # 라이다는 fix_lidar:=true(기본)면 위에서 이미 직접 띄웠고, false 면
    # slam.launch 가 맡던 것을 여기서 대신 띄운다.
    no_imu_slam = UnlessCondition(LaunchConfiguration("slam_imu"))
    ld.append(Node(
        package="tf2_ros", executable="static_transform_publisher",
        name="static_tf_base_to_laser", condition=no_imu_slam,
        arguments=["--x", "0", "--y", "0", "--z", "0.1",
                   "--yaw", "0", "--pitch", "0", "--roll", "3.14159",
                   "--frame-id", "base_link",
                   "--child-frame-id", "laser_frame"]))
    ld.append(Node(
        package="tf2_ros", executable="static_transform_publisher",
        name="static_tf_base_to_imu", condition=no_imu_slam,
        arguments=["--x", "0", "--y", "0", "--z", "0.05",
                   "--yaw", "0", "--pitch", "0", "--roll", "0",
                   "--frame-id", "base_link",
                   "--child-frame-id", "imu_link"]))
    ld.append(Node(
        package="ydlidar_ros2_driver", executable="ydlidar_ros2_driver_node",
        name="ydlidar_ros2_driver_node", output="screen", emulate_tty=True,
        condition=IfCondition(PythonExpression(
            ["'", LaunchConfiguration("slam_imu"), "' != 'true' and '",
             LaunchConfiguration("fix_lidar"), "' != 'true'"])),
        parameters=[ydlidar_yaml,
                    {"port": LaunchConfiguration("lidar_port")}]))
    ld.append(Node(
        package="cartographer_ros", executable="cartographer_node",
        name="cartographer_node", output="screen", condition=no_imu_slam,
        arguments=[
            "-configuration_directory", os.path.join(AUTO_DIR, "config"),
            "-configuration_basename", "my_robot_no_imu.lua"],
        remappings=[("scan", "/scan")]))
    ld.append(Node(
        package="cartographer_ros",
        executable="cartographer_occupancy_grid_node",
        name="occupancy_grid_node", output="screen", condition=no_imu_slam,
        arguments=["-resolution", LaunchConfiguration("map_resolution"),
                   "-publish_period_sec", "0.5"]))

    # 1a) 보정된 IMU (own_imu:=true 일 때). slam_bringup 쪽 IMU 노드는
    #     use_imu:=false 로 꺼 두었고, base_link->imu_link static TF 는
    #     조건 없이 계속 나오므로 노드만 갈아끼우면 된다.
    #
    # IMU가 종료돼도 통합 launch 전체를 내리지 않는다. Cartographer의 IMU
    # 구성에서는 /map과 TF가 멈출 수 있지만 follower가 0.6초 지난 TF를
    # 거부하고 정지하며, 웹/카메라/열화상/Pi bridge는 계속 살아 있어
    # 진단할 수 있다. IMU 없는 SLAM은 다음 실행에서 slam_imu:=false를 쓴다.
    ld.append(ExecuteProcess(
        condition=IfCondition(LaunchConfiguration("own_imu")),
        cmd=[SYS_PY, IMU_NODE, "--ros-args",
             "-p", ["calib_seconds:=",
                    LaunchConfiguration("imu_calib_seconds")],
             "-p", ["bias_alpha:=", LaunchConfiguration("imu_bias_alpha")],
             "-p", "imu_topic:=/imu",
             "-p", "frame_id:=imu_link"],
        output="screen"))

    # 1b) 라이다 장착 yaw 보정 — slam.launch.py 가 --yaw 0 으로 발행한 뒤에
    #     같은 부모/자식으로 다시 발행하면 나중 것이 이긴다(3/3 재현 확인).
    #     도면을 돌리는 대신 '스캔을 돌려서' 맞추고 싶을 때 쓴다.
    ld.append(TimerAction(period=3.0, actions=[Node(
        package="tf2_ros", executable="static_transform_publisher",
        name="lidar_yaw_fix", output="screen",
        condition=IfCondition(PythonExpression(
            ["str(", LaunchConfiguration("lidar_yaw"), ") not in ('0','0.0','-0.0')"])),
        arguments=[
            "--x", "0", "--y", "0", "--z", "0.1",
            "--yaw", PythonExpression(
                ["str(float(", LaunchConfiguration("lidar_yaw"), ") * 3.141592653589793 / 180.0)"]),
            "--pitch", "0", "--roll", "3.14159",
            "--frame-id", "base_link", "--child-frame-id", "laser_frame"])]))

    # 2) 기준 도면 발행 + 실시간 비교
    ld.append(ExecuteProcess(
        condition=IfCondition(LaunchConfiguration("diff")),
        cmd=[SYS_PY, DIFF_NODE, "--ros-args",
             "-p", ["reference:=", _expanded(LaunchConfiguration("reference"))],
             "-p", "live_topic:=/map", "-p", "frame:=map",
             "-p", ["start_x:=", LaunchConfiguration("start_x")],
             "-p", ["start_y:=", LaunchConfiguration("start_y")],
             "-p", ["start_yaw:=", LaunchConfiguration("start_yaw")]],
        output="screen"))

    # 3a) 경로계획(권장) — 실시간 SLAM 맵 /map 에 직접 A*.
    #     /map 은 이미 로봇 TF 와 같은 좌표계라 start_x/y/yaw 정렬이 필요 없다.
    ld.append(ExecuteProcess(
        condition=IfCondition(PythonExpression(
            ["'", LaunchConfiguration("planner"), "' in ('live', 'drawing')"])),
        cmd=[SYS_PY, GOAL_PLANNER, "--ros-args",
             "-p", ["source:=", PythonExpression(
                 ["'map' if '", LaunchConfiguration("planner"),
                  "' == 'live' else 'drawing'"])],
             "-p", ["reference:=", _expanded(LaunchConfiguration("reference"))],
             "-p", ["start_x:=", LaunchConfiguration("start_x")],
             "-p", ["start_y:=", LaunchConfiguration("start_y")],
             "-p", ["start_yaw:=", LaunchConfiguration("start_yaw")],
             "-p", "map_topic:=/map", "-p", "base_frame:=base_link",
             "-p", ["robot_length:=", LaunchConfiguration("robot_length")],
             "-p", ["robot_width:=", LaunchConfiguration("robot_width")],
             "-p", ["safety_margin:=", LaunchConfiguration("safety_margin")],
             "-p", ["robot_radius:=", LaunchConfiguration("robot_radius")],
             "-p", ["clearance_weight:=", LaunchConfiguration("clearance_weight")],
             "-p", ["clearance_prefer:=", LaunchConfiguration("clearance_prefer")],
             "-p", ["plan_resolution:=", LaunchConfiguration("plan_resolution")],
             "-p", ["unknown_is_obstacle:=",
                    LaunchConfiguration("unknown_is_obstacle")],
             "-p", ["dynamic_obstacles:=",
                    LaunchConfiguration("dynamic_obstacles")],
             "-p", ["obstacle_min_clear:=",
                    LaunchConfiguration("obstacle_min_clear")],
             "-p", ["obstacle_ttl:=",
                    LaunchConfiguration("obstacle_ttl")],
             "-p", ["blocked_memory_ttl:=",
                    LaunchConfiguration("blocked_memory_ttl")],
             "-p", ["front_gap:=", LaunchConfiguration("front_gap")],
             "-p", ["lidar_to_front:=", LaunchConfiguration("lidar_to_front")],
             "-p", ["obstacle_range:=", LaunchConfiguration("obstacle_range")],
             "-p", ["escape_radius:=", LaunchConfiguration("escape_radius")],
             "-p", ["spin_grace_sec:=", LaunchConfiguration("spin_grace_sec")],
             "-p", ["cmd_vel_topic:=",
                    LaunchConfiguration("planner_cmd_vel_topic")],
             "-p", ["block_check_ahead:=",
                    LaunchConfiguration("block_check_ahead")],
             "-p", ["front_min_points:=",
                    LaunchConfiguration("front_min_points")],
             "-p", ["front_min_clear:=",
                    LaunchConfiguration("front_min_clear")],
             "-p", ["front_confirm_scans:=",
                    LaunchConfiguration("front_confirm_scans")],
             "-p", ["adaptive_wall_clear:=",
                    LaunchConfiguration("adaptive_wall_clear")],
             "-p", ["wall_clear_max:=",
                    LaunchConfiguration("wall_clear_max")],
             "-p", ["wall_clear_margin:=",
                    LaunchConfiguration("wall_clear_margin")],
             "-p", ["wall_resid_percentile:=",
                    LaunchConfiguration("wall_resid_percentile")],
             "-p", ["obstacle_min_cluster:=",
                    LaunchConfiguration("obstacle_min_cluster")],
             "-p", ["blocked_memory_until_goal:=",
                    LaunchConfiguration("blocked_memory_until_goal")],
             "-p", ["blocked_memory_radius:=",
                    LaunchConfiguration("blocked_memory_radius")],
             "-p", ["blocked_memory_max:=",
                    LaunchConfiguration("blocked_memory_max")],
             "-r", ["/goal_pose:=", LaunchConfiguration("goal_topic")]],
        output="screen"))

    # 3b) 기존 도면 기반 플래너 (planner:=reference 일 때만)
    ld.append(ExecuteProcess(
        condition=IfCondition(PythonExpression(
            ["'", LaunchConfiguration("planner"), "' == 'reference'"])),
        cmd=[SYS_PY, PLANNER_NODE, "--ros-args",
             "-p", ["reference:=", _expanded(LaunchConfiguration("reference"))],
             "-p", "frame:=map",
             "-p", ["start_x:=", LaunchConfiguration("start_x")],
             "-p", ["start_y:=", LaunchConfiguration("start_y")],
             "-p", ["start_yaw:=", LaunchConfiguration("start_yaw")],
             "-p", ["robot_length:=", LaunchConfiguration("robot_length")],
             "-p", ["robot_width:=", LaunchConfiguration("robot_width")],
             "-p", ["robot_radius:=", LaunchConfiguration("robot_radius")],
             "-p", ["safety_margin:=", LaunchConfiguration("safety_margin")],
             "-r", ["/goal_pose:=", LaunchConfiguration("goal_topic")]],
        output="screen"))

    # 3c) 도면 정렬 + 웹 수동 로봇 배치 수신기.
    #     path_planner_node / map_diff_node 는 매번 get_parameter 로 읽으므로
    #     즉시 반영된다. auto_align:=false에서도 수동 배치는
    #     받아야 하므로 프로세스는 항상 띄우고 자동 측정만 끄고 켠다.
    ld.append(ExecuteProcess(
        cmd=[SYS_PY, ALIGN_NODE, "--ros-args",
             "-p", "targets:=path_planner_node,map_diff_node,goal_path_planner_node",
             "-p", ["auto_align_enabled:=", LaunchConfiguration("auto_align")],
             "-p", ["reference:=", _expanded(LaunchConfiguration("reference"))],
             "-p", ["start_x:=", LaunchConfiguration("start_x")],
             "-p", ["start_y:=", LaunchConfiguration("start_y")],
             "-p", ["start_yaw:=", LaunchConfiguration("start_yaw")],
             "-p", ["yaw_range:=", LaunchConfiguration("yaw_range")],
             "-p", ["xy_range:=", LaunchConfiguration("xy_range")],
             "-p", ["repeats:=", LaunchConfiguration("align_repeats")],
             "-p", ["agree_xy:=", LaunchConfiguration("agree_xy")],
             "-p", ["agree_yaw:=", LaunchConfiguration("agree_yaw")],
             "-p", ["refine_sec:=", LaunchConfiguration("refine_sec")],
             "-p", ["min_coverage:=", LaunchConfiguration("min_coverage")],
             "-p", ["align_only_when_still:=",
                    LaunchConfiguration("align_only_when_still")],
             "-p", ["align_min_improve:=",
                    LaunchConfiguration("align_min_improve")],
             "-p", ["align_still_wait:=",
                    LaunchConfiguration("align_still_wait")],
             "-p", ["realign_error:=", LaunchConfiguration("realign_error")],
             "-p", ["realign_hits:=", LaunchConfiguration("realign_hits")],
             "-p", ["robot_width:=", LaunchConfiguration("robot_width")],
             "-p", ["robot_radius:=", LaunchConfiguration("robot_radius")],
             "-p", ["safety_margin:=", LaunchConfiguration("safety_margin")],
             "-p", ["manual_pose_min_clearance:=",
                    LaunchConfiguration("manual_pose_min_clearance")],
             "-p", ["lidar_yaw:=", LaunchConfiguration("lidar_yaw")],
             "-p", ["heading_offset:=", LaunchConfiguration("heading_offset")]],
        output="screen"))

    # 4) 경로 추종(자율주행) — /plan + TF(map->base_link) -> /cmd_vel
    ld.append(ExecuteProcess(
        condition=IfCondition(LaunchConfiguration("drive")),
        cmd=[SYS_PY, FOLLOWER_NODE, "--ros-args",
             "-p", ["cmd_vel_topic:=", LaunchConfiguration("cmd_vel_topic")],
             "-p", "path_topic:=/plan",
             "-p", "map_frame:=map", "-p", "base_frame:=base_link",
             "-p", ["allow_reverse:=", LaunchConfiguration("allow_reverse")],
             "-p", ["turn_forward_pulse:=", LaunchConfiguration("turn_forward_pulse")],
             "-p", ["heading_offset:=", LaunchConfiguration("heading_offset")],
             "-p", ["first_move_forward:=", LaunchConfiguration("first_move_forward")],
             "-p", ["auto_heading_offset:=", LaunchConfiguration("auto_heading_offset")],
             "-p", ["first_forward_dist:=", LaunchConfiguration("first_forward_dist")],
             "-p", ["invert_angular:=", LaunchConfiguration("invert_angular")],
             "-p", ["invert_linear:=", LaunchConfiguration("invert_linear")],
             "-p", ["control_mode:=", LaunchConfiguration("control_mode")],
             "-p", ["speed_scale:=", LaunchConfiguration("speed_scale")],
             "-p", ["max_linear:=", LaunchConfiguration("max_linear")],
             "-p", ["min_linear:=", LaunchConfiguration("min_linear")],
             "-p", ["max_angular:=", LaunchConfiguration("max_angular")],
             "-p", ["min_angular:=", LaunchConfiguration("min_angular")],
             "-p", ["min_wheel_cmd:=", LaunchConfiguration("min_wheel_cmd")],
             "-p", ["key_linear:=", LaunchConfiguration("key_linear")],
             "-p", ["key_angular:=", LaunchConfiguration("key_angular")],
             "-p", ["spin_power:=", LaunchConfiguration("spin_power")],
             "-p", ["spin_nudge_lin:=", LaunchConfiguration("spin_nudge_lin")],
             "-p", ["stuck_min_turn_deg:=", LaunchConfiguration("stuck_min_turn_deg")],
             "-p", ["turn_lead_sec:=", LaunchConfiguration("turn_lead_sec")],
             "-p", ["turn_pulse_on:=", LaunchConfiguration("turn_pulse_on")],
             "-p", ["turn_pulse_off:=", LaunchConfiguration("turn_pulse_off")],
             "-p", ["turn_mode:=", LaunchConfiguration("turn_mode")],
             "-p", ["turn_in_place_angle:=",
                    LaunchConfiguration("turn_in_place_angle")],
             "-p", ["spin_on_new_path:=",
                    LaunchConfiguration("spin_on_new_path")],
             "-p", ["spin_enter_angle:=",
                    LaunchConfiguration("spin_enter_angle")],
             "-p", ["spin_exit_angle:=",
                    LaunchConfiguration("spin_exit_angle")],
             "-p", ["spin_latch_timeout:=",
                    LaunchConfiguration("spin_latch_timeout")],
             "-p", ["spin_stall_sec:=",
                    LaunchConfiguration("spin_stall_sec")],
             "-p", ["spin_stall_deg:=",
                    LaunchConfiguration("spin_stall_deg")],
             "-p", ["spin_nudge_sec:=",
                    LaunchConfiguration("spin_nudge_sec")],
             "-p", ["lookahead:=", LaunchConfiguration("lookahead")],
             "-p", ["goal_tolerance:=", LaunchConfiguration("goal_tolerance")],
             "-p", ["robot_length:=", LaunchConfiguration("robot_length")],
             "-p", ["robot_width:=", LaunchConfiguration("robot_width")],
             "-p", ["use_scan_guard:=", LaunchConfiguration("use_scan_guard")],
             "-p", ["obstacle_stop_distance:=",
                    LaunchConfiguration("obstacle_stop_distance")]],
        output="screen"))

    # 5) UART 모터 드라이버 — /cmd_vel 을 좌/우 PWM 으로 (기존 파일 그대로 실행)
    ld.append(ExecuteProcess(
        condition=IfCondition(LaunchConfiguration("motor")),
        cmd=[SYS_PY, MOTOR_NODE],
        output="screen"))

    # 6) 로봇 모델 — RViz RobotModel 용 /robot_description
    #    (링크 1개짜리 URDF 라 TF 는 추가로 발행되지 않는다)
    with open(URDF_FILE, "r") as f:
        robot_description = f.read()
    ld.append(Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        name="robot_state_publisher", output="screen",
        parameters=[{"robot_description": robot_description,
                     "publish_frequency": 10.0}]))

    # 7) RViz — 로봇모델 + 도면 + 실시간맵 + 경로 + 추종점/도착 표시
    ld.append(Node(
        package="rviz2", executable="rviz2", name="rviz2",
        output="screen", arguments=["-d", RVIZ_CFG],
        condition=IfCondition(LaunchConfiguration("use_rviz"))))

    return LaunchDescription(ld)
