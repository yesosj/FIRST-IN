#!/usr/bin/env python3
"""
room_explorer.launch.py
----------------------------------------------------------
방 탐사 미션 = ping_detour_auto_drive 전체(auto_drive.launch.py 포함)
             + room_explorer_node.py (탐사 조율)

기존 ping_detour_auto_drive 는 한 파일도 수정하지 않고 그대로 include 한다.
장애물 인식/우회(A* 재계획), 경로 추종, 도면 자동정렬, IMU 보정, RViz 는
전부 거기 것을 재사용한다.

실행
  ros2 launch ~/room_explorer/room_explorer.launch.py
  ros2 launch ~/room_explorer/room_explorer.launch.py motor:=false   # UART 따로 띄운 경우
  ros2 launch ~/room_explorer/room_explorer.launch.py reference:=~/converted_maps/2d_ex.yaml

기본값은 사용자가 실차로 검증한 조합을 그대로 쓴다:
  heading_offset 180 (라이다 180도 역장착) / control_mode smooth /
  turn_mode arc / lookahead 0.20 / robot_radius 0.11 / dynamic_obstacles true
"""

import os
import signal
import time

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HOME = os.path.expanduser("~")
THIS_DIR = os.path.dirname(os.path.abspath(__file__))
PING_DIR = os.path.join(HOME, "ping_detour_auto_drive")
AUTO_LAUNCH = os.path.join(PING_DIR, "auto_drive.launch.py")
EXPLORER_NODE = os.path.join(THIS_DIR, "room_explorer_node.py")
DEFAULT_REF = os.path.join(PING_DIR, "maps", "maze_195x162_fix.yaml")
SYS_PY = "/usr/bin/python3"

_SHELLS = ("bash", "sh", "dash", "zsh", "ksh", "fish")


def _procs():
    """(pid, argv) 목록. 셸은 뺀다 — auto_drive.launch.py 와 같은 이유
    (셸은 명령 전체를 인자로 들고 있어 이름만 보고 지우면 터미널까지 죽는다)."""
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
        if not argv or os.path.basename(argv[0]) in _SHELLS:
            continue
        out.append((int(entry), argv))
    return out


def _cleanup_previous():
    """이전 room_explorer 실행을 정리한다.

    * 이전 'ros2 launch ... room_explorer.launch.py' 부모에게 SIGINT.
      (SIGTERM 은 자식이 살아남는다 — auto_drive README 의 실측 그대로)
      부모가 정리되면 자기가 include 한 auto_drive 노드들도 같이 내려간다.
    * 그래도 남은 room_explorer_node.py 고아만 마저 지운다.
      auto_drive 계열 고아(플래너/추종기/라이다 등)는 include 되는
      auto_drive.launch.py 의 _cleanup_leftovers 가 이어서 정리한다.
    """
    me = os.getpid()
    keep = {me, os.getppid(), 1}
    stale = []
    for pid, argv in _procs():
        if pid in keep:
            continue
        if any(os.path.basename(a) == "ros2" for a in argv) \
                and any(a.endswith("room_explorer.launch.py") for a in argv):
            stale.append(pid)
    if stale:
        print("[room_explorer] 이전 실행 %d개를 먼저 종료합니다 (SIGINT)."
              % len(stale))
        for pid in stale:
            try:
                os.kill(pid, signal.SIGINT)
            except OSError:
                pass
        alive = set(stale)
        for _ in range(16):
            time.sleep(0.5)
            running = {p for p, _ in _procs()}
            alive &= running
            if not alive:
                break
        for pid in alive:
            try:
                os.kill(pid, signal.SIGKILL)
            except OSError:
                pass
    orphans = [pid for pid, argv in _procs()
               if pid not in keep
               and any("room_explorer_node.py" in a
                       or "room_explorer.rviz" in a for a in argv)]
    for pid in orphans:
        try:
            os.kill(pid, signal.SIGTERM)
        except OSError:
            pass
    if orphans:
        time.sleep(1.0)


def generate_launch_description():
    _cleanup_previous()

    ld = [
        # ---- auto_drive 로 넘기는 주행 인자 (검증된 조합이 기본값) --------
        DeclareLaunchArgument("reference", default_value=DEFAULT_REF,
                              description="기준 2D 도면 yaml"),
        DeclareLaunchArgument("heading_offset", default_value="180.0",
                              description="라이다 180도 역장착 실측 보정값"),
        DeclareLaunchArgument("control_mode", default_value="smooth"),
        DeclareLaunchArgument("turn_mode", default_value="arc"),
        DeclareLaunchArgument("lookahead", default_value="0.20"),
        # ★ 부풀림 = robot_radius + safety_margin = 0.112 가 이 도면의 상한.
        #   0.115 부터 구석 슬롯 2곳(방 #1/#11) 입구가 막혀 경로가 안 나온다
        #   (2026-08-05 스윕 실측: 0.112 는 14개 방 전부 연결).
        DeclareLaunchArgument("robot_radius", default_value="0.095"),
        DeclareLaunchArgument("safety_margin", default_value="0.015"),
        # 벽에서 이만큼 더 떨어진 길을 선호(소프트 — 통행 영역 안 줄어듦)
        DeclareLaunchArgument("clearance_prefer", default_value="0.10"),
        # 방 정중앙 도착 허용 오차. 6cm 아래로 줄이면 감속을 해도 관성으로
        # 원을 지나쳐 왔다갔다(헌팅)할 수 있다 — 그때는 다시 키울 것.
        # 이 값이 동작하려면 목적지 근처 감속(min_linear<max)이 필요하다.
        DeclareLaunchArgument("goal_tolerance", default_value="0.06"),
        DeclareLaunchArgument("dynamic_obstacles", default_value="true",
                              description="도면에 없는 장애물 감지+우회"),
        DeclareLaunchArgument("planner", default_value="drawing",
                              description="drawing 필수 — 탐사는 도면 좌표로 "
                                          "방을 정하므로 live 는 쓸 수 없다"),
        DeclareLaunchArgument("motor", default_value="true",
                              description="UART/node.py 를 따로 띄웠으면 false"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("auto_align", default_value="true"),
        # ★ 정렬 '추적' 켬 (auto_drive 기본은 0 = 1회 고정).
        #   실측(2026-08-05): 제자리 회전 중 cartographer 위치가 빗살 한 칸
        #   미끄러졌는데 1회 고정 정렬은 복구를 못 해 이후 모든 방이 한 칸
        #   옆으로 갔다. 추적 모드는 정지할 때마다 다시 재서 당겨온다.
        #   탐사 노드는 갱신을 파라미터 이벤트로 즉시 반영한다.
        DeclareLaunchArgument("refine_sec", default_value="2.0",
                              description="정렬 재측정 주기. 0=1회 고정"),
        DeclareLaunchArgument("own_imu", default_value="true"),
        DeclareLaunchArgument("speed_scale", default_value="1.0"),
        # ★ 평상시 주행은 최대 출력(max_linear 1.0 = PWM 255), 다만
        #   '방 정중앙 정밀 도착'(goal_tolerance 0.06)을 위해 목적지 앞
        #   45cm(slow_down_distance) 구간만 최소 출력으로 감속한다.
        #   min 을 1.0 으로 두면(이전 '항상 최대' 설정) 감속이 없어져
        #   6cm 원을 관성으로 지나쳐 왔다갔다한다 — 정밀 도착과 항상
        #   최대는 물리적으로 양립 불가라 접근 구간만 양보했다.
        #   항상 최대로 되돌리려면(정밀 도착 포기, goal_tolerance:=0.12 권장)
        #   min_linear:=1.0 min_angular:=1.0 min_wheel_cmd:=1.0
        DeclareLaunchArgument("min_linear", default_value="0.70"),
        DeclareLaunchArgument("min_angular", default_value="0.80"),
        DeclareLaunchArgument("min_wheel_cmd", default_value="0.80"),
        DeclareLaunchArgument("start_x", default_value="1.72"),
        DeclareLaunchArgument("start_y", default_value="1.39"),
        DeclareLaunchArgument("start_yaw", default_value="356.0"),
        DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
        DeclareLaunchArgument("lidar_port", default_value="/dev/ttyUSB0"),

        # ---- 탐사 노드 인자 ----------------------------------------------
        DeclareLaunchArgument("h_door", default_value="0.04",
                              description="방 나누기 문턱[m] — 작을수록 잘게"),
        DeclareLaunchArgument("min_room_area", default_value="0.05",
                              description="이보다 작은 조각은 방이 아님[m^2]"),
        DeclareLaunchArgument("min_room_clear", default_value="0.13",
                              description="방 최대 벽여유가 이보다 작으면 제외[m]"),
        DeclareLaunchArgument("border_margin", default_value="0.10",
                              description="도면 가장자리 여백 방 제외[m]"),
        # ★★ 탐사할 방을 도면 좌표로, 방문 순서대로 지정한다 (2026-08-06
        #   사용자 지정: 빗살 위/아래 줄의 왼쪽 3칸씩 = 실제 방 6개.
        #   오른쪽 끝 칸(x 1.67)과 가운데 통로는 방이 아니라서 뺐다).
        #   순서: 위줄 오른쪽->왼쪽(1,2,3) -> 아래줄 왼쪽->오른쪽(4,5,6) 지그재그.
        #
        #   왜 번호(exclude_rooms) 대신 좌표인가 — 도면을 조금만 고쳐도
        #   자동 분할 개수가 바뀌어 번호가 밀린다(실측: 14개->10개가 되며
        #   #3 이 다른 방을 가리켜 엉뚱한 곳으로 갔다). 좌표는 실제 미로에
        #   고정이라 도면을 고쳐도 그대로 유효하다.
        DeclareLaunchArgument(
            "room_points",
            default_value=("1.19,1.30; 0.71,1.30; 0.25,1.34; "
                           "0.25,0.26; 0.71,0.26; 1.19,0.26"),
            description="탐사할 방 정중앙 좌표(도면 기준) 'x,y; x,y; ...' "
                        "— 이 순서대로 방문. 빈 값이면 자동 분할 사용"),
        DeclareLaunchArgument("room_point_refine", default_value="0.12",
                              description="지정 좌표를 주변 이 반경 안에서 "
                                          "벽에서 가장 먼 자리로 보정[m]"),
        # 아래 둘은 room_points 가 빈 값일 때만 쓰인다(자동 분할 + 번호 제외).
        DeclareLaunchArgument("exclude_rooms", default_value="",
                              description="자동 분할 사용 시 제외할 방 번호"),
        DeclareLaunchArgument("include_rooms", default_value="",
                              description="자동 분할 사용 시 이 번호만 탐사"),
        DeclareLaunchArgument("spin_deg", default_value="360.0",
                              description="방 정중앙에서 도는 스캔 각도"),
        DeclareLaunchArgument("spin_timeout_sec", default_value="60.0"),
        DeclareLaunchArgument("spin_pause_every_deg", default_value="120.0",
                              description="회전 중 이 각도마다 잠깐 멈춰 "
                                          "SLAM 위치 재고정 (0=멈춤 없음)"),
        DeclareLaunchArgument("spin_pause_sec", default_value="1.5"),
        DeclareLaunchArgument("post_spin_settle_sec", default_value="5.0",
                              description="회전 후 다음 이동 전 정착 대기"),
        DeclareLaunchArgument("blocked_skip_sec", default_value="8.0",
                              description="막힘/경로없음이 이만큼 지속되면 "
                                          "그 방을 건너뛴다 (플래너 재시도 "
                                          "1.5초 x5회 + 장애물 TTL 3초 여유)"),
        DeclareLaunchArgument("goal_timeout_sec", default_value="180.0",
                              description="방 하나에 쓰는 최대 시간"),
        DeclareLaunchArgument("settle_sec", default_value="10.0",
                              description="출발 전 SLAM/정렬 안정화 대기"),
        # 자동 정렬(도면-라이다 매칭)이 실제로 적용된 것을 확인한 뒤에만
        # 출발한다. auto_align:=false 로 끄면 이 게이트도 같이 꺼진다.
        DeclareLaunchArgument("require_align",
                              default_value=LaunchConfiguration("auto_align"),
                              description="정렬 매칭 확인 후 출발 게이트"),
        DeclareLaunchArgument("align_apply_timeout_sec", default_value="0.0",
                              description="0=매칭될 때까지 무한 대기, "
                                          ">0 이면 그 시간 뒤 경고 후 출발"),
        DeclareLaunchArgument("retry_skipped", default_value="true",
                              description="한 바퀴 돈 뒤 못 간 방 재시도"),
        DeclareLaunchArgument("return_home", default_value="true",
                              description="다 돌면 출발 자리로 복귀"),
    ]

    # 0) RViz — auto_drive.rviz 에 방 마커/순서선 디스플레이를 더한 설정.
    #    ★ 반드시 include 보다 먼저 넣어야 한다. include 에 넘기는
    #    use_rviz:="false" 는 같은 스코프의 설정을 덮어써서(누수), include
    #    뒤에 두면 이 노드의 IfCondition 까지 false 가 되어 RViz 가 아예
    #    안 뜬다 (2026-08-05 12:49 실행에서 실제로 그랬다 — 미니 재현으로
    #    확증). 앞에 두면 조건이 사용자가 준 use_rviz 값으로 평가된다.
    ld.append(Node(
        package="rviz2", executable="rviz2", name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(THIS_DIR, "room_explorer.rviz")],
        condition=IfCondition(LaunchConfiguration("use_rviz"))))

    # 1) 기존 자율주행 전체 (SLAM + 정렬 + 플래너 + 추종기 + 모터).
    #    auto_drive.launch.py 는 스스로 이전 실행을 청소한다.
    #    RViz 는 위에서 우리 설정으로 띄우므로 여기서는 끈다.
    ld.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(AUTO_LAUNCH),
        launch_arguments={
            "reference": LaunchConfiguration("reference"),
            "heading_offset": LaunchConfiguration("heading_offset"),
            "control_mode": LaunchConfiguration("control_mode"),
            "turn_mode": LaunchConfiguration("turn_mode"),
            "lookahead": LaunchConfiguration("lookahead"),
            "robot_radius": LaunchConfiguration("robot_radius"),
            "safety_margin": LaunchConfiguration("safety_margin"),
            "clearance_prefer": LaunchConfiguration("clearance_prefer"),
            "goal_tolerance": LaunchConfiguration("goal_tolerance"),
            "dynamic_obstacles": LaunchConfiguration("dynamic_obstacles"),
            "planner": LaunchConfiguration("planner"),
            "motor": LaunchConfiguration("motor"),
            # RViz 는 우리 설정(방 마커·순서선 포함)으로 직접 띄운다
            "use_rviz": "false",
            "auto_align": LaunchConfiguration("auto_align"),
            "refine_sec": LaunchConfiguration("refine_sec"),
            "own_imu": LaunchConfiguration("own_imu"),
            "speed_scale": LaunchConfiguration("speed_scale"),
            "min_linear": LaunchConfiguration("min_linear"),
            "min_angular": LaunchConfiguration("min_angular"),
            "min_wheel_cmd": LaunchConfiguration("min_wheel_cmd"),
            "start_x": LaunchConfiguration("start_x"),
            "start_y": LaunchConfiguration("start_y"),
            "start_yaw": LaunchConfiguration("start_yaw"),
            "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
            "lidar_port": LaunchConfiguration("lidar_port"),
        }.items()))

    # 2) 방 탐사 조율 노드 — 다른 것들이 자리 잡을 시간을 조금 준 뒤 띄운다.
    #    (노드 스스로도 SLAM/정렬/플래너가 준비될 때까지 기다린다)
    ld.append(TimerAction(period=3.0, actions=[ExecuteProcess(
        name="room_explorer",
        cmd=[SYS_PY, EXPLORER_NODE, "--ros-args",
             "-p", ["reference:=", LaunchConfiguration("reference")],
             "-p", ["h_door:=", LaunchConfiguration("h_door")],
             "-p", ["min_room_area:=", LaunchConfiguration("min_room_area")],
             "-p", ["min_room_clear:=", LaunchConfiguration("min_room_clear")],
             "-p", ["border_margin:=", LaunchConfiguration("border_margin")],
             # ★ 값을 따옴표로 감싼다 — 빈 값이면 "-p include_rooms:=" 가
             #   되는데 rcl 이 이걸 파싱하지 못해 노드가 시작하자마자 죽는다
             #   (실측: Couldn't parse parameter override rule. 그래서 방
             #   마커가 안 나왔다). 따옴표를 붙이면 빈 값도 '' 로 안전하다.
             "-p", ["room_points:='",
                    LaunchConfiguration("room_points"), "'"],
             "-p", ["room_point_refine:=",
                    LaunchConfiguration("room_point_refine")],
             "-p", ["exclude_rooms:='",
                    LaunchConfiguration("exclude_rooms"), "'"],
             "-p", ["include_rooms:='",
                    LaunchConfiguration("include_rooms"), "'"],
             "-p", ["inflate_radius:=", LaunchConfiguration("robot_radius")],
             "-p", ["spin_deg:=", LaunchConfiguration("spin_deg")],
             "-p", ["spin_timeout_sec:=",
                    LaunchConfiguration("spin_timeout_sec")],
             "-p", ["spin_pause_every_deg:=",
                    LaunchConfiguration("spin_pause_every_deg")],
             "-p", ["spin_pause_sec:=", LaunchConfiguration("spin_pause_sec")],
             "-p", ["post_spin_settle_sec:=",
                    LaunchConfiguration("post_spin_settle_sec")],
             "-p", ["blocked_skip_sec:=",
                    LaunchConfiguration("blocked_skip_sec")],
             "-p", ["goal_timeout_sec:=",
                    LaunchConfiguration("goal_timeout_sec")],
             "-p", ["settle_sec:=", LaunchConfiguration("settle_sec")],
             "-p", ["require_align:=", LaunchConfiguration("require_align")],
             "-p", ["align_apply_timeout_sec:=",
                    LaunchConfiguration("align_apply_timeout_sec")],
             "-p", ["retry_skipped:=", LaunchConfiguration("retry_skipped")],
             "-p", ["return_home:=", LaunchConfiguration("return_home")],
             "-p", ["cmd_vel_topic:=", LaunchConfiguration("cmd_vel_topic")],
             "-p", ["start_x:=", LaunchConfiguration("start_x")],
             "-p", ["start_y:=", LaunchConfiguration("start_y")],
             "-p", ["start_yaw:=", LaunchConfiguration("start_yaw")]],
        output="screen")]))

    return LaunchDescription(ld)
