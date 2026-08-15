#!/usr/bin/env python3
"""
room_explorer_node.py
----------------------------------------------------------
기존 2D 도면의 모든 '방'을 자동으로 찾아 차례로 방문하는 탐사 조율 노드.

동작 (한 사이클)
  1. 도면을 방 단위로 나눈다 (room_segment.py — h-maxima + priority flood)
  2. 지금 로봇 위치에서 도달 가능한 방만 남기고, 가까운 방부터 도는
     순서를 만든다 (축소 격자 BFS 의 실제 이동거리 기준 — 직선거리로
     고르면 미로에서 벽 건너 방을 먼저 고르는 수가 있다)
  3. 방의 정중앙(방 안에서 벽으로부터 가장 먼 지점)을 /goal_pose 로 보낸다
     -> 경로계획·주행·장애물 우회는 전부 ping_detour_auto_drive 의
        goal_path_planner_node + path_follower_node 가 한다 (재사용, 무수정)
  4. 도착하면 추종기를 일시정지시키고 그 자리에서 360도 제자리 회전으로
     주변을 스캔한 뒤, 다음 방으로 이동한다 (방을 나가는 것은 다음 목적지로
     가는 A* 경로가 알아서 문을 지나며 해결한다)
  5. 장애물로 우회로가 없거나(HALTED_NO_ROUTE) 경로가 안 나오면(NO_PATH)
     "장애물로 인해 갈 수 없습니다" 를 로그로 띄우고 그 방은 건너뛴다.
     한 바퀴 다 돈 뒤 건너뛴 방을 한 번 더 시도한다 (retry_skipped).
  6. 다 돌면 출발 자리로 돌아온다 (return_home).

기존 노드와의 접점 (전부 토픽/서비스 — 기존 파일 수정 없음)
  발행  /goal_pose             다음 목적지
        /auto_drive/enable     스캔 회전 동안 추종기 일시정지
        /auto_drive/cancel     방 포기 시 주행 취소
        /cmd_vel               스캔 회전 명령 (추종기 정지 중에만)
        /room_explorer/spin_active  스캔 회전 제어권 상태
        /room_explorer/status  미션 상태 JSON (latched)
        /room_explorer/markers RViz 방 표시 (초록=방문, 빨강=실패, 파랑=지금)
  구독  /auto_drive/goal_reached  도착 (latched)
        /planner/status           계획 상태 JSON (HALTED_NO_ROUTE 등)
  서비스 /<planner_node>/get_parameters  start_x/y/yaw (도면<->map 변환.
        auto_align_node 가 실행 중 갱신하는 값을 그대로 따라간다)

실행 (단독 — 보통은 room_explorer.launch.py 가 띄운다)
  python3 room_explorer_node.py --ros-args -p reference:=~/converted_maps/2d_ex.yaml

주의: rclpy/numpy/cv2 필요. venv 에 없으므로 /usr/bin/python3 로 자동 재실행.
----------------------------------------------------------
"""

import json
import math
import os
import sys

_SYS_PY = "/usr/bin/python3"
try:
    import numpy  # noqa: F401
    import cv2    # noqa: F401
    import rclpy  # noqa: F401
except ModuleNotFoundError:
    if os.path.abspath(sys.executable) != _SYS_PY and os.path.exists(_SYS_PY):
        _env = dict(os.environ)
        _env.pop("VIRTUAL_ENV", None)
        os.execve(_SYS_PY, [_SYS_PY] + sys.argv, _env)
    raise

import numpy as np
import rclpy
import tf2_ros
from rcl_interfaces.msg import ParameterDescriptor, ParameterEvent
from rcl_interfaces.srv import GetParameters
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

from geometry_msgs.msg import Point, PoseStamped, Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Empty, String
from visualization_msgs.msg import Marker, MarkerArray

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import room_segment as rs


def latched(depth=1):
    return QoSProfile(depth=depth,
                      reliability=QoSReliabilityPolicy.RELIABLE,
                      durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                      history=QoSHistoryPolicy.KEEP_LAST)


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def norm_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class RoomExplorer(Node):

    def _num(self, name, default):
        self.declare_parameter(name, default,
                               ParameterDescriptor(dynamic_typing=True))
        v = self.get_parameter(name).value
        return float(default) if v is None else float(v)

    def _flag(self, name, default):
        self.declare_parameter(name, default,
                               ParameterDescriptor(dynamic_typing=True))
        v = self.get_parameter(name).value
        if isinstance(v, str):
            return v.strip().lower() in ("1", "true", "yes", "on")
        return bool(default) if v is None else bool(v)

    def _text(self, name, default):
        self.declare_parameter(name, default,
                               ParameterDescriptor(dynamic_typing=True))
        v = self.get_parameter(name).value
        return default if v is None else str(v)

    def _points(self, name):
        """"x,y; x,y; ..." 파라미터 -> [(x, y), ...] (도면 좌표, 순서 유지).

        방을 '번호'로 지정하면 도면을 조금만 고쳐도 분할 결과가 달라져
        번호가 밀린다(2026-08-06 실제로 그랬다: 14개 -> 10개로 바뀌며
        #3 이 다른 방이 됨). 좌표는 실제 미로에 고정된 값이라 안 밀린다.
        """
        raw = self._text(name, "")
        out = []
        for tok in raw.replace("|", ";").split(";"):
            tok = tok.strip()
            if not tok:
                continue
            parts = tok.replace(":", ",").split(",")
            if len(parts) != 2:
                self.get_logger().warn(
                    "%s 의 '%s' 는 'x,y' 형식이 아닙니다 — 무시합니다."
                    % (name, tok))
                continue
            try:
                out.append((float(parts[0]), float(parts[1])))
            except ValueError:
                self.get_logger().warn(
                    "%s 의 '%s' 를 숫자로 못 읽었습니다 — 무시합니다."
                    % (name, tok))
        return out

    def _ids(self, name):
        """"4,5, 6" 같은 방 번호 목록 파라미터 -> {4, 5, 6}."""
        raw = self._text(name, "")
        out = set()
        for tok in raw.replace(";", ",").split(","):
            tok = tok.strip().lstrip("#")
            if tok:
                try:
                    out.add(int(tok))
                except ValueError:
                    self.get_logger().warn(
                        "%s 의 '%s' 는 방 번호가 아닙니다 — 무시합니다."
                        % (name, tok))
        return out

    def __init__(self):
        super().__init__("room_explorer_node")

        # --- 도면 / 방 나누기 ------------------------------------------------
        self.ref_yaml = self._text("reference", os.path.expanduser(
            "~/ping_detour_auto_drive/maps/maze_195x162_fix.yaml"))
        self.h_door = self._num("h_door", 0.04)
        self.min_area = self._num("min_room_area", 0.05)
        self.min_clear = self._num("min_room_clear", 0.13)
        self.border = self._num("border_margin", 0.10)
        # 자동 분할이 방으로 잡았지만 실제로는 방이 아닌 구역을 번호로 제외.
        # 예: exclude_rooms:="4,5,6" — 남은 방들의 번호는 바뀌지 않는다.
        # include_rooms 를 주면 그 번호만 탐사한다(화이트리스트).
        self.exclude_ids = self._ids("exclude_rooms")
        self.include_ids = self._ids("include_rooms")
        # ★ 탐사할 방을 도면 좌표로 직접, 방문 순서대로 지정한다.
        #   "x,y; x,y; ..." — 주어지면 자동 분할 결과 대신 이 점들만 쓰고
        #   순서도 그대로 따른다(가까운 방부터 재배치하지 않음).
        #   번호(exclude_rooms/include_rooms)는 도면을 고치면 밀리므로
        #   이 방식이 기본이다.
        self.room_pts = self._points("room_points")
        # 지정한 점을 그 주변에서 가장 벽에서 먼 자리로 살짝 옮긴다(정중앙 보정).
        # 반경을 작게 둬서 옆 칸으로 넘어가지 않게 한다.
        self.pt_refine = self._num("room_point_refine", 0.12)
        # 방문 순서/도달성 판단용 부풀림. 플래너와 같은 값을 주면 판단이 같아진다.
        self.inflate = self._num("inflate_radius", 0.11)
        self.order_cell = self._num("order_cell", 0.04)

        # --- 기존 노드 접점 --------------------------------------------------
        self.planner_node = self._text("planner_node", "goal_path_planner_node")
        self.goal_topic = self._text("goal_topic", "/goal_pose")
        self.cmd_topic = self._text("cmd_vel_topic", "/cmd_vel")
        self.map_frame = self._text("map_frame", "map")
        self.base_frame = self._text("base_frame", "base_link")
        # auto_align 파라미터 서비스를 못 읽을 때의 폴백 (launch 지정값)
        self.fb_sx = self._num("start_x", 1.72)
        self.fb_sy = self._num("start_y", 1.39)
        self.fb_syaw = self._num("start_yaw", 356.0)

        # --- 미션 -----------------------------------------------------------
        self.settle_sec = self._num("settle_sec", 10.0)
        self.align_wait = self._num("align_wait_sec", 20.0)
        # ★ 도면-스캔 자동 정렬이 '실제로 적용'되기 전에는 출발하지 않는다.
        #   auto_align 은 정렬에 성공했을 때만 플래너에 SetParameters 로
        #   start_x/y/yaw 를 넣는다(실패/거부 시엔 넣지 않음). 그 순간 플래너가
        #   /parameter_events 로 변경 이벤트를 내므로, 그것이 곧 '매칭 완료'다.
        #   launch 에서 auto_align:=false 면 이 게이트도 같이 꺼진다.
        self.require_align = self._flag("require_align", True)
        # 정렬 이벤트 후 플래너가 도면 격자를 새 정렬로 다시 만들 시간(1초 타이머)
        self.align_grace = self._num("align_grace_sec", 2.0)
        # 0 = 정렬이 확인될 때까지 무한 대기(요청 사양). >0 이면 그 시간 뒤
        # 경고를 남기고 현재 정렬값으로 그냥 출발한다.
        self.align_timeout = self._num("align_apply_timeout_sec", 0.0)
        self.goal_timeout = self._num("goal_timeout_sec", 180.0)
        # 막힘(우회로 없음)이 이만큼 지속되면 그 방을 포기하고 다음으로.
        # 플래너가 1.5초마다 재시도하고 동적 장애물 TTL 이 3초라 8초면
        # '지나가는 사람' 정도는 기다려서 통과하고, 진짜 막힘은 빨리 포기한다.
        # (기본 20초는 방마다 너무 오래 기다린다는 사용자 피드백으로 단축)
        self.blocked_skip = self._num("blocked_skip_sec", 8.0)
        self.resend_sec = self._num("resend_sec", 8.0)
        self.arrive_extra = self._num("arrive_extra_sec", 1.5)
        self.cancel_settle = self._num("cancel_settle_sec", 1.0)
        # 목적지를 보내기 전에 이미 이 거리 안이면 이동 생략(바로 스캔).
        # goal_tolerance(0.06)보다 약간 크게 — 이보다 가까우면 플래너가
        # 한두 점짜리 경로를 내서 이동해 봤자 의미가 없다.
        self.near_enough = self._num("already_there_dist", 0.10)
        self.retry_skipped = self._flag("retry_skipped", True)
        self.max_retry_rounds = int(self._num("retry_rounds", 1))
        self.return_home = self._flag("return_home", True)
        self.hz = self._num("control_hz", 20.0)
        # 통합 스택에서는 노드를 항상 띄워 두고 웹 명령이 올 때만 미션을
        # 시작한다. 단독 room_explorer.launch.py의 기존 자동 시작은
        # 기본값 true로 그대로 보존한다.
        self.start_enabled = self._flag("start_enabled", True)
        # 통합 스택에서는 person_detour coordinator가 follower enable의
        # 단일 소유자다. 단독 실행일 때만 이 노드가 기존처럼 직접 관리한다.
        self.manage_follower_enable = self._flag(
            "manage_follower_enable", True)

        # --- 스캔 회전 (path_follower 의 spin/stall 파라미터와 같은 계열) ----
        self.spin_rad = math.radians(self._num("spin_deg", 360.0))
        self.spin_speed = self._num("spin_speed", 2.0)   # keyboard 'a' 와 동일
        self.spin_timeout = self._num("spin_timeout_sec", 60.0)
        # ★ 회전 중 이만큼 돌 때마다 잠깐 완전히 멈춘다.
        #   실측(2026-08-05 주행 로그): 출발점에서 360도를 쉼 없이 돌자
        #   cartographer 가 빗살 한 칸(0.5 m) 옆으로 미끄러져, 이후 모든
        #   목적지가 물리적으로 한 칸 옆에 떨어졌다. 이 미로는 같은 모양이
        #   반복돼서 회전 중 스캔매칭이 옆 칸에 걸리기 쉽다. 멈춘 동안
        #   정지 스캔으로 위치를 다시 고정하게 한다.
        self.spin_pause_deg = math.radians(self._num("spin_pause_every_deg",
                                                     120.0))
        self.spin_pause_sec = self._num("spin_pause_sec", 1.5)
        # 회전이 끝난 뒤 다음 목적지 전에 기다리는 시간. cartographer 가
        # 정지 스캔으로 자리를 다시 잡고, auto_align 추적(refine)이 어긋난
        # 정렬을 당겨올 시간이다.
        self.post_spin_settle = self._num("post_spin_settle_sec", 5.0)
        # ★ 한 틱에 이 각도 이상 yaw 가 튀면 실제 회전이 아니라 SLAM 의
        #   위치 스냅(재정렬 점프)이다 — 회전량 집계에서 뺀다.
        #   실제 회전은 PWM 255 에서 ~120도/초 = 틱(50ms)당 6도, TF 가
        #   묶여 와도 12도 정도다. 반면 이 미로는 90/180도 대칭이라
        #   회전 중 yaw 가 90도 단위로 스냅하는데, 그걸 집계에 넣으면
        #   누적각이 깎여 로봇이 '2바퀴' 도는 원인이 된다.
        self.spin_jump_rad = math.radians(self._num("spin_jump_deg", 30.0))
        # ★★ 회전각의 1차 측정은 TF 가 아니라 IMU 자이로 적분이다.
        #   실주행(2026-08-05): 점프 필터를 넣어도 물리 2바퀴가 남았다 —
        #   방 #12 실회전 7.4초에 TF 누적 362도 = 실제 ~740도. 대칭 미로라
        #   cartographer 가 회전을 30도 미만의 작은 단위로 계속 되돌려
        #   TF 로는 절반만 세어진다. 자이로는 SLAM 과 무관하게 물리 회전을
        #   그대로 적분한다(imu_fix 가 바이어스 보정, 20초에 오차 ~3도).
        #   IMU 메시지가 안 들어오면(배선 불량 등) TF 방식으로 폴백.
        self.use_imu_spin = self._flag("use_imu_spin", True)
        self.imu_topic = self._text("imu_topic", "/imu")
        self.stall_sec = self._num("spin_stall_sec", 2.0)
        self.stall_rad = math.radians(self._num("spin_stall_deg", 3.0))
        self.nudge_sec = self._num("spin_nudge_sec", 0.35)
        # keyboard 's'(2.0)와 같은 스케일 = PWM 255. 추종기의 -0.75 는
        # min_wheel_cmd 정규화를 거치지만 이 노드는 raw 로 발행하므로
        # 그대로 두면 PWM 191 이 되어 정지마찰을 못 넘는다.
        self.nudge_lin = self._num("spin_nudge_lin", -2.0)

        # --- 상태 -----------------------------------------------------------
        self.enabled = self.start_enabled
        self.mode = "WAIT" if self.enabled else "STANDBY"
        self.boot_t = None            # 첫 tick 에서 채움
        self.fixed_order = False      # room_points 로 순서가 못 박혔는가
        self.pending = []             # 갈 방 (순서대로)
        self.visited = []
        self.skipped = []
        self.cur = None               # 지금 가는 방 dict / None=출발자리 복귀
        self.retry_round = 0
        self.going_home = False
        self.home_drawing = None      # 출발 자리 (도면 좌표)
        self.align = None             # (sx, sy, yaw_deg)
        self.align_from_fallback = False
        self.align_warned = False
        self.align_applied = False    # 자동 정렬이 적용된 것을 확인했는가
        self.align_applied_t = None
        self.align_guide_logged = 0.0
        self.wait_logged = 0.0
        # 목적지 하나에 대한 추적
        self.goal_m = None
        self.t_goal = 0.0
        self.await_reach = False
        self.reach_false_seen = False
        self.arrived = False
        self.block_since = None
        self.blocked_logged = False
        self.nopath_since = None
        self.nopath_logged = False
        self.detour_logged = False
        self.last_resend = 0.0
        # 스캔 회전
        self.spin_wait_until = 0.0
        self.spin_t0 = 0.0
        self.spin_prev = None
        self.spin_accum = 0.0          # TF 기반 누적(폴백)
        self.imu_accum = 0.0           # IMU 자이로 적분(1차 측정)
        self.imu_prev_t = None
        self.imu_last_rx = 0.0
        self.imu_wz = 0.0
        self.spin_pause_mark = 0.0     # 마지막 재고정 멈춤 시점의 회전각
        self.spin_pause_until = 0.0    # 회전 중 재고정 멈춤
        self.stall_ref = None
        self.stall_t = 0.0
        self.nudge_until = 0.0
        self.nudge_sign = 1.0
        self.flush_until = 0.0
        self.pause_until = 0.0
        self.pause_cap = 0.0           # 정착 대기 연장 상한
        self.big_align_t = None        # 마지막 '큰 정렬 갱신' 시각
        # 다른 노드 상태
        self.planner_status = None
        self.planner_status_t = 0.0

        # --- 도면 읽기 + 방 나누기 ------------------------------------------
        self.get_logger().info("도면을 읽고 방을 나눕니다: %s" % self.ref_yaml)
        self.occ, self.unk, self.res = rs.load_drawing(self.ref_yaml)
        self.rooms, self.label, self.dist = rs.segment_rooms(
            self.occ, self.unk, self.res, h_door=self.h_door,
            min_area=self.min_area, min_clear=self.min_clear,
            border_margin=self.border)
        # 사용자가 '방 아님'으로 지정한 구역 제외 (번호는 그대로 유지)
        self.excluded_rooms = []
        if self.room_pts:
            # ★ 좌표로 직접 지정 — 자동 분할 결과는 참고용으로만 두고,
            #   이 점들만 지정한 순서대로 방문한다.
            self.rooms = self.rooms_from_points()
            self.fixed_order = True
            if self.exclude_ids or self.include_ids:
                self.get_logger().warn(
                    "room_points 가 지정됐으므로 exclude_rooms/include_rooms "
                    "번호 지정은 무시합니다 (좌표 지정이 우선).")
        elif self.include_ids or self.exclude_ids:
            active = []
            for rm in self.rooms:
                drop = (self.include_ids and rm["id"] not in self.include_ids) \
                    or (rm["id"] in self.exclude_ids)
                (self.excluded_rooms if drop else active).append(rm)
            self.rooms = active
            if self.excluded_rooms:
                self.get_logger().info(
                    "방 아님으로 제외(%d개): %s"
                    % (len(self.excluded_rooms),
                       ", ".join("#%d" % rm["id"]
                                 for rm in self.excluded_rooms)))
        h, w = self.occ.shape
        self.get_logger().info(
            "도면 %.2f x %.2f m (%dx%d @ %.3f m) -> 탐사할 방 %d개 (%s)"
            % (w * self.res, h * self.res, w, h, self.res, len(self.rooms),
               "좌표 지정 room_points — 이 순서대로 방문" if self.fixed_order
               else "자동 분할: h_door %.2f m, 최소넓이 %.2f m^2, 최소여유 %.2f m"
               % (self.h_door, self.min_area, self.min_clear)))
        for rm in self.rooms:
            self.get_logger().info(
                "  방 #%-2d 정중앙 (%.2f, %.2f)  넓이 %.2f m^2  벽여유 %.2f m"
                % (rm["id"], rm["cx"], rm["cy"], rm["area"], rm["peak"]))
        for line in rs.ascii_map(self.occ, self.label, self.rooms):
            self.get_logger().info("  %s" % line)
        if not self.rooms:
            self.get_logger().error(
                "탐사할 방이 없습니다 — room_points 를 확인하거나(좌표 지정), "
                "자동 분할이면 h_door(%.2f)/min_room_area/min_room_clear 를 "
                "낮춰 보세요. room_check.py 로 미리 확인할 수 있습니다."
                % self.h_door)

        # 순서 계산용 축소 통행격자
        self.safe_s, self.factor = rs.coarse_safe(
            self.dist, self.res, self.inflate, self.order_cell)

        # --- 통신 -----------------------------------------------------------
        self.goal_pub = self.create_publisher(PoseStamped, self.goal_topic, 10)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.cancel_pub = self.create_publisher(Empty, "/auto_drive/cancel", 10)
        self.enable_pub = self.create_publisher(Bool, "/auto_drive/enable", 10)
        self.spin_active_pub = self.create_publisher(
            Bool, "/room_explorer/spin_active", latched())
        self.status_pub = self.create_publisher(
            String, "/room_explorer/status", latched())
        self.marker_pub = self.create_publisher(
            MarkerArray, "/room_explorer/markers", latched())

        self.create_subscription(Bool, "/auto_drive/goal_reached",
                                 self.on_reach, latched())
        self.create_subscription(String, "/planner/status",
                                 self.on_planner_status, latched())
        self.create_subscription(Bool, "/room_explorer/enable",
                                 self.on_mission_enable, 10)
        # auto_align 이 플래너에 정렬값을 넣는 순간을 잡는다 (게이트 해제 신호)
        self.create_subscription(ParameterEvent, "/parameter_events",
                                 self.on_param_event, 10)
        if self.use_imu_spin:
            from rclpy.qos import qos_profile_sensor_data
            self.create_subscription(Imu, self.imu_topic, self.on_imu,
                                     qos_profile_sensor_data)

        self.param_cli = self.create_client(
            GetParameters, "/%s/get_parameters" % self.planner_node)
        self._align_pending = False

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_timer(1.0 / max(self.hz, 1.0), self.tick)
        self.create_timer(2.0, self.poll_align)
        self.create_timer(1.0, self.publish_status)
        self.create_timer(2.0, self.publish_markers)

        self.spin_active_pub.publish(Bool(data=False))
        if self.enabled:
            self.get_logger().info(
                "방 탐사 준비 완료 — SLAM/정렬/플래너가 뜨기를 기다립니다 "
                "(최소 %.0f초 안정화)" % self.settle_sec)
        else:
            self.get_logger().info(
                "방 탐사 노드 대기 중 — /room_explorer/enable=true에서 "
                "현재 센서/SLAM 스택을 그대로 사용해 탐사를 시작합니다.")

    def _set_follower_enabled(self, enabled):
        if self.manage_follower_enable:
            self.enable_pub.publish(Bool(data=bool(enabled)))

    def _reset_mission(self):
        """센서/정렬 상태는 보존하고 방문 진행 상태만 새 사이클로 만든다."""
        self.pending = []
        self.visited = []
        self.skipped = []
        self.cur = None
        self.retry_round = 0
        self.going_home = False
        self.home_drawing = None
        self.goal_m = None
        self.t_goal = 0.0
        self.await_reach = False
        self.reach_false_seen = False
        self.arrived = False
        self.block_since = None
        self.blocked_logged = False
        self.nopath_since = None
        self.nopath_logged = False
        self.detour_logged = False
        self.last_resend = 0.0
        self.spin_wait_until = 0.0
        self.spin_t0 = 0.0
        self.spin_prev = None
        self.spin_accum = 0.0
        self.imu_accum = 0.0
        self.imu_prev_t = None
        self.spin_pause_mark = 0.0
        self.spin_pause_until = 0.0
        self.stall_ref = None
        self.stall_t = 0.0
        self.nudge_until = 0.0
        self.flush_until = 0.0
        self.pause_until = 0.0
        self.pause_cap = 0.0
        self.big_align_t = None
        self.wait_logged = 0.0

    def on_mission_enable(self, message):
        requested = bool(message.data)
        if not requested:
            if self.enabled:
                self.cmd_pub.publish(Twist())
                self._set_follower_enabled(False)
                self.get_logger().info("전체 탐색 미션을 중지하고 대기합니다.")
            self.enabled = False
            self.mode = "STANDBY"
            self.spin_active_pub.publish(Bool(data=False))
            self.publish_status()
            return

        if self.enabled and self.mode != "DONE":
            return
        self._reset_mission()
        self.enabled = True
        self.mode = "WAIT"
        # 센서/SLAM은 대기 중에도 계속 돌았으므로 별도 10초를 다시 채우지
        # 않는다. 정렬/TF/플래너 준비 조건 자체는 tick_wait가 계속 확인한다.
        # 웹이 직전 단일 목표를 cancel한 메시지가 coordinator에 먼저
        # 반영되도록 짧은 제어권 인계 여유를 둔다.
        self.boot_t = self.now() - self.settle_sec + 0.25
        self.get_logger().info("웹 요청으로 전체 탐색 미션을 시작합니다.")
        self.publish_status()

    def rooms_from_points(self):
        """room_points 로 지정한 좌표를 방 목록으로 만든다 (순서 그대로).

        각 점은 반경 room_point_refine 안에서 '벽에서 가장 먼 자리'로 옮긴다.
        반경을 작게 둬서 옆 칸으로 새지 않으면서 칸의 중앙에 맞춘다.
        벽/부풀림 안이라 갈 수 없는 점은 경고만 남기고 목록에 그대로 둔다 —
        실제로 못 가면 주행 중 '장애물로 갈 수 없습니다'로 걸러진다.
        """
        rooms = []
        h, w = self.occ.shape
        rad = max(int(self.pt_refine / self.res), 0)
        for i, (x, y) in enumerate(self.room_pts):
            c0, r0 = int(x / self.res), int(y / self.res)
            if not (0 <= r0 < h and 0 <= c0 < w):
                self.get_logger().error(
                    "room_points 의 %d번째 점 (%.2f, %.2f) 이 도면 밖입니다 "
                    "— 건너뜁니다." % (i + 1, x, y))
                continue
            # 주변에서 가장 여유가 큰 자리로 보정. 여유가 같은 자리가 여러
            # 개면(넓은 통로에서는 흔하다) 지정한 점에 가장 가까운 것을 쓴다 —
            # argmax 만 쓰면 스캔 순서상 가장 아래 행으로 끌려가 지정 위치가
            # 수 cm 밀린다(실측 73 mm).
            r1, r2 = max(0, r0 - rad), min(h, r0 + rad + 1)
            c1, c2 = max(0, c0 - rad), min(w, c0 + rad + 1)
            win = self.dist[r1:r2, c1:c2]
            best = float(win.max())
            rows, cols = np.nonzero(win >= best - 1e-9)
            d2 = ((rows + r1 - r0) ** 2 + (cols + c1 - c0) ** 2)
            k = int(np.argmin(d2))
            rr, cc = int(rows[k]) + r1, int(cols[k]) + c1
            cx, cy = (cc + 0.5) * self.res, (rr + 0.5) * self.res
            peak = float(self.dist[rr, cc])
            lb = int(self.label[rr, cc])
            area = (float((self.label == lb).sum()) * self.res * self.res
                    if lb else 0.0)
            if peak < self.inflate:
                self.get_logger().warn(
                    "room_points 의 %d번째 점 (%.2f, %.2f) 은 벽 여유가 "
                    "%.3f m 뿐입니다 (차체 부풀림 %.3f m) — 갈 수 없을 수 "
                    "있습니다." % (i + 1, x, y, peak, self.inflate))
            moved = math.hypot(cx - x, cy - y)
            rooms.append(dict(raw=lb, id=i + 1, cx=cx, cy=cy,
                              area=area, peak=peak))
            if moved > 0.005:
                self.get_logger().info(
                    "  방 #%d: 지정 (%.2f, %.2f) -> 정중앙 보정 (%.2f, %.2f) "
                    "(%.0f mm 이동, 벽여유 %.2f m)"
                    % (i + 1, x, y, cx, cy, moved * 1000, peak))
        return rooms

    # ================= 시각/좌표 유틸 =================
    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def robot_pose_map(self):
        """map 프레임의 (x, y, yaw). 못 읽으면 None."""
        try:
            tr = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time())
        except tf2_ros.TransformException:
            return None
        t = tr.transform.translation
        return (float(t.x), float(t.y), yaw_from_quat(tr.transform.rotation))

    def d2m(self, xd, yd):
        """도면 좌표 -> map 좌표. Pm = R(-yaw) * (Pd - S).

        goal_path_planner_node.build_from_drawing 과 같은 규약이다.
        정렬값(start_x/y/yaw)은 auto_align 이 플래너에 넣어 준 최신값을 쓴다.
        """
        sx, sy, yaw_deg = self.align
        yaw = math.radians(yaw_deg)
        c, s = math.cos(-yaw), math.sin(-yaw)
        dx, dy = xd - sx, yd - sy
        return c * dx - s * dy, s * dx + c * dy

    def m2d(self, xm, ym):
        """map 좌표 -> 도면 좌표. Pd = R(yaw) * Pm + S."""
        sx, sy, yaw_deg = self.align
        yaw = math.radians(yaw_deg)
        c, s = math.cos(yaw), math.sin(yaw)
        return c * xm - s * ym + sx, s * xm + c * ym + sy

    def drawing_cell(self, xd, yd):
        """도면 좌표 -> 축소 격자 셀."""
        return (int(yd / self.res) // self.factor,
                int(xd / self.res) // self.factor)

    # ================= 콜백 =================
    def on_reach(self, msg: Bool):
        if not self.await_reach:
            return
        if msg.data:
            # False(새 경로 수신)를 본 뒤의 True 만 진짜 도착이다.
            # latched 라 이전 목적지의 True 가 남아 있을 수 있기 때문이다.
            if self.reach_false_seen:
                self.arrived = True
        else:
            self.reach_false_seen = True

    def on_planner_status(self, msg: String):
        try:
            self.planner_status = json.loads(msg.data)
            self.planner_status_t = self.now()
        except (ValueError, TypeError):
            pass

    def on_imu(self, msg: Imu):
        """스캔 회전 중 자이로 z 를 적분해 물리 회전각을 잰다."""
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.imu_wz = float(msg.angular_velocity.z)
        self.imu_last_rx = self.now()
        if self.mode == "SPIN" and self.imu_prev_t is not None:
            dt = t - self.imu_prev_t
            if 0.0 < dt < 0.1:          # 끊겼다 온 표본은 적분에서 제외
                self.imu_accum += self.imu_wz * dt
        self.imu_prev_t = t

    def on_param_event(self, msg: ParameterEvent):
        """플래너의 start_x/y/yaw 가 '변경'되는 순간 = 자동 정렬 적용/갱신.

        launch 가 시작할 때 넣는 초기값은 new_parameters(선언) 이벤트라서
        여기 걸리지 않는다. changed_parameters 로 바뀌는 것은 auto_align 의
        SetParameters(또는 사람이 ros2 param set 으로 넣는 수동 정렬)뿐이다.

        ★ 파라미터 서비스는 값 하나마다 이벤트를 따로 낼 수 있다
        (실측: start_x/y/yaw 가 3개의 이벤트로 나뉘어 옴). 그래서 첫
        이벤트에서 잠그지 말고 매번 바뀐 값만 병합해야 한다 — 예전에는 첫
        이벤트(start_x)만 반영하고 y/yaw 는 폴링(2초)까지 옛값이었다.
        refine(정렬 추적) 모드의 갱신도 이 경로로 즉시 반영된다.
        """
        if msg.node.lstrip("/") != self.planner_node:
            return
        changed = {}
        for p in msg.changed_parameters:
            if p.name in ("start_x", "start_y", "start_yaw") \
                    and p.value.type == 3:          # PARAMETER_DOUBLE
                changed[p.name] = float(p.value.double_value)
        if not changed:
            return
        base = self.align if self.align is not None \
            else (self.fb_sx, self.fb_sy, self.fb_syaw)
        new = (changed.get("start_x", base[0]),
               changed.get("start_y", base[1]),
               changed.get("start_yaw", base[2]))
        dx = abs(new[0] - base[0])
        dy = abs(new[1] - base[1])
        dyaw = abs((new[2] - base[2] + 180.0) % 360.0 - 180.0)
        self.align = new
        self.align_from_fallback = False
        now = self.now()
        if not self.align_applied:
            self.align_applied = True
            self.align_applied_t = now
            self.get_logger().info(
                "★ 도면-라이다 자동 정렬 매칭 확인 — %.0f초 반영 대기 후 "
                "출발합니다. (적용값은 '도면 정렬값 갱신' 로그로 확정)"
                % self.align_grace)
            return
        # 연속 이벤트(값 하나씩 3개)로 grace 가 끊기지 않게 시각만 갱신
        if self.align_applied_t is not None \
                and now - self.align_applied_t < self.align_grace:
            self.align_applied_t = now
        # ★ 회전 등으로 SLAM 위치가 미끄러진 것을 auto_align 추적/전역
        #   재정렬이 크게 당겨온 경우 — 주행 중이면 목적지를 새 정렬로
        #   다시 보내고, 정착 대기 중이면 대기를 조금 늘린다.
        if dx > 0.04 or dy > 0.04 or dyaw > 3.0:
            self.big_align_t = now
            self.get_logger().warn(
                "정렬이 크게 갱신되었습니다 (Δx %.2f, Δy %.2f, Δyaw %.1f도) "
                "— 새 정렬 기준으로 계속합니다." % (dx, dy, dyaw))
            if self.mode == "DRIVE":
                self.send_goal(now, log=False)
                self.get_logger().warn(
                    "%s 목적지를 새 정렬로 다시 보냈습니다."
                    % self.target_name())

    def poll_align(self):
        """플래너의 start_x/y/yaw 를 주기적으로 읽는다 (auto_align 최신값)."""
        if self._align_pending or not self.param_cli.service_is_ready():
            return
        req = GetParameters.Request()
        req.names = ["start_x", "start_y", "start_yaw"]
        self._align_pending = True
        fut = self.param_cli.call_async(req)
        fut.add_done_callback(self._align_done)

    def _align_done(self, fut):
        self._align_pending = False
        try:
            res = fut.result()
            vals = [v.double_value for v in res.values]
            if len(vals) != 3:
                return
        except Exception:                                # noqa: BLE001
            return
        new = (vals[0], vals[1], vals[2])
        old = self.align
        self.align = new
        self.align_from_fallback = False
        if old is None or any(abs(a - b) > 0.01 for a, b in zip(old, new)):
            self.get_logger().info(
                "도면 정렬값 갱신: start_x=%.2f start_y=%.2f start_yaw=%.1f도"
                % new)
        # 이 노드가 정렬 적용 '이후'에 켜졌으면 변경 이벤트를 놓쳤을 수 있다.
        # 그때는 값 자체가 launch 지정값과 달라져 있는 것으로 판별한다.
        if not self.align_applied and (
                abs(new[0] - self.fb_sx) > 0.005
                or abs(new[1] - self.fb_sy) > 0.005
                or abs((new[2] - self.fb_syaw + 180.0) % 360.0 - 180.0) > 0.1):
            self.align_applied = True
            self.align_applied_t = self.now()
            self.get_logger().info(
                "플래너의 정렬값이 launch 지정값과 다릅니다 — 자동 정렬이 "
                "이미 적용된 것으로 보고 출발 준비를 계속합니다.")

    # ================= 메인 상태기계 =================
    def tick(self):
        now = self.now()
        spin_active = self.enabled and self.mode in {
            "SPIN_WAIT", "SPIN", "FLUSH"
        }
        # 통합 cmd_vel mux의 timeout보다 빠른 heartbeat로 회전 제어권을
        # 유지한다. STANDBY/DRIVE/PAUSE에서는 일반 follower가 소유한다.
        self.spin_active_pub.publish(Bool(data=spin_active))
        if not self.enabled:
            return
        if self.boot_t is None:
            self.boot_t = now
        if self.mode == "WAIT":
            self.tick_wait(now)
        elif self.mode == "SEND":
            self.tick_send(now)
        elif self.mode == "DRIVE":
            self.tick_drive(now)
        elif self.mode == "SPIN_WAIT":
            if now >= self.spin_wait_until:
                self.start_spin(now)
        elif self.mode == "SPIN":
            self.tick_spin(now)
        elif self.mode == "FLUSH":
            self.tick_flush(now)
        elif self.mode == "PAUSE":
            # 정착 중 큰 정렬 보정이 오면 반영될 시간을 조금 더 준다
            if self.big_align_t is not None:
                self.pause_until = min(max(self.pause_until,
                                           self.big_align_t + 2.5),
                                       self.pause_cap)
            if now >= self.pause_until:
                self.big_align_t = None
                self.advance()
        # DONE 은 할 일 없음

    def tick_wait(self, now):
        if not self.rooms:
            self.get_logger().error("탐사할 방이 없어 종료합니다.")
            self.mode = "DONE"
            return
        missing = []
        if self.planner_status is None:
            missing.append("플래너 상태(/planner/status)")
        elif self.planner_status.get("state") == "WAITING_FOR_GRID":
            missing.append("계획 격자(SLAM 맵)")
        if self.align is None:
            if now - self.boot_t > self.align_wait:
                self.align = (self.fb_sx, self.fb_sy, self.fb_syaw)
                self.align_from_fallback = True
                if not self.align_warned:
                    self.align_warned = True
                    self.get_logger().warn(
                        "%.0f초 동안 %s 의 파라미터를 못 읽었습니다 — launch 의 "
                        "start_x/y/yaw (%.2f, %.2f, %.1f도) 를 그대로 씁니다."
                        % (self.align_wait, self.planner_node,
                           self.fb_sx, self.fb_sy, self.fb_syaw))
            else:
                missing.append("도면 정렬값(%s)" % self.planner_node)
        pose = self.robot_pose_map()
        if pose is None:
            missing.append("현재 위치(TF %s->%s)"
                           % (self.map_frame, self.base_frame))
        if now - self.boot_t < self.settle_sec:
            missing.append("안정화 %.0f/%.0f초"
                           % (now - self.boot_t, self.settle_sec))
        # ★ 자동 정렬 게이트 — 매칭이 확인되기 전에는 출발하지 않는다.
        if self.require_align:
            if not self.align_applied:
                if self.align_timeout > 0 \
                        and now - self.boot_t > self.align_timeout:
                    self.align_applied = True
                    self.align_applied_t = now
                    self.get_logger().warn(
                        "%.0f초 안에 자동 정렬 적용을 확인하지 못했습니다 — "
                        "현재 정렬값으로 그냥 출발합니다 "
                        "(align_apply_timeout_sec 로 지정된 동작)."
                        % self.align_timeout)
                else:
                    missing.append("도면-라이다 자동 정렬 매칭")
                    if now - self.align_guide_logged > 15.0 \
                            and now - self.boot_t > 20.0:
                        self.align_guide_logged = now
                        self.get_logger().warn(
                            "자동 정렬 적용을 계속 기다리는 중입니다. "
                            "auto_align 로그에 '정렬 측정이 서로 안 맞습니다' "
                            "또는 '정렬을 못 찾았습니다' 가 떴다면 이 상태로는 "
                            "출발하지 않습니다 — 로봇을 정해진 자리에 두고 "
                            "다시 실행하거나, require_align:=false 또는 "
                            "align_apply_timeout_sec:=60 으로 게이트를 "
                            "풀 수 있습니다.")
            elif self.align_applied_t is not None \
                    and now - self.align_applied_t < self.align_grace:
                missing.append("정렬 반영 %.0f/%.0f초"
                               % (now - self.align_applied_t,
                                  self.align_grace))
        if missing:
            if now - self.wait_logged > 5.0:
                self.wait_logged = now
                self.get_logger().info("대기 중: " + ", ".join(missing))
            return
        self.build_itinerary(pose)

    def build_itinerary(self, pose):
        """방문 순서를 정한다.

        room_points 로 좌표를 지정했으면 **그 순서 그대로** 간다(재배치 없음).
        지정이 없으면 도달 가능한 방만 남기고 실제 이동거리가 가까운 순서로
        (탐욕 최근접) 정렬한다.
        """
        xd, yd = self.m2d(pose[0], pose[1])
        self.home_drawing = (xd, yd)
        if self.fixed_order:
            self.pending = list(self.rooms)
            start = rs.snap_cell(self.safe_s, self.drawing_cell(xd, yd))
            if start is not None:
                reach = rs.bfs_dist(self.safe_s, start)
                for rm in self.rooms:
                    cell = rs.snap_cell(self.safe_s,
                                        self.drawing_cell(rm["cx"], rm["cy"]),
                                        max_r=3)
                    rm["cell"] = cell
                    if cell is None or reach[cell] < 0:
                        self.get_logger().warn(
                            "방 #%d (%.2f, %.2f) 는 도면상 로봇 위치에서 "
                            "이어져 있지 않습니다 — 순서는 그대로 두고 "
                            "실제로 가 봅니다(막히면 건너뜁니다)."
                            % (rm["id"], rm["cx"], rm["cy"]))
            self.get_logger().info(
                "★ 탐사 시작 — 지정한 방 %d개를 지정 순서대로: %s"
                % (len(self.pending),
                   " -> ".join("#%d(%.2f,%.2f)" % (rm["id"], rm["cx"], rm["cy"])
                               for rm in self.pending)))
            self.publish_markers()
            self.advance()
            return
        start = rs.snap_cell(self.safe_s, self.drawing_cell(xd, yd))
        if start is None:
            self.get_logger().error(
                "로봇의 도면상 위치 (%.2f, %.2f) 주변에 통행 가능한 곳이 "
                "없습니다 — 정렬이 크게 어긋났을 수 있습니다. 그래도 "
                "직선거리 순서로 진행합니다." % (xd, yd))
            order = sorted(self.rooms, key=lambda rm: math.hypot(
                rm["cx"] - xd, rm["cy"] - yd))
            self.pending = list(order)
        else:
            reach = rs.bfs_dist(self.safe_s, start)
            usable, dropped = [], []
            for rm in self.rooms:
                cell = rs.snap_cell(self.safe_s,
                                    self.drawing_cell(rm["cx"], rm["cy"]),
                                    max_r=3)
                if cell is None or reach[cell] < 0:
                    dropped.append(rm)
                else:
                    rm["cell"] = cell
                    usable.append(rm)
            for rm in dropped:
                self.get_logger().warn(
                    "방 #%d (%.2f, %.2f) 는 로봇 위치에서 이어져 있지 않아 "
                    "제외합니다 (도면 밖 공간이거나 통로가 차체보다 좁음)."
                    % (rm["id"], rm["cx"], rm["cy"]))
            # 탐욕 최근접: 지금 자리에서 실제 이동거리가 가장 짧은 방부터
            order = []
            cur_cell = start
            remaining = list(usable)
            while remaining:
                d = rs.bfs_dist(self.safe_s, cur_cell)
                remaining.sort(key=lambda rm: (
                    d[rm["cell"]] if d[rm["cell"]] >= 0 else 1 << 30))
                nxt = remaining.pop(0)
                order.append(nxt)
                cur_cell = nxt["cell"]
            self.pending = order
        self.get_logger().info(
            "★ 탐사 시작 — 방 %d개, 순서: %s"
            % (len(self.pending),
               " -> ".join("#%d" % rm["id"] for rm in self.pending)))
        self.publish_markers()
        self.advance()

    # ---------------- 목적지 전송/주행 ----------------
    def target_drawing(self):
        if self.cur is not None:
            return self.cur["cx"], self.cur["cy"]
        return self.home_drawing

    def target_name(self):
        if self.cur is not None:
            return "방 #%d" % self.cur["id"]
        return "출발 자리"

    def send_goal(self, now, log=True):
        xd, yd = self.target_drawing()
        mx, my = self.d2m(xd, yd)
        self.goal_m = (mx, my)
        goal = PoseStamped()
        goal.header.frame_id = self.map_frame
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = mx
        goal.pose.position.y = my
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)
        self.last_resend = now
        if log:
            self.get_logger().info(
                "▶ %s 로 이동 — 도면 (%.2f, %.2f) -> map (%.2f, %.2f)"
                % (self.target_name(), xd, yd, mx, my))

    def tick_send(self, now):
        self._set_follower_enabled(True)      # 스캔 중 꺼 둔 추종기 재개
        pose = self.robot_pose_map()
        xd, yd = self.target_drawing()
        mx, my = self.d2m(xd, yd)
        if pose is not None and math.hypot(pose[0] - mx,
                                           pose[1] - my) < self.near_enough:
            # 이미 방 정중앙 근처다(출발 위치가 방 안인 경우 등).
            # 플래너가 한 점짜리 경로를 내면 추종기가 도착 신호를 안 주므로
            # 목적지를 보내지 않고 바로 스캔으로 넘어간다.
            if self.cur is None:
                self.get_logger().info("이미 출발 자리에 있습니다.")
                self.finish()
                return
            self.get_logger().info(
                "이미 %s 정중앙 근처에 있습니다 — 바로 스캔합니다."
                % self.target_name())
            self.begin_scan(now)
            return
        self.send_goal(now)
        self.t_goal = now
        self.await_reach = True
        self.reach_false_seen = False
        self.arrived = False
        self.block_since = None
        self.blocked_logged = False
        self.nopath_since = None
        self.nopath_logged = False
        self.detour_logged = False
        self.mode = "DRIVE"

    def begin_scan(self, now):
        self.await_reach = False
        # 추종기를 일시정지시킨다. 도착 직후 플래너가 짧은 경로를 다시 내도
        # (직전 장애물 기억이 남은 경우) 스캔 회전과 겹쳐 움직이지 않게.
        self._set_follower_enabled(False)
        self.spin_wait_until = now + self.arrive_extra
        self.mode = "SPIN_WAIT"

    def tick_drive(self, now):
        if self.arrived:
            if self.cur is None:
                self.get_logger().info("✔ 출발 자리로 돌아왔습니다.")
                self.finish()
                return
            self.get_logger().info(
                "✔ %s 정중앙 도착 — %.1f초 뒤 제자리 회전 스캔을 시작합니다."
                % (self.target_name(), self.arrive_extra))
            self.begin_scan(now)
            return

        ps = self.planner_status
        # 목적지를 보낸 직후의 상태는 이전 목적지 것일 수 있다 — 1.5초 지나고
        # 상태의 goal 이 우리 목적지와 일치할 때만 판정한다.
        if ps is not None and self.planner_status_t > self.t_goal + 1.5:
            st = ps.get("state", "")
            g = ps.get("goal")
            goal_match = (
                g is not None and self.goal_m is not None
                and math.hypot(g.get("x", 1e9) - self.goal_m[0],
                               g.get("y", 1e9) - self.goal_m[1]) < 0.15)
            if not goal_match:
                if now - self.last_resend > self.resend_sec:
                    self.get_logger().warn(
                        "플래너가 아직 이 목적지를 안 갖고 있습니다 — "
                        "%s 목적지를 다시 보냅니다." % self.target_name())
                    self.send_goal(now, log=False)
            elif st == "HALTED_NO_ROUTE":
                if self.block_since is None:
                    self.block_since = now
                if not self.blocked_logged:
                    self.blocked_logged = True
                    self.get_logger().error(
                        "★ 장애물로 인해서 %s (도면 %.2f, %.2f) 에 갈 수 "
                        "없습니다 — 우회로가 없어 정지했습니다. %.0f초 안에 "
                        "길이 안 열리면 건너뛰고 다음 방을 탐색합니다."
                        % ((self.target_name(),) + self.target_drawing()
                           + (self.blocked_skip,)))
                if now - self.block_since > self.blocked_skip:
                    self.skip_current("장애물로 막혀 우회로가 없음")
                    return
            elif st == "NO_PATH":
                if self.nopath_since is None:
                    self.nopath_since = now
                if not self.nopath_logged:
                    self.nopath_logged = True
                    self.get_logger().error(
                        "★ %s (도면 %.2f, %.2f) 까지 경로를 만들 수 없습니다 "
                        "(장애물 또는 차체보다 좁은 통로). %.0f초 더 시도한 뒤 "
                        "다음 방으로 넘어갑니다."
                        % ((self.target_name(),) + self.target_drawing()
                           + (self.blocked_skip,)))
                if now - self.last_resend > self.resend_sec:
                    self.send_goal(now, log=False)   # 정렬이 바뀌었을 수 있다
                if now - self.nopath_since > self.blocked_skip:
                    self.skip_current("경로 없음")
                    return
            else:
                # 정상 주행/우회 중 — 막힘 타이머 리셋
                self.block_since = None
                self.blocked_logged = False
                self.nopath_since = None
                if st == "DETOUR_ACTIVE" and not self.detour_logged:
                    self.detour_logged = True
                    self.get_logger().warn(
                        "장애물 감지 — 우회 경로로 %s 이동을 계속합니다."
                        % self.target_name())

        if now - self.t_goal > self.goal_timeout:
            self.skip_current("시간 초과 (%.0f초)" % self.goal_timeout)

    def skip_current(self, reason):
        self.await_reach = False
        self.cancel_pub.publish(Empty())
        if self.cur is not None:
            self.get_logger().error(
                "✖ %s 포기 (%s) — 이어서 다른 방들을 탐색합니다."
                % (self.target_name(), reason))
            self.skipped.append(self.cur)
        else:
            self.get_logger().warn(
                "출발 자리 복귀를 포기합니다 (%s)." % reason)
            self.finish()
            return
        self.pause_until = self.now() + self.cancel_settle
        self.pause_cap = self.pause_until + 10.0
        self.mode = "PAUSE"
        self.publish_markers()

    # ---------------- 스캔 회전 ----------------
    def start_spin(self, now):
        self.spin_t0 = now
        self.spin_prev = None
        self.spin_accum = 0.0
        self.imu_accum = 0.0
        self.imu_prev_t = None
        self.spin_pause_mark = 0.0
        self.spin_pause_until = 0.0
        self.stall_ref = None
        self.stall_t = now
        self.nudge_until = 0.0
        self.nudge_sign = 1.0
        self.mode = "SPIN"
        imu_ok = self.use_imu_spin and (now - self.imu_last_rx) < 0.5
        self.get_logger().info(
            "%s 정중앙에서 주변 스캔 — %.0f도 제자리 회전, 회전각 측정: %s "
            "(%.0f도마다 %.1f초 멈춰 SLAM 위치 재고정)."
            % (self.target_name(), math.degrees(self.spin_rad),
               "IMU 자이로" if imu_ok else "TF(자이로 폴백 없음)",
               math.degrees(self.spin_pause_deg), self.spin_pause_sec))

    def tick_spin(self, now):
        pose = self.robot_pose_map()
        in_pause = now < self.spin_pause_until
        # 물리 회전각: IMU 자이로 적분(on_imu 가 채움)이 1차, TF 누적이 폴백
        imu_ok = self.use_imu_spin and (now - self.imu_last_rx) < 0.3
        if pose is not None:
            yaw = pose[2]
            if self.spin_prev is None:
                self.spin_prev = yaw
                self.stall_ref = yaw
                self.stall_t = now
            else:
                d = norm_angle(yaw - self.spin_prev)
                self.spin_prev = yaw
                if abs(d) > self.spin_jump_rad:
                    self.get_logger().warn(
                        "회전 중 SLAM yaw 점프 %.0f도 감지 — 실제 회전이 "
                        "아니므로 회전량 집계에서 뺍니다."
                        % math.degrees(d), throttle_duration_sec=2.0)
                else:
                    self.spin_accum += d
        total = abs(self.imu_accum) if imu_ok else abs(self.spin_accum)
        # ★ 일정 각도마다 완전히 멈춰 SLAM 이 정지 스캔으로 자리를 다시
        #   잡게 한다 (연속 회전은 이 미로에서 위치가 옆 칸으로 미끄러짐).
        if not in_pause and self.spin_pause_deg > 0 \
                and total - self.spin_pause_mark >= self.spin_pause_deg \
                and total < self.spin_rad:
            self.spin_pause_until = now + self.spin_pause_sec
            self.spin_pause_mark = total
            self.stall_t = now          # 멈춤을 고착으로 오인하지 않게
            in_pause = True
        # 정지마찰 고착 감지 — path_follower 의 spin_stall 과 같은 방식.
        # IMU 가 있으면 자이로가 곧 '실제로 돌고 있는가'다. 없으면 TF 로.
        if in_pause:
            self.stall_t = now          # 의도된 멈춤 — 고착 타이머 정지
            if pose is not None:
                self.stall_ref = self.spin_prev
        elif imu_ok:
            if abs(self.imu_wz) > math.radians(6.0):
                self.stall_t = now
            elif now - self.stall_t > self.stall_sec \
                    and now >= self.nudge_until:
                self.trigger_nudge(now)
        elif pose is not None:
            yaw = self.spin_prev
            if self.stall_ref is None \
                    or abs(norm_angle(yaw - self.stall_ref)) > self.stall_rad:
                self.stall_ref = yaw
                self.stall_t = now
            elif now - self.stall_t > self.stall_sec \
                    and now >= self.nudge_until:
                self.trigger_nudge(now)

        done = total >= self.spin_rad
        timeout = now - self.spin_t0 > self.spin_timeout
        if done or timeout:
            src = "IMU" if imu_ok else "TF"
            if done:
                self.get_logger().info(
                    "✔ %s 스캔 완료 — %.0f도 회전 (%s 기준, %.1f초)."
                    % (self.target_name(), math.degrees(total), src,
                       now - self.spin_t0))
            else:
                self.get_logger().warn(
                    "스캔 회전이 %.0f초 안에 못 끝났습니다 (%s 기준 %.0f도) "
                    "— 이 정도로 마치고 다음으로 넘어갑니다."
                    % (self.spin_timeout, src, math.degrees(total)))
            self.flush_until = now + 0.4
            self.mode = "FLUSH"
            return
        tw = Twist()
        if in_pause:
            pass                        # 재고정 멈춤 — 0 명령
        elif now < self.nudge_until:
            tw.linear.x = self.nudge_lin * self.nudge_sign
        else:
            tw.angular.z = self.spin_speed
        self.cmd_pub.publish(tw)

    def trigger_nudge(self, now):
        self.nudge_sign *= -1.0
        self.nudge_until = now + self.nudge_sec
        self.stall_t = now
        self.get_logger().warn(
            "회전이 %.0f초째 멈춰 있습니다 — 살짝 %s 움직여 정지마찰을 풉니다."
            % (self.stall_sec,
               "후진" if self.nudge_lin * self.nudge_sign < 0 else "전진"))

    def tick_flush(self, now):
        """회전을 멈추는 0 명령을 잠깐 보내고, 정착 대기 후 다음 방으로."""
        if now < self.flush_until:
            self.cmd_pub.publish(Twist())
            return
        if self.cur is not None:
            self.visited.append(self.cur)
            self.get_logger().info(
                "%s 탐색 끝 — %.0f초 정착(SLAM/정렬 재고정) 후 방을 나와 "
                "다음 목적지로 이동합니다."
                % (self.target_name(), self.post_spin_settle))
        self.publish_markers()
        # 회전 직후는 위치가 미끄러졌을 수 있는 순간이다. 로봇을 세워 둔 채
        # cartographer 정지 스캔과 auto_align 추적(refine)이 자리를 다시
        # 잡을 시간을 준다. 그 사이 큰 정렬 갱신이 오면 조금 더 기다린다.
        self.pause_until = now + self.post_spin_settle
        self.pause_cap = now + self.post_spin_settle + 10.0
        self.mode = "PAUSE"

    # ---------------- 진행 ----------------
    def advance(self):
        if self.pending:
            self.cur = self.pending.pop(0)
            self.mode = "SEND"
            return
        if (self.retry_skipped and self.skipped
                and self.retry_round < self.max_retry_rounds):
            self.retry_round += 1
            self.get_logger().warn(
                "★ 못 갔던 방 %d개를 다시 시도합니다 (%d번째 재시도): %s"
                % (len(self.skipped), self.retry_round,
                   " -> ".join("#%d" % rm["id"] for rm in self.skipped)))
            self.pending = list(self.skipped)
            self.skipped = []
            self.cur = self.pending.pop(0)
            self.mode = "SEND"
            return
        if self.return_home and not self.going_home:
            self.going_home = True
            self.cur = None
            self.get_logger().info("모든 방 탐색을 마쳤습니다 — 출발 자리로 "
                                   "돌아갑니다.")
            self.mode = "SEND"
            return
        self.finish()

    def finish(self):
        self._set_follower_enabled(True)
        total = len(self.visited) + len(self.skipped)
        self.get_logger().info("=" * 46)
        self.get_logger().info(
            "★ 탐사 종료 — 방 %d개 중 %d개 방문, %d개 실패"
            % (total, len(self.visited), len(self.skipped)))
        for rm in self.visited:
            self.get_logger().info(
                "  ✔ 방 #%-2d (%.2f, %.2f) 스캔 완료"
                % (rm["id"], rm["cx"], rm["cy"]))
        for rm in self.skipped:
            self.get_logger().error(
                "  ✖ 방 #%-2d (%.2f, %.2f) — 장애물로 인해 가지 못함"
                % (rm["id"], rm["cx"], rm["cy"]))
        self.get_logger().info("=" * 46)
        self.mode = "DONE"
        self.publish_markers()
        self.publish_status()

    # ---------------- 밖으로 보이는 상태 ----------------
    def publish_status(self):
        payload = {
            "enabled": self.enabled,
            "state": self.mode,
            "current": (self.cur["id"] if self.cur is not None
                        else ("home" if self.going_home else None)),
            "rooms_total": len(self.rooms),
            "pending": [rm["id"] for rm in self.pending],
            "visited": [rm["id"] for rm in self.visited],
            "skipped": [rm["id"] for rm in self.skipped],
            "retry_round": self.retry_round,
            "align_fallback": self.align_from_fallback,
            "timestamp_ns": self.get_clock().now().nanoseconds,
        }
        self.status_pub.publish(String(data=json.dumps(
            payload, ensure_ascii=False, separators=(",", ":"))))

    def publish_markers(self):
        if self.align is None:
            return
        arr = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        vis = {rm["id"] for rm in self.visited}
        skp = {rm["id"] for rm in self.skipped}
        cur_id = self.cur["id"] if self.cur is not None else None
        # 제외된 구역은 회색 작은 점으로만 표시 (탐사 안 함을 눈으로 확인)
        for rm in getattr(self, "excluded_rooms", []):
            mx, my = self.d2m(rm["cx"], rm["cy"])
            m = Marker()
            m.header.frame_id = self.map_frame
            m.header.stamp = stamp
            m.ns = "rooms_excluded"
            m.id = rm["id"]
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = mx
            m.pose.position.y = my
            m.pose.position.z = 0.02
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.04
            m.color.r = m.color.g = m.color.b = 0.5
            m.color.a = 0.5
            arr.markers.append(m)
        for rm in self.rooms:
            mx, my = self.d2m(rm["cx"], rm["cy"])
            m = Marker()
            m.header.frame_id = self.map_frame
            m.header.stamp = stamp
            m.ns = "rooms"
            m.id = rm["id"]
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = mx
            m.pose.position.y = my
            m.pose.position.z = 0.03
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.07
            m.color.a = 0.9
            if rm["id"] == cur_id:
                m.color.r, m.color.g, m.color.b = 0.1, 0.4, 1.0   # 지금 가는 방
            elif rm["id"] in skp:
                m.color.r, m.color.g, m.color.b = 1.0, 0.1, 0.1   # 실패
            elif rm["id"] in vis:
                m.color.r, m.color.g, m.color.b = 0.1, 0.9, 0.1   # 방문
            else:
                m.color.r, m.color.g, m.color.b = 0.9, 0.9, 0.1   # 대기
            arr.markers.append(m)
            t = Marker()
            t.header.frame_id = self.map_frame
            t.header.stamp = stamp
            t.ns = "room_ids"
            t.id = rm["id"]
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose.position.x = mx
            t.pose.position.y = my
            t.pose.position.z = 0.12
            t.pose.orientation.w = 1.0
            t.scale.z = 0.08
            t.color.r = t.color.g = t.color.b = 1.0
            t.color.a = 1.0
            t.text = "#%d" % rm["id"]
            arr.markers.append(t)
        # 앞으로 갈 순서를 잇는 선 (지금 위치 -> 지금 가는 방 -> 남은 방들)
        route = Marker()
        route.header.frame_id = self.map_frame
        route.header.stamp = stamp
        route.ns = "route"
        route.id = 0
        route.action = Marker.ADD
        route.type = Marker.LINE_STRIP
        route.pose.orientation.w = 1.0
        route.scale.x = 0.02
        route.color.r, route.color.g, route.color.b = 0.1, 0.8, 1.0
        route.color.a = 0.8
        targets = ([] if self.cur is None else [self.cur]) + self.pending
        pose = self.robot_pose_map()
        if pose is not None and (targets or self.going_home):
            p0 = Point()
            p0.x, p0.y, p0.z = pose[0], pose[1], 0.02
            route.points.append(p0)
            for rm in targets:
                mx, my = self.d2m(rm["cx"], rm["cy"])
                p = Point()
                p.x, p.y, p.z = mx, my, 0.02
                route.points.append(p)
            if self.going_home and self.home_drawing is not None:
                mx, my = self.d2m(*self.home_drawing)
                p = Point()
                p.x, p.y, p.z = mx, my, 0.02
                route.points.append(p)
        if len(route.points) < 2:
            route.action = Marker.DELETE
        arr.markers.append(route)
        self.marker_pub.publish(arr)


def main():
    rclpy.init()
    node = RoomExplorer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.cmd_pub.publish(Twist())     # 어떤 상태에서 죽든 회전은 멈춘다
            node.spin_active_pub.publish(Bool(data=False))
            node.destroy_node()
        except Exception:                     # noqa: BLE001
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
