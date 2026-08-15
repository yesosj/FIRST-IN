#!/usr/bin/env python3
"""
goal_path_planner_node.py
----------------------------------------------------------
실시간 SLAM 맵(/map)에 직접 A* 경로계획을 한다.

왜 이게 필요한가
  기존 path_planner_node.py 는 '기준 도면(yaml)' 위에서 경로를 찾고, 도면과
  로봇의 관계를 start_x / start_y / start_yaw 로 사람이 알려줘야 한다.
  그 값이 조금이라도 틀리면 벽 위치를 잘못 알고, 도면 크기 자체가 실제 방과
  다르면 아예 맞출 방법이 없다.
  (실측: room_shelf5.yaml 은 1.0 x 1.0 m 인데 라이다가 보는 공간은 7.3 x 5.1 m.
   스캔점의 18% 만 도면 안에 들어와서 어떤 정렬로도 맞지 않았다.)

  /map 은 cartographer 가 만드는 격자로, 헤더의 frame_id 와 origin 이 이미
  로봇 TF 와 같은 좌표계다. 정렬 파라미터가 필요 없고 항상 실제와 일치한다.

기존 플래너와 다른 점
  * 정렬 파라미터(start_x/y/yaw) 없음
  * 출발점이 'SLAM 시작 자리'가 아니라 **지금 로봇이 있는 자리**(TF)
  * 아직 안 가본 곳(unknown)은 기본적으로 '통행 가능'으로 본다.
    벽으로 취급하면 탐사한 좁은 영역 안에서만 경로가 나와서,
    지도의 대부분(회색)에 목적지를 찍어도 경로가 안 생긴다.
    안 가본 곳의 안전은 추종기의 라이다 비상정지가 맡는다.
    보수적으로 가려면 unknown_is_obstacle:=true
  * 목적지/출발점이 부풀린 금지영역에 걸리면 근처 통행 가능 셀로 살짝 옮겨준다

입력  /map        (nav_msgs/OccupancyGrid)  실시간 SLAM 맵
      /goal_pose  (geometry_msgs/PoseStamped) RViz 2D Goal Pose
      /web/replan_request (std_msgs/Empty)    현재 목표 수동 재계획
      /planner/clear_obstacles (std_msgs/Empty) 동적 장애물 기억 초기화
      /planner/reset (std_msgs/Empty)          정렬 변경 전 목표/경로/장애물 상태 폐기
      TF map->base_link                     현재 위치
출력  /plan             (nav_msgs/Path)          계획 경로
      /planner_inflated (nav_msgs/OccupancyGrid) 차체 반영 금지영역(RViz 확인용)
      /planner/status    (std_msgs/String/JSON)  웹/브리지용 계획 상태

차체 반영
  로봇 폭의 절반 + safety_margin 만큼 장애물을 부풀리고 그 안에서만 경로를 찾는다.
  통로가 좁아 경로가 안 나오면 safety_margin 을 줄이면 된다.

실행
  python3 goal_path_planner_node.py --ros-args -p safety_margin:=0.02 -p plan_resolution:=0.03

주의: rclpy/numpy/cv2 필요. venv 에 없으므로 /usr/bin/python3 로 자동 재실행.
----------------------------------------------------------
"""

import heapq
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

import cv2
import numpy as np
import rclpy
import tf2_ros
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import Bool, Empty, Header, String


def latched(depth=1):
    return QoSProfile(depth=depth,
                      reliability=QoSReliabilityPolicy.RELIABLE,
                      durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                      history=QoSHistoryPolicy.KEEP_LAST)


def quat_rot2d(q):
    """쿼터니언에서 xy 평면 회전 성분만 뽑는다(라이다 roll 180 장착도 정확히 반영)."""
    x, y, z, w = q.x, q.y, q.z, q.w
    return ((1 - 2 * (y * y + z * z), 2 * (x * y - z * w)),
            (2 * (x * y + z * w), 1 - 2 * (x * x + z * z)))


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def astar(safe, start, goal, penalty=None):
    """8방향 A*. safe: bool 배열(True=이동가능). 경로 셀 리스트 or None.

    penalty: safe 와 같은 크기의 float 배열(0 이상). 그 칸을 지날 때 이동비용에
    (1 + penalty) 를 곱한다. 벽에 가까울수록 크게 주면 최단거리 대신
    '벽에서 먼 길'을 고른다. None 이면 순수 최단경로(예전 동작).

    penalty >= 0 이므로 실제 비용은 항상 기하 거리 이상이다. 따라서 유클리드
    휴리스틱은 여전히 비용을 과소평가하고, A* 의 최적성이 유지된다.
    """
    h, w = safe.shape
    if not (safe[start] and safe[goal]):
        return None
    # ★ numpy 배열의 개별 원소 접근은 파이썬 루프 안에서 비싸다. A* 는 이웃을
    #   30만 번 넘게 훑으므로(390x324 도면 실측) 그 오버헤드가 그대로 지연이 된다.
    #   시작 전에 한 번 중첩 리스트로 바꿔 두면 인덱싱이 순수 파이썬이 된다.
    #   실측: 1.820초 -> 0.506초 (3.6배), 결과 경로는 동일(475칸).
    safe_l = safe.tolist()
    pen_l = penalty.tolist() if penalty is not None else None
    nbrs = ((-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
            (-1, -1, 1.4142), (-1, 1, 1.4142), (1, -1, 1.4142), (1, 1, 1.4142))
    openh = [(0.0, start)]
    came = {}
    g = {start: 0.0}
    gr, gc = goal
    while openh:
        _, cur = heapq.heappop(openh)
        if cur == goal:
            path = [cur]
            while cur in came:
                cur = came[cur]
                path.append(cur)
            return path[::-1]
        cr, cc = cur
        base = g[cur]
        for dr, dc, cost in nbrs:
            nr, nc = cr + dr, cc + dc
            if 0 <= nr < h and 0 <= nc < w and safe_l[nr][nc]:
                step = cost if pen_l is None else cost * (1.0 + pen_l[nr][nc])
                ng = base + step
                if ng < g.get((nr, nc), 1e18):
                    g[(nr, nc)] = ng
                    came[(nr, nc)] = cur
                    heapq.heappush(openh, (ng + math.hypot(nr - gr, nc - gc),
                                           (nr, nc)))
    return None


class GoalPathPlanner(Node):

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

    def __init__(self):
        super().__init__("goal_path_planner_node")
        # source: "map" = 실시간 SLAM 맵 / "drawing" = 기준 도면 yaml
        # drawing 이어도 출발점은 항상 '지금 로봇이 있는 자리'(TF)다.
        # (기존 path_planner_node.py 는 항상 map 원점에서 경로를 시작해서,
        #  로봇이 조금이라도 움직인 뒤에는 경로가 로봇 뒤에서 시작한다)
        self.source = self._text("source", "drawing").strip().lower()
        if self.source not in ("map", "drawing"):
            self.source = "drawing"
        self.ref_yaml = self._text("reference", os.path.expanduser(
            "~/map_compare/custom_maps/room_shelf5.yaml"))
        self.map_topic = self._text("map_topic", "/map")
        self.base_frame = self._text("base_frame", "base_link")
        self.robot_w = self._num("robot_width", 0.18)     # 실측 도면 17 cm
        self.robot_l = self._num("robot_length", 0.26)    # 실측 도면 25 cm
        self.margin = self._num("safety_margin", 0.02)
        # 기본은 '폭의 절반' = 진행 방향으로 정렬해 지날 때의 실제 측면 제약.
        # 외접원 반경(대각선/2 = 0.1421)을 쓰면 제자리 회전까지 안전하지만,
        # 이 방(1x1m, 가장 넓은 통로 0.36m)에서는 갈 수 있는 곳이 0.009 m^2 밖에
        # 안 남아 경로가 아예 안 나온다. 실측으로 확인한 값:
        #   반경 0.1621 -> 0.009 m^2 / 0.1421 -> 0.045 / 0.11 -> 0.197 / 0.09 -> 0.253
        # 넓은 공간에서 회전 여유까지 보장하려면 robot_radius 를 직접 크게 주면 된다.
        rr = self._num("robot_radius", 0.0)
        self.radius = rr if rr > 0.0 else 0.5 * self.robot_w
        self.radius += self.margin
        self.plan_res = self._num("plan_resolution", 0.03)
        # 벽에서 멀리 가려는 정도. 0 이면 예전처럼 순수 최단경로.
        # 값이 클수록 통로 한가운데를 고집한다(멀리 돌아가더라도).
        self.clear_w = self._num("clearance_weight", 3.0)
        # 여유가 radius + 이만큼이면 더 이상 이득 없음(벌점 0)
        self.clear_extra = self._num("clearance_prefer", 0.06)
        self.unknown_block = self._flag("unknown_is_obstacle", False)
        self.occ_thresh = self._num("occupied_threshold", 50.0)
        self.snap = self._num("snap_radius", 0.30)
        self.simplify = self._num("simplify_tolerance", 0.0)
        # 경로점 간격. 추종기가 lookahead(0.3m) 만큼 앞선 점을 찾아 쓰므로
        # 점이 희박하면(예: 3m 를 4점) lookahead 가 사실상 무의미해진다.
        self.spacing = self._num("path_spacing", 0.04)
        # 클릭한 곳이 로봇과 이어져 있지 않으면 갈 수 있는 가장 가까운 곳으로
        self.snap_reach = self._flag("snap_to_reachable", True)

        # --- 도면에 없는 장애물 감지 / 우회 ---------------------------------
        # 도면상 '자유공간' 인데 스캔점이 찍히면 그건 도면에 없는 물체다.
        # 벽은 도면에 있으므로 걸러진다 — 예전 비상정지가 코너마다 오작동한
        # 이유가 벽까지 장애물로 봤기 때문이다.
        # 기본 off. 켜려면 dynamic_obstacles:=true
        # (off 면 /scan 구독도 검사 타이머도 만들지 않아 이전과 완전히 동일하게 동작)
        self.dyn_on = self._flag("dynamic_obstacles", False)
        # 스캔점이 도면 벽에서 이만큼 이상 떨어져 있어야 '도면에 없는 것' 으로 본다.
        # 벽 근처 점은 정렬오차로 생긴 것으로 보고 버리는 안전장치다.
        #
        # ★ 이 값이 통로 폭에 비해 크면 '막혔는데 안 보이는' 구멍이 생긴다.
        #   이 미로 통로 폭은 0.28 m 이고 로봇은 여유 0.11 m 가 있어야 지난다.
        #   장애물을 통로 한가운데 놓았을 때 양옆 틈 g 에 대해
        #     g < 0.11  -> 로봇이 못 지나감(막힘)
        #     g >= 이 값 -> 옆면 스캔점이 장애물로 인정됨
        #   즉 '막혔고 동시에 보이는' 구간은 g = [이 값, 0.11) 뿐이다.
        #   0.08 이면 그 폭이 30 mm 밖에 안 돼서, 통로를 꽉 막을수록 오히려
        #   도면 벽으로 오인되어 안 보였다. 0.04 로 낮춰 70 mm 로 넓힌다.
        #   대신 정렬 잔차가 큰 점이 장애물로 잡힐 여지는 늘어난다
        #   (실측 잔차 분포: 20mm 이내 81%, 30mm 86%, 50mm 94%).
        #   헛 장애물이 뜨면 다시 올릴 것 — obstacle_min_clear:=0.06
        self.obs_clear = self._num("obstacle_min_clear", 0.04)
        self.obs_ttl = self._num("obstacle_ttl", 3.0)      # 안 보이면 이 시간 뒤 삭제
        # 재계획 직후 같은 길을 즉시 고르는 것을 막기 위한 짧은 기억이다.
        # 기본값을 obstacle_ttl 과 같게 둬 장애물을 치운 뒤에는 반드시 복구된다.
        self.blocked_memory_ttl = self._num(
            "blocked_memory_ttl", self.obs_ttl)
        # ★ true 면 blocked_memory_ttl 을 무시하고 '목적지에 도착할 때까지'
        #   막혔던 지점을 계속 기억한다. 한 번 막힌 길을 몇 초 뒤에 다시
        #   골라서 또 막히는 왕복을 없애기 위한 것이다.
        #   기억은 (1) 목적지 도착 (2) 새 목적지 지정
        #   (3) /planner/clear_obstacles 에서만 지워진다.
        self.memory_until_goal = self._flag("blocked_memory_until_goal", True)
        # ★ 기억을 '경로를 막은 자리 주변' 으로만 제한한다. 보이는 장애물을
        #   전부 기억하면 정렬 잔차로 생긴 칸까지 영구히 쌓여 지도가 봉인된다.
        self.mem_radius = self._num("blocked_memory_radius", 0.10)
        # 기억 셀 상한. 넘으면 오래된 것부터 버린다. 0 이면 무제한(위험).
        self.mem_max = int(self._num("blocked_memory_max", 60))
        self.status_max_obs_points = max(
            1, int(self._num("status_max_obstacle_points", 100)))
        # ★ 차체 앞 판정 — 도면 정렬과 무관하게 "지금 이대로는 못 간다" 를 잡는다.
        #   진행 방향으로 (차체 길이/2 + front_gap) 안, 좌우로 (차체 폭/2 + margin)
        #   안에 스캔점이 들어오면 그 칸을 장애물로 등록한다.
        self.front_gap = self._num("front_gap", 0.03)
        # ★ 차체 앞 상자 안이라도 '도면에 있는 벽' 은 장애물이 아니다.
        #   좌/우회전처럼 전진+회전을 할 때 상자가 진행방향으로 기울면서
        #   코너 바깥 벽을 덮어 버려, 실제로는 지나갈 수 있는 길인데도
        #   벽을 장애물로 잡고 우회로를 만들었다.
        #   도면 벽에서 이 거리 안에 있는 점은 앞 상자 판정에서 제외한다.
        #   벽 위의 점은 clear≈0 이라 걸러지고, 통로 한가운데를 막은 실제
        #   물체는 clear 가 통로폭/2(0.14) 근처라 그대로 남는다.
        #   0.0 으로 주면 이 필터가 꺼져 예전 동작과 완전히 같아진다.
        self.front_clear = self._num("front_min_clear", 0.02)
        # ★ 벽 제외 1겹 — 기준을 현재 정렬 오차에 맞춰 자동으로 넓힌다.
        self.adaptive_wall = self._flag("adaptive_wall_clear", True)
        self.resid_pct = self._num("wall_resid_percentile", 70.0)
        self.resid_cap = self._num("wall_resid_cap", 0.25)
        self.resid_min_pts = int(self._num("wall_resid_min_points", 40))
        self.wall_margin = self._num("wall_clear_margin", 0.02)
        # 상한. 통로를 막은 장애물은 clr 이 0.14 근처라 여기 안 걸린다.
        self.wall_clear_max = self._num("wall_clear_max", 0.10)
        self.wall_resid = 0.0
        self.eff_clear = self.obs_clear
        self.resid_logged = 0.0
        # ★ 벽 제외 2겹 — 이만큼 붙어 있는 덩어리만 장애물로 본다.
        #   흩어진 한두 칸은 정렬 잡음이다. 1 이면 이 필터가 꺼진다.
        self.min_cluster = int(self._num("obstacle_min_cluster", 4))
        # base_link(라이다)에서 차체 앞면까지의 거리[m]. 실측 도면: 0.20
        # (전체 길이 0.25 중 라이다가 뒤에서 0.05 지점에 있다)
        self.lidar_front = self._num("lidar_to_front", 0.20)
        # 라이다에서 이 거리 안의 스캔점만 장애물 후보로 본다[m].
        # 차체 앞면이 0.20 이므로 0.35 면 앞면 기준 15 cm 앞까지다.
        self.obs_range = self._num("obstacle_range", 0.25)
        # 로봇이 부풀림 안에 갇혔을 때 풀어 줄 반경[m]. 실제 벽/장애물은 그대로.
        self.escape_r = self._num("escape_radius", 0.25)
        # 경로가 막혔는지 검사할 구간 길이[m]. 로봇 앞 이만큼만 본다.
        # 목적지까지 다 보면 한참 앞의 장애물 때문에 지금 멀쩡한데도 멈춘다.
        self.block_ahead = self._num("block_check_ahead", 0.30)
        self.front_min_pts = int(self._num("front_min_points", 3))
        # ★ 연속 몇 장의 스캔에서 보여야 '진짜 장애물' 로 인정하는가.
        #   헛 장애물(정렬 잔차)은 깜빡이므로 연속으로는 잘 안 뜬다.
        #   스캔이 약 11.7 Hz 이므로 3장 = 0.26초.
        #   3장(0.26초)이면 0.2 m/s 에서 5.1 cm 를 더 가버려 앞 여유 4 cm 를
        #   넘는다. 2장(0.17초 = 3.4 cm)이 한계라 기본값을 2로 둔다.
        self.front_confirm = max(1, int(self._num("front_confirm_scans", 2)))
        self.front_hits = 0
        self.front_warned = 0.0
        self.replan_cool = self._num("replan_cooldown", 1.5)
        # ★ 제자리 회전 중에는 장애물 인식을 멈춘다.
        #   방향을 크게 바꾸는 동안 차체 앞 판정 상자에 '진짜 벽' 이 들어와서
        #   그걸 장애물로 잡고 또 우회 -> 또 회전 -> 또 우회 로 목적지가
        #   계속 바뀌었다. /cmd_vel 의 linear=0, angular!=0 이 곧 제자리 회전이다.
        #   회전이 끝난 뒤에도 TF/스캔 지연만큼은 더 무시한다(spin_grace_sec).
        self.spin_grace = self._num("spin_grace_sec", 0.5)
        self.spin_until = 0.0
        self.obs_cells = {}          # (row, col) -> 마지막으로 본 시각
        # 막힌 것으로 판정된 셀은 재계획 중 잠깐 기억한다. 영구 set 으로 두면
        # 장애물을 치운 뒤에도 길이 열리지 않으므로 반드시 TTL 로 만료시킨다.
        self.blocked_memory = {}     # (row, col) -> 막힘을 확정한 시각
        self.goal_world = None       # 마지막 목적지 (우회 재계획용)
        self.last_path = []          # 마지막으로 발행한 경로
        self.last_replan = 0.0
        self.blocked_state = False
        self.halted = False       # 장애물로 세워 둔 상태인가
        self.planner_state = "WAITING_FOR_GRID"
        self.planner_detail = "계획 격자 대기"
        self.last_replan_result = "NONE"
        self.replan_attempts = 0
        self.last_block_distance = None
        self.last_plan_length = 0.0
        self.preserve_blocked_memory_once = False

        self.grid = None          # 계획용 (safe, res, origin, yaw, shape)
        self.map_seq = 0

        self.plan_pub = self.create_publisher(Path, "/plan", latched())
        self.infl_pub = self.create_publisher(OccupancyGrid, "/planner_inflated",
                                              latched())
        self.status_pub = self.create_publisher(
            String, "/planner/status", latched())
        self.create_subscription(
            Empty, "/web/replan_request", self.on_replan_request, 10)
        self.create_subscription(
            Empty, "/planner/clear_obstacles", self.on_clear_obstacles, 10)
        self.create_subscription(
            Empty, "/planner/reset", self.on_planner_reset, 10)
        self.create_timer(1.0, self.publish_status)
        if self.source == "map":
            self.create_subscription(OccupancyGrid, self.map_topic, self.on_map,
                                     latched(depth=1))
        else:
            self.ref_occ, self.ref_res = self._load_drawing(self.ref_yaml)
            for nm, dv in (("start_x", 1.72), ("start_y", 1.39),
                           ("start_yaw", 356.0)):
                self.declare_parameter(nm, dv,
                                       ParameterDescriptor(dynamic_typing=True))
            self.create_timer(1.0, self.build_from_drawing)
        self.create_subscription(PoseStamped, "/goal_pose", self.on_goal, 10)
        if self.dyn_on:
            from sensor_msgs.msg import LaserScan
            from rclpy.qos import qos_profile_sensor_data
            self.create_subscription(LaserScan, "/scan", self.on_scan,
                                     qos_profile_sensor_data)
            # 제자리 회전 중인지 알기 위해 실제로 나가는 모터 명령을 본다
            self.create_subscription(Twist, self._text("cmd_vel_topic",
                                                       "/cmd_vel"),
                                     self.on_cmd, 10)
            # 0.5초면 0.2 m/s 주행 시 10 cm 를 더 가버린다 — 10Hz 로 본다
            self.create_timer(0.1, self.check_path_blocked)
            # 목적지에 도착하면 '막혔던 길' 기억을 비운다.
            # (추종기가 latched 로 발행하므로 같은 QoS 로 받는다)
            self.create_subscription(Bool, "/auto_drive/goal_reached",
                                     self.on_goal_reached, latched())

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_logger().info(
            "실시간 맵 기반 경로계획 | 차체 %.3fx%.3f m -> 안전반경 %.4f m | "
            "계획 격자 %.3f m | 미탐사영역=%s"
            % (self.robot_l, self.robot_w, self.radius, self.plan_res,
               "벽으로 취급" if self.unknown_block else "통행 가능"))
        self.get_logger().info(
            "정렬 파라미터(start_x/y/yaw) 가 필요 없습니다 — /map 이 이미 로봇과 "
            "같은 좌표계입니다.")
        self.get_logger().info(
            "출발점 = 지금 로봇이 있는 자리(TF) | 지도 = %s | '/goal_pose' 대기 중"
            % ('실시간 /map' if self.source == 'map' else '기준 도면'))

    # ================= /map -> 계획용 격자 =================
    def on_map(self, msg: OccupancyGrid):
        res = msg.info.resolution
        h, w = msg.info.height, msg.info.width
        if h == 0 or w == 0 or res <= 0.0:
            return
        data = np.asarray(msg.data, dtype=np.int16).reshape(h, w)

        occupied = (data >= self.occ_thresh)
        unknown = (data < 0)
        block = occupied | unknown if self.unknown_block else occupied

        # 계획 격자로 줄인다 (0.01m 격자에 A* 를 그대로 돌리면 느리다).
        # 블록 안에 장애물이 하나라도 있으면 장애물 = 보수적.
        f = max(int(round(self.plan_res / res)), 1)
        if f > 1:
            # 잘라내지 말고 '장애물'로 패딩해서 /map 전체 범위를 덮는다.
            # h//f 로 자르면 끝단 최대 f-1 셀(0.02 m)이 계획격자에서 빠지고,
            # 그 바깥을 목적지로 찍으면 "맵 밖"이라며 경로가 안 나온다.
            ph, pw = -(-h // f), -(-w // f)        # 올림
            if ph < 2 or pw < 2:
                return
            padded = np.ones((ph * f, pw * f), dtype=bool)   # 패딩=장애물(보수적)
            padded[:h, :w] = block
            block_s = padded.reshape(ph, f, pw, f).max(axis=(1, 3))
            res_s = res * f
        else:
            block_s, res_s = block, res

        free = (~block_s).astype(np.uint8)
        if free.sum() == 0:
            return
        dist = cv2.distanceTransform(free, cv2.DIST_L2, 5) * res_s
        safe = dist >= self.radius

        # 통행가능 영역이 여러 덩어리로 쪼개질 수 있다(지도가 아직 안 이어진 곳,
        # 좁아서 부풀린 뒤 끊긴 곳). 로봇이 속한 덩어리 밖은 A* 로 갈 수 없으므로
        # 미리 라벨을 만들어 두고, 목적지가 밖이면 안쪽으로 옮겨 준다.
        ncomp, labels = cv2.connectedComponents(safe.astype(np.uint8),
                                                connectivity=8)
        self.grid = dict(
            safe=safe, clear=dist, labels=labels, ncomp=ncomp,
            res=res_s, shape=block_s.shape,
            ox=msg.info.origin.position.x, oy=msg.info.origin.position.y,
            oyaw=yaw_from_quat(msg.info.origin.orientation),
            frame=msg.header.frame_id or "map")
        self.map_seq += 1
        if self.map_seq == 1 or self.map_seq % 15 == 0:
            self.get_logger().info(
                "맵 수신 %dx%d @ %.3f m (%.1f x %.1f m) | 통행가능 셀 %d개(%.0f%%)"
                % (block_s.shape[1], block_s.shape[0], res_s,
                   block_s.shape[1] * res_s, block_s.shape[0] * res_s,
                   int(safe.sum()), 100.0 * safe.mean()))
        self.publish_inflated()

    @staticmethod
    def _load_drawing(yaml_path):
        sys.path.insert(0, os.path.expanduser("~/slam_test_maps"))
        from slam_map_kit import read_pgm, parse_yaml
        yp = os.path.expanduser(yaml_path)
        meta = parse_yaml(yp)
        res = float(meta["resolution"])
        occ_t = float(meta.get("occupied_thresh", 0.65))
        pgm = os.path.join(os.path.dirname(os.path.abspath(yp)),
                           os.path.basename(meta["image"]))
        w, h, _mx, px = read_pgm(pgm)
        img = np.frombuffer(bytes(px), dtype=np.uint8).reshape(h, w)
        p = (255.0 - img.astype(np.float32)) / 255.0
        occ = np.zeros((h, w), dtype=np.uint8)
        occ[p > occ_t] = 1
        return np.flipud(occ), res

    def build_from_drawing(self):
        """기준 도면을 map 프레임 격자로 만든다.

        start_x/y/yaw 를 매번 다시 읽으므로 auto_align_node 가 실행 중에 바꾼
        값이 바로 반영된다. 값이 그대로면 다시 만들지 않는다.
        """
        sx = float(self.get_parameter("start_x").value)
        sy = float(self.get_parameter("start_y").value)
        yaw = math.radians(float(self.get_parameter("start_yaw").value))
        key = (round(sx, 4), round(sy, 4), round(yaw, 5))
        if getattr(self, "_key", None) == key:
            return
        self._key = key
        occ, res = self.ref_occ, self.ref_res
        free = (occ == 0).astype(np.uint8)
        dist = cv2.distanceTransform(free, cv2.DIST_L2, 5) * res
        safe = dist >= self.radius
        ncomp, labels = cv2.connectedComponents(safe.astype(np.uint8),
                                                connectivity=8)
        # 도면 원점(왼아래)이 map 프레임 어디에 오는가: Pm = R(-yaw)*(Pd - S)
        c, s_ = math.cos(-yaw), math.sin(-yaw)
        ox = c * (-sx) - s_ * (-sy)
        oy = s_ * (-sx) + c * (-sy)
        # 벽에 붙는 것에 매기는 벌점. 최단거리만 보면 경로가 부풀림 경계(분홍선)에
        # 딱 붙어서 나오는데, 실제로는 정렬오차·조향오차가 있으므로 통로 한가운데로
        # 가는 편이 안전하다. 여유가 radius 면 벌점 최대, prefer 이상이면 0.
        prefer = self.radius + self.clear_extra
        pen = np.clip((prefer - dist) / max(prefer - self.radius, 1e-6), 0.0, 1.0)
        pen = (pen * self.clear_w).astype(np.float32)
        self.grid = dict(safe=safe, clear=dist, penalty=pen, labels=labels,
                         ncomp=ncomp, res=res, shape=occ.shape,
                         ox=ox, oy=oy, oyaw=-yaw, frame="map")
        self.get_logger().info(
            "도면 격자 갱신 (start %.2f, %.2f, %.1f도) | 통행가능 %d셀 = %.3f m^2"
            % (sx, sy, math.degrees(yaw), int(safe.sum()),
               int(safe.sum()) * res * res))
        self.publish_inflated()

    # ================= 도면에 없는 장애물 =================
    def on_cmd(self, msg: Twist):
        """제자리 회전 중이면 그 동안 장애물 인식을 멈춘다.

        전진 없이 회전만 하는 명령(linear=0, angular!=0)이 곧 제자리 회전이다.
        control_mode 가 keys 든 smooth 든 같은 신호라 둘 다 통한다.
        회전이 끝난 직후에도 TF/스캔 지연만큼(spin_grace_sec) 더 무시한다 —
        안 그러면 회전 마지막 순간의 스캔이 뒤늦게 들어와 벽을 장애물로 잡는다.
        """
        if abs(msg.linear.x) < 1e-6 and abs(msg.angular.z) > 1e-6:
            was = self.spin_until
            self.spin_until = self.now() + self.spin_grace
            if was <= self.now():          # 회전 시작 순간에만 한 줄
                self.get_logger().info(
                    "제자리 회전 시작 — 도는 동안 장애물 인식을 멈춥니다.")

    def on_scan(self, msg):
        """도면상 자유공간에 찍힌 스캔점만 '장애물' 로 모은다."""
        if self.grid is None:
            return
        if self.now() < self.spin_until:
            return          # 제자리 회전 중 — 벽을 장애물로 잡지 않도록 멈춤
        try:
            tr = self.tf_buffer.lookup_transform(
                self.grid["frame"], msg.header.frame_id, rclpy.time.Time())
        except tf2_ros.TransformException:
            return
        q = tr.transform.rotation
        yaw = yaw_from_quat(q)
        tx = tr.transform.translation.x
        ty = tr.transform.translation.y
        r = np.asarray(msg.ranges, dtype=np.float64)
        a = msg.angle_min + np.arange(r.size) * msg.angle_increment
        # ★ 라이다에서 obs_range 안에 있는 점만 본다.
        #   예전엔 도면 안이기만 하면 몇 미터 떨어진 점도 장애물로 등록했다.
        #   그러면 정렬 잔차로 생긴 먼 곳의 잡음까지 쌓여서, 지금 코앞은
        #   멀쩡한데 한참 앞이 막힌 것으로 나와 멈춰 버린다.
        ok = (np.isfinite(r) & (r > max(msg.range_min, 0.03))
              & (r < (msg.range_max if msg.range_max > 0 else 12.0))
              & (r <= self.obs_range))
        if not ok.any():
            return
        # 라이다 roll 180 도 장착이라 y 가 뒤집힌다. TF 의 yaw 만 쓰면 안 되므로
        # 회전행렬 대신 2D 로 근사하지 말고 부호를 TF 에서 그대로 가져온다.
        m = quat_rot2d(q)
        lx = r[ok] * np.cos(a[ok])
        ly = r[ok] * np.sin(a[ok])
        wx = m[0][0] * lx + m[0][1] * ly + tx
        wy = m[1][0] * lx + m[1][1] * ly + ty
        g = self.grid
        c, s_ = math.cos(-g["oyaw"]), math.sin(-g["oyaw"])
        dx, dy = wx - g["ox"], wy - g["oy"]
        col = np.floor((c * dx - s_ * dy) / g["res"]).astype(np.int32)
        row = np.floor((s_ * dx + c * dy) / g["res"]).astype(np.int32)
        h, w = g["shape"]
        inside = (row >= 0) & (row < h) & (col >= 0) & (col < w)
        if not inside.any():
            return
        rr, cc = row[inside], col[inside]
        # 도면 벽에서 obs_clear 이상 떨어진 곳에 찍힌 점 = 도면에 없는 물체
        clr = g["clear"][rr, cc]
        # ★ 1겹: 정렬이 어긋난 만큼 기준을 넓힌다.
        #   고정 4 cm 로 두면 정렬이 그보다 어긋나는 순간 벽이 통째로 장애물이
        #   된다(실측: 4 cm 어긋남 -> 벽의 53%, 8 cm -> 100% 가 장애물).
        #   스캔점의 대부분은 벽이므로, 그 점들의 '도면 벽까지 거리' 분포가
        #   0 근처가 아니라 N 근처에 몰려 있으면 지금 N 만큼 어긋난 것이다.
        #   그 백분위수를 현재 정렬 오차로 보고 기준에 더한다.
        #   통로를 막은 진짜 장애물은 한가운데 여유가 0.14 m 라, 상한
        #   (wall_clear_max)을 두면 아무리 넓혀도 절대 걸리지 않는다.
        eff_clear = self.obs_clear
        self.wall_resid = 0.0
        if self.adaptive_wall:
            cand = clr[clr < self.resid_cap]
            if cand.size >= self.resid_min_pts:
                self.wall_resid = float(np.percentile(cand, self.resid_pct))
                eff_clear = min(
                    max(self.obs_clear, self.wall_resid + self.wall_margin),
                    self.wall_clear_max)
        self.eff_clear = eff_clear
        far = clr >= eff_clear
        # ★ 그것과 별개로, 차체 바로 앞(전방 front_gap 이내)에 뭔가 있으면
        #   도면과 얼마나 떨어졌는지와 무관하게 '못 간다'로 본다.
        #   obs_clear 만으로는 통로를 꽉 막은 장애물일수록 옆면이 벽에 가까워져
        #   오히려 걸러지는 구멍이 있었다.
        #   단, 앞 상자 안이라도 도면 벽 위의 점은 뺀다(front_min_clear).
        #   안 그러면 코너를 돌 때 상자가 바깥 벽을 덮어 헛 장애물이 된다.
        front = (self.front_zone_mask(wx[inside], wy[inside])
                 & (clr >= max(self.front_clear, eff_clear)))
        mark = far | front
        # ★ 2겹: 흩어진 점은 버린다.
        #   벽에서 새는 점은 정렬 잡음이라 여기저기 한두 개씩 흩어진다.
        #   진짜 물체는 라이다에 한 덩어리로 잡혀 붙어 있는 칸이 여러 개다.
        #   이번 스캔에서 찍힌 칸들을 이어 보고, 덩어리가 작으면 잡음으로 본다.
        if self.min_cluster > 1 and mark.any():
            mark = self.keep_clusters(rr[mark], cc[mark], mark)
        now = self.now()
        for r0, c0 in zip(rr[mark], cc[mark]):
            self.obs_cells[(int(r0), int(c0))] = now
        # 정렬 상태를 눈에 보이게 남긴다. 잔차가 커지면 도면과 실제가 그만큼
        # 벌어진 것이고, 기준이 자동으로 넓어진 것도 같이 보인다.
        if self.adaptive_wall and now - self.resid_logged > 5.0:
            self.resid_logged = now
            self.get_logger().info(
                "정렬 잔차 %.3f m -> 벽 제외 기준 %.3f m (기본 %.3f) | "
                "장애물 %d칸 (덩어리 %d칸 이상만) / 스캔점 %d개"
                % (self.wall_resid, eff_clear, self.obs_clear,
                   int(mark.sum()), self.min_cluster, int(inside.sum())))
        nf = int(front.sum())
        # ★ 한 장의 스캔만 보고 세우면 안 된다.
        #   정렬 잔차 때문에 벽 위의 점이 간헐적으로 상자 안에 들어온다
        #   (front_min_clear 2 cm 기준이면 잔차 분포상 약 19%가 샌다).
        #   그때마다 빈 경로를 내면 로봇이 가다 서다를 반복한다.
        #   헛 장애물은 '깜빡'이고 진짜 장애물은 '계속' 보인다는 차이를 쓴다.
        #   연속 front_confirm 장에서 보일 때만 진짜로 인정한다.
        if nf >= self.front_min_pts:
            self.front_hits += 1
        else:
            self.front_hits = 0
        if self.front_hits >= self.front_confirm:
            # 여기까지 왔으면 진짜다. 즉시 세운다 — 0.1초 타이머를 기다리면
            # 그 사이에도 로봇이 더 가버린다.
            if not self.halted:
                self.halted = True
                self.plan_pub.publish(Path(header=self._hdr()))
                self.get_logger().warn(
                    "차체 앞 장애물 — 연속 %d장 확인. 즉시 정지하고 우회로를 찾습니다."
                    % self.front_hits)
            self.check_path_blocked()      # 타이머를 기다리지 않고 바로 검사
        elif nf >= self.front_min_pts:
            self.get_logger().info(
                "차체 앞 스캔점 %d개 (연속 %d/%d) — 아직 확정 아님, 그대로 진행합니다."
                % (nf, self.front_hits, self.front_confirm),
                throttle_duration_sec=3.0)
        if self.front_hits >= self.front_confirm and now - self.front_warned > 3.0:
            self.front_warned = now
            self.get_logger().warn(
                "차체 앞 %.2f m 안에 장애물이 있습니다 (스캔점 %d개). "
                "차체 %.2fx%.2f m 기준으로 이대로는 지나갈 수 없습니다."
                % (self.lidar_front + self.front_gap, nf,
                   self.robot_l, self.robot_w))

    def keep_clusters(self, mr, mc, mark):
        """이번 스캔에서 찍힌 칸 중 '덩어리'만 남긴다.

        벽에서 새는 점은 정렬 잡음이라 한두 개씩 흩어진다. 진짜 물체는
        라이다 광선 여러 개가 같은 면을 때리므로 붙어 있는 칸이 여러 개다.
        8방향으로 이어 보고 min_cluster 칸 미만인 덩어리는 버린다.

        전체 격자(390x324)에 그리지 않고 찍힌 칸의 바운딩박스 안에서만
        계산한다 — 매 스캔 도는 코드라 전체를 훑으면 비싸다.
        """
        if mr.size == 0:
            return mark
        r0, r1 = int(mr.min()), int(mr.max())
        c0, c1 = int(mc.min()), int(mc.max())
        sub = np.zeros((r1 - r0 + 3, c1 - c0 + 3), np.uint8)
        sub[mr - r0 + 1, mc - c0 + 1] = 1
        n, lab, st, _ = cv2.connectedComponentsWithStats(sub, 8)
        big = {i for i in range(1, n)
               if st[i, cv2.CC_STAT_AREA] >= self.min_cluster}
        if not big:
            return np.zeros_like(mark)
        keep_lab = lab[mr - r0 + 1, mc - c0 + 1]
        ok = np.isin(keep_lab, list(big))
        out = np.zeros_like(mark)
        idx = np.flatnonzero(mark)
        out[idx[ok]] = True
        return out

    def front_zone_mask(self, px, py):
        """차체 앞쪽 상자 안에 들어온 스캔점을 True 로 표시한다.

        진행 방향은 로봇의 heading 이 아니라 **경로 진행 방향**을 쓴다.
        라이다가 180도 돌려 장착돼 있어 base_link 의 +x 가 로봇의 앞이 아니고,
        그 보정값(heading_offset)은 추종기만 알고 있기 때문이다.
        경로를 따라가는 중이라면 '경로가 가리키는 쪽'이 곧 로봇이 갈 방향이다.

        상자 크기: 진행방향으로 (lidar_to_front + front_gap),
                  좌우로 (차체 폭/2 + safety_margin)

        ★ 라이다(=base_link)는 차체 한가운데가 아니다. 실측 도면 기준으로
          앞면까지 0.20 m, 뒷면까지 0.05 m 다(전체 길이 0.25 m).
          그래서 '차체 길이/2'(0.11 m) 를 쓰면 판정 상자가 실제 앞면보다
          9 cm 뒤에서 끝나 버려서, 앞에 놓인 장애물을 못 봤다.
        """
        n = len(px)
        if n == 0 or not self.last_path:
            return np.zeros(n, bool)
        pose = self.robot_pose()
        if pose is None:
            return np.zeros(n, bool)
        ax = np.array([p[0] for p in self.last_path])
        ay = np.array([p[1] for p in self.last_path])
        i0 = int(np.argmin((ax - pose[0]) ** 2 + (ay - pose[1]) ** 2))
        j = i0
        while (j < len(ax) - 1
               and math.hypot(ax[j] - pose[0], ay[j] - pose[1]) < 0.15):
            j += 1
        ux, uy = ax[j] - pose[0], ay[j] - pose[1]
        d = math.hypot(ux, uy)
        if d < 1e-6:
            return np.zeros(n, bool)
        ux, uy = ux / d, uy / d
        dx, dy = px - pose[0], py - pose[1]
        along = dx * ux + dy * uy                 # 진행 방향 성분
        lat = np.abs(-dx * uy + dy * ux)          # 좌우 성분
        reach = self.lidar_front + self.front_gap
        half = 0.5 * self.robot_w + self.margin
        return (along > 0.0) & (along <= reach) & (lat <= half)

    def dynamic_safe(self):
        """도면 safe 에서 '도면에 없는 장애물' 을 차체 반경만큼 부풀려 뺀다."""
        g = self.grid
        self.prune_obstacles()
        cells = set(self.obs_cells) | set(self.blocked_memory)
        if not cells:
            return g["safe"], 0
        h, w = g["shape"]
        mask = np.zeros((h, w), np.uint8)
        for (r0, c0) in cells:
            mask[r0, c0] = 1
        d = cv2.distanceTransform(1 - mask, cv2.DIST_L2, 5) * g["res"]
        return g["safe"] & (d >= self.radius), len(cells)

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def prune_obstacles(self):
        now = self.now()
        for cell in [
            cell for cell, seen in self.obs_cells.items()
            if now - seen > self.obs_ttl
        ]:
            self.obs_cells.pop(cell, None)
        # 목적지 도착까지 기억하는 모드에서는 시간으로 지우지 않는다.
        if self.memory_until_goal:
            return
        for cell in [
            cell for cell, seen in self.blocked_memory.items()
            if now - seen > self.blocked_memory_ttl
        ]:
            self.blocked_memory.pop(cell, None)

    def publish_status(self):
        self.prune_obstacles()
        now = self.now()
        all_cells = sorted(set(self.obs_cells) | set(self.blocked_memory))
        points = []
        sample_cells = all_cells
        if len(all_cells) > self.status_max_obs_points:
            stride = int(math.ceil(
                len(all_cells) / float(self.status_max_obs_points)))
            sample_cells = all_cells[::stride][:self.status_max_obs_points]
        if self.grid is not None:
            for row, col in sample_cells:
                x, y = self.cell_to_world(row, col)
                seen = self.obs_cells.get(
                    (row, col), self.blocked_memory.get((row, col)))
                points.append({
                    "row": row,
                    "col": col,
                    "x": round(float(x), 4),
                    "y": round(float(y), 4),
                    "age_sec": (
                        round(max(0.0, now - seen), 3)
                        if seen is not None else None
                    ),
                    "remembered": (row, col) in self.blocked_memory,
                })
        state = self.planner_state
        detail = self.planner_detail
        if self.grid is not None and state == "WAITING_FOR_GRID":
            state, detail = "READY", "목표 대기"
        payload = {
            "state": state,
            "detail": detail,
            "source": self.source,
            "dynamic_obstacles_enabled": self.dyn_on,
            "blocked": self.blocked_state,
            "halted": self.halted,
            "goal": (
                {"x": self.goal_world[0], "y": self.goal_world[1]}
                if self.goal_world is not None else None
            ),
            "path_points": len(self.last_path),
            "path_length_m": round(self.last_plan_length, 3),
            "last_block_distance_m": self.last_block_distance,
            "last_replan_result": self.last_replan_result,
            "replan_attempts": self.replan_attempts,
            "dynamic_obstacles": {
                "count": len(all_cells),
                "live_count": len(self.obs_cells),
                "remembered_count": len(self.blocked_memory),
                "ttl_sec": self.obs_ttl,
                "memory_ttl_sec": self.blocked_memory_ttl,
                "points": points,
                "sampled_count": len(points),
                "truncated": len(all_cells) > len(points),
            },
            "parameters": {
                "obstacle_range_m": self.obs_range,
                "block_check_ahead_m": self.block_ahead,
                "replan_cooldown_sec": self.replan_cool,
                "escape_radius_m": self.escape_r,
                "inflation_radius_m": self.radius,
            },
            "timestamp_ns": self.get_clock().now().nanoseconds,
        }
        self.status_pub.publish(String(data=json.dumps(
            payload, ensure_ascii=False, separators=(",", ":"))))

    def set_planner_status(self, state, detail):
        self.planner_state = state
        self.planner_detail = detail
        self.publish_status()

    def on_replan_request(self, _msg):
        if self.goal_world is None:
            self.last_replan_result = "NO_GOAL"
            self.set_planner_status("READY", "재계획할 목표가 없습니다.")
            return
        goal = PoseStamped()
        goal.header = self._hdr()
        goal.pose.position.x = float(self.goal_world[0])
        goal.pose.position.y = float(self.goal_world[1])
        goal.pose.orientation.w = 1.0
        self.last_replan_result = "MANUAL_REQUEST"
        self.preserve_blocked_memory_once = True
        self.on_goal(goal)

    def on_goal_reached(self, msg: Bool):
        """목적지 도착 -> 이번 주행에서 막혔던 길 기억을 비운다.

        기억은 '이번 목적지로 가는 동안' 만 유효하다. 다음 주행에서는
        장애물이 치워졌을 수도 있으므로 그대로 들고 가면 안 된다.
        """
        if not msg.data or not self.blocked_memory:
            return
        self.get_logger().info(
            "목적지 도착 — 이번 주행에서 기억한 막힌 지점 %d칸을 비웁니다."
            % len(self.blocked_memory))
        self.blocked_memory.clear()
        self.blocked_state = False

    def on_clear_obstacles(self, _msg):
        removed_live = len(self.obs_cells)
        removed_memory = len(self.blocked_memory)
        self.obs_cells.clear()
        self.blocked_memory.clear()
        self.blocked_state = False
        self.last_replan_result = "OBSTACLES_CLEARED"
        self.set_planner_status(
            "READY" if self.goal_world is None else "PLANNING",
            "동적 장애물 %d셀과 막힌 길 기억 %d셀을 초기화했습니다."
            % (removed_live, removed_memory),
        )
        if self.goal_world is not None:
            self.on_replan_request(Empty())

    def on_planner_reset(self, _msg):
        """로봇 pose/도면 정렬이 바뀌기 전 이전 계획 상태를 완전히 폐기한다.

        정렬 변경 뒤에 이전 map 좌표의 목표·경로나 장애물 기억을
        재사용하면 로봇이 잘못된 위치로 출발할 수 있다. 빈 /plan 을
        latched 발행해 추종기에 남아 있는 경로까지 즉시 지운다.
        """
        removed_live = len(self.obs_cells)
        removed_memory = len(self.blocked_memory)
        had_goal = self.goal_world is not None
        path_points = len(self.last_path)

        self.obs_cells.clear()
        self.blocked_memory.clear()
        self.goal_world = None
        self.last_path = []

        self.last_replan = 0.0
        self.blocked_state = False
        self.halted = False
        self.last_replan_result = "RESET"
        self.replan_attempts = 0
        self.last_block_distance = None
        self.last_plan_length = 0.0
        self.preserve_blocked_memory_once = False

        # 장애물 확정/회전 유예도 예전 pose 기준의 실행 상태이므로
        # 함께 비운다. 새 pose 에서 다음 스캔으로 다시 판정한다.
        self.front_hits = 0
        self.front_warned = 0.0
        self.spin_until = 0.0

        self.plan_pub.publish(Path(header=self._hdr()))
        self.set_planner_status(
            "READY",
            "로봇 위치 재설정을 위해 이전 목표와 경로를 초기화했습니다.",
        )
        self.get_logger().info(
            "플래너 전체 초기화(/planner/reset) — 목표=%s, 경로 %d점, "
            "동적 장애물 %d셀, 막힌 길 기억 %d셀 폐기"
            % ("있음" if had_goal else "없음", path_points,
               removed_live, removed_memory)
        )

    def check_path_blocked(self):
        """현재 경로가 장애물로 막혔으면 우회로를 찾고, 없으면 정지시킨다."""
        if self.grid is None or self.goal_world is None or not self.last_path:
            return
        if self.now() < self.spin_until:
            return          # 제자리 회전 중
        safe_dyn, nobs = self.dynamic_safe()
        if nobs == 0:
            if self.blocked_state:
                self.blocked_state = False
                self.get_logger().info("장애물이 사라졌습니다 — 정상 주행으로 복귀")
                self.set_planner_status("NAVIGATING", "장애물이 사라졌습니다.")
            self.resume_if_halted()
            return
        pose = self.robot_pose()
        if pose is None:
            return
        # 로봇에서 가장 가까운 경로점부터 앞쪽만 검사
        px = np.array([p[0] for p in self.last_path])
        py = np.array([p[1] for p in self.last_path])
        i0 = int(np.argmin((px - pose[0]) ** 2 + (py - pose[1]) ** 2))
        hit = None
        # ★ 목적지까지 전 구간을 보면, 한참 앞(예: 0.73 m)에 있는 것 때문에
        #   지금 잘 가고 있는데도 멈춘다. 코앞만 본다.
        walked = 0.0
        for i in range(i0, len(self.last_path)):
            if i > i0:
                walked += math.hypot(
                    self.last_path[i][0] - self.last_path[i - 1][0],
                    self.last_path[i][1] - self.last_path[i - 1][1])
                if walked > self.block_ahead:
                    break
            cell = self.world_to_cell(*self.last_path[i])
            if not self.in_bounds(cell) or not safe_dyn[cell]:
                hit = i
                break
        if hit is None:
            if self.blocked_state:
                self.blocked_state = False
                self.get_logger().info("경로가 다시 열렸습니다 — 정상 주행")
                self.set_planner_status("NAVIGATING", "경로가 다시 열렸습니다.")
            self.resume_if_halted()
            return
        if self.now() - self.last_replan < self.replan_cool:
            return
        self.last_replan = self.now()
        dist_ahead = math.hypot(self.last_path[hit][0] - pose[0],
                                self.last_path[hit][1] - pose[1])
        self.last_block_distance = round(dist_ahead, 3)
        self.blocked_state = True
        # 로봇이 이미 부풀림 안에 들어가 있으면 그 주변만 풀어 준다 (안 그러면
        # 출발점이 통행 불가라서 무조건 '우회로 없음' 이 된다)
        search, relaxed = self.relax_for_escape(safe_dyn, pose)
        if relaxed:
            self.get_logger().warn(
                "로봇이 부풀림(분홍) 영역 안에 있습니다 — 출발점 주변 %.2f m 의 "
                "여유 제한을 풀고 빠져나갈 경로를 찾습니다." % self.escape_r)
        # 지금 위치에서 원래 목적지까지 우회로가 있는지
        start = self.snap_to_safe_on(search, self.world_to_cell(*pose), "출발")
        goal = self.snap_to_safe_on(search,
                                    self.world_to_cell(*self.goal_world), "목적지")
        cells = (astar(search, start, goal, self.grid.get("penalty"))
                 if (start and goal) else None)
        detect = self.lidar_front + self.front_gap
        self.replan_attempts += 1
        if cells:
            self.get_logger().warn(
                "경로가 %.2f m 앞에서 막혔습니다 — 도면에 없는 장애물 %d셀 "
                "(감지기준: 차체 앞면 %.0f cm). 다른 경로로 탐색합니다."
                % (dist_ahead, nobs, self.front_gap * 100))
            self.halted = False
            self.last_replan_result = "DETOUR_FOUND"
            self.emit_path(cells)
        else:
            self.get_logger().error(
                "경로가 %.2f m 앞에서 막혔고 목적지까지 우회로가 없습니다 "
                "(감지기준: 차체 앞면 %.0f cm, 라이다 %.2f m 이내). "
                "로봇을 정지합니다 — 장애물을 치우거나 수동 조작이 필요합니다. "
                "%s"
                % (dist_ahead, self.front_gap * 100, self.obs_range,
                   "출발점 주변을 풀어도 길이 없었습니다."
                   if relaxed else
                   "통로가 차체 여유(%.2f m)보다 좁아졌을 수 있습니다."
                   % self.radius))
            # ★ last_path 를 비우면 안 된다. 비우면 이 함수 첫 줄의
            #   'not self.last_path' 에 걸려 다음부터 검사 자체가 안 돌고,
            #   우회 시도가 딱 한 번으로 끝난다(예전 동작).
            #   경로는 그대로 들고 있고 로봇만 세운 뒤, replan_cooldown 마다
            #   계속 다시 시도한다 — 장애물이 치워지거나 길이 열리면 재개.
            self.halted = True
            self.last_replan_result = "NO_ROUTE"
            self.plan_pub.publish(Path(header=self._hdr()))
            self.set_planner_status(
                "HALTED_NO_ROUTE",
                "장애물로 경로가 막혔고 우회로가 없습니다.",
            )

    def relax_for_escape(self, safe_dyn, pose):
        """로봇이 부풀림(분홍) 안에 있으면 그 주변만 통행 가능으로 풀어 준다.

        부풀림은 '중심이 여기 있으면 벽에 닿는다'는 뜻이다. 그래서 로봇이 이미
        그 안에 들어가 있으면 출발점 자체가 통행 불가가 되고, snap 도 실패해서
        A* 가 시작조차 못 한다 -> 무조건 "우회로가 없습니다" 가 뜬다.
        (통로 0.28 m 에 부풀림 0.11 m 면 여유가 ±0.03 m 뿐이라 흔히 일어난다)

        실제 벽과 장애물 셀은 그대로 두고, **부풀림만** escape_radius 안에서
        무시한다. 그러면 로봇이 지금 자리에서 빠져나오는 경로는 찾을 수 있고,
        그 밖에서는 여전히 차체 여유를 지킨다.
        """
        g = self.grid
        cell = self.world_to_cell(*pose)
        if not self.in_bounds(cell) or safe_dyn[cell]:
            return safe_dyn, False
        h, w = g["shape"]
        rr, cc = np.ogrid[:h, :w]
        rad_px = max(self.escape_r / g["res"], 1.0)
        near = ((rr - cell[0]) ** 2 + (cc - cell[1]) ** 2) <= rad_px ** 2
        free = g["clear"] > 0.5 * g["res"]          # 도면 벽이 아닌 곳
        if self.obs_cells:                          # 장애물 셀 자체는 못 지나감
            blk = np.zeros((h, w), bool)
            for (r0, c0) in self.obs_cells:
                blk[r0, c0] = True
            free &= ~blk
        return (safe_dyn | (near & free)), True

    def snap_to_safe_on(self, safe, cell, what):
        """주어진 safe 배열 기준으로 통행 가능 셀 찾기 (우회 재계획용)."""
        if self.in_bounds(cell) and safe[cell]:
            return cell
        g = self.grid
        rad = max(int(self.snap / g["res"]), 1)
        h, w = g["shape"]
        best, bestd = None, 1e18
        r0, c0 = cell
        for dr in range(-rad, rad + 1):
            for dc in range(-rad, rad + 1):
                r, c = r0 + dr, c0 + dc
                if 0 <= r < h and 0 <= c < w and safe[r, c]:
                    d = dr * dr + dc * dc
                    if d < bestd:
                        bestd, best = d, (r, c)
        return best

    # ================= 좌표 변환 =================
    def world_to_cell(self, x, y):
        g = self.grid
        dx, dy = x - g["ox"], y - g["oy"]
        c, s = math.cos(-g["oyaw"]), math.sin(-g["oyaw"])
        lx, ly = c * dx - s * dy, s * dx + c * dy
        return int(math.floor(ly / g["res"])), int(math.floor(lx / g["res"]))

    def cell_to_world(self, row, col):
        g = self.grid
        lx, ly = (col + 0.5) * g["res"], (row + 0.5) * g["res"]
        c, s = math.cos(g["oyaw"]), math.sin(g["oyaw"])
        return c * lx - s * ly + g["ox"], s * lx + c * ly + g["oy"]

    def in_bounds(self, cell):
        h, w = self.grid["shape"]
        return 0 <= cell[0] < h and 0 <= cell[1] < w

    def snap_to_safe(self, cell, what):
        """막힌 셀이면 snap_radius 안에서 가장 가까운 통행 가능 셀로 옮긴다."""
        g = self.grid
        if self.in_bounds(cell) and g["safe"][cell]:
            return cell
        rad = max(int(self.snap / g["res"]), 1)
        h, w = g["shape"]
        best, bestd = None, 1e18
        r0, c0 = cell
        for dr in range(-rad, rad + 1):
            for dc in range(-rad, rad + 1):
                r, c = r0 + dr, c0 + dc
                if 0 <= r < h and 0 <= c < w and g["safe"][r, c]:
                    d = dr * dr + dc * dc
                    if d < bestd:
                        bestd, best = d, (r, c)
        if best is not None:
            self.get_logger().warn(
                "%s 위치가 벽/차체여유 안쪽이라 %.2f m 옮겨 잡았습니다 "
                "(그 자리 벽까지 %.3f m < 필요 %.3f m)"
                % (what, math.sqrt(bestd) * g["res"],
                   float(g["clear"][cell]) if self.in_bounds(cell) else -1.0,
                   self.radius))
        else:
            self.get_logger().error(
                "%s 위치 주변 %.2f m 안에 통행 가능한 곳이 없습니다. "
                "그 자리는 벽까지 %.3f m 인데 차체 %0.2fx%0.2f m 에는 %.3f m 가 "
                "필요합니다. robot_radius 를 줄이거나(예: %.3f) safety_margin 을 "
                "줄이세요."
                % (what, self.snap,
                   float(g["clear"][cell]) if self.in_bounds(cell) else -1.0,
                   self.robot_l, self.robot_w, self.radius, 0.5 * self.robot_w))
        return best

    def snap_to_reachable(self, start, goal):
        """목적지가 로봇과 다른 덩어리에 있으면, 같은 덩어리 안에서 목적지에
        가장 가까운 셀로 옮긴다. (안 옮기면 그냥 '경로 없음' 이 되어 버린다)"""
        g = self.grid
        lab = g["labels"]
        mine = int(lab[start])
        if mine == 0 or int(lab[goal]) == mine:
            return goal
        if not self.snap_reach:
            self.get_logger().warn(
                "목적지가 로봇과 이어져 있지 않습니다(덩어리 #%d vs #%d). "
                "지도가 아직 안 이어졌거나 통로가 차체보다 좁습니다."
                % (int(lab[goal]), mine))
            return goal
        cells = np.argwhere(lab == mine)
        if len(cells) == 0:
            return goal
        d2 = ((cells[:, 0] - goal[0]) ** 2 + (cells[:, 1] - goal[1]) ** 2)
        i = int(np.argmin(d2))
        new = (int(cells[i, 0]), int(cells[i, 1]))
        moved = math.sqrt(float(d2[i])) * g["res"]
        self.get_logger().warn(
            "목적지가 로봇과 이어져 있지 않습니다 — 갈 수 있는 가장 가까운 곳으로 "
            "%.2f m 옮겼습니다. (지도가 더 만들어지면 원래 지점까지 갈 수 있습니다)"
            % moved)
        return new

    # ================= 목적지 -> 경로 =================
    def robot_pose(self):
        try:
            tr = self.tf_buffer.lookup_transform(
                self.grid["frame"], self.base_frame, rclpy.time.Time())
        except tf2_ros.TransformException as e:
            self.get_logger().warn("현재 위치 TF 를 못 받았습니다: %s" % e)
            return None
        t = tr.transform.translation
        return float(t.x), float(t.y)

    def on_goal(self, msg: PoseStamped):
        # 새 목적지를 찍으면 '막혔던 길' 기억을 초기화한다.
        # (장애물을 치우고 다시 시도하는 경우가 대부분이므로)
        preserve_memory = self.preserve_blocked_memory_once
        self.preserve_blocked_memory_once = False
        if self.blocked_memory and not preserve_memory:
            self.get_logger().info(
                "새 목적지 — 기억해 둔 막힌 지점 %d칸을 초기화합니다."
                % len(self.blocked_memory))
            self.blocked_memory.clear()
        self.halted = False
        if self.grid is None:
            self.get_logger().warn(
                "아직 %s 를 못 받았습니다 — SLAM 이 맵을 낼 때까지 기다리세요."
                % self.map_topic)
            self.set_planner_status("WAITING_FOR_GRID", "계획 격자 대기")
            return
        pose = self.robot_pose()
        if pose is None:
            return
        gx, gy = float(msg.pose.position.x), float(msg.pose.position.y)
        self.goal_world = (gx, gy)
        self.blocked_state = False
        self.last_block_distance = None
        self.last_replan_result = (
            "MANUAL_REPLAN_REQUEST" if preserve_memory else "GOAL_REQUEST"
        )
        self.set_planner_status("PLANNING", "현재 위치에서 목표까지 경로 계산")

        start = self.world_to_cell(*pose)
        goal = self.world_to_cell(gx, gy)
        self.get_logger().info(
            "현재 (%.2f, %.2f) -> 목적지 (%.2f, %.2f) | 셀 %s -> %s"
            % (pose[0], pose[1], gx, gy, start, goal))

        for name, cell in (("출발", start), ("목적지", goal)):
            if not self.in_bounds(cell):
                self.get_logger().warn(
                    "%s 위치가 맵 밖입니다 %s — 아직 안 만들어진 영역입니다."
                    % (name, cell))
                self.last_replan_result = "GOAL_OUTSIDE_GRID"
                self.set_planner_status(
                    "NO_PATH", "%s 위치가 계획 격자 밖입니다." % name)
                return
        start = self.snap_to_safe(start, "출발")
        goal = self.snap_to_safe(goal, "목적지")
        if start is not None and goal is not None:
            goal = self.snap_to_reachable(start, goal)
        if start is None or goal is None:
            self.get_logger().warn(
                "출발 또는 목적지 주변에 통행 가능한 곳이 없습니다. "
                "safety_margin 을 줄이거나 로봇을 좀 옮겨보세요.")
            self.plan_pub.publish(Path(header=self._hdr()))
            self.last_replan_result = "NO_SAFE_ENDPOINT"
            self.set_planner_status(
                "NO_PATH", "출발 또는 목적지 주변에 통행 가능한 곳이 없습니다.")
            return

        safe_now = self.dynamic_safe()[0] if self.dyn_on else self.grid["safe"]
        cells = astar(safe_now, start, goal, self.grid.get("penalty"))
        if cells is None:
            self.get_logger().warn(
                "경로 없음 — 통로가 차체보다 좁거나 아직 지도가 이어지지 않았습니다. "
                "로봇을 조금 움직여 지도를 더 만들거나 safety_margin 을 줄이세요.")
            self.plan_pub.publish(Path(header=self._hdr()))
            self.last_replan_result = "NO_PATH"
            self.set_planner_status("NO_PATH", "목표까지 연결된 경로가 없습니다.")
            return

        self.emit_path(cells)

    def resume_if_halted(self):
        """장애물 때문에 세워 뒀다가 길이 열리면 원래 경로를 다시 보낸다."""
        if not self.halted or not self.last_path:
            return
        self.halted = False
        self.last_replan_result = "RESUMED"
        self.get_logger().info("길이 열렸습니다 — 원래 경로로 다시 이동합니다.")
        self.publish_points(self.last_path)

    def emit_path(self, cells):
        pts = [self.cell_to_world(r, c) for (r, c) in cells]
        pts = self.resample(self.thin(pts))
        self.publish_points(pts)

    def publish_points(self, pts):
        self.last_path = pts
        path = Path(header=self._hdr())
        length = 0.0
        for i, (x, y) in enumerate(pts):
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
            if i:
                length += math.hypot(x - pts[i - 1][0], y - pts[i - 1][1])
        self.plan_pub.publish(path)
        self.last_plan_length = length
        state = "DETOUR_ACTIVE" if self.blocked_state else "NAVIGATING"
        if self.blocked_state:
            self.last_replan_result = "DETOUR_FOUND"
        elif self.last_replan_result == "MANUAL_REPLAN_REQUEST":
            self.last_replan_result = "MANUAL_REPLAN_OK"
        else:
            self.last_replan_result = "PATH_FOUND"
        detail = (
            "동적 장애물을 우회하는 새 경로"
            if self.blocked_state else "목표 경로 추종 중"
        )
        self.set_planner_status(state, detail)
        self.get_logger().info("경로 생성: %d점, 길이 %.2f m" % (len(pts), length))

    def thin(self, pts):
        """Douglas-Peucker 폴리라인 단순화. tolerance 0 이면 그대로 둔다.

        예전 구현은 '인접 3점'만 보고 중간점을 버렸다. A* 결과는 1cm 격자라
        이웃 간 편차가 늘 0.005m 미만이어서 **모든 중간점이 삭제**되고
        시작-끝 2점만 남았다. 그러면 resample 이 그 사이를 직선으로 채워
        경로가 벽을 가로지르는 직선이 되어 버린다.
        전체 구간에서 가장 많이 벗어난 점을 기준으로 재귀 분할해야 한다.
        """
        if self.simplify <= 0.0 or len(pts) < 3:
            return pts

        def dp(a, b):
            # pts[a] 와 pts[b] 를 잇는 선에서 가장 멀리 벗어난 점을 찾는다
            ax, ay = pts[a]
            bx, by = pts[b]
            vx, vy = bx - ax, by - ay
            n = math.hypot(vx, vy)
            best_i, best_d = -1, 0.0
            for k in range(a + 1, b):
                px, py = pts[k]
                if n < 1e-9:
                    d = math.hypot(px - ax, py - ay)
                else:
                    d = abs(vx * (py - ay) - vy * (px - ax)) / n
                if d > best_d:
                    best_d, best_i = d, k
            if best_i < 0 or best_d <= self.simplify:
                return [a, b]
            left = dp(a, best_i)
            right = dp(best_i, b)
            return left[:-1] + right

        keep = dp(0, len(pts) - 1)
        return [pts[k] for k in keep]

    def resample(self, pts):
        """폴리라인을 일정 간격으로 다시 찍는다.

        thin() 을 거치면 직선 구간이 몇 점으로 줄어드는데(3m 를 4점 등),
        추종기는 경로점을 따라 걸어가며 lookahead 를 찾으므로 점이 희박하면
        전방주시 거리가 사실상 무의미해진다. 균일 간격으로 채워 준다.
        """
        if len(pts) < 2 or self.spacing <= 0.0:
            return pts
        out = [pts[0]]
        carry = 0.0
        for i in range(len(pts) - 1):
            ax, ay = pts[i]
            bx, by = pts[i + 1]
            seg = math.hypot(bx - ax, by - ay)
            if seg < 1e-9:
                continue
            t = self.spacing - carry
            while t <= seg:
                out.append((ax + (bx - ax) * t / seg, ay + (by - ay) * t / seg))
                t += self.spacing
            carry = (carry + seg) % self.spacing
        if math.hypot(out[-1][0] - pts[-1][0], out[-1][1] - pts[-1][1]) > 1e-6:
            out.append(pts[-1])
        return out

    def _hdr(self):
        h = Header()
        h.frame_id = self.grid["frame"] if self.grid else "map"
        h.stamp = self.get_clock().now().to_msg()
        return h

    # ================= RViz 확인용 금지영역 =================
    def publish_inflated(self):
        g = self.grid
        msg = OccupancyGrid()
        msg.header = self._hdr()
        msg.info.resolution = g["res"]
        msg.info.height, msg.info.width = g["shape"]
        msg.info.origin.position.x = g["ox"]
        msg.info.origin.position.y = g["oy"]
        msg.info.origin.orientation.z = math.sin(g["oyaw"] / 2.0)
        msg.info.origin.orientation.w = math.cos(g["oyaw"] / 2.0)
        grid = np.where(g["safe"], 0, 100).astype(np.int8)
        msg.data = grid.flatten().tolist()
        self.infl_pub.publish(msg)


def main():
    rclpy.init()
    node = GoalPathPlanner()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
