#!/usr/bin/env python3
"""
auto_align_node.py
----------------------------------------------------------
라이다 스캔을 기준 도면에 자동으로 맞춰서, start_x / start_y / start_yaw 를
사람이 재서 넣지 않아도 되게 한다.

왜 필요한가
  map 프레임의 원점은 '로봇이 SLAM 을 시작한 자리'다. 그 자리가 도면 위 어디에
  어느 방향으로 있는지는 start_x / start_y / start_yaw 로 알려줘야 하는데,
  이 값이 조금만 틀려도 RViz 에서 라이다 스캔이 도면 위에 비스듬히 얹힌다.
  경로계획도 벽 위치를 잘못 알게 된다.

  로봇을 놓을 때마다 align_check.py 를 돌려 값을 옮겨 적는 건 번거롭고,
  옮겨 적는 걸 깜빡하면 그대로 틀어진 채 주행한다.

무엇을 하는가
  1) 스캔 여러 장을 map 프레임으로 옮겨 점구름을 만든다
  2) 도면의 '벽 표면'까지 거리변환을 미리 계산한다
  3) start_x / start_y / start_yaw 를 격자탐색하며 점구름이 벽 표면에 가장 잘
     얹히는 조합을 찾는다 (로봇이 실제로 서 있을 수 있는 자리만 후보로 본다)
  4) 찾은 값을 path_planner_node 와 map_diff_node 의 파라미터로 **직접 넣는다**
     (두 노드 모두 매번 get_parameter 로 읽으므로 즉시 반영된다)

파라미터
  targets            값을 넣어 줄 노드 이름들 (기본 path_planner_node, map_diff_node)
  settle_sec         SLAM 이 안정될 때까지 기다리는 시간 (기본 3초)
  refine_sec         이 주기로 다시 재서 더 좋아지면 갱신 (0 이면 1회만, 기본 0)
  coarse_xy_step / coarse_yaw_step   1단계(조대) 격자 간격 — 전 범위를 성기게 훑는다
  xy_step / yaw_step                 2단계(정밀) 격자 간격 — 최종 해상도를 결정한다
  fine_span_xy / fine_span_yaw       정밀 단계가 조대 해 주변을 다시 볼 범위

실행 (보통은 auto_drive.launch.py 가 알아서 띄운다)
  python3 auto_align_node.py --ros-args -p settle_sec:=3.0
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

import cv2
import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSReliabilityPolicy,
    QoSProfile,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

sys.path.insert(0, os.path.expanduser("~/slam_test_maps"))
from slam_map_kit import read_pgm, parse_yaml  # noqa: E402

# 점이 벽에서 이보다 멀면 전부 똑같이 나쁜 것으로 친다 (이상치 둔감).
# ★ 이 값이 통로 폭에 비해 크면 점수가 포화돼 정렬을 못 가린다.
#   통로가 0.16 m 인 미로에서 0.12 로 두면, 20도쯤 틀어져도 점이 어차피 어떤 벽
#   12 cm 안에 들어가서 옳은 정렬과 틀린 정렬의 점수가 거의 같아진다.
#   실측(같은 스캔, 전 범위 탐색): 0.10 -> 오차 0.0440 / 0.05 -> 0.0279 /
#   0.03 -> 0.0205 / 0.02 -> 0.0150. 작을수록 최적점이 뾰족해진다.
#   너무 작으면 조대 단계에서 옳은 골짜기를 놓칠 수 있으므로 0.03 을 기본으로 둔다.
CLIP_DEFAULT = 0.03


def latched_qos():
    return QoSProfile(
        depth=1,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )


def yaw_from_quaternion(q):
    """Return yaw for a finite, non-zero quaternion, or raise ValueError."""
    values = (float(q.x), float(q.y), float(q.z), float(q.w))
    if not all(math.isfinite(value) for value in values):
        raise ValueError("자세 quaternion에 유한하지 않은 값이 있습니다")
    norm = math.sqrt(sum(value * value for value in values))
    if norm < 1e-9:
        raise ValueError("자세 quaternion이 0입니다")
    x, y, z, w = (value / norm for value in values)
    return math.atan2(2.0 * (w * z + x * y),
                      1.0 - 2.0 * (y * y + z * z))


def map_to_drawing(map_x, map_y, start_x, start_y, start_yaw_deg):
    """Apply Pd = R(start_yaw) * Pm + start_translation."""
    yaw = math.radians(start_yaw_deg)
    cosine, sine = math.cos(yaw), math.sin(yaw)
    return (
        cosine * map_x - sine * map_y + start_x,
        sine * map_x + cosine * map_y + start_y,
    )


def solve_manual_alignment(
        drawing_x, drawing_y, drawing_front_yaw,
        robot_map_x, robot_map_y, robot_base_yaw, heading_offset_deg):
    """Solve drawing<-map alignment that places the physical robot pose.

    ``drawing_*`` is the desired physical-front pose in raw reference-map
    coordinates. ``robot_*`` is the current map->base_link TF. The physical
    front in the SLAM map is base yaw plus ``heading_offset_deg``.
    """
    physical_front_yaw = robot_base_yaw + math.radians(heading_offset_deg)
    start_yaw = (drawing_front_yaw - physical_front_yaw) % (2.0 * math.pi)
    cosine, sine = math.cos(start_yaw), math.sin(start_yaw)
    start_x = drawing_x - (
        cosine * robot_map_x - sine * robot_map_y)
    start_y = drawing_y - (
        sine * robot_map_x + cosine * robot_map_y)
    return start_x, start_y, math.degrees(start_yaw) % 360.0


def quat_to_mat(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class AutoAlign(Node):

    def _num(self, name, default):
        self.declare_parameter(name, default,
                               ParameterDescriptor(dynamic_typing=True))
        v = self.get_parameter(name).value
        return float(default) if v is None else float(v)

    def _flag(self, name, default):
        self.declare_parameter(name, default,
                               ParameterDescriptor(dynamic_typing=True))
        v = self.get_parameter(name).value
        if v is None:
            return default
        if isinstance(v, bool):
            return v
        return str(v).strip().lower() in ("1", "true", "yes", "on")

    def _text(self, name, default):
        self.declare_parameter(name, default,
                               ParameterDescriptor(dynamic_typing=True))
        v = self.get_parameter(name).value
        return default if v is None else str(v)

    def __init__(self):
        super().__init__("auto_align_node")
        self.auto_enabled = self._flag("auto_align_enabled", True)
        ref = os.path.expanduser(self._text("reference", os.path.expanduser(
            "~/map_compare/custom_maps/room_shelf5.yaml")))
        self.targets = [t.strip() for t in
                        self._text("targets",
                                   "path_planner_node,map_diff_node").split(",")
                        if t.strip()]
        self.base_frame = self._text("base_frame", "base_link")
        self.map_frame = self._text("map_frame", "map")
        self.settle = self._num("settle_sec", 3.0)
        # ★ 한 번 맞추고 끝내면 안 된다.
        #   로봇이 완전히 정지해 있어도 cartographer 는 최적화할 때마다 pose 가
        #   튄다. 실측(정지 98초): x 16 mm / y 27 mm / yaw 2.29도 변동, 52초쯤에
        #   한 번에 y 17 mm + yaw 1.7도 점프. 그만큼 스캔이 map 안에서 통째로
        #   움직이므로, 처음에 맞춰 둔 도면은 시간이 지나면 반드시 어긋난다.
        #   그래서 계속 다시 재서 따라간다(추적 모드는 아래 track_* 참고).
        self.refine = self._num("refine_sec", 0.0)
        self.n_scans = int(self._num("scans", 6))
        self.yaw_step = self._num("yaw_step", 0.5)      # 정밀 단계 각도 해상도
        # 각도 탐색 범위[도]. 180 이면 0~360 전체를 본다(기본).
        # 작게 주면 start_yaw 주변만 본다 — 사용자가 정한 방향을 자동정렬이
        # 반대편 해로 뒤집어 버리는 것을 막을 때 쓴다. 예: yaw_range:=45
        self.yaw_range = self._num("yaw_range", 180.0)
        self.xy_range = self._num("xy_range", 0.30)
        self.xy_step = self._num("xy_step", 0.01)       # 정밀 단계 위치 해상도
        # 조대 단계 해상도 — 전 범위를 성기게 훑어 대략 위치를 잡는다
        self.coarse_xy = self._num("coarse_xy_step", 0.05)
        self.coarse_yaw = self._num("coarse_yaw_step", 5.0)
        # 점구름이 도면 안에 이만큼은 들어와야 후보로 인정한다.
        # 낮추면 엉뚱한 정렬도 답이 될 수 있으므로 함부로 내리지 말 것 —
        # 통과하는 후보가 없으면 아래에서 자동으로 한 번 완화해 재시도한다.
        self.min_cov = self._num("min_coverage", 0.85)
        self.clip = self._num("match_clip", CLIP_DEFAULT)
        # 벽 방향으로 yaw 후보를 좁히는 데 쓰는 값들
        self.wall_seg_max = self._num("wall_seg_max", 0.06)   # 이보다 벌어지면 다른 벽
        self.wall_share_min = self._num("wall_share_min", 0.45)  # 봉우리 집중도 하한
        self.wall_span = self._num("wall_span", 5.0)          # 후보 주변 +-이만큼만 본다
        # 3단계(초정밀). 2단계(1cm/0.5도)로 끝내면 격자 크기가 그대로 잔차로 남는다.
        # 실측(같은 스캔): 1cm/0.5도 -> 평균잔차 22.5mm, 20mm 이내 63%
        #                 2.5mm/0.1도 -> 평균잔차 16.4mm, 20mm 이내 81%
        self.ultra_xy = self._num("ultra_xy_step", 0.0025)
        self.ultra_yaw = self._num("ultra_yaw_step", 0.1)
        self.ultra_span_xy = self._num("ultra_span_xy", 0.02)
        self.ultra_span_yaw = self._num("ultra_span_yaw", 1.5)
        # 조대 단계에서 이만큼의 서로 떨어진 후보를 들고 내려가 전부 정밀화한다.
        # 1 로 두면 예전처럼 1등만 보고 빗살 한 칸 옆 별칭에 빠질 수 있다.
        self.topk = max(1, int(self._num("coarse_topk", 6)))
        # 추적 모드 — 한 번 맞춘 뒤에는 '어디인지 다시 찾을' 필요가 없다.
        # 직전 값 주변만 싸게 보면 되므로 전역 탐색(2~3초)이 아니라 0.1초면 된다.
        self.track_xy = self._num("track_xy_range", 0.06)
        self.track_yaw = self._num("track_yaw_range", 2.5)
        # ★ 갱신은 정지 중에만. 주행 중에 도면이 움직이면 경로 판정이 흔들린다.
        self.only_when_still = self._flag("align_only_when_still", True)
        self.move_lin = self._num("align_move_lin", 0.02)    # m/s
        self.move_ang = self._num("align_move_ang", 0.05)    # rad/s
        self.still_wait = self._num("align_still_wait", 0.8)  # 멈춘 뒤 대기[s]
        self.mv_prev = None
        self.mv_t = 0.0
        self.mv_moving = True
        self.still_from = None
        # 이만큼은 좋아져야 실제로 바꾼다(미세 변동으로 도면이 떨리는 것 방지)
        self.min_improve = self._num("align_min_improve", 0.003)
        # 이 오차를 연속으로 넘으면 전역 재정렬로 되돌린다
        self.realign_err = self._num("realign_error", 0.05)
        self.realign_hits = max(1, int(self._num("realign_hits", 3)))
        self.bad_ticks = 0
        # 서로 독립으로 이만큼 재서 결과가 일치할 때만 적용한다.
        # 1 로 두면 예전처럼 한 번 재고 바로 적용 (권장하지 않음).
        self.repeats = max(1, int(self._num("repeats", 3)))
        self.agree_xy = self._num("agree_xy", 0.05)     # 허용 위치 편차[m]
        self.agree_yaw = self._num("agree_yaw", 5.0)    # 허용 각도 편차[도]
        # 정밀 단계가 조대 해 주변을 다시 볼 범위
        self.fine_xy = self._num("fine_span_xy", 0.06)
        self.fine_yaw = self._num("fine_span_yaw", 6.0)
        self.robot_w = self._num("robot_width", 0.19)
        self.margin = self._num("safety_margin", 0.02)
        rr = self._num("robot_radius", 0.0)
        self.need = (rr if rr > 0.0 else 0.5 * self.robot_w) + self.margin
        # 수동 배치는 위치 보정이며 주행 명령이 아니다. 자동 정렬과 주행에서
        # 사용하는 차체 여유는 유지하되, 웹 배치 여유는 별도로 조절한다.
        # 0이면 기준 도면 내부의 모든 셀(벽 셀 포함)을 허용한다.
        self.manual_pose_min_clearance = max(
            0.0, self._num("manual_pose_min_clearance", self.need))
        # 초기 추정값 (탐색 중심)
        self.gx = self._num("start_x", 1.72)
        self.gy = self._num("start_y", 1.39)
        self.gyaw = self._num("start_yaw", 356.0)
        # 사용자가 로봇을 놓은 '의도한' 방향. 측정값과의 차이가 곧 라이다 장착 오차다.
        self.nominal_yaw = self.gyaw
        self.lidar_yaw = self._num("lidar_yaw", 0.0)
        # Follower와 웹이 모두 '실제 차체 전방 = TF yaw + offset'을 쓴다.
        self.heading_offset = self._num("heading_offset", 0.0)
        self.manual_pose_topic = self._text(
            "manual_pose_topic", "/web/set_robot_pose")
        self.manual_timeout = max(
            0.5, self._num("manual_pose_timeout_sec", 5.0))
        self.manual_apply_timeout = max(
            1.0, self._num("manual_apply_timeout_sec", 5.0))
        self.manual_service_timeout = max(
            0.2, self._num("manual_service_timeout_sec", 1.5))
        self.manual_rollback_timeout = max(
            0.5, self._num("manual_rollback_timeout_sec", 3.0))

        occ, self.res = self._load(ref)
        self.h, self.w = occ.shape

        # ★ 도면 대각선보다 먼 점은 버린다.
        #   로봇이 도면 안에 있는 한, 도면 대각선보다 먼 점은 기하학적으로
        #   절대 도면 안에 들어올 수 없다. 그런데 방이 도면보다 넓으면
        #   (미로가 트인 쪽으로 라이다가 바깥 방까지 본다) 그런 점이 수백 개씩
        #   생기고, 전부 '도면 밖'으로 세어져 coverage 를 깎아먹는다.
        #   실측: 스캔이 5.4 m 까지 나가서 1587점 중 251점(16%)이 도면 밖 →
        #   최대 coverage 82% → min_coverage 0.85 에 아무 후보도 못 들어
        #   "정렬을 못 찾았습니다" 로 끝나고 도면이 어긋난 채로 남았다.
        #   잘라내면 최대 coverage 98%, 정렬오차 0.0343 -> 0.0220 m.
        diag = math.hypot(self.w * self.res, self.h * self.res)
        mr = self._num("max_point_range", 0.0)
        self.max_range = mr if mr > 0.0 else diag
        free = (occ == 0).astype(np.uint8)
        k = np.ones((3, 3), np.uint8)
        surface = (occ.astype(np.uint8) & cv2.dilate(free, k)).astype(np.uint8)
        # 라이다가 실제로 보는 것은 '자유공간에 접한 벽 표면' 이다.
        # 벽 내부까지의 거리를 쓰면 벽이 두꺼울 때 지표가 포화된다.
        self.dsurf = cv2.distanceTransform(1 - surface, cv2.DIST_L2, 5) * self.res
        self.clear = cv2.distanceTransform(free, cv2.DIST_L2, 5) * self.res

        self.pts = []
        self.meas = []               # 아직 합의를 못 본 측정들
        self.wall_hist = np.zeros(90, dtype=np.float64)   # 벽 방향 0~90도, 1도 칸
        self.last_max_cov = 0.0
        self.best = None
        self.applied = False
        self.manual_lock = False
        self.pending_manual = None
        self.manual_txn = None
        self._param_clients = {}
        self._get_param_clients = {}
        self.t0 = self.now()

        self.buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buf, self)
        self.create_subscription(LaserScan, "/scan", self.on_scan,
                                 qos_profile_sensor_data)
        self.create_subscription(
            PoseStamped, self.manual_pose_topic, self.on_manual_pose, 10)
        self.create_timer(0.5, self.tick)
        self.create_timer(0.1, self._manual_timer)
        self.status_pub = self.create_publisher(
            String, "/alignment/status", latched_qos())
        state = "WAITING" if self.auto_enabled else "MANUAL_READY"
        detail = (
            "자동 정렬 대기; 웹 수동 배치 수신 가능"
            if self.auto_enabled else
            "자동 정렬 비활성화; 웹 수동 배치 대기"
        )
        self.publish_status(state, detail)

        self.get_logger().info(
            "도면 정렬 자동측정 대기 (%.0f초 안정 후 시작) | 도면 %dx%d @ %.3f m | "
            "로봇 시작 자리는 벽에서 %.3f m 이상 떨어져야 함 | "
            "%.2f m 보다 먼 스캔점은 도면 밖이라 버림"
            % (self.settle, self.w, self.h, self.res, self.need, self.max_range))

    @staticmethod
    def _load(yaml_path):
        meta = parse_yaml(yaml_path)
        res = float(meta["resolution"])
        occ_t = float(meta.get("occupied_thresh", 0.65))
        pgm = os.path.join(os.path.dirname(os.path.abspath(yaml_path)),
                           os.path.basename(meta["image"]))
        w, h, _mx, px = read_pgm(pgm)
        img = np.frombuffer(bytes(px), dtype=np.uint8).reshape(h, w)
        p = (255.0 - img.astype(np.float32)) / 255.0
        occ = np.zeros((h, w), dtype=np.uint8)
        occ[p > occ_t] = 1
        return np.flipud(occ), res

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    # ---------------- 스캔 수집 ----------------
    def on_scan(self, msg: LaserScan):
        if (not self.auto_enabled or self.manual_lock
                or self.pending_manual or self.manual_txn):
            return
        if self.now() - self.t0 < self.settle:
            return
        if len(self.pts) >= self.n_scans:
            return
        try:
            tr = self.buf.lookup_transform(self.map_frame, msg.header.frame_id,
                                           rclpy.time.Time())
        except tf2_ros.TransformException:
            return
        R = quat_to_mat(tr.transform.rotation)
        t = np.array([tr.transform.translation.x, tr.transform.translation.y,
                      tr.transform.translation.z])
        r = np.asarray(msg.ranges, dtype=np.float64)
        a = msg.angle_min + np.arange(r.size) * msg.angle_increment
        ok = (np.isfinite(r) & (r > max(msg.range_min, 0.03))
              & (r < (msg.range_max if msg.range_max > 0 else 12.0))
              & (r <= self.max_range))   # 도면 밖까지 본 점은 정렬에 쓸 수 없다
        if int(ok.sum()) < 40:
            return
        local = np.stack([r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok]),
                          np.zeros(int(ok.sum()))], axis=1)
        pm = (local @ R.T + t)[:, :2]
        self.pts.append(pm)
        self._accum_wall(pm)

    # ---------------- 벽 방향 ----------------
    def _accum_wall(self, p):
        """스캔의 국소 벽 방향을 0~90도 히스토그램에 쌓는다.

        스캔은 각도 순서로 들어오므로 이웃한 점이 곧 같은 벽면 위의 점이다.
        그래서 KD-tree 없이 연속 3점의 방향만 보면 된다 (O(n)).
        벽을 건너뛴 구간은 두 점 간격이 크므로 wall_seg_max 로 걸러낸다.
        """
        if len(p) < 5:
            return
        d = p[2:] - p[:-2]
        seg = np.hypot(d[:, 0], d[:, 1])
        ok = (seg > 1e-6) & (seg < self.wall_seg_max)
        if not ok.any():
            return
        ang = np.degrees(np.arctan2(d[ok, 1], d[ok, 0])) % 90.0
        np.add.at(self.wall_hist, np.clip(ang.astype(np.int32), 0, 89), 1.0)

    def wall_peak(self):
        """지배적인 벽 방향[도, 0~90)과 그 봉우리에 몰린 비율을 돌려준다."""
        h = self.wall_hist
        if h.sum() < 200.0:
            return None, 0.0
        k = np.array([1.0, 2.0, 3.0, 2.0, 1.0])
        k /= k.sum()
        s = np.convolve(np.concatenate([h[-2:], h, h[:2]]), k, "valid")
        pk = int(np.argmax(s))
        win = [(pk + d) % 90 for d in range(-7, 8)]
        return pk, float(s[win].sum() / s.sum())

    def yaw_candidates(self):
        """벽 방향으로 start_yaw 후보를 4개로 좁힌다.

        도면은 map 프레임에 R(-yaw) 로 그려지므로, 도면의 직각 벽은 map 에서
        (-yaw) mod 90 방향으로 보인다. 스캔의 지배적 벽 방향이 w 라면
        yaw = -w (mod 90) 이고, 후보는 90도 간격으로 딱 4개다.

        이게 이 노드의 핵심이다. 이 미로는 같은 빗살이 반복돼서 0~360 을 다 훑으면
        점수가 거의 같은 별칭 해가 잔뜩 나오고, 잴 때마다 다른 것에 걸린다
        (실측: yaw 가 341 / 0 / 5.5 / 1.5 로 튀는데 오차는 전부 0.035~0.040).
        벽 방향은 그런 별칭에 흔들리지 않으므로 먼저 각도를 못 박는다.
        """
        pk, share = self.wall_peak()
        if pk is None or share < self.wall_share_min:
            return None, pk, share
        return [(-pk + 90.0 * k) % 360.0 for k in range(4)], pk, share

    # ---------------- 점수 ----------------
    def score(self, pm, sx, sy, yaw):
        c, s = math.cos(yaw), math.sin(yaw)
        dx = c * pm[:, 0] - s * pm[:, 1] + sx      # Pd = R(yaw)*Pm + S
        dy = s * pm[:, 0] + c * pm[:, 1] + sy
        col = (dx / self.res).astype(np.int32)
        row = (dy / self.res).astype(np.int32)
        inside = ((row >= 0) & (row < self.h) & (col >= 0) & (col < self.w))
        n = int(inside.sum())
        if n < 30:
            return 1e9, 0.0
        d = np.minimum(self.dsurf[row[inside], col[inside]], self.clip)
        return (float(d.sum()) + self.clip * (len(pm) - n)) / len(pm), n / len(pm)

    # ---------------- 탐색 + 적용 ----------------
    def _search(self, pm, cx, cy, xy_rad, xy_st, yaw_rad, yaw_st, yaw_center,
                cov_gate=None, topk=1):
        """(cx,cy) 주변 xy_rad 안, yaw_center 주변 yaw_rad 안을 격자탐색.

        yaw_rad >= 180 이면 0~360 전체를 본다.
        cov_gate 를 넘긴 후보 중 점수가 가장 낮은 것을 고른다.
        통과한 후보가 없어도 판단 재료가 남도록 실제로 본 최대 coverage 를
        self.last_max_cov 에 남긴다.

        topk > 1 이면 서로 떨어진 상위 후보를 그만큼 목록으로 돌려준다.
        반환: topk==1 이면 (score, sx, sy, yaw_deg, coverage) 또는 None,
              topk>1 이면 그 튜플들의 리스트(점수 오름차순, 비어 있을 수 있음)
        """
        if cov_gate is None:
            cov_gate = self.min_cov
        self.last_max_cov = 0.0
        offs = np.arange(-xy_rad, xy_rad + 1e-9, xy_st)
        if yaw_rad >= 180.0:
            yaws = np.arange(0.0, 360.0, yaw_st)
        else:
            yaws = yaw_center + np.arange(-yaw_rad, yaw_rad + 1e-9, yaw_st)
        top = []
        for yd in yaws:
            yaw = math.radians(yd)
            for ddx in offs:
                for ddy in offs:
                    sx, sy = cx + ddx, cy + ddy
                    r0, c0 = int(sy / self.res), int(sx / self.res)
                    # 로봇이 실제로 서 있을 수 없는 자리는 답이 될 수 없다
                    if not (0 <= r0 < self.h and 0 <= c0 < self.w):
                        continue
                    if self.clear[r0, c0] < self.need:
                        continue
                    sc, cov = self.score(pm, sx, sy, yaw)
                    if cov > self.last_max_cov:
                        self.last_max_cov = cov
                    if cov < cov_gate:
                        continue
                    self._push(top, (sc, sx, sy, yd % 360.0, cov), topk)
        if topk > 1:
            return top
        return top[0] if top else None

    @staticmethod
    def _push(top, cand, topk, nms_xy=0.12, nms_yaw=8.0):
        """상위 topk 후보 목록에 넣되, 서로 가까운 것은 한 곳으로 친다.

        억제하지 않으면 최적점 하나 주변의 격자점들이 목록을 다 차지해서,
        정작 멀리 떨어진 '다른 골짜기'가 후보에 못 든다. 이 미로에서는
        빗살 한 칸(0.31 m) 옆이 바로 그 다른 골짜기다.
        """
        for i, c in enumerate(top):
            if (math.hypot(cand[1] - c[1], cand[2] - c[2]) < nms_xy
                    and abs((cand[3] - c[3] + 180.0) % 360.0 - 180.0) < nms_yaw):
                if cand[0] < c[0]:
                    top[i] = cand
                    top.sort(key=lambda x: x[0])
                return
        top.append(cand)
        top.sort(key=lambda x: x[0])
        del top[topk:]

    def measure(self, pm):
        """점구름 하나로 정렬을 한 번 잰다. (score, x, y, yaw_deg, cov) 또는 None.

        조대 -> 정밀 -> 초정밀 3단계. 전 범위를 최고 해상도로 훑으면 후보가
        17만개나 되어 느리면서도 정확도는 격자 크기에 묶인다.

        ★ 조대 단계에서 1등 하나만 들고 내려가면 안 된다. 이 미로는 빗살 한 칸
          (0.31 m) 옆에 점수가 비슷한 다른 골짜기가 있어서, 잡음에 따라 엉뚱한
          쪽이 1등이 되고 정밀 단계는 거기서 못 빠져나온다. 실측에서 같은 자리를
          연속으로 재는데 x 가 1.56 과 1.87 로 갈렸다 (정답은 정밀화 후 오차가
          0.012 로 더 낮은 1.87 쪽). 그래서 상위 후보 여러 개를 전부 정밀화한 뒤
          마지막에 비교한다.
        """
        t_start = self.now()
        gate = self.min_cov

        # 추적 모드 — 이미 한 번 맞춰 놨으면 그 주변만 보면 된다. 전역 탐색을
        # 다시 하면 빗살 한 칸 옆 별칭으로 튈 위험까지 생긴다.
        if self.applied:
            seed = self._search(pm, self.gx, self.gy,
                                self.track_xy, self.xy_step,
                                self.track_yaw, self.yaw_step, self.gyaw,
                                cov_gate=gate)
            if seed is None and self.last_max_cov > 0.0:
                seed = self._search(pm, self.gx, self.gy,
                                    self.track_xy, self.xy_step,
                                    self.track_yaw, self.yaw_step, self.gyaw,
                                    cov_gate=max(0.50, self.last_max_cov * 0.95))
            if seed is not None:
                ultra = self._search(pm, seed[1], seed[2],
                                     self.ultra_span_xy, self.ultra_xy,
                                     self.ultra_span_yaw, self.ultra_yaw, seed[3],
                                     cov_gate=gate)
                if ultra is not None and ultra[0] <= seed[0]:
                    seed = ultra
            self.last_search_sec = self.now() - t_start
            return seed

        seeds = []
        cands, pk, share = self.yaw_candidates()
        if cands is not None:
            # 벽 방향으로 각도가 4개로 좁혀졌다. 각 후보 주변만 본다.
            self.get_logger().info(
                "벽 방향 %d도 (점의 %.0f%%가 이 방향) -> yaw 후보 %s"
                % (pk, share * 100, ", ".join("%.0f" % c for c in sorted(cands))))
            for c in cands:
                seeds += self._search(pm, self.gx, self.gy,
                                      self.xy_range, self.coarse_xy,
                                      self.wall_span, self.coarse_yaw, c,
                                      topk=self.topk)
        else:
            if pk is not None:
                self.get_logger().warn(
                    "벽 방향이 뚜렷하지 않습니다 (봉우리 집중도 %.0f%% < %.0f%%) — "
                    "각도 전 범위를 훑습니다. 정렬이 실행마다 흔들릴 수 있습니다."
                    % (share * 100, self.wall_share_min * 100))
            seeds = self._search(pm, self.gx, self.gy,
                                 self.xy_range, self.coarse_xy,
                                 self.yaw_range, self.coarse_yaw,
                                 self.nominal_yaw, topk=self.topk)
        if not seeds and self.last_max_cov > 0.0:
            # 아무 후보도 게이트를 못 넘었다. 여기서 포기해 버리면 도면이
            # 어긋난 채로 그냥 남는다(예전에 실제로 그랬다). 실제로 닿을 수 있는
            # coverage 를 기준으로 한 번만 완화해서 다시 본다.
            gate = max(0.50, self.last_max_cov * 0.95)
            self.get_logger().warn(
                "도면 안에 들어온 점이 최대 %.0f%% 뿐이라 기준 %.0f%% 를 넘는 후보가 "
                "없습니다. 기준을 %.0f%% 로 낮춰 다시 맞춥니다. "
                "(방이 도면보다 넓거나 도면이 실제와 다를 수 있습니다)"
                % (self.last_max_cov * 100, self.min_cov * 100, gate * 100))
            # 완화해서 다시 볼 때도 각도 후보는 그대로 유지한다
            for c in (cands if cands is not None else [self.nominal_yaw]):
                span = self.wall_span if cands is not None else self.yaw_range
                seeds += self._search(pm, self.gx, self.gy,
                                      self.xy_range, self.coarse_xy,
                                      span, self.coarse_yaw, c,
                                      cov_gate=gate, topk=self.topk)

        # 여러 yaw 후보에서 모인 씨앗을 한 번 더 억제해 서로 떨어진 것만 남긴다
        merged = []
        for s in sorted(seeds, key=lambda x: x[0]):
            self._push(merged, s, self.topk)

        # 2단계(정밀)는 씨앗 전부에 돌린다 — 별칭끼리 제대로 줄을 세워야 하기
        # 때문이다. 3단계(초정밀)는 이긴 하나에만 돌린다. 정밀 단계만으로도
        # 별칭 구분은 충분히 되고(실측 0.011 vs 0.016), 초정밀은 비싸다.
        best = None
        for seed in merged:
            fine = self._search(pm, seed[1], seed[2],
                                self.fine_xy, self.xy_step,
                                self.fine_yaw, self.yaw_step, seed[3],
                                cov_gate=gate)
            cur = fine if (fine is not None and fine[0] <= seed[0]) else seed
            if best is None or cur[0] < best[0]:
                best = cur
        if best is not None:
            ultra = self._search(pm, best[1], best[2],
                                 self.ultra_span_xy, self.ultra_xy,
                                 self.ultra_span_yaw, self.ultra_yaw, best[3],
                                 cov_gate=gate)
            if ultra is not None and ultra[0] <= best[0]:
                best = ultra
        self.last_search_sec = self.now() - t_start
        return best

    def robot_pose(self):
        """Current (x, y, base yaw) in map, or None without a valid TF."""
        try:
            transform = self.buf.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time())
        except tf2_ros.TransformException:
            return None
        translation = transform.transform.translation
        try:
            yaw = yaw_from_quaternion(transform.transform.rotation)
        except ValueError:
            return None
        pose = (float(translation.x), float(translation.y), float(yaw))
        return pose if all(math.isfinite(value) for value in pose) else None

    def robot_moving(self, pose=None):
        """로봇이 지금 움직이고 있는가. TF 변화만 본다(cmd_vel 에 안 기댄다).

        ★ 주행 중에 정렬을 갱신하면 도면이 로봇 발밑에서 통째로 움직인다.
          목적지(map 좌표)와 이미 발행한 경로는 그대로인데 벽만 옮겨지므로,
          멀쩡하던 경로가 갑자기 '막힘'으로 판정돼 재계획이 쏟아진다.
          (사용자 관찰: "계속 맵이 바뀌면서 로봇이 헷갈려한다")
          그래서 **정지 중에만** 갱신한다. 그때는 도면이 움직여도 로봇이
          따라갈 경로가 없어 혼란이 없다.
        """
        if pose is None:
            pose = self.robot_pose()
        if pose is None:
            return True                    # 모르면 움직이는 것으로 본다(보수적)
        now = self.now()
        cur = pose
        if self.mv_prev is None:
            self.mv_prev, self.mv_t = cur, now
            return True
        dt = now - self.mv_t
        if dt < 0.2:
            return self.mv_moving
        d = math.hypot(cur[0] - self.mv_prev[0], cur[1] - self.mv_prev[1])
        dy = abs((cur[2] - self.mv_prev[2] + math.pi) % (2 * math.pi) - math.pi)
        self.mv_prev, self.mv_t = cur, now
        moving = (d / dt > self.move_lin) or (dy / dt > self.move_ang)
        if moving:
            self.still_from = None
        elif self.still_from is None:
            self.still_from = now
        # 멈춘 직후에는 관성/진동이 남아 있으므로 조금 기다린다
        self.mv_moving = moving or (self.still_from is None) or (
            now - self.still_from < self.still_wait)
        return self.mv_moving

    def _request_id(self, message):
        stamp = message.header.stamp
        stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        return (str(stamp_ns) if stamp_ns > 0 else
                "manual-%d" % self.get_clock().now().nanoseconds)

    def publish_status(self, state, detail, **values):
        payload = {"state": state, "detail": detail, **values}
        self.status_pub.publish(
            String(data=json.dumps(payload, ensure_ascii=False)))

    def _reject_manual(self, request_id, detail, **values):
        self.get_logger().error("웹 수동 배치 거부: %s" % detail)
        self.publish_status(
            "REJECTED", detail, source="web_manual_pose",
            request_id=request_id, **values)

    def on_manual_pose(self, message: PoseStamped):
        """Queue an authoritative physical-front pose.

        ``map`` means a pose on the currently displayed map (legacy contract),
        while ``reference_map_raw`` is already in raw drawing coordinates and
        avoids a display-refresh race after an automatic alignment update.
        """
        request_id = self._request_id(message)
        if self.pending_manual is not None or self.manual_txn is not None:
            detail = "이전 수동 pose 요청을 처리 중입니다"
            self.get_logger().warn("웹 수동 배치 BUSY: %s" % request_id)
            self.publish_status(
                "BUSY", detail, source="web_manual_pose",
                request_id=request_id)
            return

        frame_id = str(message.header.frame_id or "").lstrip("/")
        expected_frame = str(self.map_frame).lstrip("/")
        if frame_id not in (expected_frame, "reference_map_raw"):
            self._reject_manual(
                request_id,
                "frame_id '%s'는 지원하지 않습니다('%s' 또는 "
                "'reference_map_raw' 필요)"
                % (message.header.frame_id, self.map_frame))
            return

        input_x = float(message.pose.position.x)
        input_y = float(message.pose.position.y)
        if not math.isfinite(input_x) or not math.isfinite(input_y):
            self._reject_manual(request_id, "위치가 유한한 숫자가 아닙니다")
            return
        try:
            input_front_yaw = yaw_from_quaternion(message.pose.orientation)
        except ValueError as error:
            self._reject_manual(request_id, str(error))
            return

        if frame_id == "reference_map_raw":
            drawing_x, drawing_y = input_x, input_y
            drawing_front_yaw = input_front_yaw % (2.0 * math.pi)
        else:
            drawing_x, drawing_y = map_to_drawing(
                input_x, input_y, self.gx, self.gy, self.gyaw)
            drawing_front_yaw = (
                math.radians(self.gyaw) + input_front_yaw) % (2.0 * math.pi)

        # Reject clicks on/outside the raw reference drawing. This tests the
        # desired robot centre, not the old displayed map cell.
        column = int(math.floor(drawing_x / self.res))
        row = int(math.floor(drawing_y / self.res))
        if not (0 <= row < self.h and 0 <= column < self.w):
            self._reject_manual(
                request_id, "선택 위치가 기준 도면 밖입니다",
                drawing_x=drawing_x, drawing_y=drawing_y)
            return
        clearance = float(self.clear[row, column])
        if (self.manual_pose_min_clearance > 0.0
                and clearance < self.manual_pose_min_clearance):
            self._reject_manual(
                request_id,
                "선택 위치의 벽 여유 %.3f m가 필요치 %.3f m보다 작습니다"
                % (clearance, self.manual_pose_min_clearance),
                drawing_x=drawing_x, drawing_y=drawing_y,
                clearance=clearance,
                required_clearance=self.manual_pose_min_clearance)
            return

        received_at = self.now()
        self.pending_manual = {
            "request_id": request_id,
            "received_at": received_at,
            # Even if stale motion state said "still", never commit before a
            # complete post-cancel still interval has elapsed and been sampled.
            "not_before": received_at + self.still_wait,
            "input_frame": frame_id,
            "input_x": input_x,
            "input_y": input_y,
            "input_front_yaw": input_front_yaw,
            "drawing_x": drawing_x,
            "drawing_y": drawing_y,
            "drawing_front_yaw": drawing_front_yaw,
        }
        # Every request, including a second one while manual_lock is active,
        # must prove a fresh still interval after the backend's cancel/STOP.
        self.mv_prev = None
        self.mv_moving = True
        self.still_from = None
        self.pts = []
        self.meas = []
        self.publish_status(
            "MANUAL_PENDING", "로봇 정지 및 map->base_link TF 확인 중",
            source="web_manual_pose", request_id=request_id,
            requested_pose={
                "frame_id": frame_id, "x": input_x, "y": input_y,
                "front_yaw_deg": math.degrees(input_front_yaw),
            },
            desired_drawing={
                "x": drawing_x, "y": drawing_y,
                "front_yaw_deg": math.degrees(drawing_front_yaw) % 360.0,
            })
        self._try_pending_manual()

    def _try_pending_manual(self):
        request = self.pending_manual
        if request is None:
            return False
        if self.now() - request["received_at"] > self.manual_timeout:
            self.pending_manual = None
            self._reject_manual(
                request["request_id"],
                "%.1f초 안에 정지된 map->base_link TF를 확인하지 못했습니다"
                % self.manual_timeout)
            return False

        now = self.now()
        robot_pose = self.robot_pose()
        moving = True if robot_pose is None else self.robot_moving(robot_pose)
        if now < request["not_before"] or moving:
            return False

        sx, sy, yaw_deg = solve_manual_alignment(
            request["drawing_x"], request["drawing_y"],
            request["drawing_front_yaw"],
            robot_pose[0], robot_pose[1], robot_pose[2],
            self.heading_offset)
        if not all(math.isfinite(value) for value in (sx, sy, yaw_deg)):
            self.pending_manual = None
            self._reject_manual(
                request["request_id"], "정렬 계산 결과가 유한하지 않습니다")
            return False

        self.pending_manual = None
        self._start_manual_transaction(request, robot_pose, (sx, sy, yaw_deg))
        return True

    @staticmethod
    def _target_basename(target):
        return str(target).strip("/").split("/")[-1]

    def _is_map_diff_target(self, target):
        return self._target_basename(target) == "map_diff_node"

    def _is_planner_target(self, target):
        return "planner" in self._target_basename(target)

    def _manual_clients(self, target):
        clean = str(target).strip("/")
        set_client = self._param_clients.get(target)
        if set_client is None:
            set_client = self.create_client(
                SetParameters, "/%s/set_parameters" % clean)
            self._param_clients[target] = set_client
        get_client = self._get_param_clients.get(target)
        if get_client is None:
            get_client = self.create_client(
                GetParameters, "/%s/get_parameters" % clean)
            self._get_param_clients[target] = get_client
        return get_client, set_client

    def _start_manual_transaction(self, request, robot_pose, new_values):
        now = self.now()
        self.manual_txn = {
            "request": request,
            "robot_pose": robot_pose,
            "new_values": tuple(float(value) for value in new_values),
            "old_local": (self.gx, self.gy, self.gyaw),
            "old_lock": self.manual_lock,
            "old_applied": self.applied,
            "phase": "discover",
            "deadline": now + self.manual_apply_timeout,
            "future": None,
            "targets": [],
            "old_values": {},
            "get_index": 0,
            "set_index": 0,
            "applied_targets": [],
            "rollback_errors": [],
        }
        self.publish_status(
            "MANUAL_APPLYING", "필수 파라미터 서비스 확인 중",
            source="web_manual_pose", request_id=request["request_id"])
        self._manual_discover_targets()

    def _manual_discover_targets(self):
        transaction = self.manual_txn
        if transaction is None or transaction["phase"] != "discover":
            return
        ready = []
        for target in self.targets:
            get_client, set_client = self._manual_clients(target)
            if get_client.service_is_ready() and set_client.service_is_ready():
                ready.append(target)
        map_diff = [target for target in ready
                    if self._is_map_diff_target(target)]
        if not map_diff:
            return

        # Every currently active target participates.  map_diff is deliberately
        # last so the visible drawing changes only after planners accepted.
        transaction["targets"] = [
            target for target in ready if not self._is_map_diff_target(target)
        ] + map_diff
        transaction["phase"] = "get"
        self.publish_status(
            "MANUAL_APPLYING", "현재 파라미터 스냅샷 중",
            source="web_manual_pose",
            request_id=transaction["request"]["request_id"],
            targets=transaction["targets"])
        self._manual_start_get()

    @staticmethod
    def _parameter_number(value):
        if value.type == ParameterType.PARAMETER_DOUBLE:
            return float(value.double_value)
        if value.type == ParameterType.PARAMETER_INTEGER:
            return float(value.integer_value)
        raise ValueError("숫자 파라미터가 아닙니다(type=%s)" % value.type)

    def _manual_set_request(self, values):
        request = SetParameters.Request()
        for name, value in zip(
                ("start_x", "start_y", "start_yaw"), values):
            parameter = Parameter()
            parameter.name = name
            parameter.value = ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE,
                double_value=float(value))
            request.parameters.append(parameter)
        return request

    def _manual_track_future(self, phase, target, future, callback,
                             rollback=False):
        transaction = self.manual_txn
        if transaction is None:
            return
        transaction["phase"] = phase
        transaction["current_target"] = target
        transaction["future"] = future
        limit = (transaction.get("rollback_deadline") if rollback
                 else transaction["deadline"])
        transaction["future_deadline"] = min(
            self.now() + self.manual_service_timeout, limit)
        future.add_done_callback(
            lambda done, active=transaction: callback(active, target, done))

    def _manual_start_get(self):
        transaction = self.manual_txn
        if transaction is None:
            return
        index = transaction["get_index"]
        if index >= len(transaction["targets"]):
            transaction["phase"] = "set"
            self._manual_start_set()
            return
        target = transaction["targets"][index]
        get_client, _set_client = self._manual_clients(target)
        request = GetParameters.Request()
        request.names = ["start_x", "start_y", "start_yaw"]
        try:
            future = get_client.call_async(request)
        except Exception as error:  # noqa: BLE001
            self._manual_reject_transaction(
                "%s 기존 파라미터 조회 실패: %s" % (target, error))
            return
        self._manual_track_future(
            "get", target, future, self._manual_get_done)

    def _manual_get_done(self, transaction, target, future):
        if (self.manual_txn is not transaction
                or transaction.get("future") is not future
                or transaction.get("phase") != "get"):
            return
        transaction["future"] = None
        try:
            response = future.result()
            values = [] if response is None else list(response.values)
            if (not self._is_map_diff_target(target)
                    and (not values or (
                        len(values) == 3
                        and all(value.type == ParameterType.PARAMETER_NOT_SET
                                for value in values)))):
                # live planner(source=map) intentionally has no drawing
                # alignment parameters. It is active but not a transaction
                # participant; rclpy may return either three NOT_SET values or
                # an empty response for undeclared parameters.
                transaction["targets"].pop(transaction["get_index"])
                self._manual_start_get()
                return
            if len(values) != 3:
                raise ValueError("응답 개수가 3이 아닙니다")
            old_values = tuple(
                self._parameter_number(value) for value in values)
        except Exception as error:  # noqa: BLE001
            self._manual_reject_transaction(
                "%s 기존 파라미터 조회 실패: %s" % (target, error))
            return
        transaction["old_values"][target] = old_values
        transaction["get_index"] += 1
        self._manual_start_get()

    @staticmethod
    def _set_response_error(response):
        if response is None or len(response.results) != 3:
            return "응답 개수가 3이 아닙니다"
        failed = [result.reason or "reason 없음" for result in response.results
                  if not result.successful]
        return "; ".join(failed) if failed else None

    def _manual_start_set(self):
        transaction = self.manual_txn
        if transaction is None:
            return
        index = transaction["set_index"]
        if index >= len(transaction["targets"]):
            self._manual_commit(transaction)
            return
        target = transaction["targets"][index]
        _get_client, set_client = self._manual_clients(target)
        try:
            future = set_client.call_async(
                self._manual_set_request(transaction["new_values"]))
        except Exception as error:  # noqa: BLE001
            self._manual_begin_rollback(
                "%s 파라미터 적용 실패: %s" % (target, error), target)
            return
        self._manual_track_future(
            "set", target, future, self._manual_set_done)

    def _manual_set_done(self, transaction, target, future):
        if (self.manual_txn is not transaction
                or transaction.get("future") is not future
                or transaction.get("phase") != "set"):
            return
        transaction["future"] = None
        try:
            error = self._set_response_error(future.result())
        except Exception as exception:  # noqa: BLE001
            error = str(exception)
        if error:
            self._manual_begin_rollback(
                "%s 파라미터 적용 거부: %s" % (target, error), target)
            return
        transaction["applied_targets"].append(target)
        transaction["set_index"] += 1
        self._manual_start_set()

    def _manual_begin_rollback(self, reason, uncertain_target=None):
        transaction = self.manual_txn
        if transaction is None:
            return
        transaction["failure"] = reason
        changed = list(transaction["applied_targets"])
        # A timed-out/failed SetParameters response may still have partially
        # applied, so always roll that target back too.
        if uncertain_target is not None and uncertain_target not in changed:
            changed.append(uncertain_target)
        transaction["rollback_targets"] = list(reversed(changed))
        transaction["rollback_index"] = 0
        transaction["rollback_deadline"] = (
            self.now() + self.manual_rollback_timeout)
        transaction["future"] = None
        transaction["phase"] = "rollback"
        self._manual_start_rollback()

    def _manual_start_rollback(self):
        transaction = self.manual_txn
        if transaction is None:
            return
        index = transaction["rollback_index"]
        targets = transaction["rollback_targets"]
        if index >= len(targets):
            self._manual_reject_transaction(
                transaction["failure"],
                rollback_attempted=targets,
                rollback_errors=transaction["rollback_errors"])
            return
        target = targets[index]
        _get_client, set_client = self._manual_clients(target)
        if not set_client.service_is_ready():
            transaction["rollback_errors"].append(
                "%s rollback 서비스 없음" % target)
            transaction["rollback_index"] += 1
            self._manual_start_rollback()
            return
        try:
            future = set_client.call_async(
                self._manual_set_request(transaction["old_values"][target]))
        except Exception as error:  # noqa: BLE001
            transaction["rollback_errors"].append(
                "%s rollback 요청 실패: %s" % (target, error))
            transaction["rollback_index"] += 1
            self._manual_start_rollback()
            return
        self._manual_track_future(
            "rollback", target, future, self._manual_rollback_done,
            rollback=True)

    def _manual_rollback_done(self, transaction, target, future):
        if (self.manual_txn is not transaction
                or transaction.get("future") is not future
                or transaction.get("phase") != "rollback"):
            return
        transaction["future"] = None
        try:
            error = self._set_response_error(future.result())
        except Exception as exception:  # noqa: BLE001
            error = str(exception)
        if error:
            transaction["rollback_errors"].append(
                "%s rollback 거부: %s" % (target, error))
        transaction["rollback_index"] += 1
        self._manual_start_rollback()

    def _manual_commit(self, transaction):
        if self.manual_txn is not transaction:
            return
        sx, sy, yaw_deg = transaction["new_values"]
        request = transaction["request"]
        robot_pose = transaction["robot_pose"]
        # Local state and lock are committed only after every target, including
        # the last map_diff commit, returned three successful results.
        self.gx, self.gy, self.gyaw = sx, sy, yaw_deg
        self.nominal_yaw = yaw_deg
        self.applied = True
        self.manual_lock = True
        self.bad_ticks = 0
        self.pts = []
        self.meas = []
        self.wall_hist *= 0.0
        targets = list(transaction["targets"])
        self.manual_txn = None
        self.publish_status(
            "MANUAL_APPLIED",
            "웹 위치/차체 전방을 적용했습니다; 자동 재정렬은 재시작 전까지 잠급됩니다",
            source="web_manual_pose", request_id=request["request_id"],
            start_x=sx, start_y=sy, start_yaw=yaw_deg,
            heading_offset=self.heading_offset, committed_targets=targets,
            input_frame=request["input_frame"],
            robot_map={
                "x": robot_pose[0], "y": robot_pose[1],
                "base_yaw_deg": math.degrees(robot_pose[2]),
                "front_yaw_deg": (
                    math.degrees(robot_pose[2]) + self.heading_offset) % 360.0,
            },
            desired_drawing={
                "x": request["drawing_x"], "y": request["drawing_y"],
                "front_yaw_deg": math.degrees(
                    request["drawing_front_yaw"]) % 360.0,
            })
        self.get_logger().info(
            "웹 수동 배치 transaction commit: start_x=%.3f "
            "start_y=%.3f start_yaw=%.1f targets=%s request=%s"
            % (sx, sy, yaw_deg, targets, request["request_id"]))

    def _manual_reject_transaction(self, detail, **values):
        transaction = self.manual_txn
        if transaction is None:
            return
        request_id = transaction["request"]["request_id"]
        # gx/gy/gyaw and manual_lock were never tentatively changed.
        self.manual_txn = None
        self._reject_manual(request_id, detail, **values)

    def _manual_future_timeout(self, transaction):
        phase = transaction.get("phase")
        target = transaction.get("current_target", "unknown")
        transaction["future"] = None
        if phase == "get":
            self._manual_reject_transaction(
                "%s 기존 파라미터 조회 timeout" % target)
        elif phase == "set":
            self._manual_begin_rollback(
                "%s 파라미터 적용 timeout" % target, target)
        elif phase == "rollback":
            transaction["rollback_errors"].append(
                "%s rollback timeout" % target)
            transaction["rollback_index"] += 1
            self._manual_start_rollback()

    def _manual_timer(self):
        if self.pending_manual is not None:
            self._try_pending_manual()
        transaction = self.manual_txn
        if transaction is None:
            return
        now = self.now()
        if transaction["phase"] == "discover":
            if now >= transaction["deadline"]:
                self._manual_reject_transaction(
                    "%.1f초 안에 필수 map_diff_node 파라미터 "
                    "서비스를 확인하지 못했습니다"
                    % self.manual_apply_timeout)
            else:
                self._manual_discover_targets()
            return
        if transaction.get("future") is not None:
            if now >= transaction["future_deadline"]:
                self._manual_future_timeout(transaction)
            return
        if transaction["phase"] == "rollback":
            if now >= transaction["rollback_deadline"]:
                transaction["rollback_errors"].append(
                    "rollback 전체 timeout")
                self._manual_reject_transaction(
                    transaction["failure"],
                    rollback_attempted=transaction["rollback_targets"],
                    rollback_errors=transaction["rollback_errors"])
            return
        if now >= transaction["deadline"]:
            # No current future normally means a callback is chaining calls;
            # reject conservatively if the overall transaction expired there.
            self._manual_reject_transaction(
                "파라미터 transaction 전체 timeout")

    def tick(self):
        if self.pending_manual is not None or self.manual_txn is not None:
            return
        if self.manual_lock or not self.auto_enabled:
            return
        if self.applied and self.refine <= 0.0:
            return
        # 첫 정렬은 언제든 한다. 그 뒤 갱신은 정지 중에만.
        if self.applied and self.only_when_still and self.robot_moving():
            self.pts = []                  # 주행 중 모은 점은 버린다
            return
        if len(self.pts) < self.n_scans:
            return
        pm = np.vstack(self.pts)
        self.pts = []
        self.wall_hist *= 0.5      # 오래된 벽 방향은 잊는다(드리프트 추적)

        best = self.measure(pm)
        if best is None:
            self.get_logger().error(
                "도면에 맞는 정렬을 못 찾았습니다 — start_x/y/yaw 는 기본값 "
                "%.2f/%.2f/%.1f 그대로 남습니다. 도면이 어긋난 채로 보일 것입니다. "
                "도면 안에 들어온 점 최대 %.0f%%. 기준 도면이 실제 방과 다르거나, "
                "로봇이 도면 밖에 있거나, 로봇 자리가 벽에서 %.3f m 도 안 떨어져 "
                "후보가 전부 걸러졌을 수 있습니다. xy_range 를 늘리거나 "
                "min_coverage 를 낮춰 보세요."
                % (self.gx, self.gy, self.gyaw, self.last_max_cov * 100, self.need))
            self.applied = True          # 계속 재시도하며 로그를 도배하지 않는다
            return

        # ★ 한 번 재고 바로 믿으면 안 된다.
        #   이 미로처럼 같은 모양이 규칙적으로 반복되는 도면은, 스캔을 옆 칸으로
        #   통째로 밀어도 점수가 거의 같다. 실측에서 정지한 로봇을 연속 4번 쟀더니
        #   y 가 1.10~1.45(35cm), yaw 가 341~5.5도까지 벌어졌는데 오차는 전부
        #   0.035~0.049 로 사실상 동률이었다. 그걸 그대로 주입하면 도면이 30cm 씩
        #   튄다. 그래서 여러 번 재서 서로 일치할 때만 적용한다.
        # 추적 모드에서는 합의 절차를 건너뛴다. 이미 자리를 알고 있고 그 주변만
        # 좁게 봤으므로 별칭으로 튈 일이 없다. 여기서 3회를 모으면 SLAM 드리프트를
        # 따라가기엔 너무 느려진다(3회 x 탐색 = 9초).
        if self.applied:
            cur, cur_cov = self.score(pm, self.gx, self.gy, math.radians(self.gyaw))
            # ★ 크게 어긋났으면 좁은 추적창(±track_xy/±track_yaw)으로는 절대
            #   못 돌아온다. 그때는 전역 재정렬(벽 방향 후보 + 합의)로 되돌린다.
            if best[0] > self.realign_err:
                self.bad_ticks += 1
                if self.bad_ticks >= self.realign_hits:
                    self.get_logger().error(
                        "정렬 오차 %.4f m 가 기준(%.3f)을 %d회 연속 넘었습니다 — "
                        "추적창(±%.2f m/±%.1f도) 밖으로 어긋난 것으로 보고 "
                        "전역 재정렬을 다시 합니다."
                        % (best[0], self.realign_err, self.bad_ticks,
                           self.track_xy, self.track_yaw))
                    self.applied = False
                    self.bad_ticks = 0
                    self.meas = []
                    self.wall_hist *= 0.0
                    return
            else:
                self.bad_ticks = 0
            # ★ 조금 나아진 정도로는 안 바꾼다. 도면이 미세하게 계속 움직이면
            #   플래너가 격자를 다시 만들고 경로 판정이 흔들린다.
            if best[0] >= cur - self.min_improve:
                return
            self.gx, self.gy, self.gyaw = best[1], best[2], best[3]
            self.apply(best[1], best[2], best[3], quiet=True)
            self.get_logger().info(
                "정렬 갱신(정지 중): x=%.3f y=%.3f yaw=%.1f "
                "(오차 %.4f -> %.4f m, %.1f%% 개선) | %.2f초"
                % (best[1], best[2], best[3], cur, best[0],
                   100.0 * (cur - best[0]) / max(cur, 1e-6),
                   getattr(self, "last_search_sec", 0.0)))
            return

        self.meas.append(best)
        if len(self.meas) < self.repeats:
            self.get_logger().info(
                "정렬 측정 %d/%d: x=%.2f y=%.2f yaw=%.1f (오차 %.4f m, 도면내 %.0f%%)"
                % (len(self.meas), self.repeats, best[1], best[2], best[3],
                   best[0], best[4] * 100))
            return

        batch = self.meas
        self.meas = []
        xs = np.array([m[1] for m in batch])
        ys = np.array([m[2] for m in batch])
        # 각도는 0/360 을 넘나들 수 있으므로 원형 평균으로 모은다
        ang = np.radians(np.array([m[3] for m in batch]))
        mean_ang = math.atan2(float(np.sin(ang).mean()), float(np.cos(ang).mean()))
        dev = np.abs((np.degrees(ang - mean_ang) + 180.0) % 360.0 - 180.0)
        spread_xy = float(max(xs.max() - xs.min(), ys.max() - ys.min()))
        spread_yaw = float(dev.max() * 2.0)

        if spread_xy > self.agree_xy or spread_yaw > self.agree_yaw:
            self.get_logger().error(
                "정렬 측정이 서로 안 맞습니다 (위치 %.0f mm / 각도 %.1f도 벌어짐, "
                "허용 %.0f mm / %.1f도) — 자동정렬을 적용하지 않고 지정값 "
                "%.2f/%.2f/%.1f 을 그대로 씁니다. 이 도면은 같은 모양이 반복돼서 "
                "스캔만으로는 자리를 특정할 수 없습니다. 로봇을 정해진 자리에 두고 "
                "start_x/start_y/start_yaw 를 직접 주거나, xy_range/yaw_range 를 "
                "좁혀서 그 근처만 보게 하세요."
                % (spread_xy * 1000, spread_yaw, self.agree_xy * 1000,
                   self.agree_yaw, self.gx, self.gy, self.gyaw))
            self.applied = True
            return

        # 서로 일치하니 중앙값으로 모은다 (한 번의 튐에 끌려가지 않게)
        mx, my = float(np.median(xs)), float(np.median(ys))
        myaw = math.degrees(mean_ang) % 360.0
        msc, mcov = self.score(pm, mx, my, math.radians(myaw))
        best = (msc, mx, my, myaw, mcov)
        self.get_logger().info(
            "정렬 측정 %d회 일치 (위치 %.0f mm / 각도 %.1f도 안에서 모임)"
            % (self.repeats, spread_xy * 1000, spread_yaw))

        cur, cur_cov = self.score(pm, self.gx, self.gy, math.radians(self.gyaw))
        sc, sx, sy, yd, cov = best
        if self.applied and sc > cur * 0.9:
            return                        # 이미 적용했고 뚜렷이 나아지지 않으면 유지

        self.get_logger().info(
            "정렬 측정: start_x %.2f -> %.2f | start_y %.2f -> %.2f | "
            "start_yaw %.1f -> %.1f (오차 %.4f -> %.4f m, 도면내 점 %.0f%% -> %.0f%%)"
            % (self.gx, sx, self.gy, sy, self.gyaw, yd, cur, sc,
               cur_cov * 100, cov * 100)
            + " | 탐색 %.1f초" % getattr(self, "last_search_sec", 0.0))
        self.gx, self.gy, self.gyaw = sx, sy, yd
        self.apply(sx, sy, yd)
        self.applied = True

        # 측정값 - 의도한 방향 = 라이다가 차체에 대해 돌아 장착된 각도.
        # 도면을 돌리는 대신 '스캔을 돌려서' 맞추고 싶으면 이 값을 lidar_yaw 로 준다.
        resid = ((yd - self.nominal_yaw + 180.0) % 360.0) - 180.0
        if abs(resid) >= 2.0:
            self.get_logger().warn(
                "라이다 장착 오차 추정 %+.1f도 (측정 %.1f - 의도 %.1f). "
                "도면 대신 스캔을 돌려서 맞추려면 다음 실행에 "
                "lidar_yaw:=%.1f start_yaw:=%.1f 를 주세요. "
                "영구히 고치려면 slam.launch.py 의 base_link->laser_frame 에서 "
                "--yaw %.4f (rad) 로 바꾸면 됩니다."
                % (resid, yd, self.nominal_yaw,
                   self.lidar_yaw + resid, self.nominal_yaw,
                   math.radians(self.lidar_yaw + resid)))

    def apply(self, sx, sy, yaw_deg, quiet=False):
        # 종료 중이면 조용히 빠진다. 추적 모드는 몇 초마다 도므로 Ctrl+C 가
        # 콜백 한가운데를 때리면 'node's context is invalid' 로 죽는다.
        if not rclpy.ok():
            return
        try:
            self._apply(sx, sy, yaw_deg, quiet)
        except Exception as e:                      # noqa: BLE001
            if rclpy.ok() and not quiet:
                self.get_logger().warn("파라미터 적용 실패: %s" % e)

    def _apply(self, sx, sy, yaw_deg, quiet=False):
        # 추적 모드에서는 몇 초마다 부르므로 클라이언트를 캐시한다.
        # 매번 create_client 하면 계속 쌓인다.
        if not hasattr(self, "_param_clients"):
            self._param_clients = {}
        for node in self.targets:
            cli = self._param_clients.get(node)
            if cli is None:
                cli = self.create_client(SetParameters,
                                         "/%s/set_parameters" % node)
                self._param_clients[node] = cli
            if not cli.service_is_ready():
                if not cli.wait_for_service(timeout_sec=0.5 if quiet else 3.0):
                    if not quiet:
                        self.get_logger().warn(
                            "%s 파라미터 서비스가 없습니다 — 건너뜁니다." % node)
                    continue
            req = SetParameters.Request()
            for name, val in (("start_x", sx), ("start_y", sy),
                              ("start_yaw", yaw_deg)):
                p = Parameter()
                p.name = name
                p.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                                         double_value=float(val))
                req.parameters.append(p)
            cli.call_async(req)
            if not quiet:
                self.get_logger().info(
                    "%s 에 적용: start_x=%.2f start_y=%.2f start_yaw=%.1f"
                    % (node, sx, sy, yaw_deg))
        if not quiet:
            self.get_logger().info(
                "고정하려면 launch 에 start_x:=%.2f start_y:=%.2f start_yaw:=%.1f "
                "를 주고 auto_align:=false 로 실행하세요." % (sx, sy, yaw_deg))


def main():
    rclpy.init()
    node = AutoAlign()
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
