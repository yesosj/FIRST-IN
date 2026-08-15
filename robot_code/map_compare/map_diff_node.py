#!/usr/bin/env python3
"""
map_diff_node.py
----------------------------------------------------------
실시간 SLAM 맵(/map)과 기준 도면(room_shelf5.yaml 등)을 겹쳐 비교하고,
"다른 부분"만 색으로 강조해서 보여준다.

퍼블리시
  /reference_map        (OccupancyGrid) 기준 도면 — 시작 pose 기준으로 정렬돼 발행
  /map_diff             (OccupancyGrid) 차이 셀만 100, 나머지는 -1(투명)
  /map_diff_markers     (MarkerArray)   빨강=실제 벽인데 도면엔 없음(추가),
                                        파랑=도면엔 벽인데 실제 비어있음(사라짐)
콘솔
  일치율(%), 추가/사라진 장애물 칸수·면적

정렬 원리
  카토그래퍼는 "로봇 시작 위치"를 map 프레임 원점(0,0, +x 정면)으로 잡는다.
  그래서 "로봇이 도면(room_shelf5)상 어디서·어느 방향으로 출발했는지"만 알려주면
  (start_x, start_y, start_yaw) 그 pose 를 map 원점에 포개도록 도면을 이동·회전시켜
  실시간 맵과 겹친다. 실행 중 아래처럼 실시간 미세조정 가능:
    ros2 param set /map_diff_node start_x 0.30
    ros2 param set /map_diff_node start_yaw 90.0     # (도) 로봇이 90도 돌아 출발했으면

실행
  python3 map_diff_node.py --ros-args \
    -p reference:=/home/a202/map_compare/custom_maps/room_shelf5.yaml \
    -p start_x:=0.25 -p start_y:=0.18 -p start_yaw:=0.0

주의: rclpy/numpy 필요. venv 에는 없으므로 없으면 /usr/bin/python3 로 자동 재실행.
----------------------------------------------------------
"""

import math
import os
import sys

# --- rclpy/numpy 없으면 시스템 파이썬으로 1회 자동 재실행 ---------------
_SYS_PY = "/usr/bin/python3"
try:
    import numpy  # noqa: F401
    import rclpy  # noqa: F401
except ModuleNotFoundError:
    if os.path.abspath(sys.executable) != _SYS_PY and os.path.exists(_SYS_PY):
        _env = dict(os.environ)
        _env.pop("VIRTUAL_ENV", None)
        os.execve(_SYS_PY, [_SYS_PY] + sys.argv, _env)
    raise
# ---------------------------------------------------------------------

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy,
                       QoSHistoryPolicy)
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

sys.path.insert(0, os.path.expanduser("~/slam_test_maps"))
from slam_map_kit import read_pgm, parse_yaml  # noqa: E402

MAX_MARKERS = 20000  # 마커 폭주 방지 상한


def latched_qos():
    return QoSProfile(depth=1,
                      reliability=QoSReliabilityPolicy.RELIABLE,
                      durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                      history=QoSHistoryPolicy.KEEP_LAST)


def load_reference(yaml_path):
    """기준 도면 → (grid, res). grid: OccupancyGrid 규약(row0=아래, 0/100/-1)."""
    meta = parse_yaml(yaml_path)
    res = float(meta["resolution"])
    occ_t = float(meta.get("occupied_thresh", 0.65))
    free_t = float(meta.get("free_thresh", 0.196))
    pgm = os.path.join(os.path.dirname(os.path.abspath(yaml_path)),
                       os.path.basename(meta["image"]))
    w, h, _mx, px = read_pgm(pgm)
    img = np.frombuffer(bytes(px), dtype=np.uint8).reshape(h, w)
    p = (255.0 - img.astype(np.float32)) / 255.0
    grid = np.full((h, w), -1, dtype=np.int16)
    grid[p < free_t] = 0
    grid[p > occ_t] = 100
    return np.flipud(grid), res           # 이미지(위=row0) → 맵(아래=row0)


class MapDiff(Node):
    def __init__(self):
        super().__init__("map_diff_node")
        ref_path = self.declare_parameter(
            "reference",
            os.path.expanduser("~/map_compare/custom_maps/room_self5.yaml")).value
        self.live_topic = self.declare_parameter("live_topic", "/map").value
        self.frame = self.declare_parameter("frame", "map").value
        # 정렬: "로봇이 도면상 어디서·어느 방향으로 출발했는가"를 지정한다.
        # 이 pose 가 map 원점(로봇 시작점)에 포개진다.
        #   start_x, start_y : 도면 좌표계(m)에서 로봇 위치 (왼쪽아래=작은값)
        #   start_yaw        : 도면에서 로봇이 바라본 방향 [도]
        self.declare_parameter("start_x", 0.15)
        self.declare_parameter("start_y", 0.15)
        self.declare_parameter("start_yaw", 180.0)

        self.ref, self.ref_res = load_reference(ref_path)
        rh, rw = self.ref.shape
        self.get_logger().info(
            f"기준 도면 로드: {ref_path} ({rw}x{rh} @ {self.ref_res} m/px)")

        self.ref_pub = self.create_publisher(OccupancyGrid, "/reference_map", latched_qos())
        self.diff_pub = self.create_publisher(OccupancyGrid, "/map_diff", latched_qos())
        self.mk_pub = self.create_publisher(MarkerArray, "/map_diff_markers", latched_qos())

        self.last_live = None
        self.create_subscription(OccupancyGrid, self.live_topic,
                                 self.on_live_map, latched_qos())
        self.publish_reference()
        # 1Hz 로 기준도면 재발행 + 재비교 → ros2 param set 으로 바꾼 정렬값이
        # 1초 안에 자동 반영된다(RViz 에서 실시간으로 도면이 이동/회전).
        self.create_timer(1.0, self._tick)
        self.get_logger().info(f"'{self.live_topic}' 구독 시작 — 비교 대기 중...")

    # --- 시작 pose (정렬 파라미터) -------------------------------------
    def _pose(self):
        return (self.get_parameter("start_x").value,
                self.get_parameter("start_y").value,
                math.radians(self.get_parameter("start_yaw").value))

    def _tick(self):
        self.publish_reference()
        if self.last_live is not None:
            self.on_live_map(self.last_live)

    # --- 기준 도면을 시작 pose 기준으로 정렬해 발행 --------------------
    def publish_reference(self):
        sx, sy, yaw = self._pose()
        rh, rw = self.ref.shape
        msg = OccupancyGrid()
        msg.header.frame_id = self.frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = self.ref_res
        msg.info.width = rw
        msg.info.height = rh
        # 도면점 Pd → map: Pm = R(-yaw)*(Pd - S). 로봇 시작 S 는 map 원점에 온다.
        #   origin(왼아래 코너, Pd=0) = R(-yaw)*(-S)
        c, s = math.cos(-yaw), math.sin(-yaw)
        msg.info.origin.position.x = c * (-sx) - s * (-sy)
        msg.info.origin.position.y = s * (-sx) + c * (-sy)
        msg.info.origin.orientation.z = math.sin(-yaw / 2.0)
        msg.info.origin.orientation.w = math.cos(-yaw / 2.0)
        msg.data = self.ref.astype(np.int8).flatten().tolist()
        self.ref_pub.publish(msg)

    # --- 실시간 맵이 올 때마다 비교 ------------------------------------
    def on_live_map(self, msg: OccupancyGrid):
        self.last_live = msg
        w, h = msg.info.width, msg.info.height
        res = msg.info.resolution
        ox, oy = msg.info.origin.position.x, msg.info.origin.position.y
        live = np.array(msg.data, dtype=np.int16).reshape(h, w)
        sx, sy, yaw = self._pose()

        # 실시간 맵 셀 중심의 map 좌표 → 도면 좌표로 역변환: Pd = R(yaw)*Pm + S
        rh, rw = self.ref.shape
        cc, rr = np.meshgrid(np.arange(w), np.arange(h))
        mx = ox + (cc + 0.5) * res
        my = oy + (rr + 0.5) * res
        c, s = math.cos(yaw), math.sin(yaw)
        rx = c * mx - s * my + sx
        ry = s * mx + c * my + sy
        rcol = np.floor(rx / self.ref_res).astype(int)
        rrow = np.floor(ry / self.ref_res).astype(int)
        ok = (rcol >= 0) & (rcol < rw) & (rrow >= 0) & (rrow < rh)
        ref_on_live = np.full((h, w), -1, dtype=np.int16)
        ref_on_live[ok] = self.ref[np.clip(rrow, 0, rh - 1)[ok],
                                   np.clip(rcol, 0, rw - 1)[ok]]

        live_occ = live >= 50
        live_free = (live >= 0) & (live < 50)
        ref_occ = ref_on_live >= 50
        ref_free = (ref_on_live >= 0) & (ref_on_live < 50)
        both_known = (live_occ | live_free) & (ref_occ | ref_free)
        added = both_known & live_occ & ref_free      # 실제 벽인데 도면엔 없음
        missing = both_known & live_free & ref_occ     # 도면엔 벽인데 실제 빔

        area = res * res
        known_n = int(both_known.sum())
        n_add, n_mis = int(added.sum()), int(missing.sum())
        match = 100.0 * (known_n - n_add - n_mis) / known_n if known_n else 0.0
        self.get_logger().info(
            "일치율 %.1f%% | 추가(빨강) %d칸 %.3f m^2 | 사라짐(파랑) %d칸 %.3f m^2"
            % (match, n_add, n_add * area, n_mis, n_mis * area))

        # /map_diff: 차이만 100, 나머지 -1(투명)
        diff = np.full((h, w), -1, dtype=np.int8)
        diff[added | missing] = 100
        out = OccupancyGrid()
        out.header.frame_id = self.frame
        out.header.stamp = msg.header.stamp
        out.info = msg.info
        out.data = diff.flatten().tolist()
        self.diff_pub.publish(out)

        # /map_diff_markers: 빨강=추가, 파랑=사라짐
        self.publish_markers(added, missing, ox, oy, res)

    def publish_markers(self, added, missing, ox, oy, res):
        arr = MarkerArray()
        for idx, (mask, rgba, ns) in enumerate((
                (added, (1.0, 0.1, 0.1, 0.9), "added"),
                (missing, (0.1, 0.4, 1.0, 0.9), "missing"))):
            m = Marker()
            m.header.frame_id = self.frame
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = ns
            m.id = idx
            m.type = Marker.CUBE_LIST
            m.action = Marker.ADD
            m.scale.x = m.scale.y = res
            m.scale.z = 0.01
            m.color = ColorRGBA(r=rgba[0], g=rgba[1], b=rgba[2], a=rgba[3])
            m.pose.orientation.w = 1.0
            rows, cols = np.nonzero(mask)
            if len(rows) > MAX_MARKERS:
                sel = np.linspace(0, len(rows) - 1, MAX_MARKERS).astype(int)
                rows, cols = rows[sel], cols[sel]
                self.get_logger().warn(
                    f"{ns} 차이 셀이 많아 {MAX_MARKERS}개로 다운샘플링")
            m.points = [Point(x=float(ox + (c + 0.5) * res),
                              y=float(oy + (r + 0.5) * res), z=0.0)
                        for r, c in zip(rows, cols)]
            arr.markers.append(m)
        self.mk_pub.publish(arr)


def main():
    rclpy.init()
    node = MapDiff()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
