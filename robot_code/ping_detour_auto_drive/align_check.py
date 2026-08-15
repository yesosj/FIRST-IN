#!/usr/bin/env python3
"""
align_check.py
----------------------------------------------------------
"도면(reference)과 실제 로봇의 정렬(start_x / start_y / start_yaw)이 맞는가?"
를 라이다 스캔으로 직접 측정한다.

왜 필요한가
  map 프레임의 원점은 '로봇이 SLAM 을 시작한 자리'이고, 그 자리가 도면 위
  어디에 어느 방향으로 있는지는 start_x / start_y / start_yaw 로 사람이 알려준다.
  이 값이 틀리면 경로는 도면 기준으로 멀쩡해 보이는데 실제 공간에서는
  엉뚱한 방향이 된다. 특히 180도 틀리면 정확히 반대편으로 간다.
  RViz 에서 초록 경로를 잘 따라가는데 실제로는 반대편으로 가는 증상이 이것이다.

무엇을 하는가
  1) /scan 을 map 프레임으로 옮겨 점구름을 만든다 (TF 를 그대로 쓰므로
     라이다가 뒤집혀 장착된 것도 자동 반영된다)
  2) 도면의 벽까지 거리변환(distance transform)을 미리 계산한다
  3) start_x / start_y / start_yaw 를 격자탐색하며 "스캔 점들이 도면 벽에
     얼마나 잘 얹히는가"를 점수로 매긴다
  4) 가장 잘 맞는 값과 그대로 쓸 수 있는 launch 명령을 출력한다

실행 (자율주행 launch 를 띄워 둔 상태에서, 로봇을 출발 위치에 두고)
  python3 ~/auto_drive/align_check.py
  python3 ~/auto_drive/align_check.py --yaw-step 1 --xy-range 0.4

로봇은 움직이지 않는다. 읽기만 한다.
----------------------------------------------------------
"""

import argparse
import math
import os
import sys

_SYS_PY = "/usr/bin/python3"
try:
    import rclpy  # noqa: F401
    import numpy  # noqa: F401
    import cv2    # noqa: F401
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
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

sys.path.insert(0, os.path.expanduser("~/slam_test_maps"))
from slam_map_kit import read_pgm, parse_yaml  # noqa: E402

DEFAULT_REF = os.path.expanduser(
    "~/map_compare/custom_maps/room_shelf5.yaml")


def load_walls(yaml_path):
    """도면을 읽어 (벽=1 격자, 해상도) 반환. path_planner_node 와 같은 규약."""
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
    return np.flipud(occ), res          # 이미지(위=row0) -> 맵(아래=row0)


def quat_to_mat(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class AlignCheck(Node):
    def __init__(self, args):
        super().__init__("align_check")
        self.args = args
        self.pts = []                    # map 프레임 스캔점 (N,2)
        self.buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buf, self)
        self.create_subscription(LaserScan, args.scan_topic, self.on_scan,
                                 qos_profile_sensor_data)
        self.done = False

    def on_scan(self, msg: LaserScan):
        if self.done or len(self.pts) >= self.args.scans:
            return
        try:
            tr = self.buf.lookup_transform("map", msg.header.frame_id,
                                           rclpy.time.Time())
        except tf2_ros.TransformException:
            return
        R = quat_to_mat(tr.transform.rotation)
        t = np.array([tr.transform.translation.x, tr.transform.translation.y,
                      tr.transform.translation.z])
        r = np.asarray(msg.ranges, dtype=np.float64)
        a = msg.angle_min + np.arange(r.size) * msg.angle_increment
        ok = np.isfinite(r) & (r > max(msg.range_min, 0.03)) & \
             (r < (msg.range_max if msg.range_max > 0 else 12.0))
        if ok.sum() < 20:
            return
        local = np.stack([r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok]),
                          np.zeros(int(ok.sum()))], axis=1)
        world = local @ R.T + t
        self.pts.append(world[:, :2])
        self.get_logger().info("스캔 %d/%d 수집 (%d점)"
                               % (len(self.pts), self.args.scans, int(ok.sum())))
        if len(self.pts) >= self.args.scans:
            self.done = True


CLIP_M = 0.12          # 이보다 멀면 다 똑같이 나쁜 것으로 (이상치 둔감)


def score_alignment(pm, distm, res, h, w, sx, sy, yaw):
    """스캔점(map)을 도면으로 옮겨 '벽 표면'까지 평균거리[m]. 작을수록 잘 맞음.

    벽 안쪽까지의 거리(distanceTransform of free)를 쓰면 안 된다. 이 도면은
    벽이 두꺼워서(전체의 24%) 아무 정렬에서나 점들이 벽 '속'에 떨어지고
    거리 0 이 되어 버린다. 라이다는 벽 '표면'을 보므로 표면까지의 거리를 쓴다.
    """
    c, s = math.cos(yaw), math.sin(yaw)
    dx = c * pm[:, 0] - s * pm[:, 1] + sx        # Pd = R(yaw)*Pm + S
    dy = s * pm[:, 0] + c * pm[:, 1] + sy
    col = (dx / res).astype(np.int32)
    row = (dy / res).astype(np.int32)
    inside = (row >= 0) & (row < h) & (col >= 0) & (col < w)
    n = int(inside.sum())
    if n < 20:
        return 1e9, 0.0
    d = np.minimum(distm[row[inside], col[inside]], CLIP_M)
    # 도면 밖으로 나간 점은 최대 벌점
    total = (float(d.sum()) + CLIP_M * (len(pm) - n)) / len(pm)
    return total, float(inside.mean())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--reference", default=DEFAULT_REF)
    ap.add_argument("--scan-topic", default="/scan")
    ap.add_argument("--scans", type=int, default=5, help="누적할 스캔 수")
    ap.add_argument("--start-x", type=float, default=0.15, help="현재 설정값")
    ap.add_argument("--start-y", type=float, default=0.15)
    ap.add_argument("--start-yaw", type=float, default=180.0)
    ap.add_argument("--yaw-step", type=float, default=2.0, help="탐색 각도 간격[도]")
    ap.add_argument("--xy-range", type=float, default=0.30, help="x/y 탐색 반경[m]")
    ap.add_argument("--xy-step", type=float, default=0.02)
    ap.add_argument("--robot-width", type=float, default=0.123)
    ap.add_argument("--margin", type=float, default=0.02,
                    help="path_planner_node 의 safety_margin 과 같은 값")
    args, _ = ap.parse_known_args()

    ref = os.path.expanduser(args.reference)
    occ, res = load_walls(ref)
    h, w = occ.shape
    # 라이다가 실제로 보는 것은 '자유공간에 접한 벽 표면'이다. 그 표면까지의
    # 거리를 쓴다. (벽 내부까지의 거리를 쓰면 두꺼운 벽 때문에 지표가 포화된다)
    free = (occ == 0).astype(np.uint8)
    k = np.ones((3, 3), np.uint8)
    surface = (occ.astype(np.uint8) & cv2.dilate(free, k)).astype(np.uint8)
    distm = cv2.distanceTransform(1 - surface, cv2.DIST_L2, 5) * res
    # 로봇 자신이 서 있을 수 있는 곳인지 판정하려면 '벽까지의 여유'가 필요하다.
    # path_planner_node 와 같은 기준: 0.5*폭 + safety_margin
    clear = cv2.distanceTransform(free, cv2.DIST_L2, 5) * res
    need = 0.5 * args.robot_width + args.margin
    print("도면 %s  (%dx%d @ %.3f m/px, 벽 %d셀 중 표면 %d셀)"
          % (os.path.basename(ref), w, h, res, int(occ.sum()), int(surface.sum())))
    print("로봇 시작 위치는 벽에서 최소 %.4f m 떨어져야 한다 "
          "(폭 %.3f/2 + 여유 %.2f). 이 조건을 만족하는 후보만 본다."
          % (need, args.robot_width, args.margin), flush=True)

    rclpy.init()
    node = AlignCheck(args)
    print("스캔 수집 중... (자율주행 launch 가 떠 있어야 합니다)", flush=True)
    t0 = node.get_clock().now().nanoseconds / 1e9
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=0.2)
        if node.get_clock().now().nanoseconds / 1e9 - t0 > 20.0:
            break
    got = len(node.pts)
    node.destroy_node()
    rclpy.shutdown()

    if got == 0:
        print("\n스캔을 못 받았습니다. /scan 과 TF(map->laser_frame)가 나오는지 "
              "확인하세요:\n  ros2 topic hz /scan\n"
              "  ros2 run tf2_ros tf2_echo map laser_frame")
        return 1

    pm = np.vstack(node.pts)
    print("스캔점 %d개로 정렬 탐색 시작\n" % len(pm), flush=True)

    cur = score_alignment(pm, distm, res, h, w,
                          args.start_x, args.start_y, math.radians(args.start_yaw))
    print("현재 설정  start_x=%.2f start_y=%.2f start_yaw=%.1f"
          % (args.start_x, args.start_y, args.start_yaw))
    print("           벽까지 median 거리 %.3f m, 도면 안에 들어온 점 %.0f%%\n"
          % (cur[0], cur[1] * 100), flush=True)

    # --- 격자 탐색 ---
    offs = np.arange(-args.xy_range, args.xy_range + 1e-9, args.xy_step)
    best = None
    per_yaw = []
    for yd in np.arange(0.0, 360.0, args.yaw_step):
        yaw = math.radians(yd)
        byaw = None
        for ddx in offs:
            for ddy in offs:
                sx, sy = args.start_x + ddx, args.start_y + ddy
                # 로봇이 실제로 서 있을 수 없는 자리면 애초에 답이 될 수 없다.
                # (이 제약이 없으면 스캔 맞춤만 좋은, 벽 속에 로봇이 박힌 해가 나온다)
                r0, c0 = int(sy / res), int(sx / res)
                if not (0 <= r0 < h and 0 <= c0 < w) or clear[r0, c0] < need:
                    continue
                sc, cov = score_alignment(pm, distm, res, h, w, sx, sy, yaw)
                if cov < 0.85:            # 도면 밖으로 많이 나가면 실격
                    continue
                if byaw is None or sc < byaw[0]:
                    byaw = (sc, sx, sy, yd, cov)
        if byaw is not None:
            per_yaw.append(byaw)
            if best is None or byaw[0] < best[0]:
                best = byaw

    if best is None:
        print("어떤 조합도 도면 안에 들어오지 않습니다. 기준 도면이 실제 방과 "
              "다르거나 xy-range 를 늘려야 합니다 (--xy-range 0.6).")
        return 1

    per_yaw.sort(key=lambda v: v[0])
    print("가장 잘 맞는 정렬 상위 6개 (벽 표면까지 평균거리가 작을수록 좋음)")
    print("   %-10s %-8s %-8s %-10s %-10s %s"
          % ("start_yaw", "start_x", "start_y", "오차[m]", "벽여유[m]", "도면내 점"))
    for sc, sx, sy, yd, cov in per_yaw[:6]:
        cl = clear[int(sy / res), int(sx / res)]
        print("   %-10.1f %-8.2f %-8.2f %-10.4f %-10.3f %.0f%%"
              % (yd, sx, sy, sc, cl, cov * 100))

    # --- 각도별 점수 분포 (180도 뒤집힘인지 눈으로 확인) ---
    def angdiff(a, b):
        return abs(((a - b + 180.0) % 360.0) - 180.0)

    print("\n각도별 최저 오차 (막대가 짧을수록 잘 맞음)")
    bins = {}
    for sc, sx, sy, yd, cov in per_yaw:
        k = int(yd // 15) * 15
        if k not in bins or sc < bins[k][0]:
            bins[k] = (sc, sx, sy, yd, cov)
    worst = max(v[0] for v in bins.values())
    for k in sorted(bins):
        sc = bins[k][0]
        bar = "#" * max(int(40 * sc / worst), 1)
        print("  %3d~%3d도  %.4f  %s" % (k, k + 15, sc, bar))

    b = per_yaw[0]
    opp = min((v for v in per_yaw if abs(angdiff(v[3], b[3]) - 180.0) < 25.0),
              key=lambda v: v[0], default=None)
    if opp is not None and opp[0] < b[0] * 1.25:
        print("\n   ※ %.0f 도(%.4f)와 그 반대편 %.0f 도(%.4f)의 점수가 비슷합니다."
              % (b[3], b[0], opp[3], opp[0]))
        print("     도면이 180도 회전대칭에 가까워 스캔만으로는 앞뒤를 못 가립니다.")
        print("     아래 '앞뒤 확정 방법'으로 직접 확인하세요.")
    elif opp is not None:
        print("\n   반대편 %.0f 도는 오차 %.4f 로 %.1f배 나쁩니다 — 180도 뒤집힘은 아닙니다."
              % (opp[3], opp[0], opp[0] / max(b[0], 1e-9)))

    sc, sx, sy, yd, cov = best
    print()
    print("=" * 62)
    print(" 측정 결과: start_yaw = %.1f 도  (현재 설정 %.1f 도)" % (yd, args.start_yaw))
    dy = (yd - args.start_yaw + 180.0) % 360.0 - 180.0
    if abs(dy) < args.yaw_step * 1.5:
        print(" → 현재 설정이 맞습니다. 정렬 문제가 아닙니다.")
    else:
        print(" → 현재 설정과 %.0f 도 차이납니다. 이게 로봇이 엉뚱한 방향으로 가는 원인입니다."
              % dy)
        if abs(abs(dy) - 180.0) < 30.0:
            print("   (약 180도 = 도면이 정확히 뒤집혀 반영되고 있었다는 뜻)")
    print("=" * 62)
    print(" 개선: 벽까지 오차 %.3f m -> %.3f m" % (cur[0], sc))
    print()
    print(" 아래 명령으로 다시 실행하세요:")
    print("   ros2 launch ~/auto_drive/auto_drive.launch.py \\")
    print("       start_x:=%.2f start_y:=%.2f start_yaw:=%.1f" % (sx, sy, yd))
    print("=" * 62)
    print()
    print(" [앞뒤 확정 방법] 180도 헷갈릴 때는 이렇게 직접 확인한다:")
    print("   1) RViz 를 띄운 채로 keyboard_test.py 로 w 를 1초만 누른다")
    print("   2) 실제 로봇이 방의 어느 쪽으로 갔는지 본다")
    print("   3) RViz 에서 로봇 모델도 도면상 같은 쪽으로 갔는지 본다")
    print("   반대쪽으로 갔다면 start_yaw 에 180 을 더하거나 빼면 된다.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
