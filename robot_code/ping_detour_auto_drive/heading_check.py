#!/usr/bin/env python3
"""
heading_check.py
----------------------------------------------------------
"base_link 의 +x 가 정말 로봇의 앞쪽인가?" 를 실제로 움직여서 측정한다.

왜 이게 결정적인가
  SLAM(cartographer)은 라이다 스캔으로 위치를 추정한다. 라이다가 차체에 대해
  yaw 로 돌아가 장착돼 있어도(예: 커넥터가 뒤를 향하게 180도 돌려 붙임),
  slam.launch.py 의 static TF 는 "--yaw 0" 이라고 선언한다. 그러면
  base_link 의 +x 축은 '라이다의 앞'이지 '로봇의 앞'이 아니게 된다.

  이때 나타나는 증상이 정확히 이렇다:
    - 위치 추적은 완벽해 보인다 (지도와 궤적 모두 일관됨)
    - 그런데 추종기가 base_link +x 기준으로 방향오차를 계산하므로,
      "전진" 명령을 내리면 로봇은 추종기가 믿는 방향과 다른 쪽으로 간다
    - 오차가 커지니 크게 돌고, 그 뒤로도 계속 어긋나 맵을 벗어난다

무엇을 측정하는가
  1) 전진 명령(w)을 짧게 주고, TF 로 관측한 '실제 이동 방향'과 그때의 yaw 를 비교
       heading_offset = atan2(dy, dx) - yaw
     0도에 가까우면 정상. 180도면 라이다가 뒤를 보고 있다는 뜻.
  2) 좌회전 명령(a)을 짧게 주고 yaw 가 증가하는지(반시계) 확인
     감소하면 모터 좌/우가 뒤바뀐 것 -> invert_angular:=true

안전
  * 앞쪽 여유를 라이다로 먼저 확인하고, 부족하면 실행을 거부한다
  * 전진은 기본 0.6초, 회전은 0.5초. Ctrl-C 로 즉시 정지
  * 자율주행 launch 를 motor:=true 로 띄운 상태에서 실행할 것
    (또는 SLAM + UART/node.py 가 떠 있는 상태)

실행
  python3 ~/auto_drive/heading_check.py
  python3 ~/auto_drive/heading_check.py --forward-time 1.0 --need-clear 0.8
  python3 ~/auto_drive/heading_check.py --dry-run     # 움직이지 않고 사전점검만
----------------------------------------------------------
"""

import argparse
import math
import os
import sys
import time

_SOURCE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SOURCE_ROOT not in sys.path:
    sys.path.insert(0, _SOURCE_ROOT)

from common.motor_primitives import primitive_twist, select_primitive  # noqa: E402

_SYS_PY = "/usr/bin/python3"
try:
    import rclpy  # noqa: F401
except ModuleNotFoundError:
    if os.path.abspath(sys.executable) != _SYS_PY and os.path.exists(_SYS_PY):
        _env = dict(os.environ)
        _env.pop("VIRTUAL_ENV", None)
        os.execve(_SYS_PY, [_SYS_PY] + sys.argv, _env)
    raise

import rclpy
import tf2_ros
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

HZ = 20.0


def norm(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class HeadingCheck(Node):
    def __init__(self, args):
        super().__init__("heading_check")
        self.args = args
        self.scan = None
        self.pub = self.create_publisher(Twist, args.topic, 10)
        self.create_subscription(LaserScan, "/scan", self.on_scan,
                                 qos_profile_sensor_data)
        self.buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buf, self)

    def on_scan(self, m):
        self.scan = m

    def spin(self, seconds):
        n = max(int(seconds * HZ), 1)
        for _ in range(n):
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(1.0 / HZ)

    def pose(self):
        try:
            tr = self.buf.lookup_transform("map", "base_link", rclpy.time.Time())
        except tf2_ros.TransformException:
            return None
        t = tr.transform.translation
        return float(t.x), float(t.y), yaw_of(tr.transform.rotation)

    def sector_min(self, center, half_deg=20.0):
        s = self.scan
        if s is None:
            return None
        half = math.radians(half_deg)
        lo = max(float(s.range_min), 0.03)
        hi = float(s.range_max) if s.range_max > 0 else 12.0
        ang, inc, best = float(s.angle_min), float(s.angle_increment), float("inf")
        for r in s.ranges:
            a = norm(ang - center)
            ang += inc
            if -half <= a <= half and lo < r < hi and r < best:
                best = r
        return best if best < float("inf") else None

    def drive(self, lin, ang, seconds):
        """짧게 명령을 내고, 명령 전/후 pose 를 돌려준다."""
        p0 = self.pose()
        n = max(int(seconds * HZ), 1)
        m = Twist()
        m.linear.x, m.angular.z = primitive_twist(
            select_primitive(lin, ang)
        )
        for _ in range(n):
            self.pub.publish(m)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(1.0 / HZ)
        stop = Twist()
        for _ in range(int(0.6 * HZ)):
            self.pub.publish(stop)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(1.0 / HZ)
        self.spin(0.6)                     # 자세가 안정될 시간
        return p0, self.pose()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", default="/cmd_vel")
    ap.add_argument("--forward-cmd", type=float, default=2.0, help="w 값")
    ap.add_argument("--turn-cmd", type=float, default=2.0, help="a 값")
    ap.add_argument("--forward-time", type=float, default=0.5)
    ap.add_argument("--turn-time", type=float, default=0.5)
    ap.add_argument("--need-clear", type=float, default=0.35,
                    help="시험에 필요한 최소 여유[m]. 앞/뒤 양쪽 모두 확인한다")
    ap.add_argument("--min-move", type=float, default=0.03,
                    help="이 정도는 움직여야 방향을 신뢰[m]")
    ap.add_argument("--dry-run", action="store_true", help="움직이지 않고 점검만")
    args, _ = ap.parse_known_args()

    rclpy.init()
    n = HeadingCheck(args)
    print("=" * 66)
    print(" base_link +x 가 정말 로봇의 앞인가? — 실측")
    print("=" * 66)
    n.spin(2.5)                            # TF / scan 버퍼 채우기

    p = n.pose()
    if p is None:
        print(" TF map->base_link 를 못 받았습니다. SLAM 이 떠 있는지 확인하세요.")
        n.destroy_node(); rclpy.shutdown(); return 1
    print(" 현재 위치 (%.3f, %.3f) yaw %.1f도" % (p[0], p[1], math.degrees(p[2])))

    front = n.sector_min(0.0)
    rear = n.sector_min(math.pi)
    print(" 라이다 앞쪽(base_link +x) 여유 %s / 뒤쪽 여유 %s"
          % ("%.2f m" % front if front else "없음",
             "%.2f m" % rear if rear else "없음"))

    if front is None:
        print(" /scan 을 못 받았습니다. 라이다가 도는지 확인하세요.")
        n.destroy_node(); rclpy.shutdown(); return 1
    # base_link 의 +x 가 로봇의 앞이 아닐 수 있으므로(그걸 재려는 것이다)
    # 앞/뒤 양쪽 모두 여유가 있어야 안전하다.
    worst = min([d for d in (front, rear) if d is not None] or [0.0])
    if worst < args.need_clear:
        print()
        print(" 앞/뒤 중 좁은 쪽 여유가 %.2f m 뿐입니다 (필요 %.2f m)." % (worst, args.need_clear))
        print(" base_link 의 +x 가 로봇의 앞인지 아직 모르므로 양쪽 다 트여야 안전합니다.")
        print(" 로봇을 트인 곳으로 옮기거나 --need-clear 0.25 로 줄여 보세요.")
        n.destroy_node(); rclpy.shutdown(); return 2
    if args.dry_run:
        print("\n --dry-run: 여기까지. 실제 측정은 이 옵션 없이 실행하세요.")
        n.destroy_node(); rclpy.shutdown(); return 0

    print()
    print(" 이제 로봇을 전진 %.1f초, 좌회전 %.1f초 움직입니다. Ctrl-C 로 즉시 정지."
          % (args.forward_time, args.turn_time))
    for i in (3, 2, 1):
        print("   %d..." % i, flush=True); time.sleep(1.0)

    rc = 0
    try:
        # ---------- 1) 전진: 실제 이동 방향 vs yaw ----------
        p0, p1 = n.drive(args.forward_cmd, 0.0, args.forward_time)
        if p0 is None or p1 is None:
            print(" 측정 실패 (TF 끊김)"); rc = 1
        else:
            dx, dy = p1[0] - p0[0], p1[1] - p0[1]
            moved = math.hypot(dx, dy)
            yaw_avg = p0[2] + 0.5 * norm(p1[2] - p0[2])
            print()
            print(" [전진 시험] 이동 %.3f m, yaw 변화 %.1f도"
                  % (moved, math.degrees(norm(p1[2] - p0[2]))))
            if moved < args.min_move:
                print("   거의 안 움직였습니다 — PWM 부족이거나 모터 노드가 없습니다.")
                print("   (ros2 topic echo %s 로 명령이 나가는지 확인)" % args.topic)
                rc = 3
            else:
                off = norm(math.atan2(dy, dx) - yaw_avg)
                od = math.degrees(off)
                print("   실제 이동 방위 %.1f도, 그때 yaw %.1f도"
                      % (math.degrees(math.atan2(dy, dx)), math.degrees(yaw_avg)))
                print("   => heading_offset = %+.1f 도" % od)
                a = abs(od)
                if a < 25.0:
                    print("   판정: 정상. base_link +x 가 로봇의 앞입니다.")
                elif a > 155.0:
                    print("   판정: ★ 180도 뒤집힘 — base_link +x 가 로봇의 '뒤'입니다.")
                    print("         라이다가 차체에 대해 180도 돌아 장착돼 있습니다.")
                    print("         이게 '180도 돌고 맵을 벗어나는' 원인입니다.")
                    print("         조치: heading_offset:=180.0 을 주거나,")
                    print("               slam.launch.py 의 base_link->laser_frame")
                    print("               static TF 를 --yaw 3.14159 로 고칠 것.")
                else:
                    print("   판정: ★ %.0f도 어긋남 — 라이다가 그만큼 돌아 장착돼 있습니다."
                          % od)
                    print("         조치: heading_offset:=%.1f 을 주세요." % od)

        # ---------- 2) 좌회전: yaw 부호 ----------
        p0, p1 = n.drive(0.0, args.turn_cmd, args.turn_time)
        if p0 is not None and p1 is not None:
            d = math.degrees(norm(p1[2] - p0[2]))
            print()
            print(" [좌회전 시험] 좌회전(angular +%.1f) 명령에 yaw 변화 %+.1f도"
                  % (args.turn_cmd, d))
            if abs(d) < 3.0:
                print("   거의 안 돌았습니다 — 제자리 회전이 안 되는 차체이거나 PWM 부족.")
            elif d > 0:
                print("   판정: 정상 (좌회전 = yaw 증가). invert_angular:=false 유지.")
            else:
                print("   판정: ★ 반대입니다. invert_angular:=true 를 주세요.")
    except KeyboardInterrupt:
        print("\n 중단됨")
    finally:
        try:
            for _ in range(6):
                n.pub.publish(Twist())
                rclpy.spin_once(n, timeout_sec=0.0)
                time.sleep(0.05)
        except Exception:
            pass
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    print()
    print("=" * 66)
    print(" 측정값을 이렇게 적용합니다:")
    print("   ros2 launch ~/auto_drive/auto_drive.launch.py \\")
    print("       heading_offset:=<위에서 나온 도> [invert_angular:=true]")
    print("=" * 66)
    return rc


if __name__ == "__main__":
    sys.exit(main())
