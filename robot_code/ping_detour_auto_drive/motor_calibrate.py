#!/usr/bin/env python3
"""
motor_calibrate.py
----------------------------------------------------------
전출력 직진/제자리 회전/정지를 눈으로 확인하는 도구.

모든 0이 아닌 입력은 공통 모터 규약에 따라 PWM 절댓값 255로 변환된다.

** 로봇이 실제로 움직인다. 바퀴가 뜨도록 받침대에 올리거나, 앞뒤로 1m 이상
   여유가 있는 곳에서 돌릴 것. 언제든 Ctrl-C 로 즉시 정지한다. **

사전 준비 (다른 터미널에서 UART 모터 노드가 떠 있어야 한다)
  python3 ~/UART/node.py

실행
  python3 ~/auto_drive/motor_calibrate.py                 # 전진+회전
  python3 ~/auto_drive/motor_calibrate.py --mode turn     # 회전만
  python3 ~/auto_drive/motor_calibrate.py --start 0.2 --stop 1.0 --step 0.1
  python3 ~/auto_drive/motor_calibrate.py --topic /cmd_vel_test   # 안전 예행연습

주의: rclpy 는 venv 에 없으므로 /usr/bin/python3 로 자동 재실행한다.
----------------------------------------------------------
"""

import argparse
import os
import sys
import time

_SOURCE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SOURCE_ROOT not in sys.path:
    sys.path.insert(0, _SOURCE_ROOT)

from common.motor_primitives import (  # noqa: E402
    primitive_pwm,
    primitive_twist,
    select_primitive,
)

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
from rclpy.node import Node
from geometry_msgs.msg import Twist

HZ = 20.0          # keyboard_test.py / path_follower_node.py 와 동일


class Calibrator(Node):
    def __init__(self, topic):
        super().__init__("motor_calibrate")
        self.pub = self.create_publisher(Twist, topic, 10)
        self.topic = topic

    def send(self, lin, ang):
        lin, ang = primitive_twist(select_primitive(lin, ang))
        m = Twist()
        m.linear.x = float(lin)
        m.angular.z = float(ang)
        self.pub.publish(m)

    def hold(self, lin, ang, seconds):
        """seconds 동안 20Hz 로 계속 발행 (UART 노드의 0.1초 타임아웃 때문에 필수)."""
        n = max(int(seconds * HZ), 1)
        for _ in range(n):
            self.send(lin, ang)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(1.0 / HZ)

    def stop(self, seconds=0.8):
        self.hold(0.0, 0.0, seconds)


def sweep(node, label, make_cmd, start, stop, step, dwell, pause):
    print()
    print("=" * 60)
    print("[%s] %.2f 부터 %.2f 까지 %.2f 씩 올립니다" % (label, start, stop, step))
    print("=" * 60)
    cmd = start
    while cmd <= stop + 1e-9:
        lin, ang = make_cmd(cmd)
        left, right = primitive_pwm(select_primitive(lin, ang))
        print("  cmd=%.2f   ->  PWM 좌 %+4d / 우 %+4d   ... %.1f초"
              % (cmd, left, right, dwell), flush=True)
        node.hold(lin, ang, dwell)
        node.stop(pause)
        cmd += step
    print("  -- %s 끝, 정지 --" % label, flush=True)


def main():
    ap = argparse.ArgumentParser(add_help=True)
    ap.add_argument("--mode", choices=["forward", "turn", "both"], default="both",
                    help="측정 항목 (기본 both)")
    ap.add_argument("--start", type=float, default=0.20, help="시작 cmd 값")
    ap.add_argument("--stop", type=float, default=1.00, help="끝 cmd 값")
    ap.add_argument("--step", type=float, default=0.10, help="증가 폭")
    ap.add_argument("--dwell", type=float, default=1.5, help="각 단계 유지 시간[s]")
    ap.add_argument("--pause", type=float, default=1.0, help="단계 사이 정지 시간[s]")
    ap.add_argument("--topic", default="/cmd_vel", help="발행 토픽")
    args, _ = ap.parse_known_args()

    print("=" * 60)
    print(" 모터 전출력 primitive 확인  (토픽: %s)" % args.topic)
    print("=" * 60)
    print(" * 다른 터미널에서 UART 모터 노드가 떠 있어야 합니다:")
    print("     python3 ~/UART/node.py")
    print(" * 로봇이 실제로 움직입니다. 주변 여유를 확보하세요.")
    print(" * 모든 0이 아닌 cmd는 PWM 절댓값 255로 실행됩니다.")
    print(" * Ctrl-C 로 언제든 즉시 정지합니다.")
    print()
    for i in (3, 2, 1):
        print("  %d 초 후 시작..." % i, flush=True)
        time.sleep(1.0)

    rclpy.init()
    node = Calibrator(args.topic)
    try:
        node.stop(0.5)
        if args.mode in ("forward", "both"):
            sweep(node, "전진", lambda c: (c, 0.0),
                  args.start, args.stop, args.step, args.dwell, args.pause)
        if args.mode in ("turn", "both"):
            sweep(node, "제자리 회전", lambda c: (0.0, c),
                  args.start, args.stop, args.step, args.dwell, args.pause)
    except KeyboardInterrupt:
        print("\n중단됨 — 정지합니다.", flush=True)
    finally:
        try:
            node.stop(1.0)
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    print("모든 primitive 전출력 확인 완료")


if __name__ == "__main__":
    main()
