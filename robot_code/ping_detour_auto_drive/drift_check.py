#!/usr/bin/env python3
"""
drift_check.py
----------------------------------------------------------
로봇을 **가만히 세워 둔 채로** SLAM 자세가 흘러가는지 잰다.

왜 필요한가
  자동정렬(auto_align_node)이 잴 때마다 다른 답을 내놓는 일이 있었다.
  정지한 로봇을 연속으로 쟀는데 y 가 1.10~1.45 로 35 cm, yaw 가 341~5.5도까지
  벌어졌다. 그런데 '현재 설정' 기준 오차가 10분에 걸쳐
  0.039 -> 0.097 m 로 단조 증가했다. 탐색이 흔들린 게 아니라
  **맞춰야 할 대상(map 프레임의 스캔)이 계속 움직이고 있었던** 것이다.

  정지한 로봇에서 map -> base_link 는 움직이면 안 된다. 움직인다면
  cartographer 가 표류하는 것이고, 원인은 대개 IMU 자이로 바이어스다
  (my_robot.lua 의 use_imu_data = true).

읽는 법
  yaw 표류가 시간에 비례해 늘면  -> 자이로 바이어스. 정지 상태에서 /imu 의
      angular_velocity.z 평균이 0 이 아닌지 같이 찍어 준다.
  위치만 튀고 yaw 는 가만있으면 -> 스캔매칭이 미로의 반복무늬에 미끄러지는 것.
  둘 다 거의 0 이면            -> SLAM 은 멀쩡하다. 정렬 문제는 다른 데 있다.

실행 (자율주행 launch 를 띄운 상태에서, 로봇은 절대 건드리지 말 것)
  python3 ~/auto_drive/drift_check.py           # 60초
  python3 ~/auto_drive/drift_check.py 180       # 180초
----------------------------------------------------------
"""

import math
import sys
import time

import numpy as np
import rclpy
import tf2_ros
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


def yaw_of(q):
    return math.degrees(math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                   1.0 - 2.0 * (q.y * q.y + q.z * q.z)))


def main():
    dur = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
    rclpy.init()
    node = rclpy.create_node("drift_check")
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, node)

    gyro = []
    node.create_subscription(
        Imu, "/imu", lambda m: gyro.append(m.angular_velocity.z),
        qos_profile_sensor_data)

    print("로봇을 건드리지 마세요. %.0f초간 map -> base_link 를 지켜봅니다." % dur)
    rows = []
    t0 = time.time()
    nxt = 0.0
    while time.time() - t0 < dur:
        rclpy.spin_once(node, timeout_sec=0.05)
        el = time.time() - t0
        if el < nxt:
            continue
        nxt = el + 1.0
        try:
            tr = buf.lookup_transform("map", "base_link", rclpy.time.Time())
        except tf2_ros.TransformException:
            continue
        t = tr.transform.translation
        rows.append((el, t.x, t.y, yaw_of(tr.transform.rotation)))

    if len(rows) < 5:
        print("자세를 충분히 못 읽었습니다. launch 가 떠 있는지 확인하세요.")
        return

    a = np.array(rows)
    x, y = a[:, 1] - a[0, 1], a[:, 2] - a[0, 2]
    yw = np.degrees(np.unwrap(np.radians(a[:, 3])))
    yw = yw - yw[0]
    dist = np.hypot(x, y)

    print()
    print("  경과      x이동[mm]  y이동[mm]  총이동[mm]  yaw변화[도]")
    for r, dx, dy, dd, dyaw in zip(a[::max(1, len(a) // 12), 0],
                                   x[::max(1, len(a) // 12)] * 1000,
                                   y[::max(1, len(a) // 12)] * 1000,
                                   dist[::max(1, len(a) // 12)] * 1000,
                                   yw[::max(1, len(a) // 12)]):
        print("  %5.0f초   %+8.1f  %+8.1f  %9.1f  %+10.2f" % (r, dx, dy, dd, dyaw))

    span = a[-1, 0] - a[0, 0]
    print()
    print("정지 상태 %.0f초 동안" % span)
    print("  위치 표류 : %.1f mm  (분당 %.1f mm)" % (dist[-1] * 1000,
                                                dist[-1] * 1000 * 60 / span))
    print("  각도 표류 : %+.2f 도 (분당 %+.2f 도)" % (yw[-1], yw[-1] * 60 / span))
    if gyro:
        g = np.array(gyro)
        print("  /imu 자이로 z 평균 %+.5f rad/s -> 그대로 적분하면 분당 %+.2f 도"
              % (g.mean(), math.degrees(g.mean()) * 60))
    print()
    if abs(yw[-1] * 60 / span) > 1.0:
        print("판정: 각도가 분당 1도 넘게 흐릅니다 — 자이로 바이어스가 의심됩니다.")
        print("      로봇을 완전히 정지시킨 채 SLAM 을 다시 띄우거나,")
        print("      my_robot.lua 의 use_imu_data 를 꺼 보고 표류가 멎는지 보세요.")
        print("      (my_robot.lua 는 기존 패키지이므로 직접 판단해서 고칠 것)")
    elif dist[-1] * 1000 * 60 / span > 20.0:
        print("판정: 각도는 멀쩡한데 위치가 흐릅니다 — 스캔매칭이 미로의 반복무늬에")
        print("      미끄러지는 쪽입니다. 자동정렬을 좁은 범위로만 쓰는 게 낫습니다.")
    else:
        print("판정: SLAM 은 안정적입니다. 정렬이 안 맞으면 원인은 다른 데 있습니다.")

    node.destroy_node()
    rclpy.shutdown()


main()
