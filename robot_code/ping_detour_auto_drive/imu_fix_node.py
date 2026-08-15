#!/usr/bin/env python3
"""
imu_fix_node.py
----------------------------------------------------------
MPU6050 을 직접 읽어 **보정한 뒤** /imu 로 내보낸다.
slam_bringup 의 imu_filter_node.py 를 대신한다 (그 패키지는 건드리지 않는다).

왜 필요한가 — 실측으로 확인한 이 개체의 결함 4가지

  ① 자이로 바이어스가 크다
       x -0.129 rad/s (-7.4 도/초),  y +0.127,  z -0.0088 (1분에 -30도)
       정상적인 MPU6050 은 ±0.5~2 도/초 수준이라 3~15배다.
       z 바이어스는 곧바로 heading 오차가 되어 로봇이 지도 밖으로 밀려난다.

  ② 가속도 스케일이 9.5% 부족하다
       |a| 평균 8.878 m/s^2 (정상 9.807). 값은 안정적인데 일정하게 낮다
       = 개체 스케일 오차. cartographer 는 이 벡터로 중력 방향을 잡으므로
       크기가 틀리면 자세 보정이 계속 어긋난다.

  ③ 9.3도 기울어져 장착돼 있다
       평균 벡터 x +1.294  y -0.627  z +8.760
       static TF(base_link->imu_link)는 roll/pitch 가 0 이라 이 기울기를
       반영하지 못한다. 2D SLAM 에서는 회전 추정 오차로 직결된다.

  ④ ★ 기존 노드는 정지 판정을 /cmd_vel 로 한다
       이게 자기강화 고리를 만든다:
         바이어스로 TF 가 조금 돌아감
           -> 추종기가 "방향이 틀어졌다"며 회전 명령
           -> IMU 노드는 cmd_vel 이 0 이 아니므로 '이동 중' 으로 보고
              각속도 0 처리와 바이어스 갱신을 **둘 다 멈춤**
           -> TF 가 더 돌아감 -> 반복
       한 번 돌기 시작하면 안 멈춘다(RViz 로봇모델이 계속 360도 도는 현상).

  (I2C 통신 자체는 문제없다: 400표본 오류 0회, 읽기 0.56 ms, 간격 편차 0.12 ms)

무엇을 고치나
  1) 기동 시 정지 상태에서 바이어스 + 중력벡터를 잰다
  2) 중력 크기를 9.807 로 맞추는 스케일을 구한다            -> ②
  3) 측정된 중력을 +z 로 보내는 회전을 구해 6축 전체에 적용  -> ③
  4) 주행 중에도 '정지'로 판정될 때마다 바이어스를 계속 갱신 -> ①
     (온도가 오르면 바이어스가 변한다. 기존 노드는 alpha 0.001 로 느렸다)
  5) 정지 중에는 각속도를 0 으로 발행해 드리프트 누적을 끊는다

  ★ 정지 판정은 /cmd_vel 에 기대지 않고 **센서만으로** 한다. 이게 ④ 의 고리를
    끊는 핵심이다. 모터 명령이 0 이어도 관성으로 미끄러지는 구간이 있고,
    반대로 명령이 있어도 바퀴가 헛돌아 안 움직이는 구간이 있다.

실행
  (보통은 auto_drive.launch.py 가 own_imu:=true 로 알아서 띄운다)
  python3 imu_fix_node.py --ros-args -p calib_seconds:=3.0
----------------------------------------------------------
"""

import math
import time
from collections import deque

import rclpy
import smbus2
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu

PWR_MGMT_1 = 0x6B
SIGNAL_PATH_RESET = 0x68   # gyro/accel/temp 신호 경로 리셋
CONFIG = 0x1A
SMPLRT_DIV = 0x19
GYRO_CONFIG = 0x1B
ACCEL_XOUT_H = 0x3B
ACCEL_SCALE = 16384.0          # ±2g
GYRO_SCALE = 131.0             # ±250 deg/s
G = 9.80665


def to_signed(v):
    return v - 65536 if v > 32767 else v


def rot_align(v, target=(0.0, 0.0, 1.0)):
    """v 를 target 으로 보내는 최소 회전행렬 (Rodrigues).

    측정된 중력벡터를 +z 로 보내면, 그 회전을 6축에 그대로 적용해
    '수평으로 장착된 것과 같은' 값을 얻는다.
    """
    n = math.sqrt(sum(c * c for c in v))
    if n < 1e-9:
        return [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    a = [c / n for c in v]
    b = list(target)
    ax = [a[1] * b[2] - a[2] * b[1],
          a[2] * b[0] - a[0] * b[2],
          a[0] * b[1] - a[1] * b[0]]
    s = math.sqrt(sum(c * c for c in ax))
    c = sum(a[i] * b[i] for i in range(3))
    if s < 1e-9:                       # 이미 정렬됐거나 정반대
        if c > 0:
            return [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
        return [[1, 0, 0], [0, -1, 0], [0, 0, -1]]
    k = [c_ / s for c_ in ax]
    K = [[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]]
    th = math.atan2(s, c)
    sn, cs = math.sin(th), 1.0 - math.cos(th)
    R = [[0.0] * 3 for _ in range(3)]
    for i in range(3):
        for j in range(3):
            kk = sum(K[i][m] * K[m][j] for m in range(3))
            R[i][j] = (1.0 if i == j else 0.0) + sn * K[i][j] + cs * kk
    return R


def mat_vec(R, v):
    return [sum(R[i][j] * v[j] for j in range(3)) for i in range(3)]


class ImuFix(Node):

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
        super().__init__("imu_fix")
        self.bus_num = int(self._num("i2c_bus", 7))
        self.addr = int(self._num("i2c_addr", 0x68))
        self.rate = self._num("rate_hz", 100.0)
        self.dt = 1.0 / self.rate
        self.frame = self._text("frame_id", "imu_link")
        self.topic = self._text("imu_topic", "/imu")
        self.calib_sec = self._num("calib_seconds", 3.0)
        win = int(self._num("median_window", 3))
        self.win = win + 1 if win % 2 == 0 else max(1, win)

        # 정지 판정 — 센서만으로 한다 (cmd_vel 에 기대지 않는다)
        self.gyro_still = self._num("gyro_still_thresh", 0.030)   # rad/s
        self.acc_still = self._num("accel_still_thresh", 0.35)    # m/s^2
        self.still_time = self._num("still_settle_sec", 0.30)
        # 정지 중 바이어스 추종 속도. 기존 노드는 0.001 로 온도 변화를 못 따라갔다.
        self.bias_alpha = self._num("bias_alpha", 0.02)
        self.zero_still = self._flag("zero_gyro_when_still", True)
        self.fix_scale = self._flag("fix_accel_scale", True)
        self.fix_tilt = self._flag("fix_mount_tilt", True)
        self.log_period = self._num("log_period", 10.0)
        self.startup_retry_sec = max(
            0.0, self._num("startup_retry_sec", 15.0))
        self.startup_retry_interval = max(
            0.1, self._num("startup_retry_interval_sec", 0.5))

        self.bus = None
        self.open_and_setup_bus()

        self.bufs = [deque(maxlen=self.win) for _ in range(6)]
        self.bias = [0.0, 0.0, 0.0]
        self.scale = 1.0
        self.R = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
        self.still_since = None
        self.stationary = True
        self.last_log = 0.0
        self.n_still = 0
        self.dead_count = 0   # |a| ~ 0 연속 횟수 (센서 사망 감지)
        self.last_chip_reset = -1e9   # 런타임 자동 리셋 시각 (30초 간격 제한)
        self.n_move = 0

        self.pub = self.create_publisher(Imu, self.topic, 20)
        while not self.calibrate():
            self.get_logger().error(
                "1초 뒤 IMU 를 리셋하고 보정을 다시 시도합니다 — 성공할 "
                "때까지 cartographer 는 기다립니다. 반복되면 배선을 다시 "
                "잡아 주세요.")
            time.sleep(1.0)
            try:
                self.setup()
            except OSError as error:
                self.get_logger().error("IMU 리셋 실패(I2C 오류: %s)" % error)
        self.create_timer(self.dt, self.tick)

    # ---------------- 하드웨어 ----------------
    def open_and_setup_bus(self):
        """Open the MPU bus, tolerating a short boot-time I2C outage.

        The navigation launch used to lose /imu permanently when the first
        SMBus open or wake-up write failed once.  Cartographer then remained
        alive while waiting for IMU data and never produced /map.  Retry for a
        bounded interval and keep the final hardware error in the traceback.
        """
        deadline = time.monotonic() + self.startup_retry_sec
        attempt = 0
        last_error = None
        while rclpy.ok():
            attempt += 1
            try:
                self.bus = smbus2.SMBus(self.bus_num)
                self.setup()
                if attempt > 1:
                    self.get_logger().info(
                        "IMU I2C 연결 복구: /dev/i2c-%d addr=0x%02X "
                        "(%d회 시도)"
                        % (self.bus_num, self.addr, attempt))
                return
            except OSError as error:
                last_error = error
                if self.bus is not None:
                    try:
                        self.bus.close()
                    except Exception:
                        pass
                    self.bus = None
                remaining = deadline - time.monotonic()
                if self.startup_retry_sec <= 0.0 or remaining <= 0.0:
                    break
                self.get_logger().warning(
                    "IMU I2C 초기화 실패(%d회): /dev/i2c-%d "
                    "addr=0x%02X: %s — %.1f초 뒤 재시도"
                    % (attempt, self.bus_num, self.addr, error,
                       min(self.startup_retry_interval, remaining)))
                time.sleep(min(self.startup_retry_interval, remaining))
        raise RuntimeError(
            "IMU I2C 초기화 실패: /dev/i2c-%d addr=0x%02X: %s"
            % (self.bus_num, self.addr, last_error or "ROS context stopped"))

    def setup(self):
        # ★ 걸린(hung) 칩 복구 — 배선 순간단선 뒤 I2C ACK 는 되는데 레지스터가
        #   고정값/0 을 뱉는 상태가 실제로 있었다(2026-08-05, 이 데이터가
        #   cartographer 에 들어가 정지 로봇 pose 가 지도 밖으로 흘렀다).
        #   DEVICE_RESET + 신호경로 리셋을 먼저 걸어 항상 깨끗하게 시작한다.
        self.bus.write_byte_data(self.addr, PWR_MGMT_1, 0x80)  # DEVICE_RESET
        time.sleep(0.15)
        self.bus.write_byte_data(self.addr, SIGNAL_PATH_RESET, 0x07)
        time.sleep(0.15)
        self.bus.write_byte_data(self.addr, PWR_MGMT_1, 0)
        time.sleep(0.05)
        self.bus.write_byte_data(self.addr, CONFIG, 0x03)     # DLPF 42Hz
        self.bus.write_byte_data(self.addr, SMPLRT_DIV, 9)    # 100 Hz
        self.bus.write_byte_data(self.addr, GYRO_CONFIG, 0x00)
        time.sleep(0.05)

    def read_raw(self):
        d = self.bus.read_i2c_block_data(self.addr, ACCEL_XOUT_H, 14)
        ax = to_signed((d[0] << 8) | d[1]) / ACCEL_SCALE * G
        ay = to_signed((d[2] << 8) | d[3]) / ACCEL_SCALE * G
        az = to_signed((d[4] << 8) | d[5]) / ACCEL_SCALE * G
        gx = to_signed((d[8] << 8) | d[9]) / GYRO_SCALE * math.pi / 180.0
        gy = to_signed((d[10] << 8) | d[11]) / GYRO_SCALE * math.pi / 180.0
        gz = to_signed((d[12] << 8) | d[13]) / GYRO_SCALE * math.pi / 180.0
        return [ax, ay, az, gx, gy, gz]

    def read_med(self):
        s = self.read_raw()
        out = []
        for i, v in enumerate(s):
            self.bufs[i].append(v)
            b = sorted(self.bufs[i])
            out.append(b[len(b) // 2])
        return out

    # ---------------- 기동 보정 ----------------
    def calibrate(self):
        n = max(int(self.calib_sec * self.rate), 50)
        self.get_logger().info(
            "IMU 보정 시작 — %.1f초 동안 로봇을 **완전히 정지** 시켜 주세요."
            % self.calib_sec)
        sa = [0.0, 0.0, 0.0]
        sg = [0.0, 0.0, 0.0]
        ok = 0
        dead = 0
        for _ in range(n):
            try:
                s = self.read_raw()
            except OSError:
                continue
            # ★ 죽은 표본(배선 불량, |a|~0)은 하나도 평균에 넣지 않는다.
            #   섞어서 평균하면 중력이 5~6 같은 어중간한 값이 되어 아래
            #   대역 검사를 통과해 버린다(실측: 기울기 52도짜리 오염 보정).
            if math.sqrt(s[0] * s[0] + s[1] * s[1] + s[2] * s[2]) < 3.0:
                dead += 1
                time.sleep(self.dt)
                continue
            for i in range(3):
                sa[i] += s[i]
                sg[i] += s[3 + i]
            ok += 1
            time.sleep(self.dt)
        if dead > 0:
            self.get_logger().warn(
                "보정 중 죽은 표본 %d개 / 살아있는 표본 %d개 — 배선 접촉이 "
                "불안정합니다." % (dead, ok))
        total = ok + dead
        if ok < 50 or (total > 0 and ok < 0.7 * total):
            self.get_logger().error(
                "IMU 보정 실패 — 살아있는 표본 부족(%d/%d). I2C 배선 확인."
                % (ok, total))
            return False
        gvec = [v / ok for v in sa]
        self.bias = [v / ok for v in sg]
        mag = math.sqrt(sum(c * c for c in gvec))
        # ★ 보정 유효성 검사 — 배선 접촉 불량이면 보정 중 표본 일부/전부가
        #   0 으로 읽혀 중력이 3.6 같은 값이 나오고, 그대로 진행하면 스케일
        #   x2.7 / 기울기 57도짜리 오염된 보정으로 이후 모든 데이터가
        #   망가진다(2026-08-05 실측). 이 개체의 정상 중력은 ~8.8 이다.
        if not 7.0 <= mag <= 12.0:
            self.get_logger().error(
                "IMU 보정 실패 — 측정 중력 %.3f m/s^2 (정상 ~8.8). 배선 접촉 "
                "불량으로 죽은 표본이 섞였습니다. 오염된 보정은 쓰지 않습니다."
                % mag)
            return False
        if self.fix_scale and mag > 1.0:
            self.scale = G / mag
        if self.fix_tilt:
            self.R = rot_align(gvec)
        tilt = math.degrees(math.acos(
            max(-1.0, min(1.0, gvec[2] / mag)))) if mag > 1e-6 else 0.0
        self.get_logger().info(
            "IMU 보정 완료 (표본 %d개)\n"
            "  자이로 바이어스 x %+.4f  y %+.4f  z %+.4f rad/s "
            "(z 는 보정 안 하면 1분에 %+.1f도 누적)\n"
            "  중력 크기 %.3f -> 스케일 x%.4f 로 %.3f 에 맞춤\n"
            "  장착 기울기 %.1f도 -> 회전 보정 %s"
            % (ok, self.bias[0], self.bias[1], self.bias[2],
               math.degrees(self.bias[2] * 60.0),
               mag, self.scale, mag * self.scale,
               tilt, "적용" if self.fix_tilt else "끔"))
        return True

    # ---------------- 주기 처리 ----------------
    def _handle_dead_sample(self, now, magnitude):
        """죽은 표본(|a|~0)이 지속되면 칩을 자동 리셋하고 사용자에게 알린다.

        ~2초(200표본) 지속 시 DEVICE_RESET 을 시도한다(30초에 1회 제한,
        검증된 보정값은 유지). 그래도 안 살아나면 배선/모듈 문제다.
        """
        self.dead_count += 1
        if self.dead_count >= 200 and now - self.last_chip_reset > 30.0:
            self.last_chip_reset = now
            self.get_logger().warn(
                "IMU 가속도 크기 %.2f m/s^2 — 칩이 걸린 것으로 보고 "
                "DEVICE_RESET 으로 재초기화합니다 (보정값은 유지)."
                % magnitude)
            try:
                self.setup()
            except OSError as error:
                self.get_logger().error(
                    "IMU 재초기화 실패(I2C 오류: %s) — 배선/전원을 "
                    "확인하세요." % error)
        if self.dead_count == 200 or self.dead_count % 6000 == 0:
            self.get_logger().error(
                "IMU 가 죽은 값(|a|=%.2f)을 내고 있어 발행을 멈췄습니다. "
                "자동 리셋으로도 안 살아나면 배선을 다시 잡거나 모듈을 "
                "교체해야 하고, 급하면 slam_imu:=false 로 실행하세요."
                % magnitude)

    def tick(self):
        try:
            s = self.read_med()
        except OSError:
            return
        # ★ 죽은 표본 차단 — 배선 접촉 불량이면 가속도 레지스터가 0 으로
        #   읽힌다. 그 표본을 cartographer 로 내보내면 pose 가 흘러가므로
        #   발행 자체를 막는다(발행이 멈추면 pose 는 잠깐 멈출 뿐이다).
        #   사망 카운트/자동 리셋은 계속 돌도록 처리만 넘긴다.
        raw_am = math.sqrt(s[0] * s[0] + s[1] * s[1] + s[2] * s[2])
        if raw_am < 3.0:
            self._handle_dead_sample(
                self.get_clock().now().nanoseconds / 1e9, raw_am)
            return
        a = [s[0] * self.scale, s[1] * self.scale, s[2] * self.scale]
        g = [s[3] - self.bias[0], s[4] - self.bias[1], s[5] - self.bias[2]]
        if self.fix_tilt:
            a = mat_vec(self.R, a)
            g = mat_vec(self.R, g)

        # --- 정지 판정: 센서만으로 ---
        gm = math.sqrt(sum(c * c for c in g))
        am = math.sqrt(sum(c * c for c in a))
        quiet = (gm < self.gyro_still) and (abs(am - G) < self.acc_still)
        now = self.get_clock().now().nanoseconds / 1e9
        if quiet:
            if self.still_since is None:
                self.still_since = now
            self.stationary = (now - self.still_since) >= self.still_time
        else:
            self.still_since = None
            self.stationary = False

        if self.stationary:
            self.n_still += 1
            # 정지 중에만 바이어스를 따라간다. 온도가 오르면 바이어스가 변하는데
            # 이걸 안 하면 그 변화가 통째로 heading 오차가 된다.
            for i in range(3):
                self.bias[i] = ((1.0 - self.bias_alpha) * self.bias[i]
                                + self.bias_alpha * s[3 + i])
        else:
            self.n_move += 1

        out = [0.0, 0.0, 0.0] if (self.stationary and self.zero_still) else g

        m = Imu()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.frame
        m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = out
        m.linear_acceleration.x = a[0]
        m.linear_acceleration.y = a[1]
        m.linear_acceleration.z = a[2]
        # 자세는 제공하지 않는다(-1). cartographer 2D 는 각속도와 중력만 쓴다.
        m.orientation_covariance[0] = -1.0
        m.angular_velocity_covariance[0] = 2.5e-5
        m.angular_velocity_covariance[4] = 2.5e-5
        m.angular_velocity_covariance[8] = 2.5e-5
        m.linear_acceleration_covariance[0] = 2.0e-3
        m.linear_acceleration_covariance[4] = 2.0e-3
        m.linear_acceleration_covariance[8] = 2.0e-3
        self.pub.publish(m)

        self.dead_count = 0   # 여기 왔으면 살아 있는 표본이다

        if self.log_period > 0 and now - self.last_log >= self.log_period:
            self.last_log = now
            tot = max(self.n_still + self.n_move, 1)
            self.get_logger().info(
                "IMU: %s | 자이로z %+.4f rad/s | 바이어스z %+.5f | "
                "|a| %.3f | 정지비율 %.0f%%"
                % ("정지" if self.stationary else "이동",
                   out[2], self.bias[2], am, 100.0 * self.n_still / tot))
            self.n_still = self.n_move = 0


def main():
    rclpy.init()
    n = ImuFix()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            n.bus.close()
        except Exception:
            pass
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
