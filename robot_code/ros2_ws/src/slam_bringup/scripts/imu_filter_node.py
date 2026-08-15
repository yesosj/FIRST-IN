#!/usr/bin/python3
"""MPU6050 IMU 노드 (필터링 + 정지 게이팅 버전).

기존 imu_publisher/imu_node.py 를 대체한다. 원본은 건드리지 않는다.

원본 대비 바뀐 점
-----------------
1. **바이어스가 실제로 발행 값에 반영된다.**
   원본은 `gz - bias` 를 내부 yaw 적분에만 쓰고, 정작 발행하는
   `msg.angular_velocity.z` 에는 **보정 안 된 raw gz** 를 넣었다.
   Cartographer 는 orientation 을 무효(covariance[0] = -1)로 보고
   angular_velocity 만 쓰기 때문에, 바이어스가 그대로 SLAM 에 들어가
   가만히 있어도 맵이 계속 돌아가는 드리프트가 생긴다. -> 3축 모두 보정해서 발행.

2. **정지 게이팅 (ZUPT).**
   /cmd_vel 이 안 들어오거나 0 이면 로봇이 정지 상태로 보고
   - 각속도를 0 으로 발행해서 적분이 아예 누적되지 않게 하고
   - 그 시간을 이용해 자이로 바이어스를 EMA 로 계속 재추정한다.
   모터 노드(/cmd_vel 구독자)가 아예 안 떠 있으면 항상 정지로 간주한다.
   MPU6050 바이어스는 온도에 따라 변하므로, 시작 시 2초 보정만으로는 부족하다.
   단, 명령이 없어도 자이로가 임계값(gyro_motion_threshold) 이상 돌면 '이동'으로
   본다. 손으로 밀거나 부딪혀서 도는 실제 회전까지 0 으로 지워버리면
   맵이 통째로 틀어지기 때문이다.

3. **중위값 필터 + 버스트 읽기.**
   원본은 한 샘플에 I2C 바이트 읽기를 12번 했다(레지스터당 2번씩 6축).
   여기서는 0x3B 부터 14바이트를 한 번에 읽어 트랜잭션 1회로 끝낸다.
   덕분에 샘플 간 시간 왜곡이 줄고 100Hz 도 여유 있다.
   그 위에 홀수 창 중위값 필터로 I2C 튐(스파이크)을 제거한다.

4. DLPF(저역통과) 를 켜서 센서 자체 노이즈 대역을 좁힌다.
"""

import math
import time
from collections import deque

import rclpy
import smbus2
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf2_ros import (Buffer, ConnectivityException, ExtrapolationException,
                     LookupException, TransformListener)

# MPU6050 레지스터
PWR_MGMT_1 = 0x6B
CONFIG = 0x1A
GYRO_CONFIG = 0x1B
SMPLRT_DIV = 0x19
ACCEL_XOUT_H = 0x3B

ACCEL_SCALE = 16384.0          # ±2g
GYRO_SCALE = 131.0             # ±250 deg/s
G = 9.80665


def to_signed(value):
    return value - 65536 if value >= 0x8000 else value


class FilteredImuPublisher(Node):

    def __init__(self):
        super().__init__('imu_filter')

        self.declare_parameter('i2c_bus', 7)
        self.declare_parameter('i2c_addr', 0x68)
        self.declare_parameter('rate_hz', 100.0)
        self.declare_parameter('median_window', 3)
        self.declare_parameter('calib_seconds', 2.0)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('cmd_timeout', 0.3)
        self.declare_parameter('linear_eps', 0.01)
        self.declare_parameter('angular_eps', 0.01)
        self.declare_parameter('zero_rate_when_stationary', True)
        self.declare_parameter('require_cmd_vel_publisher', True)
        self.declare_parameter('bias_alpha', 0.001)
        self.declare_parameter('gyro_motion_threshold', 0.05)
        self.declare_parameter('bias_settle_time', 0.5)
        self.declare_parameter('log_period', 1.0)

        p = self.get_parameter
        self.rate_hz = p('rate_hz').value
        self.dt = 1.0 / self.rate_hz
        self.cmd_timeout = p('cmd_timeout').value
        self.linear_eps = p('linear_eps').value
        self.angular_eps = p('angular_eps').value
        self.zero_when_still = p('zero_rate_when_stationary').value
        self.require_cmd_pub = p('require_cmd_vel_publisher').value
        self.bias_alpha = p('bias_alpha').value
        self.gyro_motion_threshold = p('gyro_motion_threshold').value
        self.bias_settle_time = p('bias_settle_time').value
        self.log_period = p('log_period').value

        window = int(p('median_window').value)
        if window < 1:
            window = 1
        if window % 2 == 0:
            window += 1
            self.get_logger().warn(f'median_window 는 홀수여야 함 -> {window} 로 조정')
        self.window = window
        self.buffers = [deque(maxlen=window) for _ in range(6)]

        self.addr = int(p('i2c_addr').value)
        self.bus = smbus2.SMBus(int(p('i2c_bus').value))
        self.setup_mpu6050()

        self.gyro_bias = [0.0, 0.0, 0.0]
        self.yaw = 0.0

        # 정지 판정 상태
        self.cmd_linear = 0.0
        self.cmd_angular = 0.0
        self.last_cmd_time = -1e9
        self.stationary = True
        self.stationary_since = -1e9
        self.sensor_moving = False
        self.cmd_pub_count = 0

        self.pub = self.create_publisher(Imu, '/imu', 10)
        self.create_subscription(
            Twist, p('cmd_vel_topic').value, self.cmd_vel_callback, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.calibrate(p('calib_seconds').value)

        self.last_log = self.now()
        self.create_timer(self.dt, self.timer_callback)
        self.create_timer(1.0, self.check_cmd_publisher)
        self.get_logger().info(
            f'IMU 필터 노드 시작: {self.rate_hz:.0f}Hz, 중위값창={window}, '
            f'정지게이팅={p("cmd_vel_topic").value}')

    # --- 하드웨어 -------------------------------------------------------
    def setup_mpu6050(self):
        self.bus.write_byte_data(self.addr, PWR_MGMT_1, 0)       # sleep 해제
        time.sleep(0.05)
        # DLPF_CFG=3 -> 자이로 42Hz / 가속도 44Hz 대역폭. 100Hz 샘플링에 적합.
        self.bus.write_byte_data(self.addr, CONFIG, 0x03)
        # DLPF 켜면 기준 1kHz. DIV=9 -> 100Hz 내부 샘플레이트.
        self.bus.write_byte_data(self.addr, SMPLRT_DIV, 9)
        self.bus.write_byte_data(self.addr, GYRO_CONFIG, 0x00)   # ±250 deg/s
        time.sleep(0.05)

    def read_raw(self):
        """0x3B 부터 14바이트 버스트 읽기 -> (ax, ay, az, gx, gy, gz) SI 단위."""
        d = self.bus.read_i2c_block_data(self.addr, ACCEL_XOUT_H, 14)
        ax = to_signed((d[0] << 8) | d[1]) / ACCEL_SCALE * G
        ay = to_signed((d[2] << 8) | d[3]) / ACCEL_SCALE * G
        az = to_signed((d[4] << 8) | d[5]) / ACCEL_SCALE * G
        # d[6], d[7] 은 온도. 사용하지 않음.
        gx = to_signed((d[8] << 8) | d[9]) / GYRO_SCALE * math.pi / 180.0
        gy = to_signed((d[10] << 8) | d[11]) / GYRO_SCALE * math.pi / 180.0
        gz = to_signed((d[12] << 8) | d[13]) / GYRO_SCALE * math.pi / 180.0
        return [ax, ay, az, gx, gy, gz]

    def read_filtered(self):
        """중위값 필터를 통과시킨 6축 값."""
        sample = self.read_raw()
        out = []
        for i, v in enumerate(sample):
            buf = self.buffers[i]
            buf.append(v)
            out.append(sorted(buf)[len(buf) // 2])
        return out

    # --- 보정 ------------------------------------------------------------
    def calibrate(self, seconds):
        self.get_logger().info(f'자이로 바이어스 보정 중... {seconds:.0f}초간 센서를 움직이지 마세요.')
        acc = [0.0, 0.0, 0.0]
        n = 0
        start = time.time()
        while time.time() - start < seconds:
            s = self.read_raw()
            for i in range(3):
                acc[i] += s[3 + i]
            n += 1
            time.sleep(0.005)
        self.gyro_bias = [a / max(n, 1) for a in acc]
        self.get_logger().info(
            f'보정 완료 ({n}샘플). bias x={self.gyro_bias[0]:+.5f} '
            f'y={self.gyro_bias[1]:+.5f} z={self.gyro_bias[2]:+.5f} rad/s')

    def update_bias(self, gyro):
        """정지 중에만 호출. EMA 로 바이어스를 아주 천천히 따라간다(온도 드리프트 대응).

        alpha=0.001, 100Hz 기준 시정수 약 10초. 이보다 빠르게 잡으면
        노이즈나 짧은 실제 회전이 바이어스로 흡수되어 오히려 드리프트가 커진다.
        """
        a = self.bias_alpha
        for i in range(3):
            self.gyro_bias[i] = (1.0 - a) * self.gyro_bias[i] + a * gyro[i]

    # --- 정지 판정 --------------------------------------------------------
    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def cmd_vel_callback(self, msg):
        self.cmd_linear = msg.linear.x
        self.cmd_angular = msg.angular.z
        self.last_cmd_time = self.now()

    def check_cmd_publisher(self):
        self.cmd_pub_count = self.count_publishers(
            self.get_parameter('cmd_vel_topic').value)

    def cmd_says_moving(self):
        """모터 명령이 살아있고 0 이 아닐 때만 '움직이는 중'으로 본다."""
        if self.require_cmd_pub and self.cmd_pub_count == 0:
            return False        # 명령 주는 노드 자체가 안 떠 있음 -> 정지 취급
        if self.now() - self.last_cmd_time > self.cmd_timeout:
            return False        # 명령이 끊김 -> 모터 노드도 정지 명령을 보냄
        return (abs(self.cmd_linear) > self.linear_eps
                or abs(self.cmd_angular) > self.angular_eps)

    def gyro_says_moving(self, gyro):
        """센서 자체로 본 회전 여부.

        cmd_vel 만 믿으면 손으로 밀거나 로봇이 부딪혀 돌아갈 때
        실제 회전을 0 으로 지워버려서 맵이 크게 틀어진다.
        자이로가 임계값을 넘으면 명령이 없어도 '이동'으로 본다.
        """
        return abs(gyro[2] - self.gyro_bias[2]) > self.gyro_motion_threshold

    # --- 메인 루프 --------------------------------------------------------
    def timer_callback(self):
        s = self.read_filtered()
        accel, gyro = s[0:3], s[3:6]

        # 명령이 없어도(손으로 밀기 등) 자이로가 실제 회전을 보면 '이동'으로 본다.
        self.sensor_moving = self.gyro_says_moving(gyro)
        moving = self.cmd_says_moving() or self.sensor_moving
        now = self.now()

        if moving:
            self.stationary_since = now
        self.stationary = not moving

        # 정지 상태가 충분히 이어진 뒤에만 바이어스를 갱신한다.
        # 멈춘 직후에는 관성/진동이 남아 있어 그 값을 바이어스로 넣으면 오히려 틀어진다.
        if self.stationary and (now - self.stationary_since) > self.bias_settle_time:
            self.update_bias(gyro)

        corrected = [gyro[i] - self.gyro_bias[i] for i in range(3)]

        if self.stationary and self.zero_when_still:
            # 정지 중에는 각속도를 0 으로 내보낸다.
            # 이렇게 해야 Cartographer 의 pose extrapolator 가 제자리에서
            # 조금씩 회전을 누적하는 일이 없어진다.
            published = [0.0, 0.0, 0.0]
        else:
            published = corrected
            self.yaw += corrected[2] * self.dt
            self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'
        msg.linear_acceleration.x = accel[0]
        msg.linear_acceleration.y = accel[1]
        msg.linear_acceleration.z = accel[2]
        msg.angular_velocity.x = published[0]
        msg.angular_velocity.y = published[1]
        msg.angular_velocity.z = published[2]

        half = self.yaw / 2.0
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = math.sin(half)
        msg.orientation.w = math.cos(half)
        # Cartographer 가 이 orientation 을 쓰지 않도록 무효 표시 (원본과 동일)
        msg.orientation_covariance[0] = -1.0

        self.pub.publish(msg)
        self.maybe_log()

    def maybe_log(self):
        now = self.now()
        if now - self.last_log < self.log_period:
            return
        self.last_log = now

        if self.stationary:
            state = '정지(적분 중단)'
        elif self.sensor_moving and not self.cmd_says_moving():
            state = '이동(자이로 감지, 명령 없음)'
        else:
            state = '이동'
        if self.require_cmd_pub and self.cmd_pub_count == 0:
            state += ' / cmd_vel 발행 노드 없음'

        x, y = self.get_xy()
        pos = (f'x={x:6.3f} y={y:6.3f}' if x is not None
               else 'map->base_link TF 없음')
        self.get_logger().info(
            f'[{state}] yaw={math.degrees(self.yaw):7.2f}deg '
            f'bias_z={self.gyro_bias[2]:+.5f} | {pos}')

    def get_xy(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            return t.transform.translation.x, t.transform.translation.y
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None, None


def main(args=None):
    rclpy.init(args=args)
    node = FilteredImuPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
