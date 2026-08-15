import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import smbus2
import math
import time

from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

MPU6050_ADDR = 0x68
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43


class ImuPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        self.bus = smbus2.SMBus(7)  # i2cdetect로 확인한 버스 번호로 수정
        self.bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0)  # sleep 해제
        self.publisher_ = self.create_publisher(Imu, '/imu', 10)

        self.dt = 0.02  # 50Hz
        self.timer = self.create_timer(self.dt, self.timer_callback)

        # yaw 적분(누적)용 상태 변수
        self.yaw = 0.0  # rad, 시작 시점을 0으로 초기화
        self.gyro_z_bias = 0.0

        # 로그 출력 주기 제한용 (10틱마다 한 번, 약 5Hz)
        self._log_counter = 0

        # x, y 위치 조회용 (Cartographer가 발행하는 map -> base_link TF)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 시작 시 2초간 정지 상태로 두고 자이로 바이어스(오차) 보정
        self.calibrate_gyro_z_bias()

    def calibrate_gyro_z_bias(self):
        """정지 상태에서 자이로 z축 바이어스(0점 오차) 측정 후 보정값 저장"""
        self.get_logger().info('자이로 바이어스 보정 중... 2초간 센서를 움직이지 마세요.')
        samples = []
        start = time.time()
        while time.time() - start < 2.0:
            gz_raw = self.read_word(GYRO_XOUT_H + 4) / 131.0 * math.pi / 180.0
            samples.append(gz_raw)
            time.sleep(0.01)
        self.gyro_z_bias = sum(samples) / len(samples)
        self.get_logger().info(f'보정 완료. gyro_z_bias={self.gyro_z_bias:.5f} rad/s')

    def read_word(self, reg):
        high = self.bus.read_byte_data(MPU6050_ADDR, reg)
        low = self.bus.read_byte_data(MPU6050_ADDR, reg + 1)
        val = (high << 8) + low
        if val >= 0x8000:
            val = -((65535 - val) + 1)
        return val

    def get_xy_position(self):
        """Cartographer가 발행하는 map -> base_link TF에서 x, y 위치만 조회"""
        try:
            t = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time())
            x = t.transform.translation.x
            y = t.transform.translation.y
            return x, y
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None, None

    def timer_callback(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        ax = self.read_word(ACCEL_XOUT_H) / 16384.0 * 9.80665
        ay = self.read_word(ACCEL_XOUT_H + 2) / 16384.0 * 9.80665
        az = self.read_word(ACCEL_XOUT_H + 4) / 16384.0 * 9.80665

        gx = self.read_word(GYRO_XOUT_H) / 131.0 * math.pi / 180.0
        gy = self.read_word(GYRO_XOUT_H + 2) / 131.0 * math.pi / 180.0
        gz = self.read_word(GYRO_XOUT_H + 4) / 131.0 * math.pi / 180.0

        # 바이어스 보정 후 자이로 z축 값을 시간에 따라 적분 -> yaw 누적
        gz_corrected = gz - self.gyro_z_bias
        self.yaw += gz_corrected * self.dt

        # -pi ~ pi 범위로 정규화
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az
        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz

        # yaw만 채우고 roll/pitch는 미계산 -> orientation 유효 표시
        half_yaw = self.yaw / 2.0
        msg.orientation.z = math.sin(half_yaw)
        msg.orientation.w = math.cos(half_yaw)
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation_covariance[0] = -1.0  # Cartographer가 이 orientation을 사용하지 않도록 무효 처리

        self.publisher_.publish(msg)

        # yaw + x,y 위치 로그 출력 (10틱마다 한 번 = 약 5Hz)
        self._log_counter += 1
        if self._log_counter >= 10:
            self._log_counter = 0
            yaw_deg = math.degrees(self.yaw)
            x, y = self.get_xy_position()

            if x is not None:
                self.get_logger().info(
                    f'[MPU6050 Yaw] {yaw_deg:7.2f} deg | '
                    f'[Cartographer Position] x={x:6.3f} m, y={y:6.3f} m'
                )
            else:
                self.get_logger().info(
                    f'[MPU6050 Yaw] {yaw_deg:7.2f} deg | '
                    f'[Position] map->base_link TF 아직 없음 (cartographer_node 확인 필요)'
                )


def main(args=None):
    rclpy.init(args=args)
    node = ImuPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
