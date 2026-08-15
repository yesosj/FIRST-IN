import serial

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


PORT = "/dev/ttyTHS1"
BAUD = 115200

MAX_PWM = 255

# /cmd_vel 값이 1.0일 때 PWM 255가 되도록 설정
LINEAR_TO_PWM = 255.0
ANGULAR_TO_PWM = 255.0

# UART는 50Hz로 계속 송신
SEND_HZ = 50.0

# /cmd_vel이 0.1초 동안 안 들어오면 Jetson이 정지 명령 전송
CMD_TIMEOUT_SEC = 0.1

# UART 연결이 끊어졌을 때 재연결 시도 간격
RECONNECT_INTERVAL_SEC = 1.0


def clamp(value, min_value=-255, max_value=255):
    return max(min_value, min(max_value, int(value)))


class UartMotorCmdVelNode(Node):
    def __init__(self):
        super().__init__("uart_motor_cmd_vel_node")

        # 포트가 이미 사용 중이어도 크래시하지 않고 타이머에서 재연결을 시도한다
        self.ser = None
        self.last_reconnect_time = self.now_sec()
        self.connect_serial()

        self.left_pwm = 0
        self.right_pwm = 0

        self.last_cmd_time = self.now_sec()
        self.last_log_time = self.now_sec()

        self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)
        self.create_timer(1.0 / SEND_HZ, self.timer_callback)

        self.get_logger().info(f"UART motor node started: {PORT}, {BAUD}")
        self.get_logger().info("Listening to /cmd_vel")

    def now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def cmd_vel_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z

        left = linear * LINEAR_TO_PWM - angular * ANGULAR_TO_PWM
        right = linear * LINEAR_TO_PWM + angular * ANGULAR_TO_PWM

        self.left_pwm = clamp(left, -MAX_PWM, MAX_PWM)
        self.right_pwm = clamp(right, -MAX_PWM, MAX_PWM)
        self.last_cmd_time = self.now_sec()

    def timer_callback(self):
        now = self.now_sec()

        if now - self.last_cmd_time > CMD_TIMEOUT_SEC:
            self.left_pwm = 0
            self.right_pwm = 0

        if self.ser is None:
            if now - self.last_reconnect_time >= RECONNECT_INTERVAL_SEC:
                self.last_reconnect_time = now
                self.connect_serial()
        else:
            try:
                self.send_motor(self.left_pwm, self.right_pwm)
            except (serial.SerialException, OSError) as e:
                self.get_logger().error(f"UART write failed: {e}")
                self.close_serial()
                # 끊기기 직전 명령이 재연결 직후 그대로 나가지 않도록 정지로 리셋
                self.left_pwm = 0
                self.right_pwm = 0
                self.last_reconnect_time = now

        if now - self.last_log_time > 1.0:
            self.last_log_time = now
            state = "connected" if self.ser is not None else "disconnected"
            self.get_logger().info(
                f"uart motor left={self.left_pwm}, right={self.right_pwm} ({state})"
            )

    def connect_serial(self):
        self.close_serial()
        try:
            self.ser = serial.Serial(PORT, BAUD, timeout=0.01)
            self.get_logger().info(f"UART connected: {PORT}, {BAUD}")
            return True
        except (serial.SerialException, OSError) as e:
            self.ser = None
            self.get_logger().error(f"UART connect failed: {e}")
            return False

    def close_serial(self):
        if self.ser is None:
            return
        try:
            self.ser.close()
        except Exception:
            pass
        self.ser = None

    def send_motor(self, left, right):
        if self.ser is None:
            return
        command = f"M {left} {right}\n"
        self.ser.write(command.encode("ascii"))

    def destroy_node(self):
        try:
            self.send_motor(0, 0)
        except Exception:
            pass
        self.close_serial()

        super().destroy_node()


def main():
    rclpy.init()
    node = UartMotorCmdVelNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
