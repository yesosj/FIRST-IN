import sys
import select
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdKeyboardTest(Node):
    def __init__(self):
        super().__init__("cmd_keyboard_test")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        
        # 목표 속도 상태 저장
        self.target_linear = 0.0
        self.target_angular = 0.0
        
        # 마지막으로 키가 눌린 시간
        self.last_key_time = self.get_clock().now()
        
        # 20Hz(0.05초) 주기로 현재 목표 속도를 계속 퍼블리시하는 타이머
        self.create_timer(0.05, self.publish_cmd)

    def publish_cmd(self):
        # 현재 시간 확인
        now = self.get_clock().now()
        
        # 키보드 입력이 0.15초 이상 없으면 키를 뗐다고 판단하고 속도를 0으로 강제 초기화
        if (now - self.last_key_time).nanoseconds / 1e9 > 0.15:
            self.target_linear = 0.0
            self.target_angular = 0.0

        # 속도 발행
        msg = Twist()
        msg.linear.x = self.target_linear
        msg.angular.z = self.target_angular
        self.pub.publish(msg)

    def update_cmd(self, key):
        # 키가 눌릴 때마다 마지막 입력 시간을 갱신
        self.last_key_time = self.get_clock().now()
        
        if key == "w":
            self.target_linear = 2.0
            self.target_angular = 0.0
        elif key == "s":
            self.target_linear = -2.0
            self.target_angular = 0.0
        elif key == "a":
            self.target_linear = 0.0
            self.target_angular = 2.0
        elif key == "d":
            self.target_linear = 0.0
            self.target_angular = -2.0
        elif key == "x" or key == " ":
            self.target_linear = 0.0
            self.target_angular = 0.0

# 엔터키 없이 문자를 즉시 읽어오는 함수
def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    # select를 이용해 0.05초간 입력을 대기 (non-blocking)
    rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main():
    # 원래 터미널 설정을 백업
    settings = termios.tcgetattr(sys.stdin)
    
    rclpy.init()
    node = CmdKeyboardTest()
    
    print("-----------------------------------------")
    print("키를 꾹 누르고 있으면 로봇이 움직입니다.")
    print("키에서 손을 떼면 자동으로 멈춥니다.")
    print("w: forward, s: backward, a: left, d: right")
    print("x: stop, q: quit")
    print("-----------------------------------------")
    
    try:
        while rclpy.ok():
            key = get_key(settings)
            
            if key == 'q':  # 'q'를 누르면 종료
                break
            elif key != '': # 키 입력이 있으면 속도 업데이트
                node.update_cmd(key.lower())
            
            # ROS 2의 타이머 콜백(publish_cmd)이 동작할 수 있도록 스핀 처리
            rclpy.spin_once(node, timeout_sec=0.0)
            
    except KeyboardInterrupt:
        pass
    finally:
        # 종료 전 안전을 위해 정지 명령 1회 전송
        node.target_linear = 0.0
        node.target_angular = 0.0
        node.publish_cmd()
        
        # 터미널 설정을 원래대로 복구
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
