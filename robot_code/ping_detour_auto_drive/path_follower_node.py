#!/usr/bin/env python3
"""
path_follower_node.py
----------------------------------------------------------
map_nav_test.launch.py (path_planner_node.py) 가 만든 경로 /plan 을 실제로 따라
가는 자율주행 노드. 현재 위치는 SLAM(cartographer)의 TF map->base_link 에서 읽고,
모터는 keyboard_test.py 와 똑같은 인터페이스(/cmd_vel, geometry_msgs/Twist)로 제어한다.

  keyboard_test.py  : 키 입력   -> /cmd_vel (20Hz)
  path_follower_node: 경로+위치 -> /cmd_vel (20Hz)   <-- 이 파일
  UART/node.py      : /cmd_vel  -> "M left right" (좌/우 PWM)

  * UART/node.py 규약: left = linear*255 - angular*255, right = linear*255 + angular*255
    즉 /cmd_vel 값 1.0 이 PWM 255. 그래서 아래 속도 값은 m/s 가 아니라
    '정규화된 모터 출력(0~1)' 이다. 좌/우가 255 를 넘어 잘리면 조향이 틀어지므로
    발행 직전에 바퀴 단위로 정규화한다.

입력
  /plan            (nav_msgs/Path)        추종할 경로 (path_planner_node.py 발행)
  TF map->base_link                       현재 위치 (cartographer)
  /scan            (sensor_msgs/LaserScan) 앞쪽 비상정지용 (끄려면 use_scan_guard:=false)
  /auto_drive/enable (std_msgs/Bool)      false 면 즉시 정지/대기
  /auto_drive/cancel (std_msgs/Empty)     경로 취소 후 정지

출력
  /cmd_vel                  (geometry_msgs/Twist) 모터 명령 (20Hz, 항상 발행)
  /auto_drive/goal_reached  (std_msgs/Bool)  도착 시 true (latched)
  /auto_drive/status        (std_msgs/String) 대기/주행중/장애물정지/도착
  /auto_drive/marker        (visualization_msgs/Marker) 추종점 + 도착 표시(RViz)

동작
  1) /plan 수신 -> 현재 위치에서 가장 가까운 경로점을 찾아 거기서부터 추종 시작
  2) 매 주기(20Hz) lookahead 거리만큼 앞선 경로점을 목표로 pure pursuit 제어
     - 방향 오차가 크면 제자리 회전, 작으면 전진 + 비례 조향
     - 목적지 근처에서 감속
  3) 목적지와의 거리가 goal_tolerance 이내면
     모터 정지 + "목적지에 도착했습니다." 출력 + /auto_drive/goal_reached 발행

차체 충돌 회피
  경로 자체가 path_planner_node.py 에서 차체 폭(0.123m)+여유를 반영해 벽을
  부풀린 뒤 계산되므로, 경로를 정확히 따라가는 것이 1차 방어다.
  그 위에 /scan 앞쪽 부채꼴을 보고 (차체 길이/2 + 여유) 안에 뭐가 들어오면
  전진만 멈추고(회전은 허용) 비켜갈 수 있게 한다.

실행
  python3 path_follower_node.py --ros-args -p max_linear:=0.35 -p goal_tolerance:=0.12

주의: rclpy 는 venv 에 없으므로 /usr/bin/python3 로 자동 재실행한다.
----------------------------------------------------------
"""

import math
import os
import sys

_SYS_PY = "/usr/bin/python3"
try:
    import rclpy  # noqa: F401
except ModuleNotFoundError:
    if os.path.abspath(sys.executable) != _SYS_PY and os.path.exists(_SYS_PY):
        _env = dict(os.environ)
        _env.pop("VIRTUAL_ENV", None)
        os.execve(_SYS_PY, [_SYS_PY] + sys.argv, _env)
    raise

import numpy as np
import rclpy
import tf2_ros
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy, qos_profile_sensor_data)

from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Empty, String
from visualization_msgs.msg import Marker


# --- 상태 ---------------------------------------------------------------
IDLE = "대기(경로 없음)"
DRIVING = "주행중"
BLOCKED = "장애물 정지"
ARRIVED = "도착"


def latched_qos(depth=1):
    return QoSProfile(depth=depth,
                      reliability=QoSReliabilityPolicy.RELIABLE,
                      durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                      history=QoSHistoryPolicy.KEEP_LAST)


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def norm_angle(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


class PathFollower(Node):

    # --- 파라미터 헬퍼 (CLI 에서 0.4 대신 1 처럼 줘도 죽지 않도록 동적 타입) ---
    def _num(self, name, default):
        self.declare_parameter(name, default,
                               ParameterDescriptor(dynamic_typing=True))
        v = self.get_parameter(name).value
        return float(default) if v is None else float(v)

    def _flag(self, name, default):
        self.declare_parameter(name, default,
                               ParameterDescriptor(dynamic_typing=True))
        v = self.get_parameter(name).value
        if isinstance(v, str):
            return v.strip().lower() in ("1", "true", "yes", "on")
        return bool(default) if v is None else bool(v)

    def _text(self, name, default):
        self.declare_parameter(name, default,
                               ParameterDescriptor(dynamic_typing=True))
        v = self.get_parameter(name).value
        return default if v is None else str(v)

    def __init__(self):
        super().__init__("path_follower_node")

        # --- 토픽/프레임 ---
        self.cmd_topic = self._text("cmd_vel_topic", "/cmd_vel")
        self.path_topic = self._text("path_topic", "/plan")
        self.scan_topic = self._text("scan_topic", "/scan")
        self.map_frame = self._text("map_frame", "map")
        self.base_frame = self._text("base_frame", "base_link")

        # --- 제어 ---
        self.hz = self._num("control_hz", 20.0)             # keyboard_test 와 동일
        # speed_scale 은 '최소~최대 구간'을 좁히는 배율이다. 최소값은 정지마찰을
        # 넘기 위한 하한선이라 절대 그 아래로 내려가지 않는다.
        #   max_eff = min + (max - min) * speed_scale
        # 예전처럼 최대값에 그냥 곱하면, speed_scale:=0.6 에서 회전 명령이 0.27
        # (PWM 68) 까지 떨어져 4륜 메카넘이 아예 안 도는 상태가 된다.
        self.speed_scale = self._num("speed_scale", 1.0)
        raw_max_lin = self._num("max_linear", 1.00)      # 1.0 = PWM 255 (최대)
        raw_max_ang = self._num("max_angular", 1.00)     # 1.0 = PWM 255 (최대)
        self.min_lin = self._num("min_linear", 0.70)     # PWM 178
        self.min_ang = self._num("min_angular", 0.80)    # PWM 204 (회전은 토크가 더 필요)
        self.max_lin = max(self.min_lin,
                           self.min_lin + (raw_max_lin - self.min_lin) * self.speed_scale)
        self.max_ang = max(self.min_ang,
                           self.min_ang + (raw_max_ang - self.min_ang) * self.speed_scale)
        self.k_ang = self._num("k_angular", 1.3)
        self.turn_angle = self._num("turn_in_place_angle", 0.60)  # rad
        # 방향 오차가 클 때 어떻게 도는가.
        #   spin : 제자리 회전. 좌우 바퀴가 반대로 돈다(M 255 -255).
        #          4륜 메카넘은 네 바퀴가 바닥을 비벼야 해서 토크/전류가 최대로
        #          필요하고, 이 차체는 PWM 255 에서도 꿈쩍하지 않았다.
        #   arc  : 전진하면서 최대로 조향. 안쪽 바퀴가 절대 역회전하지 않아
        #          (|angular| <= |linear|) 비빔이 없고 전류도 훨씬 적다.
        #          한쪽 바퀴 정지 + 반대쪽 전속 -> 회전반경 = 바퀴간격/2 ≈ 0.06 m.
        self.turn_mode = self._text("turn_mode", "arc").strip().lower()
        if self.turn_mode not in ("arc", "spin"):
            self.turn_mode = "arc"

        # ★ 새 경로를 받은 '첫 시작'에만 제자리 회전으로 방향을 맞춘다.
        #   평상시(경로 중간의 좌/우회전)는 turn_mode 그대로 arc 로 돈다.
        #   우회로는 왔던 길을 되돌아가는 반전이 많은데, arc 는
        #   |angular| <= |linear| 제약 때문에 최소 회전반경(약 0.06 m)이 있어
        #   반전을 못 한다 — 그대로 두면 방금 감지한 장애물 쪽으로 전진한다.
        #   새 경로가 올 때만(= 최초 목적지, 우회로 발행) 걸리므로 평소 주행에는
        #   영향이 없다.
        self.spin_on_new = self._flag("spin_on_new_path", True)
        # 이 각도를 넘으면 제자리 회전을 시작한다(기본은 turn_in_place_angle).
        self.spin_enter = self._num("spin_enter_angle", 0.0)
        if self.spin_enter <= 0.0:
            self.spin_enter = self.turn_angle
        # 이 각도 아래로 떨어지면 끝낸다. 들어가는 각도와 달라야(히스테리시스)
        # 도는 도중에 arc 로 새지 않는다. 기본은 key_turn_on(10도) — 아래 참고.
        self.spin_exit = self._num("spin_exit_angle", 0.0)
        # 회전이 물리적으로 안 될 때 영원히 갇히지 않도록 하는 안전장치.
        # 이 시간을 넘기면 경고하고 풀어 준다(그 뒤는 turn_mode 대로).
        self.spin_timeout = self._num("spin_latch_timeout", 15.0)
        # 제자리 회전 출력. 0 이면 예전처럼 k_angular x 오차(비례).
        # 기본 1.0 = 항상 최대. 언제 끊을지는 관성 예측이 담당하므로
        # 출력까지 줄이면 정지마찰 구간에서 그대로 서 버린다.
        self.spin_power = self._num("spin_power", 1.0)
        # 정지마찰 탈출용 흔들기. 회전 명령을 내는데 각도가 안 변하면
        # 아주 짧게 전진량을 줘서 바퀴를 구르게 한다(운동마찰로 전환).
        self.spin_stall_sec = self._num("spin_stall_sec", 2.0)
        self.spin_stall_deg = self._num("spin_stall_deg", 3.0)
        self.spin_nudge_sec = self._num("spin_nudge_sec", 0.35)
        self.spin_nudge_lin = self._num("spin_nudge_lin", -0.75)
        self.spin_latch = False       # 지금 '첫 시작 정렬' 중인가
        self.spin_since = 0.0
        self.new_path_pending = False  # 새 경로를 받았고 아직 판정 전
        self.stall_yaw = None         # 흔들기 판정용 기준 각도
        self.stall_t = 0.0
        self.nudge_until = 0.0
        self.nudge_sign = 1.0
        self.nudge_count = 0

        # --- 제어 방식 -----------------------------------------------------
        #   keys   : keyboard_test.py 의 w / a / s / d 와 '완전히 같은 명령'만 낸다.
        #            w = (2.0, 0) / a = (0, +2.0) / d = (0, -2.0) / 정지 = (0, 0)
        #            섞지 않고 하나씩만 내므로 UART 로 나가는 바이트가 키보드 조작과
        #            동일하다(M 255 255 / M -255 255 / M 255 -255 / M 0 0).
        #            비례제어가 없으니 회전은 펄스(껐다 켰다)로 오버슛을 줄인다.
        #   smooth : 전진과 회전을 섞는 비례제어 (turn_mode 가 여기에 적용된다)
        self.control_mode = self._text("control_mode", "keys").strip().lower()
        if self.control_mode not in ("keys", "smooth"):
            self.control_mode = "keys"
        self.key_lin = self._num("key_linear", 2.0)    # keyboard_test 의 w/s 값
        self.key_ang = self._num("key_angular", 2.0)   # keyboard_test 의 a/d 값
        # 회전 시작/종료 임계값[rad]. 기본 10도 / 5도.
        # 좁힐수록 경로를 더 정확히 따라가지만, 관성으로 지나치면 다시 반대로
        # 돌아 헌팅하기 쉬워진다. 그래서 아래 turn_lead 자동학습이 같이 있어야 한다.
        self.key_turn_on = self._num("key_turn_on", math.radians(10.0))
        self.key_turn_off = self._num("key_turn_off", math.radians(5.0))
        # 제자리 회전의 관성 예측 정지는 key_turn_off(5도)를 겨냥해 명령을 끊는다.
        # 나가는 기준을 5도로 잡으면 딱 그 지점에서 걸려 좌우로 헌팅할 수 있어
        # key_turn_on(10도)을 쓴다. 그 뒤는 전진하며 비례 조향으로 마저 맞춘다.
        if self.spin_exit <= 0.0:
            self.spin_exit = self.key_turn_on
        self.turn_pulse_on = int(self._num("turn_pulse_on", 4))    # 회전 ON 주기수
        # ★ 회전 OFF 주기수. 0 = 끊지 않고 연속 회전 (기본).
        #   예전 기본값 2 는 20Hz 기준 200ms 돌고 100ms 완전정지(듀티 67%)라,
        #   차체 위에 무게가 얹히면 정지할 때마다 정지마찰을 다시 깨야 해서
        #   제자리 회전이 아예 안 됐다. 출력 부족이 아니다 — keys 모드는
        #   angular 2.0 을 그대로 발행하고 node.py 가 2.0x255=510 -> 255 로
        #   자르므로 회전 중 PWM 은 이미 최대치다. 끊김이 원인이었다.
        #   오버슛이 심하면 1~2 로 올리면 된다.
        self.turn_pulse_off = int(self._num("turn_pulse_off", 0))
        self.fwd_pulse_on = int(self._num("fwd_pulse_on", 2))      # 목적지 근처 전진 ON
        self.fwd_pulse_off = int(self._num("fwd_pulse_off", 2))    # 목적지 근처 전진 OFF
        self.turning = False
        self.pulse_i = 0

        # --- 후진 허용 -------------------------------------------------------
        # 목적지가 뒤쪽에 있으면 제자리에서 180도 도는 것은 순전한 낭비다.
        # 그냥 뒤로 가면 된다(keyboard_test 의 s = M -255 -255).
        # 매 주기 '앞으로 갈 때의 방향오차'와 '뒤로 갈 때의 방향오차'를 비교해
        # 덜 도는 쪽을 고른다. 경계에서 앞뒤로 덜덜거리지 않도록 히스테리시스.
        # 기본은 false = 항상 앞을 보고 주행한다. 목적지가 뒤에 있으면 그만큼
        # 돌아야 하는데, 그 회전이 아깝다면 turn_while_moving 을 켜거나
        # (제자리에서 도는 대신 전진하며 호를 그림) start_yaw 정렬을 확인할 것.
        self.allow_reverse = self._flag("allow_reverse", False)
        self.rev_hyst = self._num("reverse_hysteresis", 0.30)   # rad
        self.reversing = False
        # keys 모드에서 큰 방향전환을 할 때, 제자리 회전만 하지 않고 전진 펄스를
        # 섞어서 '전진하며 도는' 호를 그린다. 회전 반경만큼 공간이 필요하다.
        self.turn_fwd = int(self._num("turn_forward_pulse", 0))

        # --- 방향 부호 보정 -------------------------------------------------
        # 모터 좌/우가 뒤바뀌어 배선돼 있으면 angular>0(좌회전 명령)에 로봇이
        # 오른쪽으로 돈다. 그러면 조향이 오차를 줄이는 게 아니라 키워서
        # (양의 되먹임) 경로에서 계속 벗어난다. 그럴 때 true 로 준다.
        self.invert_ang = self._flag("invert_angular", False)
        self.invert_lin = self._flag("invert_linear", False)

        # --- 헤딩 오프셋 보정 (라이다가 차체에 대해 돌아 장착된 경우) ----------
        # slam.launch.py 의 base_link->laser_frame static TF 는 --yaw 0 이라
        # cartographer 는 'base_link 의 +x = 라이다의 +x' 로 믿는다. 라이다가
        # 차체에 대해 yaw 로 돌아 붙어 있으면 base_link +x 는 로봇의 앞이 아니다.
        # 그러면 위치추적은 완벽해 보이는데 제어만 어긋나서, 크게 돌고 경로를
        # 벗어나는 정확히 그 증상이 난다.
        #   실제 로봇의 앞 방향(map 기준) = TF yaw + heading_offset
        # heading_check.py 로 측정한 값을 도 단위로 넣는다.
        self.head_off = math.radians(self._num("heading_offset", 0.0))
        # 자동 진단: 전진 명령을 냈을 때 '실제 이동 방향'과 '내가 믿는 앞 방향'의 차
        self.fwd_ref = None            # (x, y, yaw) 전진 시작 시점
        self.fwd_sin = 0.0
        self.fwd_cos = 0.0
        self.fwd_n = 0
        self.fwd_reported = False

        # --- 첫 동작은 무조건 전진 (기본 꺼짐) --------------------------------
        # 켜면 새 경로를 받았을 때 제자리 회전부터 하지 않고 first_forward_dist
        # 만큼 먼저 직진한다. 원래는 기동 직후 180도 헛도는 걸 막으려고 넣었는데,
        # 경로 방향과 상관없이 무조건 앞으로 나가므로 출발부터 경로를 벗어난다.
        # heading_offset 을 제대로 주면 필요 없으므로 기본은 끈다.
        # (이 강제 전진이 heading_offset 자동 보정의 측정 구간이기도 했다.
        #  끄면 그 자동 보정도 안 하므로 heading_offset 을 직접 줘야 한다.
        #  주행 중 보정인 check_forward_direction 은 그대로 동작한다.)
        self.first_fwd = self._flag("first_move_forward", False)
        self.first_dist = self._num("first_forward_dist", 0.10)     # m
        self.first_timeout = self._num("first_forward_timeout", 2.5)  # s
        # 첫 전진으로 '앞이 어디인가'를 자동 측정해서 heading_offset 을 보정할지
        self.auto_head = self._flag("auto_heading_offset", True)
        self.first_active = False
        self.first_origin = None
        self.head_verified = False     # heading_offset 이 실측으로 확인됐나
        self.blocked_since = None      # 앞쪽 막힘이 계속된 시각
        self.blocked_warned = False
        # 교착 감지: 명령은 내는데 실제로 안 움직이는 상태
        self.stuck_ref = None          # (x, y, yaw, t)
        self.stuck_warned = 0.0
        self.stuck_sec = self._num('stuck_timeout', 3.0)
        self.stuck_dist = self._num('stuck_min_move', 0.03)
        # 제자리 회전 판정용. 이만큼 돌았으면 '움직이는 중'으로 본다.
        self.stuck_yaw = math.radians(self._num('stuck_min_turn_deg', 8.0))
        self.first_t0 = 0.0
        # 자동 진단: 보낸 회전명령과 실제 yaw 변화의 상관을 누적해서 판정
        self.rot_prev_yaw = None
        self.yaw_rate = 0.0            # 실측 회전 각속도[rad/s] (평활)
        # 명령을 끊고도 계속 도는 시간[s]. TF 갱신 지연 + 모터 관성.
        # 이 값 x 현재 각속도 = 앞으로 더 돌 각도. 그만큼 미리 끊는다.
        # 더 지나치면 키우고, 목표에 못 미치면 줄인다.
        self.turn_lead = self._num('turn_lead_sec', 0.4)
        # 회전 관성 자동학습용
        self.cut_yaw = None        # 마지막으로 '돌라고' 한 순간의 각도
        self.cut_rate = 0.0        # 그때의 각속도[rad/s]
        self.cut_done = True       # 이번 회전의 오버슛을 이미 쟀는가
        # 이 각속도 아래로 떨어지면 '멈췄다'고 본다
        self.settle_rate = math.radians(self._num('turn_settle_deg', 8.0))
        self.lead_min = self._num('turn_lead_min', 0.15)
        self.lead_max = self._num('turn_lead_max', 1.50)
        self.cut_dir = 0.0         # 마지막 회전 명령의 방향(+1/-1)
        self.turn_end_t = None     # 직전 회전이 끝난 시각
        self.turn_end_dir = 0.0
        # 회전이 끝난 뒤 이 시간 안에 반대로 다시 돌면 '지나쳤다'로 본다
        self.flip_window = self._num('turn_flip_window', 2.0)
        self.rot_prev_t = None
        self.rot_last_cmd = 0.0
        self.rot_score = 0.0
        self.rot_n = 0
        self.rot_reported = False
        self.lookahead = self._num("lookahead", 0.12)             # m
        self.goal_tol = self._num("goal_tolerance", 0.12)         # m
        self.slow_dist = self._num("slow_down_distance", 0.45)    # m
        self.accel_step = self._num("accel_step", 0.04)           # 주기당 증가
        self.decel_step = self._num("decel_step", 0.12)           # 주기당 감소
        self.ang_step = self._num("angular_step", 0.15)
        self.min_wheel = self._num("min_wheel_cmd", 0.80)   # 정지마찰 극복 최소 출력(PWM 204)
        self.max_wheel = self._num("max_wheel_cmd", 1.0)    # PWM 255 = 1.0
        self.search_ahead = self._num("search_ahead", 1.5)  # 경로 재탐색 창[m]
        self.offtrack_warn = self._num("offtrack_warn", 0.50)

        # --- 차체 / 안전 ---
        self.robot_length = self._num("robot_length", 0.23)
        self.robot_width = self._num("robot_width", 0.19)
        self.use_scan_guard = self._flag("use_scan_guard", False)
        self.front_margin = self._num("front_margin", 0.02)
        stop_d = self._num("obstacle_stop_distance", 0.0)
        if stop_d <= 0.0:                       # 0 이면 차체에서 자동 계산
            stop_d = 0.5 * self.robot_length + self.front_margin
        self.stop_dist = stop_d
        self.slow_obs_dist = self._num("obstacle_slow_distance",
                                       self.stop_dist + 0.20)
        self.front_deg = self._num("front_sector_deg", 24.0)   # +-12도
        self.tf_timeout = self._num("tf_timeout", 0.6)

        # --- 상태 ---
        self.path = []
        # 마지막으로 받은 '비어 있지 않은' 경로. 같은 경로 재발행을 가려내는 데 쓴다
        # (빈 경로가 중간에 끼므로 self.path 와 비교하면 항상 다르게 나온다).
        self.last_pts = []
        self.idx = 0
        self.need_global_search = True
        self.state = IDLE
        self.enabled = True
        self.cur_lin = 0.0
        self.cur_ang = 0.0
        self.scan = None
        self.scan_time = None
        self.last_log = 0.0
        self.last_status = ""
        self.arrived_announced = False
        # 정지 상태에서 0 을 몇 번 더 보낼지. 다 보내면 발행을 멈춰서
        # keyboard_test.py 같은 다른 /cmd_vel 발행자와 싸우지 않게 한다.
        # (모터는 UART/node.py 의 0.1초 타임아웃으로 계속 0 을 유지한다)
        self.zero_left = 20
        self.marker_i = 0

        # --- 통신 ---
        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.reach_pub = self.create_publisher(Bool, "/auto_drive/goal_reached",
                                               latched_qos())
        self.status_pub = self.create_publisher(String, "/auto_drive/status",
                                                latched_qos())
        self.marker_pub = self.create_publisher(Marker, "/auto_drive/marker",
                                                latched_qos(depth=5))

        self.create_subscription(Path, self.path_topic, self.on_path,
                                 latched_qos(depth=5))
        self.create_subscription(Bool, "/auto_drive/enable", self.on_enable, 10)
        self.create_subscription(Empty, "/auto_drive/cancel", self.on_cancel, 10)
        if self.use_scan_guard:
            self.create_subscription(LaserScan, self.scan_topic, self.on_scan,
                                     qos_profile_sensor_data)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_timer(1.0 / max(self.hz, 1.0), self.control_loop)
        self.create_timer(3.0, self.check_conflicts)

        self.publish_state(IDLE)
        self.reach_pub.publish(Bool(data=False))
        self.get_logger().info(
            "자율주행 준비 | 모터 %s | 경로 %s | 차체 %.3fx%.3f m"
            % (self.cmd_topic, self.path_topic, self.robot_length, self.robot_width))
        self.get_logger().info(
            "출력 전진 %.2f~%.2f (PWM %d~%d) | 회전 %.2f~%.2f (PWM %d~%d) | "
            "바퀴 최소 %.2f (PWM %d)"
            % (self.min_lin, self.max_lin, int(self.min_lin * 255),
               int(self.max_lin * 255), self.min_ang, self.max_ang,
               int(self.min_ang * 255), int(self.max_ang * 255),
               self.min_wheel, int(self.min_wheel * 255)))
        if self.control_mode == "keys":
            self.get_logger().info(
                "제어방식 keys — keyboard_test.py 의 w/a/d 와 같은 명령만 사용 "
                "(w=%.1f, a/d=%.1f, PWM 255). 회전 시작 %.0f도 / 종료 %.0f도, "
                "회전펄스 %d ON / %d OFF"
                % (self.key_lin, self.key_ang, math.degrees(self.key_turn_on),
                   math.degrees(self.key_turn_off), self.turn_pulse_on,
                   self.turn_pulse_off))
        else:
            self.get_logger().info(
                "제어방식 smooth | 회전방식 %s (%s)"
                % (self.turn_mode,
                   "전진하며 호를 그림 — 안쪽 바퀴 역회전 없음"
                   if self.turn_mode == "arc" else "제자리 회전 — 좌우 바퀴 역방향"))
        self.get_logger().info(
            "lookahead %.2f m | 도착오차 %.2f m | 앞쪽 정지거리 %s"
            % (self.lookahead, self.goal_tol,
               ("%.2f m (+-%.0f도)" % (self.stop_dist, self.front_deg / 2.0))
               if self.use_scan_guard else "사용안함"))
        self.get_logger().info(
            "RViz 에서 '2D Goal Pose' 로 목적지를 찍으면 경로가 생기고 바로 출발합니다.")

    # ================= 회전 방향 자동 진단 =================
    def check_rotation_sign(self, yaw, now):
        """직전에 보낸 회전명령과 실제 yaw 변화의 부호가 맞는지 누적 검사.

        명령 +(좌회전)에 yaw 가 계속 줄어들면 모터 좌/우가 뒤바뀐 것이다.
        그 상태로는 조향이 오차를 키우기만 해서 절대 경로를 못 따라간다.
        """
        if self.rot_prev_yaw is None:
            self.rot_prev_yaw, self.rot_prev_t = yaw, now
            return
        dt = now - self.rot_prev_t
        if dt < 0.04:
            return
        dyaw = norm_angle(yaw - self.rot_prev_yaw)
        self.rot_prev_yaw, self.rot_prev_t = yaw, now

        # 실제 회전 각속도[rad/s]. 회전을 언제 끊어야 딱 맞는지 예측하는 데 쓴다.
        # 자세가 튀므로 지수평활한다.
        self.yaw_rate = 0.6 * self.yaw_rate + 0.4 * (dyaw / dt)

        cmd = self.rot_last_cmd          # 이 dyaw 를 만든 것은 '직전' 명령
        if abs(cmd) < 0.1 or abs(dyaw) < 0.002:
            return
        self.rot_score += math.copysign(1.0, cmd) * dyaw
        self.rot_n += 1

        if self.rot_n >= 40 and not self.rot_reported:
            if self.rot_score < -0.15:
                self.rot_reported = True
                self.get_logger().error(
                    "회전 방향이 반대입니다! 좌회전 명령에 로봇이 오른쪽으로 돕니다 "
                    "(누적 %.2f rad). 이 상태로는 조향이 오차를 키우기만 해서 경로를 "
                    "절대 못 따라갑니다. → %s 로 다시 실행하세요."
                    % (self.rot_score,
                       "invert_angular:=false" if self.invert_ang
                       else "invert_angular:=true"))
            elif self.rot_score > 0.15:
                self.rot_reported = True
                self.get_logger().info(
                    "회전 방향 정상 확인 (누적 %.2f rad)%s"
                    % (self.rot_score,
                       " — invert_angular 보정이 적용된 상태" if self.invert_ang else ""))

    # ================= 전진 방향 자동 진단 =================
    def check_forward_direction(self, x, y, yaw, cmd_lin):
        """전진 명령을 냈을 때 '실제 이동 방향'이 '내가 믿는 앞 방향'과 같은가.

        다르면 base_link 의 +x 가 로봇의 앞이 아니라는 뜻이다(라이다 장착 yaw
        오프셋). 위치추적은 멀쩡해 보이므로 이 검사가 없으면 원인을 찾기 어렵다.
        """
        if cmd_lin <= 1e-6:
            self.fwd_ref = None
            return
        if self.fwd_ref is None:
            self.fwd_ref = (x, y, yaw)
            return
        ax, ay, ayaw = self.fwd_ref
        dx, dy = x - ax, y - ay
        moved = math.hypot(dx, dy)
        if moved < 0.05:               # 방향을 신뢰할 만큼 움직인 뒤에 판정
            return
        err = norm_angle(math.atan2(dy, dx) - ayaw)
        self.fwd_sin += math.sin(err)
        self.fwd_cos += math.cos(err)
        self.fwd_n += 1
        self.fwd_ref = (x, y, yaw)

        if self.fwd_n >= 4 and not self.fwd_reported:
            mean = math.degrees(math.atan2(self.fwd_sin, self.fwd_cos))
            if abs(mean) > 30.0:
                self.fwd_reported = True
                if self.auto_head:
                    # 첫 전진이 벽에 막혀 보정을 못 했더라도, 주행 중 관측으로
                    # 여기서 스스로 고친다. (좁은 방에서는 첫 0.1m 전진이
                    # 라이다 비상정지에 막히는 일이 흔하다)
                    self.head_off = norm_angle(self.head_off + math.radians(mean))
                    self.fwd_sin = self.fwd_cos = 0.0
                    self.fwd_n = 0
                    self.fwd_reported = False
                    self.need_global_search = True
                    self.turning = False
                    self.head_verified = True
                    self.get_logger().error(
                        "★ 자동 보정(주행 중): 전진 명령에 로봇이 %+.0f도 어긋난 "
                        "방향으로 갔습니다(%d회 관측). base_link 의 +x 가 로봇의 앞이 "
                        "아닙니다 — 라이다가 차체에 대해 돌아 장착된 경우입니다. "
                        "heading_offset 을 %+.0f도 로 잡고 계속합니다. "
                        "(고정하려면 heading_offset:=%.1f)"
                        % (mean, self.fwd_n or 4, math.degrees(self.head_off),
                           math.degrees(self.head_off)))
                else:
                    self.get_logger().error(
                        "전진 명령에 로봇이 %+.0f 도 어긋난 방향으로 갑니다 "
                        "(%d회 관측). base_link 의 +x 가 로봇의 앞이 아닙니다 — "
                        "라이다가 차체에 대해 돌아 장착된 경우입니다. "
                        "heading_offset:=%.1f 을 주고 다시 실행하세요. "
                        "(현재 heading_offset=%.1f)"
                        % (mean, self.fwd_n,
                           math.degrees(self.head_off) + mean,
                           math.degrees(self.head_off)))
            elif abs(mean) <= 30.0:
                self.fwd_reported = True
                self.head_verified = True
                self.get_logger().info(
                    "전진 방향 정상 확인 (평균 오차 %+.0f도, %d회 관측)"
                    % (mean, self.fwd_n))

    # ================= 충돌 감지 =================
    def check_conflicts(self):
        """/cmd_vel 을 두 노드가 같이 쏘거나, 모터 노드가 없는 상황을 잡아낸다.

        가장 흔한 사고: keyboard_test.py 를 켜 둔 채로 자율주행을 띄우는 것.
        keyboard_test.py 는 키를 안 눌러도 0 을 20Hz 로 계속 발행하기 때문에,
        UART/node.py 가 '마지막에 온 값'으로 덮어쓰면서 255 와 0 이 25ms 마다
        번갈아 나간다. 모터는 떨기만 하고 로봇은 제자리에 선다.
        """
        try:
            pubs = [i.node_name
                    for i in self.get_publishers_info_by_topic(self.cmd_topic)]
        except Exception:                       # noqa: BLE001
            pubs = []
        others = [n for n in pubs if n != self.get_name()]
        if others:
            self.get_logger().error(
                "%s 에 다른 발행자가 있습니다: %s. keyboard_test.py 처럼 0 을 함께 "
                "쏘는 노드가 있으면 명령이 상쇄되어 로봇이 안 움직입니다 — "
                "그 노드를 끄세요." % (self.cmd_topic, ", ".join(others)),
                throttle_duration_sec=10.0)

        # ★ 개수만 세면 안 된다. 이 장비는 계정 하나를 셋이 같이 써서 팀원 노드가
        #   /cmd_vel 을 구독하기도 하고(web_node 등), goal_path_planner_node 도
        #   제자리 회전 감지용으로 구독한다. 그런 것까지 세면 정상인데도 매번
        #   에러가 뜬다. 진짜 문제는 '모터 노드가 없거나 둘 이상' 인 경우뿐이므로
        #   노드 이름으로 판별한다.
        try:
            names = [i.node_name
                     for i in self.get_subscriptions_info_by_topic(self.cmd_topic)]
        except Exception:                       # noqa: BLE001
            names = []
        motors = [n for n in names
                  if "uart" in n.lower() or "motor" in n.lower()]
        if not names:
            return
        if not motors:
            self.get_logger().error(
                "%s 를 구독하는 모터 노드가 없습니다. UART/node.py 를 실행하세요 "
                "(launch 의 motor:=true 또는 별도 터미널). 지금 구독 중: %s"
                % (self.cmd_topic, ", ".join(names) or "없음"),
                throttle_duration_sec=10.0)
        elif len(motors) > 1:
            self.get_logger().error(
                "%s 를 구독하는 모터 노드가 %d 개입니다(%s). 두 프로세스가 같은 "
                "시리얼 포트에 써서 명령이 깨집니다 — 하나만 남기세요 "
                "(launch 를 motor:=false 로 띄우거나 기존 것을 끄세요)."
                % (self.cmd_topic, len(motors), ", ".join(motors)),
                throttle_duration_sec=10.0)

    # ================= 콜백 =================
    def on_path(self, msg: Path):
        pts = []
        for ps in msg.poses:
            x, y = float(ps.pose.position.x), float(ps.pose.position.y)
            if pts and math.hypot(x - pts[-1][0], y - pts[-1][1]) < 1e-6:
                continue                     # 중복점 제거
            pts.append((x, y))

        frame = msg.header.frame_id or self.map_frame
        if frame != self.map_frame:
            self.get_logger().warn(
                "경로 프레임이 '%s' 입니다('%s' 기대). 그대로 사용합니다."
                % (frame, self.map_frame))

        if len(pts) < 2:
            self.path = []
            self.stop_now()
            self.publish_state(IDLE)
            # ★ 정렬 래치도 푼다. 안 풀면 '길이 없어 빈 경로' 가 온 뒤 경로가
            #   잠깐 돌아올 때마다 래치가 살아 있어 전진 없이 계속 돌기만 한다.
            self.spin_latch = False
            self.new_path_pending = False
            self.stall_yaw = None
            self.nudge_until = 0.0
            self.nudge_count = 0
            self.get_logger().warn("빈 경로를 받았습니다 — 정지 상태로 대기합니다.")
            return

        # ★ '같은 경로 재발행' 을 새 경로로 취급하면 안 된다.
        #   플래너는 앞이 잠깐 막힌 것처럼 보이면 빈 경로를 냈다가(정지)
        #   막힌 게 아니면 곧바로 같은 경로를 다시 낸다(resume_if_halted).
        #   그때마다 가감속을 0 으로 되돌리면 로봇이 가다 서다를 반복하고,
        #   '첫 시작 정렬' 까지 다시 걸려 멀쩡히 가던 코너에서 제자리 회전을
        #   해 버린다. 좌표가 같으면 이어서 가던 상태를 유지한다.
        same = (len(pts) == len(self.last_pts)
                and all(abs(a[0] - b[0]) < 1e-6 and abs(a[1] - b[1]) < 1e-6
                        for a, b in zip(pts, self.last_pts)))
        self.last_pts = pts
        self.path = pts
        self.need_global_search = True
        self.arrived_announced = False
        if same:
            # 이어서 간다 — 인덱스도 가감속도 그대로 두고, 정지 중이었으면
            # 다시 굴러가도록 상태만 되살린다.
            self.idx = min(self.idx, len(pts) - 1)
            self.zero_left = 20
            self.publish_state(DRIVING)
            self.get_logger().info(
                "같은 경로를 다시 받았습니다 — 멈추지 않고 이어서 갑니다.",
                throttle_duration_sec=5.0)
            return
        self.idx = 0
        # 새 경로 -> 첫 동작은 무조건 전진 (제자리 회전부터 하지 않는다)
        self.first_active = self.first_fwd
        self.first_origin = None
        self.first_t0 = self.now()
        self.turning = False
        self.reversing = False
        self.cur_lin = 0.0
        # ★ 정렬(제자리 회전) 중이면 새 경로가 와도 회전을 끊지 않는다.
        #   플래너는 막힌 상태가 이어지면 replan_cooldown(1.5초)마다 새 우회로를
        #   낸다. 그때마다 래치를 지우면 '방향이 맞을 때까지' 가 아니라
        #   '다음 경로가 올 때까지' 만 돌게 되고, 그 순간 오차가 34도 아래면
        #   회전이 끊긴 채 전진해 버린다(실측: 66초에 26번 경로 갱신).
        #   래치를 유지하면 목표 방향만 새 경로 기준으로 다시 재고, 그 방향에
        #   맞을 때까지 계속 돈다. spin_latch_timeout 이 안전장치로 남는다.
        if self.spin_latch:
            self.new_path_pending = False   # 이미 정렬 중 — 다시 판정할 필요 없음
            # 새 경로 = 새 정렬 목표. 안전 타이머를 다시 시작한다 —
            # 안 그러면 사람 접근처럼 정렬 중 오래 끼어드는 단계가 있을 때
            # 재개하는 순간 "15초 초과" 로 오판해 제자리 회전을 건너뛰고
            # arc 로 넘어가 버린다 (2026-08-06 실측).
            self.spin_since = self.now()
            self.get_logger().info(
                "정렬 중에 새 경로를 받았습니다 — 회전을 끊지 않고 "
                "새 경로 방향으로 계속 맞춥니다.", throttle_duration_sec=3.0)
        else:
            self.cur_ang = 0.0
            self.new_path_pending = self.spin_on_new
            self.stall_yaw = None      # 정지마찰 흔들기 상태도 초기화
            self.nudge_until = 0.0
            self.nudge_count = 0
        self.reach_pub.publish(Bool(data=False))
        self.publish_state(DRIVING)

        length = sum(math.hypot(pts[i + 1][0] - pts[i][0],
                                pts[i + 1][1] - pts[i][1])
                     for i in range(len(pts) - 1))
        self.get_logger().info(
            "새 경로 수신: %d점, 길이 %.2f m, 목적지 (%.2f, %.2f) — 출발합니다."
            % (len(pts), length, pts[-1][0], pts[-1][1]))

        # 왜 도는지 한 줄로 알 수 있게: 경로가 로봇 앞에서 시작하는가 뒤에서인가.
        pose = self.current_pose()
        if pose is not None and len(pts) >= 2:
            x, y, yaw = pose
            j = min(8, len(pts) - 1)          # 약 0.3 m 앞 (0.04 m 간격 기준)
            head = math.atan2(pts[j][1] - y, pts[j][0] - x)
            e = norm_angle(head - yaw)
            self.get_logger().info(
                "  로봇 방향 %+.0f도 | 경로 시작방향 %+.0f도 | 방향오차 %+.0f도 -> %s"
                % (math.degrees(yaw), math.degrees(head), math.degrees(e),
                   "바로 전진" if abs(e) < math.radians(20) else
                   ("살짝 조향" if abs(e) < math.radians(60) else
                    "크게 돌아야 함 (경로가 로봇 뒤에서 시작)")))

    def on_scan(self, msg: LaserScan):
        self.scan = msg
        self.scan_time = self.now()

    def on_enable(self, msg: Bool):
        enabled = bool(msg.data)
        # 통합 coordinator 가 enable 을 10Hz 하트비트로 보내므로,
        # 값이 그대로면 로그·stop_now 재발행 없이 조용히 무시한다.
        if enabled == self.enabled:
            return
        self.enabled = enabled
        self.get_logger().info("자율주행 %s" % ("재개" if self.enabled else "일시정지"))
        if not self.enabled:
            self.stop_now()
        elif self.spin_latch:
            # 일시정지(사람 접근 등) 동안 흐른 시간은 '회전이 물리적으로
            # 안 되는 시간' 이 아니므로 정렬 안전 타이머를 다시 시작한다.
            self.spin_since = self.now()
            self.stall_yaw = None
            self.nudge_until = 0.0

    def on_cancel(self, _msg):
        self.path = []
        self.stop_now()
        self.publish_state(IDLE)
        self.get_logger().info("주행 취소 — 정지했습니다.")

    # ================= 위치 =================
    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def current_pose(self):
        """TF map->base_link 로 현재 (x, y, yaw). 없으면 None."""
        try:
            tr = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time())
        except tf2_ros.TransformException:
            return None
        stamp = tr.header.stamp.sec + tr.header.stamp.nanosec / 1e9
        if stamp > 0.0 and (self.now() - stamp) > self.tf_timeout:
            return None                       # 위치가 너무 오래됨 -> 신뢰 못함
        t = tr.transform.translation
        # 라이다 장착 오프셋을 더해 '로봇 차체의 앞 방향'으로 바꾼다
        yaw = norm_angle(yaw_from_quat(tr.transform.rotation) + self.head_off)
        return float(t.x), float(t.y), yaw

    # ================= 경로 =================
    def update_index(self, x, y):
        """현재 위치에서 가장 가까운 경로점 인덱스로 전진. 반환: 그 거리(횡오차)."""
        n = len(self.path)
        start = 0 if self.need_global_search else self.idx
        self.need_global_search = False
        best_i, best_d = start, float("inf")
        acc = 0.0
        i = start
        while i < n:
            px, py = self.path[i]
            d = math.hypot(px - x, py - y)
            if d < best_d:
                best_d, best_i = d, i
            if i > start:
                qx, qy = self.path[i - 1]
                acc += math.hypot(px - qx, py - qy)
                if acc > self.search_ahead:
                    break
            i += 1
        self.idx = best_i
        return best_d

    def lookahead_point(self, x, y):
        """로봇에서 lookahead 이상 떨어진 첫 경로점 (정석 pure pursuit).

        예전에는 '경로를 따라 걸어간 누적 거리'로 골랐다. 그러면 로봇이 경로에서
        조금 벗어나 있거나 경로가 로봇 옆을 스치듯 지날 때, 목표점이 로봇
        바로 옆이나 뒤에 잡힌다. 그 순간 방향오차가 90~180도로 튀어서
        출발하자마자 제자리 회전을 해버린다. lookahead 를 줄이면 더 심해진다.
        로봇으로부터의 직선거리로 고르면 목표점이 항상 앞쪽 일정 거리에 있다.
        """
        n = len(self.path)
        i = self.idx
        while i < n:
            px, py = self.path[i]
            if math.hypot(px - x, py - y) >= self.lookahead:
                return px, py
            i += 1
        return self.path[-1]

    # ================= 앞쪽 장애물 =================
    def sector_min_range(self, center=0.0):
        """center 방향(0=앞, pi=뒤) 부채꼴의 최소거리[m]. 스캔이 없으면 None.

        부채꼴이 좌우 대칭이라 라이다가 뒤집혀 장착(roll 180도)돼 있어도
        결과가 같다.
        """
        scan = self.scan
        if scan is None or self.scan_time is None:
            return None
        if self.now() - self.scan_time > 1.0:
            return None
        half = math.radians(self.front_deg) * 0.5
        rmin = max(float(scan.range_min), 0.02)
        rmax = float(scan.range_max) if scan.range_max > 0.0 else 12.0
        # numpy 로 한 번에 처리한다. 파이썬 for 문으로 430점 x 20Hz 를 돌리면
        # 이 노드만 CPU 15% 를 먹고, 그만큼 라이다 시리얼 읽기가 밀린다.
        r = np.asarray(scan.ranges, dtype=np.float64)
        if r.size == 0:
            return None
        a = scan.angle_min + np.arange(r.size) * scan.angle_increment - center
        a = (a + math.pi) % (2.0 * math.pi) - math.pi
        sel = (np.abs(a) <= half) & np.isfinite(r) & (r > rmin) & (r < rmax)
        if not sel.any():
            return None
        return float(r[sel].min())

    # ================= 제어 루프 =================
    def control_loop(self):
        if not self.enabled or not self.path or self.state == ARRIVED:
            self.hold_stop()
            return

        pose = self.current_pose()
        if pose is None:
            self.hold_stop()
            self.get_logger().warn(
                "현재 위치(TF %s->%s)를 못 받았습니다 — 정지 유지. SLAM 이 떠 있는지 확인하세요."
                % (self.map_frame, self.base_frame), throttle_duration_sec=3.0)
            return

        x, y, yaw = pose
        self.check_rotation_sign(yaw, self.now())
        cross = self.update_index(x, y)
        gx, gy = self.path[-1]
        goal_dist = math.hypot(gx - x, gy - y)

        # --- 1) 도착 판정 -------------------------------------------------
        if goal_dist <= self.goal_tol:
            self.on_arrived(x, y, goal_dist)
            return

        if cross > self.offtrack_warn:
            self.get_logger().warn(
                "경로에서 %.2f m 벗어났습니다 — 복귀 중입니다." % cross,
                throttle_duration_sec=3.0)

        # --- 2) pure pursuit ---------------------------------------------
        tx, ty = self.lookahead_point(x, y)
        bearing = math.atan2(ty - y, tx - x)
        err_fwd = norm_angle(bearing - yaw)          # 앞으로 갈 때 틀어진 각
        err_rev = norm_angle(err_fwd - math.pi)      # 뒤로 갈 때 틀어진 각

        # 덜 도는 쪽 선택 (히스테리시스로 앞뒤 채터링 방지)
        if self.allow_reverse:
            if self.reversing:
                if abs(err_fwd) < abs(err_rev) - self.rev_hyst:
                    self.reversing = False
                    self.turning = False
            elif abs(err_rev) < abs(err_fwd) - self.rev_hyst:
                self.reversing = True
                self.turning = False
        else:
            self.reversing = False
        err = err_rev if self.reversing else err_fwd

        # --- 첫 동작 강제 전진 --------------------------------------------
        # 사용자 요구: 출발은 무조건 전진. 방향오차가 크더라도 제자리 회전부터
        # 하지 않고, first_forward_dist 만큼 앞으로 나간 뒤 정상 제어로 넘어간다.
        forced_forward = False
        if self.first_active:
            if self.first_origin is None:
                self.first_origin = (x, y, yaw)
            dx0 = x - self.first_origin[0]
            dy0 = y - self.first_origin[1]
            gone = math.hypot(dx0, dy0)
            if gone >= self.first_dist or \
                    (self.now() - self.first_t0) > self.first_timeout:
                self.first_active = False
                # 이 직진이 곧 '앞이 어디인가' 측정이다. 실제 이동 방향과
                # 내가 믿는 앞 방향이 다르면 base_link +x 가 로봇의 앞이 아니라는
                # 뜻이므로(라이다 장착 yaw 오프셋) 그 자리에서 보정해 버린다.
                if gone >= 0.06:
                    off = norm_angle(math.atan2(dy0, dx0) - self.first_origin[2])
                    if self.auto_head and abs(off) > math.radians(20.0):
                        self.head_off = norm_angle(self.head_off + off)
                        self.head_verified = True
                        self.get_logger().error(
                            "★ 자동 보정: 전진했더니 실제로는 %+.0f도 방향으로 "
                            "갔습니다. base_link 의 +x 가 로봇의 앞이 아닙니다. "
                            "heading_offset 을 %+.0f도 로 잡고 계속합니다. "
                            "(고정하려면 launch 에 heading_offset:=%.1f 를 주세요)"
                            % (math.degrees(off), math.degrees(self.head_off),
                               math.degrees(self.head_off)))
                        self.need_global_search = True
                    else:
                        self.head_verified = True
                        self.get_logger().info(
                            "첫 전진 %.2f m — 앞 방향 오차 %+.0f도 (정상)"
                            % (gone, math.degrees(off)))
                else:
                    self.get_logger().warn(
                        "첫 전진이 %.2f m 뿐이라 방향 보정을 못 했습니다 "
                        "(앞이 막혔거나 모터 출력 부족)." % gone)
            else:
                forced_forward = True

        # ★ 새 경로의 '첫 시작' 정렬 판정 -------------------------------------
        #   새 경로를 받은 첫 주기에만 본다. 많이 틀어져 있으면 래치를 걸고,
        #   방향이 맞을 때까지(spin_exit) 전진 없이 제자리 회전만 한다.
        #   경로 중간의 좌/우회전에는 걸리지 않으므로 평상시는 arc 그대로다.
        if self.new_path_pending:
            self.new_path_pending = False
            if abs(err) > self.spin_enter:
                self.spin_latch = True
                self.spin_since = self.now()
                self.get_logger().info(
                    "새 경로 시작 — 방향이 %.0f도 틀어져 있습니다. "
                    "%.0f도 아래로 맞출 때까지 제자리 회전만 합니다."
                    % (math.degrees(abs(err)), math.degrees(self.spin_exit)))
        if self.spin_latch:
            if abs(err) <= self.spin_exit:
                self.spin_latch = False
                self.get_logger().info(
                    "방향이 맞았습니다 (오차 %.0f도) — 주행을 시작합니다."
                    % math.degrees(abs(err)))
            elif self.now() - self.spin_since > self.spin_timeout:
                # 회전이 물리적으로 안 되는 상황. 갇히면 스스로 못 빠져나오므로
                # 풀어 주고 turn_mode 대로(arc) 진행한다.
                self.spin_latch = False
                self.get_logger().error(
                    "%.0f초 동안 제자리 회전으로 방향을 못 맞췄습니다 "
                    "(남은 오차 %.0f도). 회전이 안 되는 상태로 보고 "
                    "'%s' 방식으로 넘어갑니다 — 바퀴 걸림/출력 부족을 확인하세요."
                    % (self.spin_timeout, math.degrees(abs(err)),
                       self.turn_mode))
        spin_now = (self.turn_mode == "spin" or self.spin_latch)

        if forced_forward:
            tgt_lin = self.key_lin if self.control_mode == "keys" else self.max_lin
            tgt_ang = 0.0
        elif self.control_mode == "keys":
            tgt_lin, tgt_ang = self.key_command(err, goal_dist)
        elif self.spin_latch or (abs(err) > self.turn_angle and spin_now):
            # 방향이 많이 틀어졌으면 제자리 회전 (전진 금지 -> 벽에 안 밀어붙임)
            tgt_lin = 0.0
            # ★ 관성 예측 정지. keys 모드에만 있던 것을 smooth 에도 넣는다.
            #   제자리 회전은 min_angular(0.80 = PWM 204) 이하로는 내려가지
            #   않으므로 목표에 가까워져도 계속 전속으로 돈다. 그대로 두면
            #   지나치고 -> 반대로 돌고 -> 또 지나치며 "계속 더 도는" 현상이 된다.
            #   지금 각속도로 계속 돌면 지나칠 상황이면 이번 주기는 쉰다.
            coast = abs(self.yaw_rate) * self.turn_lead
            if coast > abs(err) - self.key_turn_off:
                tgt_ang = 0.0
            else:
                # ★ 제자리 회전은 오차에 비례시키면 안 된다.
                #   k_angular(1.3) x 오차로 주면 오차 44도 아래에서 출력이
                #   1.00(PWM 255) -> 0.90(229) -> 0.80(204) 으로 떨어지는데,
                #   이 차체는 그 구간에서 정지마찰을 못 깨고 그대로 선다.
                #   실측: 168도에서 40도까지는 잘 돌다가(그 구간은 255)
                #   40도에서 0.90 이 되는 순간 13초간 1.9도밖에 못 돌았다.
                #   언제 끊을지는 위의 관성 예측(coast)이 이미 담당하므로
                #   출력까지 줄일 이유가 없다. 기본은 항상 최대로 준다.
                mag = (self.spin_power if self.spin_power > 0.0
                       else abs(self.k_ang * err))
                mag = min(max(mag, self.min_ang), self.max_ang)
                tgt_ang = math.copysign(mag, err)
                # ★ 그래도 안 돌면(정지마찰) 아주 짧게 앞뒤로 흔들어 깬다.
                #   바퀴가 한 번 구르면 운동마찰로 떨어져 회전이 붙는다.
                #   회전 명령은 그대로 둔 채 전진량만 잠깐 준다.
                tgt_lin = self.stall_nudge(yaw, tgt_ang)
            self.turning = True          # 관성 자동학습이 이 회전을 보게 한다
        elif abs(err) > self.turn_angle:
            # arc: 전진하면서 최대 조향. 아래 3.5 에서 |ang| <= lin 로 잘려
            # 안쪽 바퀴가 0 까지만 떨어지고 역회전하지 않는다.
            tgt_lin = self.max_lin
            tgt_ang = math.copysign(self.max_ang, err)
        else:
            # 방향 오차가 클수록 감속 (코너에서 벽 쪽으로 밀리지 않게)
            turn_scale = 1.0 - 0.6 * (abs(err) / self.turn_angle)
            tgt_lin = self.max_lin * turn_scale
            # 목적지 접근 감속
            if goal_dist < self.slow_dist:
                approach = goal_dist / max(self.slow_dist, 1e-6)
                tgt_lin = min(tgt_lin, max(self.min_lin, self.max_lin * approach))
            tgt_lin = max(tgt_lin, self.min_lin)
            tgt_ang = clamp(self.k_ang * err, -self.max_ang, self.max_ang)

        # smooth 모드의 self.turning 은 분기 안에서 켜기만 했으므로 여기서 끈다.
        # (분기를 하나 더 만들면 나머지 제어가 통째로 건너뛰어진다)
        if self.control_mode != "keys":
            self.turning = (self.spin_latch
                            or (abs(err) > self.turn_angle and spin_now))

        # smooth 모드에서 후진이면 전진량 부호만 뒤집는다 (조향 부호는 err_rev 가
        # 이미 후진 기준이라 그대로 두면 된다)
        if self.reversing and self.control_mode != "keys":
            tgt_lin = -tgt_lin

        # --- 3) 진행방향 장애물 -> 그 방향 이동만 차단 ---------------------
        state = DRIVING
        if self.use_scan_guard and abs(tgt_lin) > 0.0:
            # 반드시 '실제로 이동하는 방향'을 봐야 한다.
            # base_link 의 +x 가 로봇의 앞이 아닐 수 있으므로(heading_offset),
            # 그 오프셋을 더한 방향을 본다. 이걸 빼먹으면 heading_offset:=180 일 때
            # 로봇 뒤통수 쪽을 보고 정지시켜, 앞이 활짝 트여 있어도 영원히 멈춘다.
            look = self.head_off + (math.pi if self.reversing else 0.0)
            front = self.sector_min_range(look)
            # heading_offset 이 아직 실측으로 확인되지 않았다면, 반대편도 '접촉 직전'
            # 거리로 감시한다. 오프셋이 틀렸을 때 엉뚱한 쪽으로 들이받는 것을 막는다.
            # (정지거리보다 짧은 값이라 정상 상황에서 교착을 만들지 않는다)
            if not self.head_verified:
                # 범퍼선(차체 길이/2). 이보다 가까우면 이미 닿기 직전이다.
                # 정지거리(0.13)보다 짧아야 정상 상황에서 교착이 안 생긴다.
                touch = 0.5 * self.robot_length
                back = self.sector_min_range(look + math.pi)
                if back is not None and back <= touch:
                    front = min(front, back) if front is not None else back
                    self.get_logger().warn(
                        "반대쪽 %.2f m 에 벽 — heading_offset 이 아직 확인 전이라 "
                        "양쪽을 봅니다. 로봇을 조금 떼어 놓으세요." % back,
                        throttle_duration_sec=5.0)
            if front is not None:
                if front <= self.stop_dist:
                    tgt_lin = 0.0
                    state = BLOCKED
                    self.get_logger().warn(
                        "%s %.2f m 에 장애물 — 이동 정지(회전으로 회피 시도)"
                        % ("뒤쪽" if self.reversing else "앞쪽", front),
                        throttle_duration_sec=2.0)
                    # 경로는 앞으로 가라는데 계속 막혀 있으면 교착이다.
                    # (회전할 이유가 없으니 스스로는 절대 못 빠져나온다)
                    if self.blocked_since is None:
                        self.blocked_since = self.now()
                    elif (self.now() - self.blocked_since > 3.0
                          and not self.blocked_warned):
                        self.blocked_warned = True
                        self.get_logger().error(
                            "%.0f초째 %s %.2f m 장애물로 멈춰 있습니다(정지거리 %.2f m). "
                            "경로는 계속 전진하라고 하므로 스스로 빠져나오지 못합니다. "
                            "확인할 것: (1) heading_offset 이 틀리면 라이다가 로봇의 "
                            "'뒤'를 앞으로 보고 있을 수 있습니다 (2) 로봇이 벽에 너무 "
                            "붙어 있으면 손으로 조금 떼어 놓으세요 (3) 이 방처럼 좁으면 "
                            "use_scan_guard:=false 로 꺼도 됩니다 — 경로 자체가 이미 "
                            "차체 여유를 반영합니다."
                            % (self.now() - self.blocked_since,
                               "뒤쪽" if self.reversing else "앞쪽",
                               front, self.stop_dist))
                else:
                    self.blocked_since = None
                    self.blocked_warned = False
                if front is not None and front > self.stop_dist \
                        and front < self.slow_obs_dist and self.control_mode != "keys":
                    span = max(self.slow_obs_dist - self.stop_dist, 1e-6)
                    ratio = (front - self.stop_dist) / span
                    slowed = max(self.min_lin * 0.6, abs(tgt_lin) * ratio)
                    tgt_lin = math.copysign(slowed, tgt_lin)

        # --- 3.5) arc 모드(smooth 전용): 안쪽 바퀴가 역회전하지 않도록 제한 -----
        # left = lin - ang, right = lin + ang 이므로 |ang| <= lin 이면 두 바퀴 모두
        # 0 이상이다. 장애물로 lin 이 0 이 되면 ang 도 0 이 되어 완전 정지한다
        # (호 회전은 전진 없이는 불가능하므로).
        # --- 교착 감지 (장애물 정지와 무관) -------------------------------
        # 명령은 계속 내는데 실제 위치가 안 변하면 뭔가 잘못된 것이다.
        # 바퀴 헛돌기, 벽에 걸림, 모터 노드 없음, PWM 부족 등 원인은 여러 가지다.
        # 정지시키지는 않는다 — 알려만 준다.
        # ★ 회전량도 같이 봐야 한다. 제자리 회전은 선형 이동이 원래 0 이라
        #   거리만 재면 '잘 돌고 있는 중'을 교착으로 오판한다(실제로 그랬다).
        if abs(tgt_lin) > 1e-6 or abs(tgt_ang) > 1e-6:
            now_t = self.now()
            if self.stuck_ref is None:
                self.stuck_ref = (x, y, yaw, now_t)
            else:
                rx, ry, ryaw, rt = self.stuck_ref
                moved = math.hypot(x - rx, y - ry)
                turned = abs(norm_angle(yaw - ryaw))
                if moved >= self.stuck_dist or turned >= self.stuck_yaw:
                    self.stuck_ref = (x, y, yaw, now_t)   # 잘 움직이는 중
                elif (now_t - rt > self.stuck_sec
                      and now_t - self.stuck_warned > 5.0):
                    self.stuck_warned = now_t
                    turning_only = abs(tgt_lin) <= 1e-6
                    self.get_logger().error(
                        "교착: %.0f초째 명령(전진 %.2f, 회전 %.2f)을 내는데 "
                        "%.3f m / %.1f도 밖에 못 움직였습니다. 확인할 것 — "
                        "모터 노드가 떠 있는지 (ros2 topic info %s 의 구독자), "
                        "UART 포트를 두 프로세스가 쓰고 있지 않은지, "
                        "바퀴가 벽에 걸려 있지 않은지.%s"
                        % (now_t - rt, tgt_lin, tgt_ang, moved,
                           math.degrees(turned), self.cmd_topic,
                           # 회전만 내는데 안 돌면 원인이 좁혀진다
                           # 실제로 나가는 값을 그대로 보여 준다. 예전에는
                           # 무조건 '이미 최대치'라고 적어서, 사실은 비례제어로
                           # 0.90(PWM 229)이 나가고 있는데도 출력을 더 못 올린다고
                           # 오진하게 만들었다.
                           (" 지금은 '제자리 회전' 명령만 내고 있는데 각도가 거의 "
                            "안 변했습니다. 회전 출력 %.2f (PWM %d) / 최대 %.2f "
                            "(PWM %d), 정지마찰 흔들기 %d회.%s"
                            % (abs(self.cur_ang), int(abs(self.cur_ang) * 255),
                               self.max_ang, int(self.max_ang * 255),
                               self.nudge_count,
                               " 최대보다 낮게 나가고 있습니다 — spin_power:=1.0 "
                               "인지 확인하세요."
                               if abs(self.cur_ang) < self.max_ang - 1e-6 else
                               " 최대 출력인데도 안 돕니다. 바퀴 걸림/전압 강하/"
                               "모터 노드를 확인하세요. spin_nudge_lin 을 키우거나"
                               " spin_on_new_path:=false 로 arc 주행을 쓰세요."))
                           if turning_only else ""))
        else:
            self.stuck_ref = None

        # 회전을 끊은 뒤 실제로 얼마나 더 도는지 재서 turn_lead 를 스스로 맞춘다
        self.learn_turn_lead(yaw)

        # 전진 방향이 실제로 맞는지 계속 검증 (라이다 장착 오프셋 자동 진단)
        self.check_forward_direction(x, y, yaw, tgt_lin)

        # arc 제약(|ang| <= lin)은 '이번 주기가 arc 일 때만' 건다. 제자리 회전
        # 중에 걸면 전진이 0 이라 회전까지 0 으로 잘려 아예 안 돈다.
        if self.control_mode == "smooth" and not spin_now:
            tgt_ang = clamp(tgt_ang, -tgt_lin, tgt_lin)
            if state == BLOCKED:
                self.get_logger().warn(
                    "arc 모드는 전진 없이 방향을 못 바꿉니다 — 로봇을 앞이 트인 곳으로 "
                    "옮기거나 use_scan_guard:=false / turn_mode:=spin 을 쓰세요.",
                    throttle_duration_sec=5.0)

        self.publish_state(state)

        # --- 4) 가감속 완화 후 발행 --------------------------------------
        if self.control_mode == "keys":
            # keyboard_test.py 와 동일하게: 램프도 정규화도 없이 그대로 발행한다.
            # (2.0 은 node.py 에서 PWM 510 -> 255 로 잘린다. 키보드와 같은 값)
            self.cur_lin, self.cur_ang = tgt_lin, tgt_ang
            self.publish_raw(tgt_lin, tgt_ang)
        else:
            self.cur_lin = self.ramp(self.cur_lin, tgt_lin,
                                     self.accel_step, self.decel_step)
            self.cur_ang = self.ramp(self.cur_ang, tgt_ang,
                                     self.ang_step, self.ang_step)
            # ★ 정지마찰 구간을 기어 올라가지 않게 한다.
            #   angular_step 0.15 로 0 에서 램프하면 PWM 이
            #   38 -> 77 -> 115 -> 153 -> 191 -> 230 -> 255 로 오른다.
            #   이 모터는 PWM 204(min_angular 0.80) 부근은 돼야 확실히 도는데,
            #   앞쪽 5주기는 그 아래라 바퀴가 떨기만 한다. 제자리 회전은 그 사이
            #   정지마찰을 못 깨고 그대로 멈춰 있게 된다.
            #   목표가 최소출력 이상이면 램프 도중이라도 최소출력부터 준다.
            if (abs(tgt_ang) >= self.min_ang
                    and abs(self.cur_ang) < self.min_ang):
                self.cur_ang = math.copysign(self.min_ang, tgt_ang)
            # arc 모드의 |ang| <= lin 제약은 '실제로 나가는 값'에 걸어야 한다.
            # 목표값에만 걸면 가감속 중에 회전이 전진을 앞질러(회전 램프가 더 빠름)
            # 안쪽 바퀴가 순간적으로 역회전한다.
            if not spin_now:
                self.cur_ang = clamp(self.cur_ang, -self.cur_lin, self.cur_lin)
            self.publish_cmd(self.cur_lin, self.cur_ang)
        self.zero_left = 20          # 주행이 끝나면 0 을 이만큼 더 보내고 조용해진다
        # 마커는 눈으로 보는 용도라 5Hz 면 충분하다 (20Hz 로 쏘면 CPU 낭비)
        self.marker_i = (self.marker_i + 1) % 4
        if self.marker_i == 0:
            self.publish_lookahead_marker(tx, ty)

        # --- 5) 1초마다 상태 로그 -----------------------------------------
        t = self.now()
        if t - self.last_log > 1.0:
            self.last_log = t
            self.get_logger().info(
                "[%s%s] 위치(%.2f, %.2f, %.0f도) 경로 %d/%d 횡오차 %.02fm "
                "남은거리 %.2fm 방향오차 %.0f도 명령(전진 %.2f, 회전 %.2f)"
                % (state, " 후진" if self.reversing else "",
                   x, y, math.degrees(yaw), self.idx + 1, len(self.path),
                   cross, goal_dist, math.degrees(err), self.cur_lin, self.cur_ang))

    def stall_nudge(self, yaw, tgt_ang):
        """제자리 회전이 정지마찰에 걸렸을 때 짧게 앞뒤로 흔들어 깬다.

        이 차체는 상판 무게 때문에 PWM 이 최대여도 정지 상태에서는 네 바퀴가
        바닥을 비비지 못해 그대로 서 있는 경우가 있다(실측: 13초에 1.9도).
        한 번이라도 바퀴가 구르면 정지마찰이 운동마찰로 떨어져 회전이 붙는다.
        그래서 회전 명령은 그대로 둔 채 전진량만 아주 짧게 준다.

        방향은 뒤쪽부터 시작한다 — 제자리 회전이 필요한 상황은 대개 앞에
        장애물이 있어서 멈춘 직후라, 앞으로 미는 것보다 안전하다.
        번갈아 가며 흔들어 한쪽 벽에 계속 밀리지 않게 한다.

        반환: 이번 주기에 줄 전진량(평소에는 0.0).
        """
        if self.spin_nudge_sec <= 0.0 or abs(tgt_ang) < 1e-6:
            return 0.0
        now = self.now()
        # 흔드는 중이면 그대로 이어 간다
        if now < self.nudge_until:
            return self.nudge_sign * abs(self.spin_nudge_lin)
        # 회전 진척을 감시한다
        if self.stall_yaw is None:
            self.stall_yaw, self.stall_t = yaw, now
            return 0.0
        moved = abs(norm_angle(yaw - self.stall_yaw))
        if moved >= math.radians(self.spin_stall_deg):
            self.stall_yaw, self.stall_t = yaw, now   # 잘 돌고 있다
            self.nudge_count = 0
            return 0.0
        if now - self.stall_t < self.spin_stall_sec:
            return 0.0
        # 멈춰 있다 -> 흔든다
        self.nudge_count += 1
        self.nudge_sign = -1.0 if (self.nudge_count % 2) else 1.0
        self.nudge_sign *= math.copysign(1.0, self.spin_nudge_lin)
        self.nudge_until = now + self.spin_nudge_sec
        self.stall_yaw, self.stall_t = yaw, now
        self.get_logger().warn(
            "제자리 회전이 %.1f초째 %.1f도 미만입니다(정지마찰) — "
            "%s 으로 %.2f초 살짝 흔들어 바퀴를 깨웁니다. (%d회째)"
            % (self.spin_stall_sec, self.spin_stall_deg,
               "뒤" if self.nudge_sign < 0 else "앞",
               self.spin_nudge_sec, self.nudge_count),
            throttle_duration_sec=3.0)
        return self.nudge_sign * abs(self.spin_nudge_lin)

    def learn_turn_lead(self, yaw):
        """회전을 끊은 뒤 실제로 더 돈 각도를 재서 turn_lead 를 스스로 보정한다.

        turn_lead 는 '명령을 끊고도 계속 도는 시간'이다. TF 지연 + 모터 관성으로
        정해지는데, 이건 상판 무게에 따라 달라지므로 고정값으로 맞출 수가 없다.
        그래서 매 회전이 끝날 때마다

            실제로 필요했던 lead = (끊은 뒤 더 돈 각도) / (끊는 순간의 각속도)

        를 재서 조금씩 따라간다. 지나치면 커지고(더 일찍 끊고), 못 미치면
        작아진다(늦게까지 붙는다). 자동으로 양쪽에서 수렴한다.

        회전 중에는 마지막으로 '실제 회전 명령을 낸' 순간을 계속 기억해 둔다.
        turn_forward_pulse 로 중간에 전진(ang=0)이 섞여도 그건 '끊은 것'이
        아니므로, 판단은 명령이 아니라 self.turning(히스테리시스 상태)으로 한다.
        """
        now = self.now()
        if self.turning:
            if abs(self.cur_ang) > 1e-6:      # 실제로 돌라고 한 주기만 기록
                d = math.copysign(1.0, self.cur_ang)
                # 직전 회전이 끝난 지 얼마 안 됐는데 반대로 돈다 = 지나쳤다는 뜻.
                # 이때는 정지 판정까지 못 가서 아래 측정이 아예 발동하지 못하므로,
                # 되돌아가는 것 자체를 신호로 삼아 lead 를 키운다(부트스트랩).
                if (self.turn_end_t is not None
                        and now - self.turn_end_t < self.flip_window
                        and self.turn_end_dir != 0.0 and d != self.turn_end_dir):
                    old = self.turn_lead
                    self.turn_lead = clamp(self.turn_lead * 1.3,
                                           self.lead_min, self.lead_max)
                    self.turn_end_t = None
                    if self.turn_lead > old + 1e-6:
                        self.get_logger().warn(
                            "회전이 목표를 지나쳐 반대로 되돌아갑니다 — "
                            "turn_lead %.2f -> %.2f초로 늘려 더 일찍 끊습니다."
                            % (old, self.turn_lead))
                self.cut_yaw = yaw
                self.cut_rate = self.yaw_rate
                self.cut_dir = d
                self.cut_done = False
            return
        if self.cut_dir != 0.0:               # 방금 회전이 끝났다
            self.turn_end_t = now
            self.turn_end_dir = self.cut_dir
            self.cut_dir = 0.0
        if self.cut_yaw is None or self.cut_done:
            return
        if abs(self.yaw_rate) > self.settle_rate:
            return                            # 아직 도는 중 — 멈출 때까지 기다린다
        self.cut_done = True
        over = abs(norm_angle(yaw - self.cut_yaw))
        rate = abs(self.cut_rate)
        # 너무 느리게 돌던 끝자락은 비율이 불안정하므로 버린다
        if rate < 0.2 or over < math.radians(1.0):
            return
        measured = clamp(over / rate, 0.05, 2.0)
        old = self.turn_lead
        self.turn_lead = clamp(0.7 * self.turn_lead + 0.3 * measured,
                               self.lead_min, self.lead_max)
        self.get_logger().info(
            "회전 관성 학습: 끊은 뒤 %.1f도 더 돎 (그때 %.0f도/s) "
            "-> 필요한 lead %.2f초 | turn_lead %.2f -> %.2f초"
            % (math.degrees(over), math.degrees(rate), measured,
               old, self.turn_lead))

    # ================= keys 모드 (keyboard_test.py 와 동일한 명령) ==========
    def key_command(self, err, goal_dist):
        """w / a / d / 정지 중 하나만 돌려준다. 섞지 않는다.

        회전(a/d)과 전진(w)을 동시에 내지 않는 것이 핵심이다. keyboard_test.py 로
        잘 움직였던 명령이 바로 이 네 가지뿐이기 때문이다.
        비례제어가 없으므로 회전은 펄스(ON/OFF)로 나눠서 오버슛을 줄인다.
        """
        # 히스테리시스: 켜질 땐 key_turn_on, 꺼질 땐 key_turn_off (덜덜거림 방지)
        if self.turning:
            if abs(err) < self.key_turn_off:
                self.turning = False
                self.pulse_i = 0
        else:
            if abs(err) > self.key_turn_on:
                self.turning = True
                self.pulse_i = 0

        if self.turning:
            # ★ 관성/지연 보정 — '지금 속도로 계속 돌면 지나치는가'를 매 주기 본다.
            #   명령을 끊어도 로봇은 바로 서지 않는다. 자세(TF) 갱신 지연 + 모터
            #   관성 때문에 turn_lead_sec 만큼 더 돈다. 남은 각도가 그보다 적으면
            #   이번 주기는 쉬어서 미리 감속한다.
            #   고정 임계값을 키우는 방식보다 정확하다 — 빨리 돌수록 더 일찍 끊고,
            #   느리게 돌면 늦게까지 붙는다. 큰 각도에서는 남은 각이 훨씬 크므로
            #   그대로 연속 최대출력이라 정지마찰 문제도 안 생긴다.
            coast = abs(self.yaw_rate) * self.turn_lead
            if coast > abs(err) - self.key_turn_off:
                self.pulse_i = 0
                return 0.0, 0.0
            on = max(self.turn_pulse_on, 1)
            off = max(self.turn_pulse_off, 0)
            fwd = max(self.turn_fwd, 0)        # 0 이면 순수 제자리 회전
            p = self.pulse_i % (on + off + fwd)
            self.pulse_i += 1
            if p < on:
                return 0.0, math.copysign(self.key_ang, err)   # a(좌) / d(우)
            if p < on + off:
                return 0.0, 0.0                                # 잠깐 멈춰 자세 안정
            return self.key_lin, 0.0        # w — 전진하며 도는 호가 된다

        # 직진 구간. 목적지 근처에서는 전속밖에 없으니 펄스로 감속한다.
        if goal_dist < self.slow_dist:
            on, off = max(self.fwd_pulse_on, 1), max(self.fwd_pulse_off, 0)
            p = self.pulse_i % (on + off)
            self.pulse_i += 1
            if p >= on:
                return 0.0, 0.0
        else:
            self.pulse_i = 0
        # w(전진) 또는 s(후진) — 뒤쪽이 목적지면 180도 돌지 않고 그대로 뒤로 간다
        return (-self.key_lin if self.reversing else self.key_lin), 0.0

    @staticmethod
    def ramp(cur, target, up, down):
        if target > cur:
            return min(target, cur + up)
        return max(target, cur - down)

    # ================= 도착 처리 =================
    def on_arrived(self, x, y, dist):
        self.state = ARRIVED
        self.cur_lin = 0.0
        self.cur_ang = 0.0
        self.zero_left = 20
        # 모든 모터 정지 (여러 번 보내 확실히 0 을 잡는다)
        for _ in range(5):
            self.publish_cmd(0.0, 0.0)

        if not self.arrived_announced:
            self.arrived_announced = True
            gx, gy = self.path[-1]
            print("=" * 46, flush=True)
            print("목적지에 도착했습니다.", flush=True)
            print("  목적지 (%.2f, %.2f) / 현재 (%.2f, %.2f) / 오차 %.3f m"
                  % (gx, gy, x, y, dist), flush=True)
            print("=" * 46, flush=True)
            self.get_logger().info("목적지에 도착했습니다. (오차 %.3f m)" % dist)
            self.reach_pub.publish(Bool(data=True))
            self.publish_state(ARRIVED)
            self.publish_goal_marker(gx, gy)

    # ================= 발행 =================
    def publish_cmd(self, lin, ang):
        """좌/우 바퀴 출력으로 정규화한 뒤 발행.

        UART/node.py 가 left=lin-ang, right=lin+ang 로 PWM 을 만들고 ±255 로
        자르기 때문에, 여기서 미리 1.0 을 넘지 않게 같이 줄여야 조향이 안 틀어진다.
        반대로 너무 작으면 정지마찰 때문에 안 움직이므로 최소 출력까지 올린다.
        """
        left, right = lin - ang, lin + ang
        peak = max(abs(left), abs(right))
        if peak > self.max_wheel and peak > 1e-6:
            k = self.max_wheel / peak
            lin, ang, peak = lin * k, ang * k, self.max_wheel
        if 1e-6 < peak < self.min_wheel:
            k = self.min_wheel / peak
            lin, ang = lin * k, ang * k
        self.publish_raw(lin, ang)

    def publish_raw(self, lin, ang):
        """정규화/최소출력 보정 없이 그대로 발행 (keyboard_test.py 와 동일).

        배선이 반대인 경우를 위한 부호 반전은 모든 경로가 여기를 지나가도록
        이 한 곳에서만 적용한다.
        """
        # 진단은 '제어기가 의도한 방향'(반전 전) 기준으로 재야 한다.
        # 반전 후 값으로 재면 invert_angular:=true 로 고친 뒤에도 계속
        # "반대입니다" 라고 하게 된다.
        self.rot_last_cmd = float(ang)
        if self.invert_lin:
            lin = -lin
        if self.invert_ang:
            ang = -ang
        msg = Twist()
        msg.linear.x = float(lin)
        msg.angular.z = float(ang)
        self.cmd_pub.publish(msg)

    def hold_stop(self):
        """정지 상태 유지. 0 을 잠깐(약 1초) 더 보낸 뒤 발행을 멈춘다."""
        self.cur_lin = 0.0
        self.cur_ang = 0.0
        if self.zero_left > 0:
            self.zero_left -= 1
            self.cmd_pub.publish(Twist())

    def stop_now(self):
        self.state = IDLE if self.state != ARRIVED else ARRIVED
        self.cur_lin = 0.0
        self.cur_ang = 0.0
        self.zero_left = 20
        for _ in range(3):
            self.cmd_pub.publish(Twist())

    def publish_state(self, state):
        if state != ARRIVED:
            self.state = state
        if state != self.last_status:
            self.last_status = state
            self.status_pub.publish(String(data=state))

    def publish_lookahead_marker(self, tx, ty):
        m = Marker()
        m.header.frame_id = self.map_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "lookahead"
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = float(tx)
        m.pose.position.y = float(ty)
        m.pose.position.z = 0.05
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.07
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.85, 0.0, 1.0
        self.marker_pub.publish(m)

    def publish_goal_marker(self, gx, gy):
        dot = Marker()
        dot.header.frame_id = self.map_frame
        dot.header.stamp = self.get_clock().now().to_msg()
        dot.ns = "goal"
        dot.id = 1
        dot.type = Marker.SPHERE
        dot.action = Marker.ADD
        dot.pose.position.x = float(gx)
        dot.pose.position.y = float(gy)
        dot.pose.position.z = 0.05
        dot.pose.orientation.w = 1.0
        dot.scale.x = dot.scale.y = dot.scale.z = 0.14
        dot.color.r, dot.color.g, dot.color.b, dot.color.a = 0.0, 1.0, 0.3, 0.9
        self.marker_pub.publish(dot)

        txt = Marker()
        txt.header = dot.header
        txt.ns = "goal"
        txt.id = 2
        txt.type = Marker.TEXT_VIEW_FACING
        txt.action = Marker.ADD
        txt.pose.position.x = float(gx)
        txt.pose.position.y = float(gy)
        txt.pose.position.z = 0.25
        txt.pose.orientation.w = 1.0
        txt.scale.z = 0.12
        txt.color.r, txt.color.g, txt.color.b, txt.color.a = 1.0, 1.0, 1.0, 1.0
        txt.text = "목적지에 도착했습니다."
        self.marker_pub.publish(txt)


def main():
    rclpy.init()
    node = PathFollower()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass                      # Ctrl-C / launch 종료 — 정상 종료로 처리
    finally:
        # 종료 전 안전을 위해 정지 명령 전송 (keyboard_test.py 와 동일)
        # (rclpy 가 이미 내려갔으면 UART 노드의 0.1초 타임아웃이 모터를 멈춘다)
        try:
            for _ in range(3):
                node.cmd_pub.publish(Twist())
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
