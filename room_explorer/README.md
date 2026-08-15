# room_explorer — 도면의 모든 방 자동 탐사

기존 2D 도면을 **방 단위로 자동 분할**하고, 모든 방을 차례로 방문하는 미션 노드.
각 방은 **정중앙까지만 들어가서 제자리 360도 회전으로 주변을 스캔**하고 나온다.
장애물로 못 가는 방은 **로그에 알리고 건너뛴 뒤** 나머지 방을 계속 탐색하고,
한 바퀴 돈 뒤 한 번 더 시도한다.

주행·장애물 인식·우회·도면 정렬은 전부 `~/ping_detour_auto_drive` 를
**한 파일도 수정하지 않고** 그대로 재사용한다 (launch 로 include).

```
~/room_explorer/
  room_explorer_node.py    탐사 조율 노드 (방 분할 + 순서 + 목적지 전송 + 스캔 회전)
  room_segment.py          방 분할 라이브러리 (h-maxima + priority flood)
  room_explorer.launch.py  전부 한 번에 실행 (auto_drive.launch.py include)
  room_check.py            주행 전 방 분할 미리보기 (ROS 불필요)
  preview_maze.png         기본 미로 도면 분할 결과 (방 14개)
  preview_2d_ex.png        ~/converted_maps/2d_ex.yaml 분할 결과 (방 10개)
```

---

## 1. 실행

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch ~/room_explorer/room_explorer.launch.py
```

로봇을 도면상 시작 위치에 두고 실행하면 끝이다. SLAM·플래너 준비를 기다린 뒤,
**도면-라이다 자동 정렬(auto_align)이 실제로 매칭·적용된 것을 확인하고 나서야
출발한다.** 터미널에 이 줄이 뜨는 것이 출발 신호다:

```
[INFO] ★ 도면-라이다 자동 정렬 매칭 확인: start_x=1.74 start_y=1.40 start_yaw=355.1도 — 2초 반영 대기 후 출발합니다.
[INFO] ★ 탐사 시작 — 방 14개, 순서: #14 -> #10 -> ...
```

정렬 확인 원리: auto_align 은 측정 3회가 서로 일치할 때만 플래너에
`start_x/y/yaw` 를 넣는다(SetParameters). 그 순간 플래너가 내는
`/parameter_events` 변경 이벤트가 곧 '매칭 성공'이라, 탐사 노드는 그 이벤트를
기다린다. **정렬이 거부되면('정렬 측정이 서로 안 맞습니다' 에러) 출발하지 않고
계속 기다리며** 15초마다 안내를 띄운다 — 로봇을 정위치에 두고 다시 실행하거나,
`align_apply_timeout_sec:=60`(그 시간 뒤 그냥 출발) 또는 `require_align:=false`
(게이트 끔)로 풀 수 있다. `auto_align:=false` 로 실행하면 게이트도 같이 꺼져서
launch 지정값으로 바로 출발한다.

```bash
# UART/node.py 를 다른 터미널에서 이미 돌리고 있으면
ros2 launch ~/room_explorer/room_explorer.launch.py motor:=false

# 다른 도면 (방 있는 예제 도면)
ros2 launch ~/room_explorer/room_explorer.launch.py reference:=~/converted_maps/2d_ex.yaml

# 천천히 / 복귀 없이 / 재시도 없이
ros2 launch ~/room_explorer/room_explorer.launch.py speed_scale:=0.7 return_home:=false retry_skipped:=false
```

> ⚠ **`~/final_demo_test` 통합 데모와 동시에 못 돈다.** 둘 다 라이다 포트와
> `/cmd_vel` 을 쓴다. 데모 터미널을 Ctrl+C 로 끄고 실행할 것.
> (이 런치는 이전 `room_explorer`/`auto_drive` 실행은 스스로 정리하지만,
> `integrated.launch.py` 의 부모까지 죽이지는 않는다.)

주행 기본값은 실차로 검증된 조합을 그대로 쓴다:
`heading_offset 180 / control_mode smooth / turn_mode arc / lookahead 0.20 /
robot_radius 0.097 + safety_margin 0.015 (부풀림 0.112 = 이 도면의 상한 —
0.115부터 방 #1/#11 입구가 막힘) / clearance_prefer 0.10 / dynamic_obstacles true`.

### ★ 탐사할 방은 '좌표'로 지정한다 (room_points)

기본값이 사용자가 확정한 **실제 방 6개**다 — 빗살 위/아래 줄의 왼쪽 3칸씩.
오른쪽 끝 칸(x≈1.67)과 가운데 통로는 방이 아니라 빠져 있다.
방문 순서도 이 순서 그대로다(지그재그: 위줄 오른쪽→왼쪽, 아래줄 왼쪽→오른쪽).

```
room_points := "1.19,1.30; 0.71,1.30; 0.25,1.34; 0.25,0.26; 0.71,0.26; 1.19,0.26"
                   #1          #2          #3          #4          #5          #6
```

> **왜 번호가 아니라 좌표인가.** 예전에는 `exclude_rooms:="4,5,...,14"` 처럼
> 자동 분할 번호로 걸렀는데, **도면을 조금만 고치면 분할 개수가 바뀌어 번호가
> 통째로 밀린다.** 실제로 2026-08-06 도면 수정 후 방이 14개→10개가 되면서
> `#3` 이 다른 칸을 가리켰고, 로봇이 엉뚱한 곳(오른쪽 끝 칸)으로 갔다.
> 좌표는 실제 미로에 고정된 값이라 도면을 고쳐도 그대로 유효하다.

바꾸려면 좌표만 고치면 된다(순서 = 방문 순서):

```bash
ros2 launch ~/room_explorer/room_explorer.launch.py \
  room_points:="0.25,0.26; 0.71,0.26; 1.19,0.26"      # 아래줄 3칸만, 왼→오른쪽
ros2 launch ~/room_explorer/room_explorer.launch.py room_points:=""   # 자동 분할로
```

지정한 좌표가 맞는지 그림으로 먼저 확인할 수 있다
([preview_rooms.png](preview_rooms.png) 가 기본 6개를 그린 것):

```bash
python3 ~/room_explorer/room_check.py \
  --points "1.19,1.30; 0.71,1.30; 0.25,1.34; 0.25,0.26; 0.71,0.26; 1.19,0.26" \
  --png ~/room_explorer/preview_rooms.png
```

각 점의 벽 여유(0.13 m 이상이어야 함)와 방문 순서선이 함께 표시된다.

### 주행 전 미리보기 — 방이 몇 개로 나뉘나

```bash
python3 ~/room_explorer/room_check.py                               # 기본 미로
python3 ~/room_explorer/room_check.py ~/converted_maps/2d_ex.yaml   # 다른 도면
python3 ~/room_explorer/room_check.py --h-door 0.06 --png out.png   # 문턱 조정 + 그림
```

ASCII 로 방 배치가 찍히고, `--png` 를 주면 색칠된 그림이 저장된다.
원하는 개수로 나뉘는 `h_door` 를 찾아 launch 에 `h_door:=...` 로 주면 된다.

---

## 2. 동작 순서

1. **방 분할** — 도면 자유공간의 여유거리(distance transform)를 지형으로 보면
   방 한가운데는 봉우리, 문(좁은 통로)은 안장이다. 주변 안장보다
   `h_door`(0.04 m) 이상 도드라진 봉우리만 방의 씨앗으로 남기고(h-maxima),
   여유거리가 높은 곳부터 낮은 곳으로 흘러내리며 전체를 배정한다
   (priority flood). 실측: 미로 도면 14개, 2d_ex 도면 10개.
   * 도면 가장자리 여백(건물 밖 흰 공간)은 `border_margin` 으로 제외
   * 차체가 못 들어가는 좁은 곳(`min_room_clear`)과 부스러기(`min_room_area`) 제외
2. **방문 순서** — 로봇 위치에서 **실제 이동거리**(축소 격자 BFS)가 가까운
   방부터 탐욕 선택. 직선거리로 고르면 미로에서 벽 건너 방을 먼저 고른다.
   로봇과 이어져 있지 않은 방(도면 밖 등)은 이때 걸러진다.
3. **방의 정중앙** = 방 안에서 벽으로부터 가장 먼 지점. 도면 좌표를
   플래너의 `start_x/y/yaw`(auto_align 이 실행 중 갱신하는 값을 파라미터
   서비스로 매번 읽음)로 map 좌표로 바꿔 `/goal_pose` 로 보낸다.
4. **주행** 은 기존 `goal_path_planner_node`(A* + 동적장애물 우회) +
   `path_follower_node` 가 한다. 탐사 노드는 `/planner/status` 와
   `/auto_drive/goal_reached` 만 지켜본다.
5. **도착하면** 추종기를 일시정지(`/auto_drive/enable false`)시키고
   그 자리에서 360도 제자리 회전(키보드 `a` 와 같은 PWM 255 명령).
   TF yaw 로 누적 회전각을 재고, 정지마찰에 걸리면 추종기와 같은 방식으로
   앞뒤로 살짝 움직여 푼다(`spin_stall_sec`/`spin_nudge_*`).
   **회전은 120도마다 1.5초씩 멈추고**(`spin_pause_every_deg`), 끝난 뒤
   **5초 정착**(`post_spin_settle_sec`) 후에 다음 방으로 간다 — 아래
   '위치가 미끄러지던 문제' 대책이다.
6. **방을 나오는 것**은 다음 목적지로 가는 A* 경로가 문을 지나며 해결한다.
7. 다 돌면 **건너뛴 방을 한 번 더** 시도하고(`retry_skipped`),
   **출발 자리로 복귀**(`return_home`) 후 결과를 요약한다.

### 장애물로 못 갈 때 나오는 로그

플래너가 우회로까지 찾다가 실패하면(HALTED_NO_ROUTE) 이렇게 찍힌다:

```
[ERROR] ★ 장애물로 인해서 방 #2 (도면 0.71, 0.26) 에 갈 수 없습니다 — 우회로가
        없어 정지했습니다. 20초 안에 길이 안 열리면 건너뛰고 다음 방을 탐색합니다.
[ERROR] ✖ 방 #2 포기 (장애물로 막혀 우회로가 없음) — 이어서 다른 방들을 탐색합니다.
```

20초(`blocked_skip_sec`)를 기다리는 이유: 동적 장애물은 3초 TTL 로 지워지므로
사람이 지나가는 정도면 기다리는 동안 저절로 열린다. 우회 가능한 장애물은
건너뛰지 않는다 — `장애물 감지 — 우회 경로로 계속 이동합니다` 로그와 함께
그냥 돌아서 간다.

미션이 끝나면 요약이 나온다:

```
★ 탐사 종료 — 방 14개 중 13개 방문, 1개 실패
  ✔ 방 #14 (1.67, 1.35) 스캔 완료
  ...
  ✖ 방 #2  (0.71, 0.26) — 장애물로 인해 가지 못함
```

### ★ "방이 아니라 아예 다른 곳으로 간다" — 원인과 대책 (2026-08-05 실측)

첫 실주행에서 로봇이 엉뚱한 곳으로 갔다. 로그를 대조해 보니 **좌표는 전부
정확했다** — 탐사 노드가 보낸 방 #10 목적지를 플래너가 셀 (203,237) = 도면
(1.19, 1.02), 즉 방 #10 정중앙 그대로 해석했다. 진짜 원인은:

**출발점에서 360도를 쉼 없이 돌자(36.8초, 정지마찰 고착 반복) cartographer
위치가 빗살 한 칸(≈0.5 m) 옆으로 미끄러졌다.** 회전 전 로봇은 도면
(1.76, 1.36)에 있었는데 회전 후 TF 는 도면 (1.26, 1.28). 이 미로는 같은
빗살이 반복돼 회전 중 스캔매칭이 옆 칸에 걸리기 쉽다. 그 뒤로는 지도 전체가
한 칸 어긋난 채라 모든 목적지가 물리적으로 옆 칸에 떨어졌고(방 #10까지
경로가 0.32 m 밖에 안 나온 것이 증거), 정렬이 1회 고정(refine 0)이라
복구되지 않았다.

대책 3중 (전부 기본 켜짐):

1. **회전을 끊어서 돈다** — 120도마다 1.5초 완전 정지(`spin_pause_every_deg`
   / `spin_pause_sec`). 멈춘 동안 정지 스캔으로 위치를 다시 고정한다.
2. **회전 후 정착** — 다음 이동 전 5초 대기(`post_spin_settle_sec`).
3. **정렬 추적** — 이 launch 는 `refine_sec` 기본값을 0이 아니라 **2.0**
   으로 준다. auto_align 이 정지할 때마다 다시 재서 어긋난 정렬을 당겨오고
   (±6 cm 추적창, 크게 어긋나면 전역 재정렬), 탐사 노드는 그 갱신을
   파라미터 이벤트로 **즉시** 반영한다. 주행 중 4 cm/3도 이상 크게 바뀌면
   현재 목적지도 새 정렬로 다시 보낸다. 1회 고정으로 되돌리려면
   `refine_sec:=0.0`.

---

## 3. RViz / 상태 토픽

RViz 는 auto_drive.rviz 를 복사해 방 탐사 디스플레이를 더한
`room_explorer.rviz` 로 뜬다 (launch 가 자동으로 이걸 쓴다). 보이는 것:

| 표시 | 내용 |
|---|---|
| `5_계획경로(/plan)` | **지금 따라가는 A\* 경로 (초록 선)** — auto_drive 그대로 |
| `6_자율주행(추종점/도착)` | 노란 구슬 = 전방주시점, 도착 표시 |
| `7_방탐사(방번호·상태·순서선)` | 방 구슬+번호 (노랑=대기, 파랑=지금 가는 방, 초록=완료, 빨강=실패), 하늘색 선 = 앞으로 돌 방문 순서 |

| 토픽 | 내용 |
|---|---|
| `/room_explorer/markers` | 위 방 마커 + 순서선 (latched) |
| `/room_explorer/status` | 미션 상태 JSON (state, current, pending/visited/skipped, latched) |

```bash
ros2 topic echo /room_explorer/status
```

---

## 4. 자주 쓰는 인자

| 인자 | 기본값 | 설명 |
|---|---|---|
| `reference` | maze_195x162_fix.yaml | 기준 도면 (auto_drive 와 같은 것을 써야 한다) |
| `h_door` | 0.04 | 방 나누기 문턱[m]. 작을수록 잘게 나뉜다 |
| `min_room_area` | 0.05 | 이보다 작은 조각은 방이 아님[m²] |
| `min_room_clear` | 0.13 | 방의 최대 벽여유가 이보다 작으면 제외[m] |
| `border_margin` | 0.10 | 도면 가장자리에서 이 안에 중심이 있는 방 제외[m] |
| **`room_points`** | "1.19,1.30; 0.71,1.30; 0.25,1.34; 0.25,0.26; 0.71,0.26; 1.19,0.26" | **탐사할 방을 도면 좌표로, 방문 순서대로 지정** (사용자 확정 6개). 빈 값이면 자동 분할 사용 |
| `room_point_refine` | 0.12 | 지정 좌표를 이 반경 안에서 벽에서 가장 먼 자리로 보정[m] |
| `exclude_rooms` / `include_rooms` | "" | `room_points` 가 빈 값일 때만 쓰이는 번호 기반 필터 |
| `goal_tolerance` | **0.06** | 정중앙 도착 허용 오차. 감속(min<max)이 있어야 동작 — 더 줄이면 헌팅 위험 |
| `min_linear` / `min_angular` / `min_wheel_cmd` | 0.70/0.80/0.80 | 평상시 최대 출력, 목적지 앞 45cm만 감속(정밀 도착용). 항상 최대: 전부 1.0 + goal_tolerance:=0.12 |
| `use_imu_spin` | true | **회전각을 IMU 자이로 적분으로 측정** (1차). TF 는 대칭 미로에서 회전을 절반까지 깎아 세서(스냅) 로봇이 2바퀴 돌았다 — 자이로는 SLAM 과 무관하게 물리 회전을 그대로 잰다. IMU 끊기면 TF 폴백 |
| `imu_topic` | /imu | imu_fix_node 가 내는 보정된 IMU |
| `spin_jump_deg` | 30 | TF 폴백일 때 SLAM yaw 점프 제외 문턱 |
| `blocked_skip_sec` | **8** (20에서 단축) | 막힌 방 포기까지 대기 — 플래너 재시도 1.5초×5회 + 장애물 TTL 3초면 충분 |
| `spin_deg` | 360 | 정중앙에서 도는 스캔 각도 |
| `spin_pause_every_deg` | 120 | 회전 중 이 각도마다 잠깐 멈춤 (0=끔) |
| `spin_pause_sec` | 1.5 | 회전 중 멈춤 시간 |
| `post_spin_settle_sec` | 5 | 회전 후 다음 이동 전 정착 대기 |
| `refine_sec` | **2.0** (auto_drive 기본 0 과 다름) | 정렬 추적 주기, 0=1회 고정 |
| `blocked_skip_sec` | 20 | 막힘이 이만큼 지속되면 그 방 포기 |
| `goal_timeout_sec` | 180 | 방 하나에 쓰는 최대 시간 |
| `retry_skipped` | true | 한 바퀴 돈 뒤 못 간 방 재시도 (1회) |
| `return_home` | false 로 끄기 가능 | 다 돌면 출발 자리로 복귀 |
| `settle_sec` | 10 | 출발 전 SLAM/정렬 안정화 대기 |
| `require_align` | auto_align 과 동일 | 정렬 매칭 확인 후에만 출발하는 게이트 |
| `align_apply_timeout_sec` | 0 (무한 대기) | >0 이면 그 시간 뒤 경고 후 그냥 출발 |

`motor` / `use_rviz` / `speed_scale` / `heading_offset` / `start_x/y/yaw` /
`auto_align` 등 주행 인자는 그대로 auto_drive 로 전달된다.

---

## 5. 검증한 것

* **방 분할**: 미로 도면 14개(빗살 칸 8 + 통로 구간 6), 2d_ex 10개 방 —
  `preview_*.png` 로 확인 가능. `h_door` 0.03~0.06 에서 안정적.
* **미션 전체**(가짜 월드 — 실제 토픽/서비스 계약 그대로, 플래너 파라미터
  서비스 + TF + goal_reached 시뮬레이션): 방 14개 중 13개 방문·스캔(각 361도
  회전 실측), 막힌 방 1개는 장애물 로그 → 건너뜀 → 마지막에 재시도 →
  재실패 → 출발 자리 복귀 → 요약까지 74초에 완주.
* **launch**: `--print` 평가 통과 (인자 30개 + auto_drive include + 탐사 노드).
* 시작 위치가 이미 방 안이면 목적지를 보내지 않고 바로 스캔한다
  (한 점짜리 경로는 추종기가 도착 신호를 안 주는 구멍을 피함).

### 알아둘 것

* `planner:=drawing` 이어야 한다(기본값). 방 좌표가 도면 기준이기 때문.
* 정렬(auto_align)이 어긋나면 방 중앙도 같이 어긋난다. 탐사 노드는 목적지를
  보낼 때마다 플래너의 최신 `start_x/y/yaw` 를 읽으므로, 실행 중 재정렬도
  자동으로 따라간다. 파라미터 서비스를 20초간 못 읽으면 launch 지정값으로
  진행하고 경고를 남긴다.
* 스캔 회전 명령은 `/cmd_vel` 로 직접 낸다(PWM 255, keyboard `a` 동일).
  이 동안 추종기는 enable false 로 잠들어 있고, 플래너는 "제자리 회전 중"을
  인식해 장애물 오인식을 멈춘다(기존 `spin_grace_sec` 로직 재사용).
