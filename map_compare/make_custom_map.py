#!/usr/bin/env python3
"""
make_custom_map.py
----------------------------------------------------------
벽/장애물을 좌표로 직접 정의해서 ROS2 nav2_map_server용
.pgm + .yaml 지도를 만드는 커스텀 맵 생성기.

아래 "여기만 수정" 구역의 WALLS / BOXES / CIRCLES 와
지도 크기(MAP_W, MAP_H)만 고치면 원하는 도면을 만들 수 있다.
좌표 단위는 전부 미터(m), 원점(0,0)은 지도의 왼쪽 아래 모서리.

실행:
    python3 make_custom_map.py [출력이름] [출력폴더]
    (생략 시 map_baseline / ~/map_compare/custom_maps)

주의: cv2/numpy/yaml 이 필요하다. cv2 가 없는 파이썬(가상환경 등)에서 돌리면
자동으로 /usr/bin/python3 로 재실행한다.
----------------------------------------------------------
"""

import os
import sys

# --- cv2 가 없으면 시스템 파이썬으로 1회 자동 재실행 -------------------
_SYS_PY = "/usr/bin/python3"
try:
    import cv2  # noqa: F401
except ModuleNotFoundError:
    if os.path.abspath(sys.executable) != _SYS_PY and os.path.exists(_SYS_PY):
        _env = dict(os.environ)
        _env.pop("VIRTUAL_ENV", None)
        os.execve(_SYS_PY, [_SYS_PY] + sys.argv, _env)
    raise
# ---------------------------------------------------------------------

import cv2
import numpy as np
import yaml


# ======================================================================
#                          ▼▼▼ 여기만 수정 ▼▼▼
# ======================================================================
# 벽돌(0.19 x 0.05 m, 반벽돌 0.095 m) 미로 도면을 좌표로 옮긴 것.
# 전체 1.95 x 1.62 m, 원점(0,0)=왼쪽 아래.
#   - 아래/위 외벽: 전체 폭 1.95 m, 왼쪽 외벽: 벽돌 8개 = 1.52 m
#   - 오른쪽 면은 개방. 위/아래-오른쪽 모서리 안쪽에 세로 반벽돌(0.095)
#     스텁이 있고, 그 앞(x 1.85~1.90)에 떠 있는 세로 벽이 있다.
#     떠 있는 벽 = 세로 벽돌 4개(0.76) + 위/아래 반벽돌 캡, y 0.38~1.24
#     (도면의 0.38 치수 = 지도 위/아래 가장자리에서 캡까지 거리)
#   - 위/아래 벽에서 나온 T자 벽 6개: 세로 벽돌 2개(0.38) + 가로 캡(0.19)
#   - 가운데 가로 벽: 벽돌 6개 = 1.14 m, 바닥 벽 위 0.735 m 지점
#     (0.735 = 위/아래 외벽 안쪽 면 ~ 가운데 벽 간격, 세로 중앙 대칭)
MAP_W, MAP_H = 1.95, 1.62       # 지도 크기 [m]  (가로, 세로)
RESOLUTION = 0.005              # 해상도 [m/px]  (5mm/px: 5cm 벽이 딱 10px)
WALL_THICKNESS_M = 0.05         # 벽 두께 [m]  (벽돌 폭 5cm 통일)

# 벽: 선분 [x1, y1, x2, y2]  또는  [x1, y1, x2, y2, 두께(m)]
#   좌표는 벽 중심선 기준. 두께를 생략하면 WALL_THICKNESS_M 을 사용.
WALLS = [
    # --- 외벽 ---
    [0.0,   0.025, 1.95,  0.025],    # 아래 외벽 (전체 폭)
    [0.0,   1.595, 1.95,  1.595],    # 위 외벽 (전체 폭)
    [0.025, 0.0,   0.025, 1.62],     # 왼쪽 외벽
    [1.925, 1.475, 1.925, 1.57],     # 위-오른쪽 모서리 세로 반벽돌 (0.095)
    [1.925, 0.05,  1.925, 0.145],    # 아래-오른쪽 모서리 세로 반벽돌 (0.095)

    # --- 위 외벽에서 내려오는 T자 벽 3개 (세로 0.38 + 끝 가로 캡 0.19) ---
    [0.475, 1.19, 0.475, 1.57],      # 세로 (x중심 0.475)
    [0.38,  1.165, 0.57, 1.165],     # 캡
    [0.95,  1.19, 0.95,  1.57],      # 세로 (x중심 0.95)
    [0.855, 1.165, 1.045, 1.165],    # 캡
    [1.425, 1.19, 1.425, 1.57],      # 세로 (x중심 1.425)
    [1.33,  1.165, 1.52, 1.165],     # 캡

    # --- 아래 외벽에서 올라오는 T자 벽 3개 (위와 x 정렬 동일) ---
    [0.475, 0.05, 0.475, 0.43],      # 세로
    [0.38,  0.455, 0.57, 0.455],     # 캡
    [0.95,  0.05, 0.95,  0.43],      # 세로
    [0.855, 0.455, 1.045, 0.455],    # 캡
    [1.425, 0.05, 1.425, 0.43],      # 세로
    [1.33,  0.455, 1.52, 0.455],     # 캡

    # --- 왼쪽 외벽에서 돌출된 반벽돌 스텁 2개 (캡과 같은 높이) ---
    [0.05, 1.165, 0.145, 1.165],     # 위 스텁 (0.095)
    [0.05, 0.455, 0.145, 0.455],     # 아래 스텁 (0.095)

    # --- 가운데 가로 벽 (벽돌 6개 = 1.14 m, 아래 외벽 윗면에서 0.735 위) ---
    [0.38, 0.81, 1.52, 0.81],        # y 0.785~0.835 (지도 세로 중앙)

    # --- 오른쪽 떠 있는 세로 벽 + 위/아래 반벽돌 캡 (0.095) ---
    # 도면상 이 기둥은 위/아래 모서리 스텁과 같은 x(맵 오른쪽 끝에 붙음)다.
    # 예전 좌표는 0.05 m 안쪽(1.85~1.90)이라 기둥 바깥에 폭 0.05 m 짜리
    # 막다른 슬롯이 생기고 오른쪽 벽선이 끊겨 보였다.
    [1.925, 0.38, 1.925, 1.24],      # 세로 벽돌 4개 (x 1.90~1.95)
    [1.855, 1.215, 1.95, 1.215],     # 위 캡 (y 1.19~1.24, 왼쪽으로 0.095)
    [1.855, 0.405, 1.95, 0.405],     # 아래 캡 (y 0.38~0.43)
]
# 채워진 사각형 장애물: [x1, y1, x2, y2]  (없으면 빈 리스트)
BOXES = [
]

# 원기둥 장애물: [cx, cy, r]  (없으면 빈 리스트)
CIRCLES = [
]

# 참고 — 오른쪽 면이 개방부: 모서리 스텁과 떠 있는 벽 캡 사이
#        위 개구부 y=1.24~1.475, 아래 개구부 y=0.145~0.38 (각 0.235 m).
# ======================================================================
#                          ▲▲▲ 여기까지 ▲▲▲
# ======================================================================

FREE = 254
OCC = 0


def to_px(x, y, h_px):
    """월드 좌표(m) → 픽셀 (col, row). 원점은 왼쪽 아래."""
    return int(round(x / RESOLUTION)), h_px - int(round(y / RESOLUTION))


def build_grid():
    w_px = int(round(MAP_W / RESOLUTION))
    h_px = int(round(MAP_H / RESOLUTION))
    grid = np.full((h_px, w_px), FREE, dtype=np.uint8)   # 전체 free 로 시작

    for wall in WALLS:
        x1, y1, x2, y2 = wall[:4]
        t_m = wall[4] if len(wall) > 4 else WALL_THICKNESS_M   # 벽별 두께(없으면 기본값)
        t_px = max(1, int(round(t_m / RESOLUTION)))
        c1, r1 = to_px(x1, y1, h_px)
        c2, r2 = to_px(x2, y2, h_px)
        a = t_px // 2
        # cv2.line 은 두께를 주면 ±1px 넓게 그려져 정확히 안 맞는다.
        # 수평/수직 벽은 정확한 사각형으로 그려 두께를 t_px 로 딱 맞춘다.
        if r1 == r2:        # 수평 벽
            cv2.rectangle(grid, (min(c1, c2), r1 - a),
                          (max(c1, c2), r1 - a + t_px - 1), OCC, -1)
        elif c1 == c2:      # 수직 벽
            cv2.rectangle(grid, (c1 - a, min(r1, r2)),
                          (c1 - a + t_px - 1, max(r1, r2)), OCC, -1)
        else:               # 대각선 벽 (근사)
            cv2.line(grid, (c1, r1), (c2, r2), OCC, thickness=t_px)

    for x1, y1, x2, y2 in BOXES:
        cv2.rectangle(grid, to_px(x1, y1, h_px), to_px(x2, y2, h_px),
                      color=OCC, thickness=-1)

    for cx, cy, r in CIRCLES:
        cv2.circle(grid, to_px(cx, cy, h_px), int(round(r / RESOLUTION)),
                   color=OCC, thickness=-1)

    return grid, w_px, h_px


def save_map(output_name, output_dir):
    grid, w_px, h_px = build_grid()
    os.makedirs(output_dir, exist_ok=True)
    pgm_path = os.path.join(output_dir, output_name + ".pgm")
    yaml_path = os.path.join(output_dir, output_name + ".yaml")

    if not cv2.imwrite(pgm_path, grid):
        raise IOError(f"pgm 저장 실패: {pgm_path}")

    meta = {
        'image': output_name + ".pgm",
        'mode': 'trinary',
        'resolution': float(RESOLUTION),
        'origin': [0.0, 0.0, 0.0],       # 원점 = 지도 왼쪽 아래
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.196,
    }
    with open(yaml_path, 'w') as f:
        yaml.safe_dump(meta, f, sort_keys=False, default_flow_style=None)

    return pgm_path, yaml_path, w_px, h_px


def suggest_pose(grid, h_px, w_px):
    """벽에서 충분히 떨어진(로봇 반경 여유) free 셀 하나를 추천 시작 위치로."""
    occ = (grid == OCC).astype(np.uint8)
    dist = cv2.distanceTransform(1 - occ, cv2.DIST_L2, 5)  # free 셀의 벽까지 거리(px)
    row, col = np.unravel_index(int(np.argmax(dist)), dist.shape)
    x = col * RESOLUTION
    y = (h_px - row) * RESOLUTION
    clearance = float(dist[row, col]) * RESOLUTION
    return round(x, 2), round(y, 2), round(clearance, 2)


if __name__ == "__main__":
    output_name = sys.argv[1] if len(sys.argv) > 1 else "map_baseline"
    output_dir = sys.argv[2] if len(sys.argv) > 2 \
        else os.path.expanduser("~/map_compare/custom_maps")

    pgm_path, yaml_path, w_px, h_px = save_map(output_name, output_dir)
    grid, _, _ = build_grid()
    px, py, clr = suggest_pose(grid, h_px, w_px)

    print("=" * 58)
    print("맵 생성 완료")
    print(f"  -> {pgm_path}")
    print(f"  -> {yaml_path}")
    print(f"  크기 : {MAP_W} x {MAP_H} m  ({w_px} x {h_px} px @ {RESOLUTION} m/px)")
    print(f"  벽 {len(WALLS)}개 / 상자 {len(BOXES)}개 / 원기둥 {len(CIRCLES)}개")
    print(f"  추천 로봇 시작 pose: {px},{py}  (벽까지 여유 {clr} m)")
    print("=" * 58)
    print("RViz 로 보기 (ROS 환경 source 후):")
    print(f"  python3 ~/slam_test_maps/slam_map_kit.py run \\")
    print(f"      --map {yaml_path} --rviz --pose {px},{py},0")
    print("=" * 58)

