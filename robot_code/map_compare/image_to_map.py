#!/usr/bin/env python3
"""
image_to_map.py
----------------------------------------------------------
2D 도면 이미지(png/jpg 등) 1장을 ROS2 nav2_map_server용
map.pgm + map.yaml 로 자동 변환하는 스크립트.

단독 실행:
    python3 image_to_map.py <입력이미지> <출력이름(확장자 없이)> [출력폴더]

예시:
    python3 image_to_map.py floorplan.png map_from_image
    -> map_from_image.pgm, map_from_image.yaml 생성

주의: 이 스크립트는 cv2/numpy/yaml 이 필요하다. cv2 가 없는 파이썬(가상환경 등)에서
돌리면 자동으로 /usr/bin/python3 로 재실행한다
(시스템 파이썬에는 cv2 4.5.4 / numpy / yaml 이 이미 설치돼 있음).
----------------------------------------------------------
"""

import os
import sys

# --- cv2 가 없으면 시스템 파이썬으로 1회 자동 재실행 -------------------
# venv 의 python3 는 /usr/bin/python3 와 같은 바이너리를 심링크하지만
# 옆의 pyvenv.cfg 때문에 venv site-packages(=cv2 없음)를 쓴다.
# /usr/bin/python3 를 '경로 그대로' 다시 실행하면(그 옆엔 pyvenv.cfg 가 없어)
# 시스템 site-packages(cv2 4.5.4 / numpy / yaml)로 동작한다.
_SYS_PY = "/usr/bin/python3"
try:
    import cv2  # noqa: F401
except ModuleNotFoundError:
    if os.path.abspath(sys.executable) != _SYS_PY and os.path.exists(_SYS_PY):
        _env = dict(os.environ)
        _env.pop("VIRTUAL_ENV", None)          # venv 환경변수 제거
        os.execve(_SYS_PY, [_SYS_PY] + sys.argv, _env)
    raise  # 시스템 파이썬에도 없으면 그대로 에러
# ---------------------------------------------------------------------

import cv2
import numpy as np
import yaml

# 항상 고정할 해상도 (m/px)
FIXED_RESOLUTION = 0.016667


def convert_image_to_map(input_path: str, output_name: str, output_dir: str = ".",
                          resolution: float = FIXED_RESOLUTION,
                          clean_noise: bool = True,
                          min_span_ratio: float = 0.15) -> dict:
    """
    이미지 파일을 읽어 occupancy grid pgm/yaml 로 변환.

    핵심 필터링 원리 (min_span_ratio):
      실제 건물 벽은 서로 코너/T자 접합부에서 맞닿아 있어서
      "하나의 거대한 연결 덩어리(connected component)"를 이룬다.
      반면 글자, 아이콘, 문 스윙 곡선 등은 벽과 물리적으로 붙어있지 않아
      항상 "작고 국소적인 별개의 덩어리"로 분리되어 나타난다.

      따라서 각 연결 덩어리의 가로/세로 폭이 전체 이미지 크기의
      min_span_ratio(기본 15%) 이상을 차지하는지만 보면
      벽(건물 전체에 걸쳐 넓게 퍼짐) vs 글자/아이콘(방 하나 크기로 국소적)을
      확실하게 구분할 수 있다.

    반환값: 변환 결과 요약 정보(dict)
    """
    if not os.path.isfile(input_path):
        raise FileNotFoundError(f"입력 파일을 찾을 수 없습니다: {input_path}")

    # 1. 이미지 로드 (그레이스케일로)
    img = cv2.imread(input_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise ValueError(f"이미지를 읽을 수 없습니다 (지원 포맷 확인 필요): {input_path}")

    # 2. Otsu 임계값으로 이진화 (벽=어두운 점, 배경=밝은 값 가정)
    _, binary = cv2.threshold(img, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

    # 3. 극성(polarity) 자동 판별
    #    이진화 후 흰색(255) 픽셀이 다수여야 정상(배경=free 가 더 넓음).
    #    검은색이 다수면 반전된 것이므로 뒤집어준다.
    white_ratio = float(np.mean(binary == 255))
    if white_ratio < 0.5:
        binary = cv2.bitwise_not(binary)
        white_ratio = 1.0 - white_ratio

    removed_components = 0
    kept_components = 0

    if clean_noise:
        h_img, w_img = binary.shape

        # 3-1. 아주 작은 픽셀 덩어리(먼지) 노이즈만 제거 (벽 두께 자체는 보존)
        kernel2 = np.ones((2, 2), np.uint8)
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel2, iterations=1)

        # 3-2. 핵심 필터: 연결 성분의 bbox 가 이미지 전체 크기의 일정 비율 이상
        #      차지해야만 "벽"으로 인정. 그렇지 않으면 글자/아이콘으로 간주해 제거.
        occupied = (binary == 0).astype(np.uint8)
        n_labels, labels, stats, _ = cv2.connectedComponentsWithStats(
            occupied, connectivity=8)

        wall_mask = np.zeros_like(occupied)
        for i in range(1, n_labels):  # 0번 라벨은 배경
            x, y, w, h, area = stats[i]
            spans_wide = w >= min_span_ratio * w_img
            spans_tall = h >= min_span_ratio * h_img
            if spans_wide or spans_tall:
                wall_mask[labels == i] = 1
                kept_components += 1
            else:
                removed_components += 1

        # occupied=0(검정), free=255(흰색) 형식으로 되돌림
        binary = np.where(wall_mask == 1, 0, 255).astype(np.uint8)

    # 5. pgm 저장 (표준: free=255, occupied=0)
    os.makedirs(output_dir, exist_ok=True)
    pgm_path = os.path.join(output_dir, f"{output_name}.pgm")
    yaml_path = os.path.join(output_dir, f"{output_name}.yaml")
    if not cv2.imwrite(pgm_path, binary):
        raise IOError(f"pgm 저장 실패: {pgm_path}")

    # 6. yaml 저장 (resolution 은 항상 고정값 사용)
    meta = {
        'image': f'{output_name}.pgm',
        'mode': 'trinary',
        'resolution': float(resolution),
        'origin': [0.0, 0.0, 0.0],
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.196,
    }
    with open(yaml_path, 'w') as f:
        yaml.safe_dump(meta, f, sort_keys=False, default_flow_style=None)

    h_px, w_px = binary.shape
    real_w_m = w_px * resolution
    real_h_m = h_px * resolution

    result = {
        'pgm_path': pgm_path,
        'yaml_path': yaml_path,
        'width_px': w_px,
        'height_px': h_px,
        'resolution': resolution,
        'real_width_m': real_w_m,
        'real_height_m': real_h_m,
        'white_ratio': white_ratio,
        'kept_components': kept_components,
        'removed_components': removed_components,
    }
    return result


def print_report(result: dict, input_path: str):
    print("=" * 55)
    print(f"[변환 완료] {input_path}")
    print(f"  -> {result['pgm_path']}")
    print(f"  -> {result['yaml_path']}")
    print(f"  이미지 크기 : {result['width_px']} x {result['height_px']} px")
    print(f"  resolution  : {result['resolution']} m/px (고정값)")
    print(f"  환산 실제 크기: {result['real_width_m']:.2f} m x {result['real_height_m']:.2f} m")
    if 'kept_components' in result:
        print(f"  벽으로 인정된 덩어리: {result['kept_components']}개 / "
              f"글자·아이콘으로 판단해 제거된 덩어리: {result['removed_components']}개")
    if result['real_width_m'] > 50 or result['real_height_m'] > 50:
        print("  ※ 주의: 환산된 실제 크기가 비정상적으로 큽니다.")
        print("     입력 이미지 해상도(px)가 너무 크거나, 실제 스케일과 안 맞을 수 있습니다.")
        print("     원본 도면 이미지를 실제 축척에 맞게 리사이즈 한 뒤 넣는 것을 권장합니다.")
    print("=" * 55)


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("사용법: python3 image_to_map.py <입력이미지> <출력이름> [출력폴더]")
        print("예시  : python3 image_to_map.py floorplan.png map_from_image")
        sys.exit(1)

    input_path = sys.argv[1]
    output_name = sys.argv[2]
    output_dir = sys.argv[3] if len(sys.argv) > 3 else "."

    try:
        result = convert_image_to_map(input_path, output_name, output_dir)
        print_report(result, input_path)
    except Exception as e:
        print(f"[에러] 변환 실패: {e}")
        sys.exit(1)
