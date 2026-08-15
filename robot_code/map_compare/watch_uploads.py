#!/usr/bin/env python3
"""
watch_uploads.py
----------------------------------------------------------
지정한 폴더(WATCH_DIR)를 계속 감시하다가 새로운 이미지 파일이
올라오면 자동으로 image_to_map.py의 변환 로직을 호출해서
pgm/yaml을 OUTPUT_DIR에 생성한다.

실행:
    python3 watch_uploads.py

종료:
    Ctrl+C

주의: cv2 가 없으면(=venv 로 실행하면) 자동으로 /usr/bin/python3 로
재실행한다. 시스템 파이썬에는 cv2/numpy/yaml 이 모두 있다.
----------------------------------------------------------
"""

import os
import sys

# --- cv2 가 없으면 시스템 파이썬으로 1회 자동 재실행 -------------------
# (자세한 이유는 image_to_map.py 상단 주석 참고)
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

import time
import shutil
from image_to_map import convert_image_to_map, print_report, FIXED_RESOLUTION

# ---- 경로 설정 (환경에 맞게 수정) ----
WATCH_DIR = os.path.expanduser("~/map_uploads")       # 여기에 이미지 파일을 넣으면 자동 감지
OUTPUT_DIR = os.path.expanduser("~/converted_maps")   # 변환 결과(pgm/yaml)가 저장될 폴더
PROCESSED_DIR = os.path.join(WATCH_DIR, "processed")           # 처리 완료된 원본 이미지 보관 폴더
POLL_INTERVAL_SEC = 2                                          # 폴더 확인 주기(초)

VALID_EXT = {".png", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff"}


def ensure_dirs():
    os.makedirs(WATCH_DIR, exist_ok=True)
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    os.makedirs(PROCESSED_DIR, exist_ok=True)


def is_image_file(filename: str) -> bool:
    _, ext = os.path.splitext(filename.lower())
    return ext in VALID_EXT


def wait_until_stable(filepath: str, checks: int = 3, interval: float = 0.5) -> bool:
    """파일 업로드(복사)가 완전히 끝났는지 확인. 크기가 연속으로 같으면 완료된 것으로 판단."""
    last_size = -1
    stable_count = 0
    for _ in range(30):  # 최대 15초 대기
        try:
            size = os.path.getsize(filepath)
        except OSError:
            return False
        if size == last_size:
            stable_count += 1
            if stable_count >= checks:
                return True
        else:
            stable_count = 0
        last_size = size
        time.sleep(interval)
    return False


def process_file(filename: str):
    input_path = os.path.join(WATCH_DIR, filename)
    print(f"\n[감지됨] 새 이미지: {filename}")

    if not wait_until_stable(input_path):
        print(f"[경고] 파일이 안정화되지 않았습니다. 건너뜁니다: {filename}")
        return

    output_name = os.path.splitext(filename)[0]
    try:
        result = convert_image_to_map(
            input_path=input_path,
            output_name=output_name,
            output_dir=OUTPUT_DIR,
            resolution=FIXED_RESOLUTION,
        )
        print_report(result, input_path)

        # 처리 완료된 원본을 processed 폴더로 이동 (재처리 방지)
        dst = os.path.join(PROCESSED_DIR, filename)
        if os.path.exists(dst):
            os.remove(dst)  # 같은 이름 재업로드 시 이동 실패 방지
        shutil.move(input_path, dst)
        print(f"[완료] {filename} -> {OUTPUT_DIR}/{output_name}.pgm(.yaml) 생성, "
              f"원본은 processed/ 로 이동")
    except Exception as e:
        print(f"[에러] {filename} 변환 실패: {e}")


def main():
    ensure_dirs()
    print("=" * 55)
    print(" 도면 이미지 자동 변환 감시 시작")
    print(f" 감시 폴더 : {WATCH_DIR}")
    print(f" 출력 폴더 : {OUTPUT_DIR}")
    print(f" 고정 resolution : {FIXED_RESOLUTION} m/px")
    print(" 이 폴더에 이미지 파일을 넣으면 자동으로 변환됩니다.")
    print(" 종료하려면 Ctrl+C")
    print("=" * 55)

    # 시작 시점에 이미 있던 파일은 무시 (새로 들어오는 것만 처리)
    seen = set(os.listdir(WATCH_DIR))

    try:
        while True:
            time.sleep(POLL_INTERVAL_SEC)
            current = set(os.listdir(WATCH_DIR))
            new_files = sorted(current - seen)

            for filename in new_files:
                # 처리 여부와 무관하게 seen 에 즉시 추가한다.
                # (변환에 최대 15초가 걸리므로, 그 사이 새로 들어온 파일을
                #  다음 루프에서 놓치지 않도록 wholesale 리셋 대신 개별 추가)
                seen.add(filename)

                full_path = os.path.join(WATCH_DIR, filename)
                if os.path.isdir(full_path):
                    continue
                if not is_image_file(filename):
                    continue
                process_file(filename)

    except KeyboardInterrupt:
        print("\n감시를 종료합니다.")


if __name__ == "__main__":
    main()
