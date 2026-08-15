#!/usr/bin/env python3
"""
room_check.py
----------------------------------------------------------
도면이 몇 개의 방으로 나뉘는지 주행 전에 미리 확인하는 도구.
로봇/ROS 없이 도면 파일만 읽는다.

  python3 room_check.py                                   # 기본 미로 도면
  python3 room_check.py ~/converted_maps/2d_ex.yaml       # 다른 도면
  python3 room_check.py --h-door 0.06 --png preview.png   # 문턱 조정 + 그림 저장

h_door 를 바꿔 가며 원하는 개수로 나뉘는 값을 찾은 뒤, 그 값을
room_explorer.launch.py 에 h_door:=... 로 주면 된다.
----------------------------------------------------------
"""

import argparse
import os
import sys

_SYS_PY = "/usr/bin/python3"
try:
    import numpy  # noqa: F401
    import cv2    # noqa: F401
except ModuleNotFoundError:
    if os.path.abspath(sys.executable) != _SYS_PY and os.path.exists(_SYS_PY):
        _env = dict(os.environ)
        _env.pop("VIRTUAL_ENV", None)
        os.execve(_SYS_PY, [_SYS_PY] + sys.argv, _env)
    raise

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import room_segment as rs


def save_png(path, occ, unk, label, rooms, dist, res, points=None):
    """방을 색으로 칠한 그림을 저장한다 (위가 도면의 위쪽이 되게 뒤집어서).

    points 를 주면(도면 좌표 목록) 자동 분할 대신 그 점들을 방문 순서대로
    번호와 함께 표시한다 — launch 의 room_points 를 눈으로 확인하는 용도.
    """
    h, w = occ.shape
    img = np.full((h, w, 3), 255, np.uint8)
    img[unk] = (190, 190, 190)
    rng = np.random.default_rng(7)
    for rm in rooms:
        color = tuple(int(c) for c in rng.integers(60, 230, 3))
        img[label == rm["raw"]] = color
    img[occ] = (30, 30, 30)
    scale = max(1, int(round(900 / max(h, w))))
    img = cv2.resize(img, (w * scale, h * scale),
                     interpolation=cv2.INTER_NEAREST)
    # ★ 글자·표식은 뒤집은 뒤에 그린다. 먼저 그리면 글자가 거울처럼 뒤집힌다.
    img = np.flipud(img).copy()
    ih = img.shape[0]

    def px(x, y):
        return int(x / res * scale), ih - int(y / res * scale)

    if points:
        prev = None
        for i, (x, y) in enumerate(points):
            p = px(x, y)
            if prev is not None:                    # 방문 순서 선
                cv2.line(img, prev, p, (255, 160, 0), 2)
            prev = p
        for i, (x, y) in enumerate(points):
            p = px(x, y)
            cv2.circle(img, p, 9, (0, 140, 0), -1)
            cv2.putText(img, "%d" % (i + 1), (p[0] + 11, p[1] - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 100, 0), 2)
    else:
        for rm in rooms:
            p = px(rm["cx"], rm["cy"])
            cv2.circle(img, p, 5, (0, 0, 255), -1)
            cv2.putText(img, "#%d" % rm["id"], (p[0] + 6, p[1] - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
    cv2.imwrite(path, img)


def main():
    ap = argparse.ArgumentParser(description="도면 방 나누기 미리보기")
    ap.add_argument("yaml", nargs="?", default=os.path.expanduser(
        "~/ping_detour_auto_drive/maps/maze_195x162_fix.yaml"))
    ap.add_argument("--h-door", type=float, default=0.04,
                    help="방 나누기 문턱[m] (기본 0.04, 작을수록 잘게)")
    ap.add_argument("--min-area", type=float, default=0.05,
                    help="최소 방 넓이[m^2] (기본 0.05)")
    ap.add_argument("--min-clear", type=float, default=0.13,
                    help="최소 벽여유[m] (기본 0.13)")
    ap.add_argument("--border", type=float, default=0.10,
                    help="도면 가장자리 제외 폭[m] (기본 0.10)")
    ap.add_argument("--png", default="",
                    help="색칠한 그림을 저장할 경로 (예: preview.png)")
    ap.add_argument("--points", default="",
                    help="launch 의 room_points 를 그대로 붙여 확인 "
                         "('x,y; x,y; ...'). 각 점의 벽여유와 방문 순서를 "
                         "출력하고 --png 에도 번호를 그린다")
    args = ap.parse_args()

    occ, unk, res = rs.load_drawing(args.yaml)
    rooms, label, dist = rs.segment_rooms(
        occ, unk, res, h_door=args.h_door, min_area=args.min_area,
        min_clear=args.min_clear, border_margin=args.border)
    h, w = occ.shape
    print("도면 %s  %.2f x %.2f m (%dx%d @ %.4f m)"
          % (os.path.basename(args.yaml), w * res, h * res, w, h, res))
    print("방 %d개 (h_door %.2f, min_area %.2f, min_clear %.2f, border %.2f)"
          % (len(rooms), args.h_door, args.min_area, args.min_clear,
             args.border))
    for rm in rooms:
        print("  방 #%-2d 정중앙 (%.2f, %.2f)  넓이 %.3f m^2  벽여유 %.2f m"
              % (rm["id"], rm["cx"], rm["cy"], rm["area"], rm["peak"]))
    points = []
    if args.points:
        for tok in args.points.replace("|", ";").split(";"):
            tok = tok.strip()
            if not tok:
                continue
            xs, ys = tok.replace(":", ",").split(",")[:2]
            points.append((float(xs), float(ys)))
        print("\n=== room_points (%d개, 이 순서대로 방문) ===" % len(points))
        h, w = occ.shape
        for i, (x, y) in enumerate(points):
            r, c = int(y / res), int(x / res)
            ok = 0 <= r < h and 0 <= c < w
            clr = float(dist[r, c]) if ok else -1.0
            print("  %d. (%.2f, %.2f)  벽여유 %.3f m  %s"
                  % (i + 1, x, y, clr,
                     "OK" if clr >= 0.13 else "★ 좁음/벽 — 확인 필요"))
    print()
    for line in rs.ascii_map(occ, label, rooms):
        print("  " + line)
    if args.png:
        save_png(os.path.expanduser(args.png), occ, unk, label, rooms,
                 dist, res, points=points)
        print("\n그림 저장: %s" % args.png)


if __name__ == "__main__":
    main()
