#!/usr/bin/env python3
"""
room_segment.py
----------------------------------------------------------
2D 도면(pgm/yaml)에서 '방'을 자동으로 나누는 라이브러리.
room_explorer_node.py 와 room_check.py 가 같이 쓴다.

원리
  자유공간의 여유거리(distance transform)를 지형으로 보면,
  방의 한가운데는 봉우리이고 문(좁은 통로)은 안장(saddle)이다.
  주변 안장보다 h_door 이상 높은 봉우리(h-maxima)만 마커로 남기고,
  여유거리가 높은 곳부터 낮은 곳으로 흘러내리며(priority flood)
  자유공간 전체를 마커별로 배정하면 방 단위로 나뉜다.

  h_door 가 작을수록 잘게 나뉘고, 클수록 통짜로 붙는다.
  maze_195x162(빗살 미로)는 0.03~0.05 에서 빗살 칸+통로 구간 14~15개,
  2d_ex(방 있는 도면)는 0.04~0.06 에서 안쪽 방 10개가 나온다(실측).

방의 '정중앙'
  방 영역 안에서 벽으로부터 가장 먼 지점(여유거리 최대점)을 쓴다.
  ㄱ자 방에서도 항상 방 안에 있고, 제자리 회전 여유가 가장 큰 자리다.
  (도형 무게중심은 ㄱ자 방에서 벽 안에 떨어질 수 있다)
----------------------------------------------------------
"""

import heapq
import math
import os
import sys
from collections import deque

import cv2
import numpy as np


def load_drawing(yaml_path):
    """도면을 읽는다. 반환: (occ, unk, res)

    occ: bool (H,W), True=벽. 세로로 뒤집어 row 0 = 아래(도면 y=0).
         goal_path_planner_node._load_drawing 과 같은 좌표 규약이다.
    unk: bool (H,W), True=미지(회색) — 자유공간으로 치지 않는다.
    res: 셀 한 칸의 크기[m]
    """
    sys.path.insert(0, os.path.expanduser("~/slam_test_maps"))
    from slam_map_kit import read_pgm, parse_yaml
    yp = os.path.expanduser(yaml_path)
    meta = parse_yaml(yp)
    res = float(meta["resolution"])
    occ_t = float(meta.get("occupied_thresh", 0.65))
    free_t = float(meta.get("free_thresh", 0.196))
    pgm = os.path.join(os.path.dirname(os.path.abspath(yp)),
                       os.path.basename(meta["image"]))
    w, h, _mx, px = read_pgm(pgm)
    img = np.frombuffer(bytes(px), dtype=np.uint8).reshape(h, w)
    p = (255.0 - img.astype(np.float32)) / 255.0
    occ = p > occ_t
    unk = (p > free_t) & (p <= occ_t)
    return np.flipud(occ), np.flipud(unk), res


def h_maxima_markers(dist, h):
    """여유거리 지형에서 주변 안장보다 h 이상 높은 봉우리 영역을 True 로.

    형태학적 재구성: (dist - h) 를 시드로, dist 를 천장으로 반복 팽창.
    수렴한 결과는 '봉우리마다 h 만큼 깎인 지형'이고, 원래 지형과의 차이가
    남는 곳이 곧 h-maxima 다.
    """
    seed = np.clip(dist - h, 0.0, None)
    kernel = np.ones((3, 3), np.uint8)
    while True:
        nxt = np.minimum(cv2.dilate(seed, kernel), dist)
        if np.array_equal(nxt, seed):
            break
        seed = nxt
    return (dist - seed) > 1e-6


def priority_flood(dist, markers, free):
    """마커에서 시작해 여유거리 높은 곳 -> 낮은 곳 순서로 자유공간을 배정.

    반환: (label int32 (H,W) — 0 은 미배정, 마커 개수 n)
    """
    h, w = dist.shape
    n, lab = cv2.connectedComponents(markers.astype(np.uint8), connectivity=8)
    label = np.where(markers, lab, 0).astype(np.int32)
    # numpy 원소 접근은 파이썬 루프 안에서 비싸다(플래너 astar 와 같은 이유).
    dl = dist.tolist()
    labl = label.tolist()
    freel = free.tolist()
    pq = []
    ys, xs = np.nonzero(markers)
    for y, x in zip(ys.tolist(), xs.tolist()):
        heapq.heappush(pq, (-dl[y][x], y, x))
    nbrs = ((-1, 0), (1, 0), (0, -1), (0, 1),
            (-1, -1), (-1, 1), (1, -1), (1, 1))
    while pq:
        _negd, y, x = heapq.heappop(pq)
        l = labl[y][x]
        for dy, dx in nbrs:
            ny, nx = y + dy, x + dx
            if 0 <= ny < h and 0 <= nx < w and freel[ny][nx] \
                    and labl[ny][nx] == 0:
                labl[ny][nx] = l
                heapq.heappush(pq, (-dl[ny][nx], ny, nx))
    return np.asarray(labl, np.int32), n - 1


def segment_rooms(occ, unk, res, h_door=0.04, min_area=0.05,
                  min_clear=0.13, border_margin=0.10):
    """방 목록을 만든다.

    반환: (rooms, label, dist)
      rooms: [{id, cx, cy, area, peak}]  cx/cy 는 도면 좌표[m](왼아래 원점),
             정중앙 = 방 안 여유거리 최대점. id 는 아래->위, 왼->오른쪽 순서로
             1부터 다시 매긴 번호(실행마다 같은 방이 같은 번호를 받게).
      label: int32 (H,W) 방 라벨(rooms 의 내부 라벨과는 다를 수 있으니
             rooms[i]['raw'] 로 대조)
      dist:  여유거리[m]

    거르는 것
      * area < min_area          — 너무 작은 조각(문턱 주변 부스러기)
      * peak < min_clear         — 차체 여유보다 좁아 들어갈 수 없는 곳
      * 중심이 도면 가장자리에서 border_margin 안 — 도면 밖 여백
        (2d_ex 처럼 건물 바깥까지 흰색인 도면에서 바깥 띠가 방으로 잡힌다)
    """
    free = (~occ) & (~unk)
    dist = cv2.distanceTransform(free.astype(np.uint8),
                                 cv2.DIST_L2, 5) * res
    peaks = h_maxima_markers(dist, h_door)
    label, n = priority_flood(dist, peaks, free)
    hh, ww = occ.shape
    max_x, max_y = ww * res, hh * res
    rooms = []
    for i in range(1, n + 1):
        m = label == i
        area = float(m.sum()) * res * res
        if area <= 0.0:
            continue
        d_in = np.where(m, dist, 0.0)
        r, c = np.unravel_index(int(np.argmax(d_in)), d_in.shape)
        peak = float(dist[r, c])
        cx, cy = (c + 0.5) * res, (r + 0.5) * res
        if area < min_area or peak < min_clear:
            continue
        if (cx < border_margin or cy < border_margin
                or cx > max_x - border_margin or cy > max_y - border_margin):
            continue
        rooms.append(dict(raw=i, cx=cx, cy=cy, area=area, peak=peak))
    rooms.sort(key=lambda rm: (round(rm["cy"], 1), rm["cx"]))
    for k, rm in enumerate(rooms):
        rm["id"] = k + 1
    return rooms, label, dist


# ================= 방문 순서용 축소 격자 =================

def coarse_safe(dist, res, inflate, cell=0.04):
    """차체 여유(inflate)를 준 통행가능 격자를 cell 크기로 줄인다.

    블록 안에 통행가능 셀이 하나라도 있으면 통행가능(낙관적)으로 본다 —
    보수적으로 줄이면 좁은 통로가 끊겨 '갈 수 있는 방'이 못 가는 것으로
    잘못 걸러진다. 순서 계산용이라 낙관적이어도 안전하다(실제 주행은
    플래너의 원해상도 A* 가 판단한다).
    """
    safe = dist >= inflate
    f = max(1, int(round(cell / res)))
    if f == 1:
        return safe, 1
    h, w = safe.shape
    ph, pw = -(-h // f), -(-w // f)
    padded = np.zeros((ph * f, pw * f), dtype=bool)
    padded[:h, :w] = safe
    return padded.reshape(ph, f, pw, f).any(axis=(1, 3)), f


def bfs_dist(safe, start):
    """축소 격자에서 start 로부터의 이동거리(셀 수). 못 가는 곳은 -1."""
    h, w = safe.shape
    out = np.full((h, w), -1, dtype=np.int32)
    r0, c0 = start
    if not (0 <= r0 < h and 0 <= c0 < w) or not safe[r0, c0]:
        return out
    out[r0, c0] = 0
    q = deque([(r0, c0)])
    while q:
        r, c = q.popleft()
        d = out[r, c] + 1
        for nr, nc in ((r - 1, c), (r + 1, c), (r, c - 1), (r, c + 1),
                       (r - 1, c - 1), (r - 1, c + 1),
                       (r + 1, c - 1), (r + 1, c + 1)):
            if 0 <= nr < h and 0 <= nc < w and safe[nr, nc] \
                    and out[nr, nc] < 0:
                out[nr, nc] = d
                q.append((nr, nc))
    return out


def snap_cell(safe, cell, max_r=8):
    """막힌 셀이면 주변에서 가장 가까운 통행가능 셀을 찾는다. 없으면 None."""
    h, w = safe.shape
    r0, c0 = cell
    r0 = min(max(r0, 0), h - 1)
    c0 = min(max(c0, 0), w - 1)
    if safe[r0, c0]:
        return (r0, c0)
    best, bestd = None, 1e18
    for dr in range(-max_r, max_r + 1):
        for dc in range(-max_r, max_r + 1):
            r, c = r0 + dr, c0 + dc
            if 0 <= r < h and 0 <= c < w and safe[r, c]:
                d = dr * dr + dc * dc
                if d < bestd:
                    bestd, best = d, (r, c)
    return best


def ascii_map(occ, label, rooms, width=78):
    """방 배치를 한눈에 보는 ASCII 렌더(로그/미리보기용)."""
    h, w = occ.shape
    f = max(1, int(math.ceil(w / float(width))))
    ph, pw = h // f, w // f
    ch = {}
    for i, rm in enumerate(sorted(rooms, key=lambda r: r["id"])):
        ch[rm["raw"]] = chr(ord('a') + i % 26)
    lines = []
    for rr in range(ph - 1, -1, -1):
        line = []
        for cc in range(pw):
            ob = occ[rr * f:(rr + 1) * f, cc * f:(cc + 1) * f]
            if ob.any():
                line.append("#")
                continue
            blk = label[rr * f:(rr + 1) * f, cc * f:(cc + 1) * f]
            v = int(blk.max())
            line.append(ch.get(v, " "))
        lines.append("|" + "".join(line) + "|")
    return lines
