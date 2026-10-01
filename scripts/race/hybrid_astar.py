#!/usr/bin/env python3
"""막힘 탈출용 Hybrid A* (Ackermann 전진/후진 모션 프리미티브)

plan(start, goal, ok_fn, ...) -> [(x, y, yaw, dir), ...] 또는 None
  ok_fn(P(N,2), H(N)) -> bool 배열 (차체 충돌 없음)
"""
import math
import heapq
import time
import numpy as np
from skimage.graph import MCP_Geometric

STEP = 0.2                       # 프리미티브 길이(m)
K_PRIM = 1.0 / 0.6               # 프리미티브 최대 곡률(조향 한계보다 약간 여유)
CURVS = (-K_PRIM, -K_PRIM / 2, 0.0, K_PRIM / 2, K_PRIM)
XY_RES = 0.1
TH_BINS = 48
REV_COST = 2.0
SWITCH_COST = 1.0
STEER_COST = 0.1


def heuristic_grid(goal, ok_point_fn, origin, n, res):
    """장애물 고려한 2D 거리(목표점 기준) 격자"""
    ys, xs = np.mgrid[0:n, 0:n]
    P = np.stack([origin[0] + (xs.ravel() + 0.5) * res, origin[1] + (ys.ravel() + 0.5) * res], 1)
    free = ok_point_fn(P).reshape(n, n)
    cost = np.where(free, 1.0, np.inf)
    gi = int((goal[1] - origin[1]) / res), int((goal[0] - origin[0]) / res)
    if not (0 <= gi[0] < n and 0 <= gi[1] < n):
        return None
    cost[gi] = 1.0
    mcp = MCP_Geometric(cost, fully_connected=True)
    acc, _ = mcp.find_costs([gi])
    return acc * res


def plan(start, goal, ok_fn, ok_point_fn, half=4.0, max_time=1.5, goal_tol=0.25, goal_th=0.5):
    t0 = time.time()
    res_h = 0.1
    n = int(2 * half / res_h)
    cx, cy = (start[0] + goal[0]) / 2, (start[1] + goal[1]) / 2
    origin = (cx - half, cy - half)
    hg = heuristic_grid(goal, ok_point_fn, origin, n, res_h)
    if hg is None:
        return None

    def h(x, y):
        i, j = int((y - origin[1]) / res_h), int((x - origin[0]) / res_h)
        if 0 <= i < n and 0 <= j < n and np.isfinite(hg[i, j]):
            return hg[i, j]
        return 1e6

    def key(x, y, th):
        return (int(math.floor(x / XY_RES)), int(math.floor(y / XY_RES)),
                int(round((th % (2 * math.pi)) / (2 * math.pi) * TH_BINS)) % TH_BINS)

    # 프리미티브 미리 계산(로컬 좌표, 3개 하위점)
    prims = []
    for d in (1, -1):
        for k in CURVS:
            sub = []
            for f in (1 / 3, 2 / 3, 1.0):
                s = d * STEP * f
                if abs(k) < 1e-6:
                    sub.append((s, 0.0, 0.0))
                else:
                    sub.append((math.sin(k * s) / k, (1 - math.cos(k * s)) / k, k * s))
            prims.append((d, k, sub))

    sx, sy, sth = start
    openq = [(h(sx, sy), 0.0, sx, sy, sth, 0, 0.0, None)]
    nodes = {}
    closed = set()
    cnt = 0
    while openq and time.time() - t0 < max_time:
        f, g, x, y, th, d0, k0, parent = heapq.heappop(openq)
        kk = key(x, y, th)
        if kk in closed:
            continue
        closed.add(kk)
        nodes[kk] = (x, y, th, d0, parent)
        cnt += 1
        dth = math.atan2(math.sin(th - goal[2]), math.cos(th - goal[2]))
        if math.hypot(x - goal[0], y - goal[1]) < goal_tol and abs(dth) < goal_th:
            path = []
            q = kk
            while q is not None:
                px, py, pth, pd, pp = nodes[q]
                path.append((px, py, pth, pd))
                q = pp
            return path[::-1]
        c, s_ = math.cos(th), math.sin(th)
        cand = []
        for d, k, sub in prims:
            pts = [(x + c * a - s_ * b, y + s_ * a + c * b, th + t) for a, b, t in sub]
            cand.append((d, k, pts))
        P = np.array([[p[0], p[1]] for _, _, pts in cand for p in pts])
        H = np.array([p[2] for _, _, pts in cand for p in pts])
        ok = ok_fn(P, H).reshape(len(cand), 3).all(1)
        for (d, k, pts), good in zip(cand, ok):
            if not good:
                continue
            nx, ny, nth = pts[-1]
            if key(nx, ny, nth) in closed:
                continue
            g2 = g + STEP * (1.0 if d > 0 else REV_COST) + (SWITCH_COST if (d0 and d != d0) else 0.0) \
                + STEER_COST * abs(k - k0)
            heapq.heappush(openq, (g2 + h(nx, ny), g2, nx, ny, nth, d, k, kk))
    return None


def plan_loop(clear_fn, start, goal, h_fn, curvs, step=0.2, cost_fn=None, max_time=30.0,
              goal_tol=0.3, goal_th=0.4, xy_res=0.1, th_bins=72, blocked_fn=None, h_w=1.8):
    """전진 전용 Hybrid A* (레이스라인용). clear_fn(P,H)->bool, h_fn(x,y)->남은거리, cost_fn(P,H)->추가비용"""
    t0 = time.time()

    def key(x, y, th):
        return (int(math.floor(x / xy_res)), int(math.floor(y / xy_res)),
                int(round((th % (2 * math.pi)) / (2 * math.pi) * th_bins)) % th_bins)
    prims = []
    for k in curvs:
        sub = []
        for f in (0.25, 0.5, 0.75, 1.0):
            s = step * f
            if abs(k) < 1e-6:
                sub.append((s, 0.0, 0.0))
            else:
                sub.append((math.sin(k * s) / k, (1 - math.cos(k * s)) / k, k * s))
        prims.append((k, sub))
    sx, sy, sth = start
    openq = [(h_w * h_fn(sx, sy), 0.0, sx, sy, sth, 0.0, None)]
    nodes = {}
    closed = set()
    nexp = 0
    while openq and time.time() - t0 < max_time:
        f, g, x, y, th, k0, parent = heapq.heappop(openq)
        kk = key(x, y, th)
        if kk in closed:
            continue
        closed.add(kk)
        nodes[kk] = (x, y, th, parent)
        nexp += 1
        dth = math.atan2(math.sin(th - goal[2]), math.cos(th - goal[2]))
        if nexp > 20 and math.hypot(x - goal[0], y - goal[1]) < goal_tol and abs(dth) < goal_th:
            path = []
            q = kk
            while q is not None:
                px, py, pth, pp = nodes[q]
                path.append((px, py, pth))
                q = pp
            return path[::-1], nexp
        c, s_ = math.cos(th), math.sin(th)
        P = np.array([[x + c * a - s_ * b, y + s_ * a + c * b] for _, sub in prims for a, b, _ in sub])
        H = np.array([th + t for _, sub in prims for _, _, t in sub])
        ok = clear_fn(P, H).reshape(len(prims), 4).all(1)
        if blocked_fn is not None:
            ok &= ~blocked_fn(P).reshape(len(prims), 4).any(1)
        extra = cost_fn(P, H).reshape(len(prims), 4).mean(1) if cost_fn is not None else np.zeros(len(prims))
        for (k, sub), good, ex, i in zip(prims, ok, extra, range(len(prims))):
            if not good:
                continue
            nx, ny = P[i * 4 + 3]
            nth = th + sub[-1][2]
            if key(nx, ny, nth) in closed:
                continue
            g2 = g + step * (1.0 + ex) + 0.02 * abs(k - k0)
            heapq.heappush(openq, (g2 + h_w * h_fn(nx, ny), g2, nx, ny, nth, k, kk))
    return None, nexp
