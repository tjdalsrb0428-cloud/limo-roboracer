#!/usr/bin/env python3
"""맵(.yaml)만으로 폐루프 레이스라인 자동 생성 (수동 클릭 불필요)

사용: python3 scripts/race/raceline.py MAP.yaml OUT.csv [X Y YAW] [--clear 0.30] [--kmax 1.3]
  X Y YAW : 출발 pose(map 좌표). 진행 방향 = YAW 방향. 생략 시 0 0 0
결과: OUT.csv (x,y), OUT.png (오버레이: 파랑=추출 루프, 빨강=레이스라인)

1) 출발점에서 진행방향 수직선으로 트랙을 자르고, 벽거리 비용 최소경로로 루프 추출
2) 벽거리 >= clear(불가능한 좁은 곳은 중앙), 곡률 <= kmax 조건에서 길이 최소화(인코스)
"""
import os
import math
import time
import argparse
import numpy as np
import yaml
from PIL import Image, ImageDraw
from scipy import ndimage
from scipy.interpolate import CubicSpline
from skimage.graph import MCP_Geometric


class GridMap:
    def __init__(self, yml):
        y = yaml.safe_load(open(yml))
        p = y['image']
        if not os.path.isabs(p):
            p = os.path.join(os.path.dirname(os.path.abspath(yml)), p)
        img = np.array(Image.open(p).convert('L')).astype(np.float32)
        if int(y.get('negate', 0)):
            img = 255 - img
        self.img = img.astype(np.uint8)
        self.res = float(y['resolution'])
        self.ox, self.oy = float(y['origin'][0]), float(y['origin'][1])
        self.H, self.W = img.shape
        occ_p = (255 - img) / 255.0
        self.occ = occ_p > float(y.get('occupied_thresh', 0.65))
        free = occ_p < float(y.get('free_thresh', 0.25))
        # 작은 unknown 구멍(매핑 잡음)은 free로 메움, 벽(occupied)은 유지
        free = ndimage.binary_closing(free, iterations=2) & ~self.occ
        self.free = free
        self.dist = ndimage.distance_transform_edt(free) * self.res
        dsm = ndimage.gaussian_filter(self.dist, 1.0)
        self.gr, self.gc = np.gradient(dsm)

    def rc(self, P):
        P = np.atleast_2d(P)
        return self.H - 0.5 - (P[:, 1] - self.oy) / self.res, (P[:, 0] - self.ox) / self.res - 0.5

    def xy(self, r, c):
        return np.stack([(np.asarray(c) + 0.5) * self.res + self.ox,
                         (self.H - 0.5 - np.asarray(r)) * self.res + self.oy], -1)

    def samp(self, A, P, order=1):
        r, c = self.rc(P)
        return ndimage.map_coordinates(A, [r, c], order=order, mode='constant', cval=0.0)

    def clear(self, P):
        return self.samp(self.dist, P)

    def grad(self, P):
        g = np.stack([self.samp(self.gc, P), -self.samp(self.gr, P)], 1)
        return g / np.maximum(np.hypot(g[:, 0], g[:, 1]), 1e-9)[:, None]


def resample(P, step=None, n=None):
    Q = np.vstack([P, P[:1]])
    seg = np.hypot(*np.diff(Q, axis=0).T)
    keep = np.concatenate([[True], seg > 1e-6])
    Q = Q[keep]
    s = np.concatenate([[0], np.cumsum(np.hypot(*np.diff(Q, axis=0).T))])
    L = s[-1]
    if n is None:
        n = max(8, int(round(L / step)))
    Q[-1] = Q[0]
    cs = CubicSpline(s, Q, bc_type='periodic')
    return cs(np.linspace(0, L, n, endpoint=False)), L


def curvature(P):
    d1 = (np.roll(P, -1, 0) - np.roll(P, 1, 0)) / 2
    d2 = np.roll(P, -1, 0) - 2 * P + np.roll(P, 1, 0)
    num = d1[:, 0] * d2[:, 1] - d1[:, 1] * d2[:, 0]
    return num / np.maximum(np.hypot(d1[:, 0], d1[:, 1]) ** 3, 1e-9)


def extract_loop(m, x0, y0, yaw0, c_min):
    """출발선 절단 + 최소비용 경로로 트랙 한 바퀴 추출 (진행방향 순서)"""
    t = np.array([math.cos(yaw0), math.sin(yaw0)])
    nv = np.array([-t[1], t[0]])
    p0 = np.array([x0, y0])
    if m.clear(p0[None])[0] < 0.05:
        raise RuntimeError('출발점이 free 영역이 아님')
    passable = m.dist >= c_min
    cost = np.where(passable, 1.0 + 0.5 / np.maximum(m.dist, 0.05), np.inf)
    cut = []
    for sg in (1, -1):
        for l in np.arange(0, 6.0, m.res / 2):
            q = p0 + sg * nv * l
            if m.clear(q[None])[0] <= 0.0:
                break
            cut.append(q)
    r, c = m.rc(np.array(cut))
    r = np.round(r).astype(int)
    c = np.round(c).astype(int)
    for dr in (-1, 0, 1):
        for dc in (-1, 0, 1):
            cost[np.clip(r + dr, 0, m.H - 1), np.clip(c + dc, 0, m.W - 1)] = np.inf

    def seed(sg):
        for l in np.arange(0.15, 1.5, m.res):
            for lat in (0.0, 0.1, -0.1, 0.2, -0.2, 0.3, -0.3):
                q = p0 + sg * t * l + nv * lat
                rq, cq = m.rc(q[None])
                rq, cq = int(round(rq[0])), int(round(cq[0]))
                if np.isfinite(cost[rq, cq]):
                    return rq, cq
        raise RuntimeError('출발점 주변 통과 가능 셀 없음')
    s_front = seed(1)
    s_back = seed(-1)
    mcp = MCP_Geometric(cost, fully_connected=True)
    acc, _ = mcp.find_costs([s_front], [s_back])
    if not np.isfinite(acc[s_back]):
        raise RuntimeError('루프가 닫히지 않음')
    path = np.array(mcp.traceback(s_back))
    return m.xy(path[:, 0], path[:, 1])


def normals(P):
    t = np.roll(P, -1, 0) - np.roll(P, 1, 0)
    t /= np.maximum(np.hypot(t[:, 0], t[:, 1]), 1e-9)[:, None]
    return np.stack([-t[:, 1], t[:, 0]], 1)


def bounds(m, P, N, clear, lmax=1.5, step=0.02):
    """법선 방향 오프셋 a 중 clearance>=clear 인 구간 (0 포함 구간, 없으면 가장 넓은 쪽 근처)"""
    ls = np.arange(-lmax, lmax + 1e-9, step)
    Q = P[:, None, :] + N[:, None, :] * ls[None, :, None]
    c = m.clear(Q.reshape(-1, 2)).reshape(len(P), len(ls))
    i0 = len(ls) // 2
    lb = np.zeros(len(P)); ub = np.zeros(len(P))
    for i in range(len(P)):
        ok = c[i] >= clear
        if not ok[i0]:
            # 기준점이 조건 위반: clearance 최대 지점 쪽으로 (좁은 틈이면 중앙)
            j = int(np.argmax(np.where(np.abs(ls) < 0.5, c[i], -1)))
            if not ok[j]:
                lb[i] = ub[i] = ls[j]
                continue
            i_c = j
        else:
            i_c = i0
        lo = i_c
        while lo > 0 and ok[lo - 1]:
            lo -= 1
        hi = i_c
        while hi < len(ls) - 1 and ok[hi + 1]:
            hi += 1
        lb[i], ub[i] = ls[lo], ls[hi]
    return lb, ub


def taut(m, P0, clear, ds=0.1):
    """벽거리 clear 를 지키는 팽팽한(최단) 루프 - 곡률 무시"""
    Q, _ = resample(ndimage.gaussian_filter1d(P0, 4, axis=0, mode='wrap'), step=0.4)
    for dsl, iters in ((0.4, 800), (0.2, 800), (ds, 600)):
        Q, _ = resample(Q, step=dsl)
        for it in range(iters):
            lap = np.roll(Q, 1, 0) + np.roll(Q, -1, 0) - 2 * Q
            Q = Q + 0.3 * lap
            for _ in range(2):
                d = m.clear(Q)
                bad = d < clear
                if not bad.any():
                    break
                Q[bad] += np.minimum(clear - d[bad], 0.04)[:, None] * m.grad(Q[bad])
            if it % 40 == 39:
                Q, _ = resample(Q, n=len(Q))
    return resample(Q, step=ds)[0]


def solve_offsets(Rf, N, lb, ub, wc):
    """min sum|p_i+1 - p_i|^2 + sum wc_i |p_i+1 - 2p_i + p_i-1|^2, p_i = r_i + a_i n_i, lb<=a<=ub"""
    from scipy.sparse import coo_matrix
    from scipy.optimize import minimize
    n = len(Rf)
    I = np.arange(n)
    rows, cols, vals, b = [], [], [], []
    r0 = 0
    for k in range(2):           # 1차 차분 (길이)
        rows += [r0 + I, r0 + I]
        cols += [(I + 1) % n, I]
        vals += [N[(I + 1) % n, k], -N[I, k]]
        b.append(-(Rf[(I + 1) % n, k] - Rf[I, k]))
        r0 += n
    sw = np.sqrt(wc)
    for k in range(2):           # 2차 차분 (곡률)
        rows += [r0 + I, r0 + I, r0 + I]
        cols += [(I + 1) % n, I, (I - 1) % n]
        vals += [sw * N[(I + 1) % n, k], -2 * sw * N[I, k], sw * N[(I - 1) % n, k]]
        b.append(-sw * (Rf[(I + 1) % n, k] - 2 * Rf[I, k] + Rf[(I - 1) % n, k]))
        r0 += n
    A = coo_matrix((np.concatenate(vals), (np.concatenate(rows), np.concatenate(cols))),
                   shape=(r0, n)).tocsr()
    b = np.concatenate(b)
    At = A.T.tocsr()

    def f(a):
        e = A @ a - b
        return float(e @ e), 2 * (At @ e)
    a0 = np.clip(np.zeros(n), lb, ub)
    res = minimize(f, a0, jac=True, method='L-BFGS-B', bounds=list(zip(lb, ub)),
                   options={'maxiter': 5000, 'ftol': 1e-12, 'gtol': 1e-8})
    return res.x


def optimize(m, P0, clear, kmax, verbose=True, ds=0.1):
    T = taut(m, P0, clear, ds)
    Lt = float(np.sum(np.hypot(*np.diff(np.vstack([T, T[:1]]), axis=0).T)))
    C, _ = resample(ndimage.gaussian_filter1d(T, 0.5 / ds, axis=0, mode='wrap'), step=ds)
    N = normals(ndimage.gaussian_filter1d(C, 0.3 / ds, axis=0, mode='wrap'))
    lb, ub = bounds(m, C, N, clear)
    eps = 1e-3
    ub = np.maximum(ub, lb + eps)
    if verbose:
        print(f'  최단선(곡률 무시) {Lt:.2f}m, 기준선 {len(C)}점')
    wc = np.full(len(C), (0.3 / ds) ** 2)
    for rep in range(25):
        a = solve_offsets(C, N, lb, ub, wc)
        Q = C + N * a[:, None]
        k = ndimage.uniform_filter1d(np.abs(curvature(Q)), 3, mode='wrap')
        bad = k > kmax * 0.92
        if not bad.any():
            break
        grow = ndimage.binary_dilation(bad, iterations=int(0.6 / ds))
        wc[grow] *= 1.8
    Q2, L = resample(Q, step=0.05)
    k2 = ndimage.uniform_filter1d(np.abs(curvature(Q2)), 3, mode='wrap')
    dmin = float(m.clear(Q2).min())
    if verbose:
        print(f'  곡률제한 결과: 길이 {L:6.2f}m  최대곡률 {k2.max():.2f}  최소벽거리 {dmin:.2f}m  (재가중 {rep})')
    return Q2, L, float(k2.max()), dmin


def make(yml, x0=0.0, y0=0.0, yaw0=0.0, clear=0.30, kmax=1.3, out=None, verbose=True):
    t0 = time.time()
    m = GridMap(yml)
    P = None
    for c_min in (0.25, 0.20, 0.16, 0.13):
        try:
            P = extract_loop(m, x0, y0, yaw0, c_min)
            break
        except RuntimeError as e:
            if verbose:
                print(f'  루프 추출 c_min={c_min}: {e}')
    if P is None:
        raise RuntimeError('루프 추출 실패')
    Lc = np.sum(np.hypot(*np.diff(P, axis=0).T))
    if verbose:
        print(f'루프 추출: 길이 {Lc:.2f}m (c_min={c_min})  {time.time() - t0:.1f}s')
    R, L, kx, dmin = optimize(m, P, clear, kmax, verbose=verbose)
    i0 = int(np.argmin(np.hypot(R[:, 0] - x0, R[:, 1] - y0)))
    R = np.roll(R, -i0, 0)
    if verbose:
        print(f'레이스라인: 길이 {L:.2f}m, 최대곡률 {kx:.2f}(한도 {kmax}), 최소벽거리 {dmin:.2f}m, '
              f'점 {len(R)}개  총 {time.time() - t0:.1f}s')
    if out:
        np.savetxt(out, R, delimiter=',', fmt='%.4f', header='x,y', comments='')
        overlay(m, [(P, (0, 0, 255)), (R, (255, 0, 0))], os.path.splitext(out)[0] + '.png',
                start=(x0, y0, yaw0))
    return R, m


def overlay(m, paths, png, start=None, S=3):
    rgb = Image.fromarray(np.stack([m.img] * 3, 2)).resize((m.W * S, m.H * S), Image.NEAREST)
    dr = ImageDraw.Draw(rgb)
    for A, col in paths:
        r, c = m.rc(np.vstack([A, A[:1]]))
        dr.line([(float(cc) * S + S / 2, float(rr) * S + S / 2) for rr, cc in zip(r, c)], fill=col, width=2)
    if start is not None:
        x, y, yaw = start
        r, c = m.rc(np.array([[x, y], [x + 0.6 * math.cos(yaw), y + 0.6 * math.sin(yaw)]]))
        pts = [(float(cc) * S, float(rr) * S) for rr, cc in zip(r, c)]
        dr.line(pts, fill=(0, 170, 0), width=4)
        dr.ellipse([pts[0][0] - 6, pts[0][1] - 6, pts[0][0] + 6, pts[0][1] + 6], outline=(0, 170, 0), width=3)
    rgb.save(png)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('map')
    ap.add_argument('out')
    ap.add_argument('pose', nargs='*', type=float)
    ap.add_argument('--clear', type=float, default=0.30)
    ap.add_argument("--kmax", type=float, default=1.25)
    a = ap.parse_args()
    x0, y0, yaw0 = (list(a.pose) + [0.0, 0.0, 0.0])[:3]
    make(a.map, x0, y0, yaw0, a.clear, a.kmax, a.out)


if __name__ == '__main__':
    main()
