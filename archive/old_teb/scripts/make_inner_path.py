#!/usr/bin/env python3
"""spline_path.csv -> 벽 여유거리를 지키는 최단(인코스) 경로 csv
사용: python3 scripts/make_inner_path.py maps/my_map.yaml maps/spline_path.csv maps/spline_path_inner.csv [clear=0.30] [N=원본점수]
"""
import sys, os
import numpy as np
import yaml
from PIL import Image, ImageDraw
from scipy import ndimage
from scipy.interpolate import CubicSpline

KMAX = 1.861
STEP = 0.05
NB_DIST = 0.40
ALPHA = 0.3
ITERS = 4000
CL_MAX = 0.50
CL_STEP = 0.04
CL_HARD = 0.20
SIGMAS = (3, 5, 8, 12, 16, 22)


def load_pts(p):
    rows = []
    for line in open(p):
        t = line.strip().split(',')
        try:
            rows.append([float(t[0]), float(t[1])])
        except (ValueError, IndexError):
            continue
    P = np.array(rows)
    if np.hypot(*(P[-1] - P[0])) < 0.03:
        P = P[:-1]
    return P


def resample(P, step=None, n=None):
    Q = np.vstack([P, P[:1]])
    s = np.concatenate([[0], np.cumsum(np.hypot(*np.diff(Q, axis=0).T))])
    L = s[-1]
    if n is None:
        n = int(round(L / step))
    cs = CubicSpline(s, Q, bc_type='periodic')
    return cs(np.linspace(0, L, n, endpoint=False)), L


def curvature(P):
    d1 = (np.roll(P, -1, 0) - np.roll(P, 1, 0)) / 2
    d2 = np.roll(P, -1, 0) - 2 * P + np.roll(P, 1, 0)
    num = np.abs(d1[:, 0] * d2[:, 1] - d1[:, 1] * d2[:, 0])
    return num / np.maximum(np.hypot(d1[:, 0], d1[:, 1]) ** 3, 1e-9)


def sm(P, sigma=3):
    return ndimage.gaussian_filter1d(P, sigma, axis=0, mode='wrap')


def main():
    if len(sys.argv) < 4:
        print(__doc__)
        sys.exit(1)
    yml, src, dst = sys.argv[1:4]
    clear0 = float(sys.argv[4]) if len(sys.argv) > 4 else 0.30

    y = yaml.safe_load(open(yml))
    img_path = y['image']
    if not os.path.isabs(img_path):
        img_path = os.path.join(os.path.dirname(os.path.abspath(yml)), img_path)
    img = np.array(Image.open(img_path).convert('L'))
    if int(y.get('negate', 0)):
        img = 255 - img
    res = float(y['resolution'])
    ox, oy = float(y['origin'][0]), float(y['origin'][1])
    H = img.shape[0]

    free = img >= 250
    dist = ndimage.distance_transform_edt(free) * res
    dsm = ndimage.gaussian_filter(dist, 1.0)
    gr, gc = np.gradient(dsm)

    def to_rc(P):
        return H - 0.5 - (P[:, 1] - oy) / res, (P[:, 0] - ox) / res - 0.5

    def samp(A, P):
        r, c = to_rc(P)
        return ndimage.map_coordinates(A, [r, c], order=1, mode='nearest')

    def push(P, clear):
        for _ in range(3):
            d = samp(dist, P)
            g = np.stack([samp(gc, P), -samp(gr, P)], 1)
            g /= np.maximum(np.hypot(g[:, 0], g[:, 1]), 1e-9)[:, None]
            m = d < clear
            P[m] += (clear - d[m])[:, None] * g[m]
        return P

    orig = load_pts(src)
    n_out = int(sys.argv[5]) if len(sys.argv) > 5 else len(orig)
    P0, L0 = resample(orig, step=STEP)
    print(f'원본: 길이 {L0:.2f}m, 최소 벽거리 {samp(dist, P0).min():.2f}m, 최대곡률 {curvature(sm(P0)).max():.2f}')
    print(f'맵: free 픽셀 {int(free.sum())}개')

    k = max(1, int(NB_DIST / STEP))
    clear = clear0
    best = None
    done = False
    while not done:
        P = P0.copy()
        for _ in range(ITERS):
            Q = 0.5 * (np.roll(P, k, 0) + np.roll(P, -k, 0))
            P = push(P + ALPHA * (Q - P), clear)
        for sg in SIGMAS:
            Pf, L1 = resample(sm(P, sg), n=n_out)
            kap = curvature(Pf)
            nbad = int((kap > KMAX).sum())
            dmin = float(samp(dist, Pf).min())
            print(f'  clear={clear:.2f} smooth={sg * STEP:.2f}m -> 길이 {L1:.2f}m, 최대곡률 {kap.max():.2f}, 초과 {nbad}개, 최소벽거리 {dmin:.2f}m')
            if dmin < CL_HARD:
                break
            cand = (nbad, float(kap.max()), clear, sg, Pf, L1)
            if best is None or cand[:2] < best[:2]:
                best = cand
            if nbad == 0:
                done = True
                break
        if not done:
            if clear + CL_STEP > CL_MAX + 1e-9:
                break
            clear += CL_STEP
    if best is None:
        print('경로를 만들지 못함: 벽거리 조건 불만족. 맵/clear 확인 필요')
        sys.exit(2)
    _, _, clear, sg, Pf, L1 = best

    i0 = int(np.argmin(np.hypot(Pf[:, 0] - orig[0, 0], Pf[:, 1] - orig[0, 1])))
    Pf = np.roll(Pf, -i0, 0)
    kap = curvature(Pf)
    dw = samp(dist, Pf)
    bad = np.where(kap > KMAX)[0]
    print(f'결과: clear={clear:.2f}m, 길이 {L1:.2f}m ({L1 - L0:+.2f}m), 점 {len(Pf)}개')
    print(f'      최소 벽거리 {dw.min():.2f}m, 최대곡률 {kap.max():.2f} (허용 {KMAX}), 곡률 초과점 {len(bad)}개')
    for i in bad[:8]:
        print(f'      초과 idx={i} pos=({Pf[i, 0]:.2f},{Pf[i, 1]:.2f}) k={kap[i]:.2f}')
    if len(bad):
        print('      -> 한도를 못 맞춤: 오버레이 확인 필요')

    np.savetxt(dst, Pf, delimiter=',', fmt='%.4f')
    print(f'저장: {dst}')

    S = 3
    rgb = Image.fromarray(np.stack([img] * 3, 2)).resize((img.shape[1] * S, H * S), Image.NEAREST)
    dr = ImageDraw.Draw(rgb)
    for A, col in ((orig, (0, 0, 255)), (Pf, (255, 0, 0))):
        r, c = to_rc(np.vstack([A, A[:1]]))
        dr.line([(float(cc) * S + S / 2, float(rr) * S + S / 2) for rr, cc in zip(r, c)], fill=col, width=2)
    out_png = os.path.join(os.path.dirname(os.path.abspath(dst)), 'inner_overlay.png')
    rgb.save(out_png)
    print(f'오버레이: {out_png}')


if __name__ == '__main__':
    main()
