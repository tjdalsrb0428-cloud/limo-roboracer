#!/usr/bin/env python3
import sys
import numpy as np
from PIL import Image
import yaml
import networkx as nx
from skimage.morphology import skeletonize
from scipy.signal import savgol_filter


def load_map(pgm_path, yaml_path):
    raw = np.array(Image.open(pgm_path).transpose(Image.FLIP_TOP_BOTTOM))
    with open(yaml_path) as f:
        meta = yaml.safe_load(f)
    return raw, meta['resolution'], meta['origin']


def to_binary_free(raw, occ_thresh=200):
    return (raw >= occ_thresh)


def skeleton_to_graph(skel):
    G = nx.Graph()
    ys, xs = np.nonzero(skel)
    pts = set(zip(ys.tolist(), xs.tolist()))
    for y, x in pts:
        G.add_node((y, x))
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dy == 0 and dx == 0:
                    continue
                n = (y + dy, x + dx)
                if n in pts:
                    G.add_edge((y, x), n)
    return G


def prune_spurs(G, min_spur_len=15):
    changed = True
    while changed:
        changed = False
        leaves = [n for n in G.nodes if G.degree(n) == 1]
        for leaf in leaves:
            if leaf not in G:
                continue
            path = [leaf]
            cur = leaf
            prev = None
            while True:
                neighbors = [n for n in G.neighbors(cur) if n != prev]
                if len(neighbors) != 1:
                    break
                prev, cur = cur, neighbors[0]
                path.append(cur)
                if G.degree(cur) != 2:
                    break
            if len(path) < min_spur_len and G.degree(path[-1]) > 2:
                G.remove_nodes_from(path[:-1])
                changed = True
    return G


def largest_cycle_order(G):
    comp = max(nx.connected_components(G), key=len)
    sub = G.subgraph(comp).copy()

    while True:
        branch_nodes = [n for n in sub.nodes if sub.degree(n) > 2]
        if not branch_nodes:
            break
        n = branch_nodes[0]
        neighbors = list(sub.neighbors(n))
        best = None
        for nb in neighbors:
            length = 1
            prev, cur = n, nb
            while sub.degree(cur) == 2:
                nxt = [x for x in sub.neighbors(cur) if x != prev]
                if not nxt:
                    break
                prev, cur = cur, nxt[0]
                length += 1
            if best is None or length < best[0]:
                best = (length, nb)
        sub.remove_edge(n, best[1])
        sub = sub.subgraph(max(nx.connected_components(sub), key=len)).copy()

    start = next(iter(sub.nodes))
    order = [start]
    prev, cur = None, start
    while True:
        neighbors = [n for n in sub.neighbors(cur) if n != prev]
        if not neighbors:
            break
        prev, cur = cur, neighbors[0]
        if cur == start:
            break
        order.append(cur)
    return order


def order_to_world(order, resolution, origin):
    pts = np.array(order, dtype=float)
    world_x = pts[:, 1] * resolution + origin[0]
    world_y = pts[:, 0] * resolution + origin[1]
    return np.column_stack([world_x, world_y])


def resample_equal_arclength(pts, n_out):
    closed = np.vstack([pts, pts[0]])
    diffs = np.diff(closed, axis=0)
    seglen = np.sqrt((diffs ** 2).sum(axis=1))
    seglen[seglen == 0] = 1e-9
    t = np.concatenate([[0], np.cumsum(seglen)])
    total = t[-1]
    t_new = np.linspace(0, total, n_out, endpoint=False)
    x_new = np.interp(t_new, t, closed[:, 0])
    y_new = np.interp(t_new, t, closed[:, 1])
    return np.column_stack([x_new, y_new]), total


def curvature_of(x, y):
    dx = np.gradient(x)
    dy = np.gradient(y)
    ddx = np.gradient(dx)
    ddy = np.gradient(dy)
    denom = (dx ** 2 + dy ** 2) ** 1.5
    denom[denom < 1e-6] = 1e-6
    return np.abs(dx * ddy - dy * ddx) / denom


def smooth_until_ok(pts, max_curvature, max_window=101):
    x, y = pts[:, 0].copy(), pts[:, 1].copy()
    n = len(x)
    window = 7
    xs, ys, curv = x, y, curvature_of(x, y)
    while window < max_window:
        w = min(window, n - (1 - n % 2))
        if w % 2 == 0:
            w -= 1
        if w < 5:
            window += 4
            continue
        pad = w
        xp = np.concatenate([x[-pad:], x, x[:pad]])
        yp = np.concatenate([y[-pad:], y, y[:pad]])
        xs = savgol_filter(xp, w, 3)
        ys = savgol_filter(yp, w, 3)
        xs = xs[pad:-pad]
        ys = ys[pad:-pad]

        curv = curvature_of(xs, ys)
        n_exceed = np.sum(curv > max_curvature)
        print(f"  window={w}: 초과점 {n_exceed}/{n}, 최대곡률={curv.max():.3f}")
        if n_exceed == 0:
            return xs, ys, curv
        window += 6
    return xs, ys, curv


def main():
    pgm = sys.argv[1] if len(sys.argv) > 1 else "maps/my_map.pgm"
    ymlf = sys.argv[2] if len(sys.argv) > 2 else "maps/my_map.yaml"
    out_csv = sys.argv[3] if len(sys.argv) > 3 else "maps/centerline_path.csv"
    n_out = int(sys.argv[4]) if len(sys.argv) > 4 else 300

    wheelbase, max_steer = 0.24, 0.42
    max_curvature = np.tan(max_steer) / wheelbase

    raw, resolution, origin = load_map(pgm, ymlf)
    free = to_binary_free(raw)
    print(f"맵 로드: {raw.shape}, free space 픽셀 {free.sum()}개")

    skel = skeletonize(free)
    print(f"skeleton 픽셀 {skel.sum()}개")

    G = skeleton_to_graph(skel)
    print(f"그래프 노드 {G.number_of_nodes()}개")

    G = prune_spurs(G, min_spur_len=15)
    print(f"가지치기 후 노드 {G.number_of_nodes()}개")

    order = largest_cycle_order(G)
    print(f"폐루프 정렬 완료: {len(order)}개 점")

    world_pts = order_to_world(order, resolution, origin)
    resampled, total_len = resample_equal_arclength(world_pts, n_out)
    print(f"등간격 재샘플링: {n_out}개 점, 총 길이 {total_len:.2f}m")

    print(f"곡률 제약: max_curvature={max_curvature:.3f} 1/m 만족할 때까지 스무딩 반복")
    xs, ys, curv = smooth_until_ok(resampled, max_curvature)

    dx = np.gradient(xs)
    dy = np.gradient(ys)
    yaw = np.arctan2(dy, dx)

    with open(out_csv, 'w') as f:
        f.write("x,y,yaw\n")
        for xi, yi, yawi in zip(xs, ys, yaw):
            f.write(f"{xi:.4f},{yi:.4f},{yawi:.4f}\n")
    print(f"저장 완료: {out_csv}")
    print(f"최종 곡률 초과점: {np.sum(curv > max_curvature)}/{n_out}, 최대곡률={curv.max():.3f}")


if __name__ == "__main__":
    main()
