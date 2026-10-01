#!/usr/bin/env python3
import sys
import csv
import numpy as np
from scipy.interpolate import CubicSpline

def load_waypoints(path):
    pts = []
    with open(path) as f:
        r = csv.reader(f)
        next(r)
        for row in r:
            pts.append((float(row[0]), float(row[1])))
    return np.array(pts)

def make_periodic_spline(pts, n_out):
    closed = np.vstack([pts, pts[0]])
    diffs = np.diff(closed, axis=0)
    seglen = np.sqrt((diffs ** 2).sum(axis=1))
    t = np.concatenate([[0], np.cumsum(seglen)])
    total_len = t[-1]

    cs_x = CubicSpline(t, closed[:, 0], bc_type='periodic')
    cs_y = CubicSpline(t, closed[:, 1], bc_type='periodic')

    t_new = np.linspace(0, total_len, n_out, endpoint=False)
    x_new = cs_x(t_new)
    y_new = cs_y(t_new)
    dx = cs_x(t_new, 1)
    dy = cs_y(t_new, 1)
    yaw_new = np.arctan2(dy, dx)
    return x_new, y_new, yaw_new, total_len

def check_curvature(x, y, wheelbase=0.24, max_steer=0.42):
    min_radius = wheelbase / np.tan(max_steer)
    max_curvature = 1.0 / min_radius
    dx = np.gradient(x)
    dy = np.gradient(y)
    ddx = np.gradient(dx)
    ddy = np.gradient(dy)
    denom = (dx ** 2 + dy ** 2) ** 1.5
    denom[denom < 1e-6] = 1e-6
    curvature = np.abs(dx * ddy - dy * ddx) / denom
    n_exceed = np.sum(curvature > max_curvature)
    return n_exceed, max_curvature, curvature.max()

def main():
    in_path = sys.argv[1] if len(sys.argv) > 1 else "maps/waypoints_manual.csv"
    out_path = sys.argv[2] if len(sys.argv) > 2 else "maps/spline_path.csv"
    n_out = int(sys.argv[3]) if len(sys.argv) > 3 else 300

    pts = load_waypoints(in_path)
    print(f"입력 waypoint {len(pts)}개 로드: {in_path}")

    x, y, yaw, total_len = make_periodic_spline(pts, n_out)
    print(f"폐루프 총 길이: {total_len:.2f}m, 보간 점 개수: {n_out}")

    n_exceed, max_curv, actual_max = check_curvature(x, y)
    print(f"곡률 체크: LIMO 최대허용곡률={max_curv:.3f} 1/m, 실제최대={actual_max:.3f} 1/m, 초과점 {n_exceed}/{n_out}개")
    if n_exceed > 0:
        print("경고: 곡률 초과 지점 있음. waypoint 간격을 더 촘촘히 찍거나 점개수를 늘려보세요.")

    with open(out_path, 'w') as f:
        f.write("x,y,yaw\n")
        for xi, yi, yawi in zip(x, y, yaw):
            f.write(f"{xi:.4f},{yi:.4f},{yawi:.4f}\n")
    print(f"저장 완료: {out_path}")

if __name__ == "__main__":
    main()
