#!/usr/bin/env python3
import cv2
import yaml
import os
import random
import subprocess

MAP_YAML = "/home/mksung0428/Downloads/LIMO_GAZEBO/ws_limo_humble/maps/my_map.yaml"
SDF_PATH = "/home/mksung0428/Downloads/LIMO_GAZEBO/ws_limo_humble/models/obstacle_box/box.sdf"
N_RANDOM = 2
MIN_WALL_DIST_M = 0.35   # 벽에서 최소 이 거리 이상 떨어진 곳만 후보로
MIN_SEP_M = 1.5          # 장애물끼리 최소 이 거리 이상 떨어지게

with open(MAP_YAML) as f:
    m = yaml.safe_load(f)
img_path = m["image"]
if not img_path.startswith("/"):
    img_path = os.path.join(os.path.dirname(MAP_YAML), img_path)

img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
res = m["resolution"]
ox, oy = m["origin"][0], m["origin"][1]
occ_thresh = m.get("occupied_thresh", 0.65) * 255
h = img.shape[0]

binary = (img > occ_thresh).astype("uint8")
dist_px = cv2.distanceTransform(binary, cv2.DIST_L2, 5)
min_wall_dist_px = MIN_WALL_DIST_M / res

ys, xs = (dist_px > min_wall_dist_px).nonzero()
candidates = list(zip(xs.tolist(), ys.tolist()))

def px_to_world(px, py):
    return ox + px * res, oy + (h - py) * res

random.shuffle(candidates)
chosen = []
for px, py in candidates:
    wx, wy = px_to_world(px, py)
    if all((wx - cx)**2 + (wy - cy)**2 > MIN_SEP_M**2 for cx, cy in chosen):
        chosen.append((wx, wy))
    if len(chosen) >= N_RANDOM:
        break

print(f"랜덤 장애물 {len(chosen)}개 위치:")
for i, (x, y) in enumerate(chosen):
    name = f"obstacle_random_{i}"
    print(f"  {name}: x={x:.2f}, y={y:.2f}")
    cmd = [
        "ros2", "run", "gazebo_ros", "spawn_entity.py",
        "-file", SDF_PATH,
        "-entity", name,
        "-x", str(x), "-y", str(y), "-z", "0.15"
    ]
    subprocess.run(cmd)

print("완료")
