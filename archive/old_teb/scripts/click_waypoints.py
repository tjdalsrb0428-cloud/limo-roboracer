import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
import yaml

MAP_PGM = "/home/mksung0428/Downloads/LIMO_GAZEBO/ws_limo_humble/maps/my_map.pgm"
MAP_YAML = "/home/mksung0428/Downloads/LIMO_GAZEBO/ws_limo_humble/maps/my_map.yaml"
OUT_CSV = "/home/mksung0428/Downloads/LIMO_GAZEBO/ws_limo_humble/maps/waypoints_manual_new.csv"

raw = np.array(Image.open(MAP_PGM).transpose(Image.FLIP_TOP_BOTTOM))

with open(MAP_YAML, 'r') as f:
    meta = yaml.safe_load(f)
resolution = meta['resolution']
origin = meta['origin']

clicked = []

fig, ax = plt.subplots(figsize=(14, 6))
ax.imshow(raw, cmap='gray', origin='lower')
ax.set_title("트랙 순서대로 클릭 (닫으면 저장됨)")

def onclick(event):
    if event.xdata is None or event.ydata is None:
        return
    px, py = event.xdata, event.ydata
    wx = px * resolution + origin[0]
    wy = py * resolution + origin[1]
    clicked.append((wx, wy))
    ax.plot(px, py, 'ro')
    ax.annotate(str(len(clicked)), (px, py), color='red')
    fig.canvas.draw()

cid = fig.canvas.mpl_connect('button_press_event', onclick)
plt.show()

with open(OUT_CSV, 'w') as f:
    f.write("x,y\n")
    for x, y in clicked:
        f.write(f"{x:.4f},{y:.4f}\n")

print(f"저장 완료: {OUT_CSV} ({len(clicked)}개 점)")
