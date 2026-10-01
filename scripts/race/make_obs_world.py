#!/usr/bin/env python3
"""테스트용 장애물 world 생성 (0.65m 정육면체)

사용: python3 scripts/race/make_obs_world.py BASE.world OUT.world X,Y[,YAW_DEG] X,Y ...
  X,Y : world 좌표 (map 좌표 + (0.06, 0.06) ≈ world, my_map 기준)
"""
import sys

SIZE = 0.65
TEMPLATE = """    <model name="obs_box_{i}">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 {yaw:.4f}</pose>
      <link name="link">
        <collision name="collision"><geometry><box><size>{s} {s} {s}</size></box></geometry></collision>
        <visual name="visual"><geometry><box><size>{s} {s} {s}</size></box></geometry>
          <material><ambient>0.9 0.1 0.1 1</ambient><diffuse>0.9 0.1 0.1 1</diffuse></material></visual>
      </link>
    </model>
"""


def main():
    base, out = sys.argv[1], sys.argv[2]
    w = open(base).read()
    models = ''
    for i, a in enumerate(sys.argv[3:]):
        v = [float(t) for t in a.split(',')]
        yaw = v[2] * 3.14159265 / 180 if len(v) > 2 else 0.0
        models += TEMPLATE.format(i=i, x=v[0], y=v[1], z=SIZE / 2, yaw=yaw, s=SIZE)
    k = w.rfind('</world>')
    open(out, 'w').write(w[:k] + models + w[k:])
    print(f'{out}: 장애물 {len(sys.argv) - 3}개')


if __name__ == '__main__':
    main()
