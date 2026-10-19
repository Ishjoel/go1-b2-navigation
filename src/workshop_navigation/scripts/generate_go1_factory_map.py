#!/usr/bin/env python3
"""Rasterize the Go1 MuJoCo factory scene into a Nav2 occupancy map.

Only geom group 0 (static obstacles) is rasterized; floor markings and lamps
live in group 1 so painted lines never become obstacles.
"""
from pathlib import Path

import mujoco
import numpy as np

workspace = Path(__file__).resolve().parents[3]
model_path = workspace / "go1_mujoco_locomotion/config/go1_factory.mjb"
output = Path(__file__).resolve().parents[1] / "maps"
name = "go1_factory"
resolution = 0.05
x_min, x_max = -4.0, 22.0
y_min, y_max = -8.5, 8.5
hall_x, hall_y = (-3.0, 21.0), (-7.5, 7.5)
width = round((x_max - x_min) / resolution)
height = round((y_max - y_min) / resolution)

model = mujoco.MjModel.from_binary_path(str(model_path))
data = mujoco.MjData(model)
mujoco.mj_resetDataKeyframe(model, data, 0)
mujoco.mj_forward(model, data)
floor = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "floor")

groups = np.zeros(6, dtype=np.uint8)
groups[0] = 1
pixels = np.full((height, width), 254, dtype=np.uint8)
origin = np.array([0.0, 0.0, 2.95])
down = np.array([0.0, 0.0, -1.0])
geom_id = np.zeros(1, dtype=np.int32)
for row in range(height):
    y = y_max - (row + 0.5) * resolution
    for col in range(width):
        x = x_min + (col + 0.5) * resolution
        origin[:2] = x, y
        distance = mujoco.mj_ray(model, data, origin, down, groups, True, -1, geom_id)
        outside = not (hall_x[0] < x < hall_x[1] and hall_y[0] < y < hall_y[1])
        if outside or (distance >= 0 and geom_id[0] != floor):
            pixels[row, col] = 0

output.mkdir(parents=True, exist_ok=True)
with (output / f"{name}.pgm").open("wb") as image:
    image.write(f"P5\n{width} {height}\n255\n".encode("ascii"))
    image.write(pixels.tobytes())
(output / f"{name}.yaml").write_text(
    f"image: {name}.pgm\n"
    "mode: trinary\n"
    f"resolution: {resolution}\n"
    f"origin: [{x_min}, {y_min}, 0.0]\n"
    "negate: 0\n"
    "occupied_thresh: 0.65\n"
    "free_thresh: 0.25\n"
)
print(f"Wrote {width}x{height} factory map to {output / (name + '.yaml')}")
