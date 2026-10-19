#!/usr/bin/env python3
"""Rasterize the bundled Go1 MuJoCo maze into a Nav2 occupancy map."""
from pathlib import Path

import mujoco
import numpy as np

workspace = Path(__file__).resolve().parents[3]
model_path = workspace / "go1_mujoco_locomotion/config/go1_maze.mjb"
output = Path(__file__).resolve().parents[1] / "maps"
resolution = 0.05
x_min, x_max = -3.0, 18.0
y_min, y_max = -4.0, 4.0
width = round((x_max - x_min) / resolution)
height = round((y_max - y_min) / resolution)

model = mujoco.MjModel.from_binary_path(str(model_path))
data = mujoco.MjData(model)
key = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
if key >= 0:
    mujoco.mj_resetDataKeyframe(model, data, key)
mujoco.mj_forward(model, data)

# Group 0 contains the floor and static walls. Dynamic Go1 geoms are excluded.
groups = np.zeros(6, dtype=np.uint8)
groups[0] = 1
pixels = np.full((height, width), 254, dtype=np.uint8)
origin = np.array([0.0, 0.0, 3.0])
down = np.array([0.0, 0.0, -1.0])
geom_id = np.zeros(1, dtype=np.int32)
for row in range(height):
    y = y_max - (row + 0.5) * resolution
    for col in range(width):
        x = x_min + (col + 0.5) * resolution
        origin[:2] = x, y
        distance = mujoco.mj_ray(model, data, origin, down, groups, True, -1, geom_id)
        if distance >= 0 and geom_id[0] != 0:
            pixels[row, col] = 0

# Outside the measured enclosure is not traversable.
for row in range(height):
    y = y_max - (row + 0.5) * resolution
    for col in range(width):
        x = x_min + (col + 0.5) * resolution
        if x < -2.2 or x > 17.2 or y < -3.4 or y > 3.4:
            pixels[row, col] = 0

output.mkdir(parents=True, exist_ok=True)
with (output / "go1_maze.pgm").open("wb") as image:
    image.write(f"P5\n{width} {height}\n255\n".encode("ascii"))
    image.write(pixels.tobytes())
(output / "go1_maze.yaml").write_text(
    "image: go1_maze.pgm\n"
    "mode: trinary\n"
    "resolution: 0.05\n"
    "origin: [-3.0, -4.0, 0.0]\n"
    "negate: 0\n"
    "occupied_thresh: 0.65\n"
    "free_thresh: 0.25\n"
)
print(f"Wrote {width}x{height} maze map to {output / 'go1_maze.yaml'}")
