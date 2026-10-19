#!/usr/bin/env python3
"""Build a Go1-friendly alternating-gap maze from the supplied MuJoCo model."""
from pathlib import Path
import mujoco

root = Path(__file__).resolve().parents[2]
source = root / "go1_mujoco_locomotion/config/go1_corridor.mjb"
target = root / "go1_mujoco_locomotion/config/go1_maze.mjb"
model = mujoco.MjModel.from_binary_path(str(source))

# Reuse four existing static collision boxes as staggered barriers. Each wall
# leaves a 2.1 m opening at alternating ends, forcing an S-shaped route while
# keeping generous clearance for the Go1 base and its turning maneuvers.
barriers = {
    "rack_left":  ((3.0, -1.1, 0.75), (0.15, 2.25, 0.75)),
    "rack_right": ((6.0,  1.1, 0.75), (0.15, 2.25, 0.75)),
    "crate_1":    ((9.0, -1.1, 0.75), (0.15, 2.25, 0.75)),
    "crate_2":    ((12.0, 1.1, 0.75), (0.15, 2.25, 0.75)),
}
for name, (position, size) in barriers.items():
    geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
    if geom_id < 0:
        raise RuntimeError(f"Expected reusable static geom {name!r} in source MJB")
    model.geom_pos[geom_id] = position
    model.geom_size[geom_id] = size
    model.geom_rgba[geom_id] = (0.12, 0.42, 0.58, 1.0)

mujoco.mj_saveModel(model, str(target))
print(f"Built maze scene: {target}")
