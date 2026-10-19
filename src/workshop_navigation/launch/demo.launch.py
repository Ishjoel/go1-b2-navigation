"""Backward-compatible entry point; the active demo is Go1 MuJoCo + Nav2."""
from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path


def generate_launch_description():
    target = Path(__file__).with_name("go1_mujoco.launch.py")
    spec = spec_from_file_location("workshop_go1_mujoco_launch", target)
    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()
