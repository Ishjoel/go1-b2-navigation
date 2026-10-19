#!/usr/bin/env python3
"""Build a factory-floor Go1 scene from the supplied MuJoCo model.

The supplied ``go1_corridor.mjb`` is a compiled binary, so new geometry cannot
be appended to it directly. This script rebuilds the Go1 robot (bodies,
inertia, joints, actuators, meshes, keyframe, floor contact) into an editable
``MjSpec`` from the compiled model, verifies it against the original with the
trained policy, then adds a warehouse/factory hall around it.

Geom groups:
  0  static obstacles: seen by the 2D laser and rasterized into the Nav2 map
  1  decoration (floor markings, lamps): no collision, excluded from the map
  2  unmapped obstacles (dropped cartons, carts, cones): collide, are seen
     by the laser but are NOT in the Nav2 map, so local avoidance must react
"""
import argparse
from pathlib import Path

import mujoco
import numpy as np

root = Path(__file__).resolve().parents[2]
config = root / "go1_mujoco_locomotion/config"
source = config / "go1_corridor.mjb"
target = config / "go1_factory.mjb"

OBJ = mujoco.mjtObj
HALL_X = (-3.0, 24.0)
HALL_Y = (-8.0, 10.0)
WALL_H = 2.8          # keep below the map rasterizer's 3 m ray origin


def name_of(model, obj, i):
    return mujoco.mj_id2name(model, obj, i) or ""


# ---------------------------------------------------------------------------
# Robot reconstruction
# ---------------------------------------------------------------------------
def robot_spec(model):
    spec = mujoco.MjSpec()
    spec.modelname = "go1 factory scene"
    spec.compiler.degree = False
    spec.compiler.autolimits = True
    spec.compiler.balanceinertia = False

    for field in ("timestep", "impratio", "tolerance", "ls_tolerance", "noslip_tolerance",
                  "ccd_tolerance", "gravity", "wind", "magnetic", "density", "viscosity",
                  "o_margin", "o_solref", "o_solimp", "o_friction", "integrator", "cone",
                  "jacobian", "solver", "iterations", "ls_iterations", "noslip_iterations",
                  "ccd_iterations", "disableflags", "enableflags", "disableactuator",
                  "sdf_initpoints", "sdf_iterations"):
        if hasattr(spec.option, field) and hasattr(model.opt, field):
            setattr(spec.option, field, getattr(model.opt, field))

    # Robot meshes: compiled vertices are already in the mesh frame, so the
    # recompiled mesh frame is the identity and geom poses carry over as-is.
    for i in range(model.nmesh):
        v0, nv = model.mesh_vertadr[i], model.mesh_vertnum[i]
        f0, nf = model.mesh_faceadr[i], model.mesh_facenum[i]
        spec.add_mesh(name=name_of(model, OBJ.mjOBJ_MESH, i),
                      uservert=model.mesh_vert[v0:v0 + nv].ravel().tolist(),
                      userface=model.mesh_face[f0:f0 + nf].ravel().tolist())
    spec.add_material(name="dark", rgba=model.mat_rgba[0].tolist())

    bodies = {0: spec.worldbody}
    for i in range(1, model.nbody):
        parent = bodies[model.body_parentid[i]]
        bodies[i] = parent.add_body(
            name=name_of(model, OBJ.mjOBJ_BODY, i),
            pos=model.body_pos[i].tolist(), quat=model.body_quat[i].tolist(),
            mass=float(model.body_mass[i]), ipos=model.body_ipos[i].tolist(),
            iquat=model.body_iquat[i].tolist(), inertia=model.body_inertia[i].tolist(),
            explicitinertial=True)

    for j in range(model.njnt):
        body = bodies[model.jnt_bodyid[j]]
        if model.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE:
            body.add_freejoint()
            continue
        d = model.jnt_dofadr[j]
        body.add_joint(
            name=name_of(model, OBJ.mjOBJ_JOINT, j), type=model.jnt_type[j],
            pos=model.jnt_pos[j].tolist(), axis=model.jnt_axis[j].tolist(),
            range=model.jnt_range[j].tolist(),
            limited=mujoco.mjtLimited.mjLIMITED_TRUE if model.jnt_limited[j]
            else mujoco.mjtLimited.mjLIMITED_FALSE,
            stiffness=float(model.jnt_stiffness[j]), ref=float(model.qpos0[model.jnt_qposadr[j]]),
            springref=float(model.qpos_spring[model.jnt_qposadr[j]]),
            margin=float(model.jnt_margin[j]),
            solref_limit=model.jnt_solref[j].tolist(), solimp_limit=model.jnt_solimp[j].tolist(),
            damping=float(model.dof_damping[d]), armature=float(model.dof_armature[d]),
            frictionloss=float(model.dof_frictionloss[d]),
            solref_friction=model.dof_solref[d].tolist(),
            solimp_friction=model.dof_solimp[d].tolist())

    for g in range(model.ngeom):
        if model.geom_bodyid[g] == 0:
            continue                      # world geoms are replaced by the factory
        kwargs = dict(
            name=name_of(model, OBJ.mjOBJ_GEOM, g), type=model.geom_type[g],
            pos=model.geom_pos[g].tolist(), quat=model.geom_quat[g].tolist(),
            contype=int(model.geom_contype[g]), conaffinity=int(model.geom_conaffinity[g]),
            condim=int(model.geom_condim[g]), friction=model.geom_friction[g].tolist(),
            solref=model.geom_solref[g].tolist(), solimp=model.geom_solimp[g].tolist(),
            solmix=float(model.geom_solmix[g]), margin=float(model.geom_margin[g]),
            gap=float(model.geom_gap[g]), priority=int(model.geom_priority[g]),
            group=int(model.geom_group[g]), rgba=model.geom_rgba[g].tolist())
        if model.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH:
            kwargs["meshname"] = name_of(model, OBJ.mjOBJ_MESH, model.geom_dataid[g])
        else:
            kwargs["size"] = model.geom_size[g].tolist()
        if model.geom_matid[g] >= 0:
            kwargs["material"] = name_of(model, OBJ.mjOBJ_MATERIAL, model.geom_matid[g])
        bodies[model.geom_bodyid[g]].add_geom(**kwargs)

    for s in range(model.nsite):
        bodies[model.site_bodyid[s]].add_site(
            name=name_of(model, OBJ.mjOBJ_SITE, s), type=model.site_type[s],
            pos=model.site_pos[s].tolist(), quat=model.site_quat[s].tolist(),
            size=model.site_size[s].tolist(), rgba=model.site_rgba[s].tolist(),
            group=int(model.site_group[s]))

    for c in range(model.ncam):
        bodies[model.cam_bodyid[c]].add_camera(
            name=name_of(model, OBJ.mjOBJ_CAMERA, c), mode=model.cam_mode[c],
            pos=model.cam_pos[c].tolist(), quat=model.cam_quat[c].tolist(),
            fovy=float(model.cam_fovy[c]))

    for a in range(model.nu):
        spec.add_actuator(
            name=name_of(model, OBJ.mjOBJ_ACTUATOR, a),
            target=name_of(model, OBJ.mjOBJ_JOINT, model.actuator_trnid[a, 0]),
            trntype=mujoco.mjtTrn.mjTRN_JOINT,
            dyntype=model.actuator_dyntype[a], gaintype=model.actuator_gaintype[a],
            biastype=model.actuator_biastype[a],
            dynprm=model.actuator_dynprm[a].tolist(), gainprm=model.actuator_gainprm[a].tolist(),
            biasprm=model.actuator_biasprm[a].tolist(), gear=model.actuator_gear[a].tolist(),
            ctrllimited=int(model.actuator_ctrllimited[a]),
            ctrlrange=model.actuator_ctrlrange[a].tolist(),
            forcelimited=int(model.actuator_forcelimited[a]),
            forcerange=model.actuator_forcerange[a].tolist())

    for k in range(model.nkey):
        spec.add_key(name=name_of(model, OBJ.mjOBJ_KEY, k), qpos=model.key_qpos[k].tolist(),
                     ctrl=model.key_ctrl[k].tolist(), time=float(model.key_time[k]))

    # Floor: identical contact parameters to the original plane, new look.
    floor = mujoco.mj_name2id(model, OBJ.mjOBJ_GEOM, "floor")
    spec.add_texture(name="concrete", type=mujoco.mjtTexture.mjTEXTURE_2D,
                     builtin=mujoco.mjtBuiltin.mjBUILTIN_CHECKER, mark=mujoco.mjtMark.mjMARK_EDGE,
                     rgb1=[0.56, 0.57, 0.56], rgb2=[0.53, 0.54, 0.53],
                     markrgb=[0.40, 0.40, 0.40], width=512, height=512)
    # 2 x 2 checks per texture tile, 4 m per tile -> 2 m saw-cut concrete slabs.
    floor_mat = spec.add_material(name="factory_floor", texrepeat=[0.25, 0.25], texuniform=True,
                                  reflectance=0.03, specular=0.15, shininess=0.2)
    floor_mat.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = "concrete"
    spec.worldbody.add_geom(
        name="floor", type=mujoco.mjtGeom.mjGEOM_PLANE, size=[0, 0, 0.01],
        pos=model.geom_pos[floor].tolist(), quat=model.geom_quat[floor].tolist(),
        contype=int(model.geom_contype[floor]), conaffinity=int(model.geom_conaffinity[floor]),
        condim=int(model.geom_condim[floor]), friction=model.geom_friction[floor].tolist(),
        solref=model.geom_solref[floor].tolist(), solimp=model.geom_solimp[floor].tolist(),
        solmix=float(model.geom_solmix[floor]), margin=float(model.geom_margin[floor]),
        gap=float(model.geom_gap[floor]), priority=int(model.geom_priority[floor]),
        group=0, material="factory_floor")
    return spec


# ---------------------------------------------------------------------------
# Factory dressing
# ---------------------------------------------------------------------------
class Factory:
    def __init__(self, spec):
        self.spec = spec
        self.world = spec.worldbody
        self.n = 0
        for name, rgba, kw in (
            ("wall", [0.78, 0.79, 0.80, 1], dict(specular=0.1)),
            ("wall_band", [0.16, 0.36, 0.58, 1], {}),
            ("pillar", [0.85, 0.65, 0.10, 1], dict(specular=0.3)),
            ("steel", [0.55, 0.57, 0.60, 1], dict(specular=0.6, shininess=0.6)),
            ("rack_upright", [0.10, 0.30, 0.65, 1], dict(specular=0.4)),
            ("rack_beam", [0.95, 0.45, 0.05, 1], dict(specular=0.4)),
            ("pallet", [0.62, 0.45, 0.25, 1], {}),
            ("carton", [0.72, 0.55, 0.33, 1], {}),
            ("carton_dark", [0.58, 0.42, 0.24, 1], {}),
            ("wrap", [0.80, 0.85, 0.90, 0.85], dict(specular=0.8, shininess=0.9)),
            ("machine", [0.88, 0.89, 0.86, 1], dict(specular=0.4)),
            ("machine_accent", [0.12, 0.50, 0.32, 1], dict(specular=0.4)),
            ("glass", [0.35, 0.55, 0.65, 0.55], dict(specular=0.9, shininess=0.9)),
            ("belt", [0.12, 0.12, 0.13, 1], {}),
            ("yellow", [0.95, 0.78, 0.05, 1], {}),
            ("black", [0.08, 0.08, 0.08, 1], {}),
            ("green", [0.15, 0.62, 0.30, 1], {}),
            ("red", [0.80, 0.12, 0.10, 1], {}),
            ("white", [0.95, 0.95, 0.95, 1], {}),
            ("door", [0.60, 0.62, 0.64, 1], dict(specular=0.5)),
            ("lamp", [1.0, 1.0, 0.92, 1], dict(emission=1.0)),
        ):
            spec.add_material(name=name, rgba=rgba, **kw)

    @staticmethod
    def _flags(solid, mapped):
        if not solid:
            return dict(contype=0, conaffinity=0, group=1)
        # Same contact flags as the original walls: obstacle for laser and feet.
        return dict(contype=1, conaffinity=0, group=0 if mapped else 2)

    def box(self, center, half, material, solid=True, quat=None, mapped=True):
        self.n += 1
        kw = dict(name=f"f{self.n}", type=mujoco.mjtGeom.mjGEOM_BOX, pos=list(center),
                  size=list(half), material=material, **self._flags(solid, mapped))
        if quat is not None:
            kw["quat"] = quat
        self.world.add_geom(**kw)

    def cyl(self, center, radius, half_len, material, solid=True, quat=None, mapped=True):
        self.n += 1
        kw = dict(name=f"f{self.n}", type=mujoco.mjtGeom.mjGEOM_CYLINDER, pos=list(center),
                  size=[radius, half_len, 0], material=material, **self._flags(solid, mapped))
        if quat is not None:
            kw["quat"] = quat
        self.world.add_geom(**kw)

    def floor_line(self, x0, y0, x1, y1, material="yellow", width=0.1):
        cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
        hx = max(abs(x1 - x0) / 2, width / 2)
        hy = max(abs(y1 - y0) / 2, width / 2)
        self.box((cx, cy, 0.002), (hx, hy, 0.001), material, solid=False)

    def hazard_frame(self, x0, y0, x1, y1, stripe=0.25):
        """Yellow/black dashed rectangle painted on the floor."""
        for (a, b, horizontal, fixed) in ((x0, x1, True, y0), (x0, x1, True, y1),
                                         (y0, y1, False, x0), (y0, y1, False, x1)):
            n = max(1, int(round((b - a) / stripe)))
            step = (b - a) / n
            for k in range(n):
                s, e = a + k * step, a + (k + 1) * step
                mat = "yellow" if k % 2 == 0 else "black"
                if horizontal:
                    self.floor_line(s, fixed, e, fixed, mat, 0.08)
                else:
                    self.floor_line(fixed, s, fixed, e, mat, 0.08)

    # -- assets -------------------------------------------------------------
    def walls(self):
        (x0, x1), (y0, y1) = HALL_X, HALL_Y
        t = 0.2
        h = WALL_H / 2
        self.box(((x0 + x1) / 2, y0 - t, h), ((x1 - x0) / 2 + 2 * t, t, h), "wall")
        self.box(((x0 + x1) / 2, y1 + t, h), ((x1 - x0) / 2 + 2 * t, t, h), "wall")
        self.box((x0 - t, (y0 + y1) / 2, h), (t, (y1 - y0) / 2, h), "wall")
        self.box((x1 + t, (y0 + y1) / 2, h), (t, (y1 - y0) / 2, h), "wall")
        # Blue dado band and steel columns along the walls.
        for y in (y0 + 0.005, y1 - 0.005):
            self.box(((x0 + x1) / 2, y, 0.6), ((x1 - x0) / 2, 0.01, 0.6), "wall_band", solid=False)
        for x in (x0 + 0.005, x1 - 0.005):
            self.box((x, (y0 + y1) / 2, 0.6), (0.01, (y1 - y0) / 2, 0.6), "wall_band", solid=False)
        # Columns: south wall every 6 m; north wall only where a rack row
        # meets it, so no column narrows a maze turn.
        columns = [(x, y0 + 0.2) for x in np.arange(x0 + 3, x1, 6.0)]
        columns += [(x, y1 - 0.2) for x in (-0.5, 4.5, 8.5, 16.5)]
        for x, y in columns:
            self.box((x, y, h), (0.2, 0.2, h), "pillar")
            self.box((x, y, 0.35), (0.22, 0.22, 0.35), "yellow", solid=False)
        # Roll-up loading doors on the west wall.
        for y in (-4.5, 4.5):
            self.box((x0 + 0.02, y, 1.3), (0.02, 1.4, 1.3), "door", solid=False)
            for k in range(13):
                self.box((x0 + 0.045, y, 0.1 + k * 0.2), (0.005, 1.4, 0.01), "steel", solid=False)
            self.hazard_frame(x0 + 0.05, y - 1.5, x0 + 1.2, y + 1.5)

    def pallet(self, x, y, layers=2, yaw90=False, wrapped=False, mapped=True):
        hx, hy = (0.6, 0.5) if not yaw90 else (0.5, 0.6)
        self.box((x, y, 0.07), (hx, hy, 0.07), "pallet", mapped=mapped)
        z = 0.14
        rng = np.random.default_rng(int(abs(x * 31 + y * 17)))
        for layer in range(layers):
            for ix in (-1, 1):
                for iy in (-1, 1):
                    mat = "carton" if rng.random() > 0.35 else "carton_dark"
                    self.box((x + ix * hx / 2, y + iy * hy / 2, z + 0.2),
                             (hx / 2 - 0.015, hy / 2 - 0.015, 0.2 - 0.005), mat, mapped=mapped)
            z += 0.4
        if wrapped:
            self.box((x, y, 0.14 + layers * 0.2), (hx + 0.01, hy + 0.01, layers * 0.2), "wrap",
                     solid=False)

    def rack(self, a0, a1, c, axis="x", depth=1.0, levels=(0.0, 0.85, 1.7), height=2.6):
        """Pallet racking from a0 to a1 along ``axis`` centred on c across it."""
        def at(a, b, z):
            return (a, b, z) if axis == "x" else (b, a, z)

        def half(ha, hb, hz):
            return (ha, hb, hz) if axis == "x" else (hb, ha, hz)

        hc = depth / 2
        bays = max(1, int(round((a1 - a0) / 2.7)))
        bw = (a1 - a0) / bays
        # Continuous kick plate so laser and map both see one clean footprint.
        self.box(at((a0 + a1) / 2, c, 0.08), half((a1 - a0) / 2, hc, 0.08), "steel")
        for b in range(bays + 1):
            for side in (c - hc + 0.04, c + hc - 0.04):
                self.box(at(a0 + b * bw, side, height / 2), (0.04, 0.04, height / 2),
                         "rack_upright")
        for lvl in levels[1:] + (height - 0.05,):
            for side in (c - hc + 0.04, c + hc - 0.04):
                self.box(at((a0 + a1) / 2, side, lvl), half((a1 - a0) / 2, 0.03, 0.05),
                         "rack_beam")
        rng = np.random.default_rng(int(a0 * 13 + c * 7 + 101))
        for b in range(bays):
            ba = a0 + (b + 0.5) * bw
            for lvl in levels:
                base = lvl + (0.16 if lvl == 0 else 0.05)
                for k in (-1, 0, 1):
                    if lvl > 0 and rng.random() < 0.18:
                        continue           # an empty slot here and there looks real
                    pa = ba + k * bw / 3
                    self.box(at(pa, c, base + 0.06), half(bw / 6 - 0.06, hc - 0.08, 0.06),
                             "pallet")
                    hgt = 0.28 + 0.06 * rng.random()
                    mat = "carton" if rng.random() > 0.4 else "carton_dark"
                    self.box(at(pa, c, base + 0.12 + hgt), half(bw / 6 - 0.1, hc - 0.12, hgt),
                             mat)
        # Yellow floor line along each aisle-facing side.
        for side in (c - hc - 0.15, c + hc + 0.15):
            if axis == "x":
                self.floor_line(a0, side, a1, side, "yellow", 0.08)
            else:
                self.floor_line(side, a0, side, a1, "yellow", 0.08)

    # -- unmapped clutter for local avoidance ---------------------------------
    def dropped_cartons(self, x, y):
        self.box((x, y, 0.07), (0.35, 0.35, 0.07), "pallet", mapped=False)
        self.box((x - 0.05, y, 0.39), (0.3, 0.3, 0.25), "carton", mapped=False)
        self.box((x + 0.05, y + 0.02, 0.86), (0.24, 0.22, 0.22), "carton_dark", mapped=False,
                 quat=[0.966, 0, 0, 0.259])

    def cart(self, x, y, along_y=True):
        hx, hy = (0.3, 0.5) if along_y else (0.5, 0.3)
        self.box((x, y, 0.75), (hx, hy, 0.02), "steel", mapped=False)
        self.box((x, y, 0.35), (hx - 0.03, hy - 0.03, 0.02), "steel", mapped=False)
        self.box((x, y, 0.55), (hx - 0.04, hy - 0.04, 0.18), "carton", mapped=False)
        for dx in (-hx + 0.03, hx - 0.03):
            for dy in (-hy + 0.03, hy - 0.03):
                self.box((x + dx, y + dy, 0.42), (0.02, 0.02, 0.36), "black", mapped=False)
        self.box((x, y + (hy if along_y else 0), 1.0) if along_y else (x + hx, y, 1.0),
                 (hx, 0.02, 0.02) if along_y else (0.02, hy, 0.02), "red", mapped=False)

    def cone(self, x, y):
        self.cyl((x, y, 0.02), 0.2, 0.02, "black", mapped=False)
        self.cyl((x, y, 0.37), 0.13, 0.33, "red", mapped=False)
        self.cyl((x, y, 0.5), 0.135, 0.05, "white", solid=False)

    def conveyor(self, x0, x1, yc, width=0.8, height=0.8):
        hy = width / 2
        cx, hx = (x0 + x1) / 2, (x1 - x0) / 2
        self.box((cx, yc, height - 0.05), (hx, hy, 0.05), "steel")             # frame
        self.box((cx, yc, height + 0.005), (hx - 0.05, hy - 0.06, 0.005), "belt")
        self.box((cx, yc, 0.42), (hx, hy - 0.05, 0.32), "machine")            # skirt (laser)
        for side in (-1, 1):
            self.box((cx, yc + side * (hy + 0.02), height + 0.08), (hx, 0.02, 0.04), "yellow")
        for x in np.arange(x0 + 0.4, x1, 1.5):
            for side in (-1, 1):
                self.box((x, yc + side * (hy - 0.05), height / 2), (0.04, 0.04, height / 2), "black")
        rng = np.random.default_rng(7)
        for x in np.arange(x0 + 0.8, x1 - 0.4, 1.6 + 0.0):
            s = 0.16 + 0.08 * rng.random()
            self.box((x + 0.3 * rng.random(), yc, height + 0.01 + s), (s * 1.2, s, s),
                     "carton" if rng.random() > 0.5 else "carton_dark")
        # Control cabinet and emergency stop at the head end.
        self.box((x0 - 0.35, yc, 0.8), (0.25, 0.35, 0.8), "machine_accent")
        self.cyl((x0 - 0.35, yc, 1.62), 0.06, 0.02, "red")

    def machine(self, x, y, w=2.2, d=1.6, h=2.1):
        self.box((x, y, h / 2), (w / 2, d / 2, h / 2), "machine")
        self.box((x, y + d / 2 + 0.01, h * 0.55), (w * 0.3, 0.01, h * 0.22), "glass", solid=False)
        self.box((x, y, h + 0.05), (w / 2, d / 2, 0.05), "machine_accent")
        self.box((x - w / 2 + 0.15, y + d / 2 + 0.08, 1.1), (0.12, 0.08, 0.25), "machine_accent")
        # Stack light.
        for k, mat in enumerate(("green", "yellow", "red")):
            self.cyl((x + w / 2 - 0.15, y + d / 2 - 0.15, h + 0.2 + k * 0.12), 0.05, 0.05, mat,
                     solid=False)
        self.hazard_frame(x - w / 2 - 0.3, y - d / 2 - 0.1, x + w / 2 + 0.3, y + d / 2 + 0.45)

    def forklift(self, x, y):
        self.box((x, y, 0.55), (0.75, 0.5, 0.4), "yellow")
        self.box((x - 0.55, y, 0.85), (0.2, 0.48, 0.25), "black")             # counterweight
        for dx in (-0.45, 0.45):
            for dy in (-0.5, 0.5):
                self.cyl((x + dx, y + dy, 0.22), 0.22, 0.08, "black",
                         quat=[0.7071, 0.7071, 0, 0])
        for dx, dy in ((-0.2, -0.45), (-0.2, 0.45), (0.45, -0.45), (0.45, 0.45)):
            self.box((x + dx, y + dy, 1.55), (0.03, 0.03, 0.6), "black", solid=False)
        self.box((x + 0.12, y, 2.15), (0.4, 0.5, 0.03), "black", solid=False)    # roof guard
        self.box((x + 0.85, y, 1.1), (0.04, 0.45, 0.95), "steel")              # mast
        for dy in (-0.25, 0.25):
            self.box((x + 1.35, y + dy, 0.06), (0.5, 0.06, 0.03), "steel")     # forks

    def workbench(self, x, y, w=1.8, d=0.8):
        self.box((x, y, 0.9), (w / 2, d / 2, 0.03), "steel")
        for dx in (-w / 2 + 0.05, w / 2 - 0.05):
            for dy in (-d / 2 + 0.05, d / 2 - 0.05):
                self.box((x + dx, y + dy, 0.45), (0.03, 0.03, 0.45), "black")
        self.box((x, y, 0.45), (w / 2 - 0.05, d / 2 - 0.05, 0.4), "machine_accent")   # drawers
        self.box((x - 0.3, y, 1.05), (0.25, 0.18, 0.12), "carton")
        self.box((x + 0.4, y - 0.1, 0.99), (0.2, 0.15, 0.06), "red")

    def charger(self, x, y):
        self.box((x, y, 0.4), (0.15, 0.4, 0.4), "machine_accent")
        self.box((x - 0.16, y, 0.45), (0.01, 0.25, 0.15), "green", solid=False)
        self.hazard_frame(x - 1.4, y - 0.7, x - 0.15, y + 0.7)

    def lamps(self):
        for x in np.arange(HALL_X[0] + 3, HALL_X[1], 6.0):
            for y in (-5.0, 0.0, 5.0):
                self.box((x, y, 4.2), (0.8, 0.12, 0.04), "lamp", solid=False)
        spec = self.spec
        for x in (2.0, 11.0, 20.0):
            spec.worldbody.add_light(name=f"bay_light_{x:.0f}", pos=[x, 0, 7.5],
                                     dir=[0, 0, -1], type=mujoco.mjtLightType.mjLIGHT_SPOT, castshadow=int(x == 11.0),
                                     diffuse=[0.45, 0.45, 0.42], specular=[0.15, 0.15, 0.15],
                                     cutoff=75, exponent=4, attenuation=[1, 0, 0])
        spec.worldbody.add_light(name="sun", pos=[10.5, 1, 12], dir=[0.2, 0.25, -1],
                                 type=mujoco.mjtLightType.mjLIGHT_DIRECTIONAL, castshadow=1,
                                 diffuse=[0.35, 0.35, 0.33], specular=[0.1, 0.1, 0.1])

    def build(self):
        self.walls()
        # North storage maze (x 4..24, y 1.5..10). A south rack row closes it
        # off from the main aisle except for a 3 m entrance at the east end;
        # four north-south rack rows alternate between the south row and the
        # north wall, leaving 3 m aisles and 3 m turn openings (the old 2.1 m
        # maze openings were too tight for the Go1 to turn through).
        self.rack(1.5, 10.0, 4.5, axis="y")                 # west boundary
        self.rack(5.0, 21.0, 2.0, axis="x")                 # south boundary
        self.rack(2.5, 7.0, 20.5, axis="y")                 # R1, open to the north
        self.rack(5.5, 10.0, 16.5, axis="y")                # R2, open to the south
        self.rack(2.5, 7.0, 12.5, axis="y")                 # R3
        self.rack(5.5, 10.0, 8.5, axis="y")                 # R4
        # South production line: conveyor plus four machining cells.
        self.conveyor(4.5, 19.0, -2.6)
        for x in (6.0, 10.0, 14.0, 18.0):
            self.machine(x, -6.0)
        # Start area: dock pallets, packing benches, a parked forklift.
        self.pallet(-1.8, 8.6, layers=3, wrapped=True)
        self.pallet(-0.2, 8.6, layers=2)
        self.pallet(1.4, 8.6, layers=3, wrapped=True)
        self.pallet(2.8, 6.0, layers=2, yaw90=True)
        self.workbench(0.3, -7.4)
        self.workbench(2.4, -7.4)
        self.forklift(-0.8, -4.6)
        self.pallet(22.8, -7.2, layers=2, wrapped=True)
        self.charger(23.8, -0.4)
        # Unmapped clutter: the global plan goes straight through these until
        # the laser sees them; the local planner has to steer around.
        self.pallet(10.0, 0.9, layers=2, mapped=False)       # main aisle, north side
        for x in (13.5, 14.2, 14.9):                         # cone line, south side
            self.cone(x, -1.0)
        self.dropped_cartons(23.5, 5.0)                      # entrance aisle
        self.cart(13.45, 4.5)                                # aisle 2, against R3
        self.dropped_cartons(11.6, 4.0)                      # aisle 3, against R3
        self.cone(5.5, 6.5)                                  # aisle 4
        # Floor markings: main aisle edges, centre dashes, zebra crossing.
        self.floor_line(-2.0, 1.35, 21.0, 1.35)
        self.floor_line(-2.0, -2.05, 24.0, -2.05)
        for x in np.arange(-1.5, 23.0, 1.2):
            self.floor_line(x, -0.35, x + 0.6, -0.35, "white", 0.08)
        for k, x in enumerate(np.arange(-2.6, 2.4, 0.5)):
            self.floor_line(x, -2.05, x + 0.25, -2.05, "green" if k % 2 else "white", 0.5)
        self.hazard_frame(-1.8, -5.4, 0.6, -3.8)                            # forklift bay
        self.hazard_frame(21.1, 1.5, 23.9, 2.5)                             # maze entrance
        self.lamps()


def build_factory(model):
    spec = robot_spec(model)
    spec.stat.center = [10.5, 1.0, 0.5]
    spec.stat.extent = 14.0
    spec.visual.headlight.ambient = [0.32, 0.32, 0.32]
    spec.visual.headlight.diffuse = [0.35, 0.35, 0.35]
    spec.visual.quality.shadowsize = 4096
    spec.visual.global_.offwidth = 1920
    spec.visual.global_.offheight = 1080
    spec.visual.map.zfar = 60.0
    spec.add_texture(name="sky", type=mujoco.mjtTexture.mjTEXTURE_SKYBOX,
                     builtin=mujoco.mjtBuiltin.mjBUILTIN_GRADIENT,
                     rgb1=[0.55, 0.6, 0.66], rgb2=[0.15, 0.17, 0.2], width=512, height=3072)
    Factory(spec).build()
    return spec


# ---------------------------------------------------------------------------
# Verification against the original model
# ---------------------------------------------------------------------------
def compare_static(a, b):
    """Every robot-side array must match the supplied model exactly."""
    checks = {
        "body_pos": (a.body_pos, b.body_pos), "body_quat": (a.body_quat, b.body_quat),
        "body_mass": (a.body_mass, b.body_mass), "body_inertia": (a.body_inertia, b.body_inertia),
        "body_ipos": (a.body_ipos, b.body_ipos), "body_iquat": (a.body_iquat, b.body_iquat),
        "jnt_range": (a.jnt_range, b.jnt_range), "jnt_axis": (a.jnt_axis, b.jnt_axis),
        "dof_damping": (a.dof_damping, b.dof_damping), "dof_armature": (a.dof_armature, b.dof_armature),
        "dof_frictionloss": (a.dof_frictionloss, b.dof_frictionloss),
        "actuator_gainprm": (a.actuator_gainprm, b.actuator_gainprm),
        "actuator_biasprm": (a.actuator_biasprm, b.actuator_biasprm),
        "actuator_ctrlrange": (a.actuator_ctrlrange, b.actuator_ctrlrange),
        "actuator_forcerange": (a.actuator_forcerange, b.actuator_forcerange),
        "key_qpos": (a.key_qpos, b.key_qpos), "key_ctrl": (a.key_ctrl, b.key_ctrl),
        "dof_invweight0": (a.dof_invweight0, b.dof_invweight0),
    }
    ga = np.where(a.geom_bodyid > 0)[0]
    gb = np.where(b.geom_bodyid > 0)[0]
    for f in ("geom_type", "geom_size", "geom_contype", "geom_conaffinity", "geom_condim",
              "geom_friction", "geom_solref", "geom_solimp", "geom_priority"):
        arr_a, arr_b = getattr(a, f)[ga], getattr(b, f)[gb]
        if f == "geom_size":                   # mesh sizes are recomputed (visual only)
            mask = a.geom_type[ga] != mujoco.mjtGeom.mjGEOM_MESH
            arr_a, arr_b = arr_a[mask], arr_b[mask]
        checks[f] = (arr_a, arr_b)
    for f in ("geom_friction", "geom_solref", "geom_solimp", "geom_condim", "geom_priority"):
        checks["floor_" + f] = (getattr(a, f)[0], getattr(b, f)[0])
    for f in ("timestep", "integrator", "cone", "solver", "iterations", "ls_iterations",
              "disableflags", "enableflags", "impratio", "gravity"):
        checks["opt." + f] = (np.asarray(getattr(a.opt, f)), np.asarray(getattr(b.opt, f)))
    bad = [k for k, (x, y) in checks.items() if np.shape(x) != np.shape(y) or not np.allclose(x, y, atol=1e-9)]
    return bad


def run_policy(model, command, seconds):
    """Same observation/policy/control path as locomotion_node.py."""
    w = np.load(config / "go1_policy_weights.npz")
    nz = np.load(config / "go1_obs_normalizer.npz")
    layers = [(w[f"{i}.kernel"], w[f"{i}.bias"]) for i in range(4)]
    mean, std, eps = nz["mean"], nz["std"], nz["std_eps"]
    model.opt.timestep = 0.004
    model.opt.ccd_iterations = 20
    model.dof_damping[6:] = 0.5
    model.actuator_gainprm[:, 0] = 35.0
    model.actuator_biasprm[:, 1] = -35.0
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    data.ctrl[:] = data.qpos[7:]
    mujoco.mj_forward(model, data)
    default = model.key_qpos[0][7:].astype(np.float32)
    last = np.zeros(12, np.float32)
    cmd = np.asarray(command, np.float32)
    trace = []
    for _ in range(int(seconds * 50)):
        wq, x, y, z = data.qpos[3:7]
        rinv = np.array([[1 - 2*(y*y + z*z), 2*(x*y + z*wq), 2*(x*z - y*wq)],
                         [2*(x*y - z*wq), 1 - 2*(x*x + z*z), 2*(y*z + x*wq)],
                         [2*(x*z + y*wq), 2*(y*z - x*wq), 1 - 2*(x*x + y*y)]], np.float32)
        obs = np.zeros(48, np.float32)
        obs[0:3] = rinv @ data.qvel[0:3].astype(np.float32)
        obs[3:6] = data.qvel[3:6]
        obs[6:9] = [2*(wq*y - x*z), -2*(y*z + wq*x), 2*(x*x + y*y) - 1]
        obs[9:21] = data.qpos[7:19] - default
        obs[21:33] = data.qvel[6:18]
        obs[33:45] = last
        obs[45:48] = cmd
        h = ((obs - mean) / (std + eps)).reshape(1, -1).astype(np.float32)
        for i, (k, b) in enumerate(layers):
            h = h @ k + b
            if i < 3:
                h = h * (1.0 / (1.0 + np.exp(-h)))
        last = np.tanh(h.squeeze()[:12]).astype(np.float32)
        data.ctrl[:] = last * 0.5 + default
        for _ in range(5):
            mujoco.mj_step(model, data)
        trace.append(data.qpos[:3].copy())
    return np.array(trace), data


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--skip-verify", action="store_true")
    args = parser.parse_args()

    original = mujoco.MjModel.from_binary_path(str(source))
    robot_only = robot_spec(original).compile()
    bad = compare_static(original, robot_only)
    if bad:
        raise SystemExit(f"Rebuilt Go1 differs from {source.name}: {bad}")
    print("Static check: rebuilt Go1 matches the supplied model (bodies, joints, actuators, contacts).")

    spec = build_factory(original)
    factory = spec.compile()
    if not args.skip_verify:
        for cmd in ((0.8, 0.0, 0.0), (0.5, 0.0, 0.8)):
            ref, _ = run_policy(mujoco.MjModel.from_binary_path(str(source)), cmd, 8.0)
            new, d = run_policy(spec.compile(), cmd, 8.0)
            drift = np.abs(ref - new).max()
            print(f"Policy check cmd={cmd}: final ref={ref[-1].round(3)} new={new[-1].round(3)} "
                  f"max drift={drift:.2e} m, height={d.qpos[2]:.3f} m")
            if drift > 0.05 or d.qpos[2] < 0.2:
                raise SystemExit("Policy behaviour diverged in the rebuilt model")
    mujoco.mj_saveModel(factory, str(target))
    print(f"Built factory scene: {target} ({factory.ngeom} geoms, {factory.nlight} lights)")


if __name__ == "__main__":
    main()
