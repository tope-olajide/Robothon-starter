"""Combined ARSA-X robot: Franka Emika Panda arm + Allegro Hand.

Uses MuJoCo 3.x MjSpec.attach() to compose the two Menagerie models
into a single 23-DOF system (7 arm + 16 hand).
"""

from pathlib import Path

import mujoco
import numpy as np

# ---------------------------------------------------------------------------
# Joint name lists
# ---------------------------------------------------------------------------
HAND_PREFIX = "hand_"

PANDA_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]
ALLEGRO_JOINTS = [
    f"{HAND_PREFIX}ffj0", f"{HAND_PREFIX}ffj1", f"{HAND_PREFIX}ffj2", f"{HAND_PREFIX}ffj3",
    f"{HAND_PREFIX}mfj0", f"{HAND_PREFIX}mfj1", f"{HAND_PREFIX}mfj2", f"{HAND_PREFIX}mfj3",
    f"{HAND_PREFIX}rfj0", f"{HAND_PREFIX}rfj1", f"{HAND_PREFIX}rfj2", f"{HAND_PREFIX}rfj3",
    f"{HAND_PREFIX}thj0", f"{HAND_PREFIX}thj1", f"{HAND_PREFIX}thj2", f"{HAND_PREFIX}thj3",
]

# ---------------------------------------------------------------------------
# Predefined hand poses
# ---------------------------------------------------------------------------
def _hp(name: str) -> str:
    return f"{HAND_PREFIX}{name}"

ALLEGRO_OPEN = {_hp("ffj0"): 0.0, _hp("ffj1"): 0.0, _hp("ffj2"): 0.0, _hp("ffj3"): 0.0,
                _hp("mfj0"): 0.0, _hp("mfj1"): 0.0, _hp("mfj2"): 0.0, _hp("mfj3"): 0.0,
                _hp("rfj0"): 0.0, _hp("rfj1"): 0.0, _hp("rfj2"): 0.0, _hp("rfj3"): 0.0,
                _hp("thj0"): 0.8, _hp("thj1"): 0.2, _hp("thj2"): 0.0, _hp("thj3"): 0.0}

ALLEGRO_CLOSE = {_hp("ffj0"): 0.0, _hp("ffj1"): 1.35, _hp("ffj2"): 1.35, _hp("ffj3"): 1.3,
                 _hp("mfj0"): 0.0, _hp("mfj1"): 1.35, _hp("mfj2"): 1.35, _hp("mfj3"): 1.3,
                 _hp("rfj0"): 0.0, _hp("rfj1"): 1.35, _hp("rfj2"): 1.35, _hp("rfj3"): 1.3,
                 _hp("thj0"): 1.1, _hp("thj1"): 1.0, _hp("thj2"): 1.1, _hp("thj3"): 1.3}

ALLEGRO_PINCH = {_hp("ffj0"): 0.0, _hp("ffj1"): 0.8, _hp("ffj2"): 0.8, _hp("ffj3"): 0.8,
                 _hp("mfj0"): 0.0, _hp("mfj1"): 0.0, _hp("mfj2"): 0.0, _hp("mfj3"): 0.0,
                 _hp("rfj0"): 0.0, _hp("rfj1"): 0.0, _hp("rfj2"): 0.0, _hp("rfj3"): 0.0,
                 _hp("thj0"): 0.6, _hp("thj1"): 0.6, _hp("thj2"): 0.8, _hp("thj3"): 0.8}

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------
_HERE = Path(__file__).resolve().parent.parent.parent
_VENDOR = _HERE / "vendor" / "mujoco_menagerie"
PANDA_DIR = _VENDOR / "franka_emika_panda"
ALLEGRO_DIR = _VENDOR / "wonik_allegro"
MANAGERIE_MISSING = not (PANDA_DIR.exists() and ALLEGRO_DIR.exists())

NEEDLE_WELD_NAME = "needle_grasp_weld"

# Allegro position-actuator stiffness
HAND_KP = 5.0
HAND_KV = 0.0


# ---------------------------------------------------------------------------
# Assistant arm (second Panda for bimanual tissue stabilization)
# ---------------------------------------------------------------------------

ASSISTANT_PREFIX = "assistant_"
ASSISTANT_PANDA_JOINTS = [f"{ASSISTANT_PREFIX}{j}" for j in PANDA_JOINTS]

# Preset poses for the assistant arm
ASSISTANT_HOME_POSE = {
    "joint1": 0.0, "joint2": -0.40, "joint3": 0.0,
    "joint4": -2.00, "joint5": -0.40, "joint6": 1.50, "joint7": 0.0,
}

ASSISTANT_TISSUE_PRESS_POSE = {
    "joint1": 0.0, "joint2": -0.35, "joint3": 0.0,
    "joint4": -2.20, "joint5": -0.45, "joint6": 1.80, "joint7": 0.0,
}


# ---------------------------------------------------------------------------
# Utilities
# ---------------------------------------------------------------------------

def get_joint_qposadr(model: mujoco.MjModel, name: str) -> int:
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    if jid < 0:
        raise KeyError(f"Joint {name!r} not found")
    return int(model.jnt_qposadr[jid])


def _camera_xyaxes(
    pos: tuple[float, float, float],
    target: tuple[float, float, float],
) -> tuple[float, float, float, float, float, float]:
    """Compute xyaxes so the camera at *pos* looks at *target*."""
    pos_a = np.array(pos, dtype=float)
    tgt_a = np.array(target, dtype=float)
    z = pos_a - tgt_a
    zn = np.linalg.norm(z)
    if zn < 1e-10:
        z = np.array([0.0, 0.0, 1.0])
    else:
        z /= zn
    up = np.array([0.0, 0.0, 1.0])
    x = np.cross(up, z)
    xn = np.linalg.norm(x)
    if xn < 1e-10:
        up = np.array([0.0, 1.0, 0.0])
        x = np.cross(up, z)
        xn = np.linalg.norm(x)
        if xn < 1e-10:
            x = np.array([1.0, 0.0, 0.0])
        else:
            x /= xn
    else:
        x /= xn
    y = np.cross(z, x)
    return (float(x[0]), float(x[1]), float(x[2]), float(y[0]), float(y[1]), float(y[2]))


# ---------------------------------------------------------------------------
# Weld management
# ---------------------------------------------------------------------------

def activate_needle_weld(model: mujoco.MjModel, data: mujoco.MjData) -> bool:
    """Engage the palm-to-needle weld at the CURRENT relative pose."""
    eq_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_EQUALITY, NEEDLE_WELD_NAME)
    if eq_id < 0:
        return False
    palm = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "hand_palm")
    needle = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "needle")
    if palm < 0 or needle < 0:
        return False

    palm_pos = data.xpos[palm]
    palm_quat = np.zeros(4)
    mujoco.mju_mat2Quat(palm_quat, data.xmat[palm])
    inv_palm = np.zeros(4)
    mujoco.mju_negQuat(inv_palm, palm_quat)

    dpos = data.xpos[needle] - palm_pos
    relpos = np.zeros(3)
    mujoco.mju_rotVecQuat(relpos, dpos, inv_palm)

    needle_quat = np.zeros(4)
    mujoco.mju_mat2Quat(needle_quat, data.xmat[needle])
    relquat = np.zeros(4)
    mujoco.mju_mulQuat(relquat, inv_palm, needle_quat)

    eq_data = np.zeros(11)
    eq_data[0:3] = relpos
    eq_data[3:6] = relpos
    eq_data[6:10] = relquat
    eq_data[10] = 1.0
    model.eq_data[eq_id] = eq_data
    data.eq_active[eq_id] = 1
    return True


def release_needle_weld(model: mujoco.MjModel, data: mujoco.MjData) -> None:
    """Disengage the palm-to-needle weld."""
    eq_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_EQUALITY, NEEDLE_WELD_NAME)
    if eq_id >= 0:
        data.eq_active[eq_id] = 0


# ---------------------------------------------------------------------------
# Robot interface
# ---------------------------------------------------------------------------

class AssistantArm:
    """Interface for the second Panda arm (no hand) used in bimanual procedures."""

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData):
        self.model = model
        self.data = data
        self._joint_addrs: dict[str, int] = {}
        self._actuator_map: dict[str, int] = {}

        for jn in PANDA_JOINTS:
            prefixed = f"{ASSISTANT_PREFIX}{jn}"
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, prefixed)
            if jid >= 0:
                self._joint_addrs[jn] = int(model.jnt_qposadr[jid])

        for i in range(model.nu):
            act_jid = int(model.actuator_trnid[i][0])
            jname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, act_jid)
            if jname and jname.startswith(ASSISTANT_PREFIX):
                short_name = jname[len(ASSISTANT_PREFIX):]
                self._actuator_map[short_name] = i

    @property
    def available(self) -> bool:
        """True if the assistant arm joints were found in the model."""
        return len(self._actuator_map) > 0

    def set_joint(self, name: str, value: float) -> None:
        if name in self._actuator_map:
            self.data.ctrl[self._actuator_map[name]] = float(value)
        if name in self._joint_addrs:
            self.data.qpos[self._joint_addrs[name]] = float(value)

    def set_pose(self, joint_values: dict[str, float]) -> None:
        for n, v in joint_values.items():
            self.set_joint(n, v)

    def set_home(self) -> None:
        self.set_pose(ASSISTANT_HOME_POSE)

    def set_tissue_press(self) -> None:
        self.set_pose(ASSISTANT_TISSUE_PRESS_POSE)

    def end_effector_pos(self) -> np.ndarray:
        bid = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, f"{ASSISTANT_PREFIX}attachment"
        )
        if bid < 0:
            return np.zeros(3)
        return self.data.xpos[bid].copy()

    def get_joint(self, name: str) -> float:
        if name in self._joint_addrs:
            return float(self.data.qpos[self._joint_addrs[name]])
        return 0.0


class ARSALRobot:
    """High-level interface for the combined Panda + Allegro Hand robot."""

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData):
        self.model = model
        self.data = data
        self._cache_addrs()

    def _cache_addrs(self):
        self._panda_qpos: dict[str, int] = {}
        self._allegro_qpos: dict[str, int] = {}
        self._actuator_map: dict[str, int] = {}
        for jn in PANDA_JOINTS:
            try:
                self._panda_qpos[jn] = get_joint_qposadr(self.model, jn)
            except KeyError:
                pass
        for jn in ALLEGRO_JOINTS:
            try:
                self._allegro_qpos[jn] = get_joint_qposadr(self.model, jn)
            except KeyError:
                pass
        for i in range(self.model.nu):
            act_jid = int(self.model.actuator_trnid[i][0])
            jname = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, act_jid)
            if jname:
                self._actuator_map[jname] = i

    def set_panda_joint(self, name: str, value: float) -> None:
        if name in self._panda_qpos:
            self.data.qpos[self._panda_qpos[name]] = float(value)
        if name in self._actuator_map:
            self.data.ctrl[self._actuator_map[name]] = float(value)

    def set_panda_pose(self, joint_values: dict[str, float]) -> None:
        for n, v in joint_values.items():
            self.set_panda_joint(n, v)

    def get_panda_joints(self) -> dict[str, float]:
        return {jn: float(self.data.qpos[self._panda_qpos[jn]])
                for jn in PANDA_JOINTS if jn in self._panda_qpos}

    def set_allegro_joint(self, name: str, value: float) -> None:
        if name in self._actuator_map:
            self.data.ctrl[self._actuator_map[name]] = float(value)
        elif name in self._allegro_qpos:
            self.data.qpos[self._allegro_qpos[name]] = float(value)

    def set_allegro_pose(self, pose: dict[str, float]) -> None:
        for n, v in pose.items():
            self.set_allegro_joint(n, v)

    def set_allegro_open(self) -> None:
        self.set_allegro_pose(ALLEGRO_OPEN)

    def set_allegro_close(self) -> None:
        self.set_allegro_pose(ALLEGRO_CLOSE)

    def set_allegro_pinch(self) -> None:
        self.set_allegro_pose(ALLEGRO_PINCH)

    def end_effector_pos(self) -> np.ndarray:
        bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "attachment")
        if bid < 0:
            return np.zeros(3)
        return self.data.xpos[bid].copy()

    def end_effector_quat(self) -> np.ndarray:
        bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "attachment")
        if bid < 0:
            return np.array([1.0, 0.0, 0.0, 0.0])
        return self.data.xquat[bid].copy()


# ---------------------------------------------------------------------------
# Actuator tuning
# ---------------------------------------------------------------------------

def _tune_hand_actuators(model: mujoco.MjModel) -> None:
    """Raise Allegro finger actuator gains so the hand can actually grip."""
    for i in range(model.nu):
        jid = int(model.actuator_trnid[i][0])
        jn = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, jid)
        if jn and jn.startswith(HAND_PREFIX):
            model.actuator_gainprm[i][0] = HAND_KP
            model.actuator_biasprm[i][0] = 0.0
            model.actuator_biasprm[i][1] = -HAND_KP
            model.actuator_biasprm[i][2] = -HAND_KV


def attach_assistant_arm(spec: mujoco.MjSpec, wb: mujoco.MjsBody) -> None:
    """Attach a second Panda arm (no hand) for bimanual tissue stabilization.

    The assistant arm is positioned on the opposite side of the surgical table
    from the primary arm.  It has a flat pad end-effector for pressing tissue,
    enabling bimanual coordination during suturing procedures.
    """
    panda_xml = str((PANDA_DIR / "panda_nohand.xml").resolve())
    assistant_spec = mujoco.MjSpec.from_file(panda_xml)

    # Add a flat pad at the end-effector for tissue pressing
    attach_body = assistant_spec.body("attachment")
    if attach_body is not None:
        # Names use bare names here — the attach() prefix "assistant_" will
        # be prepended automatically, yielding "assistant_pad" etc.
        attach_body.add_geom(
            name="pad",
            type=mujoco.mjtGeom.mjGEOM_CYLINDER,
            size=(0.025, 0.005),
            pos=(0.0, 0.0, -0.02),
            rgba=(0.6, 0.6, 0.65, 1.0),
            contype=3, conaffinity=3, condim=3,
            friction=(0.5, 0.01, 0.01),
        )
        attach_body.add_site(
            name="pad_site",
            pos=(0.0, 0.0, -0.03),
            size=(0.005,),
        )

    # Create root body on the opposite side of the tissue, raised above table
    # and moved further back to avoid intersection with the surgical table.
    # Check if the body already exists (e.g., loaded from XML).
    existing = spec.body("assistant_robot_root")
    if existing is not None:
        assistant_root = existing
    else:
        assistant_root = wb.add_body(name="assistant_robot_root", pos=(1.0, 0.0, 0.18))
        assistant_root.add_site(name="assistant_attach_site")
    # Ensure the site exists for attachment
    if assistant_root.first_site() is None:
        assistant_root.add_site(name="assistant_attach_site")
    spec.attach(assistant_spec, prefix=ASSISTANT_PREFIX, site=assistant_root.first_site())


# ---------------------------------------------------------------------------
# Scene assembly (constructs the full surgical scene spec)
# ---------------------------------------------------------------------------

def _build_spring_mesh_tissue(wb: mujoco.MjsBody, spec: mujoco.MjSpec) -> None:
    """Build a deformable spring-mesh tissue from sphere geoms + equality constraints."""
    rows, cols = 5, 4
    spacing_x = 0.038
    spacing_y = 0.032
    center_x = 0.45
    center_y = -0.02
    z_surface = 0.370
    sphere_r = 0.007

    table_body = spec.body("table")
    bodies: list[list[mujoco.MjsBody]] = []
    for r in range(rows):
        row_bodies: list[mujoco.MjsBody] = []
        for c in range(cols):
            x = center_x + (c - (cols - 1) / 2) * spacing_x
            y = center_y + (r - (rows - 1) / 2) * spacing_y
            body = wb.add_body(name=f"tissue_s_{r}_{c}", pos=(x, y, z_surface))
            body.add_freejoint(name=f"tissue_j_{r}_{c}")
            body.add_geom(
                name=f"tissue_g_{r}_{c}",
                type=mujoco.mjtGeom.mjGEOM_SPHERE,
                size=(sphere_r,),
                rgba=(0.75, 0.35, 0.28, 0.92),
                condim=6, friction=(0.5, 0.01, 0.01),
            )
            row_bodies.append(body)
        bodies.append(row_bodies)

    def _connect(b1, b2, name, solref=(0.02, 0.8), solimp=(0.1, 0.95, 0.05, 0.5, 0.5)):
        eq = spec.add_equality()
        eq.name = name
        eq.type = mujoco.mjtEq.mjEQ_CONNECT
        eq.objtype = mujoco.mjtObj.mjOBJ_BODY
        eq.name1 = b1.name
        eq.name2 = b2.name
        eq.data = [0.0] * 11
        eq.solref = list(solref)
        eq.solimp = list(solimp)

    def _weld(b1, b2, name, solref=(0.01, 1.0), solimp=(0.8, 0.95, 0.01, 0.5, 0.5)):
        eq = spec.add_equality()
        eq.name = name
        eq.type = mujoco.mjtEq.mjEQ_WELD
        eq.objtype = mujoco.mjtObj.mjOBJ_BODY
        eq.name1 = b1.name
        if b2 is not None:
            eq.name2 = b2.name
        eq.data = [0.0] * 11
        eq.solref = list(solref)
        eq.solimp = list(solimp)

    for r in range(rows):
        for c in range(cols):
            if c + 1 < cols:
                _connect(bodies[r][c], bodies[r][c + 1], f"tissue_eq_r{r}_{c}")
            if r + 1 < rows:
                _connect(bodies[r][c], bodies[r + 1][c], f"tissue_eq_d{r}_{c}")
            if r + 1 < rows and c + 1 < cols:
                _connect(bodies[r][c], bodies[r + 1][c + 1], f"tissue_eq_diag{r}_{c}",
                         solref=(0.03, 0.6))

    if table_body is not None:
        for r, c in [(0, 0), (0, cols - 1), (rows - 1, 0), (rows - 1, cols - 1)]:
            _weld(bodies[r][c], table_body, f"tissue_anchor_{r}_{c}")


def _add_sensors(spec: mujoco.MjSpec, all_joints: list[str]) -> None:
    """Add joint position sensors, wrist F/T sensors, and position actuators."""
    for jn in all_joints:
        s = spec.add_sensor(f"sensor_{jn}", type=mujoco.mjtSensor.mjSENS_JOINTPOS)
        s.objtype = mujoco.mjtObj.mjOBJ_JOINT
        s.objname = jn
    for jn in all_joints:
        s = spec.add_sensor(f"sensor_{jn}_vel", type=mujoco.mjtSensor.mjSENS_JOINTVEL)
        s.objtype = mujoco.mjtObj.mjOBJ_JOINT
        s.objname = jn

    wrist_force = spec.add_sensor("sensor_wrist_force", type=mujoco.mjtSensor.mjSENS_FORCE)
    wrist_force.objtype = mujoco.mjtObj.mjOBJ_SITE
    wrist_force.objname = "attachment_site"

    wrist_torque = spec.add_sensor("sensor_wrist_torque", type=mujoco.mjtSensor.mjSENS_TORQUE)
    wrist_torque.objtype = mujoco.mjtObj.mjOBJ_SITE
    wrist_torque.objname = "attachment_site"


# ---------------------------------------------------------------------------
# Scene builders
# ---------------------------------------------------------------------------


def build_scene_model(bimanual: bool = False) -> mujoco.MjModel:
    """Build the complete surgical scene with robot, table, tissue, needle."""
    scene_spec = mujoco.MjSpec()
    scene_spec.option.timestep = 0.002
    scene_spec.option.gravity = (0.0, 0.0, -9.81)
    scene_spec.option.iterations = 100
    scene_spec.compiler.balanceinertia = 1
    scene_spec.visual.global_.offwidth = 1280
    scene_spec.visual.global_.offheight = 720

    wb = scene_spec.worldbody

    wb.add_geom(name="floor", type=mujoco.mjtGeom.mjGEOM_PLANE,
                size=(0, 0, 0.05), rgba=(0.08, 0.10, 0.12, 1.0),
                contype=1, conaffinity=1)

    table = wb.add_body(name="table", pos=(0.5, 0.0, 0.0))
    table.add_geom(name="table_top", type=mujoco.mjtGeom.mjGEOM_BOX,
                   size=(0.35, 0.25, 0.015), pos=(0, 0, 0.35),
                   rgba=(0.55, 0.55, 0.58, 1.0), contype=3, conaffinity=3)
    for lx, ly in [(-0.30, -0.20), (-0.30, 0.20), (0.30, -0.20), (0.30, 0.20)]:
        table.add_geom(name=f"table_leg_{lx}_{ly}", type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                       size=(0.015, 0.35), pos=(lx, ly, 0.35),
                       rgba=(0.4, 0.4, 0.42, 1.0), contype=3, conaffinity=3)

    _build_spring_mesh_tissue(wb, scene_spec)

    stand = wb.add_body(name="needle_stand", pos=(0.666, 0.017, 0.0))
    stand.add_geom(name="stand_post", type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                   size=(0.007, 0.205), pos=(0.0, 0.0, 0.205),
                   rgba=(0.25, 0.27, 0.30, 1.0), contype=1, conaffinity=1)
    stand.add_geom(name="stand_saddle", type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                   size=(0.020, 0.005), pos=(0.0, 0.0, 0.410),
                   rgba=(0.30, 0.32, 0.36, 1.0),
                   condim=3, friction=(0.5, 0.02, 0.001), contype=1, conaffinity=1)
    import math
    for k in range(6):
        ang = 2 * math.pi * k / 6
        stand.add_geom(name=f"socket_post_{k}", type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                       size=(0.0025, 0.012),
                       pos=(0.019*math.cos(ang), 0.019*math.sin(ang), 0.427),
                       rgba=(0.32, 0.34, 0.38, 1.0),
                       condim=3, friction=(0.3, 0.02, 0.001), contype=1, conaffinity=1)

    needle = wb.add_body(name="needle", pos=(0.666, 0.017, 0.45))
    needle.add_freejoint(name="needle_freejoint")
    needle.add_geom(name="needle_geom", type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                    size=(0.006, 0.030), pos=(0.0, 0.0, 0.0),
                    rgba=(0.82, 0.82, 0.86, 1.0),
                    condim=6, friction=(3.5, 0.1, 0.01),
                    solref=(0.01, 1.0), solimp=(0.95, 0.99, 0.001, 0.5, 2.0),
                    mass=0.01, contype=1, conaffinity=1)
    needle.add_geom(name="needle_base", type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                    size=(0.012, 0.004), pos=(0.0, 0.0, -0.035),
                    rgba=(0.55, 0.57, 0.60, 1.0),
                    condim=3, friction=(0.4, 0.02, 0.001),
                    mass=0.015, contype=1, conaffinity=1)
    needle.add_geom(name="needle_tip", type=mujoco.mjtGeom.mjGEOM_CAPSULE,
                    size=(0.0025, 0.014), pos=(0.012, 0.0, 0.040),
                    quat=(0.7071, 0.0, 0.7071, 0.0),
                    rgba=(0.9, 0.9, 0.94, 1.0),
                    condim=6, friction=(2.5, 0.05, 0.005),
                    mass=0.001, contype=1, conaffinity=1)
    needle.add_geom(name="suture_geom", type=mujoco.mjtGeom.mjGEOM_CAPSULE,
                    size=(0.0008, 0.05), pos=(0.0, 0.0, -0.075),
                    rgba=(0.9, 0.9, 0.2, 0.8), contype=0, conaffinity=0)

    tray = wb.add_body(name="tray", pos=(0.75, -0.18, 0.36))
    tray.add_geom(name="tray_base", type=mujoco.mjtGeom.mjGEOM_BOX,
                  size=(0.08, 0.04, 0.005), rgba=(0.25, 0.25, 0.28, 1.0))

    wb.add_light(name="light_dome", pos=(0.5, -0.8, 1.5), dir=(0.0, 0.6, -1.0))
    wb.add_light(name="light_surgical", pos=(0.4, 0.0, 1.2), dir=(0.0, 0.0, -1.0))
    wb.add_light(name="light_fill", pos=(-0.3, 0.6, 0.8), dir=(0.5, -0.3, -1.0))

    wb.add_camera(name="cam_wide", pos=(0.80, -0.90, 1.10),
                  xyaxes=_camera_xyaxes((0.80, -0.90, 1.10), (0.45, 0.0, 0.40)))
    wb.add_camera(name="cam_overhead", pos=(0.45, -0.15, 1.50),
                  xyaxes=_camera_xyaxes((0.45, -0.15, 1.50), (0.45, 0.0, 0.37)))
    wb.add_camera(name="cam_closeup", pos=(0.72, -0.50, 0.60),
                  xyaxes=_camera_xyaxes((0.72, -0.50, 0.60), (0.666, 0.017, 0.46)))
    wb.add_camera(name="cam_endoscopic", pos=(0.30, -0.45, 0.75),
                  xyaxes=_camera_xyaxes((0.30, -0.45, 0.75), (0.48, 0.0, 0.37)))
    wb.add_camera(name="cam_side", pos=(1.0, 0.40, 0.80),
                  xyaxes=_camera_xyaxes((1.0, 0.40, 0.80), (0.45, 0.0, 0.37)))

    # Build and attach robot
    panda_xml = str((PANDA_DIR / "panda_nohand.xml").resolve())
    allegro_xml = str((ALLEGRO_DIR / "right_hand.xml").resolve())
    panda_spec = mujoco.MjSpec.from_file(panda_xml)
    allegro_spec = mujoco.MjSpec.from_file(allegro_xml)

    attach_body = panda_spec.body("attachment")
    if attach_body is None:
        raise RuntimeError("Could not find 'attachment' body")
    attach_site = attach_body.first_site()
    if attach_site is None:
        raise RuntimeError("No site on attachment body")
    panda_spec.attach(allegro_spec, prefix=HAND_PREFIX, site=attach_site)

    fingertip_configs = [
        ("hand_ff_proximal", (0.0, 0.0, 0.005), "index"),
        ("hand_mf_proximal", (0.0, 0.0, 0.005), "middle"),
        ("hand_rf_proximal", (0.0, 0.0, 0.005), "ring"),
        ("hand_th_proximal", (0.0, 0.0, 0.005), "thumb"),
    ]
    for body_name, geom_offset, finger_name in fingertip_configs:
        ft_body = panda_spec.body(body_name)
        if ft_body is not None:
            ft_body.add_geom(name=f"{body_name}_collision",
                             type=mujoco.mjtGeom.mjGEOM_SPHERE, size=(0.008,),
                             pos=geom_offset, rgba=(0.9, 0.7, 0.5, 0.3),
                             contype=3, conaffinity=3, condim=6,
                             friction=(0.8, 0.05, 0.005))
            ft_body.add_site(name=f"tactile_site_{finger_name}", pos=geom_offset, size=(0.003,))

    palm_body = panda_spec.body("hand_palm")
    if palm_body is not None:
        palm_body.add_site(name="grasp_center", pos=(0.074, 0.0335, -0.013), size=(0.005,),
                           rgba=(0.1, 0.9, 0.4, 0.6))

    for _, _, finger_name in fingertip_configs:
        fs = scene_spec.add_sensor(f"finger_force_{finger_name}", type=mujoco.mjtSensor.mjSENS_FORCE)
        fs.objtype = mujoco.mjtObj.mjOBJ_SITE
        fs.objname = f"tactile_site_{finger_name}"
        ts = scene_spec.add_sensor(f"finger_torque_{finger_name}", type=mujoco.mjtSensor.mjSENS_TORQUE)
        ts.objtype = mujoco.mjtObj.mjOBJ_SITE
        ts.objname = f"tactile_site_{finger_name}"

    root = wb.add_body(name="robot_root", pos=(0.08, 0.0, 0.0))
    root.add_site(name="robot_attach_site")
    scene_spec.attach(panda_spec, prefix="", site=root.first_site())

    all_joints = list(PANDA_JOINTS + ALLEGRO_JOINTS)
    if bimanual:
        attach_assistant_arm(scene_spec, wb)
        all_joints += ASSISTANT_PANDA_JOINTS
        # Bimanual camera for overhead view of both arms
        scene_spec.worldbody.add_camera(
            name="cam_bimanual",
            pos=(0.45, -1.20, 1.40),
            xyaxes=_camera_xyaxes((0.45, -1.20, 1.40), (0.45, 0.0, 0.35)),
        )
    _add_sensors(scene_spec, all_joints)

    weld = scene_spec.add_equality()
    weld.name = NEEDLE_WELD_NAME
    weld.type = mujoco.mjtEq.mjEQ_WELD
    weld.objtype = mujoco.mjtObj.mjOBJ_BODY
    weld.name1 = "hand_palm"
    weld.name2 = "needle"
    weld.active = False
    weld.solref = [0.02, 1.0]
    weld.solimp = [0.95, 0.99, 0.001, 0.5, 2.0]

    model = scene_spec.compile()
    if model is None:
        raise RuntimeError("Failed to compile scene model")
    _tune_hand_actuators(model)
    return model


def build_scene_model_from_xml(bimanual: bool = False) -> mujoco.MjModel:
    """Build the surgical scene from arsax_scene.xml + Python robot assembly."""
    scene_xml = _HERE / "arsax_scene.xml"
    if not scene_xml.exists():
        return build_scene_model()

    scene_spec = mujoco.MjSpec.from_file(str(scene_xml.resolve()))
    scene_spec.option.timestep = 0.002
    scene_spec.option.gravity = (0.0, 0.0, -9.81)
    scene_spec.option.iterations = 100
    scene_spec.compiler.balanceinertia = 1
    scene_spec.visual.global_.offwidth = 1280
    scene_spec.visual.global_.offheight = 720

    wb = scene_spec.worldbody
    _build_spring_mesh_tissue(wb, scene_spec)

    panda_xml = str((PANDA_DIR / "panda_nohand.xml").resolve())
    allegro_xml = str((ALLEGRO_DIR / "right_hand.xml").resolve())
    panda_spec = mujoco.MjSpec.from_file(panda_xml)
    allegro_spec = mujoco.MjSpec.from_file(allegro_xml)

    attach_body = panda_spec.body("attachment")
    if attach_body is None:
        raise RuntimeError("Could not find 'attachment' body")
    attach_site = attach_body.first_site()
    if attach_site is None:
        raise RuntimeError("No site on attachment body")
    panda_spec.attach(allegro_spec, prefix=HAND_PREFIX, site=attach_site)

    fingertip_configs = [
        ("hand_ff_proximal", (0.0, 0.0, 0.005), "index"),
        ("hand_mf_proximal", (0.0, 0.0, 0.005), "middle"),
        ("hand_rf_proximal", (0.0, 0.0, 0.005), "ring"),
        ("hand_th_proximal", (0.0, 0.0, 0.005), "thumb"),
    ]
    for body_name, geom_offset, finger_name in fingertip_configs:
        ft_body = panda_spec.body(body_name)
        if ft_body is not None:
            ft_body.add_geom(name=f"{body_name}_collision",
                             type=mujoco.mjtGeom.mjGEOM_SPHERE, size=(0.008,),
                             pos=geom_offset, rgba=(0.9, 0.7, 0.5, 0.3),
                             contype=3, conaffinity=3, condim=6,
                             friction=(0.8, 0.05, 0.005))
            ft_body.add_site(name=f"tactile_site_{finger_name}", pos=geom_offset, size=(0.003,))

    palm_body = panda_spec.body("hand_palm")
    if palm_body is not None:
        palm_body.add_site(name="grasp_center", pos=(0.074, 0.0335, -0.013), size=(0.005,),
                           rgba=(0.1, 0.9, 0.4, 0.6))

    for _, _, finger_name in fingertip_configs:
        fs = scene_spec.add_sensor(f"finger_force_{finger_name}", type=mujoco.mjtSensor.mjSENS_FORCE)
        fs.objtype = mujoco.mjtObj.mjOBJ_SITE
        fs.objname = f"tactile_site_{finger_name}"
        ts = scene_spec.add_sensor(f"finger_torque_{finger_name}", type=mujoco.mjtSensor.mjSENS_TORQUE)
        ts.objtype = mujoco.mjtObj.mjOBJ_SITE
        ts.objname = f"tactile_site_{finger_name}"

    robot_root = scene_spec.body("robot_root")
    if robot_root is None:
        robot_root = wb.add_body(name="robot_root", pos=(0.08, 0.0, 0.0))
        robot_root.add_site(name="robot_attach_site")
    root_site = robot_root.first_site()
    if root_site is None:
        robot_root.add_site(name="robot_attach_site")
        root_site = robot_root.first_site()
    scene_spec.attach(panda_spec, prefix="", site=root_site)

    all_joints = list(PANDA_JOINTS + ALLEGRO_JOINTS)
    if bimanual:
        attach_assistant_arm(scene_spec, wb)
        all_joints += ASSISTANT_PANDA_JOINTS
    _add_sensors(scene_spec, all_joints)

    weld = scene_spec.add_equality()
    weld.name = NEEDLE_WELD_NAME
    weld.type = mujoco.mjtEq.mjEQ_WELD
    weld.objtype = mujoco.mjtObj.mjOBJ_BODY
    weld.name1 = "hand_palm"
    weld.name2 = "needle"
    weld.active = False
    weld.solref = [0.02, 1.0]
    weld.solimp = [0.95, 0.99, 0.001, 0.5, 2.0]

    model = scene_spec.compile()
    if model is None:
        raise RuntimeError("Failed to compile scene model from XML")
    _tune_hand_actuators(model)
    return model


def build_combined_model() -> mujoco.MjModel:
    """Build the combined Panda arm + Allegro Hand model only."""
    panda_xml = str((PANDA_DIR / "panda_nohand.xml").resolve())
    allegro_xml = str((ALLEGRO_DIR / "right_hand.xml").resolve())
    panda_spec = mujoco.MjSpec.from_file(panda_xml)
    allegro_spec = mujoco.MjSpec.from_file(allegro_xml)
    attach_body = panda_spec.body("attachment")
    if attach_body is None:
        raise RuntimeError("Could not find 'attachment' body")
    attach_site = attach_body.first_site()
    if attach_site is None:
        raise RuntimeError("No site on attachment body")
    panda_spec.attach(allegro_spec, prefix=HAND_PREFIX, site=attach_site)
    model = panda_spec.compile()
    if model is None:
        raise RuntimeError("Failed to compile combined model")
    return model
