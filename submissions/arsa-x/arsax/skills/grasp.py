"""Grasp the surgical needle with the Allegro hand + activated weld.

Enhanced with:
- Force-guided descent: monitors wrist F/T during approach and slows/stops
  when excessive contact force is detected, preventing table collision.
- Adaptive position offset: offsets approach behind the needle's long axis.
"""

import mujoco
import numpy as np

from ..scene.robot import PANDA_JOINTS, ALLEGRO_JOINTS, ALLEGRO_OPEN, ALLEGRO_CLOSE, activate_needle_weld
from .base import SkillBase, SkillResult

_SEED = {"joint1": 0.0, "joint2": -0.35, "joint3": 0.0,
         "joint4": -2.30, "joint5": -0.50, "joint6": 2.10, "joint7": 0.6}

# Home phase: arm first lifts to 25cm above needle before approaching,
# preventing the arm from sweeping through the needle during transit.
_HOME_S = 1.5       # Time to move to safe home position first
_APPROACH_S = 1.2
_DESCEND_S = 0.9
_CLOSE_S = 1.4
_LIFT_S = 1.1

# Force-guided descent parameters
_FORCE_LIMIT_N = 3.0       # wrist force threshold to trigger descent slowdown (N)
_FORCE_STOP_N = 6.0        # wrist force threshold to halt descent entirely (N)
_FORCE_SENSOR_NAME = "sensor_wrist_force"


class GraspNeedle(SkillBase):
    """Grasp the surgical needle with the Allegro hand + activated weld.

    Phases: approach → descend → close → lift. On close, once finger-needle
    contact is detected (or on a stub model), the activated grasp weld engages.

    Force-guided descent: during descend, reads wrist F/T sensor. If contact
    force exceeds _FORCE_LIMIT_N, descent speed is proportionally reduced.
    If it exceeds _FORCE_STOP_N, descent halts to prevent table collision.

    Adaptive position offset: approaches slightly behind the needle's long axis
    based on its body orientation, improving approach geometry.
    """

    def initialize(self, target_pos=None, **kwargs) -> None:
        super().initialize(**kwargs)
        self._duration = 60.0

        from ..control.ik import ArmIK
        self._ik = ArmIK(self.model, self.data, "grasp_center", damping=0.05)
        self._needle_bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "needle")
        self._sid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "grasp_center")
        self._wrist_force_sensor_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_SENSOR, _FORCE_SENSOR_NAME
        )

        # Use the proven seed-site orientation for IK (the adaptive quaternion
        # was calibrated against a different frame and caused finger misalignment).
        # The adaptive *position offset* is still used for approach geometry.
        self._down_quat = self._seed_site_quat()

        needle = self.data.xpos[self._needle_bid].copy()
        self._needle0 = needle

        # Compute approach offset aligned with needle orientation
        approach_offset = self._compute_approach_offset()

        # Solve IK for all waypoints: home → approach → descend → lift
        # Home position is high above workspace to avoid needle collision during transit
        self._home_q = self._ik.solve_qpos(
            needle + [0.0, 0.0, 0.25], self._down_quat, seed=_SEED, iters=600, rot_gain=0.35)
        self._approach_q = self._ik.solve_qpos(
            needle + approach_offset, self._down_quat, seed=self._home_q, iters=600, rot_gain=0.35)
        self._descend_q = self._ik.solve_qpos(
            needle, self._down_quat, seed=self._approach_q, iters=600, rot_gain=0.35)
        self._lift_q = self._ik.solve_qpos(
            needle + [0.0, 0.0, 0.12], self._down_quat, seed=self._descend_q, iters=600, rot_gain=0.35)

        self._phase = "home"  # Start with safe home position first
        self._phase_t = 0.0
        self._grasp_engaged = False
        self._has_actuators = self.model.nu > 0
        # Force-guided descent parameters
        self._force_halt = False          # True if descent was halted by force limit
        self._force_slowdown = 0.0        # [0,1] fraction of speed reduction from force
        self._last_ramp_p = 0.0           # frozen progress when force_halt is True
        self._capture_ctrl()

    def _compute_approach_offset(self) -> np.ndarray:
        """Compute a 3D offset for the approach position, offsetting slightly
        behind the needle's long axis so the hand approaches from behind."""
        if self._needle_bid < 0:
            return np.array([0.0, 0.0, 0.08])

        needle_mat = self.data.xmat[self._needle_bid].reshape(3, 3)
        needle_long_axis = needle_mat[:, 0]

        # Approach from 8cm above, offset 2cm behind the needle's long axis
        # (negative long axis direction = "behind")
        offset = np.array([0.0, 0.0, 0.08]) - 0.02 * needle_long_axis
        return offset

    def _seed_site_quat(self) -> np.ndarray:
        scratch = mujoco.MjData(self.model)
        scratch.qpos[:] = self.data.qpos
        for jn, v in _SEED.items():
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, jn)
            if jid >= 0:
                scratch.qpos[int(self.model.jnt_qposadr[jid])] = v
        mujoco.mj_forward(self.model, scratch)
        q = np.zeros(4)
        mujoco.mju_mat2Quat(q, scratch.site_xmat[self._sid])
        return q

    def _capture_ctrl(self) -> None:
        self._start_ctrl = {}
        for jn in PANDA_JOINTS + ALLEGRO_JOINTS:
            self._start_ctrl[jn] = self._ctrl_of(jn)

    def _ctrl_of(self, jn: str) -> float:
        for i in range(self.model.nu):
            if int(self.model.actuator_trnid[i][0]) == mujoco.mj_name2id(
                    self.model, mujoco.mjtObj.mjOBJ_JOINT, jn):
                return float(self.data.ctrl[i])
        return 0.0

    def _set_phase(self, name: str) -> None:
        self._phase = name
        self._phase_t = 0.0
        self._last_ramp_p = 0.0
        self._force_halt = False
        self._force_slowdown = 0.0
        self._capture_ctrl()

    def _ramp(self, arm_q: dict | None, hand_pose: dict | None, dur: float) -> float:
        p = min(1.0, self._phase_t / max(dur, 1e-6))
        s = p * p * (3.0 - 2.0 * p)
        if arm_q is not None:
            for jn in PANDA_JOINTS:
                a = self._start_ctrl.get(jn, 0.0)
                self.set_joint(jn, a + (arm_q[jn] - a) * s)
        if hand_pose is not None:
            for jn in ALLEGRO_JOINTS:
                a = self._start_ctrl.get(jn, 0.0)
                self.set_joint(jn, a + (hand_pose.get(jn, 0.0) - a) * s)
        return p

    def _ramp_force_guided(self, arm_q: dict | None, hand_pose: dict | None, dur: float) -> float:
        """Like _ramp, but proportionally slows interpolation based on wrist force.

        When wrist force exceeds _FORCE_LIMIT_N, the effective time is stretched
        so the arm moves slower.  At _FORCE_STOP_N the arm freezes.
        """
        # Read wrist force magnitude
        wrist_force_mag = self._read_wrist_force_mag()

        # Compute force-based slowdown factor: 1.0 = full speed, 0.0 = frozen
        if wrist_force_mag > _FORCE_STOP_N:
            self._force_halt = True
            self._force_slowdown = 1.0
            # Freeze at the last known progress so the arm stops moving
            p = self._last_ramp_p
            s = p * p * (3.0 - 2.0 * p)
            if arm_q is not None:
                for jn in PANDA_JOINTS:
                    a = self._start_ctrl.get(jn, 0.0)
                    self.set_joint(jn, a + (arm_q[jn] - a) * s)
            if hand_pose is not None:
                for jn in ALLEGRO_JOINTS:
                    a = self._start_ctrl.get(jn, 0.0)
                    self.set_joint(jn, a + (hand_pose.get(jn, 0.0) - a) * s)
            return p
        elif wrist_force_mag > _FORCE_LIMIT_N:
            # Linear interpolation: at _FORCE_LIMIT_N → factor 1.0, at _FORCE_STOP_N → factor 0.0
            fraction = (wrist_force_mag - _FORCE_LIMIT_N) / max(_FORCE_STOP_N - _FORCE_LIMIT_N, 1e-6)
            self._force_slowdown = fraction
            self._force_halt = False
        else:
            self._force_slowdown = 0.0
            self._force_halt = False

        # Stretch effective time: if slowdown=0.5, the arm moves at half speed
        effective_t = self._phase_t * (1.0 - self._force_slowdown * 0.8)
        p = min(1.0, effective_t / max(dur, 1e-6))
        self._last_ramp_p = p  # remember for potential force-halt freeze
        s = p * p * (3.0 - 2.0 * p)
        if arm_q is not None:
            for jn in PANDA_JOINTS:
                a = self._start_ctrl.get(jn, 0.0)
                self.set_joint(jn, a + (arm_q[jn] - a) * s)
        if hand_pose is not None:
            for jn in ALLEGRO_JOINTS:
                a = self._start_ctrl.get(jn, 0.0)
                self.set_joint(jn, a + (hand_pose.get(jn, 0.0) - a) * s)
        return p

    def _read_wrist_force_mag(self) -> float:
        """Read wrist F/T sensor magnitude scaled to display Newtons (N).

        Returns 0.0 if sensor unavailable.  Raw MuJoCo values are in
        hundred-to-thousand Newton range; we scale by FORCE_SCALE (0.01)
        to match the display-unit thresholds used throughout the overlay
        and the residual controller.
        """
        if self._wrist_force_sensor_id < 0:
            return 0.0
        adr = int(self.model.sensor_adr[self._wrist_force_sensor_id])
        dim = int(self.model.sensor_dim[self._wrist_force_sensor_id])
        if dim >= 3 and adr + 3 <= len(self.data.sensordata):
            f = self.data.sensordata[adr:adr + 3]
            return float(np.linalg.norm(f)) * 0.01  # FORCE_SCALE
        return 0.0

    def _finger_contact_force(self) -> float:
        f6 = np.zeros(6)
        total = 0.0
        for i in range(self.data.ncon):
            c = self.data.contact[i]
            g1 = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, c.geom1) or ""
            g2 = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, c.geom2) or ""
            if "needle" not in g1 and "needle" not in g2:
                continue
            other = g2 if "needle" in g1 else g1
            if any(k in other for k in ("stand", "socket", "table", "tissue", "needle")):
                continue
            mujoco.mj_contactForce(self.model, self.data, i, f6)
            total += abs(float(f6[0]))
        return total

    def on_tick(self, dt: float) -> SkillResult:
        self._phase_t += dt

        if self._phase == "home":
            # Phase 0: Move to safe home position high above workspace
            # This prevents the arm from sweeping through the needle during transit
            self._ramp_force_guided(self._home_q, ALLEGRO_OPEN, _HOME_S)
            if self._phase_t >= _HOME_S:
                self._set_phase("approach")
            return SkillResult.running("home")

        if self._phase == "approach":
            # Use force-guided approach to prevent collision
            self._ramp_force_guided(self._approach_q, ALLEGRO_OPEN, _APPROACH_S)
            if self._phase_t >= _APPROACH_S:
                self._set_phase("descend")
            return SkillResult.running("approach")

        if self._phase == "descend":
            # Force-guided descent: slows when contact detected
            self._ramp_force_guided(self._descend_q, ALLEGRO_OPEN, _DESCEND_S)
            if self._phase_t >= _DESCEND_S or self._force_halt:
                # If force halted descent, skip to close anyway — we're close enough
                self._set_phase("close")
            return SkillResult.running("descend")

        if self._phase == "close":
            for jn in PANDA_JOINTS:
                self.set_joint(jn, self._descend_q[jn])
            self._ramp(None, ALLEGRO_CLOSE, _CLOSE_S)
            if self._phase_t >= _CLOSE_S:
                if not self._has_actuators or self._finger_contact_force() > 0.05:
                    if activate_needle_weld(self.model, self.data):
                        self._grasp_engaged = True
                self._set_phase("lift")
            return SkillResult.running("close")

        if self._phase == "lift":
            for jn in ALLEGRO_JOINTS:
                self.set_joint(jn, ALLEGRO_CLOSE.get(jn, 0.0))
            self._ramp(self._lift_q, ALLEGRO_CLOSE, _LIFT_S)
            if self._phase_t >= _LIFT_S:
                if self._grasp_engaged:
                    return SkillResult.success("Needle grasped, welded and lifted")
                return SkillResult.failed("Grasp closed but no needle contact detected")
            return SkillResult.running("lift")

        return SkillResult.running(self._phase)
