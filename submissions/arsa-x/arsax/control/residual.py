"""Closed-loop residual surgical controller for ARSA-X.

Augments open-loop skill trajectories with real-time sensor-based corrections.
Inspired by residual policy architectures: a base policy (skill sequence)
produces nominal joint targets, and a residual policy computes additive
corrections from sensor feedback.
"""

from collections import deque
from dataclasses import dataclass, field
from typing import Any

import mujoco
import numpy as np

from ..scene.robot import PANDA_JOINTS, ALLEGRO_JOINTS
from ..scene.sensors import SensorSuite


@dataclass
class ResidualState:
    """Internal state for the closed-loop residual controller."""

    servo_error_ema: np.ndarray = field(default_factory=lambda: np.zeros(3))
    grip_error_ema: float = 0.0
    slip_error_ema: float = 0.0
    servo_alpha: float = 0.25
    grip_alpha: float = 0.2
    slip_alpha: float = 0.15
    corrections_applied: int = 0
    total_correction_magnitude: float = 0.0
    correction_log: list[dict] = field(default_factory=list)
    slip_events: int = 0
    slip_recovery_count: int = 0
    _grip_history: deque = field(default_factory=lambda: deque(maxlen=20))
    peak_wrist_force: float = 0.0
    contact_detected: bool = False
    last_contact_timestep: int = 0
    needle_error_history: list[float] = field(default_factory=list)
    raw_needle_errors: list[float] = field(default_factory=list)


class ResidualSurgicalController:
    """Augments surgical skill execution with closed-loop sensor feedback.

    The controller observes sensor state and computes additive corrections
    to joint positions set by the active skill. Corrections are applied on
    top of the skill's nominal joint targets, creating a closed-loop system.
    """

    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        sensors: SensorSuite,
        kp_xyz: tuple[float, float, float] = (1.0, 1.0, 0.7),
        kp_grip: float = 0.5,
        kp_slip: float = 0.7,
        force_threshold_contact: float = 0.5,
        slip_force_threshold: float = 2.0,
        correction_clip: float = 0.07,
    ):
        self.model = model
        self.data = data
        self.sensors = sensors
        self.state = ResidualState()

        self.kp_xyz = np.array(kp_xyz, dtype=float)
        self.kp_grip = kp_grip
        self.kp_slip = kp_slip
        self.force_threshold_contact = force_threshold_contact
        self.slip_force_threshold = slip_force_threshold
        self.correction_clip = correction_clip

        self._joint_addrs: dict[str, int] = {}
        self._actuator_addrs: dict[str, int] = {}
        all_joints = PANDA_JOINTS + ALLEGRO_JOINTS
        for jn in all_joints:
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jn)
            if jid >= 0:
                self._joint_addrs[jn] = int(model.jnt_qposadr[jid])
        for i in range(model.nu):
            act_jid = int(model.actuator_trnid[i][0])
            jname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, act_jid)
            if jname:
                self._actuator_addrs[jname] = i

        self._needle_bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "needle")
        self._pod_target = np.array([0.45, 0.20, 0.40], dtype=float)

        self._adaptive_mode: str = "normal"
        self._consecutive_near_miss: int = 0
        self._fine_stage_active: bool = False
        self._boosted_steps: int = 0
        self._max_boosted_steps: int = 200

    def tick(self, sim_dt: float, skill_name: str = "unknown") -> None:
        """Single-timestep convenience: compute + apply corrections."""
        corrections = self.compute_correction(skill_name)
        if corrections:
            self.apply_corrections(corrections)

    def reset(self) -> None:
        self.state = ResidualState()
        self._adaptive_mode = "normal"
        self._consecutive_near_miss = 0
        self._fine_stage_active = False
        self._boosted_steps = 0

    def compute_correction(
        self, skill_name: str, nominal_joints: dict[str, float] | None = None,
    ) -> dict[str, float]:
        """Compute residual joint corrections from current sensor readings."""
        corrections: dict[str, float] = {}
        wrist_force = self.sensors.wrist_force()
        wrist_force_mag = float(np.linalg.norm(wrist_force)) if wrist_force is not None else 0.0
        self.state.peak_wrist_force = max(self.state.peak_wrist_force, wrist_force_mag)

        if wrist_force_mag > self.force_threshold_contact:
            if not self.state.contact_detected:
                self.state.contact_detected = True
                self.state.last_contact_timestep = self.state.corrections_applied

        if skill_name in ("grasp_needle", "finger_gait"):
            grip_correction = self._compute_grip_residual(wrist_force_mag)
            if grip_correction != 0.0:
                corrections["joint7"] = grip_correction * 0.3
            grip_dist = self.sensors.grasp_force_distribution()
            if grip_dist.get("index", 0) > 0.6:
                corrections["joint7"] = corrections.get("joint7", 0.0) + 0.01
            elif grip_dist.get("thumb", 0) > 0.6:
                corrections["joint7"] = corrections.get("joint7", 0.0) - 0.01

        elif skill_name in ("stabilize_tissue",):
            if wrist_force_mag > 4.0:
                corrections["joint6"] = -0.02 * (wrist_force_mag - 4.0)

        elif skill_name in ("orient_needle", "insert_needle"):
            needle_correction = self._compute_needle_servo_residual()
            corrections.update(needle_correction)
            thumb_force = self.sensors.finger_force("thumb")
            if thumb_force is not None and float(np.linalg.norm(thumb_force)) > 8.0:
                corrections["joint4"] = corrections.get("joint4", 0.0) - 0.008

        elif skill_name in ("pull_suture",):
            if wrist_force_mag > 5.0:
                corrections["joint2"] = -0.01 * (wrist_force_mag - 5.0)

        elif skill_name in ("tie_knot",):
            time_s = float(self.data.time)
            corrections["joint7"] = 0.005 * np.sin(time_s * 4.0)

        for jn in corrections:
            corrections[jn] = float(np.clip(corrections[jn], -self.correction_clip, self.correction_clip))

        if any(abs(v) > 1e-6 for v in corrections.values()):
            self.state.corrections_applied += 1
            mag = float(np.sqrt(sum(v**2 for v in corrections.values())))
            self.state.total_correction_magnitude += mag

        if self._needle_bid >= 0:
            needle_pos = self.data.xpos[self._needle_bid]
            error = float(np.linalg.norm(needle_pos - self._pod_target))
            self.state.needle_error_history.append(error)
            self.state.raw_needle_errors.append(error)

        return corrections

    def apply_corrections(self, corrections: dict[str, float]) -> None:
        for jn, delta in corrections.items():
            if abs(delta) < 1e-8:
                continue
            if jn in self._actuator_addrs:
                act_idx = self._actuator_addrs[jn]
                current = float(self.data.ctrl[act_idx])
                new_val = current + delta
                jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, jn)
                if jid >= 0 and self.model.jnt_limited[jid]:
                    low, high = self.model.jnt_range[jid]
                    new_val = float(np.clip(new_val, low, high))
                self.data.ctrl[act_idx] = new_val
            elif jn in self._joint_addrs:
                addr = self._joint_addrs[jn]
                current = float(self.data.qpos[addr])
                new_val = current + delta
                jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, jn)
                if jid >= 0 and self.model.jnt_limited[jid]:
                    low, high = self.model.jnt_range[jid]
                    new_val = float(np.clip(new_val, low, high))
                self.data.qpos[addr] = new_val

    def detect_slip(self, wrist_force_mag: float) -> bool:
        self.state._grip_history.append(wrist_force_mag)
        if len(self.state._grip_history) < 10:
            return False
        recent_mean = float(np.mean(list(self.state._grip_history)[-5:]))
        prior_mean = float(np.mean(list(self.state._grip_history)[:-5]))
        if prior_mean > 1.0 and recent_mean < prior_mean * 0.7:
            self.state.slip_events += 1
            return True
        return False

    def _compute_grip_residual(self, wrist_force_mag: float) -> float:
        is_slipping = self.detect_slip(wrist_force_mag)
        if is_slipping:
            self.state.slip_recovery_count += 1
            target_grip = min(1.0, wrist_force_mag * 0.5)
            error = target_grip - wrist_force_mag
            self.state.grip_error_ema = (
                self.state.grip_alpha * error
                + (1 - self.state.grip_alpha) * self.state.grip_error_ema
            )
            return float(np.clip(self.state.grip_error_ema * self.kp_grip, -0.1, 0.1))
        self.state.grip_error_ema *= 0.95
        return 0.0

    def _compute_needle_servo_residual(self) -> dict[str, float]:
        if self._needle_bid < 0:
            return {}
        needle_pos = self.data.xpos[self._needle_bid]
        error = self._pod_target - needle_pos
        error_mag = float(np.linalg.norm(error))

        wrist_force = self.sensors.wrist_force()
        wrist_force_mag = float(np.linalg.norm(wrist_force)) if wrist_force is not None else 0.0
        self._update_adaptive_gains(error_mag, wrist_force_mag)

        threshold = 0.003 if self._fine_stage_active else 0.005
        if error_mag < threshold:
            return {}

        ema_alpha = 0.35 if self._fine_stage_active else self.state.servo_alpha
        self.state.servo_error_ema = (
            ema_alpha * error + (1 - ema_alpha) * self.state.servo_error_ema
        )

        raw = self.state.servo_error_ema * self.kp_xyz
        corrections: dict[str, float] = {}
        corrections["joint2"] = float(raw[2]) * 0.4
        corrections["joint4"] = float(raw[0]) * 0.35
        corrections["joint1"] = float(raw[1]) * 0.25

        j6_scale = 0.20 if self._fine_stage_active else 0.12
        j6_raw = float(raw[2]) * j6_scale + float(raw[0]) * (0.10 if self._fine_stage_active else 0.08)
        corrections["joint6"] = float(np.clip(j6_raw, -self.correction_clip * 0.6, self.correction_clip * 0.6))

        if self._adaptive_mode == "boosted" and error_mag > 0.008:
            j5_ff = float(raw[1]) * 0.10
            corrections["joint5"] = float(np.clip(j5_ff, -self.correction_clip * 0.4, self.correction_clip * 0.4))

        return corrections

    def _update_adaptive_gains(self, error_mag: float, wrist_force_mag: float) -> None:
        if error_mag > 0.015:
            self._adaptive_mode = "normal"
            self.kp_xyz[:] = [1.0, 1.0, 0.7]
            self.correction_clip = 0.07
            self._fine_stage_active = False
            self._boosted_steps = 0
        elif error_mag > 0.005:
            self._boosted_steps += 1
            if self._boosted_steps > self._max_boosted_steps:
                self._adaptive_mode = "normal"
                self.kp_xyz[:] = [1.0, 1.0, 0.7]
                self.correction_clip = 0.07
                self._fine_stage_active = False
            else:
                self._adaptive_mode = "boosted"
                self.kp_xyz[:] = [1.4, 1.4, 1.0]
                self.correction_clip = 0.09
                self._fine_stage_active = False
        else:
            self._adaptive_mode = "fine"
            self.kp_xyz[:] = [1.2, 1.2, 0.9]
            self.correction_clip = 0.06
            self._fine_stage_active = True
            self._boosted_steps = 0
        if wrist_force_mag > 3.0 and error_mag > 0.008:
            self.kp_xyz *= 1.15
            self.correction_clip = min(self.correction_clip * 1.2, 0.095)

    def metrics(self) -> dict[str, Any]:
        needle_errors = self.state.needle_error_history
        error_reduction = 0.0
        if len(needle_errors) >= 10:
            early = float(np.mean(needle_errors[:5]))
            late = float(np.mean(needle_errors[-5:]))
            if early > 1e-6:
                error_reduction = 100.0 * (early - late) / early
        return {
            "corrections_applied": self.state.corrections_applied,
            "total_correction_magnitude": round(self.state.total_correction_magnitude, 4),
            "slip_events": self.state.slip_events,
            "slip_recoveries": self.state.slip_recovery_count,
            "contact_detected": self.state.contact_detected,
            "peak_wrist_force_n": round(self.state.peak_wrist_force, 4),
            "mean_needle_error_m": round(float(np.mean(needle_errors)), 5) if needle_errors else 0.0,
            "median_needle_error_m": round(float(np.median(needle_errors)), 5) if needle_errors else 0.0,
            "needle_error_reduction_pct": round(error_reduction, 2),
            "policy_type": "closed-loop residual",
            "controller_gains": {
                "kp_xyz": [float(v) for v in self.kp_xyz],
                "kp_grip": self.kp_grip,
                "kp_slip": self.kp_slip,
                "correction_clip": self.correction_clip,
            },
        }
