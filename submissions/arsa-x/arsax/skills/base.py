"""Base class for all surgical skills — shared infrastructure for joint-space control."""

from enum import Enum, auto

import mujoco
import numpy as np


class SkillStatus(Enum):
    IDLE = auto()
    RUNNING = auto()
    SUCCESS = auto()
    FAILED = auto()


class SkillResult:
    """Outcome of a skill execution."""

    def __init__(self, status: SkillStatus, message: str = "", data: dict | None = None):
        self.status = status
        self.message = message
        self.data = data or {}

    @classmethod
    def success(cls, message: str = "", **data) -> "SkillResult":
        return cls(SkillStatus.SUCCESS, message, data)

    @classmethod
    def failed(cls, message: str = "", **data) -> "SkillResult":
        return cls(SkillStatus.FAILED, message, data)

    @classmethod
    def running(cls, message: str = "") -> "SkillResult":
        return cls(SkillStatus.RUNNING, message)


class SkillBase:
    """Abstract base for a parameterised, reusable surgical skill.

    Each skill is a deterministic MuJoCo control policy operating on
    the combined Panda + Allegro Hand robot (23 DOF).
    """

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData, name: str = ""):
        self.model = model
        self.data = data
        self.name = name or self.__class__.__name__
        self.status = SkillStatus.IDLE
        self._progress = 0.0
        self._time_elapsed = 0.0
        self._duration = 3.0
        self._params: dict = {}

    def can_run(self) -> bool:
        return True

    def initialize(self, **params) -> None:
        self._params = dict(params)
        self._duration = float(params.get("duration", self._duration))
        self.status = SkillStatus.RUNNING
        self._progress = 0.0
        self._time_elapsed = 0.0

    def tick(self, dt: float) -> SkillResult:
        self._time_elapsed += dt
        self._progress = min(1.0, self._time_elapsed / max(self._duration, 1e-6))
        if self._progress >= 1.0:
            return self.on_finish()
        return self.on_tick(dt)

    def reset(self) -> None:
        self.status = SkillStatus.IDLE
        self._progress = 0.0
        self._time_elapsed = 0.0

    def on_tick(self, dt: float) -> SkillResult:
        return SkillResult.running()

    def on_finish(self) -> SkillResult:
        return SkillResult.success(f"{self.name} completed")

    @property
    def _eased(self) -> float:
        """Smoothstep easing for natural acceleration/deceleration."""
        p = self._progress
        return p * p * (3.0 - 2.0 * p)

    def set_joint(self, name: str, value: float) -> None:
        jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if jid < 0:
            return
        for i in range(self.model.nu):
            act_jid = int(self.model.actuator_trnid[i][0])
            if act_jid == jid:
                self.data.ctrl[i] = float(value)
                return
        qpos_addr = int(self.model.jnt_qposadr[jid])
        if self.model.jnt_limited[jid]:
            low, high = self.model.jnt_range[jid]
            value = float(np.clip(value, low, high))
        self.data.qpos[qpos_addr] = value

    def set_joints(self, joint_values: dict[str, float]) -> None:
        for name, val in joint_values.items():
            self.set_joint(name, val)

    def lerp(self, a: float, b: float) -> float:
        return a + (b - a) * self._eased

    def lerp_joint(self, name: str, start: float, end: float) -> None:
        self.set_joint(name, self.lerp(start, end))

    def lerp_joints(self, start: dict[str, float], end: dict[str, float]) -> None:
        eased = self._eased
        for name in start:
            if name in end:
                val = start[name] + (end[name] - start[name]) * eased
                self.set_joint(name, val)
            else:
                self.set_joint(name, start[name])

    def _get_joint(self, name: str) -> float:
        jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if jid < 0:
            return 0.0
        return float(self.data.qpos[int(self.model.jnt_qposadr[jid])])

    def body_pos(self, name: str) -> np.ndarray:
        bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)
        if bid < 0:
            return np.zeros(3)
        return self.data.xpos[bid].copy()

    def smoothstep(self, edge0: float, edge1: float, value: float) -> float:
        if value <= edge0:
            return 0.0
        if value >= edge1:
            return 1.0
        x = (value - edge0) / max(edge1 - edge0, 1e-6)
        return x * x * (3.0 - 2.0 * x)
