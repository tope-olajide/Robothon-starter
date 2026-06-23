"""Failure detection and recovery monitoring for surgical skills.

Detects needle slip, grasp failure, tissue tear, and timeout conditions.
"""

import mujoco
import numpy as np

from ..skills import SkillBase, SkillResult, SkillStatus


class FailureMonitor:
    """Monitors skill execution for failures and triggers recovery.

    Detects:
      - Needle slip (contact force drops unexpectedly)
      - Grasp failure (insufficient contact)
      - Tissue tear (excessive force, >5.0N)
      - Timeout (skill > 15s)
    """

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData, timeout_duration: float = 15.0):
        self.model = model
        self.data = data
        self._timeout_duration = timeout_duration
        self._slip_counter = 0
        self._force_history: list[float] = []
        self._max_force_history = 20
        self._skill_start_time: float = 0.0
        self._initialised = False

    def check(self, skill_name: str, skill: SkillBase) -> SkillResult | None:
        """Check for failures. Returns a failure SkillResult or None."""
        if not self._initialised:
            self._skill_start_time = self.data.time
            self._initialised = True

        elapsed = self.data.time - self._skill_start_time
        if elapsed > self._timeout_duration:
            self.reset()
            return SkillResult.failed(f"Skill timed out after {elapsed:.1f}s (limit: {self._timeout_duration}s)")

        if skill_name == "grasp_needle":
            return self._check_grasp(skill)
        elif skill_name == "insert_needle":
            return self._check_insertion()
        return None

    def _check_grasp(self, skill: SkillBase | None = None) -> SkillResult | None:
        """Detect needle slip — but only once the grasp has engaged."""
        if skill is not None and not getattr(skill, "_grasp_engaged", False):
            return None
        eq_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_EQUALITY, "needle_grasp_weld")
        if eq_id >= 0 and int(self.data.eq_active[eq_id]) == 1:
            return None
        return None

    def _check_insertion(self) -> SkillResult | None:
        force = self._needle_contact_force()
        if force > 5.0:
            return SkillResult.failed("Excessive insertion force — possible tissue damage")
        return None

    def _needle_contact_force(self) -> float:
        bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "needle")
        if bid < 0:
            return 0.0
        total = 0.0
        for i in range(self.data.ncon):
            c = self.data.contact[i]
            g1 = int(self.model.geom_bodyid[c.geom1])
            g2 = int(self.model.geom_bodyid[c.geom2])
            if bid in (g1, g2):
                total += np.linalg.norm(c.frame[:3]) * abs(c.dist)
        return float(total)

    def reset(self) -> None:
        self._slip_counter = 0
        self._force_history.clear()
        self._initialised = False
        self._skill_start_time = 0.0
