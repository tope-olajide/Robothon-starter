"""Orient the surgical needle to the correct insertion angle."""

import numpy as np
from .base import SkillBase, SkillResult


class OrientNeedle(SkillBase):
    """Orient the grasped needle to a target angle for tissue insertion.

    Uses wrist rotation (joint7) and forearm rotation (joint5) to achieve
    the desired needle tip orientation, while maintaining finger pinch.
    """

    def initialize(self, target_angle_deg: float = 45.0, **kwargs) -> None:
        super().initialize(**kwargs)
        self._target_angle = np.radians(target_angle_deg)
        self._start_j7 = self._get_joint("joint7")
        self._start_j5 = self._get_joint("joint5")

    def on_tick(self, dt: float) -> SkillResult:
        p = self._progress
        target_j7 = self._start_j7 - self._target_angle * 0.7
        self.lerp_joint("joint7", self._start_j7, target_j7)
        target_j5 = self._start_j5 + self._target_angle * 0.3
        self.lerp_joint("joint5", self._start_j5, target_j5)

        for jn, val in {"hand_ffj1": 0.8, "hand_ffj2": 0.8, "hand_ffj3": 0.8,
                         "hand_thj1": 0.6, "hand_thj2": 0.8, "hand_thj3": 1.0}.items():
            self.set_joint(jn, val)

        if self._progress >= 1.0:
            return SkillResult.success(f"Needle oriented to {np.degrees(self._target_angle):.0f}°")
        return SkillResult.running()
