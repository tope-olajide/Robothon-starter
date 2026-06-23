"""Finger gaiting skill for in-hand needle reorientation.

Implements coordinated multi-finger gaiting — alternating contact between
finger pairs to rotate an object in-hand without releasing it.
"""

import numpy as np

from .base import SkillBase, SkillResult

_GAIT_PHASE_A = {
    "hand_ffj0": 0.0, "hand_ffj1": 0.9, "hand_ffj2": 0.9, "hand_ffj3": 0.9,
    "hand_mfj0": 0.0, "hand_mfj1": 0.1, "hand_mfj2": 0.1, "hand_mfj3": 0.1,
    "hand_rfj0": 0.0, "hand_rfj1": 0.1, "hand_rfj2": 0.1, "hand_rfj3": 0.1,
    "hand_thj0": 0.7, "hand_thj1": 0.7, "hand_thj2": 0.8, "hand_thj3": 0.9,
}

_GAIT_PHASE_B = {
    "hand_ffj0": 0.0, "hand_ffj1": 0.1, "hand_ffj2": 0.1, "hand_ffj3": 0.1,
    "hand_mfj0": 0.0, "hand_mfj1": 0.9, "hand_mfj2": 0.9, "hand_mfj3": 0.9,
    "hand_rfj0": 0.0, "hand_rfj1": 0.8, "hand_rfj2": 0.8, "hand_rfj3": 0.8,
    "hand_thj0": 0.7, "hand_thj1": 0.7, "hand_thj2": 0.8, "hand_thj3": 0.9,
}


class FingerGait(SkillBase):
    """Rotate the surgical needle in-hand using coordinated finger gaits.

    Performs N gait cycles, each alternating between Phase A (index+thumb hold,
    middle+ring advance) and Phase B (middle+ring hold, index advance).
    """

    def initialize(self, rotation_rad: float = np.pi / 6, n_cycles: int = 3, **kwargs) -> None:
        super().initialize(**kwargs)
        self._target_rotation = rotation_rad
        self._n_cycles = max(1, n_cycles)
        self._phase = "A"
        self._phase_progress = 0.0
        self._cycle_count = 0
        self._start_j7 = self._get_joint("joint7")

    def on_tick(self, dt: float) -> SkillResult:
        total_phases = self._n_cycles * 2
        phase_duration = self._duration / max(total_phases, 1)
        self._phase_progress += dt / max(phase_duration, 1e-6)
        p = min(self._phase_progress, 1.0)

        if self._phase == "A":
            self._apply_phase(_GAIT_PHASE_A, p)
        else:
            self._apply_phase(_GAIT_PHASE_B, p)

        cycle_progress = (self._cycle_count + (0.0 if self._phase == "A" else 0.5))
        total_progress = cycle_progress / max(total_phases, 1)
        wrist_target = self._start_j7 + self._target_rotation * 0.3 * total_progress
        self.set_joint("joint7", wrist_target)

        if self._phase_progress >= 1.0:
            if self._phase == "A":
                self._phase = "B"
                self._phase_progress = 0.0
            else:
                self._phase = "A"
                self._phase_progress = 0.0
                self._cycle_count += 1

        if self._cycle_count >= self._n_cycles:
            for jn, val in _GAIT_PHASE_A.items():
                self.set_joint(jn, val)
            return SkillResult.success(
                f"Completed {self._n_cycles} gait cycles, "
                f"rotated {np.degrees(self._target_rotation):.0f} degrees"
            )
        return SkillResult.running(
            f"gait cycle {self._cycle_count + 1}/{self._n_cycles} phase {self._phase}"
        )

    def _apply_phase(self, target_pose: dict[str, float], p: float) -> None:
        from ..scene.robot import ALLEGRO_JOINTS
        for jn in ALLEGRO_JOINTS:
            target = target_pose.get(jn)
            if target is None:
                continue
            current = self._get_joint(jn)
            self.set_joint(jn, current + (target - current) * p)
