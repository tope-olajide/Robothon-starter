"""Regrasp the surgical needle when grasp is lost or suboptimal."""

from ..scene.robot import ALLEGRO_OPEN, ALLEGRO_PINCH
from .base import SkillBase, SkillResult


class RegraspNeedle(SkillBase):
    """Release and re-grasp the needle at a better position/orientation.

    Phases: release → reposition → regrasp.
    """

    def initialize(self, **kwargs) -> None:
        super().initialize(**kwargs)
        self._phase = "release"
        self._phase_progress = 0.0
        self._record_current_joints()

    def _record_current_joints(self) -> None:
        self._start_j7 = self._get_joint("joint7")

    def on_tick(self, dt: float) -> SkillResult:
        phase_dur = self._duration / 3.0
        self._phase_progress += dt / max(phase_dur, 1e-6)
        p = min(self._phase_progress, 1.0)

        if self._phase == "release":
            for jn in ALLEGRO_OPEN:
                current_pos = self._get_joint(jn)
                target = ALLEGRO_OPEN.get(jn, 0.0)
                self.set_joint(jn, current_pos + (target - current_pos) * p)
            if self._phase_progress >= 1.0:
                self._phase = "reposition"
                self._phase_progress = 0.0
                self._record_current_joints()
        elif self._phase == "reposition":
            self.lerp_joint("joint7", self._start_j7, self._start_j7 + 0.15 * p)
            if self._phase_progress >= 1.0:
                self._phase = "regrasp"
                self._phase_progress = 0.0
                self._record_current_joints()
        elif self._phase == "regrasp":
            for jn in ALLEGRO_PINCH:
                current_pos = self._get_joint(jn)
                target = ALLEGRO_PINCH.get(jn, 0.0)
                self.set_joint(jn, current_pos + (target - current_pos) * p)
            if self._phase_progress >= 1.0:
                return SkillResult.success("Needle regrasped successfully")

        return SkillResult.running(f"regrasp {self._phase} {p:.2f}")
