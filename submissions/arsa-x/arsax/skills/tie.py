"""Tie a surgical knot using the Allegro Hand."""

from .base import SkillBase, SkillResult


class TieKnot(SkillBase):
    """Tie a surgical knot by coordinated arm + hand motion.

    Phases: wrap → pull → tighten. Simulates a simplified instrument-tie.
    """

    def initialize(self, **kwargs) -> None:
        super().initialize(**kwargs)
        self._phase = "wrap"
        self._phase_progress = 0.0
        self._record_phase_starts()

    def _record_phase_starts(self) -> None:
        self._start_j5 = self._get_joint("joint5")
        self._start_j6 = self._get_joint("joint6")
        self._start_j7 = self._get_joint("joint7")

    def on_tick(self, dt: float) -> SkillResult:
        phase_dur = self._duration / 3.0
        self._phase_progress += dt / max(phase_dur, 1e-6)
        p = min(self._phase_progress, 1.0)

        for jn, val in {"hand_ffj1": 0.6, "hand_ffj2": 0.6, "hand_ffj3": 0.5,
                         "hand_mfj1": 0.8, "hand_mfj2": 0.8, "hand_mfj3": 0.6,
                         "hand_rfj1": 0.8, "hand_rfj2": 0.8, "hand_rfj3": 0.6,
                         "hand_thj1": 0.5, "hand_thj2": 0.7, "hand_thj3": 0.9}.items():
            self.set_joint(jn, val)

        if self._phase == "wrap":
            self.lerp_joint("joint5", self._start_j5, self._start_j5 + 0.5 * p)
            self.lerp_joint("joint6", self._start_j6, self._start_j6 + 0.3 * p)
            if self._phase_progress >= 1.0:
                self._phase = "pull"
                self._phase_progress = 0.0
                self._record_phase_starts()
        elif self._phase == "pull":
            self.lerp_joint("joint6", self._start_j6, self._start_j6 - 0.5 * p)
            self.lerp_joint("joint7", self._start_j7, self._start_j7 + 0.4 * p)
            for jn, val in {"hand_ffj1": 0.7, "hand_ffj2": 0.7, "hand_ffj3": 0.6,
                             "hand_thj1": 0.6, "hand_thj2": 0.8, "hand_thj3": 1.0}.items():
                self.set_joint(jn, val)
            if self._phase_progress >= 1.0:
                self._phase = "tighten"
                self._phase_progress = 0.0
                self._record_phase_starts()
        elif self._phase == "tighten":
            self.lerp_joint("joint5", self._start_j5, self._start_j5 + 0.2 * p)
            if self._phase_progress >= 1.0:
                return SkillResult.success("Knot tied securely")

        return SkillResult.running(f"{self._phase} {self._phase_progress:.2f}")
