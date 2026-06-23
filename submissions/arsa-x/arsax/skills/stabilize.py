"""Stabilize tissue near the entry point."""

from .base import SkillBase, SkillResult


class StabilizeTissue(SkillBase):
    """Hold the hand steady near the tissue surface to prevent movement."""

    def initialize(self, **kwargs) -> None:
        super().initialize(**kwargs)
        self._start_j4 = self._get_joint("joint4")

    def on_tick(self, dt: float) -> SkillResult:
        if self._progress >= 1.0:
            return SkillResult.success("Tissue stabilized")
        return SkillResult.running()
