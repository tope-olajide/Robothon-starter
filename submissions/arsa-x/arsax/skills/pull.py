"""Pull the suture thread through the tissue after needle insertion."""

from .base import SkillBase, SkillResult


class PullSuture(SkillBase):
    """Pull the suture thread to advance it through the tissue track."""

    def initialize(self, pull_distance: float = 0.04, **kwargs) -> None:
        super().initialize(**kwargs)
        self._pull_distance = pull_distance
        self._start_j4 = self._get_joint("joint4")
        self._start_j7 = self._get_joint("joint7")

    def on_tick(self, dt: float) -> SkillResult:
        p = self._progress
        self.lerp_joint("joint4", self._start_j4, self._start_j4 - 0.325 * p)
        self.lerp_joint("joint7", self._start_j7, self._start_j7 + 0.3 * p)

        for jn, val in {"hand_ffj1": 0.8, "hand_ffj2": 0.8, "hand_ffj3": 0.8,
                         "hand_thj1": 0.6, "hand_thj2": 0.8, "hand_thj3": 1.0}.items():
            self.set_joint(jn, val)

        if p >= 1.0:
            return SkillResult.success("Suture pulled through tissue")
        return SkillResult.running()
