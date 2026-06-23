"""Drive the needle tip through the tissue phantom along a curved trajectory."""

from .base import SkillBase, SkillResult


class InsertNeedle(SkillBase):
    """Drive the needle tip through the tissue phantom along a curved trajectory.

    Three-phase insertion: approach tissue surface (0–40%), drive through (40–80%),
    exit and clear (80–100%).
    """

    _J4_OFFSET = 0.073
    _J6_OFFSET = 0.650

    def initialize(self, **kwargs) -> None:
        super().initialize(**kwargs)
        self._entry = [0.42, -0.02, 0.39]
        self._exit = [0.45, 0.02, 0.37]
        self._start_j4 = self._get_joint("joint4")
        self._start_j5 = self._get_joint("joint5")
        self._start_j6 = self._get_joint("joint6")
        self._start_j7 = self._get_joint("joint7")

    def on_tick(self, dt: float) -> SkillResult:
        p = self._progress

        for jn, val in {"hand_ffj1": 0.85, "hand_ffj2": 0.85, "hand_ffj3": 0.85,
                         "hand_thj1": 0.65, "hand_thj2": 0.85, "hand_thj3": 1.0}.items():
            self.set_joint(jn, val)

        j4_end = self._start_j4 + self._J4_OFFSET
        j6_end = self._start_j6 + self._J6_OFFSET

        if p < 0.4:
            local_p = p / 0.4
            self.lerp_joint("joint4", self._start_j4, self._start_j4 + self._J4_OFFSET * 0.4 * local_p)
            self.lerp_joint("joint6", self._start_j6, self._start_j6 + self._J6_OFFSET * 0.3 * local_p)
        elif p < 0.8:
            local_p = (p - 0.4) / 0.4
            self.lerp_joint("joint4", self._start_j4 + self._J4_OFFSET * 0.4,
                            self._start_j4 + self._J4_OFFSET * 0.8 * local_p)
            self.lerp_joint("joint6", self._start_j6 + self._J6_OFFSET * 0.3,
                            self._start_j6 + self._J6_OFFSET * 0.7 * local_p)
        else:
            local_p = (p - 0.8) / 0.2
            self.lerp_joint("joint4", self._start_j4 + self._J4_OFFSET * 0.8, j4_end)
            self.lerp_joint("joint6", self._start_j6 + self._J6_OFFSET * 0.7, j6_end)
            self.lerp_joint("joint7", self._start_j7, self._start_j7 + 0.2 * local_p)

        if p >= 1.0:
            return SkillResult.success("Needle passed through tissue")
        return SkillResult.running()
