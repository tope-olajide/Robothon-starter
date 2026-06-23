"""Release the grasped object by lowering the arm and opening the Allegro Hand."""

from ..scene.robot import ALLEGRO_OPEN, ALLEGRO_PINCH
from .base import SkillBase, SkillResult


class ReleaseObject(SkillBase):
    """Lower the arm toward the table, then open the hand to release the object.

    Phase 1 (0–40%): Arm lowers to place the needle gently on the table.
    Phase 2 (40–100%): Fingers open to release the needle.
    """

    def initialize(self, **kwargs) -> None:
        super().initialize(**kwargs)
        # Store initial arm position for the lowering motion
        self._initial_j2 = self._get_joint("joint2")
        self._lower_j2 = self._initial_j2 + 0.12  # lower ~0.12 rad toward table
        self._pinch_pose = dict(ALLEGRO_PINCH)
        self._open_pose = dict(ALLEGRO_OPEN)

    def on_tick(self, dt: float) -> SkillResult:
        p = self._progress
        if p < 0.4:
            # Phase 1: Lower arm toward table while keeping grip firm
            lp = p / 0.4
            target_j2 = self._initial_j2 + (self._lower_j2 - self._initial_j2) * lp
            self.set_joint("joint2", target_j2)
            for jn in self._pinch_pose:
                self.set_joint(jn, self._pinch_pose[jn])
        else:
            # Phase 2: Open fingers to release the needle
            op = (p - 0.4) / 0.6
            for jn in self._open_pose:
                start = self._pinch_pose.get(jn, 0.0)
                target = self._open_pose[jn]
                self.set_joint(jn, start + (target - start) * op)

        if p >= 1.0:
            return SkillResult.success("Object released")
        return SkillResult.running()
