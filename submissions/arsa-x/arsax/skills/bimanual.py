"""Bimanual tissue stabilization — second arm stabilizes tissue while primary arm sutures."""

import mujoco

from .base import SkillBase, SkillResult
from ..scene.robot import (
    ASSISTANT_PREFIX, ASSISTANT_HOME_POSE, ASSISTANT_TISSUE_PRESS_POSE,
    PANDA_JOINTS,
)


class BimanualStabilizeTissue(SkillBase):
    """Position the assistant arm to press down on tissue for stabilization.

    Controls the second Panda arm (no hand) to descend and apply gentle
    downward pressure on the tissue surface near the suture entry point.
    This mimics how a surgical assistant stabilizes tissue while the
    primary surgeon operates — the key innovation of bimanual surgical
    autonomy.

    Phases:
      0–40%:  Move assistant arm from home to approach position above tissue
      40–70%: Descend toward tissue surface
      70–100%: Apply pressure and hold — tissue deformation visible
    """

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData, name: str = "bimanual_stabilize"):
        super().__init__(model, data, name)
        self._duration = 4.0
        self._assistant_addrs: dict[str, int] = {}
        self._assistant_actuators: dict[str, int] = {}

        # Cache assistant arm joint and actuator addresses
        for jn in PANDA_JOINTS:
            prefixed = f"{ASSISTANT_PREFIX}{jn}"
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, prefixed)
            if jid >= 0:
                self._assistant_addrs[jn] = int(model.jnt_qposadr[jid])

        for i in range(model.nu):
            act_jid = int(model.actuator_trnid[i][0])
            jname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, act_jid)
            if jname and jname.startswith(ASSISTANT_PREFIX):
                short = jname[len(ASSISTANT_PREFIX):]
                self._assistant_actuators[short] = i

    def _set_assistant(self, name: str, value: float) -> None:
        """Set a single assistant arm joint via actuator or qpos."""
        if name in self._assistant_actuators:
            self.data.ctrl[self._assistant_actuators[name]] = float(value)
        if name in self._assistant_addrs:
            self.data.qpos[self._assistant_addrs[name]] = float(value)

    def can_run(self) -> bool:
        """Only run if the assistant arm joints exist in the model."""
        return len(self._assistant_actuators) > 0

    def initialize(self, **kwargs) -> None:
        super().initialize(**kwargs)
        self._target_pos = kwargs.get("target_pos", (0.45, -0.02, 0.37))

    def on_tick(self, dt: float) -> SkillResult:
        t = self._eased

        if t < 0.4:
            # Phase 1: Move to approach position above tissue
            approach_t = min(1.0, t / 0.4)
            for jn in PANDA_JOINTS:
                home_val = ASSISTANT_HOME_POSE.get(jn, 0.0)
                self._set_assistant(jn, home_val * approach_t)

        elif t < 0.7:
            # Phase 2: Descend toward tissue surface
            descend_t = (t - 0.4) / 0.3
            for jn in PANDA_JOINTS:
                start_val = ASSISTANT_HOME_POSE.get(jn, 0.0)
                end_val = ASSISTANT_TISSUE_PRESS_POSE.get(jn, 0.0)
                val = start_val + (end_val - start_val) * descend_t
                self._set_assistant(jn, val)

        else:
            # Phase 3: Apply pressure and hold — tissue deformation visible
            for jn in PANDA_JOINTS:
                self._set_assistant(jn, ASSISTANT_TISSUE_PRESS_POSE.get(jn, 0.0))

        return SkillResult.running()

    def on_finish(self) -> SkillResult:
        # Hold the tissue press position — assistant arm stays locked
        for jn in PANDA_JOINTS:
            self._set_assistant(jn, ASSISTANT_TISSUE_PRESS_POSE.get(jn, 0.0))
        return SkillResult.success("Assistant arm stabilizing tissue — bimanual mode active")
