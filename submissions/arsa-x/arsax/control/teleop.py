"""Keyboard-based teleoperation controller for the surgical robot."""

import mujoco
import numpy as np

from ..scene.robot import PANDA_JOINTS, ALLEGRO_JOINTS, ALLEGRO_OPEN, ALLEGRO_CLOSE, ALLEGRO_PINCH


class TeleopController:
    """Direct teleoperation of the Panda arm + Allegro Hand via keyboard.

    Key mappings:
      Arm: 1-7 select joint, Q/E increase/decrease joint angle
      Hand: O = open, C = close, P = pinch
      Speed: [/] adjust step size
    """

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData, sensors=None):
        self.model = model
        self.data = data
        self.sensors = sensors
        self._enabled = False
        self._selected_joint = 0
        self._step_size = 0.05
        self._cartesian_mode = False

    @property
    def enabled(self) -> bool:
        return self._enabled

    def enable(self) -> None:
        self._enabled = True

    def disable(self) -> None:
        self._enabled = False

    def handle_key(self, key: str) -> bool:
        if not self._enabled:
            return False
        key = key.upper()

        if key in "1234567":
            idx = int(key) - 1
            if idx < len(PANDA_JOINTS):
                self._selected_joint = idx
            return True
        if key == "Q":
            self._adjust_joint(self._selected_joint, self._step_size)
            return True
        if key == "E":
            self._adjust_joint(self._selected_joint, -self._step_size)
            return True
        if key == "O":
            self._set_allegro_pose(ALLEGRO_OPEN)
            return True
        if key == "C":
            self._set_allegro_pose(ALLEGRO_CLOSE)
            return True
        if key == "P":
            self._set_allegro_pose(ALLEGRO_PINCH)
            return True
        if key == "[":
            self._step_size = max(0.005, self._step_size - 0.01)
            return True
        if key == "]":
            self._step_size = min(0.5, self._step_size + 0.01)
            return True
        return False

    def _adjust_joint(self, joint_idx: int, delta: float) -> None:
        if joint_idx >= len(PANDA_JOINTS):
            return
        jn = PANDA_JOINTS[joint_idx]
        jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, jn)
        if jid < 0:
            return
        qa = int(self.model.jnt_qposadr[jid])
        current = self.data.qpos[qa]
        new_val = current + delta
        if self.model.jnt_limited[jid]:
            low, high = self.model.jnt_range[jid]
            new_val = float(np.clip(new_val, low, high))
        self.data.qpos[qa] = new_val

    def _set_allegro_pose(self, pose: dict[str, float]) -> None:
        for jn, val in pose.items():
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, jn)
            if jid < 0:
                continue
            qa = int(self.model.jnt_qposadr[jid])
            if self.model.jnt_limited[jid]:
                low, high = self.model.jnt_range[jid]
                val = float(np.clip(val, low, high))
            self.data.qpos[qa] = val

    @staticmethod
    def help_text() -> str:
        return (
            "KEYBOARD CONTROLS:\n"
            "  1-7    : Select arm joint\n"
            "  Q/E    : Increase/Decrease selected joint\n"
            "  O/C/P  : Open / Close / Pinch hand\n"
            "  [/]    : Decrease/Increase step size\n"
            "  SPACE  : Toggle pause\n"
            "  ESC    : Exit\n"
        )
