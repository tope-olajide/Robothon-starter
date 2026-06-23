"""Closed-loop differential inverse kinematics for the Panda arm.

Drives a chosen MuJoCo site (e.g., a grasp point) to a desired Cartesian pose
using damped least-squares (DLS) on the arm's 7 revolute joints.
"""

import mujoco
import numpy as np


PANDA_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]


class ArmIK:
    """Damped least-squares IK over the 7 Panda arm joints for a site target."""

    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        site_name: str,
        damping: float = 0.08,
        max_step: float = 0.06,
    ):
        self.model = model
        self.data = data
        self.site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site_name)
        if self.site_id < 0:
            raise ValueError(f"IK site {site_name!r} not found in model")
        self.damping = damping
        self.max_step = max_step

        self._dof_cols: list[int] = []
        self._qpos_adr: list[int] = []
        self._jnt_range: list[tuple[float, float]] = []
        for jn in PANDA_JOINTS:
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jn)
            if jid < 0:
                continue
            self._dof_cols.append(int(model.jnt_dofadr[jid]))
            self._qpos_adr.append(int(model.jnt_qposadr[jid]))
            lo, hi = model.jnt_range[jid]
            self._jnt_range.append((float(lo), float(hi)))
        self._jacp = np.zeros((3, model.nv))
        self._jacr = np.zeros((3, model.nv))

    def site_pos(self) -> np.ndarray:
        return self.data.site_xpos[self.site_id].copy()

    def site_quat(self) -> np.ndarray:
        q = np.zeros(4)
        mujoco.mju_mat2Quat(q, self.data.site_xmat[self.site_id])
        return q

    def solve(
        self,
        target_pos: np.ndarray,
        target_quat: np.ndarray | None = None,
        pos_gain: float = 1.0,
        rot_gain: float = 0.6,
    ) -> dict[str, float]:
        """Return new arm joint-angle targets stepping the site toward the goal."""
        mujoco.mj_jacSite(self.model, self.data, self._jacp, self._jacr, self.site_id)

        err = np.zeros(6)
        err[:3] = (np.asarray(target_pos) - self.site_pos()) * pos_gain

        if target_quat is not None:
            cur_q = self.site_quat()
            dq = np.zeros(3)
            neg_cur = np.zeros(4)
            mujoco.mju_negQuat(neg_cur, cur_q)
            err_q = np.zeros(4)
            mujoco.mju_mulQuat(err_q, np.asarray(target_quat, dtype=float), neg_cur)
            mujoco.mju_quat2Vel(dq, err_q, 1.0)
            err[3:] = dq * rot_gain

        J = np.vstack([self._jacp, self._jacr])[:, self._dof_cols]
        lam2 = self.damping ** 2
        JJt = J @ J.T + lam2 * np.eye(6)
        dq = J.T @ np.linalg.solve(JJt, err)

        norm = np.linalg.norm(dq)
        if norm > self.max_step:
            dq *= self.max_step / norm

        targets: dict[str, float] = {}
        for i, jn in enumerate(PANDA_JOINTS[: len(self._dof_cols)]):
            cur = float(self.data.qpos[self._qpos_adr[i]])
            lo, hi = self._jnt_range[i]
            targets[jn] = float(np.clip(cur + dq[i], lo, hi))
        return targets

    def position_error(self, target_pos: np.ndarray) -> float:
        return float(np.linalg.norm(np.asarray(target_pos) - self.site_pos()))

    def solve_qpos(
        self,
        target_pos: np.ndarray,
        target_quat: np.ndarray | None = None,
        seed: dict[str, float] | None = None,
        iters: int = 200,
        tol: float = 1e-3,
        rot_gain: float = 0.5,
    ) -> dict[str, float]:
        """Kinematically solve for arm joint angles reaching the target pose.

        Runs Newton-style DLS iterations on a scratch copy of the state,
        returning a joint-angle dict for the arm (no physics integration).
        """
        scratch = mujoco.MjData(self.model)
        scratch.qpos[:] = self.data.qpos
        if seed:
            for jn, v in seed.items():
                jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, jn)
                if jid >= 0:
                    scratch.qpos[int(self.model.jnt_qposadr[jid])] = v
        mujoco.mj_forward(self.model, scratch)

        jacp = np.zeros((3, self.model.nv))
        jacr = np.zeros((3, self.model.nv))
        target_pos = np.asarray(target_pos, dtype=float)
        for _ in range(iters):
            site_pos = scratch.site_xpos[self.site_id]
            err = np.zeros(6)
            err[:3] = target_pos - site_pos
            if target_quat is not None:
                cur_q = np.zeros(4)
                mujoco.mju_mat2Quat(cur_q, scratch.site_xmat[self.site_id])
                neg_cur = np.zeros(4)
                mujoco.mju_negQuat(neg_cur, cur_q)
                err_q = np.zeros(4)
                mujoco.mju_mulQuat(err_q, np.asarray(target_quat, dtype=float), neg_cur)
                dvel = np.zeros(3)
                mujoco.mju_quat2Vel(dvel, err_q, 1.0)
                err[3:] = dvel * rot_gain
            if np.linalg.norm(err[:3]) < tol and (
                target_quat is None or np.linalg.norm(err[3:]) < 1e-2
            ):
                break
            mujoco.mj_jacSite(self.model, scratch, jacp, jacr, self.site_id)
            J = np.vstack([jacp, jacr])[:, self._dof_cols]
            lam2 = self.damping ** 2
            dq = J.T @ np.linalg.solve(J @ J.T + lam2 * np.eye(6), err)
            for i in range(len(self._dof_cols)):
                qa = self._qpos_adr[i]
                lo, hi = self._jnt_range[i]
                scratch.qpos[qa] = float(np.clip(scratch.qpos[qa] + dq[i], lo, hi))
            mujoco.mj_forward(self.model, scratch)

        return {
            PANDA_JOINTS[i]: float(scratch.qpos[self._qpos_adr[i]])
            for i in range(len(self._dof_cols))
        }
