"""Sensor suite for the ARSA-X surgical perception layer.

Provides instrument pose tracking, needle pose estimation, contact force
sensing, and joint state tracking by reading MuJoCo data structures.
"""

import mujoco
import numpy as np


class SensorSuite:
    """Reads simulation state to provide a perception-layer API."""

    FINGER_NAMES = ("index", "middle", "ring", "thumb")

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData):
        self.model = model
        self.data = data

    # ---- pose tracking -------------------------------------------------------

    def body_pos(self, name: str) -> np.ndarray:
        bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)
        if bid < 0:
            return np.zeros(3)
        return self.data.xpos[bid].copy()

    def body_quat(self, name: str) -> np.ndarray:
        bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)
        if bid < 0:
            return np.array([1.0, 0.0, 0.0, 0.0])
        return self.data.xquat[bid].copy()

    def needle_pose(self) -> dict:
        """Return position, orientation, and velocity of the surgical needle."""
        return {
            "pos": self.body_pos("needle").tolist(),
            "quat": self.body_quat("needle").tolist(),
            "vel": self.data.cvel[
                mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "needle")
            ].copy().tolist(),
        }

    def instrument_pose(self, name: str = "attachment") -> dict:
        """Return the end-effector (attachment site) pose."""
        return {
            "pos": self.body_pos(name).tolist(),
            "quat": self.body_quat(name).tolist(),
        }

    # ---- joint state ---------------------------------------------------------

    def joint_pos(self, name: str) -> float:
        jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if jid < 0:
            return 0.0
        return float(self.data.qpos[int(self.model.jnt_qposadr[jid])])

    def joint_vel(self, name: str) -> float:
        jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if jid < 0:
            return 0.0
        dof_adr = int(self.model.jnt_dofadr[jid])
        return float(self.data.qvel[dof_adr])

    def all_joint_states(self) -> dict[str, float]:
        states = {}
        for i in range(self.model.njnt):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i)
            if name:
                states[name] = self.joint_pos(name)
        return states

    def all_joint_velocities(self) -> dict[str, float]:
        velocities = {}
        for i in range(self.model.njnt):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i)
            if name:
                dof_adr = int(self.model.jnt_dofadr[i])
                velocities[name] = float(self.data.qvel[dof_adr])
        return velocities

    # ---- force / contact sensing ---------------------------------------------

    def contact_forces(self, body_name: str) -> list[dict]:
        """Return all contact forces acting on a specific body."""
        bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        if bid < 0:
            return []
        results = []
        for i in range(self.data.ncon):
            c = self.data.contact[i]
            g1, g2 = int(self.model.geom_bodyid[c.geom1]), int(self.model.geom_bodyid[c.geom2])
            if bid in (g1, g2):
                results.append({
                    "dist": float(c.dist),
                    "friction": c.friction[:2].copy().tolist(),
                    "normal": c.frame[:3].copy().tolist(),
                    "force": (c.friction[:2] * c.priority).tolist(),
                    "geom1": mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, int(c.geom1)),
                    "geom2": mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, int(c.geom2)),
                })
        return results

    def needle_grasp_force(self) -> float:
        """Return the estimated grasp force on the needle (sum of contact normals)."""
        bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "needle")
        if bid < 0:
            return 0.0
        total = 0.0
        for i in range(self.data.ncon):
            c = self.data.contact[i]
            g1, g2 = int(self.model.geom_bodyid[c.geom1]), int(self.model.geom_bodyid[c.geom2])
            if bid in (g1, g2):
                total += np.linalg.norm(c.frame[:3]) * abs(c.dist)
        return float(total)

    def is_needle_grasped(self, threshold: float = 0.01) -> bool:
        """Return True if the needle is held (contact force exceeds threshold)."""
        return self.needle_grasp_force() > threshold

    # ---- raw sensor data -----------------------------------------------------

    def raw_sensor(self, name: str) -> np.ndarray | None:
        """Read raw sensor data by name from the sensordata array."""
        sid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, name)
        if sid < 0:
            return None
        adr = int(self.model.sensor_adr[sid])
        dim = int(self.model.sensor_dim[sid])
        return self.data.sensordata[adr: adr + dim].copy()

    def wrist_force(self) -> np.ndarray | None:
        """Return the 3-axis force at the wrist (N)."""
        return self.raw_sensor("sensor_wrist_force")

    def wrist_torque(self) -> np.ndarray | None:
        """Return the 3-axis torque at the wrist (Nm)."""
        return self.raw_sensor("sensor_wrist_torque")

    # ---- tactile sensing -----------------------------------------------------

    def tactile_force(self, finger: str) -> float:
        """Return the scalar touch magnitude for a fingertip (0-1 scale)."""
        data = self.raw_sensor(f"tactile_{finger}")
        if data is None or len(data) == 0:
            return 0.0
        return float(data[0])

    def finger_force(self, finger: str) -> np.ndarray | None:
        """Return the 3-axis force vector at a fingertip site (N)."""
        return self.raw_sensor(f"finger_force_{finger}")

    def finger_torque(self, finger: str) -> np.ndarray | None:
        """Return the 3-axis torque vector at a fingertip site (Nm)."""
        return self.raw_sensor(f"finger_torque_{finger}")

    def all_tactile(self) -> dict[str, float]:
        """Return touch magnitude for all four fingertips."""
        return {f: self.tactile_force(f) for f in self.FINGER_NAMES}

    def all_finger_forces(self) -> dict[str, np.ndarray | None]:
        """Return 3-axis force vectors for all four fingertips."""
        return {f: self.finger_force(f) for f in self.FINGER_NAMES}

    def grasp_force_distribution(self) -> dict[str, float]:
        """Compute normalized force distribution across fingertips."""
        forces = {}
        for f in self.FINGER_NAMES:
            fvec = self.finger_force(f)
            forces[f] = float(np.linalg.norm(fvec)) if fvec is not None else 0.0
        total = sum(forces.values())
        if total < 1e-6:
            return {f: 0.0 for f in self.FINGER_NAMES}
        return {f: v / total for f, v in forces.items()}
