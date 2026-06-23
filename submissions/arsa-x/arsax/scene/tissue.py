"""Deformable spring-mesh tissue phantom for surgical simulation.

Uses a grid of small sphere bodies connected by equality constraints
(spring-mass) that deform realistically when the needle passes through.
"""

import mujoco
import numpy as np


class DeformableTissue:
    """Manages a spring-mesh deformable tissue phantom.

    The tissue is a grid of small sphere bodies with free joints, connected
    by ``mjEQ_CONNECT`` equality constraints that act like springs. Corner
    spheres are anchored to the table via ``mjEQ_WELD``. When the needle
    pushes against or through the mesh, individual spheres move, creating
    realistic deformation visible in the rendered video.
    """

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData):
        self.model = model
        self.data = data

        self._rows = 5
        self._cols = 4
        self._sphere_ids: list[int] = []
        for r in range(self._rows):
            for c in range(self._cols):
                bid = mujoco.mj_name2id(
                    model, mujoco.mjtObj.mjOBJ_BODY, f"tissue_s_{r}_{c}"
                )
                self._sphere_ids.append(bid)
                jid = mujoco.mj_name2id(
                    model, mujoco.mjtObj.mjOBJ_JOINT, f"tissue_j_{r}_{c}"
                )
                if jid >= 0:
                    for dof in range(6):
                        dof_adr = int(model.jnt_dofadr[jid]) + dof
                        if dof_adr < model.nv:
                            model.dof_damping[dof_adr] = 0.03

        self._rest_pos = np.array([0.45, -0.02, 0.370])
        self._needle_bid = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_BODY, "needle"
        )

    @property
    def available(self) -> bool:
        return len(self._sphere_ids) > 0 and self._sphere_ids[0] >= 0

    def sphere_positions(self) -> np.ndarray:
        """Return (N, 3) array of current sphere positions."""
        positions = []
        for bid in self._sphere_ids:
            if bid >= 0:
                positions.append(self.data.xpos[bid].copy())
        return np.array(positions) if positions else np.zeros((0, 3))

    def sphere_displacements(self) -> np.ndarray:
        """Return (N,) array of displacement magnitudes from rest."""
        pos = self.sphere_positions()
        if len(pos) == 0:
            return np.zeros(0)
        rows, cols = self._rows, self._cols
        spacing_x = 0.038
        spacing_y = 0.032
        displacements = []
        idx = 0
        for r in range(rows):
            for c in range(cols):
                rx = 0.45 + (c - (cols - 1) / 2) * spacing_x
                ry = -0.02 + (r - (rows - 1) / 2) * spacing_y
                rest = np.array([rx, ry, 0.370])
                d = np.linalg.norm(pos[idx] - rest)
                displacements.append(d)
                idx += 1
        return np.array(displacements)

    def max_displacement(self) -> float:
        """Return the maximum sphere displacement from rest (m)."""
        d = self.sphere_displacements()
        return float(np.max(d)) if len(d) > 0 else 0.0

    def contact_force_estimate(self) -> float:
        """Estimate contact force magnitude on the needle from the tissue."""
        if self._needle_bid < 0:
            return 0.0
        total = 0.0
        for i in range(self.data.ncon):
            c = self.data.contact[i]
            g1 = int(c.geom1)
            g2 = int(c.geom2)
            b1 = int(self.model.geom_bodyid[g1])
            b2 = int(self.model.geom_bodyid[g2])
            if self._needle_bid in (b1, b2) and (b1 != b2):
                total += abs(c.dist)
        return float(total)

    def is_punctured(self, threshold: float = 0.008) -> bool:
        """Check if the needle has penetrated the tissue surface."""
        if self._needle_bid < 0:
            return False
        needle_z = self.data.xpos[self._needle_bid][2]
        return needle_z < (self._rest_pos[2] - threshold)

    def indentation_depth(self) -> float:
        """Return how far (in m) the needle has pressed into the tissue."""
        if self._needle_bid < 0:
            return 0.0
        dz = self._rest_pos[2] - self.data.xpos[self._needle_bid][2]
        return max(0.0, float(dz))
