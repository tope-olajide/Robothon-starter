"""SurgicalScene — high-level scene configuration and simulation orchestration."""

from pathlib import Path

import mujoco
import numpy as np

from .robot import (
    build_combined_model, build_scene_model, build_scene_model_from_xml,
    PANDA_JOINTS, ALLEGRO_JOINTS, MANAGERIE_MISSING,
)
from .tissue import DeformableTissue


class SurgicalScene:
    """Owns the MuJoCo model, data, renderer, and all scene elements."""

    def __init__(
        self,
        width: int = 1280,
        height: int = 720,
        bimanual: bool = False,
    ):
        if MANAGERIE_MISSING:
            raise RuntimeError(
                "MuJoCo Menagerie models not found.\n"
                "Run: python setup.py"
            )

        self.width = width
        self.height = height
        self.bimanual = bimanual

        try:
            self.model = build_scene_model_from_xml(bimanual=bimanual)
        except Exception as exc:
            print(f"[scene] XML load failed ({exc}), falling back to Python")
            self.model = build_scene_model(bimanual=bimanual)
        self.data = mujoco.MjData(self.model)
        self._renderer_available = True
        try:
            self.renderer = mujoco.Renderer(self.model, width=width, height=height)
        except Exception:
            self.renderer = None
            self._renderer_available = False
        self.tissue = DeformableTissue(self.model, self.data)

        self.camera = mujoco.MjvCamera()
        self._active_camera = "cam_overhead"
        self._set_camera(self._active_camera)
        self.opt = mujoco.MjvOption()

    def _set_camera(self, name: str) -> None:
        cam_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, name)
        if cam_id >= 0:
            self.camera.type = mujoco.mjtCamera.mjCAMERA_FIXED
            self.camera.fixedcamid = cam_id
        else:
            self.camera.type = mujoco.mjtCamera.mjCAMERA_FREE
            self.camera.lookat[:] = (0.45, 0.0, 0.35)
            self.camera.distance = 0.8
            self.camera.azimuth = 120.0
            self.camera.elevation = -25.0
        self._active_camera = name

    def set_camera(self, name: str) -> None:
        self._set_camera(name)

    def set_free_camera(
        self,
        lookat: tuple[float, float, float] = (0.5, 0.0, 0.35),
        distance: float = 0.8,
        azimuth: float = 120.0,
        elevation: float = -25.0,
    ) -> None:
        self.camera.type = mujoco.mjtCamera.mjCAMERA_FREE
        self.camera.lookat[:] = lookat
        self.camera.distance = distance
        self.camera.azimuth = azimuth
        self.camera.elevation = elevation
        self._active_camera = "free"

    def reset(self) -> None:
        mujoco.mj_resetData(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)

    def step(self, nsteps: int = 1) -> None:
        for _ in range(nsteps):
            mujoco.mj_step(self.model, self.data)

    @property
    def renderer_available(self) -> bool:
        return self._renderer_available

    def render(self) -> np.ndarray:
        if not self._renderer_available:
            return np.zeros((self.height, self.width, 4), dtype=np.uint8)
        self.renderer.update_scene(self.data, camera=self.camera, scene_option=self.opt)
        return self.renderer.render().copy()

    def close(self) -> None:
        if self._renderer_available and self.renderer is not None:
            self.renderer.close()

    def body_pos(self, name: str) -> np.ndarray:
        bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)
        if bid < 0:
            return np.zeros(3)
        return self.data.xpos[bid].copy()

    @property
    def needle_pos(self) -> np.ndarray:
        return self.body_pos("needle")

    @property
    def time(self) -> float:
        return float(self.data.time)
