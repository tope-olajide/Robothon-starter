"""Latency simulation for communication-delay experiments.

Simulates the effect of network/communication latency on robot control by
delaying joint-position commands before they reach the physics simulation.
"""

from collections import deque

import mujoco
import numpy as np


class LatencySimulator:
    """Buffers joint commands and applies them after a configurable delay."""

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData, delay_seconds: float = 0.0):
        self.model = model
        self.data = data
        self._delay = max(0.0, delay_seconds)
        self._buffer: deque[tuple[float, np.ndarray]] = deque()
        self._sim_time = 0.0
        self._enabled = self._delay > 1e-6
        self._max_queue: int = 0

    @property
    def delay(self) -> float:
        return self._delay

    @property
    def enabled(self) -> bool:
        return self._enabled

    @property
    def queue_depth(self) -> int:
        return len(self._buffer)

    @property
    def buffer_seconds(self) -> float:
        if not self._buffer:
            return 0.0
        return self._buffer[-1][0] - self._buffer[0][0]

    def record_and_delay(self, dt: float) -> None:
        """Record current data.qpos and apply a delayed version.

        Call after skills have written joint targets but before mj_step().
        """
        if not self._enabled:
            return
        cmd_qpos = self.data.qpos.copy()
        self._buffer.append((self._sim_time + self._delay, cmd_qpos))
        while len(self._buffer) > 1 and self._buffer[0][0] <= self._sim_time:
            self._buffer.popleft()
        self._max_queue = max(self._max_queue, len(self._buffer))
        if self._buffer:
            _, delayed_qpos = self._buffer[0]
            self.data.qpos[:] = delayed_qpos
        self._sim_time += dt

    def reset(self) -> None:
        self._buffer.clear()
        self._sim_time = 0.0
        self._max_queue = 0

    def get_stats(self) -> dict:
        return {
            "delay_s": self._delay,
            "delay_ms": round(self._delay * 1000, 1),
            "enabled": self._enabled,
            "queue_depth": len(self._buffer),
            "buffer_s": round(self.buffer_seconds, 4),
            "max_queue": self._max_queue,
        }
