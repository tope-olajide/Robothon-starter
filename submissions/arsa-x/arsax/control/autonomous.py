"""Autonomous / agent-assisted control mode."""

import mujoco

from ..planning import SurgicalPlanner, SkillExecutor, FailureMonitor
from ..scene.sensors import SensorSuite
from ..skills import SkillResult, SkillStatus


class AutonomousController:
    """Agent-assisted autonomous control mode.

    The surgeon provides a high-level goal (e.g. "place interrupted suture"),
    and the system autonomously executes the skill sequence.
    """

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData, sensors: SensorSuite | None = None):
        self.model = model
        self.data = data
        self.sensors = sensors

        self._planner = SurgicalPlanner()
        self._monitor = FailureMonitor(model, data)
        self._executor = SkillExecutor(
            model, data, self._planner, self._monitor,
            on_step_complete=self._on_step_done,
        )
        self._active_goal: str = ""
        self._log: list[str] = []

    @property
    def is_active(self) -> bool:
        return self._executor.is_running

    @property
    def current_skill(self) -> str:
        return self._executor.current_skill_name

    @property
    def completed_steps(self) -> list[str]:
        return self._executor.completed_steps

    @property
    def plan(self) -> list[tuple[str, dict]]:
        return self._planner.plan

    @property
    def log(self) -> list[str]:
        return list(self._log)

    def start_procedure(self, goal: str) -> None:
        self._active_goal = goal
        self._log.clear()
        self._log.append(f"START: {goal}")
        self._executor.start(goal)

    def stop(self) -> None:
        self._log.append("STOP: procedure interrupted by surgeon")
        self._executor._is_running = False

    def tick(self, dt: float) -> SkillResult | None:
        return self._executor.tick(dt)

    def _on_step_done(self, skill_name: str, result: SkillResult) -> None:
        status = "✓" if result.status == SkillStatus.SUCCESS else "✗"
        self._log.append(f"  {status} {skill_name}: {result.message}")

    def get_status(self) -> dict:
        return {
            "active": self.is_active,
            "goal": self._active_goal,
            "current_skill": self.current_skill,
            "completed_steps": self.completed_steps,
            "log": self.log,
        }
