"""Skill execution engine — runs skills step by step."""

from typing import Callable

import mujoco

from ..skills import SkillBase, SkillResult, SkillStatus
from .planner import SurgicalPlanner, SKILL_MAP


class SkillExecutor:
    """Orchestrates the execution of a planned skill sequence.

    Manages the skill lifecycle: init → tick (iteratively) → finish,
    with progress callbacks and integration with the failure monitor.
    """

    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        planner: SurgicalPlanner,
        monitor: "FailureMonitor | None" = None,
        on_step_complete: Callable[[str, SkillResult], None] | None = None,
    ):
        self._model = model
        self._data = data
        self._planner = planner
        self._monitor = monitor
        self._on_step_complete = on_step_complete

        self._current_skill: SkillBase | None = None
        self._current_skill_name: str = ""
        self._current_params: dict = {}
        self._is_running = False
        self._completed_steps: list[str] = []

    @property
    def current_skill_name(self) -> str:
        return self._current_skill_name

    @property
    def is_running(self) -> bool:
        return self._is_running

    @property
    def completed_steps(self) -> list[str]:
        return list(self._completed_steps)

    def start(self, goal: str | None = None) -> None:
        """Start or reset the executor and begin the first skill."""
        self._completed_steps.clear()
        self._is_running = True
        if goal:
            self._planner.plan_from_goal(goal)
        self._advance()

    def _advance(self) -> None:
        """Move to the next skill in the plan."""
        next_skill = self._planner.next_skill()
        if next_skill is None:
            self._is_running = False
            self._current_skill = None
            self._current_skill_name = ""
            return
        name, params = next_skill
        skill_cls = SKILL_MAP.get(name)
        if skill_cls is None:
            raise ValueError(f"Unknown skill: {name}")
        if self._monitor:
            self._monitor.reset()
        self._current_skill_name = name
        self._current_params = params
        self._current_skill = skill_cls(self._model, self._data, name=name)
        self._current_skill.initialize(**params)

    def tick(self, dt: float) -> SkillResult | None:
        """Advance the current skill by one simulation timestep."""
        if self._current_skill is None or not self._is_running:
            return None
        result = self._current_skill.tick(dt)

        if result.status in (SkillStatus.SUCCESS, SkillStatus.FAILED):
            self._completed_steps.append(self._current_skill_name)
            self._planner.record_result(self._current_skill_name, result)
            if self._on_step_complete:
                self._on_step_complete(self._current_skill_name, result)
            if result.status == SkillStatus.FAILED:
                self._planner.replan(self._current_skill_name, result)
            self._advance()
            return result

        if self._monitor and self._current_skill:
            failure = self._monitor.check(self._current_skill_name, self._current_skill)
            if failure:
                self._planner.record_result(self._current_skill_name, failure)
                self._planner.replan(self._current_skill_name, failure)
                self._advance()
                return failure

        return None

    def get_status(self) -> dict:
        return {
            "running": self._is_running,
            "current_skill": self._current_skill_name,
            "completed": self._completed_steps,
            "progress": self._planner.get_progress(),
        }
