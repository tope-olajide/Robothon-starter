"""SurgicalPlanner — decomposes a high-level surgical goal into skill sequences."""

from ..skills import (
    GraspNeedle, OrientNeedle, InsertNeedle, PullSuture,
    TieKnot, RegraspNeedle, StabilizeTissue, BimanualStabilizeTissue,
    ReleaseObject, FingerGait, SkillResult,
)

SUTURE_WORKFLOW = [
    ("stabilize_tissue", {"target_pos": (0.45, -0.02, 0.38)}),
    ("grasp_needle", {"target_pos": (0.38, -0.08, 0.48)}),
    ("orient_needle", {"target_angle_deg": 45.0}),
    ("insert_needle", {}),
    ("pull_suture", {"pull_distance": 0.04}),
    ("regrasp_needle", {}),
    ("tie_knot", {}),
]

DOUBLE_SUTURE_WORKFLOW = [
    ("stabilize_tissue", {"target_pos": (0.45, -0.02, 0.38)}),
    ("grasp_needle", {"target_pos": (0.38, -0.08, 0.48)}),
    ("orient_needle", {"target_angle_deg": 45.0}),
    ("insert_needle", {}),
    ("pull_suture", {"pull_distance": 0.04}),
    ("orient_needle", {"target_angle_deg": 30.0}),
    ("insert_needle", {}),
    ("pull_suture", {"pull_distance": 0.04}),
    ("tie_knot", {}),
]

MATTRESS_SUTURE_WORKFLOW = [
    ("stabilize_tissue", {"target_pos": (0.45, -0.02, 0.38)}),
    ("grasp_needle", {"target_pos": (0.38, -0.08, 0.48)}),
    ("orient_needle", {"target_angle_deg": 45.0}),
    ("insert_needle", {}),
    ("pull_suture", {"pull_distance": 0.04}),
    ("regrasp_needle", {}),
    ("orient_needle", {"target_angle_deg": 35.0}),
    ("insert_needle", {}),
    ("pull_suture", {"pull_distance": 0.04}),
    ("regrasp_needle", {}),
    ("tie_knot", {}),
]

FIGURE_EIGHT_WORKFLOW = [
    ("stabilize_tissue", {"target_pos": (0.45, -0.02, 0.38)}),
    ("grasp_needle", {"target_pos": (0.38, -0.08, 0.48)}),
    ("orient_needle", {"target_angle_deg": 50.0}),
    ("insert_needle", {}),
    ("pull_suture", {"pull_distance": 0.04}),
    ("regrasp_needle", {}),
    ("orient_needle", {"target_angle_deg": -40.0}),
    ("insert_needle", {}),
    ("pull_suture", {"pull_distance": 0.04}),
    ("regrasp_needle", {}),
    ("tie_knot", {}),
]

RUNNING_SUTURE_WORKFLOW = [
    ("stabilize_tissue", {"target_pos": (0.45, -0.02, 0.38)}),
    ("grasp_needle", {"target_pos": (0.38, -0.08, 0.48)}),
    ("orient_needle", {"target_angle_deg": 40.0}),
    ("insert_needle", {}),
    ("pull_suture", {"pull_distance": 0.03}),
    ("orient_needle", {"target_angle_deg": 35.0}),
    ("insert_needle", {}),
    ("pull_suture", {"pull_distance": 0.03}),
    ("orient_needle", {"target_angle_deg": 30.0}),
    ("insert_needle", {}),
    ("pull_suture", {"pull_distance": 0.03}),
    ("regrasp_needle", {}),
    ("tie_knot", {}),
]

BIMANUAL_SUTURE_WORKFLOW = [
    ("bimanual_stabilize", {"target_pos": (0.45, -0.02, 0.37)}),
    ("grasp_needle", {"target_pos": (0.38, -0.08, 0.48)}),
    ("orient_needle", {"target_angle_deg": 45.0}),
    ("insert_needle", {}),
    ("pull_suture", {"pull_distance": 0.04}),
    ("regrasp_needle", {}),
    ("tie_knot", {}),
]

BIMANUAL_DOUBLE_WORKFLOW = [
    ("bimanual_stabilize", {"target_pos": (0.45, -0.02, 0.37)}),
    ("grasp_needle", {"target_pos": (0.38, -0.08, 0.48)}),
    ("orient_needle", {"target_angle_deg": 45.0}),
    ("insert_needle", {}),
    ("pull_suture", {"pull_distance": 0.04}),
    ("orient_needle", {"target_angle_deg": 30.0}),
    ("insert_needle", {}),
    ("pull_suture", {"pull_distance": 0.04}),
    ("tie_knot", {}),
]

SKILL_MAP = {
    "grasp_needle": GraspNeedle,
    "orient_needle": OrientNeedle,
    "insert_needle": InsertNeedle,
    "pull_suture": PullSuture,
    "tie_knot": TieKnot,
    "regrasp_needle": RegraspNeedle,
    "stabilize_tissue": StabilizeTissue,
    "bimanual_stabilize": BimanualStabilizeTissue,
    "release_object": ReleaseObject,
    "finger_gait": FingerGait,
}


class SurgicalPlanner:
    """Decomposes surgical goals into skill sequences and manages execution."""

    def __init__(self):
        self._plan: list[tuple[str, dict]] = []
        self._current_step = 0
        self._history: list[dict] = []

    def plan_interrupted_suture(self) -> list[tuple[str, dict]]:
        self._plan = list(SUTURE_WORKFLOW)
        self._current_step = 0
        return self._plan

    def plan_double_suture(self) -> list[tuple[str, dict]]:
        self._plan = list(DOUBLE_SUTURE_WORKFLOW)
        self._current_step = 0
        return self._plan

    def plan_mattress_suture(self) -> list[tuple[str, dict]]:
        self._plan = list(MATTRESS_SUTURE_WORKFLOW)
        self._current_step = 0
        return self._plan

    def plan_figure_eight(self) -> list[tuple[str, dict]]:
        self._plan = list(FIGURE_EIGHT_WORKFLOW)
        self._current_step = 0
        return self._plan

    def plan_running_suture(self) -> list[tuple[str, dict]]:
        self._plan = list(RUNNING_SUTURE_WORKFLOW)
        self._current_step = 0
        return self._plan

    def plan_bimanual_suture(self) -> list[tuple[str, dict]]:
        self._plan = list(BIMANUAL_SUTURE_WORKFLOW)
        self._current_step = 0
        return self._plan

    def plan_bimanual_double(self) -> list[tuple[str, dict]]:
        self._plan = list(BIMANUAL_DOUBLE_WORKFLOW)
        self._current_step = 0
        return self._plan

    def plan_from_goal(self, goal: str) -> list[tuple[str, dict]]:
        """Plan from a natural-language surgical goal using keyword matching."""
        goal_lower = goal.lower()
        # Bimanual workflows (requires assistant arm in scene)
        if "bimanual" in goal_lower and ("double" in goal_lower or "two" in goal_lower):
            return self.plan_bimanual_double()
        if "bimanual" in goal_lower:
            return self.plan_bimanual_suture()
        if "figure" in goal_lower or "eight" in goal_lower:
            return self.plan_figure_eight()
        if "running" in goal_lower or "continuous" in goal_lower:
            return self.plan_running_suture()
        if "mattress" in goal_lower:
            return self.plan_mattress_suture()
        if "double" in goal_lower or "two" in goal_lower:
            return self.plan_double_suture()
        if "knot" in goal_lower or "suture" in goal_lower or "interrupted" in goal_lower:
            return self.plan_interrupted_suture()
        return self.plan_interrupted_suture()

    @property
    def current_step(self) -> int:
        return self._current_step

    @property
    def total_steps(self) -> int:
        return len(self._plan)

    @property
    def plan(self) -> list[tuple[str, dict]]:
        return list(self._plan)

    def next_skill(self) -> tuple[str, dict] | None:
        if self._current_step >= len(self._plan):
            return None
        step = self._plan[self._current_step]
        self._current_step += 1
        return step

    def peek_next(self) -> tuple[str, dict] | None:
        if self._current_step >= len(self._plan):
            return None
        return self._plan[self._current_step]

    def get_progress(self) -> dict:
        return {
            "step": self._current_step,
            "total": self.total_steps,
            "plan": self._plan,
            "history": self._history,
        }

    def record_result(self, skill_name: str, result: SkillResult) -> None:
        self._history.append({
            "skill": skill_name,
            "status": result.status.name,
            "message": result.message,
        })

    def replan(self, failed_skill: str, result: SkillResult) -> list[tuple[str, dict]]:
        """Replan after a failure — insert recovery and retry."""
        if failed_skill == "grasp_needle":
            self._plan.insert(self._current_step, ("regrasp_needle", {}))
        elif failed_skill == "insert_needle":
            self._plan.insert(self._current_step, ("stabilize_tissue", {"target_pos": (0.45, -0.02, 0.38)}))
        self._plan.insert(self._current_step + 1, (failed_skill, {}))
        return self._plan
