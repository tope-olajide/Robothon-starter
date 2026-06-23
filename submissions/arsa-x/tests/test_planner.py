"""Unit tests for SurgicalPlanner."""

from __future__ import annotations

import pytest

from src.skills import SkillResult, SkillStatus
from src.agent import (
    SurgicalPlanner,
    SUTURE_WORKFLOW,
    DOUBLE_SUTURE_WORKFLOW,
    SKILL_MAP,
)


class TestPredefinedWorkflows:
    def test_suture_workflow_has_7_steps(self):
        assert len(SUTURE_WORKFLOW) == 7

    def test_suture_workflow_has_expected_skills(self):
        names = [name for name, _ in SUTURE_WORKFLOW]
        assert names == [
            "stabilize_tissue",
            "grasp_needle",
            "orient_needle",
            "insert_needle",
            "pull_suture",
            "regrasp_needle",
            "tie_knot",
        ]

    def test_double_suture_workflow_has_9_steps(self):
        assert len(DOUBLE_SUTURE_WORKFLOW) == 9

    def test_skill_map_has_all_skills(self):
        assert len(SKILL_MAP) == 10

    def test_skill_map_has_bimanual_skill(self):
        assert "bimanual_stabilize" in SKILL_MAP

    def test_skill_map_values_are_classes(self):
        for name, cls in SKILL_MAP.items():
            assert hasattr(cls, "initialize")
            assert hasattr(cls, "tick")
            assert hasattr(cls, "reset")


class TestSurgicalPlanner:
    def test_initial_state(self):
        planner = SurgicalPlanner()
        assert planner.current_step == 0
        assert planner.total_steps == 0
        assert planner.plan == []

    def test_plan_interrupted_suture(self):
        planner = SurgicalPlanner()
        plan = planner.plan_interrupted_suture()
        assert len(plan) == 7
        assert plan[0][0] == "stabilize_tissue"
        assert plan[-1][0] == "tie_knot"

    def test_plan_double_suture(self):
        planner = SurgicalPlanner()
        plan = planner.plan_double_suture()
        assert len(plan) == 9

    def test_plan_from_goal_suture_keyword(self):
        planner = SurgicalPlanner()
        plan = planner.plan_from_goal("Place interrupted suture")
        assert len(plan) == 7
        assert planner.total_steps == 7

    def test_plan_from_goal_knot_keyword(self):
        planner = SurgicalPlanner()
        plan = planner.plan_from_goal("Tie a surgical knot")
        assert len(plan) == 7

    def test_plan_from_goal_double_keyword(self):
        planner = SurgicalPlanner()
        plan = planner.plan_from_goal("Double suture")
        assert len(plan) == 9

    def test_plan_from_goal_two_keyword(self):
        planner = SurgicalPlanner()
        plan = planner.plan_from_goal("Two sutures")
        assert len(plan) == 9

    def test_plan_from_goal_fallback_to_suture(self):
        planner = SurgicalPlanner()
        plan = planner.plan_from_goal("some unknown goal")
        assert len(plan) == 7  # defaults to interrupted suture

    def test_next_skill_returns_first(self):
        planner = SurgicalPlanner()
        planner.plan_interrupted_suture()
        name, params = planner.next_skill()
        assert name == "stabilize_tissue"
        assert planner.current_step == 1

    def test_next_skill_returns_none_when_done(self):
        planner = SurgicalPlanner()
        planner.plan_interrupted_suture()
        for _ in range(7):
            planner.next_skill()
        assert planner.next_skill() is None

    def test_next_skill_increments_step(self):
        planner = SurgicalPlanner()
        planner.plan_interrupted_suture()
        planner.next_skill()
        assert planner.current_step == 1
        planner.next_skill()
        assert planner.current_step == 2

    def test_peek_next_returns_next_without_advancing(self):
        planner = SurgicalPlanner()
        planner.plan_interrupted_suture()
        first = planner.peek_next()
        assert first == SUTURE_WORKFLOW[0]
        # step should NOT have advanced
        assert planner.current_step == 0

    def test_peek_next_returns_none_when_done(self):
        planner = SurgicalPlanner()
        planner.plan_interrupted_suture()
        for _ in range(7):
            planner.next_skill()
        assert planner.peek_next() is None

    def test_total_steps(self):
        planner = SurgicalPlanner()
        planner.plan_interrupted_suture()
        assert planner.total_steps == 7
        planner.next_skill()
        assert planner.total_steps == 7  # total steps doesn't change

    def test_get_progress(self):
        planner = SurgicalPlanner()
        planner.plan_interrupted_suture()
        planner.next_skill()
        progress = planner.get_progress()
        assert progress["step"] == 1
        assert progress["total"] == 7
        assert len(progress["plan"]) == 7
        assert progress["history"] == []

    def test_record_result(self):
        planner = SurgicalPlanner()
        result = SkillResult.success("All good")
        planner.record_result("grasp_needle", result)
        assert len(planner._history) == 1
        entry = planner._history[0]
        assert entry["skill"] == "grasp_needle"
        assert entry["status"] == "SUCCESS"
        assert entry["message"] == "All good"


class TestReplan:
    def test_replan_on_grasp_failure_inserts_regrasp(self):
        planner = SurgicalPlanner()
        planner.plan_interrupted_suture()
        # Advance past stabilize_tissue
        planner.next_skill()  # stabilize_tissue
        planner.next_skill()  # grasp_needle (step=2)

        result = SkillResult.failed("Needle slip detected")
        new_plan = planner.replan("grasp_needle", result)

        # Should have inserted regrasp at current step and a retry after
        assert len(new_plan) == 9  # original 7 + regrasp + retry
        # Check insertion at current step (index 2)
        inserted = new_plan[2]
        assert inserted[0] == "regrasp_needle"

    def test_replan_on_insert_failure_inserts_stabilize(self):
        planner = SurgicalPlanner()
        planner.plan_interrupted_suture()
        # Advance to insert_needle
        for _ in range(4):
            planner.next_skill()

        result = SkillResult.failed("Excessive force")
        new_plan = planner.replan("insert_needle", result)

        # Should have inserted stabilize_tissue at step 4
        assert len(new_plan) == 9
        assert new_plan[4][0] == "stabilize_tissue"

    def test_replan_adds_retry_after_inserted_skill(self):
        planner = SurgicalPlanner()
        planner.plan_interrupted_suture()
        planner.next_skill()  # stabilize_tissue (step=1)
        planner.next_skill()  # grasp_needle (step=2)

        result = SkillResult.failed("slip")
        planner.replan("grasp_needle", result)

        # After regrasp_needle at step 2, there should be a retry at step 3
        assert planner._plan[2][0] == "regrasp_needle"
        assert planner._plan[3][0] == "grasp_needle"  # retry

    def test_history_records_replan_outcomes(self):
        planner = SurgicalPlanner()
        planner.plan_interrupted_suture()
        planner.next_skill()  # stabilize_tissue
        planner.next_skill()  # grasp_needle

        fail_result = SkillResult.failed("Slip detected")
        planner.record_result("grasp_needle", fail_result)
        planner.replan("grasp_needle", fail_result)

        # History should have the failure
        assert len(planner._history) == 1
        assert planner._history[0]["status"] == "FAILED"
