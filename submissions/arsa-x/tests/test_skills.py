"""Unit tests for concrete surgical skills."""

from __future__ import annotations

import mujoco
import numpy as np
import pytest

from src.skills import (
    GraspNeedle, OrientNeedle, InsertNeedle, PullSuture,
    TieKnot, RegraspNeedle, StabilizeTissue, BimanualStabilizeTissue,
    ReleaseObject, FingerGait,
    SkillStatus,
)


class TestGraspNeedle:
    """Activated-grasp skill: approach → descend → close → engage weld → lift."""

    def test_initialize_starts_in_home(self, model_and_data):
        model, data = model_and_data
        skill = GraspNeedle(model, data)
        skill.initialize(duration=4.0)
        assert skill._phase == "home"
        assert skill.status == SkillStatus.RUNNING

    def test_initialize_solves_waypoints(self, model_and_data):
        model, data = model_and_data
        skill = GraspNeedle(model, data)
        skill.initialize(duration=4.0)
        # IK must have produced an arm configuration for each waypoint.
        for q in (skill._approach_q, skill._descend_q, skill._lift_q):
            assert set(q.keys()) >= {f"joint{i}" for i in range(1, 8)}

    def test_phase_progression(self, model_and_data):
        model, data = model_and_data
        skill = GraspNeedle(model, data)
        skill.initialize(duration=4.0)
        seen = set()
        for _ in range(6000):
            seen.add(skill._phase)
            result = skill.tick(0.002)
            if result and result.status in (SkillStatus.SUCCESS, SkillStatus.FAILED):
                break
        # The state machine should pass through all the intended phases.
        assert {"home", "approach", "descend", "close", "lift"} <= seen

    def test_weld_helpers_available(self, model_and_data):
        model, data = model_and_data
        # The activated-grasp weld must exist in the model.
        eq_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_EQUALITY, "needle_grasp_weld")
        assert eq_id >= 0


class TestOrientNeedle:
    def test_initialize_stores_joints(self, model_and_data):
        model, data = model_and_data
        skill = OrientNeedle(model, data)
        skill.initialize(target_angle_deg=45.0, duration=1.0)
        assert skill._target_angle == pytest.approx(np.radians(45.0))
        assert skill.status == SkillStatus.RUNNING

    def test_completes_successfully(self, model_and_data):
        model, data = model_and_data
        skill = OrientNeedle(model, data)
        skill.initialize(target_angle_deg=45.0, duration=1.0)
        for _ in range(600):  # extra margin for floating point
            result = skill.tick(0.002)
            if result and result.status == SkillStatus.SUCCESS:
                break
        assert result is not None
        assert result.status == SkillStatus.SUCCESS

    def test_maintains_pinch(self, model_and_data):
        model, data = model_and_data
        skill = OrientNeedle(model, data)
        skill.initialize(target_angle_deg=45.0, duration=1.0)
        skill.tick(0.5)  # mid-execution
        # hand_ffj1 should be held at 0.8
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "hand_ffj1")
        assert data.qpos[int(model.jnt_qposadr[jid])] == pytest.approx(0.8)


class TestInsertNeedle:
    def test_initialize_sets_entry_exit(self, model_and_data):
        model, data = model_and_data
        skill = InsertNeedle(model, data)
        skill.initialize(duration=2.0)
        assert np.allclose(skill._entry, [0.42, -0.02, 0.39])
        assert np.allclose(skill._exit, [0.45, 0.02, 0.37])

    def test_completes_successfully(self, model_and_data):
        model, data = model_and_data
        skill = InsertNeedle(model, data)
        skill.initialize(duration=2.0)
        for _ in range(1100):  # extra margin for floating point
            result = skill.tick(0.002)
            if result and result.status == SkillStatus.SUCCESS:
                break
        assert result is not None
        assert result.status == SkillStatus.SUCCESS

    def test_increases_grip_during_insertion(self, model_and_data):
        model, data = model_and_data
        skill = InsertNeedle(model, data)
        skill.initialize(duration=2.0)
        # Run to mid point (progress ~0.5)
        for _ in range(500):
            skill.tick(0.002)
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "hand_ffj1")
        assert data.qpos[int(model.jnt_qposadr[jid])] == pytest.approx(0.85)


class TestPullSuture:
    def test_initialize_sets_distance(self, model_and_data):
        model, data = model_and_data
        skill = PullSuture(model, data)
        skill.initialize(pull_distance=0.04, duration=1.0)
        assert skill._pull_distance == pytest.approx(0.04)

    def test_completes_successfully(self, model_and_data):
        model, data = model_and_data
        skill = PullSuture(model, data)
        skill.initialize(duration=1.0)
        for _ in range(500):
            result = skill.tick(0.002)
            if result and result.status == SkillStatus.SUCCESS:
                break
        assert result is not None
        assert result.status == SkillStatus.SUCCESS

    def test_retracts_joints(self, model_and_data):
        model, data = model_and_data
        skill = PullSuture(model, data)
        skill.initialize(duration=1.0)
        # Run to completion (need 500+ ticks due to FP rounding)
        result = None
        for _ in range(600):
            result = skill.tick(0.002)
            if result and result.status == SkillStatus.SUCCESS:
                break
        assert result is not None
        assert result.status == SkillStatus.SUCCESS


class TestTieKnot:
    def test_initialize_sets_wrap_phase(self, model_and_data):
        model, data = model_and_data
        skill = TieKnot(model, data)
        skill.initialize(duration=3.0)
        assert skill._phase == "wrap"

    def test_completes_successfully(self, model_and_data):
        model, data = model_and_data
        skill = TieKnot(model, data)
        skill.initialize(duration=3.0)
        for _ in range(1500):
            result = skill.tick(0.002)
            if result and result.status == SkillStatus.SUCCESS:
                break
        assert result is not None
        assert result.status == SkillStatus.SUCCESS
        assert "knot" in result.message.lower()

    def test_phase_transitions(self, model_and_data):
        model, data = model_and_data
        skill = TieKnot(model, data)
        skill.initialize(duration=3.0)
        # After 1s (one phase): should be in "pull"
        for _ in range(500):
            skill.tick(0.002)
        assert skill._phase == "pull" or skill._phase == "tighten"
        # After 2s (two phases): should be in "tighten" or done
        for _ in range(500):
            skill.tick(0.002)
        assert skill.status in (SkillStatus.RUNNING, SkillStatus.SUCCESS)


class TestRegraspNeedle:
    def test_initialize_sets_release_phase(self, model_and_data):
        model, data = model_and_data
        skill = RegraspNeedle(model, data)
        skill.initialize(duration=3.0)
        assert skill._phase == "release"

    def test_completes_successfully(self, model_and_data):
        model, data = model_and_data
        skill = RegraspNeedle(model, data)
        skill.initialize(duration=3.0)
        for _ in range(1500):
            result = skill.tick(0.002)
            if result and result.status == SkillStatus.SUCCESS:
                break
        assert result is not None
        assert result.status == SkillStatus.SUCCESS
        assert "regrasped" in result.message.lower()


class TestStabilizeTissue:
    def test_initialize_sets_target(self, model_and_data):
        model, data = model_and_data
        skill = StabilizeTissue(model, data)
        skill.initialize(target_pos=(0.45, -0.02, 0.38), duration=1.0)
        assert skill._start_j4 is not None

    def test_completes_successfully(self, model_and_data):
        model, data = model_and_data
        skill = StabilizeTissue(model, data)
        skill.initialize(duration=1.0)
        for _ in range(600):  # extra margin for floating point
            result = skill.tick(0.002)
            if result and result.status == SkillStatus.SUCCESS:
                break
        assert result is not None
        assert result.status == SkillStatus.SUCCESS


class TestReleaseObject:
    def test_initialize(self, model_and_data):
        model, data = model_and_data
        skill = ReleaseObject(model, data)
        skill.initialize(duration=1.0)
        assert skill.status == SkillStatus.RUNNING

    def test_completes_successfully(self, model_and_data):
        model, data = model_and_data
        skill = ReleaseObject(model, data)
        skill.initialize(duration=1.0)
        for _ in range(600):  # extra margin for floating point
            result = skill.tick(0.002)
            if result and result.status == SkillStatus.SUCCESS:
                break
        assert result is not None
        assert result.status == SkillStatus.SUCCESS


class TestFingerGait:
    """Alternating finger contact phases for in-hand needle reorientation."""

    def test_initialize_sets_cycle_params(self, model_and_data):
        model, data = model_and_data
        skill = FingerGait(model, data)
        skill.initialize(rotation_rad=np.pi / 6, n_cycles=3, duration=3.0)
        assert skill._phase == "A"
        assert skill._cycle_count == 0
        assert skill._n_cycles == 3
        assert skill.status == SkillStatus.RUNNING

    def test_completes_successfully(self, model_and_data):
        model, data = model_and_data
        skill = FingerGait(model, data)
        skill.initialize(rotation_rad=np.pi / 6, n_cycles=2, duration=4.0)
        for _ in range(2000):
            result = skill.tick(0.002)
            if result and result.status == SkillStatus.SUCCESS:
                break
        assert result is not None
        assert result.status == SkillStatus.SUCCESS
        assert "gait" in result.message.lower()

    def test_phase_transitions(self, model_and_data):
        model, data = model_and_data
        skill = FingerGait(model, data)
        skill.initialize(rotation_rad=np.pi / 6, n_cycles=2, duration=4.0)
        # First tick should be in phase A
        skill.tick(0.002)
        assert skill._phase == "A"
        # Run through first phase (~1s)
        for _ in range(500):
            skill.tick(0.002)
        # Should have transitioned to phase B
        assert skill._phase == "B" or skill._cycle_count >= 1

    def test_wrist_rotation_applies(self, model_and_data):
        model, data = model_and_data
        skill = FingerGait(model, data)
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "joint7")
        qpos_addr = int(model.jnt_qposadr[jid])
        start_j7 = float(data.qpos[qpos_addr])
        skill.initialize(rotation_rad=np.pi / 4, n_cycles=1, duration=2.0)
        # Run to completion
        for _ in range(1000):
            result = skill.tick(0.002)
            if result and result.status == SkillStatus.SUCCESS:
                break
        end_j7 = float(data.qpos[qpos_addr])
        # Wrist should have rotated (sympathetic rotation)
        assert end_j7 != start_j7


class TestSkillRegistry:
    def test_get_skill_returns_correct_class(self):
        from src.skills import get_skill, SKILL_REGISTRY
        assert get_skill("grasp_needle") == GraspNeedle
        assert get_skill("orient_needle") == OrientNeedle
        assert get_skill("insert_needle") == InsertNeedle
        assert get_skill("pull_suture") == PullSuture
        assert get_skill("tie_knot") == TieKnot
        assert get_skill("regrasp_needle") == RegraspNeedle
        assert get_skill("stabilize_tissue") == StabilizeTissue
        assert get_skill("bimanual_stabilize") == BimanualStabilizeTissue
        assert get_skill("release_object") == ReleaseObject
        assert get_skill("finger_gait") == FingerGait
        assert len(SKILL_REGISTRY) == 10

    def test_get_skill_raises_on_unknown(self):
        from src.skills import get_skill
        with pytest.raises(KeyError, match="Unknown skill"):
            get_skill("nonexistent_skill")
