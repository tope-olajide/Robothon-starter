"""Unit tests for SkillBase, SkillResult, and SkillStatus."""

from __future__ import annotations

import math

import mujoco
import numpy as np
import pytest

from src.skills import SkillBase, SkillResult, SkillStatus


# ===========================================================================
# SkillResult & SkillStatus
# ===========================================================================


class TestSkillStatus:
    def test_enum_values(self):
        assert SkillStatus.IDLE.value == 1
        assert SkillStatus.RUNNING.value == 2
        assert SkillStatus.SUCCESS.value == 3
        assert SkillStatus.FAILED.value == 4

    def test_enum_unique(self):
        values = [s.value for s in SkillStatus]
        assert len(values) == len(set(values))


class TestSkillResult:
    def test_success_creates_success_result(self):
        r = SkillResult.success("All good", score=95)
        assert r.status == SkillStatus.SUCCESS
        assert r.message == "All good"
        assert r.data == {"score": 95}

    def test_failed_creates_failed_result(self):
        r = SkillResult.failed("Something broke", error_code=-1)
        assert r.status == SkillStatus.FAILED
        assert r.message == "Something broke"
        assert r.data == {"error_code": -1}

    def test_running_creates_running_result(self):
        r = SkillResult.running("In progress")
        assert r.status == SkillStatus.RUNNING
        assert r.message == "In progress"
        assert r.data == {}

    def test_default_message_empty(self):
        r = SkillResult.success()
        assert r.message == ""

    def test_default_data_empty_dict(self):
        r = SkillResult.failed()
        assert r.data == {}


# ===========================================================================
# SkillBase helpers (lerp, smoothstep, set_joint)
# ===========================================================================


class TestSkillBaseHelpers:
    """Test the shared math and joint helpers on SkillBase."""

    def test_lerp_at_zero(self, model_and_data):
        model, data = model_and_data
        skill = SkillBase(model, data, name="test")
        skill.initialize(duration=1.0)
        assert skill.lerp(10.0, 20.0) == 10.0

    def test_lerp_at_half(self, model_and_data):
        model, data = model_and_data
        skill = SkillBase(model, data, name="test")
        skill.initialize(duration=1.0)
        skill._progress = 0.5
        assert skill.lerp(10.0, 20.0) == 15.0

    def test_lerp_at_one(self, model_and_data):
        model, data = model_and_data
        skill = SkillBase(model, data, name="test")
        skill.initialize(duration=1.0)
        skill._progress = 1.0
        assert skill.lerp(10.0, 20.0) == 20.0

    def test_smoothstep_before_edge(self, model_and_data):
        model, data = model_and_data
        skill = SkillBase(model, data, name="test")
        assert skill.smoothstep(0.2, 0.8, 0.1) == 0.0

    def test_smoothstep_after_edge(self, model_and_data):
        model, data = model_and_data
        skill = SkillBase(model, data, name="test")
        assert skill.smoothstep(0.2, 0.8, 0.9) == 1.0

    def test_smoothstep_at_midpoint(self, model_and_data):
        model, data = model_and_data
        skill = SkillBase(model, data, name="test")
        val = skill.smoothstep(0.0, 1.0, 0.5)
        assert val == pytest.approx(0.5)  # x^2 * (3-2x) at x=0.5 = 0.5

    def test_body_pos_found(self, model_and_data):
        model, data = model_and_data
        skill = SkillBase(model, data, name="test")
        pos = skill.body_pos("needle")
        assert len(pos) == 3
        assert np.allclose(pos, [0.5, 0.0, 0.5])

    def test_body_pos_not_found(self, model_and_data):
        model, data = model_and_data
        skill = SkillBase(model, data, name="test")
        pos = skill.body_pos("nonexistent_body")
        assert np.allclose(pos, [0.0, 0.0, 0.0])

    def test_set_joint_writes_to_data_qpos(self, model_and_data):
        model, data = model_and_data
        skill = SkillBase(model, data, name="test")
        skill.set_joint("joint1", 0.5)
        # Read back through MuJoCo ID lookup
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "joint1")
        qa = int(model.jnt_qposadr[jid])
        assert data.qpos[qa] == pytest.approx(0.5)

    def test_set_joint_clamps_to_limits(self, model_and_data):
        model, data = model_and_data
        skill = SkillBase(model, data, name="test")
        # joint1 range is [-2.9, 2.9] — try setting beyond
        skill.set_joint("joint1", 100.0)
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "joint1")
        qa = int(model.jnt_qposadr[jid])
        assert data.qpos[qa] == pytest.approx(2.9)

    def test_set_joint_nonexistent_name(self, model_and_data):
        model, data = model_and_data
        skill = SkillBase(model, data, name="test")
        # Should not raise
        skill.set_joint("not_a_joint", 1.0)

    def test_set_joints_batch(self, model_and_data):
        model, data = model_and_data
        skill = SkillBase(model, data, name="test")
        skill.set_joints({"joint1": 0.3, "joint2": -0.2})
        jid1 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "joint1")
        jid2 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "joint2")
        assert data.qpos[int(model.jnt_qposadr[jid1])] == pytest.approx(0.3)
        assert data.qpos[int(model.jnt_qposadr[jid2])] == pytest.approx(-0.2)

    def test_get_joint_reads_correctly(self, model_and_data):
        model, data = model_and_data
        skill = SkillBase(model, data, name="test")
        # Set then read
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "joint3")
        data.qpos[int(model.jnt_qposadr[jid])] = 0.7
        assert skill._get_joint("joint3") == pytest.approx(0.7)

    def test_get_joint_nonexistent(self, model_and_data):
        model, data = model_and_data
        skill = SkillBase(model, data, name="test")
        assert skill._get_joint("nosuchjoint") == 0.0


# ===========================================================================
# SkillBase lifecycle
# ===========================================================================


class ConcreteTestSkill(SkillBase):
    """A minimal concrete skill for testing lifecycle."""

    def __init__(self, model, data, name=""):
        super().__init__(model, data, name)
        self.tick_count = 0
        self.finish_called = False

    def initialize(self, **params):
        super().initialize(**params)
        self.tick_count = 0
        self.finish_called = False

    def on_tick(self, dt: float) -> SkillResult:
        self.tick_count += 1
        # Move joint1 from 0 → 1.0 over the duration
        self.lerp_joint("joint1", 0.0, 1.0)
        return super().on_tick(dt)

    def on_finish(self) -> SkillResult:
        self.finish_called = True
        return SkillResult.success("Test skill completed")


class TestSkillBaseLifecycle:
    def test_initial_state_is_idle(self, model_and_data):
        model, data = model_and_data
        skill = ConcreteTestSkill(model, data)
        assert skill.status == SkillStatus.IDLE
        assert skill._progress == 0.0

    def test_initialize_sets_running(self, model_and_data):
        model, data = model_and_data
        skill = ConcreteTestSkill(model, data, name="test_skill")
        skill.initialize(duration=0.5)
        assert skill.status == SkillStatus.RUNNING
        assert skill.name == "test_skill"
        assert skill._duration == pytest.approx(0.5)

    def test_tick_advances_progress(self, model_and_data):
        model, data = model_and_data
        skill = ConcreteTestSkill(model, data)
        skill.initialize(duration=1.0)

        skill.tick(0.3)
        assert skill._progress == pytest.approx(0.3)
        assert skill._time_elapsed == pytest.approx(0.3)

        skill.tick(0.3)
        assert skill._progress == pytest.approx(0.6)

    def test_tick_calls_on_tick(self, model_and_data):
        model, data = model_and_data
        skill = ConcreteTestSkill(model, data)
        skill.initialize(duration=1.0)
        skill.tick(0.5)
        assert skill.tick_count == 1

    def test_tick_completes_at_end(self, model_and_data):
        model, data = model_and_data
        skill = ConcreteTestSkill(model, data)
        skill.initialize(duration=1.0)

        result = skill.tick(0.5)
        assert result.status == SkillStatus.RUNNING

        result = skill.tick(0.5)  # now _progress = 1.0
        assert result.status == SkillStatus.SUCCESS
        assert skill.finish_called
        assert result.message == "Test skill completed"

    def test_tick_caps_progress_at_one(self, model_and_data):
        model, data = model_and_data
        skill = ConcreteTestSkill(model, data)
        skill.initialize(duration=1.0)
        skill.tick(2.0)  # overshoot
        assert skill._progress == 1.0

    def test_reset_returns_to_idle(self, model_and_data):
        model, data = model_and_data
        skill = ConcreteTestSkill(model, data)
        skill.initialize(duration=1.0)
        skill.tick(0.5)
        skill.reset()
        assert skill.status == SkillStatus.IDLE
        assert skill._progress == 0.0
        assert skill._time_elapsed == 0.0

    def test_can_run_default_true(self, model_and_data):
        model, data = model_and_data
        skill = ConcreteTestSkill(model, data)
        assert skill.can_run() is True

    def test_lerp_joint_moves_joint(self, model_and_data):
        model, data = model_and_data
        skill = ConcreteTestSkill(model, data)
        skill.initialize(duration=1.0)
        skill._progress = 0.5
        skill.lerp_joint("joint1", 0.0, 1.0)
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "joint1")
        assert data.qpos[int(model.jnt_qposadr[jid])] == pytest.approx(0.5)

    def test_lerp_joints(self, model_and_data):
        model, data = model_and_data
        skill = ConcreteTestSkill(model, data)
        skill.initialize(duration=1.0)
        skill._progress = 0.3
        skill.lerp_joints(
            {"joint1": 0.0, "joint2": 0.0},
            {"joint1": 1.0, "joint2": -1.0},
        )
        # smoothstep(0.3) = 0.3² * (3 - 2*0.3) = 0.216
        eased = 0.3 * 0.3 * (3.0 - 2.0 * 0.3)
        jid1 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "joint1")
        jid2 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "joint2")
        assert data.qpos[int(model.jnt_qposadr[jid1])] == pytest.approx(eased)
        assert data.qpos[int(model.jnt_qposadr[jid2])] == pytest.approx(-eased)

    def test_on_finish_base_returns_success(self, model_and_data):
        model, data = model_and_data
        skill = SkillBase(model, data)
        # Directly call on_finish (it's the default hook)
        result = skill.on_finish()
        assert result.status == SkillStatus.SUCCESS
        assert "completed" in result.message
