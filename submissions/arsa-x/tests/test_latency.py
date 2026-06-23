"""Unit tests for LatencySimulator (communication delay simulation)."""

from __future__ import annotations

import mujoco
import numpy as np
import pytest

from src.control import LatencySimulator


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _all_qpos(data: mujoco.MjData) -> np.ndarray:
    """Return a copy of the current qpos array."""
    return data.qpos.copy()


def _set_qpos(data: mujoco.MjData, values: np.ndarray) -> None:
    """Overwrite data.qpos in-place."""
    data.qpos[:] = values


# ---------------------------------------------------------------------------
# Disabled state (delay ≤ 1 μs)
# ---------------------------------------------------------------------------

class TestDisabled:
    """When delay is zero or negligible, record_and_delay should be a no-op."""

    def test_zero_delay_disabled(self, model_and_data):
        model, data = model_and_data
        lat = LatencySimulator(model, data, delay_seconds=0.0)
        assert lat.enabled is False
        assert lat.delay == 0.0

    def test_very_small_delay_disabled(self, model_and_data):
        model, data = model_and_data
        lat = LatencySimulator(model, data, delay_seconds=1e-7)
        assert lat.enabled is False

    def test_no_op_when_disabled(self, model_and_data):
        model, data = model_and_data
        lat = LatencySimulator(model, data, delay_seconds=0.0)

        before = _all_qpos(data)
        data.qpos[0] = 999.0  # controller writes a new value
        lat.record_and_delay(0.01)
        # Should NOT touch qpos at all when disabled
        after = _all_qpos(data)
        assert after[0] == 999.0  # value should stay as written by controller

    def test_disabled_returns_no_buffer(self, model_and_data):
        model, data = model_and_data
        lat = LatencySimulator(model, data, delay_seconds=0.0)
        lat.record_and_delay(0.01)
        assert lat.queue_depth == 0


# ---------------------------------------------------------------------------
# Properties
# ---------------------------------------------------------------------------

class TestProperties:
    def test_delay_property(self, model_and_data):
        model, data = model_and_data
        lat = LatencySimulator(model, data, delay_seconds=0.2)
        assert lat.delay == 0.2

    def test_delay_clamped_to_zero(self, model_and_data):
        model, data = model_and_data
        lat = LatencySimulator(model, data, delay_seconds=-0.1)
        assert lat.delay == 0.0
        assert lat.enabled is False

    def test_enabled_property(self, model_and_data):
        model, data = model_and_data
        lat = LatencySimulator(model, data, delay_seconds=0.05)
        assert lat.enabled is True

    def test_queue_depth_starts_zero(self, model_and_data):
        model, data = model_and_data
        lat = LatencySimulator(model, data, delay_seconds=0.1)
        assert lat.queue_depth == 0

    def test_buffer_seconds_empty(self, model_and_data):
        model, data = model_and_data
        lat = LatencySimulator(model, data, delay_seconds=0.1)
        assert lat.buffer_seconds == 0.0


# ---------------------------------------------------------------------------
# Core delay behavior
# ---------------------------------------------------------------------------

class TestDelayBehavior:
    """Verify the command buffer actually introduces the expected lag."""

    def test_immediate_step_no_delay_yet(self, model_and_data):
        """On the first step, the only buffered command is applied immediately."""
        model, data = model_and_data
        initial = _all_qpos(data)

        lat = LatencySimulator(model, data, delay_seconds=0.1)
        mujoco.mj_forward(model, data)

        # Controller writes a change
        new_qpos = initial.copy()
        new_qpos[0] = 0.5
        _set_qpos(data, new_qpos)

        # Record & delay — first call populates the buffer and applies the only entry
        lat.record_and_delay(0.01)
        mujoco.mj_forward(model, data)

        assert data.qpos[0] == pytest.approx(0.5)

    def test_delay_shifts_command_by_exact_amount(self, model_and_data):
        """End-to-end: verify commands are delayed by the configured amount.

        Timeline (dt=0.05, delay=0.1):
          Call 1 (t=0.00): append A(0.10), apply A,        sim→0.05
          Call 2 (t=0.05): append B(0.15), 0.10>0.05→keep, apply A, sim→0.10
          Call 3 (t=0.10): append C(0.20), 0.10≤0.10→evict,apply B, sim→0.15
          Call 4 (t=0.15): append D(0.25), 0.15≤0.15→evict,apply C, sim→0.20
        """
        model, data = model_and_data
        initial = _all_qpos(data)
        dt = 0.05

        lat = LatencySimulator(model, data, delay_seconds=0.1)
        mujoco.mj_forward(model, data)

        cmd_a = initial.copy(); cmd_a[0] = 0.73
        cmd_b = initial.copy(); cmd_b[0] = 0.99
        cmd_c = initial.copy(); cmd_c[0] = 1.25
        cmd_d = initial.copy(); cmd_d[0] = 1.51

        # Call 1: write A, apply immediately
        _set_qpos(data, cmd_a)
        lat.record_and_delay(dt)
        assert data.qpos[0] == pytest.approx(0.73)

        # Call 2: write B, apply A (still delayed)
        _set_qpos(data, cmd_b)
        lat.record_and_delay(dt)
        assert data.qpos[0] == pytest.approx(0.73)

        # Call 3: write C, apply B (A evicted)
        _set_qpos(data, cmd_c)
        lat.record_and_delay(dt)
        assert data.qpos[0] == pytest.approx(0.99)

        # Call 4: write D, apply C (B evicted)
        _set_qpos(data, cmd_d)
        lat.record_and_delay(dt)
        assert data.qpos[0] == pytest.approx(1.25)

    def test_eviction_never_empties_buffer(self, model_and_data):
        """Even after many steps, the buffer should never drop to zero."""
        model, data = model_and_data
        lat = LatencySimulator(model, data, delay_seconds=0.1)

        for _ in range(100):
            lat.record_and_delay(0.01)
            assert lat.queue_depth >= 1, "Buffer should never be empty"


# ---------------------------------------------------------------------------
# Precise timing test — simpler, more reliable
# ---------------------------------------------------------------------------

class TestPreciseTiming:
    """Use a larger time step to make timing assertions unambiguous."""

    def test_delay_shifts_command_in_time(self, model_and_data):
        """Write LOW then HIGH with 50ms delay.

        Timeline (dt=0.02, delay=0.05):
          Call 1 (t=0.00): append low(0.05), apply low,          sim→0.02
          Call 2 (t=0.02): append high(0.07),0.05>0.02→keep,     apply low, sim→0.04
          Call 3 (t=0.04): append high(0.09),0.05>0.04→keep,     apply low, sim→0.06
          Call 4 (t=0.06): append high(0.11),0.05≤0.06→evict,    apply high,sim→0.08
        """
        model, data = model_and_data
        initial = _all_qpos(data)
        dt = 0.02

        lat = LatencySimulator(model, data, delay_seconds=0.05)
        mujoco.mj_forward(model, data)

        cmd_low = initial.copy();  cmd_low[0] = 0.1
        cmd_high = initial.copy(); cmd_high[0] = 0.9

        # Call 1: write LOW, apply immediately
        _set_qpos(data, cmd_low)
        lat.record_and_delay(dt)
        assert data.qpos[0] == pytest.approx(0.1)

        # Call 2: write HIGH, still apply LOW (delay not elapsed)
        _set_qpos(data, cmd_high)
        lat.record_and_delay(dt)
        assert data.qpos[0] == pytest.approx(0.1), \
            "Before delay expires, should still show old value"

        # Call 3: write HIGH, still apply LOW (delay still not elapsed)
        _set_qpos(data, cmd_high)
        lat.record_and_delay(dt)
        assert data.qpos[0] == pytest.approx(0.1), \
            "Before delay expires, should still show old value"

        # Call 4: write HIGH, apply HIGH (delay elapsed, low evicted)
        _set_qpos(data, cmd_high)
        lat.record_and_delay(dt)
        assert data.qpos[0] == pytest.approx(0.9), \
            "After delay expires, should show the delayed value"

    def test_delay_introduces_lag_vs_no_delay(self, model_and_data):
        """Side-by-side: delayed qpos should lag behind the no-delay version."""
        model, data = model_and_data
        initial = _all_qpos(data)
        data2 = mujoco.MjData(model)

        mujoco.mj_forward(model, data)
        mujoco.mj_forward(model, data2)

        no_delay = LatencySimulator(model, data, delay_seconds=0.0)
        with_delay = LatencySimulator(model, data2, delay_seconds=0.1)

        dt = 0.02

        for step in range(20):
            cmd = initial.copy()
            cmd[0] = 0.5 + step * 0.01

            _set_qpos(data, cmd)
            no_delay.record_and_delay(dt)
            mujoco.mj_forward(model, data)

            _set_qpos(data2, cmd)
            with_delay.record_and_delay(dt)
            mujoco.mj_forward(model, data2)

            if step < 3:
                continue  # buffer still filling

            # No-delay should always match the latest command
            assert abs(data.qpos[0] - cmd[0]) < 1e-6, \
                f"No-delay should show latest command at step {step}"

            # Delayed version should LAG (not match the latest command)
            # Unless the delay has not been reached yet on step 3 (t=0.06 < 0.1)
            # By step 8 (t=0.16 > 0.1), the lag should be visible
            lag = abs(data2.qpos[0] - cmd[0])
            if step >= 8:
                assert lag > 0.01, \
                    f"Step {step}: delayed should lag. Lag={lag:.4f}"


# ---------------------------------------------------------------------------
# Buffer management
# ---------------------------------------------------------------------------

class TestBufferManagement:
    def test_buffer_grows_then_stabilizes(self, model_and_data):
        """Buffer size should grow to (delay / dt) entries, then stabilize."""
        model, data = model_and_data
        lat = LatencySimulator(model, data, delay_seconds=0.1)
        dt = 0.01

        # Record many steps
        for _ in range(30):
            lat.record_and_delay(dt)

        # After 30 steps at 0.01s, sim_time = 0.30
        # The buffer should have roughly delay/dt = 10 entries
        # (since new entries keep coming and old ones get evicted)
        assert 5 < lat.queue_depth < 30
        assert lat.queue_depth <= 20  # reasonable upper bound

    def test_buffer_seconds_grows_to_delay(self, model_and_data):
        """buffer_seconds should approximate the configured delay once steady."""
        model, data = model_and_data
        lat = LatencySimulator(model, data, delay_seconds=0.1)
        dt = 0.01

        for _ in range(30):
            lat.record_and_delay(dt)

        # Once steady, buffer spans roughly delay seconds
        assert lat.buffer_seconds >= 0.08  # at least close to delay

    def test_buffer_never_empty_after_first_call(self, model_and_data):
        """The buffer should always have at least one entry once recording starts."""
        model, data = model_and_data
        lat = LatencySimulator(model, data, delay_seconds=0.1)

        lat.record_and_delay(0.01)
        assert lat.queue_depth >= 1

        # Run many steps — should never drop to 0
        for _ in range(100):
            lat.record_and_delay(0.01)
            assert lat.queue_depth >= 1, "Buffer should never be empty"


# ---------------------------------------------------------------------------
# Reset
# ---------------------------------------------------------------------------

class TestReset:
    def test_reset_clears_buffer(self, model_and_data):
        model, data = model_and_data
        lat = LatencySimulator(model, data, delay_seconds=0.1)
        for _ in range(10):
            lat.record_and_delay(0.01)
        assert lat.queue_depth > 0

        lat.reset()
        assert lat.queue_depth == 0
        assert lat.buffer_seconds == 0.0

    def test_reset_clears_max_queue(self, model_and_data):
        model, data = model_and_data
        lat = LatencySimulator(model, data, delay_seconds=0.1)
        for _ in range(5):
            lat.record_and_delay(0.01)
        stats_before = lat.get_stats()
        assert stats_before["max_queue"] > 0

        lat.reset()
        stats_after = lat.get_stats()
        assert stats_after["max_queue"] == 0

    def test_reset_restarts_sim_time(self, model_and_data):
        model, data = model_and_data
        lat = LatencySimulator(model, data, delay_seconds=0.1)
        for _ in range(10):
            lat.record_and_delay(0.01)
        stats_before = lat.get_stats()
        assert stats_before["buffer_s"] > 0.05

        lat.reset()
        stats_after = lat.get_stats()
        assert stats_after["buffer_s"] == 0.0


# ---------------------------------------------------------------------------
# get_stats
# ---------------------------------------------------------------------------

class TestStats:
    def test_stats_structure(self, model_and_data):
        model, data = model_and_data
        lat = LatencySimulator(model, data, delay_seconds=0.2)
        stats = lat.get_stats()

        assert isinstance(stats, dict)
        assert stats["delay_s"] == 0.2
        assert stats["delay_ms"] == 200.0
        assert stats["enabled"] is True
        assert stats["queue_depth"] == 0
        assert stats["buffer_s"] == 0.0
        assert stats["max_queue"] == 0

    def test_stats_after_some_steps(self, model_and_data):
        model, data = model_and_data
        lat = LatencySimulator(model, data, delay_seconds=0.1)
        for _ in range(10):
            lat.record_and_delay(0.01)

        stats = lat.get_stats()
        assert stats["queue_depth"] > 0
        assert stats["buffer_s"] > 0.0
        assert stats["max_queue"] > 0

    def test_stats_disabled(self, model_and_data):
        model, data = model_and_data
        lat = LatencySimulator(model, data, delay_seconds=0.0)
        stats = lat.get_stats()
        assert stats["enabled"] is False
        assert stats["delay_ms"] == 0.0


# ---------------------------------------------------------------------------
# Edge cases
# ---------------------------------------------------------------------------

class TestEdgeCases:
    def test_large_delay(self, model_and_data):
        """A 5-second delay should not crash and should accumulate a large buffer."""
        model, data = model_and_data
        lat = LatencySimulator(model, data, delay_seconds=5.0)
        for _ in range(50):
            lat.record_and_delay(0.01)
        # Buffer should be building up since 50*0.01 = 0.5 << 5.0
        assert lat.queue_depth == 50  # none evicted yet
        assert lat.buffer_seconds == pytest.approx(0.49, abs=0.01)

    def test_zero_delay_no_buffer(self, model_and_data):
        """Zero delay = no buffering at all."""
        model, data = model_and_data
        lat = LatencySimulator(model, data, delay_seconds=0.0)
        for _ in range(10):
            lat.record_and_delay(0.01)
        assert lat.queue_depth == 0
        assert lat.buffer_seconds == 0.0
        assert lat.get_stats()["max_queue"] == 0

    def test_reset_then_continue(self, model_and_data):
        """After reset, should work like a fresh instance."""
        model, data = model_and_data
        lat = LatencySimulator(model, data, delay_seconds=0.1)
        for _ in range(10):
            lat.record_and_delay(0.01)
        assert lat.queue_depth > 0

        lat.reset()
        assert lat.queue_depth == 0

        # Continue recording
        lat.record_and_delay(0.01)
        assert lat.queue_depth == 1
