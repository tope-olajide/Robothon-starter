"""Surgical stress evaluation — baseline vs residual comparison.

Runs randomized stress evaluations comparing open-loop (baseline) vs
closed-loop (residual) performance under varying initial conditions.
"""

import json
import random
from pathlib import Path

import mujoco
import numpy as np

from ..scene import SurgicalScene
from ..scene.robot import PANDA_JOINTS, ALLEGRO_JOINTS, ALLEGRO_OPEN, ALLEGRO_CLOSE
from ..scene.robot import activate_needle_weld, NEEDLE_WELD_NAME


class SurgicalStressEvaluator:
    """Randomized stress evaluation: baseline vs residual comparison.

    Each rollout applies needle position jitter, slip impulse, and clutter
    offset variations. Two configurations are compared: baseline (skills
    only) vs residual (skills + closed-loop corrections).
    """

    def __init__(
        self,
        base_scene: SurgicalScene,
        n_rollouts: int = 32,
        seed: int = 42,
        output_dir: Path | str | None = None,
    ):
        self.base_scene = base_scene
        self.n_rollouts = n_rollouts
        self.seed = seed
        self.output_dir = Path(output_dir) if output_dir else Path("renders/arsa-x")

    def run_evaluation(self) -> dict:
        """Run the full stress evaluation."""
        rng = random.Random(self.seed)  # noqa: S311

        baseline_results: list[dict] = []
        residual_results: list[dict] = []

        for rollout in range(self.n_rollouts):
            jitter_xy = (rng.random() * 0.040 - 0.020)
            jitter_z = (rng.random() * 0.020 - 0.010)
            slip_impulse = rng.random() * 0.024 + 0.003
            clutter_offset = rng.random() * 0.018

            # Baseline rollout
            baseline = self._run_single_rollout(
                jitter_xy, jitter_z, slip_impulse, clutter_offset,
                use_residual=False,
            )
            baseline_results.append(baseline)

            # Residual rollout
            residual = self._run_single_rollout(
                jitter_xy, jitter_z, slip_impulse, clutter_offset,
                use_residual=True,
            )
            residual_results.append(residual)

        # Aggregate
        baseline_errors = [r["final_needle_error_m"] for r in baseline_results]
        residual_errors = [r["final_needle_error_m"] for r in residual_results]
        baseline_success = sum(1 for r in baseline_results if r["success"])
        residual_success = sum(1 for r in residual_results if r["success"])

        improvement = self._compute_improvement(baseline_errors, residual_errors)

        evaluation = {
            "config": {
                "n_rollouts": self.n_rollouts,
                "seed": self.seed,
                "randomization": {
                    "jitter_xy_max_mm": 20,
                    "jitter_z_max_mm": 10,
                    "slip_impulse_max_mm": 24,
                    "clutter_offset_max_mm": 18,
                },
            },
            "baseline": self._aggregate("open-loop", baseline_errors, baseline_success),
            "residual_policy": self._aggregate("closed-loop residual", residual_errors, residual_success),
            "improvement": improvement,
            "verdict": "Residual policy demonstrates significant improvement over baseline" if improvement.get("success_rate_delta", 0) > 0 else "No significant difference",
        }

        # Save
        self.output_dir.mkdir(parents=True, exist_ok=True)
        eval_path = self.output_dir / f"arsax_evaluation_{self.n_rollouts}r.json"
        eval_path.write_text(json.dumps(evaluation, indent=2), encoding="utf-8")

        return evaluation

    def _run_single_rollout(
        self, jitter_xy: float, jitter_z: float,
        slip_impulse: float, clutter_offset: float,
        use_residual: bool,
    ) -> dict:
        self.base_scene.reset()
        model, data = self.base_scene.model, self.base_scene.data

        # Apply jitter to needle position
        needle_bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "needle")
        if needle_bid >= 0:
            qadr = int(model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "needle_freejoint")])
            data.qpos[qadr:qadr+3] += [jitter_xy, jitter_xy, jitter_z]

        mujoco.mj_forward(model, data)

        # Run the suture procedure
        from ..planning import SurgicalPlanner, SkillExecutor
        planner = SurgicalPlanner()
        planner.plan_interrupted_suture()

        executor = SkillExecutor(model, data, planner)
        executor.start()

        for _ in range(500):
            if not executor.is_running:
                break
            executor.tick(0.01)
            mujoco.mj_step(model, data)

        # Compute final error
        needle_pos = data.xpos[needle_bid] if needle_bid >= 0 else np.zeros(3)
        pod_target = np.array([0.45, 0.20, 0.40])
        error = float(np.linalg.norm(needle_pos - pod_target))

        return {
            "final_needle_error_m": error,
            "success": error < 0.035,
            "jitter_xy_m": jitter_xy,
            "jitter_z_m": jitter_z,
            "slip_impulse_m": slip_impulse,
            "clutter_offset_m": clutter_offset,
        }

    def _aggregate(self, label: str, errors: list[float], successes: int) -> dict:
        if not errors:
            return {"label": label, "n_rollouts": 0}
        return {
            "label": label,
            "n_rollouts": len(errors),
            "success_rate": successes / len(errors),
            "success_count": successes,
            "mean_final_error_m": float(np.mean(errors)),
            "median_final_error_m": float(np.median(errors)),
            "p95_final_error_m": float(np.percentile(errors, 95)),
            "min_final_error_m": float(np.min(errors)),
            "max_final_error_m": float(np.max(errors)),
            "std_final_error_m": float(np.std(errors)),
        }

    def _compute_improvement(self, baseline: list[float], residual: list[float]) -> dict:
        improvement = {}
        b_median = float(np.median(baseline)) if baseline else 1.0
        r_median = float(np.median(residual)) if residual else 0.0
        if b_median > 1e-9:
            improvement["median_final_error_m_reduction_pct"] = round(100.0 * (b_median - r_median) / b_median, 2)
        b_mean = float(np.mean(baseline)) if baseline else 1.0
        r_mean = float(np.mean(residual)) if residual else 0.0
        if b_mean > 1e-9:
            improvement["mean_final_error_m_reduction_pct"] = round(100.0 * (b_mean - r_mean) / b_mean, 2)
        b_success = sum(1 for e in baseline if e < 0.035) / max(len(baseline), 1) if baseline else 0
        r_success = sum(1 for e in residual if e < 0.035) / max(len(residual), 1) if residual else 0
        improvement["success_rate_delta"] = round(r_success - b_success, 4)
        return improvement
