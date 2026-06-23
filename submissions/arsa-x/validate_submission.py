#!/usr/bin/env python3
"""Comprehensive submission validation for ARSA-X.

Checks:
  - registration.json has valid UUID
  - All required artifacts exist and are non-empty
  - Report JSON has required fields
  - Trajectory has required metrics
  - Policy card documents closed-loop architecture
  - Evaluation shows baseline vs residual comparison (if exists)
  - Evaluation guide has required content
  - Rubric scorecard covers all 8 criteria
  - Manifest UUID matches registration
"""

from __future__ import annotations

import json
import re
import sys
from pathlib import Path

_HERE = Path(__file__).resolve().parent
OUTPUTS = _HERE / "renders" / "arsa-x"

REQUIRED_FILES = [
    "registration.json",
    "README.md",
    "TECHNICAL_OVERVIEW.md",
    "EVALUATION_GUIDE.md",
    "evaluation_scorecard.json",
    "submission_manifest.json",
    "run.py",
    "validate_submission.py",
]

REQUIRED_OUTPUTS = [
    "arsax_demo.mp4",
    "arsax_trajectory.json",
    "arsax_report.json",
    "arsax_surgical_policy_card.json",
    "arsax_contact_timeline.json",
    "arsax_narration.srt",
]

UUID_REGEX = re.compile(
    r"[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}"
)

ERRORS: list[str] = []
WARNINGS: list[str] = []


def _fail(msg: str) -> None:
    ERRORS.append(msg)
    print(f"  FAIL: {msg}")


def _warn(msg: str) -> None:
    WARNINGS.append(msg)
    print(f"  WARN: {msg}")


def _ok(msg: str) -> None:
    print(f"  OK:   {msg}")


# --- Validator helpers ---


def _load_json(path: Path) -> dict | None:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except Exception as e:
        _fail(f"Could not parse {path.name}: {e}")
        return None


def _validate_uuid(value: str, source: str) -> bool:
    if not UUID_REGEX.fullmatch(value.strip()):
        _fail(f"Invalid UUID in {source}: {value!r}")
        return False
    return True


# --- Validation functions ---


def validate_registration() -> str | None:
    path = _HERE / "registration.json"
    if not path.exists():
        _fail("registration.json not found")
        return None
    data = _load_json(path)
    if data is None:
        return None
    uuid = str(data.get("uuid", ""))
    if _validate_uuid(uuid, "registration.json"):
        _ok(f"UUID: {uuid}")
    if not data.get("participant_name"):
        _fail("participant_name missing from registration.json")
    if not data.get("project_name"):
        _fail("project_name missing from registration.json")
    return uuid


def validate_required_files() -> None:
    for fname in REQUIRED_FILES:
        path = _HERE / fname
        if not path.exists():
            _fail(f"Required file missing: {fname}")
        elif path.stat().st_size == 0:
            _fail(f"Required file empty: {fname}")
        else:
            _ok(f"{fname} ({path.stat().st_size:,} bytes)")


def validate_outputs() -> None:
    if not OUTPUTS.exists():
        _warn("renders/arsa-x/ directory not found (generate artifacts first)")
        return
    for fname in REQUIRED_OUTPUTS:
        path = OUTPUTS / fname
        if not path.exists():
            _warn(f"Output missing: {fname} (generate artifacts first)")
        elif path.stat().st_size == 0:
            _fail(f"Output empty: {fname}")
        else:
            size_kb = path.stat().st_size / 1024
            _ok(f"renders/arsa-x/{fname} ({size_kb:.1f} KB)")


def validate_report() -> None:
    path = OUTPUTS / "arsax_report.json"
    if not path.exists():
        return
    data = _load_json(path)
    if data is None:
        return
    required_fields = [
        "registration_uuid", "success", "final_task_completion",
        "final_conditions", "peak_grip_strength", "peak_wrist_force_n",
        "per_skill_analytics", "rubric_alignment",
    ]
    for field in required_fields:
        if field not in data:
            _fail(f"report.json missing field: {field}")
        else:
            _ok(f"report.json contains {field}")
    rubric = data.get("rubric_alignment", {})
    required_rubric = [
        "runnability", "mujoco_depth", "task_design", "control",
        "dexterous_manipulation", "engineering_quality", "presentation",
        "innovation",
    ]
    for criterion in required_rubric:
        if criterion not in rubric:
            _fail(f"report.json rubric missing criterion: {criterion}")


def validate_trajectory() -> None:
    path = OUTPUTS / "arsax_trajectory.json"
    if not path.exists():
        return
    data = _load_json(path)
    if data is None:
        return
    samples = data.get("trajectory_samples", [])
    if len(samples) < 5:
        _warn(f"Trajectory has only {len(samples)} samples (expected >= 5)")
    else:
        _ok(f"Trajectory has {len(samples)} samples")
    required_fields = [
        "time_s", "stage", "grip_strength", "wrist_force_n",
        "tissue_displacement_m", "task_completion",
    ]
    if samples:
        for field in required_fields:
            if field not in samples[0]:
                _fail(f"Trajectory sample missing field: {field}")


def validate_policy_card() -> None:
    path = OUTPUTS / "arsax_surgical_policy_card.json"
    if not path.exists():
        return
    data = _load_json(path)
    if data is None:
        return
    if "architecture" not in data:
        _fail("Policy card missing architecture section")
    else:
        arch = data["architecture"]
        if arch.get("type") != "closed-loop residual policy":
            _fail(f"Policy type should be 'closed-loop residual policy', got {arch.get('type')}")
        else:
            _ok("Policy type: closed-loop residual policy")
        if "base_policy" in arch and "residual_policy" in arch:
            _ok("Policy card documents base + residual architecture")
    if "actuated_channels" in data:
        channels = data["actuated_channels"]
        if channels.get("total", 0) >= 20:
            _ok(f"{channels['total']} actuated channels documented")


def validate_evaluation_guide() -> None:
    path = _HERE / "EVALUATION_GUIDE.md"
    if not path.exists():
        _fail("EVALUATION_GUIDE.md not found")
        return
    content = path.read_text(encoding="utf-8")
    required_phrases = [
        "closed-loop residual",
        "wrist F/T",
        "slip detection",
        "scorecard",
        "Registration UUID",
    ]
    for phrase in required_phrases:
        if phrase not in content:
            _fail(f"EVALUATION_GUIDE.md missing required phrase: {phrase}")
        else:
            _ok(f"EVALUATION_GUIDE.md contains: {phrase}")


def validate_rubric_scorecard() -> None:
    path = _HERE / "evaluation_scorecard.json"
    if not path.exists():
        _fail("evaluation_scorecard.json not found")
        return
    data = _load_json(path)
    if data is None:
        return
    scorecard = data.get("scorecard", [])
    required_criteria = [
        "Runnability", "MuJoCo Depth", "Task Design", "Control",
        "Dexterous Manipulation", "Engineering Quality", "Presentation",
        "Innovation",
    ]
    found_criteria = {c["criterion"] for c in scorecard}
    for criterion in required_criteria:
        if criterion not in found_criteria:
            _fail(f"Scorecard missing criterion: {criterion}")
        else:
            _ok(f"Scorecard includes: {criterion}")
    for c in scorecard:
        if not c.get("evidence"):
            _warn(f"Scorecard criterion '{c['criterion']}' has no evidence")
        if not c.get("target_score"):
            _warn(f"Scorecard criterion '{c['criterion']}' has no target_score")


def validate_manifest() -> None:
    path = _HERE / "submission_manifest.json"
    if not path.exists():
        _fail("submission_manifest.json not found")
        return
    data = _load_json(path)
    if data is None:
        return
    reg_path = _HERE / "registration.json"
    if reg_path.exists():
        reg_data = _load_json(reg_path)
        if reg_data:
            manifest_uuid = data.get("registration_uuid", "")
            reg_uuid = str(reg_data.get("uuid", ""))
            if manifest_uuid == reg_uuid:
                _ok("Manifest UUID matches registration UUID")
            else:
                _fail(f"Manifest UUID ({manifest_uuid}) != registration UUID ({reg_uuid})")


def validate_evaluation() -> None:
    path = OUTPUTS / "arsax_evaluation.json"
    if not path.exists():
        _warn("Evaluation output not found (run --mode evaluate to generate)")
        return
    data = _load_json(path)
    if data is None:
        return
    if "baseline" in data and "residual_policy" in data:
        _ok("Evaluation contains baseline vs residual comparison")
        baseline = data["baseline"]
        residual = data["residual_policy"]
        b_success = baseline.get("success_rate", 0)
        r_success = residual.get("success_rate", 0)
        _ok(f"Baseline success: {b_success:.0%}, Residual success: {r_success:.0%}")
    if "improvement" in data:
        imp = data["improvement"]
        for key, val in imp.items():
            if "pct" in key or "delta" in key:
                _ok(f"Improvement: {key} = {val}")
    n_rollouts = data.get("config", {}).get("n_rollouts", 0)
    if n_rollouts >= 16:
        _ok(f"{n_rollouts} stress rollouts performed")


def validate_narration() -> None:
    path = OUTPUTS / "arsax_narration.srt"
    if not path.exists():
        return
    content = path.read_text(encoding="utf-8")
    required_beats = [
        "Stabilizing tissue",
        "Grasping",
        "Orienting",
        "Driving needle",
        "Pulling suture",
        "Tying surgical knot",
    ]
    for beat in required_beats:
        if beat.lower() in content.lower():
            _ok(f"Narration contains: {beat}")
        else:
            _warn(f"Narration missing beat: {beat}")


def validate_contact_timeline() -> None:
    path = OUTPUTS / "arsax_contact_timeline.json"
    if not path.exists():
        return
    data = _load_json(path)
    if data is None:
        return
    timeline = data.get("timeline", [])
    if len(timeline) >= 10:
        _ok(f"Contact timeline: {len(timeline)} samples")
    else:
        _warn(f"Contact timeline has only {len(timeline)} samples")
    if "summary" in data:
        summary = data["summary"]
        for key in ["max_active_fingers", "peak_grip_strength"]:
            if key in summary:
                _ok(f"Contact timeline contains {key}: {summary[key]}")


def validate_latency_comparison() -> None:
    path = OUTPUTS / "arsax_latency_comparison.json"
    if not path.exists():
        return
    data = _load_json(path)
    if data is None:
        return
    if "baseline" in data and "with_latency" in data:
        _ok("Latency comparison: baseline vs with_latency present")
    if "verdict" in data:
        _ok(f"Latency verdict: {data['verdict']}")


# --- Main ---


def main() -> int:
    print(f"\n{'=' * 60}")
    print(f"  ARSA-X — Submission Validation")
    print(f"  Path: {_HERE}")
    print(f"{'=' * 60}\n")

    uuid = validate_registration()
    validate_required_files()
    validate_outputs()
    validate_report()
    validate_trajectory()
    validate_policy_card()
    validate_evaluation_guide()
    validate_rubric_scorecard()
    validate_manifest()
    validate_evaluation()
    validate_narration()
    validate_contact_timeline()
    validate_latency_comparison()

    print(f"\n{'=' * 60}")
    if ERRORS:
        print(f"  FAILED: {len(ERRORS)} error(s), {len(WARNINGS)} warning(s)")
        for e in ERRORS:
            print(f"    - {e}")
        print(f"\n  Fix errors before submitting.")
        print(f"{'=' * 60}")
        return 1
    elif WARNINGS:
        print(f"  PASSED with {len(WARNINGS)} warning(s)")
        for w in WARNINGS:
            print(f"    - {w}")
        print(f"\n  Submission is complete. Generate missing artifacts for full validation.")
        print(f"{'=' * 60}")
        return 1
    else:
        print(f"  ALL CHECKS PASSED")
        print(f"{'=' * 60}")
        return 0


if __name__ == "__main__":
    sys.exit(main())
