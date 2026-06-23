"""ARSA-X — Agentic Robotic Surgery Assistant eXtended.

A closed-loop residual-controlled autonomous surgical suturing system
combining a Franka Panda arm (7-DOF) with an Allegro Hand (16-DOF)
in the MuJoCo physics engine.

Subpackages:
  arsax.scene       — Model composition, scene construction, tissue, sensors
  arsax.skills      — 9 atomic surgical skills with shared base class
  arsax.control     — IK, teleoperation, autonomous, latency, residual control
  arsax.planning    — Task planner, skill executor, failure monitor
  arsax.evaluation  — Stress testing, policy card generation
"""

__version__ = "2.0.0"
__author__ = "ARSA-X Team"

from . import scene
from . import skills
from . import control
from . import planning
from . import evaluation

# --- Scene ---
from .scene import (
    SurgicalScene, DeformableTissue, ARSALRobot, SensorSuite,
    build_scene_model, build_scene_model_from_xml, build_combined_model,
    PANDA_JOINTS, ALLEGRO_JOINTS,
    ALLEGRO_OPEN, ALLEGRO_CLOSE, ALLEGRO_PINCH,
    HAND_PREFIX, HAND_KP, HAND_KV,
    MANAGERIE_MISSING, PANDA_DIR, ALLEGRO_DIR,
    NEEDLE_WELD_NAME,
    activate_needle_weld, release_needle_weld,
)

# --- Skills ---
from .skills import (
    SkillBase, SkillResult, SkillStatus,
    GraspNeedle, OrientNeedle, InsertNeedle,
    PullSuture, TieKnot, RegraspNeedle,
    StabilizeTissue, ReleaseObject, FingerGait,
    SKILL_REGISTRY, get_skill,
)

# --- Control ---
from .control import (
    ArmIK, TeleopController, AutonomousController,
    LatencySimulator, ResidualSurgicalController, ResidualState,
)

# --- Planning ---
from .planning import (
    SurgicalPlanner, SkillExecutor, FailureMonitor,
    SUTURE_WORKFLOW, DOUBLE_SUTURE_WORKFLOW,
    MATTRESS_SUTURE_WORKFLOW, FIGURE_EIGHT_WORKFLOW,
    RUNNING_SUTURE_WORKFLOW, SKILL_MAP,
)

# --- Evaluation ---
from .evaluation import (
    SurgicalStressEvaluator, generate_policy_card,
)

__all__ = [
    # Scene
    "SurgicalScene", "DeformableTissue", "ARSALRobot", "SensorSuite",
    "build_scene_model", "build_scene_model_from_xml", "build_combined_model",
    "PANDA_JOINTS", "ALLEGRO_JOINTS",
    "ALLEGRO_OPEN", "ALLEGRO_CLOSE", "ALLEGRO_PINCH",
    "HAND_PREFIX", "HAND_KP", "HAND_KV",
    "MANAGERIE_MISSING", "PANDA_DIR", "ALLEGRO_DIR",
    "NEEDLE_WELD_NAME",
    "activate_needle_weld", "release_needle_weld",
    # Skills
    "SkillBase", "SkillResult", "SkillStatus",
    "GraspNeedle", "OrientNeedle", "InsertNeedle",
    "PullSuture", "TieKnot", "RegraspNeedle",
    "StabilizeTissue", "ReleaseObject", "FingerGait",
    "SKILL_REGISTRY", "get_skill",
    # Control
    "ArmIK", "TeleopController", "AutonomousController",
    "LatencySimulator", "ResidualSurgicalController", "ResidualState",
    # Planning
    "SurgicalPlanner", "SkillExecutor", "FailureMonitor",
    "SUTURE_WORKFLOW", "DOUBLE_SUTURE_WORKFLOW",
    "MATTRESS_SUTURE_WORKFLOW", "FIGURE_EIGHT_WORKFLOW",
    "RUNNING_SUTURE_WORKFLOW", "SKILL_MAP",
    # Evaluation
    "SurgicalStressEvaluator", "generate_policy_card",
]
