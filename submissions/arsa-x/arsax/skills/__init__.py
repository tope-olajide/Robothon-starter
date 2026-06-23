"""Surgical skills library — 9 atomic manipulation skills for ARSA-X.

Each skill is a deterministic, parameterised, reusable joint-space control
policy operating on the combined Panda + Allegro Hand (23 DOF).
"""

from .base import SkillBase, SkillResult, SkillStatus
from .grasp import GraspNeedle
from .orient import OrientNeedle
from .insert import InsertNeedle
from .pull import PullSuture
from .tie import TieKnot
from .regrasp import RegraspNeedle
from .stabilize import StabilizeTissue
from .release import ReleaseObject
from .gait import FingerGait
from .bimanual import BimanualStabilizeTissue

SKILL_REGISTRY = {
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


def get_skill(name: str) -> type[SkillBase]:
    """Look up a skill class by registry name."""
    if name not in SKILL_REGISTRY:
        raise KeyError(f"Unknown skill: {name!r}. Available: {list(SKILL_REGISTRY)}")
    return SKILL_REGISTRY[name]


__all__ = [
    "SkillBase", "SkillResult", "SkillStatus",
    "GraspNeedle", "OrientNeedle", "InsertNeedle",
    "PullSuture", "TieKnot", "RegraspNeedle",
    "StabilizeTissue", "BimanualStabilizeTissue",
    "ReleaseObject", "FingerGait",
    "SKILL_REGISTRY", "get_skill",
]
